use std::string::String;
use anyhow::Result;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::*;
use esp_idf_hal::i2s::{config::{Config, DataBitWidth, SlotMode, StdClkConfig, StdConfig, StdGpioConfig, StdSlotConfig}, I2sDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::log::EspLogger;
use esp_idf_svc::wifi::{BlockingWifi, EspWifi};
use esp_idf_svc::wifi::AccessPointConfiguration;
use esp_idf_svc::http::server::{Configuration, EspHttpServer};
use esp_idf_svc::io::EspIOError;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use log::{info, error};
use rustfft::{num_complex::Complex, FftPlanner};
use std::sync::{Arc, Mutex};
use std::thread;

// Shared state between FFT processing and web server
struct SharedState {
	magnitudes: Vec<f32>,
	dominant_frequency: f32,
}

fn main() -> Result<()> {
	// Initialize the ESP-IDF system
	esp_idf_svc::sys::link_patches();
	EspLogger::initialize_default();

	// Initialize NVS (fix for WiFi calibration data)
	let nvs = EspDefaultNvsPartition::take()?;

	info!("FFT Spectrum Analyzer starting...");

	// Take control of the peripherals
	let peripherals = Peripherals::take()?;
	let pins = peripherals.pins;

	let sysloop = EspSystemEventLoop::take()?;

	// Configure and initialize the I2S driver
	let clock_config = StdClkConfig::from_sample_rate_hz(44100);
	let slot_config = StdSlotConfig::philips_slot_default(DataBitWidth::Bits32, SlotMode::Mono);
	let config = StdConfig::new(Config::default(), clock_config, slot_config, StdGpioConfig::default());

	let mut i2s = I2sDriver::new_std_rx(
		peripherals.i2s0,
		&config,
		pins.gpio25,
		pins.gpio26,
		None::<AnyIOPin>,
		pins.gpio27,
	)?;

	// Enable I2S receiver
	i2s.rx_enable()?;

	// Create FFT planner and Hann window
	let mut planner = FftPlanner::new();
	let fft = planner.plan_fft_forward(1024);

	// Pre-calculate Hann window for better performance
	let hann_window: [f32; 1024] = core::array::from_fn(|i| 0.5 * (1.0 - (2.0 * std::f32::consts::PI * i as f32 / 1023.0).cos()));
	info!("hann_window initialized with length: {}", hann_window.len());

	// Set up WiFi in Access Point mode
	info!("Setting up WiFi Access Point...");
	let mut wifi = BlockingWifi::wrap(
		EspWifi::new(peripherals.modem, sysloop.clone(), Some(nvs))?, // Pass NVS to WiFi
		sysloop,
	)?;

	wifi.set_configuration(&esp_idf_svc::wifi::Configuration::AccessPoint(
		AccessPointConfiguration {
			ssid: "ESP32-FFT-Analyzer".try_into().unwrap(),
			password: "spectrum123".try_into().unwrap(),
			auth_method: esp_idf_svc::wifi::AuthMethod::WPA2Personal,
			..Default::default()
		}
	))?;

	wifi.start()?;
	info!("WiFi Access Point started: SSID='ESP32-FFT-Analyzer', Password='spectrum123'");

	// Create shared state for FFT data with mutex
	let shared_state = Arc::new(Mutex::new(SharedState {
		magnitudes: vec![0.0; 512],
		dominant_frequency: 0.0,
	}));

	// Start HTTP server
	info!("Starting HTTP server...");
	let server_state = shared_state.clone();
	let mut server = EspHttpServer::new(&Configuration::default())?;

	// Serve the main HTML page
	server.fn_handler("/", esp_idf_svc::http::Method::Get, move |request| {
		let html = include_str!("../index.html");
		let mut resp = request.into_ok_response()?;
		resp.write(html.as_bytes())?;
		Ok::<(), EspIOError>(())
	})?;

	// API endpoint to get FFT data as JSON
	server.fn_handler("/api/fft", esp_idf_svc::http::Method::Get, move |request| {
		let state = match server_state.lock() {
			Ok(state) => state,
			Err(_) => {
				// Handle lock error gracefully
				let mut resp = request.into_response(500, Some("Internal Server Error"), &[])?;
				resp.write(b"Internal server error")?;
				return Ok(());
			}
		};

		// Create JSON string from the FFT data
		let mut json = String::from("{\"magnitudes\":[");
		for (i, &mag) in state.magnitudes.iter().enumerate() {
			if i > 0 {
				json.push_str(",");
			}
			json.push_str(&format!("{:.6}", mag));
		}
		json.push_str("],\"dominantFrequency\":");
		json.push_str(&format!("{:.2}", state.dominant_frequency));
		json.push_str("}");

		let mut resp = request.into_ok_response()?;
		resp.write(json.as_bytes())?;
		Ok::<(), EspIOError>(())
	})?;

	info!("HTTP server started - Connect to http://192.168.71.1");

	// Create a safe copy of I2S driver for the task
	let i2s_mutex = Arc::new(Mutex::new(i2s));
	let i2s_for_task = i2s_mutex.clone();

	// Start FFT processing task using ESP-IDF's native task system
	let fft_state = shared_state.clone();
	let fft_arc = Arc::new(fft);  // Make FFT shareable across tasks
	let hann_window_arc = Arc::new(hann_window);

	let builder = thread::Builder::new().stack_size(16384); // 16 KB
	builder.spawn(move || {
		let mut buffer = [0u8; 512];
		let mut accumulated_buffer = vec![0u8; 4096];
		let mut acc_index = 0;
		let timeout = 100;

		info!("FFT task started"); // Confirm task begins

		loop {
			let mut i2s = match i2s_for_task.lock() {
				Ok(i2s) => i2s,
				Err(e) => {
					error!("Failed to lock I2S driver: {:?}", e);
					FreeRtos::delay_ms(100);
					continue;
				}
			};
			info!("I2S driver locked, attempting read...");

			while acc_index < 4096 {
				match i2s.read(&mut buffer, timeout) {
					Ok(bytes_read) => {
						let space_left = 4096 - acc_index;
						let bytes_to_copy = bytes_read.min(space_left);
						for i in 0..bytes_to_copy {
							accumulated_buffer[acc_index + i] = buffer[i];
						}
						acc_index += bytes_to_copy;
					}
					Err(e) => {
						error!("I2S read error: {:?}", e);
						break;
					}
				}
				FreeRtos::delay_ms(1);
			}

			drop(i2s);

			if acc_index >= 4096 {
				let mut samples = vec![Complex::<f32>::new(0.0, 0.0); 1024];
				let hann_window = &hann_window_arc;

				// Add length check before accessing hann_window
				if hann_window.len() != 1024 {
					error!("hann_window length is {}, expected 1024", hann_window.len());
					acc_index = 0; // Reset and retry
					continue;
				}

				info!("Processing FFT with hann_window length: {}", hann_window.len());

				for i in 0..1024 {
					let offset = i * 4;
					let val = i32::from_le_bytes([
						accumulated_buffer[offset],
						accumulated_buffer[offset + 1],
						accumulated_buffer[offset + 2],
						accumulated_buffer[offset + 3],
					]);
					let sample_f32 = (val as f32) / 2147483648.0 * hann_window[i];
					samples[i] = Complex::new(sample_f32, 0.0);
				}

				let fft = &fft_arc;
				fft.process(&mut samples);

				let scale_factor = 1.0 / 1024.0;
				let mut magnitudes = [0.0f32; 512];
				let mut max_mag = 0.0;
				let mut max_index = 0;

				for i in 0..512 {
					magnitudes[i] = samples[i].norm() * scale_factor;
					if magnitudes[i] > max_mag {
						max_mag = magnitudes[i];
						max_index = i;
					}
				}

				let frequency = max_index as f32 * (44100.0 / 1024.0);

				match fft_state.lock() {
					Ok(mut state) => {
						for i in 0..512 {
							state.magnitudes[i] = magnitudes[i];
						}
						state.dominant_frequency = frequency;
						info!("Updated FFT data, dominant frequency: {:.2} Hz", frequency);
					}
					Err(e) => {
						error!("Failed to update FFT data: {:?}", e);
					}
				}

				acc_index = 0;
			}

			FreeRtos::delay_ms(100);
		}
	}).expect("Failed to spawn ftt thread!!!");

	// Keep the main task alive
	loop {
		FreeRtos::delay_ms(1000);
	}
}