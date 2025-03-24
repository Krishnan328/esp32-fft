mod constants;

use anyhow::Result;
use constants::*;
use esp_idf_hal::{
	delay::FreeRtos,
	gpio::*,
	i2s::{
		config::{
			Config, DataBitWidth, SlotMode, StdClkConfig, StdConfig,
			StdGpioConfig, StdSlotConfig,
		},
		I2sDriver,
	},
	peripherals::Peripherals,
};
use esp_idf_svc::{
	eventloop::EspSystemEventLoop,
	http::server::{Configuration, EspHttpServer},
	io::EspIOError,
	log::EspLogger,
	nvs::EspDefaultNvsPartition,
	wifi::AccessPointConfiguration,
	wifi::{BlockingWifi, EspWifi},
};
use log::{error, info};
use rustfft::{num_complex::Complex, FftPlanner};
use std::{
	string::String,
	sync::{Arc, RwLock},
	thread,
};

// Shared state between FFT processing and web server
struct SharedState {
	magnitudes: Vec<f32>,
	dominant_frequency: f32,
}

fn main() -> Result<()> {
	// Initialize the ESP-IDF system
	esp_idf_svc::sys::link_patches();
	EspLogger::initialize_default();

	info!("FFT Spectrum Analyzer starting...");

	// Take control of the peripherals
	let peripherals = Peripherals::take()?;
	let pins = peripherals.pins;
	let modem = peripherals.modem;

	// Initialize NVS (fix for WiFi calibration data)
	let nvs = EspDefaultNvsPartition::take()?;

	// Get system event loop
	let sysloop = EspSystemEventLoop::take()?;

	// Configure and initialize the I2S driver
	let clock_config = StdClkConfig::from_sample_rate_hz(SAMPLING_RATE);
	let slot_config = StdSlotConfig::philips_slot_default(
		DataBitWidth::Bits32,
		SlotMode::Mono,
	);
	let config = StdConfig::new(
		Config::default(),
		clock_config,
		slot_config,
		StdGpioConfig::default(),
	);

	let mut i2s = I2sDriver::new_std_rx(
		peripherals.i2s0,
		&config,
		pins.gpio25, // sck
		pins.gpio26, // sd
		None::<AnyIOPin>,
		pins.gpio27, // ws
	)?;

	// Enable I2S receiver
	i2s.rx_enable()?;

	// Create FFT planner and Hann window
	let mut planner = FftPlanner::new();
	let fft = planner.plan_fft_forward(FFT_LENGTH);

	// Pre-calculate Hann window for better performance
	let hann_window: Vec<f32> = (0..FFT_LENGTH)
		.map(|i| {
			0.5 * (1.0
				- (2.0 * std::f32::consts::PI * i as f32 / HANN_WINDOW_LENGHT)
					.cos())
		})
		.collect();
	info!("hann_window initialized with length: {}", hann_window.len());

	// Create shared state for FFT data with RwLock
	let shared_state: Arc<RwLock<SharedState>> =
		Arc::new(RwLock::new(SharedState {
			magnitudes: vec![0.0; FREQUENCY_MAGNITUDE_LENGHT],
			dominant_frequency: 0.0,
		}));

	// Clone state for the Wi-Fi/server thread
	let server_state = shared_state.clone();

	let wifi_builder = thread::Builder::new()
		.stack_size(8192)
		.name("wifi_server".into());

	// Start Wi-Fi and HTTP server on a separate thread
	wifi_builder
		.spawn(move || {
			info!("Setting up WiFi Access Point...");
			let mut wifi = BlockingWifi::wrap(
				EspWifi::new(modem, sysloop.clone(), Some(nvs)).unwrap(),
				sysloop,
			)
			.expect("Failed to initialize WiFi");

			wifi.set_configuration(&esp_idf_svc::wifi::Configuration::AccessPoint(
				AccessPointConfiguration {
					ssid: "ESP32-FFT-Analyzer".try_into().unwrap(),
					password: "spectrum123".try_into().unwrap(),
					auth_method: esp_idf_svc::wifi::AuthMethod::WPA2Personal,
					..Default::default()
				},
			))
			.expect("Failed to set WiFi configuration");

			wifi.start().expect("Failed to start WiFi");
			info!("WiFi Access Point started: SSID='ESP32-FFT-Analyzer', Password='spectrum123'");

			// Start HTTP server
			info!("Starting HTTP server...");
			let mut server = EspHttpServer::new(&Configuration::default())
				.expect("Failed to create HTTP server");

			// Serve the main HTML page
			server
				.fn_handler("/", esp_idf_svc::http::Method::Get, move |request| {
					let html = include_str!("../index.html");
					let mut resp = request.into_ok_response()?;
					resp.write(html.as_bytes())?;
					Ok::<(), EspIOError>(())
				})
				.expect("Failed to register index handler");

			// API endpoint to get FFT data as JSON
			let api_state = server_state.clone();
			server
				.fn_handler("/api/fft", esp_idf_svc::http::Method::Get, move |request| {
					let state = match api_state.read() {
						Ok(state) => state,
						Err(_) => {
							// Handle lock error gracefully
							let mut resp =
								request.into_response(500, Some("Internal Server Error"), &[])?;
							resp.write(b"Internal server error")?;
							return Ok(());
						}
					};

					// Create JSON string from the FFT data
					let mut json = String::from("{\"magnitudes\":[");
					for (i, &mag) in state.magnitudes.iter().enumerate() {
						if i > 0 {
							json.push(',');
						}
						json.push_str(&format!("{:.6}", mag));
					}
					json.push_str("],\"dominantFrequency\":");
					json.push_str(&format!("{:.2}", state.dominant_frequency));
					json.push('}');

					let mut resp = request.into_ok_response()?;
					resp.write(json.as_bytes())?;
					Ok::<(), EspIOError>(())
				})
				.expect("Failed to register API handler");

			info!("HTTP server started - Connect to http://192.168.71.1");

			// Keep this thread alive
			loop {
				FreeRtos::delay_ms(1000);
			}
		})
		.expect("Failed to spawn WiFi/server thread!");

	let mut buffer: Vec<u8> = vec![0; FREQUENCY_MAGNITUDE_LENGHT];
	let mut accumulated_buffer: Vec<u8> = vec![0; FFT_LENGTH_BYTES];
	let mut acc_index = 0;
	let timeout = 100;

	// Main FFT loop
	loop {
		while acc_index < FFT_LENGTH_BYTES {
			match i2s.read(&mut buffer, timeout) {
				Ok(bytes_read) => {
					let space_left = FFT_LENGTH_BYTES - acc_index;
					let bytes_to_copy = bytes_read.min(space_left);
					accumulated_buffer[acc_index..acc_index + bytes_to_copy]
						.copy_from_slice(&buffer[..bytes_to_copy]);
					acc_index += bytes_to_copy;
				}
				Err(e) => {
					error!("I2S read error: {:?}", e);
					break;
				}
			}
			FreeRtos::delay_ms(1);
		}

		if acc_index >= FFT_LENGTH_BYTES {
			// Process accumulated buffer for FFT
			let mut samples: Vec<Complex<f32>> =
				vec![Complex::<f32>::new(0.0, 0.0); FFT_LENGTH];

			// Apply Hann window and prepare samples
			for i in 0..FFT_LENGTH {
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

			// Perform FFT
			fft.process(&mut samples);

			// Calculate magnitudes and find dominant frequency
			let scale_factor: f32 = 1.0 / FFT_LENGTH as f32;
			let mut magnitudes: Vec<f32> =
				vec![0.0; FREQUENCY_MAGNITUDE_LENGHT];
			let mut max_mag: f32 = 0.0;
			let mut max_index = 0;

			for i in 0..FREQUENCY_MAGNITUDE_LENGHT {
				magnitudes[i] = samples[i].norm() * scale_factor;
				if magnitudes[i] > max_mag {
					max_mag = magnitudes[i];
					max_index = i;
				}
			}

			let frequency: f32 =
				max_index as f32 * (SAMPLING_RATE as f32 / FFT_LENGTH as f32);

			// Update shared state with new FFT data
			match shared_state.write() {
				Ok(mut state) => {
					state.magnitudes.copy_from_slice(
						&magnitudes[..FREQUENCY_MAGNITUDE_LENGHT],
					);
					state.dominant_frequency = frequency;
					info!(
						"Updated FFT data, dominant frequency: {:.2} Hz",
						frequency
					);
				}
				Err(e) => {
					error!("Failed to update FFT data: {:?}", e);
				}
			}

			acc_index = 0;
		}
	}
}
