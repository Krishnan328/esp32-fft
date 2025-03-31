mod constants;
mod web_server;

use anyhow::Result;
use constants::*;
use esp_idf_hal::{
	gpio::*,
	i2s::{
		config::{
			Config, DataBitWidth, SlotMode, StdClkConfig, StdConfig, StdGpioConfig, StdSlotConfig,
		},
		I2sDriver,
	},
	peripherals::Peripherals,
};
use esp_idf_svc::log::EspLogger;
use log::{error, info};
use rustfft::{num_complex::Complex, FftPlanner};
use std::sync::{Arc, RwLock};
use web_server::*;

// Shared state between FFT processing and web server

struct FFTData {
	magnitudes: [f32; FREQUENCY_MAGNITUDE_LENGHT],
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

	// Create shared state for FFT data with RwLock
	// let system_state: Arc<RwLock<SystemState>> = Arc::new(RwLock::new(SystemState {
	// 	magnitudes: vec![0.0; FREQUENCY_MAGNITUDE_LENGHT],
	// 	dominant_frequency: 0.0,
	// }));

	let system_state: Arc<RwLock<Arc<FFTData>>> = Arc::new(RwLock::new(Arc::new(FFTData {
		magnitudes: [0.0; FREQUENCY_MAGNITUDE_LENGHT],
		dominant_frequency: 0.0,
	})));

	// Clone state for the Wi-Fi/server thread
	let server_state = system_state.clone();

	// Configure and initialize the I2S driver
	let clock_config = StdClkConfig::from_sample_rate_hz(SAMPLING_RATE);
	let slot_config = StdSlotConfig::philips_slot_default(DataBitWidth::Bits32, SlotMode::Mono);
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
		.map(|i| 0.5 * (1.0 - (2.0 * std::f32::consts::PI * i as f32 / HANN_WINDOW_LENGHT).cos()))
		.collect();
	info!("hann_window initialized with length: {}", hann_window.len());

	spawn_wifi_thread(modem, server_state).expect("Failed to spawn WiFi/server thread!");

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
		}

		if acc_index >= FFT_LENGTH_BYTES {
			// Process accumulated buffer for FFT
			let mut samples: Vec<Complex<f32>> = vec![Complex::<f32>::new(0.0, 0.0); FFT_LENGTH];

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
			let mut magnitudes: [f32; FREQUENCY_MAGNITUDE_LENGHT] =
				[0.0; FREQUENCY_MAGNITUDE_LENGHT];
			let mut max_mag: f32 = 0.0;
			let mut max_index = 0;

			for i in 0..FREQUENCY_MAGNITUDE_LENGHT {
				magnitudes[i] = samples[i].norm() * scale_factor; // Magnitude from FFT
				let frequency = i as f32 * FREQ_BIN_WIDTH;
				let threshold = if frequency < AMPLITUDE_THRESHOLD.frequency_cutoff {
					AMPLITUDE_THRESHOLD.low_freq_threshold
				} else {
					AMPLITUDE_THRESHOLD.high_freq_threshold
				};
				if magnitudes[i] > threshold && magnitudes[i] > max_mag {
					max_mag = magnitudes[i];
					max_index = i;
				}
			}

			// Quadratic Interpolation to find dominant frequency
			let frequency: f32 = if max_mag > 0.0 {
				if max_index > 0 && max_index < FREQUENCY_MAGNITUDE_LENGHT - 1 {
					let y_km1 = magnitudes[max_index - 1];
					let y_k = magnitudes[max_index];
					let y_kp1 = magnitudes[max_index + 1];
					let denom = y_km1 - 2.0 * y_k + y_kp1;
					if denom != 0.0 {
						let p = 0.5 * (y_km1 - y_kp1) / denom;
						(max_index as f32 + p) * FREQ_BIN_WIDTH
					} else {
						max_index as f32 * FREQ_BIN_WIDTH
					}
				} else {
					max_index as f32 * FREQ_BIN_WIDTH
				}
			} else {
				0.0
			};

			let updated_fft_data: Arc<FFTData> = Arc::new(FFTData {
				magnitudes,
				dominant_frequency: frequency,
			});

			// info!("Updated FFT data, dominant frequency: {:.2} Hz", frequency); // Uncomment to print dominant freq. to console
			// info!("Updated FFT data, magnitudes: {:?}", &state.magnitudes); // Uncomment to print all magnitudes

			// Update shared state with new FFT data
			*system_state.write().unwrap() = updated_fft_data;

			acc_index = 0;
		}
	}
}
