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
	is_recording: bool,
	latest_impulse: Option<ImpulseData>,
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

	// Configure button pin as input with pull-down
	let mut button = PinDriver::input(pins.gpio0.downgrade())?;
	button.set_pull(Pull::Down)?; // Configure with pull-down resistor

	// Create shared state for FFT data with RwLock
	// let system_state: Arc<RwLock<SystemState>> = Arc::new(RwLock::new(SystemState {
	// 	magnitudes: vec![0.0; FREQUENCY_MAGNITUDE_LENGHT],
	// 	dominant_frequency: 0.0,
	// }));

	let system_state: Arc<RwLock<Arc<FFTData>>> = Arc::new(RwLock::new(Arc::new(FFTData {
		magnitudes: [0.0; FREQUENCY_MAGNITUDE_LENGHT],
		dominant_frequency: 0.0,
		is_recording: false,
		latest_impulse: None,
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
		pins.gpio4,  // sck
		pins.gpio22, // sd
		None::<AnyIOPin>,
		pins.gpio21, // ws
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
	let timeout = 16;

	// Main FFT loop
	loop {
		let is_button_pressed = button.get_level() == Level::High;
		// let is_button_pressed = true;

		// Update recording state in shared data
		let current_state = {
			let read_guard = system_state.read().unwrap();
			let current_data = read_guard.clone();

			// If state changed, log it
			if current_data.is_recording != is_button_pressed {
				if is_button_pressed {
					info!("Button pressed: Starting FFT recording");
				} else {
					info!("Button released: Pausing FFT recording");
					// Reset buffer when stopping recording
					acc_index = 0;
				}
			}

			current_data
		};

		// If recording state changed, update shared state
		if current_state.is_recording != is_button_pressed {
			*system_state.write().unwrap() = Arc::new(FFTData {
				magnitudes: current_state.magnitudes,
				dominant_frequency: current_state.dominant_frequency,
				is_recording: is_button_pressed,
				latest_impulse: None,
			});
		}

		if is_button_pressed {
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

				// Impulse detection logic
				static mut PREV_MAGNITUDE_SUM: f32 = 0.0;
				static mut LAST_IMPULSE_TIME: u64 = 0;

				let magnitude_sum: f32 = magnitudes.iter().sum();
				let now: u64 = unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 / 1000 }; // Convert to ms
				let delta = (magnitude_sum - unsafe { PREV_MAGNITUDE_SUM }).abs();

				unsafe {
					PREV_MAGNITUDE_SUM = magnitude_sum;
				}

				let mut detected_impulse = None;

				// Check if this is an impulse (sudden change in magnitude)
				if delta > IMPULSE_THRESHOLD
					&& now - unsafe { LAST_IMPULSE_TIME } > IMPULSE_TIME_THRESHOLD
					&& frequency > 100.0
				// No impulse below 100hz is required as the coconut is giving impulse at and above 100Hz.
				{
					unsafe {
						LAST_IMPULSE_TIME = now;
					}

					// Find additional peaks around dominant frequency
					let peak_indices = find_peaks(&magnitudes, max_index);

					// Create peak data
					let peaks = peak_indices
						.iter()
						.map(|&idx| PeakData {
							index: idx,
							frequency: idx as f32 * FREQ_BIN_WIDTH,
							magnitude: magnitudes[idx],
						})
						.collect();

					// Create impulse data
					detected_impulse = Some(ImpulseData {
						timestamp: now,
						dominant_frequency: frequency,
						peaks,
					});

					info!(
						"Impulse detected at {} ms, dominant frequency: {:.2} Hz",
						now, frequency
					);
				}

				let updated_fft_data: Arc<FFTData> = Arc::new(FFTData {
					magnitudes,
					dominant_frequency: frequency,
					is_recording: true,
					latest_impulse: detected_impulse,
				});

				// info!("Updated FFT data, dominant frequency: {:.2} Hz", frequency); // Uncomment to print dominant freq. to console
				// info!("Updated FFT data, magnitudes: {:?}", &state.magnitudes); // Uncomment to print all magnitudes

				// Update shared state with new FFT data
				*system_state.write().unwrap() = updated_fft_data;

				acc_index = 0;
			}
		} else {
			// Small delay to avoid busy-waiting when not recording
			std::thread::sleep(std::time::Duration::from_millis(16));
		}
	}
}

fn find_peaks(magnitudes: &[f32; FREQUENCY_MAGNITUDE_LENGHT], dominant_index: usize) -> Vec<usize> {
	let mut result = vec![dominant_index];
	let range = 5; // 5 peaks before and after

	// Find local maxima before dominant peak
	let start_index = (dominant_index as i32 - 25).max(0) as usize;
	for _ in 0..range {
		let mut max_idx = start_index;
		let mut max_val = f32::NEG_INFINITY;

		(start_index..dominant_index).for_each(|j| {
			if magnitudes[j] > max_val && !result.contains(&j) {
				max_val = magnitudes[j];
				max_idx = j;
			}
		});

		if max_val > 0.0 {
			result.push(max_idx);
		}
	}

	// Find local maxima after dominant peak
	let end_index = (dominant_index + 25).min(FREQUENCY_MAGNITUDE_LENGHT - 1);
	let mut current_idx = dominant_index;
	for _ in 0..range {
		let mut max_idx = current_idx;
		let mut max_val = f32::NEG_INFINITY;

		(current_idx + 1..=end_index).for_each(|j| {
			if magnitudes[j] > max_val && !result.contains(&j) {
				max_val = magnitudes[j];
				max_idx = j;
			}
		});

		if max_idx > current_idx && max_val > 0.0 {
			result.push(max_idx);
			current_idx = max_idx;
		}
	}

	// Sort by frequency (index)
	result.sort();
	result
}
