use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::i2s::I2sDriver;
use esp_idf_hal::i2s::config::{DataBitWidth, StdConfig};
use esp_idf_hal::gpio::*;
use esp_idf_svc::log::EspLogger;
use rustfft::{FftPlanner, num_complex::Complex};
use anyhow::Result;
use log::info;

#[tokio::main]
async fn main() -> Result<()> {
	// Initialize patches and logging
	esp_idf_svc::sys::link_patches();
	EspLogger::initialize_default();

	// Take peripherals
	let peripherals = Peripherals::take()?;
	let pins = peripherals.pins;

	// Configure I2S for INMP441 (Philips I2S, 44.1 kHz, 24-bit, mono)
	let config = StdConfig::philips(44100, DataBitWidth::Bits32);

	// Initialize I2S driver in standard receive mode
	let mut i2s = I2sDriver::new_std_rx(
		peripherals.i2s0,
		&config,
		pins.gpio25,       // BCLK (bit clock)
		pins.gpio26,       // DIN (data in)
		None::<AnyIOPin>,  // MCLK with explicit type
		pins.gpio27,       // WS (word select)
	)?;

	i2s.rx_enable()?;

	// Create FFT planner once, outside the loop
	let mut planner = FftPlanner::new();
	let fft = planner.plan_fft_forward(1024);

	// Pre-compute Hann window coefficients
	let hann_window: Vec<f32> = (0..1024)
		.map(|i| 0.5 * (1.0 - (2.0 * std::f32::consts::PI * i as f32 / 1023.0).cos()))
		.collect();

	// Buffer for accumulated samples
	let mut accumulated_buffer = Vec::with_capacity(4096);
	let timeout = 100; // Shorter timeout to prevent long blocking

	// Main loop
	loop {
		// Use a smaller read buffer for more frequent yields
		let mut buffer = [0u8; 512];

		// Read data in chunks until we have enough for processing
		while accumulated_buffer.len() < 4096 {
			match i2s.read(&mut buffer, timeout) {
				Ok(bytes_read) => {
					accumulated_buffer.extend_from_slice(&buffer[..bytes_read]);
				}
				Err(e) => {
					info!("I2S read error: {:?}", e);
					break;
				}
			}

			// Yield to scheduler occasionally
			esp_idf_hal::task::yield_now().await;
		}

		// Process only if we have enough data
		if accumulated_buffer.len() >= 4096 {
			// Extract 1024 samples (4096 bytes)
			let process_buffer = accumulated_buffer.drain(..4096).collect::<Vec<u8>>();

			// Convert byte buffer to f32 samples with windowing
			let mut samples = Vec::with_capacity(1024);
			for i in 0..1024 {
				let offset = i * 4;
				let val = i32::from_le_bytes([
					process_buffer[offset],
					process_buffer[offset + 1],
					process_buffer[offset + 2],
					process_buffer[offset + 3],
				]);

				// Normalize and apply Hann window
				let sample_f32 = (val as f32) / 2147483648.0 * hann_window[i];
				samples.push(sample_f32);
			}

			// Prepare complex signal for FFT
			let mut signal: Vec<Complex<f32>> = samples.into_iter()
				.map(|s| Complex::new(s, 0.0))
				.collect();

			// Perform FFT
			fft.process(&mut signal);

			// Calculate properly scaled magnitudes (up to Nyquist frequency)
			// Scale by 1/N for proper amplitude scaling
			let scale_factor = 1.0 / 1024.0;
			let magnitudes: Vec<f32> = signal.iter()
				.take(512)
				.map(|c| c.norm() * scale_factor)
				.collect();

			// Find dominant frequency (account for DC component properly)
			let max_index = magnitudes.iter()
				.enumerate()
				.skip(1) // Skip DC component
				.max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
				.unwrap()
				.0;

			// Calculate frequency (account for the skipped DC when using max_index)
			let frequency = max_index as f32 * (44100.0 / 1024.0);
			info!("Dominant frequency: {:.2} Hz", frequency);
		}

		// Small delay to avoid starving other tasks
		esp_idf_hal::delay::FreeRtos::delay_ms(10);
	}
}