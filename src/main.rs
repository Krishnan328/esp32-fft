use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::i2s::I2sDriver;
use esp_idf_hal::i2s::config::{DataBitWidth, StdConfig};
use esp_idf_hal::gpio::*;  // Add this import for GPIO pin types
use esp_idf_svc::log::EspLogger;
use rustfft::{FftPlanner, num_complex::Complex};
use anyhow::Result;
use log::info;

fn main() -> Result<()> {
	// Initialize patches and logging
	esp_idf_svc::sys::link_patches();
	EspLogger::initialize_default();

	// Take peripherals
	let peripherals = Peripherals::take()?;
	let pins = peripherals.pins;

	// Configure I2S for INMP441 (Philips I2S, 44.1 kHz, 24-bit, mono)
	let config = StdConfig::philips(44100, DataBitWidth::Bits32);

	// Initialize I2S driver in standard receive mode
	// For pins used with the I2S driver, we just pass them directly
	// They will be configured by the driver
	let mut i2s = I2sDriver::new_std_rx(
		peripherals.i2s0,
		&config,
		pins.gpio25,       // BCLK (bit clock)
		pins.gpio26,       // DIN (data in)
		None::<AnyIOPin>,  // MCLK with explicit type
		pins.gpio27,       // WS (word select)
	)?;

	i2s.rx_enable()?;

	// Main loop
	loop {
		// Buffer for 1024 samples, each 32-bit (4 bytes)
		let mut buffer = [0u8; 1024 * 4];
		let timeout = 1000; // 1-second timeout
		let bytes_read = i2s.read(&mut buffer, timeout)?;
		if bytes_read != 4096 {
			info!("Partial read: {} bytes", bytes_read);
			continue;
		}

		// Convert byte buffer to f32 samples
		let mut samples = Vec::with_capacity(1024);
		for i in 0..1024 {
			let offset = i * 4;
			let val = i32::from_le_bytes([
				buffer[offset],
				buffer[offset + 1],
				buffer[offset + 2],
				buffer[offset + 3],
			]);
			let sample_f32 = (val as f32) / 2147483648.0; // Normalize by 2^31
			samples.push(sample_f32);
		}

		// Perform FFT
		let mut planner = FftPlanner::new();
		let fft = planner.plan_fft_forward(1024);
		let mut signal: Vec<Complex<f32>> = samples.into_iter()
			.map(|s| Complex::new(s, 0.0))
			.collect();
		fft.process(&mut signal);

		// Calculate magnitudes (up to Nyquist frequency)
		let magnitudes: Vec<f32> = signal.iter()
			.take(512)
			.map(|c| c.norm())
			.collect();

		// Find dominant frequency
		let max_index = magnitudes.iter()
			.enumerate()
			.skip(1) // Skip DC component
			.max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
			.unwrap()
			.0;
		let frequency = max_index as f32 * (44100.0 / 1024.0);
		info!("Dominant frequency: {} Hz", frequency);

		// Delay before next iteration
		esp_idf_hal::delay::FreeRtos::delay_ms(1000);
	}
}