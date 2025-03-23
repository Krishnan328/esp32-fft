//! # Microphone Frequency Analyzer Module
//!
//! This module implements real-time frequency analysis using an INMP441 MEMS microphone 
//! connected via I2S to an ESP32 microcontroller. It performs continuous sampling and 
//! FFT-based frequency domain analysis to identify dominant frequencies in the audio input.
//!
//! ## Hardware Requirements
//! - ESP32 microcontroller
//! - INMP441 MEMS microphone (I2S interface)
//! - Connections:
//!   - GPIO25: BCLK (Bit Clock)
//!   - GPIO26: DIN (Data In)
//!   - GPIO27: WS (Word Select/LRCLK)
//!
//! ## Features
//! - Continuous audio sampling at 44.1kHz
//! - 1024-point FFT with Hann window for spectral leakage reduction
//! - Real-time dominant frequency detection
//! - Asynchronous operation with Tokio
//!
//! ## Flow of Operation
//! ```
//! 1. INITIALIZATION:
//!    - Initialize ESP-IDF system
//!    - Configure logging
//!    - Take control of peripherals
//!    - Configure I2S interface for INMP441 (Philips I2S, 44.1kHz, 32-bit)
//!    - Initialize I2S driver in receive mode
//!    - Create FFT planner
//!    - Precompute Hann window coefficients
//!    - Initialize sample buffer
//!
//! 2. MAIN LOOP:
//!    REPEAT FOREVER:
//!        - Read audio data chunks from I2S microphone
//!        - Accumulate data until buffer has 4096 bytes (1024 samples)
//!        - Periodically yield to scheduler
//!        
//!        IF accumulated_buffer ≥ 4096 bytes:
//!            - Extract 1024 samples (4096 bytes)
//!            - Convert bytes to normalized float samples
//!            - Apply Hann window to reduce spectral leakage
//!            - Prepare complex signal for FFT
//!            - Perform 1024-point FFT
//!            - Calculate magnitude spectrum (up to Nyquist frequency)
//!            - Find index of peak magnitude (skip DC component)
//!            - Calculate actual frequency from index
//!            - Log the dominant frequency
//!        
//!        - Small delay to allow other tasks to run
//! ```
//!
//! ## Limitations
//! - Fixed FFT size of 1024 points
//! - Single-channel audio processing
//! - No advanced filtering or noise reduction

use anyhow::Result;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::*;
use esp_idf_hal::i2s::config::{Config, DataBitWidth, SlotMode, StdClkConfig, StdConfig, StdGpioConfig, StdSlotConfig};
use esp_idf_hal::i2s::I2sDriver;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::task;
use esp_idf_svc::log::EspLogger;
use log::info;
use rustfft::{num_complex::Complex, FftPlanner};

/// Main entry point for the frequency analyzer application.
///
/// This function initializes the system, configures the I2S interface for 
/// the INMP441 microphone, and enters the main processing loop to continuously
/// analyze microphone input for dominant frequencies.
///
/// # Returns
///
/// * `Result<()>` - Returns Ok(()) if execution completes successfully, or an error if something fails
#[tokio::main]
async fn main() -> Result<()> {
	// Initialize the ESP-IDF system with necessary patches
	esp_idf_svc::sys::link_patches();

	// Initialize logging system to see debug output
	EspLogger::initialize_default();
	info!("Frequency analyzer starting...");

	// Take exclusive control of the ESP32 peripherals
	let peripherals = Peripherals::take()?;
	let pins = peripherals.pins;

	// Configure I2S for INMP441 MEMS microphone
	// - Standard I2S format (standard I2S protocol)
	// - 44.1 kHz sample rate (standard audio rate)
	// - 32-bit data width in Mono slot_mode (INMP441 uses 24-bit output, but 32-bit alignment is common)
	let clock_config = StdClkConfig::from_sample_rate_hz(44100);
	let slot_config = StdSlotConfig::philips_slot_default(DataBitWidth::Bits32, SlotMode::Mono);
	let config = StdConfig::new(Config::default(), clock_config, slot_config, StdGpioConfig::default());
	info!("I2S configured for 44.1 kHz, 32-bit depth");

	// Initialize I2S driver in standard receive mode with appropriate pin connections
	// BCLK: Bit clock - provides timing for individual bits
	// DIN: Data input - receives serial audio data from microphone
	// MCLK: Master clock - not used with INMP441, so set to None
	// WS: Word select - indicates left/right channel (frame sync)
	let mut i2s = I2sDriver::new_std_rx(
		peripherals.i2s0,        // Using I2S peripheral 0
		&config,                 // Configuration from above
		pins.gpio25,             // BCLK (bit clock)
		pins.gpio26,             // DIN (data in from mic)
		None::<AnyIOPin>,        // MCLK not needed, with explicit type annotation
		pins.gpio27,             // WS (word select/LRCLK)
	)?;

	// Enable receive mode on the I2S interface
	i2s.rx_enable()?;
	info!("I2S driver initialized and enabled for receiving");

	// Create FFT planner for frequency analysis
	// This will be reused for all FFT operations
	let mut planner = FftPlanner::new();

	// Plan a 1024-point forward FFT (time domain to frequency domain)
	// 1024 points provides good frequency resolution (~43Hz per bin at 44.1kHz)
	let fft = planner.plan_fft_forward(1024);
	info!("FFT planner initialized for 1024-point FFT");

	// Pre-compute Hann window coefficients to reduce spectral leakage
	// The Hann window tapers samples toward zero at the edges of the window
	// Formula: w(n) = 0.5 * (1 - cos(2π*n/(N-1)))
	let hann_window: Vec<f32> = (0..1024)
		.map(|i| 0.5 * (1.0 - (2.0 * std::f32::consts::PI * i as f32 / 1023.0).cos()))
		.collect();
	info!("Hann window coefficients pre-computed");

	// Buffer for accumulated audio samples
	// We accumulate samples until we have enough for FFT processing
	let mut accumulated_buffer = Vec::with_capacity(4096);

	// Timeout for I2S read operations (in milliseconds)
	// Shorter timeout prevents blocking too long on I2S reads
	let timeout = 100;
	info!("Starting main processing loop");

	// Main processing loop
	loop {
		// Temporary buffer for I2S reads
		// Smaller chunks allow yielding to the scheduler more frequently
		let mut buffer = [0u8; 512];

		// Accumulate data until we have enough for FFT processing
		// 4096 bytes = 1024 samples (4 bytes per sample)
		while accumulated_buffer.len() < 4096 {
			// Read audio data from I2S with timeout
			match i2s.read(&mut buffer, timeout) {
				Ok(bytes_read) => {
					// Add read bytes to our accumulated buffer
					accumulated_buffer.extend_from_slice(&buffer[..bytes_read]);

					// Log progress occasionally
					if accumulated_buffer.len() % 1024 == 0 {
						info!("Accumulated {} bytes of audio data", accumulated_buffer.len());
					}
				}
				Err(e) => {
					// Log error but continue operation
					info!("I2S read error: {:?}", e);
					break;
				}
			}

			// Cooperatively yield to scheduler to allow other tasks to run
			// This is important in async environments to avoid starving other tasks
			task::yield_now().await;
		}

		// Process data if we have accumulated enough
		if accumulated_buffer.len() >= 4096 {
			info!("Processing audio buffer of 1024 samples");

			// Extract 1024 samples (4096 bytes) for processing
			// We use drain to remove processed data from the buffer
			let process_buffer = accumulated_buffer.drain(..4096).collect::<Vec<u8>>();

			// Convert byte buffer to f32 samples with windowing
			// Each sample is 4 bytes (32-bit), so we need to reconstruct the original values
			let mut samples = Vec::with_capacity(1024);
			for i in 0..1024 {
				// Calculate offset in the byte buffer for this sample
				let offset = i * 4;

				// Reconstruct 32-bit integer from 4 bytes (little-endian)
				let val = i32::from_le_bytes([
					process_buffer[offset],
					process_buffer[offset + 1],
					process_buffer[offset + 2],
					process_buffer[offset + 3],
				]);

				// Normalize sample to range [-1.0, 1.0] and apply Hann window
				// i32::MAX is 2^31-1, but we use 2^31 for symmetrical range
				let sample_f32 = (val as f32) / 2147483648.0 * hann_window[i];
				samples.push(sample_f32);
			}

			// Prepare complex signal for FFT
			// FFT requires complex numbers, so we set real part to sample value and imaginary part to 0
			let mut signal: Vec<Complex<f32>> = samples.into_iter()
				.map(|s| Complex::new(s, 0.0))
				.collect();

			// Perform FFT (Fast Fourier Transform)
			// This converts time-domain signal to frequency domain
			fft.process(&mut signal);

			// Calculate properly scaled magnitudes (up to Nyquist frequency)
			// We only need half the FFT result due to Nyquist theorem
			// Scale by 1/N for proper amplitude scaling
			let scale_factor = 1.0 / 1024.0;
			let magnitudes: Vec<f32> = signal.iter()
				.take(512)  // Only need first half of FFT result (up to Nyquist)
				.map(|c| c.norm() * scale_factor)  // Calculate magnitude and scale
				.collect();

			// Find dominant frequency (account for DC component properly)
			// We skip the first bin (index 0) which represents the DC component
			let max_index = magnitudes.iter()
				.enumerate()
				.skip(1)  // Skip DC component at index 0
				.max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
				.unwrap()
				.0;

			// Calculate actual frequency from FFT bin index
			// frequency = index * (sample_rate / fft_size)
			let frequency = max_index as f32 * (44100.0 / 1024.0);
			info!("Dominant frequency detected: {:.2} Hz (magnitude: {:.6})", 
                 frequency, magnitudes[max_index]);

			// Check if magnitude exceeds a meaningful threshold
			// This helps avoid reporting noise as significant frequencies
			if magnitudes[max_index] > 0.01 {
				info!("Significant sound detected at {:.2} Hz", frequency);
			}
		}

		// Small delay to avoid starving other tasks
		// This introduces a 100ms gap between consecutive FFT analyses
		info!("Waiting before next analysis cycle");
		FreeRtos::delay_ms(1000);
	}
}