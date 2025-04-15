use embedded_graphics::{
	mono_font::{
		ascii::{FONT_6X10, FONT_9X15},
		MonoTextStyle,
	},
	pixelcolor::BinaryColor,
	prelude::*,
	text::{Baseline, Text},
};

use anyhow::Result;
use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use std::{
	sync::{Arc, RwLock},
	thread,
	time::Duration,
};

// sdl 26, sda 27
use esp_idf_svc::hal::{gpio::*, i2c::*, prelude::*};
use log::{error, info};
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

use crate::{constants::AUDIO_SAMPLE_PER_SECOND, FFTData};

pub fn init_oled_display(
	i2c: I2cDriver,
) -> Result<
	Ssd1306<I2CInterface<I2cDriver>, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>,
	anyhow::Error,
> {
	// Initialize the display
	let interface = I2CDisplayInterface::new(i2c);
	let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
		.into_buffered_graphics_mode();

	// Initialize and clear display
	display
		.init()
		.map_err(|e| anyhow::anyhow!("Display init error: {:?}", e))?;
	display
		.clear(BinaryColor::Off)
		.map_err(|e| anyhow::anyhow!("Display clear error: {:?}", e))?;

	// Create text style
	let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

	// Draw text
	Text::with_baseline(
		"Press red button",
		Point::new(0, 16),
		text_style,
		Baseline::Top,
	)
	.draw(&mut display)
	.map_err(|e| anyhow::anyhow!("Text draw error: {:?}", e))?;

	Text::with_baseline(
		"to start recording!",
		Point::new(0, 32),
		text_style,
		Baseline::Top,
	)
	.draw(&mut display)
	.map_err(|e| anyhow::anyhow!("Text draw error: {:?}", e))?;

	// Update display with all changes
	display
		.flush()
		.map_err(|e| anyhow::anyhow!("Display flush error: {:?}", e))?;

	Ok(display)
}

pub fn spawn_display_thread(
	system_state: Arc<RwLock<Arc<FFTData>>>,
	i2c0: I2C0,
	sda: Gpio14,
	scl: Gpio27,
) {
	let config = ThreadSpawnConfiguration {
		name: Some(b"Wifi Thread\0"),
		priority: 5,
		pin_to_core: Some(Core::Core1),
		..Default::default()
	};
	config.set().expect("Failed to set thread configuration");
	let display_thread_builder = thread::Builder::new()
		.stack_size(8192)
		.name("display thread".into());

	display_thread_builder
		.spawn(move || {
			// Initialize the display once in the thread
			info!("Initializing OLED display");

			// Create I2C configuration
			let i2c_config = I2cConfig::new().baudrate(400.kHz().into());

			// Create I2C driver
			let i2c_error = I2cDriver::new(i2c0, sda, scl, &i2c_config);
			if i2c_error.is_err() {
				info!("Failed to initialise I2C Driver!");
				return;
			}
			let i2c = i2c_error.unwrap();

			match init_oled_display(i2c) {
				Ok(mut display) => {
					// Display initialized successfully
					info!("OLED display initialized successfully");

					// Update display in a loop
					loop {
						// Access the latest FFT data
						let fft_data = {
							let read_guard = system_state.read().unwrap();
							read_guard.clone()
						};

						// Create text style
						let text_style = MonoTextStyle::new(&FONT_9X15, BinaryColor::On);
						let is_recording = &fft_data.is_recording;
						if let Some(impulse) = &fft_data.latest_impulse {
							display
								.clear(BinaryColor::Off)
								.map_err(|e| anyhow::anyhow!("Display clear error: {:?}", e))
								.unwrap();

							// Draw text
							Text::with_baseline(
								"Coconut Type:",
								Point::new(0, 16),
								text_style,
								Baseline::Top,
							)
							.draw(&mut display)
							.map_err(|e| anyhow::anyhow!("Text draw error: {:?}", e))
							.unwrap();

							// Draw text
							Text::with_baseline(
								impulse.coconut_type.as_str(),
								Point::new(0, 32),
								text_style,
								Baseline::Top,
							)
							.draw(&mut display)
							.map_err(|e| anyhow::anyhow!("Text draw error: {:?}", e))
							.unwrap();
						} else if !*is_recording {
							display
								.clear(BinaryColor::Off)
								.map_err(|e| anyhow::anyhow!("Display clear error: {:?}", e))
								.unwrap();

							Text::with_baseline(
								"Press red button",
								Point::new(0, 16),
								text_style,
								Baseline::Top,
							)
							.draw(&mut display)
							.map_err(|e| anyhow::anyhow!("Text draw error: {:?}", e))
							.unwrap();

							Text::with_baseline(
								"to start recording!",
								Point::new(0, 32),
								text_style,
								Baseline::Top,
							)
							.draw(&mut display)
							.map_err(|e| anyhow::anyhow!("Text draw error: {:?}", e))
							.unwrap();
						}

						// Update display with all changes
						display
							.flush()
							.map_err(|e| anyhow::anyhow!("Display flush error: {:?}", e))
							.unwrap();
						// Here you can update the display content
						// For example:
						// display.clear(BinaryColor::Off).unwrap();
						// display.flush().unwrap();

						thread::sleep(Duration::from_millis(AUDIO_SAMPLE_PER_SECOND / 4));
					}
				}
				Err(e) => {
					error!("Failed to initialize OLED display: {:?}", e);
				}
			}
		})
		.expect("Failed to spawn Display thread!");
}
