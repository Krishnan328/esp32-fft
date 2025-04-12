use anyhow::Result;
use esp_idf_hal::modem::Modem;
use esp_idf_svc::{
	eventloop::EspSystemEventLoop,
	http::server::{Configuration, EspHttpServer},
	io::EspIOError,
	nvs::EspDefaultNvsPartition,
	wifi::AccessPointConfiguration,
	wifi::{BlockingWifi, EspWifi},
};
use log::*;
use std::{
	sync::{Arc, RwLock},
	thread,
	time::Duration,
};

use crate::constants::*;
use crate::FFTData;

pub fn init_wifi_module() -> Result<(EspDefaultNvsPartition, EspSystemEventLoop)> {
	let nvs = EspDefaultNvsPartition::take()?;

	let sysloop = EspSystemEventLoop::take()?;

	info!("Initializing the WIFI module for ESP...");

	Ok((nvs, sysloop))
}

pub fn init_wifi_ap(modem: Modem) -> Result<BlockingWifi<EspWifi<'static>>> {
	let (nvs, sysloop) = init_wifi_module()?;

	let mut wifi = BlockingWifi::wrap(
		EspWifi::new(modem, sysloop.clone(), Some(nvs)).unwrap(),
		sysloop,
	)
	.expect("Failed to initialise WiFi");

	wifi.set_configuration(&esp_idf_svc::wifi::Configuration::AccessPoint(
		AccessPointConfiguration {
			ssid: WIFI_SSID.try_into().unwrap(),
			password: WIFI_PASSWORD.try_into().unwrap(),
			auth_method: esp_idf_svc::wifi::AuthMethod::WPA2Personal,
			..Default::default()
		},
	))
	.expect("Failed to set WiFi configuration");

	wifi.start().expect("Failed to start WiFi");
	info!(
		"WiFi Access Point started: SSID=`{}`, Password=`{}`",
		WIFI_SSID, WIFI_PASSWORD
	);

	Ok(wifi)
}

pub fn init_http_server(server_state: Arc<RwLock<Arc<FFTData>>>) -> Result<EspHttpServer<'static>> {
	info!("Starting HTTP server...");
	let mut server =
		EspHttpServer::new(&Configuration::default()).expect("Failed to create HTTP server");

	let server_state_impulse = server_state.clone();
	let server_state_fft = server_state.clone();

	// Serve the main HTML page
	server
		.fn_handler("/", esp_idf_svc::http::Method::Get, move |request| {
			let html = include_str!("../index.html");
			let mut resp = request.into_ok_response()?;
			resp.write(html.as_bytes())?;
			Ok::<(), EspIOError>(())
		})
		.expect("Failed to register index handler");

	// server
	// 	.fn_handler(
	// 		"/api/latest_impulse",
	// 		esp_idf_svc::http::Method::Get,
	// 		move |request| {
	// 			let current_data = if let Ok(state) = server_state_impulse.read() {
	// 				state.clone()
	// 			} else {
	// 				let mut resp =
	// 					request.into_response(500, Some("Internal Server Error"), &[])?;
	// 				resp.write(b"Internal server error")?;
	// 				return Ok(());
	// 			};

	// 			let mut resp = request.into_response(
	// 				200,
	// 				Some("OK"),
	// 				&[("Content-Type", "application/json")],
	// 			)?;

	// 			if let Some(impulse) = &current_data.latest_impulse {
	// 				// Convert impulse data to JSON
	// 				let peaks_json = impulse
	// 					.peaks
	// 					.iter()
	// 					.map(|peak| {
	// 						format!(
	// 							"{{\"index\":{},\"frequency\":{},\"magnitude\":{}}}",
	// 							peak.index, peak.frequency, peak.magnitude
	// 						)
	// 					})
	// 					.collect::<Vec<String>>()
	// 					.join(",");

	// 				let json = format!(
	// 					"{{\"timestamp\":{},\"dominantFrequency\":{},\"peaks\":[{}]}}",
	// 					impulse.timestamp, impulse.dominant_frequency, peaks_json
	// 				);

	// 				resp.write(json.as_bytes())?;
	// 			} else {
	// 				resp.write(b"{\"status\":\"no_impulse\"}")?;
	// 			}

	// 			Ok::<(), EspIOError>(())
	// 		},
	// 	)
	// 	.expect("Failed to register latest impulse API handler");

	// API endpoint to get FFT data
	server
		.fn_handler("/api/fft", esp_idf_svc::http::Method::Get, move |request| {
			let current_data = if let Ok(state) = server_state_fft.read() {
				state.clone()
			} else {
				// Handle lock error gracefully
				let mut resp = request.into_response(500, Some("Internal Server Error"), &[])?;
				resp.write(b"Internal server error")?;
				return Ok(());
			};

			let mut resp = request.into_ok_response()?;
			let magnitudes_bytes: &[u8] = unsafe {
				std::slice::from_raw_parts(
					current_data.magnitudes.as_ptr() as *const u8,
					current_data.magnitudes.len() * std::mem::size_of::<f32>(),
				)
			};
			// Write dominant frequency as binary data
			let dominant_freq_bytes = current_data.dominant_frequency.to_le_bytes();

			resp.write(magnitudes_bytes)?;
			resp.write(&dominant_freq_bytes)?;
			Ok::<(), EspIOError>(())
		})
		.expect("Failed to register API handler");

	info!("HTTP server started - Connect to http://192.168.71.1");
	Ok(server)
}

pub fn spawn_wifi_thread(modem: Modem, server_state: Arc<RwLock<Arc<FFTData>>>) -> Result<()> {
	let wifi_thread_builder = thread::Builder::new()
		.stack_size(8192)
		.name("Wi-Fi Server".into());

	wifi_thread_builder
		.spawn(move || {
			let _wifi = init_wifi_ap(modem).expect("Failed to initialise Wi-Fi Access Point.");

			let _server =
				init_http_server(server_state).expect("Failed to initialise HTTP server.");

			loop {
				thread::sleep(Duration::from_millis(1000));
			}
		})
		.expect("Failed to spawn WiFi/server thread!");
	Ok(())
}
