use anyhow::Result;
use esp_idf_hal::{delay::FreeRtos, modem::Modem};
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
	string::String,
	sync::{Arc, RwLock},
	thread,
};

use crate::constants::*;
use crate::SystemState;

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

pub fn init_http_server(server_state: Arc<RwLock<SystemState>>) -> Result<EspHttpServer<'static>> {
	info!("Starting HTTP server...");
	let mut server =
		EspHttpServer::new(&Configuration::default()).expect("Failed to create HTTP server");

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
	Ok(server)
}

pub fn spawn_wifi_thread(modem: Modem, server_state: Arc<RwLock<SystemState>>) -> Result<()> {
	let wifi_thread_builder = thread::Builder::new()
		.stack_size(8192)
		.name("Wi-Fi Server".into());

	wifi_thread_builder
		.spawn(move || {
			let _wifi = init_wifi_ap(modem).expect("Failed to initialise Wi-Fi Access Point.");

			let _server = init_http_server(server_state).expect("Failed to initialise HTTP server.");

			loop {
				FreeRtos::delay_ms(1000);
			}
		})
		.expect("Failed to spawn WiFi/server thread!");
	Ok(())
}
