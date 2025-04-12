use anyhow::Result;
use esp_idf_hal::modem::Modem;
use esp_idf_svc::http::server::ws::EspHttpWsConnection;
use esp_idf_svc::ws::FrameType;
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

	let server_state_ws = server_state.clone();
	// let server_state_fft = server_state.clone();

	// Serve the main HTML page
	server
		.fn_handler("/", esp_idf_svc::http::Method::Get, move |request| {
			let html = include_str!("../index.html");
			let mut resp = request.into_ok_response()?;
			resp.write(html.as_bytes())?;
			Ok::<(), EspIOError>(())
		})
		.expect("Failed to register index handler");

	// WebSocket endpoint
	server.ws_handler("/ws", move |ws: &mut EspHttpWsConnection| {
		let mut last_impulse_timestamp = None;
		loop {
			// Access the latest FFT data
			let current_data = {
				let read_guard = server_state_ws.read().unwrap();
				read_guard.clone()
			};

			// Serialize and send FFT data as binary
			let mut fft_data = Vec::with_capacity(FREQUENCY_MAGNITUDE_LENGHT * 4 + 4);
			for &mag in &current_data.magnitudes {
				fft_data.extend_from_slice(&mag.to_le_bytes());
			}
			fft_data.extend_from_slice(&current_data.dominant_frequency.to_le_bytes());
			if ws.send(FrameType::Binary(false), &fft_data).is_err() {
				info!("WebSocket client disconnected");
				break;
			}

			// Send impulse data as JSON if new
			if let Some(impulse) = &current_data.latest_impulse {
				if last_impulse_timestamp != Some(impulse.timestamp) {
					let peaks_json = impulse
						.peaks
						.iter()
						.map(|peak| {
							format!(
								"{{\"index\":{},\"frequency\":{},\"magnitude\":{}}}",
								peak.index, peak.frequency, peak.magnitude
							)
						})
						.collect::<Vec<String>>()
						.join(",");
					let json = format!(
					    "{{\"timestamp\":{},\"dominantFrequency\":{},\"peaks\":[{}],\"coconutType\":\"{}\"}}",
					    impulse.timestamp, impulse.dominant_frequency, peaks_json, impulse.coconut_type
					);
					if ws.send(FrameType::Text(false), json.as_bytes()).is_err() {
						info!("WebSocket client disconnected");
						break;
					}
					last_impulse_timestamp = Some(impulse.timestamp);
				}
			}

			// Control update rate (~60 Hz)
			std::thread::sleep(Duration::from_millis(AUDIO_UPDATE_PER_SECOND / 2));
		}
		Ok::<(), EspIOError>(())
	})?;

	info!("HTTP and WebSocket server started - Connect to ws://192.168.71.1/ws");
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
