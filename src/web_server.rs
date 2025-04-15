use anyhow::Result;
use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::{
	eventloop::EspSystemEventLoop,
	hal::modem::Modem,
	http::server::{ws::EspHttpWsConnection, Configuration, EspHttpServer},
	io::EspIOError,
	nvs::EspDefaultNvsPartition,
	wifi::{AccessPointConfiguration, BlockingWifi, EspWifi},
	ws::FrameType,
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

			// Add a header byte to indicate this is FFT data (e.g., 0x01)
			let mut message = vec![0x01];
			message.extend_from_slice(&fft_data);

			if ws.send(FrameType::Binary(false), &message).is_err() {
				info!("WebSocket client disconnected");
				break;
			}

			// Send impulse data as binary if new
			if let Some(impulse) = &current_data.latest_impulse {
				if last_impulse_timestamp != Some(impulse.timestamp) {
					// Create binary message for impulse data
					let mut impulse_data = Vec::new();

					// Add a header byte to indicate this is impulse data (e.g., 0x02)
					impulse_data.push(0x02);

					// Add timestamp (8 bytes)
					impulse_data.extend_from_slice(&impulse.timestamp.to_le_bytes());

					// Add dominant frequency (4 bytes)
					impulse_data.extend_from_slice(&impulse.dominant_frequency.to_le_bytes());

					// Add number of peaks (1 byte should be enough)
					impulse_data.push(impulse.peaks.len() as u8);

					// Add each peak data
					for peak in &impulse.peaks {
						// Add index (2 bytes should be enough)
						impulse_data.extend_from_slice(&(peak.index as u16).to_le_bytes());

						// Add frequency (4 bytes)
						impulse_data.extend_from_slice(&peak.frequency.to_le_bytes());

						// Add magnitude (4 bytes)
						impulse_data.extend_from_slice(&peak.magnitude.to_le_bytes());
					}

					// Add coconut type string length (1 byte)
					let coconut_type_bytes = impulse.coconut_type.as_bytes();
					impulse_data.push(coconut_type_bytes.len() as u8);

					// Add coconut type string
					impulse_data.extend_from_slice(coconut_type_bytes);

					if ws.send(FrameType::Binary(false), &impulse_data).is_err() {
						info!("WebSocket client disconnected");
						break;
					}
					last_impulse_timestamp = Some(impulse.timestamp);
				}
			}

			// Control update rate (~60 Hz)
			std::thread::sleep(Duration::from_millis(AUDIO_SAMPLE_PER_SECOND / 4));
		}
		Ok::<(), EspIOError>(())
	})?;

	info!("HTTP and WebSocket server started - Connect to ws://192.168.71.1/ws");
	Ok(server)
}

pub fn spawn_wifi_thread(modem: Modem, server_state: Arc<RwLock<Arc<FFTData>>>) -> Result<()> {
	let config = ThreadSpawnConfiguration {
		name: Some(b"Wifi Thread\0"),
		priority: 5,
		pin_to_core: Some(Core::Core1),
		..Default::default()
	};
	config.set().expect("Failed to set thread configuration");
	let wifi_thread_builder = thread::Builder::new()
		.stack_size(8192)
		.name("Wi-Fi Server".into());

	wifi_thread_builder
		.spawn(move || {
			info!(
				"WiFi server thread running on core: {:#?}",
				esp_idf_svc::hal::cpu::core()
			);
			let _wifi = init_wifi_ap(modem).expect("Failed to initialise Wi-Fi Access Point.");

			let _server =
				init_http_server(server_state).expect("Failed to initialise HTTP server.");

			thread::park();
		})
		.expect("Failed to spawn WiFi/server thread!");
	Ok(())
}
