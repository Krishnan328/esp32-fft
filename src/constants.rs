pub const SAMPLING_RATE: u32 = 48000; // Audio Sampling Rate in Hertz

/// Represents the number of samples that will be processed in a single FFT operation.
///
/// More specifically, it indicates that you're creating a 2048-point FFT, which means
/// the FFT will process 2048 complex data points at once. This parameter determines the
/// frequency resolution of your spectrum analysis - larger FFT sizes give you finer frequency
/// resolution but require more computational resources and introduce more latency.
pub const FFT_LENGTH: usize = 2048; // Keep 1024 for faster processing or 4096 for better resolution

pub const HANN_WINDOW_LENGHT: f32 = (FFT_LENGTH - 1) as f32;

pub const FREQUENCY_MAGNITUDE_LENGHT: usize = FFT_LENGTH / 2;

pub const FFT_LENGTH_BYTES: usize = FFT_LENGTH * 4;

pub const FREQ_BIN_WIDTH: f32 = SAMPLING_RATE as f32 / FFT_LENGTH as f32;

// Tune this value to remove noise of low amplitude from signal
pub const AMPLITUDE_THRESHOLD: AmplitudeThreshold = AmplitudeThreshold {
	frequency_cutoff: 1000.0,    // amplitude threshold boundary
	low_freq_threshold: 0.0005,  // For <frequency_cutoff (keep between 0.001 to 0.0005)
	high_freq_threshold: 0.0005, // For ≥frequency_cutoff (keep between 0.05 to 0.005)
};

pub const IMPULSE_THRESHOLD: f32 = 0.0012;
pub const IMPULSE_TIME_THRESHOLD: u64 = 100; // ms

pub static WIFI_SSID: &str = "ESP32-FFT-Analyzer";

pub const WIFI_PASSWORD: &str = "spectrum123";

pub const AUDIO_SAMPLE_DELTA: u64 = 2; // in milliseconds, for sps divide APS by 1000

pub struct AmplitudeThreshold {
	pub frequency_cutoff: f32,
	pub low_freq_threshold: f32,
	pub high_freq_threshold: f32,
}

pub struct ImpulseData {
	pub timestamp: u64,
	pub dominant_frequency: f32,
	pub peaks: Vec<PeakData>,
	pub coconut_type: String,
}

pub struct PeakData {
	pub index: usize,
	pub frequency: f32,
	pub magnitude: f32,
}
