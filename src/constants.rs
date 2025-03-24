pub const SAMPLING_RATE: u32 = 48000; // Audio Sampling Rate in Hertz

/// Represents the number of samples that will be processed in a single FFT operation.
///
/// More specifically, it indicates that you're creating a 2048-point FFT, which means
/// the FFT will process 2048 complex data points at once. This parameter determines the
/// frequency resolution of your spectrum analysis - larger FFT sizes give you finer frequency
/// resolution but require more computational resources and introduce more latency.
pub const FFT_LENGTH: usize = 2048;

pub const HANN_WINDOW_LENGHT: f32 = (FFT_LENGTH - 1) as f32;

pub const FREQUENCY_MAGNITUDE_LENGHT: usize = FFT_LENGTH / 2;

pub const FFT_LENGTH_BYTES: usize = FFT_LENGTH * 4;
