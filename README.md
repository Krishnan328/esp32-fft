# ESP32-FFT

ESP32-FFT is a high-performance audio spectrum analyzer for ESP32 development boards. This powerful application leverages I2S digital audio input at 48kHz sample rate with 32-bit resolution to perform real-time frequency analysis.

## Features
- Real-time FFT processing at 200 samples per second
- 1024-bin FFT resolution for detailed spectrum analysis
- Compatible with I2S microphones (tested with INMP411 at 48kHz/24-bit)
- Optimized for ESP32 XTENSA core architecture
- Built with Rust using ESP-IDF-SVC framework and RUSTFFT library

## Applications
- Audio visualization systems
- Sound-reactive lighting
- Acoustic monitoring
- Audio frequency analysis
- DIY spectrum analyzers

## Performance
Achieves impressive processing speeds on standard ESP32 hardware without requiring additional DSP chips or external processing.

## Licensing
This code is open source but requires licensing for commercial applications. Please contact RedddFoxxyy( Suyog Tandel ) for exclusive commercial licensing options. Check LICENSE and LICENSE-CC files.
