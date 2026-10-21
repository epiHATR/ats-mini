#ifndef AUDIO_CAPTURE_H
#define AUDIO_CAPTURE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// On-device audio via ESP32-S3 ADC on GPIO11 (factory tap on V4a/V4b).
// No external ADC and no soldering required on those boards. Disabled on LILYGO.
#if !defined(LILYGO_SI473X)
#ifndef AUDIO_ADC_CAPTURE
#define AUDIO_ADC_CAPTURE 1
#endif
#else
#undef AUDIO_ADC_CAPTURE
#define AUDIO_ADC_CAPTURE 0
#endif

#if AUDIO_ADC_CAPTURE
#define AUDIO_ADC_GPIO 11
#endif

static constexpr int AUDIO_FT8_RATE_HZ = 12000;
// ~16 s of mono int16 at 12 kHz for one FT8 slot (+margin), in PSRAM.
static constexpr size_t AUDIO_RING_SAMPLES = AUDIO_FT8_RATE_HZ * 16;

void audioCaptureInit();
// SAR ADC kickback into LOUT is audible on the speaker — only run while Waterfall/FT8 needs it.
void audioCaptureSetEnabled(bool on);
bool audioCaptureIsEnabled();
void audioCaptureService();
bool audioCaptureIsActive();
size_t audioCaptureRingCount();
float audioCaptureRms();
float audioCapturePeak();
// Measured mono ring fill rate (Hz); 0 until the first 1 s window.
float audioCaptureMeasuredHz();
float audioCaptureDigGain();
int audioCaptureDcBias(); // raw 12-bit DC estimate (LOUT ≈ mid-scale ~1800–2200)

// Copy up to maxSamples of newest 12 kHz mono int16 audio (oldest→newest in dst).
size_t audioCaptureCopyRecent(int16_t *dst, size_t maxSamples);

// Sequential consume for FT8 (does not disturb spectrum copy-recent).
size_t audioCaptureUnread();
size_t audioCaptureConsume(int16_t *dst, size_t n);
void audioCaptureFlushUnread();

// Digipan-scaled magnitudes over 0..hzMax from recent audio.
bool audioCaptureSpectrumRow(uint8_t *dst, size_t nBins, float hzMax);

#endif /* AUDIO_CAPTURE_H */
