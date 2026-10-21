#ifndef FT8_LOCAL_H
#define FT8_LOCAL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// On-device FT8 from AudioCapture (ESP32 ADC on GPIO11). No-ops when AUDIO_ADC_CAPTURE=0.
void ft8LocalInit();
void ft8LocalService(uint32_t nowMs);

// Short status for Waterfall table (e.g. "FT8 blocks=72" / "LOWER VOLUME").
void ft8LocalStatus(char *buf, size_t buflen);

// Like hoanb1 FT8: SI4732 stays loud for the ADC; speaker amp is gated separately.
// Call after volume/mute/mode changes. No-op when capture is disabled.
void ft8ApplyAudio();
bool ft8IsActive();

#endif /* FT8_LOCAL_H */
