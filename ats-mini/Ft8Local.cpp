#include "Ft8Local.h"
#include "AudioCapture.h"
#include "Waterfall.h"
#include "Common.h"
#include "Menu.h"
#include "Utils.h"

#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

#if AUDIO_ADC_CAPTURE

#include "src/decode.h"
#include "src/message.h"
#include "src/constants.h"
#include "src/monitor.h"

static constexpr float FT8_SLOT_S = 15.0f;
static constexpr uint32_t WF_ROW_MS = 200;
// Decode knobs aligned with hoanb1/esp32s3-si4732-radio (working on-air fork).
static constexpr int K_MIN_SCORE = 4;
static constexpr int K_MAX_CANDIDATES = 100;
static constexpr int K_LDPC_ITERS = 35;
static constexpr int K_MAX_DECODED = 32;
static constexpr int K_FREQ_OSR = 1;
static constexpr int K_TIME_OSR = 1;
static constexpr float K_RATE_MIN = 8000.0f;
static constexpr float K_RATE_MAX = 16000.0f;
static constexpr float K_RATE_REINIT_PCT = 0.03f; // 3% drift → re-init
// SI4732 line level for GPIO11; volume ≤36 is near the ADC floor on LOUT taps.
static constexpr uint8_t K_ADC_VOLUME = 48;

static monitor_t mon;
static bool monReady = false;
static bool decoding = false;
static uint32_t lastRowMs = 0;
static int lastSlot = -1;
static float *symbolBuf = nullptr;
static int16_t *pcmBuf = nullptr;
static int lastCandidates = 0;
static int lastDecoded = 0;
static int lastCrcFail = 0;
static uint8_t lastMode = 0xFF;
static float configuredRate = 0.0f;
static float hpfX1 = 0.0f;
static float hpfY1 = 0.0f;
static bool waitSlotEdge = false;
static bool ampMutedForFt8 = false;

#define CALLSIGN_HASHTABLE_SIZE 128
static struct
{
  char callsign[12];
  uint32_t hash;
} callsign_hashtable[CALLSIGN_HASHTABLE_SIZE];

static void hashtable_init(void)
{
  memset(callsign_hashtable, 0, sizeof(callsign_hashtable));
}

extern "C" {

static void hashtable_add(const char *callsign, uint32_t hash)
{
  uint16_t hash10 = (hash >> 12) & 0x3FFu;
  int idx = (hash10 * 23) % CALLSIGN_HASHTABLE_SIZE;
  for(int n = 0; n < CALLSIGN_HASHTABLE_SIZE; n++)
  {
    if(callsign_hashtable[idx].callsign[0] == '\0')
    {
      strncpy(callsign_hashtable[idx].callsign, callsign, 11);
      callsign_hashtable[idx].callsign[11] = '\0';
      callsign_hashtable[idx].hash = hash;
      return;
    }
    if((callsign_hashtable[idx].hash & 0x3FFFFFu) == (hash & 0x3FFFFFu) &&
       strcmp(callsign_hashtable[idx].callsign, callsign) == 0)
    {
      callsign_hashtable[idx].hash = hash;
      return;
    }
    idx = (idx + 1) % CALLSIGN_HASHTABLE_SIZE;
  }
}

static bool hashtable_lookup(ftx_callsign_hash_type_t hash_type, uint32_t hash, char *callsign)
{
  uint8_t hash_shift = (hash_type == FTX_CALLSIGN_HASH_10_BITS) ? 12 :
                       (hash_type == FTX_CALLSIGN_HASH_12_BITS) ? 10 : 0;
  uint16_t hash10 = (hash >> (12 - hash_shift)) & 0x3FFu;
  int idx = (hash10 * 23) % CALLSIGN_HASHTABLE_SIZE;
  for(int n = 0; n < CALLSIGN_HASHTABLE_SIZE; n++)
  {
    if(callsign_hashtable[idx].callsign[0] == '\0') return false;
    if(((callsign_hashtable[idx].hash & 0x3FFFFFu) >> hash_shift) == hash)
    {
      strcpy(callsign, callsign_hashtable[idx].callsign);
      return true;
    }
    idx = (idx + 1) % CALLSIGN_HASHTABLE_SIZE;
  }
  return false;
}

} // extern "C"

static ftx_callsign_hash_interface_t hash_if = {
  .lookup_hash = hashtable_lookup,
  .save_hash = hashtable_add,
};

static int utcSecondInSlot(time_t now)
{
  if(now < 100000) return (int)((millis() / 1000U) % 15U);
  return (int)(now % (time_t)FT8_SLOT_S);
}

static int currentUtcSlot()
{
  time_t now = time(nullptr);
  if(now < 100000)
    return (int)(millis() / (uint32_t)(FT8_SLOT_S * 1000.0f));
  return (int)(now / (time_t)FT8_SLOT_S);
}

static void pushSpectrumRow()
{
  uint8_t bins[WATERFALL_BINS];
  if(!audioCaptureSpectrumRow(bins, WATERFALL_BINS, 3000.0f))
  {
    // No fabricated rows — quiet/black when FFT has nothing useful.
    memset(bins, 0, WATERFALL_BINS);
  }
  waterfallPushRow(bins, WATERFALL_BINS);
}

// Fixed linear scale for the whole slot (hoanb1): per-block peak AGC destroys SNR ratios.
static void fillSymbolFloats(const int16_t *pcm, size_t n)
{
  int64_t sum = 0;
  for(size_t i = 0; i < n; i++)
    sum += pcm[i];
  const float mean = (float)sum / (float)n;

  // One-pole HPF after mean removal (~200 Hz @ 12 kHz).
  const float R = 0.90f;
  const float normScale = 1.0f / 8192.0f; // ~+12 dB vs full-scale int16; stable across symbols
  for(size_t i = 0; i < n; i++)
  {
    float x = ((float)pcm[i] - mean) * normScale;
    float y = x - hpfX1 + R * hpfY1;
    hpfX1 = x;
    hpfY1 = y;
    if(y > 1.0f) y = 1.0f;
    if(y < -1.0f) y = -1.0f;
    symbolBuf[i] = y;
  }
}

void ft8ApplyAudio()
{
  // Only FT8 samples ADC (not all Waterfall modes). Amp mute while sampling so
  // LOUT kickback is not heard; SI4732 stays loud for GPIO11.
  const bool sampling = (currentMode == FT8);

  if(sampling)
  {
    if(!muteOn(MUTE_MAIN) && !muteOn(MUTE_SQUELCH))
    {
      rx.setVolume(K_ADC_VOLUME);
      rx.setAudioMute(false);
    }
    if(audioClamp) digitalWrite(AUDIO_MUTE, LOW);
#if PIN_AMP_EN >= 0
    digitalWrite(PIN_AMP_EN, LOW);
    ampMutedForFt8 = true;
#endif
  }
  else
  {
#if PIN_AMP_EN >= 0
    if(ampMutedForFt8 && !muteOn(MUTE_MAIN) && !muteOn(MUTE_SQUELCH) && !muteOn(MUTE_TEMP))
      digitalWrite(PIN_AMP_EN, HIGH);
#endif
    ampMutedForFt8 = false;
    if(!muteOn(MUTE_MAIN) && !muteOn(MUTE_SQUELCH))
    {
      rx.setVolume(volume);
      rx.setAudioMute(false);
    }
  }
}

bool ft8IsActive()
{
  return currentMode == FT8 && monReady;
}

static bool allocSymbolBufs(int blockSize)
{
  if(symbolBuf)
  {
    free(symbolBuf);
    symbolBuf = nullptr;
  }
  if(pcmBuf)
  {
    free(pcmBuf);
    pcmBuf = nullptr;
  }
  symbolBuf = static_cast<float *>(ps_malloc((size_t)blockSize * sizeof(float)));
  pcmBuf = static_cast<int16_t *>(ps_malloc((size_t)blockSize * sizeof(int16_t)));
  return symbolBuf && pcmBuf;
}

static bool initMonitorAtRate(float sampleRate)
{
  if(sampleRate < K_RATE_MIN) sampleRate = K_RATE_MIN;
  if(sampleRate > K_RATE_MAX) sampleRate = K_RATE_MAX;

  if(monReady)
  {
    monitor_free(&mon);
    monReady = false;
  }

  monitor_config_t cfg = {};
  cfg.f_min = 200.0f;
  cfg.f_max = 2900.0f;
  cfg.sample_rate = (int)lroundf(sampleRate);
  cfg.time_osr = K_TIME_OSR;
  cfg.freq_osr = K_FREQ_OSR;
  cfg.protocol = FTX_PROTOCOL_FT8;
  monitor_init(&mon, &cfg);

  if(!allocSymbolBufs(mon.block_size))
  {
    Serial.println("Ft8Local: buffer alloc failed");
    monitor_free(&mon);
    return false;
  }

  configuredRate = sampleRate;
  monReady = true;
  hpfX1 = hpfY1 = 0.0f;
  Serial.printf("Ft8Local: monitor @ %.0f Hz (block=%d nfft=%d)\n",
                sampleRate, mon.block_size, mon.nfft);
  return true;
}

static void maybeRecalibrateRate()
{
  const float hz = audioCaptureMeasuredHz();
  if(hz < K_RATE_MIN || hz > K_RATE_MAX) return;
  if(configuredRate <= 0.0f) return;

  const float err = fabsf(hz - configuredRate) / configuredRate;
  if(err < K_RATE_REINIT_PCT) return;

  // Only re-init between slots so we don't trash an in-progress capture.
  const int sec = utcSecondInSlot(time(nullptr));
  if(sec > 2) return;

  Serial.printf("Ft8Local: rate recalibrate %.0f → %.0f Hz\n", configuredRate, hz);
  initMonitorAtRate(hz);
  lastSlot = currentUtcSlot();
  audioCaptureFlushUnread();
}

static void decodeSlot()
{
  if(!monReady || decoding) return;
  decoding = true;

  ftx_candidate_t candidates[K_MAX_CANDIDATES];
  int num = ftx_find_candidates(&mon.wf, K_MAX_CANDIDATES, candidates, K_MIN_SCORE);
  lastCandidates = num;
  int crcFail = 0;

  Serial.printf("Ft8Local: decode blocks=%d candidates=%d rms=%.3f rate=%.0f\n",
                mon.wf.num_blocks, num, audioCaptureRms(), configuredRate);

  char messages[K_MAX_DECODED][128];
  int nMsg = 0;
  hashtable_init();

  for(int idx = 0; idx < num && nMsg < K_MAX_DECODED; idx++)
  {
    ftx_message_t msg;
    ftx_decode_status_t status;
    if(!ftx_decode_candidate(&mon.wf, &candidates[idx], K_LDPC_ITERS, &msg, &status))
      continue;
    if(status.crc_calculated != status.crc_extracted)
    {
      crcFail++;
      continue;
    }

    char text[128];
    ftx_message_offsets_t offsets;
    if(ftx_message_decode(&msg, &hash_if, text, &offsets) != FTX_MESSAGE_RC_OK)
      continue;

    bool dup = false;
    for(int i = 0; i < nMsg; i++)
    {
      if(strcmp(messages[i], text) == 0) { dup = true; break; }
    }
    if(dup) continue;
    strncpy(messages[nMsg], text, sizeof(messages[0]) - 1);
    messages[nMsg][sizeof(messages[0]) - 1] = '\0';
    nMsg++;

    const int snr = (int)candidates[idx].score / 4 - 20;
    char line[WATERFALL_MSG_LEN + 1];
    time_t now = time(nullptr);
    struct tm tmUtc;
    if(now > 100000 && gmtime_r(&now, &tmUtc))
    {
      snprintf(line, sizeof(line), "%02d:%02d:%02d %-20.20s%+3d",
               tmUtc.tm_hour, tmUtc.tm_min, tmUtc.tm_sec, text, snr);
    }
    else
    {
      snprintf(line, sizeof(line), "--:--:-- %-20.20s%+3d", text, snr);
    }
    waterfallPushMessage(line);
    Serial.printf("Ft8Local: %s\n", line);
  }

  lastDecoded = nMsg;
  lastCrcFail = crcFail;
  if(nMsg == 0)
  {
    char line[WATERFALL_MSG_LEN + 1];
    snprintf(line, sizeof(line), "(no decode c=%d blk=%d crc=%d)",
             num, mon.wf.num_blocks, crcFail);
    waterfallPushMessage(line);
  }

  monitor_reset(&mon);
  hpfX1 = hpfY1 = 0.0f;
  // Keep ring backlog — it is the start of the new slot (Costas sync).
  decoding = false;
}

void ft8LocalStatus(char *buf, size_t buflen)
{
  if(!buf || buflen == 0) return;
  if(currentMode != FT8)
  {
    snprintf(buf, buflen, "Set Mode to FT8");
    return;
  }
  if(!monReady)
  {
    snprintf(buf, buflen, "FT8 init failed");
    return;
  }

  const float rms = audioCaptureRms();
  const int sec = utcSecondInSlot(time(nullptr));
  const int blk = mon.wf.num_blocks;
  const int bias = audioCaptureDcBias();

  // LOUT is mid-rail (~1.6 V → bias ~1800–2200 @ 12-bit). Low bias = wrong pad / after coupling cap / open wire.
  if(bias < 1200 || bias > 3000)
  {
    snprintf(buf, buflen, "BAD TAP b=%d (want~2000)", bias);
    return;
  }

  snprintf(buf, buflen, "b=%d rms=%.2f t=%d/%d c=%d d=%d",
           bias, rms, sec, blk, lastCandidates, lastDecoded);
}

void ft8LocalInit()
{
  audioCaptureInit();
  if(!audioCaptureIsActive()) return;

  if(!initMonitorAtRate((float)AUDIO_FT8_RATE_HZ))
    return;
  hashtable_init();
}

void ft8LocalService(uint32_t nowMs)
{
  // ADC2 continuous + SSB patch load on the main loop freezes the UI — only
  // capture while Mode is FT8 (Waterfall alone must not keep the ADC running).
  const bool needAdc = (currentMode == FT8);
  static bool lastNeedAdc = false;
  if(needAdc != lastNeedAdc)
  {
    lastNeedAdc = needAdc;
    ft8ApplyAudio();
  }
  audioCaptureSetEnabled(needAdc);
  if(needAdc)
    audioCaptureService();

  if(needAdc && uiLayoutIdx == UI_WATERFALL && nowMs - lastRowMs >= WF_ROW_MS)
  {
    lastRowMs = nowMs;
    pushSpectrumRow();
  }

  // Reset decoder state when entering/leaving FT8 mode.
  if(currentMode != lastMode)
  {
    lastMode = currentMode;
    ft8ApplyAudio();
    if(currentMode == FT8 && monReady)
    {
      monitor_reset(&mon);
      lastSlot = currentUtcSlot();
      lastCandidates = lastDecoded = lastCrcFail = 0;
      hpfX1 = hpfY1 = 0.0f;
      audioCaptureFlushUnread();
      // Start capture on the next UTC :00/:15/:30/:45 (hoanb1 block-0 alignment).
      waitSlotEdge = true;
      Serial.println("Ft8Local: waiting for UTC slot edge");
    }
  }

  if(currentMode != FT8 || !monReady || !symbolBuf || !pcmBuf) return;

  maybeRecalibrateRate();
  if(!monReady || !symbolBuf || !pcmBuf) return;

  // Slot boundary first — decode/reset before feeding the new slot's audio.
  const int slot = currentUtcSlot();
  if(lastSlot < 0) lastSlot = slot;
  if(slot != lastSlot)
  {
    lastSlot = slot;
    if(waitSlotEdge)
    {
      waitSlotEdge = false;
      monitor_reset(&mon);
      hpfX1 = hpfY1 = 0.0f;
      audioCaptureFlushUnread();
      Serial.println("Ft8Local: slot edge — capture start");
    }
    else if(mon.wf.num_blocks >= 40)
      decodeSlot();
    else
    {
      Serial.printf("Ft8Local: skip decode blocks=%d\n", mon.wf.num_blocks);
      monitor_reset(&mon);
      hpfX1 = hpfY1 = 0.0f;
    }
  }

  if(waitSlotEdge)
  {
    audioCaptureFlushUnread();
    return;
  }

  // Feed sequential symbol-sized chunks; catch up quickly after a slow decode.
  const size_t need = (size_t)mon.block_size;
  int fed = 0;
  while(!decoding && audioCaptureUnread() >= need &&
        mon.wf.num_blocks < mon.wf.max_blocks && fed < 16)
  {
    if(audioCaptureConsume(pcmBuf, need) != need) break;
    fillSymbolFloats(pcmBuf, need);
    monitor_process(&mon, symbolBuf);
    fed++;
  }
}

#else // !AUDIO_ADC_CAPTURE

void ft8LocalInit() {}
void ft8LocalService(uint32_t) {}
void ft8LocalStatus(char *buf, size_t buflen)
{
  if(buf && buflen) snprintf(buf, buflen, "ADC off");
}
void ft8ApplyAudio() {}
bool ft8IsActive() { return false; }

#endif
