#include "AudioCapture.h"
#include "Common.h"

#if AUDIO_ADC_CAPTURE

#include <Arduino.h>
#include <string.h>
#include <math.h>
#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include "src/kiss_fftr.h"

static constexpr uint32_t kAdcSampleHz = 24000;
static constexpr size_t kAdcReadMax = 256;

static adc_continuous_handle_t adcCont = nullptr;
static adc_oneshot_unit_handle_t adcOne = nullptr;
static adc_channel_t adcChannel = ADC_CHANNEL_0;
static adc_unit_t adcUnit = ADC_UNIT_2;
static bool useContinuous = false;
static int16_t *ring12k = nullptr;
static size_t ringWrite = 0;
static size_t ringCount = 0;
static size_t ringConsume = 0;
static size_t ringUnread = 0;
static float lastRms = 0.0f;
static float lastPeak = 0.0f;
static uint32_t lastRmsLogMs = 0;
static bool captureOk = false;
static int32_t dsAcc = 0;
static int dsCount = 0;
static int dcBias = 2048;
static int64_t dcAcc = 0;
static int dcN = 0;
static uint32_t oneshotNextUs = 0;
// Weak VOP/LOUT taps need gain; adapt toward ~0.25 peak after gain.
static float digGain = 16.0f;
static float preGainPeakEma = 0.0f;
static uint32_t rateWindowSamples = 0;
static uint32_t rateWindowStartMs = 0;
static float measuredHz = 0.0f;
static bool captureEnabled = false;

static void ringPush(int16_t sample)
{
  if(!ring12k) return;
  if(ringUnread == AUDIO_RING_SAMPLES)
  {
    // Drop oldest unread sample so producers never block.
    ringConsume = (ringConsume + 1) % AUDIO_RING_SAMPLES;
    ringUnread--;
  }
  ring12k[ringWrite] = sample;
  ringWrite = (ringWrite + 1) % AUDIO_RING_SAMPLES;
  ringUnread++;
  if(ringCount < AUDIO_RING_SAMPLES) ringCount++;
  rateWindowSamples++;
}

static void ingestRaw12(int raw12, double *sumSq, float *peak, size_t *monoCount)
{
  dcAcc += raw12;
  dcN++;
  if(dcN >= 2048)
  {
    dcBias = (int)(dcAcc / dcN);
    dcAcc = 0;
    dcN = 0;
  }

  int centered = raw12 - dcBias;
  if(centered > 2047) centered = 2047;
  if(centered < -2048) centered = -2048;
  // 12-bit → int16 scale, then digital gain for quiet taps (max vol rms~0.03).
  int32_t s = (int32_t)lroundf((float)(centered * 16) * digGain);
  if(s > 32767) s = 32767;
  if(s < -32768) s = -32768;

  const float a = fabsf((float)s);
  if(a > *peak) *peak = a;
  *sumSq += (double)s * (double)s;
  (*monoCount)++;

  const float pre = fabsf((float)(centered * 16)) / 32768.0f;
  if(pre > preGainPeakEma) preGainPeakEma = 0.9f * preGainPeakEma + 0.1f * pre;
  else preGainPeakEma = 0.99f * preGainPeakEma + 0.01f * pre;

  dsAcc += s;
  dsCount++;
  if(dsCount >= 2)
  {
    int32_t v = dsAcc / 2;
    dsAcc = 0;
    dsCount = 0;
    if(v > 32767) v = 32767;
    if(v < -32768) v = -32768;
    ringPush((int16_t)v);
  }
}

static bool initContinuous()
{
  adc_continuous_handle_cfg_t handleCfg = {
    .max_store_buf_size = 4096,
    .conv_frame_size = 256,
  };
  esp_err_t err = adc_continuous_new_handle(&handleCfg, &adcCont);
  if(err != ESP_OK)
  {
    Serial.printf("AudioCapture: continuous new_handle %d\n", (int)err);
    adcCont = nullptr;
    return false;
  }

  // 11 dB (~0–2.5 V): SI4732 LOUT is mid-rail biased (~1.6 V). 0 dB (~1.1 V) saturates.
  adc_digi_pattern_config_t pattern = {
    .atten = ADC_ATTEN_DB_11,
    .channel = adcChannel,
    .unit = adcUnit,
    .bit_width = ADC_BITWIDTH_12,
  };
  adc_continuous_config_t digiCfg = {
    .pattern_num = 1,
    .adc_pattern = &pattern,
    .sample_freq_hz = kAdcSampleHz,
    .conv_mode = (adcUnit == ADC_UNIT_1) ? ADC_CONV_SINGLE_UNIT_1 : ADC_CONV_SINGLE_UNIT_2,
    .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
  };
  err = adc_continuous_config(adcCont, &digiCfg);
  if(err != ESP_OK)
  {
    Serial.printf("AudioCapture: continuous config %d\n", (int)err);
    adc_continuous_deinit(adcCont);
    adcCont = nullptr;
    return false;
  }
  // Do not start here — continuous SAR switching couples into LOUT/speaker.
  return true;
}

static bool initOneshot()
{
  adc_oneshot_unit_init_cfg_t unitCfg = {
    .unit_id = adcUnit,
  };
  esp_err_t err = adc_oneshot_new_unit(&unitCfg, &adcOne);
  if(err != ESP_OK)
  {
    Serial.printf("AudioCapture: oneshot new_unit %d\n", (int)err);
    adcOne = nullptr;
    return false;
  }
  adc_oneshot_chan_cfg_t chanCfg = {
    .atten = ADC_ATTEN_DB_11,
    .bitwidth = ADC_BITWIDTH_12,
  };
  err = adc_oneshot_config_channel(adcOne, adcChannel, &chanCfg);
  if(err != ESP_OK)
  {
    Serial.printf("AudioCapture: oneshot config %d\n", (int)err);
    adc_oneshot_del_unit(adcOne);
    adcOne = nullptr;
    return false;
  }
  return true;
}

void audioCaptureInit()
{
  if(captureOk) return;

  ring12k = static_cast<int16_t *>(ps_malloc(AUDIO_RING_SAMPLES * sizeof(int16_t)));
  if(!ring12k)
  {
    Serial.println("AudioCapture: PSRAM ring alloc failed");
    return;
  }
  memset(ring12k, 0, AUDIO_RING_SAMPLES * sizeof(int16_t));
  ringWrite = ringCount = ringConsume = ringUnread = 0;

  if(adc_oneshot_io_to_channel(AUDIO_ADC_GPIO, &adcUnit, &adcChannel) != ESP_OK)
  {
    Serial.printf("AudioCapture: GPIO%d not an ADC pin\n", AUDIO_ADC_GPIO);
    return;
  }

  useContinuous = initContinuous();
  if(!useContinuous)
  {
    Serial.println("AudioCapture: falling back to oneshot ADC");
    if(!initOneshot()) return;
  }

  captureOk = true;
  captureEnabled = false;
  oneshotNextUs = micros();
  // High-Z until Waterfall/FT8 enables capture (reduces boot/speaker buzz into LOUT).
  pinMode(AUDIO_ADC_GPIO, INPUT);
  Serial.printf("AudioCapture: GPIO%d %s ready (idle until Waterfall/FT8)\n",
                AUDIO_ADC_GPIO, useContinuous ? "continuous" : "oneshot");
}

void audioCaptureSetEnabled(bool on)
{
  if(!captureOk) return;
  if(on == captureEnabled) return;
  captureEnabled = on;

  if(useContinuous && adcCont)
  {
    if(on)
    {
      esp_err_t err = adc_continuous_start(adcCont);
      if(err != ESP_OK)
        Serial.printf("AudioCapture: continuous start %d\n", (int)err);
      else
        Serial.println("AudioCapture: ADC on (speaker amp should be muted)");
    }
    else
    {
      adc_continuous_stop(adcCont);
      // Fully detach pad from ADC — stop alone still couples buzz into LOUT.
      gpio_reset_pin((gpio_num_t)AUDIO_ADC_GPIO);
      pinMode(AUDIO_ADC_GPIO, INPUT);
      Serial.println("AudioCapture: ADC detached from GPIO11");
    }
  }
  else if(!on)
  {
    gpio_reset_pin((gpio_num_t)AUDIO_ADC_GPIO);
    pinMode(AUDIO_ADC_GPIO, INPUT);
  }

  if(!on)
  {
    ringWrite = ringCount = ringConsume = ringUnread = 0;
    lastRms = lastPeak = 0.0f;
  }
}

bool audioCaptureIsEnabled()
{
  return captureOk && captureEnabled;
}

void audioCaptureService()
{
  if(!captureOk || !ring12k || !captureEnabled) return;

  double sumSq = 0.0;
  float peak = 0.0f;
  size_t monoCount = 0;

  if(useContinuous && adcCont)
  {
    uint8_t raw[kAdcReadMax * sizeof(adc_digi_output_data_t)];
    uint32_t nBytes = 0;
    esp_err_t err = adc_continuous_read(adcCont, raw, sizeof(raw), &nBytes, 0);
    if(err == ESP_OK && nBytes >= sizeof(adc_digi_output_data_t))
    {
      const size_t n = nBytes / sizeof(adc_digi_output_data_t);
      const adc_digi_output_data_t *samples =
        reinterpret_cast<const adc_digi_output_data_t *>(raw);
      for(size_t i = 0; i < n; i++)
      {
#if SOC_ADC_DIGI_RESULT_BYTES == 4
        ingestRaw12((int)samples[i].type2.data, &sumSq, &peak, &monoCount);
#else
        ingestRaw12((int)samples[i].type1.data, &sumSq, &peak, &monoCount);
#endif
      }
    }
  }
  else if(adcOne)
  {
    // Pace oneshot reads toward ~24 kHz (then 2:1 → 12 kHz).
    const uint32_t periodUs = 1000000UL / kAdcSampleHz;
    uint32_t now = micros();
    // Cap work per loop so UI stays responsive.
    for(int n = 0; n < 64; n++)
    {
      if((int32_t)(now - oneshotNextUs) < 0) break;
      oneshotNextUs += periodUs;
      if((int32_t)(now - oneshotNextUs) > 5000)
        oneshotNextUs = now; // avoid spiral after stalls

      int raw12 = 0;
      if(adc_oneshot_read(adcOne, adcChannel, &raw12) != ESP_OK) break;
      ingestRaw12(raw12, &sumSq, &peak, &monoCount);
      now = micros();
    }
  }

  if(monoCount)
  {
    lastPeak = peak / 32768.0f;
    lastRms = sqrtf((float)(sumSq / (double)monoCount)) / 32768.0f;
  }

  const uint32_t nowMs = millis();
  if(rateWindowStartMs == 0)
    rateWindowStartMs = nowMs;
  else if(nowMs - rateWindowStartMs >= 1000)
  {
    const uint32_t elapsed = nowMs - rateWindowStartMs;
    measuredHz = (float)rateWindowSamples * 1000.0f / (float)elapsed;
    rateWindowSamples = 0;
    rateWindowStartMs = nowMs;
  }

  if(nowMs - lastRmsLogMs >= 500)
  {
    lastRmsLogMs = nowMs;
    // Adapt gain toward ~0.20 peak (pre-gain). Fast attack when hot (nearby TX).
    const float target = 0.20f;
    float g = digGain;
    if(preGainPeakEma > 1e-4f)
      g = digGain * (target / preGainPeakEma);
    if(g < 0.25f) g = 0.25f;
    // Cap boost so mute/idle noise cannot be amplified into a full waterfall.
    if(g > 24.0f) g = 24.0f;
    if(g < digGain)
      digGain = 0.3f * digGain + 0.7f * g; // fast reduce when too loud
    else
      digGain = 0.9f * digGain + 0.1f * g; // slow raise when quiet

    Serial.printf("AudioCapture rms=%.4f peak=%.4f gain=%.1f rate=%.0f bias=%d ring=%u unread=%u mode=%s\n",
                  lastRms, lastPeak, digGain, measuredHz, dcBias,
                  (unsigned)ringCount, (unsigned)ringUnread,
                  useContinuous ? "cont" : "oneshot");
  }
}

bool audioCaptureIsActive()
{
  return captureOk;
}

size_t audioCaptureRingCount()
{
  return ringCount;
}

float audioCaptureRms()
{
  return lastRms;
}

float audioCapturePeak()
{
  return lastPeak;
}

float audioCaptureMeasuredHz()
{
  return measuredHz;
}

float audioCaptureDigGain()
{
  return digGain;
}

int audioCaptureDcBias()
{
  return dcBias;
}

size_t audioCaptureCopyRecent(int16_t *dst, size_t maxSamples)
{
  if(!ring12k || !dst || maxSamples == 0 || ringCount == 0) return 0;
  const size_t n = ringCount < maxSamples ? ringCount : maxSamples;
  size_t idx = (ringWrite + AUDIO_RING_SAMPLES - n) % AUDIO_RING_SAMPLES;
  for(size_t i = 0; i < n; i++)
  {
    dst[i] = ring12k[idx];
    idx = (idx + 1) % AUDIO_RING_SAMPLES;
  }
  return n;
}

size_t audioCaptureUnread()
{
  return ringUnread;
}

size_t audioCaptureConsume(int16_t *dst, size_t n)
{
  if(!ring12k || !dst || n == 0 || ringUnread < n) return 0;
  for(size_t i = 0; i < n; i++)
  {
    dst[i] = ring12k[ringConsume];
    ringConsume = (ringConsume + 1) % AUDIO_RING_SAMPLES;
  }
  ringUnread -= n;
  return n;
}

void audioCaptureFlushUnread()
{
  ringConsume = ringWrite;
  ringUnread = 0;
}

static void scaleMagsToRow(const float *mags, size_t bins, uint8_t *dst, size_t nBins)
{
  // Mix absolute floor with mild per-row contrast so real LOUT audio is visible
  // without turning mute/noise into a full-bright waterfall.
  float hi = mags[0];
  for(size_t k = 1; k < bins; k++)
    if(mags[k] > hi) hi = mags[k];
  const float ref = fmaxf(hi, 0.008f);
  for(size_t k = 0; k < nBins; k++)
  {
    const size_t src = k < bins ? k : bins - 1;
    float t = mags[src] / ref;
    if(t < 0.0f) t = 0.0f;
    if(t > 1.0f) t = 1.0f;
    // Quiet rows stay dark: scale by absolute hi so noise-only stays dim.
    const float absGate = fminf(hi / 0.02f, 1.0f);
    t *= absGate;
    t = powf(t, 0.75f);
    dst[k] = (uint8_t)lroundf(t * 255.0f);
  }
}

bool audioCaptureSpectrumRow(uint8_t *dst, size_t nBins, float hzMax)
{
  if(!dst || nBins == 0 || !ring12k || ringCount < 64) return false;

  // Small FFT in PSRAM — avoid large stack buffers (was breaking row push).
  constexpr int N = 256;
  static int16_t *tmp = nullptr;
  static float *timedom = nullptr;
  static kiss_fft_cpx *freqdom = nullptr;
  static kiss_fftr_cfg cfg = nullptr;
  static void *fftWork = nullptr;
  static bool fftFailed = false;

  if(!tmp)
  {
    tmp = static_cast<int16_t *>(ps_malloc(N * sizeof(int16_t)));
    if(!tmp) return false;
  }

  const size_t need = ringCount < (size_t)N ? ringCount : (size_t)N;
  if(need < 64) return false;
  memset(tmp, 0, N * sizeof(int16_t));
  if(audioCaptureCopyRecent(tmp + (N - need), need) < need) return false;

  float magsStack[128];
  const size_t bins = nBins < 128 ? nBins : 128;

  if(!fftFailed && !cfg)
  {
    size_t workSize = 0;
    kiss_fftr_alloc(N, 0, nullptr, &workSize);
    fftWork = ps_malloc(workSize);
    timedom = static_cast<float *>(ps_malloc(N * sizeof(float)));
    freqdom = static_cast<kiss_fft_cpx *>(ps_malloc((N / 2 + 1) * sizeof(kiss_fft_cpx)));
    if(!fftWork || !timedom || !freqdom)
    {
      fftFailed = true;
      Serial.println("AudioCapture: FFT alloc failed, using band-energy fallback");
    }
    else
    {
      cfg = kiss_fftr_alloc(N, 0, fftWork, &workSize);
      if(!cfg)
      {
        fftFailed = true;
        Serial.println("AudioCapture: kiss_fftr_alloc failed, using band-energy fallback");
      }
    }
  }

  if(cfg && timedom && freqdom)
  {
    for(int n = 0; n < N; n++)
    {
      const float hann = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * (float)n / (float)(N - 1));
      timedom[n] = hann * ((float)tmp[n] / 32768.0f);
    }
    kiss_fftr(cfg, timedom, freqdom);

    const float hzPerBin = (float)AUDIO_FT8_RATE_HZ / (float)N;
    const int maxK = (int)(hzMax / hzPerBin);
    for(size_t b = 0; b < bins; b++)
    {
      const int k0 = (int)((float)b * maxK / (float)bins);
      const int k1 = (int)((float)(b + 1) * maxK / (float)bins);
      float acc = 0.0f;
      int cnt = 0;
      for(int k = k0; k < k1 && k <= N / 2; k++)
      {
        if(k < 1) continue; // skip DC
        const float re = freqdom[k].r;
        const float im = freqdom[k].i;
        acc += sqrtf(re * re + im * im);
        cnt++;
      }
      magsStack[b] = cnt ? (acc / (float)cnt) : 0.0f;
    }
  }
  else
  {
    // Band-energy fallback: |x| in successive time slices (not true Hz, but scrolls).
    for(size_t b = 0; b < bins; b++)
    {
      const size_t i0 = (size_t)b * need / bins;
      const size_t i1 = (size_t)(b + 1) * need / bins;
      float acc = 0.0f;
      int cnt = 0;
      for(size_t i = i0; i < i1; i++)
      {
        acc += fabsf((float)tmp[N - need + i]);
        cnt++;
      }
      magsStack[b] = cnt ? (acc / (float)cnt / 32768.0f) : 0.0f;
    }
  }

  scaleMagsToRow(magsStack, bins, dst, nBins);
  return true;
}

#else // !AUDIO_ADC_CAPTURE

void audioCaptureInit() {}
void audioCaptureSetEnabled(bool) {}
bool audioCaptureIsEnabled() { return false; }
void audioCaptureService() {}
bool audioCaptureIsActive() { return false; }
size_t audioCaptureRingCount() { return 0; }
float audioCaptureRms() { return 0.0f; }
float audioCapturePeak() { return 0.0f; }
float audioCaptureMeasuredHz() { return 0.0f; }
float audioCaptureDigGain() { return 0.0f; }
int audioCaptureDcBias() { return 0; }
size_t audioCaptureCopyRecent(int16_t *, size_t) { return 0; }
size_t audioCaptureUnread() { return 0; }
size_t audioCaptureConsume(int16_t *, size_t) { return 0; }
void audioCaptureFlushUnread() {}
bool audioCaptureSpectrumRow(uint8_t *, size_t, float) { return false; }

#endif
