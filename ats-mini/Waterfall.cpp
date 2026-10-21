#include "Waterfall.h"
#include "Common.h"
#include "Themes.h"
#include "Utils.h"
#include "Menu.h"
#include "Draw.h"
#include "AudioCapture.h"
#include "Ft8Local.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static uint8_t *wfRows = nullptr;
static uint16_t wfHead = 0;
static uint16_t wfCount = 0;
static char wfMsgs[WATERFALL_MSG_LINES][WATERFALL_MSG_LEN + 1];
static uint8_t wfMsgHead = 0;
static uint8_t wfMsgCount = 0;
static uint32_t wfLastLocalMs = 0;
static bool wfDirty = true;
static constexpr uint32_t WF_IDLE_MS = 3000;

static inline uint8_t *rowPtr(uint16_t index)
{
  return wfRows + (size_t)index * WATERFALL_BINS;
}

void waterfallInit()
{
  if(wfRows) return;
  wfRows = static_cast<uint8_t *>(ps_malloc(WATERFALL_ROWS * WATERFALL_BINS));
  if(!wfRows) return;
  memset(wfRows, 0, WATERFALL_ROWS * WATERFALL_BINS);
  memset(wfMsgs, 0, sizeof(wfMsgs));
  wfHead = wfCount = 0;
  wfMsgHead = wfMsgCount = 0;
  wfDirty = true;
}

bool waterfallPushRow(const uint8_t *bins, size_t len)
{
  if(!bins || len != WATERFALL_BINS) return false;
  waterfallInit();
  if(!wfRows) return false;

  memcpy(rowPtr(wfHead), bins, WATERFALL_BINS);
  wfHead = (wfHead + 1) % WATERFALL_ROWS;
  if(wfCount < WATERFALL_ROWS) wfCount++;
  wfLastLocalMs = millis();
  wfDirty = true;
  return true;
}

bool waterfallPushMessage(const char *text)
{
  if(!text) return false;
  waterfallInit();

  char *dst = wfMsgs[wfMsgHead];
  size_t i = 0;
  for(; text[i] && i < WATERFALL_MSG_LEN; i++)
  {
    const char c = text[i];
    dst[i] = (c >= 0x20 && c <= 0x7e) ? c : '.';
  }
  dst[i] = 0;
  wfMsgHead = (wfMsgHead + 1) % WATERFALL_MSG_LINES;
  if(wfMsgCount < WATERFALL_MSG_LINES) wfMsgCount++;
  wfLastLocalMs = millis();
  wfDirty = true;
  return true;
}

static void waterfallClearSpectrum()
{
  if(wfRows)
    memset(wfRows, 0, WATERFALL_ROWS * WATERFALL_BINS);
  wfHead = 0;
  wfCount = 0;
  wfDirty = true;
}

bool waterfallTick(uint32_t now)
{
  if(uiLayoutIdx != UI_WATERFALL) return false;
  waterfallInit();
  if(!wfRows) return false;

  // Keep spectrum while ADC capture is enabled (do not wipe between slow rows).
  const bool localLive = wfLastLocalMs && (now - wfLastLocalMs) < WF_IDLE_MS;
  const bool adcLive = audioCaptureIsEnabled() && audioCaptureRingCount() > 0;
  if(!localLive && !adcLive && wfCount)
    waterfallClearSpectrum();

  // Force redraw while live so new rows appear even if other UI is idle.
  if(localLive || adcLive) wfDirty = true;

  return wfDirty;
}

// Digipan-style LUT: navy → cyan → green → yellow → red.
static const uint8_t digipanStops[][4] = {
  { 0,   0,   0,  24 },
  { 24,  0,  24,  96 },
  { 48,  0,  80, 180 },
  { 80,  0, 180, 200 },
  { 120, 0, 220,  80 },
  { 160, 40, 220,  0 },
  { 200, 200, 200, 0 },
  { 240, 255, 120, 0 },
  { 255, 255,  40,  0 },
};

static uint16_t digipanColor(uint8_t v)
{
  const size_t n = sizeof(digipanStops) / sizeof(digipanStops[0]);
  size_t i = 0;
  while(i + 1 < n && v > digipanStops[i + 1][0]) i++;
  if(i + 1 >= n) i = n - 2;
  const uint8_t *a = digipanStops[i];
  const uint8_t *b = digipanStops[i + 1];
  const float t = (b[0] == a[0]) ? 0.0f : (float)(v - a[0]) / (float)(b[0] - a[0]);
  const uint8_t r = (uint8_t)lroundf(a[1] + t * (b[1] - a[1]));
  const uint8_t g = (uint8_t)lroundf(a[2] + t * (b[2] - a[2]));
  const uint8_t bl = (uint8_t)lroundf(a[3] + t * (b[3] - a[3]));
  return spr.color565(r, g, bl);
}

void drawLayoutWaterfall()
{
  waterfallInit();
  const uint32_t nowBadge = millis();
  const bool localLive = wfLastLocalMs && (nowBadge - wfLastLocalMs) < WF_IDLE_MS;

  // Leave the left column for the same side-bar menu as other layouts.
  const int menuW = 76 + MENU_DELTA_X + 2;
  const int contentX = menuW;
  const int contentW = DISPLAY_WIDTH - contentX;

  drawSaveIndicator(SAVE_OFFSET_X, SAVE_OFFSET_Y);

  // Top status strip (compact), right of the menu column.
  spr.setTextDatum(TL_DATUM);
  spr.setTextColor(TH.text, TH.bg);
  char freqBuf[24];
  snprintf(freqBuf, sizeof(freqBuf), "%.3f", currentFrequency / 1000.0);
  spr.drawString(freqBuf, contentX + 2, 1, FONT_SMALL);
  spr.setTextColor(localLive ? TH.text : TH.text_muted, TH.bg);
  spr.drawString(localLive ? "LOCAL" : "IDLE", contentX + contentW - 90, 1, FONT_SMALL);
#if AUDIO_ADC_CAPTURE
  if(audioCaptureIsActive())
  {
    // Always-visible ADC bias/rms (does not require Mode=FT8).
    const int bias = audioCaptureDcBias();
    char adcBuf[28];
    snprintf(adcBuf, sizeof(adcBuf), "b=%d %.2f", bias, audioCaptureRms());
    spr.setTextColor((bias < 1200 || bias > 3000) ? TH.text_warn : TH.text_muted, TH.bg);
    spr.drawString(adcBuf, contentX + contentW - 72, 1, FONT_SMALL);
  }
#endif

  const int top = 14;
  const int wfH = (DISPLAY_HEIGHT - top) / 3;
  const int logY = top + wfH;
  const int logH = DISPLAY_HEIGHT - logY;

  // Spectrum: newest row at bottom of waterfall band.
  if(wfRows && wfCount)
  {
    const int rowH = 1;
    const int maxRows = wfH / rowH;
    const int drawRows = wfCount < (uint16_t)maxRows ? (int)wfCount : maxRows;
    for(int r = 0; r < drawRows; r++)
    {
      const int age = drawRows - 1 - r; // 0 = newest
      const uint16_t idx = (wfHead + WATERFALL_ROWS - 1 - age) % WATERFALL_ROWS;
      const uint8_t *row = rowPtr(idx);
      const int y = top + (wfH - drawRows * rowH) + r * rowH;
      for(size_t x = 0; x < WATERFALL_BINS; x++)
      {
        const int x0 = contentX + (int)(x * contentW / WATERFALL_BINS);
        const int x1 = contentX + (int)((x + 1) * contentW / WATERFALL_BINS);
        spr.fillRect(x0, y, x1 - x0, rowH, digipanColor(row[x]));
      }
    }
  }
  else
  {
    spr.setTextDatum(MC_DATUM);
    spr.setTextColor(TH.text_muted, TH.bg);
#if AUDIO_ADC_CAPTURE
    char waitBuf[40];
    if(!audioCaptureIsActive())
      snprintf(waitBuf, sizeof(waitBuf), "ADC GPIO%d init fail", AUDIO_ADC_GPIO);
    else if(audioCaptureRingCount() < 128)
      snprintf(waitBuf, sizeof(waitBuf), "GPIO%d waiting rms=%.3f",
               AUDIO_ADC_GPIO, audioCaptureRms());
    else
      snprintf(waitBuf, sizeof(waitBuf), "GPIO%d rms=%.3f (no row yet)",
               AUDIO_ADC_GPIO, audioCaptureRms());
    spr.drawString(waitBuf, contentX + contentW / 2, top + wfH / 2, FONT_SMALL);
#else
    spr.drawString("ADC capture disabled", contentX + contentW / 2, top + wfH / 2, FONT_SMALL);
#endif
  }

  // Separator
  spr.drawFastHLine(contentX, logY, contentW, TH.box_border);

  // Decode table (FT8 mode) — HH:MM:SS message ±dB
  spr.setTextDatum(TL_DATUM);
  const int lineH = 10;
  int y = logY + 2;
  if(currentMode == FT8)
  {
    char st[64];
    ft8LocalStatus(st, sizeof(st));
    spr.setTextColor(TH.text_warn, TH.bg);
    spr.drawString(st, contentX + 2, y, FONT_SMALL);
    y += lineH;

    spr.setTextColor(TH.text_muted, TH.bg);
    spr.drawString("UTC      MESSAGE               dB", contentX + 2, y, FONT_DEFAULT);
    y += lineH;
    spr.drawFastHLine(contentX, y, contentW, TH.box_border);
    y += 1;

    spr.setTextColor(TH.text, TH.bg);
    const int maxLines = (logY + logH - y) / lineH;
    const int n = wfMsgCount < (uint8_t)maxLines ? (int)wfMsgCount : maxLines;
    if(n == 0)
    {
      spr.setTextColor(TH.text_muted, TH.bg);
      spr.drawString("Listening… (need ~15s slot)", contentX + 2, y, FONT_DEFAULT);
    }
    for(int i = 0; i < n; i++)
    {
      const int age = n - 1 - i;
      const uint8_t idx = (wfMsgHead + WATERFALL_MSG_LINES - 1 - age) % WATERFALL_MSG_LINES;
      spr.drawString(wfMsgs[idx], contentX + 2, y + i * lineH, FONT_DEFAULT);
    }
  }
  else
  {
    spr.setTextColor(TH.text_muted, TH.bg);
    spr.drawString("Mode → FT8 for decode table", contentX + 2, y, FONT_DEFAULT);
  }

  // Same left menu / info bar as Default and S-Meter (open UI Layout to leave Waterfall).
  drawSideBar(currentCmd, MENU_OFFSET_X, MENU_OFFSET_Y, MENU_DELTA_X);

  wfDirty = false;
}
