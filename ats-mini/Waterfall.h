#ifndef WATERFALL_H
#define WATERFALL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

static constexpr size_t WATERFALL_BINS = 100;
static constexpr size_t WATERFALL_ROWS = 100;
static constexpr size_t WATERFALL_MSG_LINES = 14;
static constexpr size_t WATERFALL_MSG_LEN = 42;

void waterfallInit();
bool waterfallPushRow(const uint8_t *bins, size_t len);
bool waterfallPushMessage(const char *text);
bool waterfallTick(uint32_t now);
void drawLayoutWaterfall();

#endif /* WATERFALL_H */
