/*
 * ws2812_rmt - the driver's diagnostic surface.
 *
 * Not part of the led_strip API. "The driver says the frame failed" is two different
 * claims - the waveform never left the channel, or it left and completion was never
 * observed - and the return code cannot tell them apart. A board with no console can
 * only answer that if the readings reach the network, which is what this exposes.
 *
 * Copyright (c) 2026 maybetaken
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef WS2812_RMT_H
#define WS2812_RMT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WS2812_RMT_LAST_TX_WORDS 7

/* The last chunk's readings, as the transmit path recorded them. out[] is
 * {written, status, raw, passes, raddr_max, raw_before, limit}:
 *   written    channel status after the pushes, before start (did the symbols land);
 *   status     channel status at finish/give-up, sampled before the stop resets it;
 *   raw        raw interrupt word after the stop (done/threshold/error bits);
 *   passes     polls the wait took (== cap means it ran out);
 *   raddr_max  furthest mem_raddr_ex reached during the wait (0 = counter never ran,
 *              >= symbols = the frame was consumed even if done never announced);
 *   raw_before raw interrupt word one instruction before the stop;
 *   limit      the tx_lim this chunk asked to stop at.
 * rc is the driver's return for that chunk (0 = done observed). False if no chunk ran. */
bool ws2812_rmt_last_tx(uint32_t out[WS2812_RMT_LAST_TX_WORDS], int *rc);

#ifdef __cplusplus
}
#endif

#endif  /* WS2812_RMT_H */