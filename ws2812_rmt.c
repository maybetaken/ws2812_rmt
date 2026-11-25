/*
 * WS2812-class addressable LED driver for the ESP32 RMT peripheral.
 *
 * Implements Zephyr's led_strip API behind the compatible
 * "espressif,esp32-ws2812-rmt", so a board picks it in devicetree and the application
 * never learns which peripheral produced the waveform. Zephyr ships no led_strip driver
 * for Espressif RMT, which is why this lives in a module.
 *
 * RMT rather than CPU bit-banging: the strip tells a one from a zero by how long the
 * line is high (400 ns / 800 ns within a 1250 ns bit), and on an ESP32-C3 one GPIO
 * store through Zephyr's API costs 312 ns while one system-clock read costs 687 ns -
 * the loop's own instrumentation is coarser than the phase it has to control. RMT
 * counts 12.5 ns ticks off a hardware clock, so a phase is an exact tick count that no
 * CPU speed, ISR or register read can stretch, and a frame needs no interrupt blackout.
 *
 * Only Espressif's hal/ll layer is used: this HAL copy ships esp_hal_rmt but not the
 * IDF rmt driver, and the ll functions are all static inline.
 *
 * The measurements behind the per-part defines below are recorded in CHANGELOG.md
 * (2.0.3.18 through 2.0.3.21); the register field layouts are in soc/rmt_struct.h.
 *
 * Copyright (c) 2026 maybetaken
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT espressif_esp32_ws2812_rmt

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/dt-bindings/led/led.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <hal/rmt_ll.h>
#include <hal/rmt_periph.h>
#include <soc/clk_tree_defs.h>
#include <soc/rmt_struct.h>
#include <soc/soc.h>
#include <esp_rom_gpio.h>

#include <ws2812_rmt/ws2812_rmt.h>

/* What the hal/ll layer does not hide between these parts: where a symbol is written,
 * which channel registers to read back, the counter clock's name, and how a frame is
 * ended. All four are stated per part rather than guessed - a wrong guess compiles and
 * then shows up as an LED that never answers.
 *
 *                       ESP32-C3                    ESP32-C5 / C6
 *   symbol push register  data_ch[ch]                chndata[ch].chndata   (32 bit both)
 *   channel readback      tx_conf/tx_status/tx_lim   chnconf0/chnstatus/chn_tx_lim
 *   counter clock source  RMT_CLK_SRC_APB            RMT_CLK_SRC_PLL_F80M  (both 80 MHz)
 *   tx_lim means          last symbol index          number of entries
 *
 * kChannelsPerInstance is the size of the per-channel *config* array rather than the
 * data array because the vendor headers disagree between them (the C5 declares
 * chndata[4] with chnstatus[2]). Two is also what the hardware offers: the SoC signal
 * table has TX signals only for channels 0 and 1, the upper two are RX. */
#if defined(CONFIG_SOC_ESP32C3)
#define RMT_CHANNEL_DATA(dev, ch)      ((dev)->data_ch[ch])
#define RMT_CHANNEL_STATUS(dev, ch)    ((dev)->tx_status[ch].val)
#define RMT_CHANNEL_CONF(dev, ch)      ((dev)->tx_conf[ch].val)
#define RMT_CHANNEL_LIMIT(dev, ch)     ((dev)->tx_lim[ch].val)
#define RMT_TX_CLK_SRC                 RMT_CLK_SRC_APB
/* tx_lim is the index of the last symbol: a 24-symbol frame ends the transmitter at 23
 * and raises TX_DONE there (bench C3 probe: waddr 24, raddr 0 -> 25, raw 0x101 = done +
 * threshold, observed during the wait). 2*symbols-1 also raises done but only by reading
 * the stale entries past the data - see the tx_result note. */
#define RMT_TX_LIMIT(symbols)          ((symbols) - 1U)
#define kTxAppendEof                   false
#define kTxWrapAtLimit                 false
#define RMT_ARRAY_LEN(field) (sizeof(((rmt_dev_t *)0)->field) / sizeof(((rmt_dev_t *)0)->field[0]))
static const uint32_t kChannelsPerInstance = RMT_ARRAY_LEN(tx_conf);
#elif defined(CONFIG_SOC_ESP32C5) || defined(CONFIG_SOC_ESP32C6)
#define RMT_CHANNEL_DATA(dev, ch)      ((dev)->chndata[ch].chndata)
#define RMT_CHANNEL_STATUS(dev, ch)    ((dev)->chnstatus[ch].val)
#define RMT_CHANNEL_CONF(dev, ch)      ((dev)->chnconf0[ch].val)
#define RMT_CHANNEL_LIMIT(dev, ch)     ((dev)->chn_tx_lim[ch].val)
#define RMT_TX_CLK_SRC                 RMT_CLK_SRC_PLL_F80M
/* tx_lim counts entries on this generation ("the maximum entries that CHANNELn can send
 * out"), so a 24-symbol frame needs 24, not the C3's 23. Probed with
 * CONFIG_WS2812_RMT_PROBE: 23 never raised TX_DONE - the read pointer ran to the end of
 * the 48-entry block and only the threshold event fired; 24 raised done with no
 * mem_empty; 47 and 48 also raise done, but only by streaming the stale entries that
 * followed the data, which is a wrong colour rather than no colour. */
#define RMT_TX_LIMIT(symbols)          (symbols)
#define kTxAppendEof                   false
#define kTxWrapAtLimit                 false
#define RMT_ARRAY_LEN(field) (sizeof(((rmt_dev_t *)0)->field) / sizeof(((rmt_dev_t *)0)->field[0]))
static const uint32_t kChannelsPerInstance = RMT_ARRAY_LEN(chnconf0);
#else
#error "ws2812_rmt: this part's RMT register names are not described here yet"
#endif

LOG_MODULE_REGISTER(ws2812_rmt, CONFIG_LOG_DEFAULT_LEVEL);

/* A WS2812 bit is one RMT symbol and one channel owns 48 of them, so a strip is sent in
 * chunks of six bytes, each started after the previous reports done. The gap between
 * chunks is a few register writes - microseconds, far under the ~50 us of low that
 * latches a frame - so the strip still sees one frame. */
#define kSymbolsPerChannel SOC_RMT_MEM_WORDS_PER_CHANNEL
#define kBytesPerChunk     (kSymbolsPerChannel / 8)

/* WS2812 phases in 12.5 ns ticks of the 80 MHz source: t0H 400 ns -> 32, t1H 800 ns ->
 * 64, bit cell 1250 ns -> 100. */
#define kT0hTicks     32
#define kT1hTicks     64
#define kBitCellTicks 100

/* How many reads the wait for one chunk may take before the chunk is called a failure.
 *
 * A bound on damage, not an estimate of a slow frame: a chunk is at most 48 symbols of
 * 1250 ns = 60 us of hardware time, and one poll of the raw interrupt word is a single
 * register read, so a few thousand passes already sits an order of magnitude past
 * anything that is going to finish at all. The value this file carried before, 200000,
 * was chosen for being comfortably larger than anything plausible - which on a channel
 * that never raises tx_end turns every frame into forty-odd milliseconds of busy-wait
 * on the cooperative system workqueue, the same thread that runs the button debounce
 * and the BluFi receive work. Whatever else remains open about this part, one status
 * LED must not be able to hold a board up like that.
 */
#define kWaitPassCap 4000U

struct ws2812_rmt_config {
	struct gpio_dt_spec data;
	uint8_t channel;      /* RMT TX channel: 0 or 1 on these parts */
	uint16_t length;      /* pixels */
	uint16_t reset_delay; /* microseconds of low that latches a frame */
	uint8_t color_order[3]; /* DT color-mapping, i.e. the board states its wiring */
};

struct ws2812_rmt_data {
	rmt_dev_t *regs;
};

static rmt_dev_t *const rmt_base(void)
{
	/* Not the `RMT` extern: IDF startup files define that, and this port does not
	 * build them, so a link error there would look like a missing driver. */
	return (rmt_dev_t *)DR_REG_RMT_BASE;
}

static inline rmt_dev_t *regs_of(const struct device *dev)
{
	return ((struct ws2812_rmt_data *const)dev->data)->regs;
}

static inline uint32_t chan_of(const struct device *dev)
{
	return ((const struct ws2812_rmt_config *const)dev->config)->channel;
}

/* The hardware's own symbol layout: duration0(15) level0(1) duration1(15) level1(1). */
static inline uint32_t ws2812_symbol(bool one)
{
	const uint32_t high = one ? kT1hTicks : kT0hTicks;
	const uint32_t low = kBitCellTicks - high;

	return (high & 0x7FFFU) | (1U << 15) | ((low & 0x7FFFU) << 16);
}

/* One transaction, as measured rather than assumed.
 *
 * 'written' is read before the channel is started and is the only evidence that the
 * pushes landed (non-FIFO mode on the C3 accepted 24 writes, left the write pointer at
 * zero, and then transmitted empty symbols while reporting done).
 *
 * 'status' is a raw word, and for five releases this comment described the C3's
 * *rx_status* as its tx_status - read pointer high, write pointer low, the other way
 * round - and code was written against it. Reading the wrong half of a status word
 * does not fail to compile; it just answers a question nobody asked. The layout below
 * is transcribed from each part's soc/rmt_struct.h, field width by field width:
 *
 *   ESP32-C3 tx_status: mem_raddr_ex[8:0], state[11:9], waddr[20:12], then
 *                       mem_rd_err(21)/mem_empty(22)/mem_wr_err(23) and raddr[31:24].
 *   C5/C6 chnstatus:    mem_raddr_ex_chn[8:0], state_chn[11:9], apb_mem_waddr_chn[20:12],
 *                       then rd_err(21)/empty(22)/wr_err(23).
 * The two generations agree: the read pointer is the low 9 bits, the write pointer is
 * [20:12]. An earlier revision of this comment had state as 2 bits and waddr as [19:11]
 * - one bit short - so it decoded the write pointer with >>11 instead of >>12 and read
 * 0x00018000 as "48 written". The correct decode is waddr 24, raddr 0: 24 symbols pushed,
 * none read yet. That single off-by-one is what spawned the "the pointers count 16-bit
 * entries" theory and the two completion rules built on it (a green strip, then a false
 * all-clear), so the widths are stated here rather than paraphrased.
 *
 * What tx_lim, raddr and waddr count on the C3 is now MEASURED, not argued. The probe
 * (CONFIG_WS2812_RMT_PROBE) ran the ten frame-end candidates on the bench C3 for a
 * 24-symbol single-pixel frame, sampling the read pointer inside the wait and the raw
 * interrupt word on both sides of the stop:
 *
 *   lim=s-1 (23):  waddr 24, raddr 0 -> 25 during the wait, raw 0x101 (threshold +
 *                  TX_DONE) already set BEFORE the stop, done observed in 78 passes.
 *   lim=2s-1 / 2s: raw 0x001 - TX_DONE without the threshold, reached only by streaming
 *                  the stale entries that follow the data (the C6's wrong-colour trap).
 *
 * So the pointers count 32-bit symbols (waddr 24 for 24 symbols, not 48) and tx_lim =
 * symbols-1 is the right frame end. But TX_DONE is NOT universal: the bench/dev C3 raises
 * it (raw 0x101), while the deployed production C3 never does - its readings (read over
 * MQTT via ws2812_rmt_last_tx, no console) are waddr 24, raddr running to 28, int_raw
 * stuck at 0x100 (threshold only), wait running to its 4000-pass cap. The frame went out
 * (the read pointer consumed well past the 24 symbols) and only the announcement is
 * missing. So completion accepts EITHER the done bit OR the read pointer reaching the
 * symbol count - the pointer starts at 0 and advances only as symbols are consumed, so
 * unlike the write pointer (full before start, the source of the old green-strip false
 * complete) it cannot judge a frame done before it is. A dev-board pass did not prove the
 * production part; the production readings did.
 *
 * A trap that survives the corrected decode: stopping the channel resets the read
 * pointer, so a status word captured after rmt_ll_tx_stop() shows 0 whatever happened
 * during the frame. Progress has to be sampled inside the wait to mean anything - which
 * is why raddr_max below is collected there, and raw_before is taken before the stop. */
struct tx_result {
	uint32_t written;  /* status word after the pushes, before the channel is started */
	uint32_t status;   /* channel status word at the moment it finished or gave up */
	uint32_t raw;      /* raw interrupt word after the stop */
	uint32_t passes;   /* how many polls the wait took */
	uint32_t raddr_max; /* furthest mem_raddr_ex got during the wait (decoded, low 9 bits) */
	uint32_t raw_before; /* raw interrupt word one instruction before the stop */
	uint32_t limit;    /* the tx_lim this chunk asked to stop at */
	int rc;            /* 0 = the hardware reported TX_DONE */
};

/* Push one chunk, start it, wait for it.
 *
 * limit / append_eof / wrap are arguments because the two generations end a frame
 * differently and the ll layer does not say so - both describe tx_set_limit as "the
 * amount of symbols that can trigger the limitation interrupt", and only the C3 stops
 * at it. The values come from the probe below, run on hardware. */
static void ws2812_rmt_tx_chunk(const struct device *dev, const uint8_t *bytes, uint8_t count,
				uint32_t limit, bool append_eof, bool wrap, uint32_t pass_cap,
				struct tx_result *out)
{
	const struct ws2812_rmt_config *const cfg = dev->config;
	rmt_dev_t *const regs = regs_of(dev);
	const uint32_t ch = cfg->channel;

	rmt_ll_tx_stop(regs, ch);
	rmt_ll_tx_reset_pointer(regs, ch);
	rmt_ll_tx_enable_wrap(regs, ch, wrap);
	rmt_ll_tx_set_limit(regs, ch, limit);
	rmt_ll_clear_interrupt_status(regs,
				      RMT_LL_EVENT_TX_MASK(ch) |
				      RMT_LL_EVENT_TX_ERROR(ch));

	for (uint8_t i = 0; i < count; i++) {
		for (int bit = 7; bit >= 0; bit--) {
			RMT_CHANNEL_DATA(regs, ch) =
				ws2812_symbol(((bytes[i] >> bit) & 1U) != 0U);
		}
	}
	/* An all-zero symbol is four empty fields: no edge, no time on the line, so it is a
	 * stop marker for the transmitter rather than extra data. */
	if (append_eof) {
		RMT_CHANNEL_DATA(regs, ch) = 0U;
	}

	out->written = RMT_CHANNEL_STATUS(regs, ch);
	out->limit = limit;

	rmt_ll_tx_start(regs, ch);

	/* The RAW interrupt word, not int_st: the masked one is "raw AND interrupt
	 * enabled", and this driver takes no interrupt, so polling it can only return zero.
	 * Raw costs nothing and keeps the wait free of an ISR and its shared state.
	 *
	 * The wait is bounded because a chunk is 48 symbols = 60 us of hardware time; a
	 * channel that never finishes is mis-configured, and saying so beats hanging the
	 * caller - which for a status LED is the system workqueue. */
	const uint32_t done_bit = RMT_LL_EVENT_TX_DONE(ch);
	const uint32_t err_bit = RMT_LL_EVENT_TX_ERROR(ch);
	const uint32_t symbols = (uint32_t)count * 8U;

	out->rc = -EIO;
	out->raddr_max = 0;
	for (uint32_t passes = 0; passes <= pass_cap; passes++) {
		const uint32_t raw = rmt_ll_tx_get_interrupt_status_raw(regs, ch);
		/* mem_raddr_ex is the low 9 bits on both generations, and it starts at 0 and
		 * advances as the counter consumes symbols - so it is a completion signal too,
		 * not just a diagnostic. Some parts never raise TX_DONE: the production C3 runs
		 * the read pointer past the whole frame (raddr 28 for 24 symbols) yet int_raw
		 * stays 0x100 (threshold only), so waiting for the done bit alone times out on
		 * a frame that already went out. The read pointer reaching the symbol count is
		 * the evidence that survives both behaviours. */
		const uint32_t raddr = RMT_CHANNEL_STATUS(regs, ch) & 0x1FFU;

		if (raddr > out->raddr_max) {
			out->raddr_max = raddr;
		}
		if ((raw & done_bit) != 0U) {
			out->rc = 0;
			out->passes = passes;
			break;
		}
		if (raddr >= symbols) {
			/* The counter consumed the last symbol; let its bit cell finish shifting
			 * out before the stop, so the tail of the frame is not clipped. */
			k_busy_wait(3);
			out->rc = 0;
			out->passes = passes;
			break;
		}
		if ((raw & err_bit) != 0U) {
			/* An error means the symbols never reached the counter - the write
			 * path or the clock, not the strip - so it gets its own line. */
			out->passes = passes;
			break;
		}
		out->passes = passes;
	}

	/* The interrupt word and the status word on the live side of the stop: the stop
	 * resets the read pointer, and on this part TX_DONE is already set before it, so
	 * these are the readings that describe the frame rather than the reset. */
	out->raw_before = rmt_ll_tx_get_interrupt_status_raw(regs, ch);
	out->status = RMT_CHANNEL_STATUS(regs, ch);

	rmt_ll_tx_stop(regs, ch);
	/* Read before the status is cleared: reading after is how an early version printed
	 * raw=0 for a channel that had just reported done. */
	out->raw = rmt_ll_tx_get_interrupt_status_raw(regs, ch);
	rmt_ll_clear_interrupt_status(regs, RMT_LL_EVENT_TX_MASK(ch) | err_bit);
}

/* Bring-up aid for a part this driver has not run on: put each plausible frame-end
 * setting through one test frame and print what the hardware answered. Compiled only
 * with CONFIG_WS2812_RMT_PROBE=y; the answer it produced on the ESP32-C6 is in the
 * RMT_TX_LIMIT comment above. Worth keeping: three of these ten candidates report TX_DONE
 * and only one sends exactly the data, so the completion bit is a filter, not evidence. */
#ifdef CONFIG_WS2812_RMT_PROBE
static void ws2812_rmt_probe_frame_end(const struct device *dev)
{
	/* One pixel: 3 bytes, 24 symbols. */
	static const uint8_t kPattern[3] = { 0x80U, 0x00U, 0x00U };
	const uint32_t s = 24U;

	struct Try { const char *what; uint32_t limit; bool eof; bool wrap; };
	static const struct Try kTries[] = {
		{ "lim=s-1          ", s - 1U, false, false },
		{ "lim=s-1 wrap     ", s - 1U, false, true },
		{ "lim=s            ", s, false, false },
		{ "lim=2s-1         ", 2U * s - 1U, false, false },
		{ "lim=2s           ", 2U * s, false, false },
		{ "lim=2s wrap      ", 2U * s, false, true },
		{ "lim=s-1 eof      ", s - 1U, true, false },
		{ "lim=s eof        ", s, true, false },
		{ "lim=2s eof       ", 2U * s, true, false },
		{ "lim=2s eof wrap  ", 2U * s, true, true },
	};

	for (unsigned t = 0; t < ARRAY_SIZE(kTries); t++) {
		struct tx_result r;

		ws2812_rmt_tx_chunk(dev, kPattern, sizeof(kPattern), kTries[t].limit,
				   kTries[t].eof, kTries[t].wrap, 30000U, &r);
		LOG_INF("probe ch%lu [%s] rc=%d passes=%u waddr=%u raddrMax=%u "
			"rawBefore=0x%03x raw=0x%03x written=0x%08x status=0x%08x",
			(unsigned long)chan_of(dev), kTries[t].what, r.rc,
			(unsigned)r.passes, (unsigned)((r.written >> 12) & 0x1FFU),
			(unsigned)r.raddr_max, (unsigned)r.raw_before, (unsigned)r.raw,
			(unsigned)r.written, (unsigned)r.status);
	}
}
#endif /* CONFIG_WS2812_RMT_PROBE */

/* The last chunk's readings, kept for the diagnostic surface (ws2812_rmt_last_tx) so a
 * board with no console can still say whether the counter ran. Written by the transmit
 * path, read by the application over the network; a torn read of a 60 us transaction is
 * cheaper than a lock and reads as "the last frame", never "the frame on the wire now". */
static struct tx_result g_last_tx;
static bool g_last_tx_valid;

bool ws2812_rmt_last_tx(uint32_t out[WS2812_RMT_LAST_TX_WORDS], int *rc)
{
	if (!g_last_tx_valid || out == NULL || rc == NULL) {
		return false;
	}
	out[0] = g_last_tx.written;
	out[1] = g_last_tx.status;
	out[2] = g_last_tx.raw;
	out[3] = g_last_tx.passes;
	out[4] = g_last_tx.raddr_max;
	out[5] = g_last_tx.raw_before;
	out[6] = g_last_tx.limit;
	*rc = g_last_tx.rc;
	return true;
}

/* Push one chunk of the real frame and wait for it. */
static int ws2812_rmt_flush_chunk(const struct device *dev, const uint8_t *bytes, uint8_t count)
{
	const uint32_t symbols = (uint32_t)count * 8U;
	struct tx_result r;

	/* The limit is where this chunk stops, never the channel's capacity: left at the
	 * capacity the read pointer runs past the written symbols and waits for ones that
	 * never arrive, so the frame neither ends nor reports done. */
	ws2812_rmt_tx_chunk(dev, bytes, count, RMT_TX_LIMIT(symbols), kTxAppendEof, kTxWrapAtLimit,
			   kWaitPassCap, &r);
	g_last_tx = r;
	g_last_tx_valid = true;

	/* Once per boot: the three readings that say what happened on the hardware, since
	 * a completion bit alone has proved nothing (see struct tx_result for the field
	 * layout of the words printed here). */
	static bool first_reported;

	if (!first_reported) {
		first_reported = true;
		LOG_INF("ch%lu %u symbols: waddr=%u raddrMax=%u rawBefore=0x%03x "
			"written=0x%08x status=0x%08x raw=0x%08x sys_conf=0x%08x",
			(unsigned long)chan_of(dev), (unsigned)symbols,
			(unsigned)((r.written >> 12) & 0x1FFU), (unsigned)r.raddr_max,
			(unsigned)r.raw_before, (unsigned)r.written, (unsigned)r.status,
			(unsigned)r.raw, (unsigned)regs_of(dev)->sys_conf.val);
	}

	if (r.rc == 0) {
		static bool done_reported;

		if (!done_reported) {
			done_reported = true;
			LOG_INF("ch%lu done: written=0x%08x status=0x%08x raw=0x%08x",
				(unsigned long)chan_of(dev), (unsigned)r.written,
				(unsigned)r.status, (unsigned)r.raw);
		}
		return 0;
	}
	if ((r.raw & RMT_LL_EVENT_TX_ERROR(chan_of(dev))) != 0U) {
		LOG_ERR("rmt channel %lu reported an error", (unsigned long)chan_of(dev));
	} else {
		/* raddrMax is the discriminator: 0 means the counter never ran (clock or
		 * symbol path), >= symbols means the frame went out and only the done bit
		 * was missed. Both used to read as the same "never done". */
		LOG_ERR("rmt ch%lu never done: waddr=%u raddrMax=%u rawBefore=0x%03x "
			"written=0x%08x status=0x%08x raw=0x%08x sys_conf=0x%08x",
			(unsigned long)chan_of(dev), (unsigned)((r.written >> 12) & 0x1FFU),
			(unsigned)r.raddr_max, (unsigned)r.raw_before,
			(unsigned)r.written, (unsigned)r.status, (unsigned)r.raw,
			(unsigned)regs_of(dev)->sys_conf.val);
	}
	return r.rc;
}

/* Which byte of a pixel goes on the wire at each position, by LED_COLOR_ID name rather
 * than by casting struct led_rgb - whose layout is nobody's contract. */
static uint8_t color_component(const struct led_rgb *pixel, uint8_t id)
{
	switch (id) {
	case LED_COLOR_ID_RED:
		return pixel->r;
	case LED_COLOR_ID_GREEN:
		return pixel->g;
	default:
		return pixel->b;
	}
}

static int ws2812_rmt_update_rgb(const struct device *dev,
				 struct led_rgb *pixels, size_t count)
{
	const struct ws2812_rmt_config *const cfg = dev->config;
	uint8_t chunk[kBytesPerChunk];
	size_t filled = 0;

	for (size_t i = 0; i < count; i++) {
		const uint8_t wire[3] = {
			color_component(&pixels[i], cfg->color_order[0]),
			color_component(&pixels[i], cfg->color_order[1]),
			color_component(&pixels[i], cfg->color_order[2]),
		};

		for (int j = 0; j < 3; j++) {
			chunk[filled++] = wire[j];

			/* Flush on a full chunk, and on the last byte of the frame so a
			 * strip whose byte count is not a multiple of the chunk size still
			 * gets its tail out. */
			const bool last = (i + 1 == count) && (j == 2);

			if (filled == kBytesPerChunk || last) {
				const int rc = ws2812_rmt_flush_chunk(
					dev, chunk, (uint8_t)filled);

				if (rc != 0) {
					return rc;
				}
				filled = 0;
			}
		}
	}

	/* Latch: the line must stay low for reset-delay before the next frame. */
	k_busy_wait(cfg->reset_delay);
	return 0;
}

static size_t ws2812_rmt_length(const struct device *dev)
{
	const struct ws2812_rmt_config *const cfg = dev->config;

	return cfg->length;
}

static const struct led_strip_driver_api ws2812_rmt_api = {
	.update_rgb = ws2812_rmt_update_rgb,
	.length = ws2812_rmt_length,
};

static int ws2812_rmt_init(const struct device *dev)
{
	const struct ws2812_rmt_config *const cfg = dev->config;
	struct ws2812_rmt_data *const data = dev->data;
	const uint32_t ch = cfg->channel;

	data->regs = rmt_base();

	if (cfg->channel >= kChannelsPerInstance) {
		LOG_ERR("rmt channel %u does not exist on this part (%u per instance)",
			cfg->channel, (unsigned)kChannelsPerInstance);
		return -EINVAL;
	}
	if (!gpio_is_ready_dt(&cfg->data)) {
		LOG_ERR("gpio controller for the data line is not ready");
		return -ENODEV;
	}
	/* Reject a color-mapping that repeats an id or is not RGB: it would send the same
	 * component twice and look like a broken strip rather than a bad devicetree. */
	for (int i = 0; i < 3; i++) {
		if (cfg->color_order[i] > LED_COLOR_ID_BLUE) {
			LOG_ERR("color-mapping[%d] is not an RGB id", i);
			return -EINVAL;
		}
		for (int j = i + 1; j < 3; j++) {
			if (cfg->color_order[i] == cfg->color_order[j]) {
				LOG_ERR("color-mapping repeats id %u", cfg->color_order[i]);
				return -EINVAL;
			}
		}
	}
	/* Configuring the pin as an output is what selects the GPIO matrix for the pad and
	 * enables its driver; the matrix source is re-pointed at RMT below. */
	int rc = gpio_pin_configure_dt(&cfg->data, GPIO_OUTPUT_INACTIVE);

	if (rc != 0) {
		return rc;
	}

	/* Three switches and a power bit, all independent, and the registers only answer
	 * once every one of them is on: bus clock, out of reset, symbol RAM power. */
	rmt_ll_enable_bus_clock(0, true);
	rmt_ll_reset_register(0); /* asserts then de-asserts, so it is init-safe */
	rmt_ll_mem_force_power_on(data->regs);

	rmt_ll_set_group_clock_src(data->regs, ch, RMT_TX_CLK_SRC, 1, 1, 0);
	/* The counter clock is a fourth switch of its own: sys_conf.sclk_active on the C3
	 * (reset value 1, which is why this driver worked there without touching it) and
	 * PCR.rmt_sclk_conf.rmt_sclk_en on the C5/C6 (reset value 0). Left cleared on the
	 * C6, the symbols reached the channel RAM - chnstatus write address 24 - while the
	 * read address stayed at 0: no waveform, no done bit, and every register this
	 * driver writes read back exactly as written. */
	rmt_ll_enable_group_clock(data->regs, true);
	/* FIFO mode: each DATA write appends one symbol and advances the write address.
	 * In non-FIFO mode 24 writes left the pointer at zero, and the channel then
	 * transmitted empty symbols, reached its limit and still reported done. */
	rmt_ll_enable_mem_access_nonfifo(data->regs, false);
	rmt_ll_tx_reset_channels_clock_div(data->regs, BIT(ch));
	rmt_ll_tx_set_channel_clock_div(data->regs, ch, 1);
	rmt_ll_tx_reset_pointer(data->regs, ch);

	/* Carrier modulation is asserted in the channel's reset value (C3 read back
	 * tx_conf=0x00710140), and a modulated output with the carrier registers at reset
	 * is a flat line on the pin - the counter runs, the limit is reached, done is
	 * reported, and only tx_conf in the log below says anything happened. IDF's rmt
	 * driver disables the carrier in its channel reset for the same reason. */
	rmt_ll_tx_enable_carrier_modulation(data->regs, ch, false);
	rmt_ll_tx_enable_carrier_always_on(data->regs, ch, false);
	rmt_ll_tx_set_carrier_level(data->regs, ch, 0);

	/* Rest low between frames, so the tail of one is not read as the head of the next. */
	rmt_ll_tx_fix_idle_level(data->regs, ch, 0, true);
	rmt_ll_tx_set_mem_blocks(data->regs, ch, 1);
	/* The limit belongs to a chunk, not to the channel: it is set per flush. */
	rmt_ll_tx_enable_loop(data->regs, ch, false);

	/* The GPIO-matrix signal index comes from the SoC's own table: "channel N" is a
	 * different PMUX entry on every part, which is exactly the number not to copy out
	 * of a TRM into a driver. */
	esp_rom_gpio_connect_out_signal(
		cfg->data.pin, soc_rmt_signals[0].channels[ch].tx_sig, false, false);

	LOG_INF("ws2812 on rmt ch%lu gpio%u, %u px, %u ticks/bit, tx_conf=0x%08x "
		"sys_conf=0x%08x", (unsigned long)ch, cfg->data.pin, cfg->length,
		(unsigned)kBitCellTicks, (unsigned)RMT_CHANNEL_CONF(data->regs, ch),
		(unsigned)data->regs->sys_conf.val);
#ifdef CONFIG_WS2812_RMT_PROBE
	ws2812_rmt_probe_frame_end(dev);
#endif
	return 0;
}

/* DEVICE_DT_INST_DEFINE, not LED_DEVICE_DT_INST_DEFINE: this Zephyr's led_strip headers
 * do not define the latter. */
#define WS2812_RMT_DEFINE(idx)                                                   \
	static const struct ws2812_rmt_config ws2812_rmt_cfg_##idx = {           \
		.data = GPIO_DT_SPEC_INST_GET(idx, data_gpios),                  \
		.channel = DT_INST_PROP(idx, espressif_rmt_channel),             \
		.length = DT_INST_PROP(idx, chain_length),                       \
		.reset_delay = DT_INST_PROP(idx, reset_delay),                   \
		.color_order = { DT_INST_PROP_BY_IDX(idx, color_mapping, 0),     \
				 DT_INST_PROP_BY_IDX(idx, color_mapping, 1),     \
				 DT_INST_PROP_BY_IDX(idx, color_mapping, 2) },   \
	};                                                                       \
	static struct ws2812_rmt_data ws2812_rmt_data_##idx;                     \
	DEVICE_DT_INST_DEFINE(idx, ws2812_rmt_init, NULL,                        \
			      &ws2812_rmt_data_##idx, &ws2812_rmt_cfg_##idx,     \
			      POST_KERNEL, CONFIG_LED_STRIP_INIT_PRIORITY,       \
			      &ws2812_rmt_api);

DT_INST_FOREACH_STATUS_OKAY(WS2812_RMT_DEFINE)
