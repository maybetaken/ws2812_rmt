# ws2812_rmt — WS2812 over the ESP32 RMT peripheral, for Zephyr

Drives WS2812-class addressable LEDs from an RMT TX channel, behind Zephyr's
standard `led_strip` API. Each data bit becomes one RMT symbol whose high time is
the WS2812 phase (t0H 400 ns, t1H 800 ns of a 1250 ns bit), counted by the
peripheral rather than by the CPU.

Zephyr ships no `led_strip` driver for Espressif RMT, and CPU bit-banging cannot
hold these phase widths: on an ESP32-C3 a single GPIO store costs 312 ns and a
system-clock read 687 ns, against a 400 ns phase.

```
ws2812_rmt.c                     the driver: symbols, channel setup, transmit, wait
include/ws2812_rmt/ws2812_rmt.h  diagnostic surface (not part of led_strip)
dts/bindings/led_strip/          the compatible, so a board picks it by DT alone
```

## Using it

Describe a node; no Kconfig symbol has to be named by the board.

```dts
led_strip0: led-strip {
        compatible = "espressif,esp32-ws2812-rmt";
        data-gpios = <&gpio0 8 GPIO_ACTIVE_HIGH>;
        chain-length = <1>;
};
```

`LED_STRIP_ESP_RMT` defaults to y once such a node is enabled, and selects
`LED_STRIP`. The data pin is routed to the channel's RMT signal through the GPIO
matrix, so no pinmux table entry is needed.

`espressif,rmt-channel` picks the TX channel (default 0). Channels are not
allocated: two strip nodes must name different ones, and the channel must exist on
the part — the C3 declares 4 data registers with 2 TX config sets, the C5/C6 2 — so
the driver checks it at init rather than the binding.

## Bringing it up on a new part

`CONFIG_WS2812_RMT_PROBE=y` puts each candidate frame-end setting (tx_lim value,
EOF symbol, memory wrap) through one test frame at init and logs what the hardware
answered. For a part this driver has not run on, where "the two register
generations look the same in the ll layer" is not a safe assumption. Costs a few
tens of milliseconds at boot and about ten log lines. Off by default.

## Why there is a diagnostic surface

"The frame failed" is two different claims — the waveform never left the channel,
or it left and completion was never observed — and a return code cannot tell them
apart. A board with no console can only answer that if the readings reach the
network, so `ws2812_rmt_last_tx()` exposes the last chunk's raw readings: channel
status before and after the pushes, the raw interrupt word, the poll count, the
furthest `mem_raddr_ex` reached, and the `tx_lim` the chunk asked to stop at.

`raddr_max` is the one that settles it: 0 means the counter never ran, at or above
the symbol count means the frame was consumed even if `done` was never announced.
The application reaches this symbol weakly, so a build that selects no such node
still links.

## Dependencies

Zephyr, and the `hal_espressif` west project: the GPIO-matrix signal index per RMT
channel comes from the SoC's own table (`esp_hal_rmt/<soc>/rmt_periph.c`) rather
than from arithmetic on `RMT_SIG_OUT0_IDX`. Nothing here names a board, a product
or an application.

## License

Apache-2.0. See `LICENSE`.