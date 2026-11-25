# WS2812-over-RMT driver for ESP32 parts (Zephyr module).
#
# Named library rather than the default, so a link error names this module instead of
# "app". Built only when a board describes such a node (see Kconfig).
zephyr_library_named(ws2812_rmt)
# include/ carries the driver's diagnostic surface (ws2812_rmt.h), reached weakly by the
# application; reachable even in a build that selects no such node, where the weak
# reference resolves to nothing.
zephyr_include_directories(include)
zephyr_library_sources_ifdef(CONFIG_LED_STRIP_ESP_RMT ws2812_rmt.c)

# The GPIO-matrix signal index per RMT channel comes from the SoC's own table rather
# than from arithmetic on RMT_SIG_OUT0_IDX, so esp_hal_rmt/<soc>/rmt_periph.c has to be
# compiled in. It needs only hal/rmt_periph.h and soc/gpio_sig_map.h.
if(CONFIG_LED_STRIP_ESP_RMT)
  set(ESP_HAL_RMT ${ZEPHYR_BASE}/../modules/hal/espressif/components/esp_hal_rmt)
  zephyr_library_sources(${ESP_HAL_RMT}/${CONFIG_SOC}/rmt_periph.c)
endif()
