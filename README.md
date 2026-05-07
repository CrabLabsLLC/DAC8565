# DAC8565

Platform-agnostic C driver for the [Texas Instruments DAC8565](docs/dac8565_datasheet.pdf) — a quad-channel, 16-bit, voltage-output DAC with 2.5 V on-chip reference, double-buffered registers, and a 3-wire SPI interface (mode 1, up to 50 MHz).

## At a glance

| | |
|---|---|
| Resolution | 16 bits, monotonic by design |
| Channels | 4 (A, B, C, D) |
| Reference | 2.5 V internal (default) OR external on `V_REFH` |
| Output | Rail-to-rail buffered, 0 V to V_REF |
| Coding | Straight binary (RSTSEL=0) or two's complement (RSTSEL=1) |
| Bus | 3-wire SPI, mode 1 (CPOL=0, CPHA=1), up to 50 MHz |
| Frame | 24 bits: `[0][0][LD1][LD0][0][DSEL1][DSEL0][PD0][D15..D0]` |
| Power-down | Per-channel: Normal · 1 kΩ-to-GND · 100 kΩ-to-GND · Hi-Z |
| Update modes | Single-channel, simultaneous (sync), broadcast, LDAC-buffered |
| Reset | Power-on, software (write 0), or hardware (~RST pin) |
| Package | TSSOP-16 |

## Repository layout

```
DAC8565/
├── include/
│   └── dac8565.h               Public API + types + frame layout
├── src/
│   └── dac8565.c               Driver implementation
├── example/
│   └── main.c                  ESP-IDF four-channel sync ramp
├── test/
│   └── system/
│       └── main.c              Hardware-in-the-loop comprehensive system test
├── docs/
│   ├── dac8565_datasheet.pdf
│   └── dac8565_notes.md        Datasheet extract: command tables, timing, RSTSEL behavior
├── CMakeLists.txt              Tri-mode (ESP-IDF / Zephyr / plain CMake)
├── Justfile                    format / format_check / build
├── .clang-format               Tabs, Allman, no col limit
├── LICENSE
└── README.md
```

## Public API surface

Everything is namespaced under `dac8565*`. Returns a `DAC8565Error`. Caller allocates the `DAC8565Device` handle and provides a `DAC8565HAL` of function pointers.

### Lifecycle

```c
DAC8565Error dac8565Init(DAC8565Device* dev, const DAC8565Config* config, const DAC8565HAL* hal);
DAC8565Error dac8565Reset(DAC8565Device* dev);            /* software: write 0 to all 4 channels */
DAC8565Error dac8565ResetHardware(DAC8565Device* dev);    /* pulses ~RST (requires HAL.resetSet) */
DAC8565Error dac8565Deinit(DAC8565Device* dev);
```

### Reference configuration

```c
DAC8565Error dac8565SetReferenceMode(DAC8565Device* dev, DAC8565ReferenceMode mode);
DAC8565Error dac8565GetReferenceMode(const DAC8565Device* dev, DAC8565ReferenceMode* mode);
DAC8565Error dac8565SetExternalReferenceVoltageUv(DAC8565Device* dev, uint32_t voltage_uv);
DAC8565Error dac8565GetReferenceVoltageUv(const DAC8565Device* dev, uint32_t* voltage_uv);
```

### Per-channel data

```c
DAC8565Error dac8565WriteCode(DAC8565Device* dev, DAC8565Channel channel, uint16_t code);
DAC8565Error dac8565WriteVoltageUv(DAC8565Device* dev, DAC8565Channel channel, uint32_t voltage_uv);
```

### Multi-channel synchronous

```c
DAC8565Error dac8565WriteAllCodes(DAC8565Device* dev, const uint16_t codes[4]);
DAC8565Error dac8565WriteAllVoltagesUv(DAC8565Device* dev, const uint32_t voltages_uv[4]);
DAC8565Error dac8565WriteBroadcastCode(DAC8565Device* dev, uint16_t code);
DAC8565Error dac8565WriteBroadcastVoltageUv(DAC8565Device* dev, uint32_t voltage_uv);
```

### Buffered (LDAC) staging

```c
DAC8565Error dac8565StageCode(DAC8565Device* dev, DAC8565Channel channel, uint16_t code);
DAC8565Error dac8565StageVoltageUv(DAC8565Device* dev, DAC8565Channel channel, uint32_t voltage_uv);
DAC8565Error dac8565CommitBuffers(DAC8565Device* dev);
```

### Power-down

```c
DAC8565Error dac8565SetChannelPowerMode(DAC8565Device* dev, DAC8565Channel channel, DAC8565PowerMode mode);
DAC8565Error dac8565SetAllPowerModes(DAC8565Device* dev, const DAC8565PowerMode modes[4]);
DAC8565Error dac8565GetChannelPowerMode(const DAC8565Device* dev, DAC8565Channel channel, DAC8565PowerMode* mode);
```

### State accessors and pin control

```c
DAC8565Error dac8565GetLastCode(const DAC8565Device* dev, DAC8565Channel channel, uint16_t* code);
DAC8565Error dac8565GetLastVoltageUv(const DAC8565Device* dev, DAC8565Channel channel, uint32_t* voltage_uv);
DAC8565Error dac8565GetEncoding(const DAC8565Device* dev, DAC8565Encoding* encoding);
DAC8565Error dac8565SetEnable(DAC8565Device* dev, bool level);
DAC8565Error dac8565PulseLDAC(DAC8565Device* dev);
```

The chip is **write-only** — all `Get*` accessors return values cached in the device handle.

## Wiring the HAL on your platform

```c
static int my_spi_write(const void* const data, const uint8_t length) { /* … */ }
static void my_ldac_set(bool level) { /* drive LDAC GPIO */ }

const DAC8565HAL hal = {
    .spiWrite = my_spi_write,
    .ldacSet  = my_ldac_set,    /* needed for Stage*/CommitBuffers */
    .resetSet = my_reset_set,   /* needed for ResetHardware */
    .delayUs  = my_delay_us,    /* recommended for clean PD-exit / LDAC pulses */
};

const DAC8565Config config = {
    .encoding                      = DAC8565_ENCODING_BINARY,    /* match RSTSEL pin strap */
    .initial_reference_mode        = DAC8565_REF_MODE_ALWAYS_ON, /* internal 2.5 V always on */
    .external_reference_voltage_uv = 0U,                         /* unused unless DISABLED */
};

DAC8565Device dev;
dac8565Init(&dev, &config, &hal);
dac8565WriteAllVoltagesUv(&dev, (uint32_t[4]){0, 625000, 1250000, 1875000});
```

The `RSTSEL` pin is hardware-strapped — tell the driver which level via `config.encoding` so the volt↔code conversion is correct. On a board with `RSTSEL` tied high (two's-complement input coding), use `DAC8565_ENCODING_TWOS_COMPLEMENT`.

## Building

### As an ESP-IDF component

Drop the repo into `components/DAC8565` and ESP-IDF builds it automatically.

### As a Zephyr module

```cmake
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/components/DAC8565)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
```

### As a plain CMake library (STM32, desktop tests)

```cmake
add_subdirectory(components/DAC8565)
target_link_libraries(my_app PRIVATE dac8565)
```

Or build standalone for verification:

```bash
just build
just format_check
```

## System testing

[test/system/main.c](test/system/main.c) is a hardware-in-the-loop test covering:

- NULL-pointer + invalid-arg rejection at every public entry point
- Use-before-init / use-after-deinit guards
- Init / deinit / re-init lifecycle and `COMM_FAIL` propagation
- Voltage ↔ code round-trip across the input domain (≤ 1 LSB)
- Per-channel writes and read-back-from-cache
- Multi-channel synchronous write (Example 1 sequence)
- Broadcast write
- Buffered staging + LDAC commit
- Per-channel power-down cycling
- `SetAllPowerModes` (Example 3 sequence)
- Reference-mode swap
- `NOT_SUPPORTED` returned when optional HAL pins are absent
- 16-bit code sweep (visible on V_OUT_A with a scope)

## Conformance

* C11 source; C99 compatible
* Style: [Crab Labs C Style Guide](../../style-guide.md) — tabs (4-wide), Allman, `camelCase` functions, `PascalCase` types, `ALL_CAPS` macros/enums, `snake_case` variables, units in identifiers
* Architecture: [Crab Labs Driver Development Guide](../../driver-guide.md) — HAL via function pointers, caller-allocated device handle, layered internal helpers, no platform headers, no logging, no asserts, no dynamic memory

## License

[BSD 3-Clause](LICENSE).
