/**
 * @file dac8565.h
 * @brief Public API, types, and frame layout for the TI DAC8565 quad-channel 16-bit DAC.
 *
 * The DAC8565 is a 4-channel, 16-bit, voltage-output DAC with an on-chip
 * 2.5 V reference (enabled by default), a 3-wire SPI interface (mode 1, up
 * to 50 MHz), and double-buffered registers per channel. Every transaction
 * is a 24-bit frame (MSB first) following Table 5 of SBAS411C:
 *
 *   ┌───┬───┬─────┬─────┬───┬───────┬───────┬─────┬───────────────────┐
 *   │ 0 │ 0 │ LD1 │ LD0 │ 0 │ DSEL1 │ DSEL0 │ PD0 │ D15..D0 (16 data) │
 *   └───┴───┴─────┴─────┴───┴───────┴───────┴─────┴───────────────────┘
 *
 * The chip is **write-only**. `Get*` accessors return state cached in the
 * device handle. A separate `LDAC` GPIO pulse can be used to commit
 * software-staged buffer writes (see `dac8565Stage*` and ::dac8565CommitBuffers).
 *
 * **Hardware-strapped behavior the driver must know about:**
 *  - `RSTSEL` selects input coding: low → straight binary; high → two's complement.
 *    Tell the driver via ::DAC8565Config::encoding so voltage ↔ code conversion is correct.
 *  - `RST` (active-low) performs an asynchronous chip-wide reset to either
 *    zero-scale (RSTSEL=0) or midscale (RSTSEL=1).
 *  - `LDAC` (rising-edge triggered) loads all DAC registers from their buffers.
 *  - `ENABLE` (active-low) gates the SPI shift register.
 *
 * **Thread safety:** the driver is *not* thread-safe. Concurrent calls
 * against the same `DAC8565Device` will corrupt cached state. Callers must
 * serialize access.
 *
 * API layering (top = callers, bottom = hardware):
 *   1. Initialization / Lifecycle
 *   2. Reference configuration
 *   3. Per-channel data output
 *   4. Multi-channel synchronous output
 *   5. Buffered (LDAC) staging
 *   6. Power-down configuration
 *   7. Cached state accessors
 *   8. Hardware pin control
 *
 * @author Orion Serup <orion@crablabs.io>
 *
 * @reviewer TBD <reviewer@crablabs.io>
 */

#pragma once
#ifndef DAC8565_H
#define DAC8565_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Frame layout ─────────────────────────────────────────────────────────── */

/** @brief Width of a DAC8565 SPI command frame in bytes (24 bits, MSB first). */
#define DAC8565_FRAME_BYTES 3U

/** @brief Number of analog channels (A, B, C, D). */
#define DAC8565_CHANNEL_COUNT 4U

/** @brief Maximum 16-bit DAC code (full-scale). */
#define DAC8565_CODE_MAX 0xFFFFU

/** @brief Full-scale divisor: V_OUT = V_REF × code / 65536 (binary coding). */
#define DAC8565_FULL_SCALE 65536UL

/** @brief Internal reference voltage in microvolts (2.5 V). */
#define DAC8565_INTERNAL_REFERENCE_UV 2500000UL

/* ── Error codes ──────────────────────────────────────────────────────────── */

typedef enum
{
	DAC8565_ERROR_OK            = 0,  ///< Success
	DAC8565_ERROR_INVALID_PARAM = -1, ///< NULL pointer or out-of-range argument
	DAC8565_ERROR_COMM_FAIL     = -2, ///< SPI transaction reported failure
	DAC8565_ERROR_NOT_INIT      = -3, ///< Driver not initialized
	DAC8565_ERROR_NOT_SUPPORTED = -4, ///< Operation requires a HAL pin the caller did not provide
} DAC8565Error;

/* ── Channels ─────────────────────────────────────────────────────────────── */

/**
 * @brief DAC channel selector. Values are the literal DSEL1:DSEL0 bit pattern.
 */
typedef enum
{
	DAC8565_CHANNEL_A = 0x0U, ///< DSEL = 00
	DAC8565_CHANNEL_B = 0x1U, ///< DSEL = 01
	DAC8565_CHANNEL_C = 0x2U, ///< DSEL = 10
	DAC8565_CHANNEL_D = 0x3U, ///< DSEL = 11
} DAC8565Channel;

/* ── Power-down modes (datasheet Table 7) ─────────────────────────────────── */

/**
 * @brief Per-channel power-down mode.
 *
 * Encoded as the literal PD1:PD2 bit pattern (with PD0=1 implicit when
 * non-NORMAL). The NORMAL value (0) is sent with PD0=0 and ignored bits.
 */
typedef enum
{
	DAC8565_POWER_MODE_NORMAL  = 0x0U, ///< PD0=0 — output drives VOUT
	DAC8565_POWER_MODE_PD_1K   = 0x1U, ///< PD0=1, PD1=0, PD2=1 — output via 1 kΩ to GND
	DAC8565_POWER_MODE_PD_100K = 0x2U, ///< PD0=1, PD1=1, PD2=0 — output via 100 kΩ to GND
	DAC8565_POWER_MODE_PD_HIZ  = 0x3U, ///< PD0=1, PD1=1, PD2=1 — output Hi-Z
} DAC8565PowerMode;

/* ── Reference modes (datasheet §"Enable/Disable Internal Reference") ────── */

typedef enum
{
	DAC8565_REF_MODE_DEFAULT   = 0, ///< Internal 2.5 V ref auto-tracks DAC power state
	DAC8565_REF_MODE_ALWAYS_ON = 1, ///< Internal 2.5 V ref always powered (regardless of DAC PD)
	DAC8565_REF_MODE_DISABLED  = 2, ///< Internal ref disabled; external V_REF on VREFH pin
} DAC8565ReferenceMode;

/* ── Input coding (selected by hardware RSTSEL pin) ───────────────────────── */

typedef enum
{
	DAC8565_ENCODING_BINARY          = 0, ///< RSTSEL=0: code 0 = 0 V, 0xFFFF ≈ V_REF
	DAC8565_ENCODING_TWOS_COMPLEMENT = 1, ///< RSTSEL=1: code 0x8000 = 0 V, 0x7FFF ≈ V_REF
} DAC8565Encoding;

/* ── HAL ──────────────────────────────────────────────────────────────────── */

/**
 * @brief Platform-provided primitives.
 *
 * `spiWrite` is mandatory and owns the full ~SYNC chip-select envelope for
 * each frame. The four GPIO callbacks (`ldacSet`, `resetSet`, `enableSet`,
 * `delayUs`) are optional; functions that need them return
 * ::DAC8565_ERROR_NOT_SUPPORTED when not provided.
 */
typedef struct
{
	/**
	 * @brief Send `length` bytes MSB-first over SPI in mode 1 (CPOL=0, CPHA=1).
	 *
	 * Caller asserts ~SYNC low, clocks the bytes, raises ~SYNC, returns 0
	 * on success. Up to 50 MHz SCLK at V_DD ≥ 2.7 V.
	 */
	int (*spiWrite)(const void* const data, const uint8_t length);

	/** @brief Drive the LDAC pin. Required only for `Stage*` / `CommitBuffers`. */
	void (*ldacSet)(bool level);

	/** @brief Drive the ~RST pin. Required only for ::dac8565ResetHardware. */
	void (*resetSet)(bool level);

	/** @brief Drive the ~ENABLE pin. Required only for ::dac8565SetEnable. */
	void (*enableSet)(bool level);

	/** @brief Spin or sleep at least `delay_us` microseconds. Optional. */
	void (*delayUs)(uint32_t delay_us);
} DAC8565HAL;

/* ── Configuration ────────────────────────────────────────────────────────── */

/**
 * @brief Init-time configuration.
 *
 * The driver needs to know:
 *   - which input coding the hardware RSTSEL pin selects (so volt↔code conversion is correct);
 *   - which reference mode to commit at init;
 *   - the external V_REF voltage if the internal reference is disabled.
 */
typedef struct
{
	DAC8565Encoding      encoding;                      ///< Hardware RSTSEL setting
	DAC8565ReferenceMode initial_reference_mode;        ///< Reference mode applied during init
	uint32_t             external_reference_voltage_uv; ///< Used when initial_reference_mode == DISABLED; ignored otherwise
} DAC8565Config;

/* ── Device handle ────────────────────────────────────────────────────────── */

typedef struct
{
	DAC8565HAL           hal;
	DAC8565Encoding      encoding;
	DAC8565ReferenceMode reference_mode;
	uint32_t             reference_voltage_uv;                ///< 2_500_000 if internal ref active, else external
	uint32_t             external_reference_voltage_uv;       ///< Always-cached external V_REF (used when ref switches to DISABLED)
	uint16_t             last_codes[DAC8565_CHANNEL_COUNT];   ///< Raw chip-register bits per channel (cached)
	uint16_t             staged_codes[DAC8565_CHANNEL_COUNT]; ///< Codes pending an LDAC commit (raw chip bits)
	uint8_t              staged_valid;                        ///< Per-channel bitmap: bit n set if staged_codes[n] is pending
	DAC8565PowerMode     power_modes[DAC8565_CHANNEL_COUNT];
	bool                 is_initialized;
} DAC8565Device;

/* ── 1. Initialization / Lifecycle ────────────────────────────────────────── */

/**
 * @brief Initialize the driver, commit the requested reference mode, push
 *        a known-good `0V` to all four channels.
 *
 * @param[in,out] dev     Device handle to initialize. Must not be NULL.
 * @param[in]     config  Configuration. Must not be NULL.
 * @param[in]     hal     HAL function pointers. Must not be NULL; `spiWrite` must be non-NULL.
 * @return ::DAC8565_ERROR_OK on success.
 */
DAC8565Error dac8565Init(DAC8565Device* const       dev,
                         const DAC8565Config* const config,
                         const DAC8565HAL* const    hal);

/**
 * @brief Drive every channel to 0 V in NORMAL power mode (software reset).
 *
 * Does not toggle the hardware ~RST pin; for that, see ::dac8565ResetHardware.
 */
DAC8565Error dac8565Reset(DAC8565Device* const dev);

/**
 * @brief Pulse the hardware ~RST pin to perform an asynchronous chip reset.
 *
 * Outputs go to zero-scale (if RSTSEL=0) or midscale (if RSTSEL=1).
 * Requires `hal.resetSet`; otherwise returns ::DAC8565_ERROR_NOT_SUPPORTED.
 */
DAC8565Error dac8565ResetHardware(DAC8565Device* const dev);

/**
 * @brief Tear down the driver. Does not touch the chip.
 */
DAC8565Error dac8565Deinit(DAC8565Device* const dev);

/* ── 2. Reference configuration ───────────────────────────────────────────── */

/**
 * @brief Commit a new reference mode (default-tracking / always-on / disabled).
 *
 * When switching to ::DAC8565_REF_MODE_DISABLED, the cached
 * `reference_voltage_uv` is replaced with `external_reference_voltage_uv`
 * supplied at init. To change the external reference at runtime, call
 * ::dac8565SetExternalReferenceVoltageUv.
 */
DAC8565Error dac8565SetReferenceMode(DAC8565Device* const dev, const DAC8565ReferenceMode mode);

DAC8565Error dac8565GetReferenceMode(const DAC8565Device* const dev, DAC8565ReferenceMode* const mode);

/**
 * @brief Update the cached external V_REF used in volt↔code conversion when
 *        the internal reference is disabled.
 */
DAC8565Error dac8565SetExternalReferenceVoltageUv(DAC8565Device* const dev, const uint32_t voltage_uv);

DAC8565Error dac8565GetReferenceVoltageUv(const DAC8565Device* const dev, uint32_t* const voltage_uv);

/* ── 3. Per-channel data output ───────────────────────────────────────────── */

/**
 * @brief Single-channel UPDATE (LD=01): write code to the channel's buffer
 *        AND DAC register in one frame; output settles immediately.
 *
 * `code` is sent verbatim — caller is responsible for matching the chip's
 * input coding (see ::DAC8565Encoding). For voltage-based writes that
 * automatically apply the correct encoding, use ::dac8565WriteVoltageUv.
 */
DAC8565Error dac8565WriteCode(DAC8565Device* const dev, const DAC8565Channel channel, const uint16_t code);

/**
 * @brief Write a voltage to a channel, encoding per the configured RSTSEL setting.
 */
DAC8565Error dac8565WriteVoltageUv(DAC8565Device* const dev, const DAC8565Channel channel, const uint32_t voltage_uv);

/* ── 4. Multi-channel synchronous output ──────────────────────────────────── */

/**
 * @brief Write all four channels in one transaction; outputs all settle on
 *        the same SCLK edge (datasheet "Operating Examples — Example 1").
 *
 * Issues 4 frames: A/B/C single-channel STORE (LD=00), then D store-and-load-all (LD=10).
 *
 * @param[in,out] dev    Initialized device handle.
 * @param[in]     codes  Array of 4 raw 16-bit codes (caller-encoded). Order: A, B, C, D.
 */
DAC8565Error dac8565WriteAllCodes(DAC8565Device* const dev, const uint16_t codes[DAC8565_CHANNEL_COUNT]);

/**
 * @brief Voltage variant of ::dac8565WriteAllCodes; encodes each per RSTSEL.
 */
DAC8565Error dac8565WriteAllVoltagesUv(DAC8565Device* const dev, const uint32_t voltages_uv[DAC8565_CHANNEL_COUNT]);

/**
 * @brief Broadcast: write the same code to all four channels in one frame
 *        (datasheet "Broadcast Modes" with DB18=1).
 */
DAC8565Error dac8565WriteBroadcastCode(DAC8565Device* const dev, const uint16_t code);

DAC8565Error dac8565WriteBroadcastVoltageUv(DAC8565Device* const dev, const uint32_t voltage_uv);

/* ── 5. Buffered (LDAC) staging ───────────────────────────────────────────── */

/**
 * @brief Stage a code into a channel's buffer (LD=00) without updating its DAC register.
 *
 * The output remains at its previous value until ::dac8565CommitBuffers
 * pulses LDAC, at which point all staged buffers transfer to their
 * corresponding DAC registers simultaneously.
 *
 * Returns ::DAC8565_ERROR_NOT_SUPPORTED if the HAL did not provide `ldacSet`.
 */
DAC8565Error dac8565StageCode(DAC8565Device* const dev, const DAC8565Channel channel, const uint16_t code);

DAC8565Error dac8565StageVoltageUv(DAC8565Device* const dev, const DAC8565Channel channel, const uint32_t voltage_uv);

/**
 * @brief Pulse LDAC LOW→HIGH→LOW to commit any staged buffer writes.
 *
 * Idle level is LOW. The pulse needs to be at least t15_min ≈ 10 ns wide;
 * a 1 µs pulse via the HAL `delayUs` (when provided) is generous and safe.
 */
DAC8565Error dac8565CommitBuffers(DAC8565Device* const dev);

/* ── 6. Power-down configuration ──────────────────────────────────────────── */

/**
 * @brief Set a single channel's power-down mode (single-channel UPDATE frame).
 */
DAC8565Error dac8565SetChannelPowerMode(DAC8565Device* const   dev,
                                        const DAC8565Channel   channel,
                                        const DAC8565PowerMode mode);

/**
 * @brief Set all four power-down modes synchronously (datasheet "Operating Examples — Example 3").
 *
 * Issues 3× single-channel STORE PD frames + 1× store-and-load-all PD frame
 * so all four outputs change state on the same SCLK edge.
 */
DAC8565Error dac8565SetAllPowerModes(DAC8565Device* const   dev,
                                     const DAC8565PowerMode modes[DAC8565_CHANNEL_COUNT]);

DAC8565Error dac8565GetChannelPowerMode(const DAC8565Device* const dev,
                                        const DAC8565Channel       channel,
                                        DAC8565PowerMode* const    mode);

/* ── 7. Cached state accessors ────────────────────────────────────────────── */

DAC8565Error dac8565GetLastCode(const DAC8565Device* const dev,
                                const DAC8565Channel       channel,
                                uint16_t* const            code);

DAC8565Error dac8565GetLastVoltageUv(const DAC8565Device* const dev,
                                     const DAC8565Channel       channel,
                                     uint32_t* const            voltage_uv);

DAC8565Error dac8565GetEncoding(const DAC8565Device* const dev, DAC8565Encoding* const encoding);

/* ── 8. Hardware pin control ──────────────────────────────────────────────── */

/**
 * @brief Drive the active-low ~ENABLE pin. Requires `hal.enableSet`.
 *
 * @param[in] level  `true` = drive HIGH (SPI port disconnected), `false` = drive LOW (SPI active).
 */
DAC8565Error dac8565SetEnable(DAC8565Device* const dev, const bool level);

/**
 * @brief Manually pulse LDAC. Useful for advanced scheduling. Requires `hal.ldacSet`.
 */
DAC8565Error dac8565PulseLDAC(DAC8565Device* const dev);

#ifdef __cplusplus
}
#endif

#endif /* DAC8565_H */
