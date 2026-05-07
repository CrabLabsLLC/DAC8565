/**
 * @file dac8565.c
 * @brief DAC8565 quad-channel 16-bit DAC driver implementation.
 *
 * @author Orion Serup <orion@crablabs.io>
 *
 * @reviewer TBD <reviewer@crablabs.io>
 */

#include "dac8565.h"

#include <stddef.h>

/* ── Frame field positions in byte 0 (datasheet Table 5) ──────────────────── */
/* byte0 = [0][0][LD1][LD0][0][DSEL1][DSEL0][PD0]
 *           7  6   5    4   3    2     1     0   bit positions */
#define DAC8565_LD_SHIFT   4U
#define DAC8565_LD_MASK    (0x3U << DAC8565_LD_SHIFT)
#define DAC8565_DSEL_SHIFT 1U
#define DAC8565_DSEL_MASK  (0x3U << DAC8565_DSEL_SHIFT)
#define DAC8565_PD0_SHIFT  0U
#define DAC8565_PD0_MASK   (0x1U << DAC8565_PD0_SHIFT)

/* ── Load-command (LD1:LD0) values (datasheet text after Table 5) ─────────── */
#define DAC8565_LD_STORE          0x0U /* Single-channel STORE: write buffer only */
#define DAC8565_LD_UPDATE         0x1U /* Single-channel UPDATE: write buffer + DAC */
#define DAC8565_LD_STORE_LOAD_ALL 0x2U /* Write buffer + load all 4 DAC registers */
#define DAC8565_LD_BROADCAST      0x3U /* Broadcast (DB18 selects sub-mode) */

/* ── PD1:PD2 in byte 1 (when PD0 = 1; datasheet Table 7) ──────────────────── */
#define DAC8565_PD_BITS_SHIFT 6U /* PD1 → bit 7, PD2 → bit 6 of byte 1 */

/* ── Reference command bits in byte 1 (datasheet Tables 2-4) ──────────────── */
#define DAC8565_REF_BYTE0         0x01U /* All ref commands: byte0 = 0x01 (PD0=1, DSEL=00, LD=00) */
#define DAC8565_REF_BYTE1_DEFAULT 0x00U /* 010000h — default mode (auto-tracks DAC PD) */
#define DAC8565_REF_BYTE1_ALWAYS  0x10U /* 011000h — always-on */
#define DAC8565_REF_BYTE1_DISABLE 0x20U /* 012000h — always-disabled */

/* ── Two's-complement MSB flip (RSTSEL = high) ────────────────────────────── */
#define DAC8565_TC_MSB_FLIP 0x8000U

/* ── Settle / pulse timings ───────────────────────────────────────────────── */
#define DAC8565_LDAC_PULSE_US    1U  /* tLDAC_min ≈ 10 ns; 1 µs is generous */
#define DAC8565_RESET_PULSE_US   5U  /* RST hold time */
#define DAC8565_PD_EXIT_DELAY_US 10U /* Coming-out-of-PD power-up time */

/* ── Forward declarations ─────────────────────────────────────────────────── */

static DAC8565Error dac8565BuildDataFrame(uint8_t              frame[DAC8565_FRAME_BYTES],
                                          const uint8_t        ld_bits,
                                          const DAC8565Channel channel,
                                          const uint16_t       code);

static DAC8565Error dac8565BuildPdFrame(uint8_t                frame[DAC8565_FRAME_BYTES],
                                        const uint8_t          ld_bits,
                                        const DAC8565Channel   channel,
                                        const DAC8565PowerMode mode);

static DAC8565Error dac8565BuildReferenceFrame(uint8_t                    frame[DAC8565_FRAME_BYTES],
                                               const DAC8565ReferenceMode mode);

static DAC8565Error dac8565BuildBroadcastFrame(uint8_t        frame[DAC8565_FRAME_BYTES],
                                               const uint16_t code);

static DAC8565Error dac8565Transmit(const DAC8565Device* const dev,
                                    const uint8_t              frame[DAC8565_FRAME_BYTES]);

static DAC8565Error dac8565CommitReferenceMode(DAC8565Device* const dev, const DAC8565ReferenceMode mode);

static uint16_t dac8565EncodeVoltage(const DAC8565Device* const dev, const uint32_t voltage_uv);
static uint32_t dac8565DecodeVoltage(const DAC8565Device* const dev, const uint16_t code);

static bool dac8565IsValidChannel(const DAC8565Channel channel);
static bool dac8565IsValidPowerMode(const DAC8565PowerMode mode);

/* ── 1. Initialization / Lifecycle ────────────────────────────────────────── */

DAC8565Error dac8565Init(DAC8565Device* const       dev,
                         const DAC8565Config* const config,
                         const DAC8565HAL* const    hal)
{
	if (dev == NULL || config == NULL || hal == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (hal->spiWrite == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (config->encoding != DAC8565_ENCODING_BINARY && config->encoding != DAC8565_ENCODING_TWOS_COMPLEMENT)
		return DAC8565_ERROR_INVALID_PARAM;
	if (config->initial_reference_mode > DAC8565_REF_MODE_DISABLED)
		return DAC8565_ERROR_INVALID_PARAM;
	if (config->initial_reference_mode == DAC8565_REF_MODE_DISABLED &&
	    config->external_reference_voltage_uv == 0U)
		return DAC8565_ERROR_INVALID_PARAM;

	dev->hal = *hal;

	dev->encoding                      = config->encoding;
	dev->reference_mode                = config->initial_reference_mode;
	dev->external_reference_voltage_uv = config->external_reference_voltage_uv;
	dev->reference_voltage_uv          = (config->initial_reference_mode == DAC8565_REF_MODE_DISABLED)
	                                         ? config->external_reference_voltage_uv
	                                         : DAC8565_INTERNAL_REFERENCE_UV;
	for (uint32_t i = 0U; i < DAC8565_CHANNEL_COUNT; i++)
	{
		dev->last_codes[i]   = 0U;
		dev->staged_codes[i] = 0U;
		dev->power_modes[i]  = DAC8565_POWER_MODE_NORMAL;
	}
	dev->staged_valid   = 0U;
	dev->is_initialized = false;

	/* Commit the reference mode first (one frame). The chip is write-only,
	 * so a SPI bus error here is the only connectivity check we have. */
	uint8_t            ref_frame[DAC8565_FRAME_BYTES];
	const DAC8565Error build_err = dac8565BuildReferenceFrame(ref_frame, config->initial_reference_mode);
	if (build_err != DAC8565_ERROR_OK)
		return build_err;
	if (dev->hal.spiWrite(ref_frame, DAC8565_FRAME_BYTES) != 0)
		return DAC8565_ERROR_COMM_FAIL;

	/* Drive every channel to "0 V" (per the configured encoding). */
	const uint16_t zero_code = dac8565EncodeVoltage(dev, 0U);
	uint8_t        frame[DAC8565_FRAME_BYTES];
	for (uint32_t i = 0U; i < DAC8565_CHANNEL_COUNT; i++)
	{
		const DAC8565Error f_err = dac8565BuildDataFrame(frame, DAC8565_LD_UPDATE,
		                                                 (DAC8565Channel)i, zero_code);
		if (f_err != DAC8565_ERROR_OK)
			return f_err;
		if (dev->hal.spiWrite(frame, DAC8565_FRAME_BYTES) != 0)
			return DAC8565_ERROR_COMM_FAIL;
		dev->last_codes[i] = zero_code;
	}

	dev->is_initialized = true;
	return DAC8565_ERROR_OK;
}

DAC8565Error dac8565Reset(DAC8565Device* const dev)
{
	if (dev == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;

	const uint16_t zero_code = dac8565EncodeVoltage(dev, 0U);
	uint16_t       codes[DAC8565_CHANNEL_COUNT];
	for (uint32_t i = 0U; i < DAC8565_CHANNEL_COUNT; i++)
		codes[i] = zero_code;

	return dac8565WriteAllCodes(dev, codes);
}

DAC8565Error dac8565ResetHardware(DAC8565Device* const dev)
{
	if (dev == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;
	if (dev->hal.resetSet == NULL)
		return DAC8565_ERROR_NOT_SUPPORTED;

	dev->hal.resetSet(false); /* assert ~RST low */
	if (dev->hal.delayUs != NULL)
		dev->hal.delayUs(DAC8565_RESET_PULSE_US);
	dev->hal.resetSet(true); /* release */

	/* The chip's DAC register holds the raw value 0x0000 after either kind of
	 * hardware reset:
	 *   RSTSEL=0 (binary): 0x0000 == zero-scale == 0 V
	 *   RSTSEL=1 (TC):     0x0000 == midscale == V_REF / 2
	 * The driver stores raw register bits in `last_codes`; the encoding-aware
	 * decode path (dac8565GetLastVoltageUv) reports 0 V or V_REF/2 accordingly. */
	for (uint32_t i = 0U; i < DAC8565_CHANNEL_COUNT; i++)
	{
		dev->last_codes[i]   = 0U;
		dev->staged_codes[i] = 0U;
		dev->staged_valid    = 0U;
		dev->power_modes[i]  = DAC8565_POWER_MODE_NORMAL;
	}
	return DAC8565_ERROR_OK;
}

DAC8565Error dac8565Deinit(DAC8565Device* const dev)
{
	if (dev == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;

	for (uint32_t i = 0U; i < DAC8565_CHANNEL_COUNT; i++)
	{
		dev->last_codes[i]  = 0U;
		dev->power_modes[i] = DAC8565_POWER_MODE_NORMAL;
	}
	dev->is_initialized       = false;
	dev->reference_voltage_uv = 0U;
	dev->reference_mode       = DAC8565_REF_MODE_DEFAULT;
	dev->encoding             = DAC8565_ENCODING_BINARY;

	dev->hal.spiWrite  = NULL;
	dev->hal.ldacSet   = NULL;
	dev->hal.resetSet  = NULL;
	dev->hal.enableSet = NULL;
	dev->hal.delayUs   = NULL;
	return DAC8565_ERROR_OK;
}

/* ── 2. Reference configuration ───────────────────────────────────────────── */

DAC8565Error dac8565SetReferenceMode(DAC8565Device* const dev, const DAC8565ReferenceMode mode)
{
	if (dev == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;
	if (mode > DAC8565_REF_MODE_DISABLED)
		return DAC8565_ERROR_INVALID_PARAM;

	return dac8565CommitReferenceMode(dev, mode);
}

DAC8565Error dac8565GetReferenceMode(const DAC8565Device* const dev, DAC8565ReferenceMode* const mode)
{
	if (dev == NULL || mode == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;

	*mode = dev->reference_mode;
	return DAC8565_ERROR_OK;
}

DAC8565Error dac8565SetExternalReferenceVoltageUv(DAC8565Device* const dev, const uint32_t voltage_uv)
{
	if (dev == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;
	if (voltage_uv == 0U)
		return DAC8565_ERROR_INVALID_PARAM;

	/* Always cache the external value so a subsequent switch to
	 * DAC8565_REF_MODE_DISABLED picks it up. If the internal reference is
	 * currently active, reference_voltage_uv stays at 2.5 V until that
	 * switch happens. */
	dev->external_reference_voltage_uv = voltage_uv;
	if (dev->reference_mode == DAC8565_REF_MODE_DISABLED)
		dev->reference_voltage_uv = voltage_uv;
	return DAC8565_ERROR_OK;
}

DAC8565Error dac8565GetReferenceVoltageUv(const DAC8565Device* const dev, uint32_t* const voltage_uv)
{
	if (dev == NULL || voltage_uv == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;

	*voltage_uv = dev->reference_voltage_uv;
	return DAC8565_ERROR_OK;
}

/* ── 3. Per-channel data output ───────────────────────────────────────────── */

DAC8565Error dac8565WriteCode(DAC8565Device* const dev, const DAC8565Channel channel, const uint16_t code)
{
	if (dev == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;
	if (!dac8565IsValidChannel(channel))
		return DAC8565_ERROR_INVALID_PARAM;

	uint8_t            frame[DAC8565_FRAME_BYTES];
	const DAC8565Error build_err = dac8565BuildDataFrame(frame, DAC8565_LD_UPDATE, channel, code);
	if (build_err != DAC8565_ERROR_OK)
		return build_err;

	const DAC8565Error tx_err = dac8565Transmit(dev, frame);
	if (tx_err != DAC8565_ERROR_OK)
		return tx_err;

	const bool was_powered_down = (dev->power_modes[channel] != DAC8565_POWER_MODE_NORMAL);
	dev->last_codes[channel]    = code;
	dev->power_modes[channel]   = DAC8565_POWER_MODE_NORMAL; /* a data write resumes normal mode */
	if (was_powered_down && dev->hal.delayUs != NULL)
		dev->hal.delayUs(DAC8565_PD_EXIT_DELAY_US);
	return DAC8565_ERROR_OK;
}

DAC8565Error dac8565WriteVoltageUv(DAC8565Device* const dev, const DAC8565Channel channel, const uint32_t voltage_uv)
{
	if (dev == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;
	if (!dac8565IsValidChannel(channel))
		return DAC8565_ERROR_INVALID_PARAM;
	if (voltage_uv > dev->reference_voltage_uv)
		return DAC8565_ERROR_INVALID_PARAM;

	return dac8565WriteCode(dev, channel, dac8565EncodeVoltage(dev, voltage_uv));
}

/* ── 4. Multi-channel synchronous output ──────────────────────────────────── */

DAC8565Error dac8565WriteAllCodes(DAC8565Device* const dev, const uint16_t codes[DAC8565_CHANNEL_COUNT])
{
	if (dev == NULL || codes == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;

	/* Datasheet "Operating Examples — Example 1": A/B/C single-channel STORE
	 * (LD=00), then D store-and-load-all (LD=10). All four outputs settle on
	 * the 24th SCLK falling edge of the 4th frame. */
	uint8_t frame[DAC8565_FRAME_BYTES];
	for (uint32_t i = 0U; i < DAC8565_CHANNEL_COUNT; i++)
	{
		const uint8_t      ld    = (i == DAC8565_CHANNEL_COUNT - 1U) ? DAC8565_LD_STORE_LOAD_ALL : DAC8565_LD_STORE;
		const DAC8565Error f_err = dac8565BuildDataFrame(frame, ld, (DAC8565Channel)i, codes[i]);
		if (f_err != DAC8565_ERROR_OK)
			return f_err;
		const DAC8565Error t_err = dac8565Transmit(dev, frame);
		if (t_err != DAC8565_ERROR_OK)
			return t_err;
	}

	bool any_was_powered_down = false;
	for (uint32_t i = 0U; i < DAC8565_CHANNEL_COUNT; i++)
	{
		if (dev->power_modes[i] != DAC8565_POWER_MODE_NORMAL)
			any_was_powered_down = true;
		dev->last_codes[i]  = codes[i];
		dev->power_modes[i] = DAC8565_POWER_MODE_NORMAL;
	}
	if (any_was_powered_down && dev->hal.delayUs != NULL)
		dev->hal.delayUs(DAC8565_PD_EXIT_DELAY_US);
	return DAC8565_ERROR_OK;
}

DAC8565Error dac8565WriteAllVoltagesUv(DAC8565Device* const dev, const uint32_t voltages_uv[DAC8565_CHANNEL_COUNT])
{
	if (dev == NULL || voltages_uv == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;

	uint16_t codes[DAC8565_CHANNEL_COUNT];
	for (uint32_t i = 0U; i < DAC8565_CHANNEL_COUNT; i++)
	{
		if (voltages_uv[i] > dev->reference_voltage_uv)
			return DAC8565_ERROR_INVALID_PARAM;
		codes[i] = dac8565EncodeVoltage(dev, voltages_uv[i]);
	}
	return dac8565WriteAllCodes(dev, codes);
}

DAC8565Error dac8565WriteBroadcastCode(DAC8565Device* const dev, const uint16_t code)
{
	if (dev == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;

	uint8_t            frame[DAC8565_FRAME_BYTES];
	const DAC8565Error build_err = dac8565BuildBroadcastFrame(frame, code);
	if (build_err != DAC8565_ERROR_OK)
		return build_err;

	const DAC8565Error tx_err = dac8565Transmit(dev, frame);
	if (tx_err != DAC8565_ERROR_OK)
		return tx_err;

	bool any_was_powered_down = false;
	for (uint32_t i = 0U; i < DAC8565_CHANNEL_COUNT; i++)
	{
		if (dev->power_modes[i] != DAC8565_POWER_MODE_NORMAL)
			any_was_powered_down = true;
		dev->last_codes[i]  = code;
		dev->power_modes[i] = DAC8565_POWER_MODE_NORMAL;
	}
	if (any_was_powered_down && dev->hal.delayUs != NULL)
		dev->hal.delayUs(DAC8565_PD_EXIT_DELAY_US);
	return DAC8565_ERROR_OK;
}

DAC8565Error dac8565WriteBroadcastVoltageUv(DAC8565Device* const dev, const uint32_t voltage_uv)
{
	if (dev == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;
	if (voltage_uv > dev->reference_voltage_uv)
		return DAC8565_ERROR_INVALID_PARAM;

	return dac8565WriteBroadcastCode(dev, dac8565EncodeVoltage(dev, voltage_uv));
}

/* ── 5. Buffered (LDAC) staging ───────────────────────────────────────────── */

DAC8565Error dac8565StageCode(DAC8565Device* const dev, const DAC8565Channel channel, const uint16_t code)
{
	if (dev == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;
	if (!dac8565IsValidChannel(channel))
		return DAC8565_ERROR_INVALID_PARAM;
	if (dev->hal.ldacSet == NULL)
		return DAC8565_ERROR_NOT_SUPPORTED;

	uint8_t            frame[DAC8565_FRAME_BYTES];
	const DAC8565Error build_err = dac8565BuildDataFrame(frame, DAC8565_LD_STORE, channel, code);
	if (build_err != DAC8565_ERROR_OK)
		return build_err;
	const DAC8565Error tx_err = dac8565Transmit(dev, frame);
	if (tx_err != DAC8565_ERROR_OK)
		return tx_err;

	/* Track the staged value so dac8565CommitBuffers can flush it into
	 * last_codes. last_codes is NOT updated here — the chip's V_OUT does
	 * not change until LDAC pulses. */
	dev->staged_codes[channel] = code;
	dev->staged_valid          = (uint8_t)(dev->staged_valid | (1U << (uint8_t)channel));
	return DAC8565_ERROR_OK;
}

DAC8565Error dac8565StageVoltageUv(DAC8565Device* const dev, const DAC8565Channel channel, const uint32_t voltage_uv)
{
	if (dev == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;
	if (!dac8565IsValidChannel(channel))
		return DAC8565_ERROR_INVALID_PARAM;
	if (voltage_uv > dev->reference_voltage_uv)
		return DAC8565_ERROR_INVALID_PARAM;

	return dac8565StageCode(dev, channel, dac8565EncodeVoltage(dev, voltage_uv));
}

DAC8565Error dac8565CommitBuffers(DAC8565Device* const dev)
{
	if (dev == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;
	if (dev->hal.ldacSet == NULL)
		return DAC8565_ERROR_NOT_SUPPORTED;

	dev->hal.ldacSet(true);
	if (dev->hal.delayUs != NULL)
		dev->hal.delayUs(DAC8565_LDAC_PULSE_US);
	dev->hal.ldacSet(false);

	/* Flush staged values into the cached last_codes for any channel that
	 * was staged. Channels with no staged write keep their previous cache. */
	for (uint32_t i = 0U; i < DAC8565_CHANNEL_COUNT; i++)
	{
		if ((dev->staged_valid & (1U << i)) != 0U)
		{
			dev->last_codes[i]  = dev->staged_codes[i];
			dev->power_modes[i] = DAC8565_POWER_MODE_NORMAL;
		}
	}
	dev->staged_valid = 0U;
	return DAC8565_ERROR_OK;
}

/* ── 6. Power-down configuration ──────────────────────────────────────────── */

DAC8565Error dac8565SetChannelPowerMode(DAC8565Device* const   dev,
                                        const DAC8565Channel   channel,
                                        const DAC8565PowerMode mode)
{
	if (dev == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;
	if (!dac8565IsValidChannel(channel))
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dac8565IsValidPowerMode(mode))
		return DAC8565_ERROR_INVALID_PARAM;

	uint8_t            frame[DAC8565_FRAME_BYTES];
	const DAC8565Error build_err = dac8565BuildPdFrame(frame, DAC8565_LD_UPDATE, channel, mode);
	if (build_err != DAC8565_ERROR_OK)
		return build_err;

	const DAC8565Error tx_err = dac8565Transmit(dev, frame);
	if (tx_err != DAC8565_ERROR_OK)
		return tx_err;

	const bool was_powered_down = (dev->power_modes[channel] != DAC8565_POWER_MODE_NORMAL);
	const bool now_normal       = (mode == DAC8565_POWER_MODE_NORMAL);
	dev->power_modes[channel]   = mode;
	if (was_powered_down && now_normal && dev->hal.delayUs != NULL)
		dev->hal.delayUs(DAC8565_PD_EXIT_DELAY_US);
	return DAC8565_ERROR_OK;
}

DAC8565Error dac8565SetAllPowerModes(DAC8565Device* const   dev,
                                     const DAC8565PowerMode modes[DAC8565_CHANNEL_COUNT])
{
	if (dev == NULL || modes == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;
	for (uint32_t i = 0U; i < DAC8565_CHANNEL_COUNT; i++)
	{
		if (!dac8565IsValidPowerMode(modes[i]))
			return DAC8565_ERROR_INVALID_PARAM;
	}

	/* Datasheet Example 3: 3× single-channel PD STORE + 1× PD store-and-load-all. */
	uint8_t frame[DAC8565_FRAME_BYTES];
	for (uint32_t i = 0U; i < DAC8565_CHANNEL_COUNT; i++)
	{
		const uint8_t      ld    = (i == DAC8565_CHANNEL_COUNT - 1U) ? DAC8565_LD_STORE_LOAD_ALL : DAC8565_LD_STORE;
		const DAC8565Error f_err = dac8565BuildPdFrame(frame, ld, (DAC8565Channel)i, modes[i]);
		if (f_err != DAC8565_ERROR_OK)
			return f_err;
		const DAC8565Error t_err = dac8565Transmit(dev, frame);
		if (t_err != DAC8565_ERROR_OK)
			return t_err;
	}

	for (uint32_t i = 0U; i < DAC8565_CHANNEL_COUNT; i++)
		dev->power_modes[i] = modes[i];
	return DAC8565_ERROR_OK;
}

DAC8565Error dac8565GetChannelPowerMode(const DAC8565Device* const dev,
                                        const DAC8565Channel       channel,
                                        DAC8565PowerMode* const    mode)
{
	if (dev == NULL || mode == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;
	if (!dac8565IsValidChannel(channel))
		return DAC8565_ERROR_INVALID_PARAM;

	*mode = dev->power_modes[channel];
	return DAC8565_ERROR_OK;
}

/* ── 7. Cached state accessors ────────────────────────────────────────────── */

DAC8565Error dac8565GetLastCode(const DAC8565Device* const dev,
                                const DAC8565Channel       channel,
                                uint16_t* const            code)
{
	if (dev == NULL || code == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;
	if (!dac8565IsValidChannel(channel))
		return DAC8565_ERROR_INVALID_PARAM;

	*code = dev->last_codes[channel];
	return DAC8565_ERROR_OK;
}

DAC8565Error dac8565GetLastVoltageUv(const DAC8565Device* const dev,
                                     const DAC8565Channel       channel,
                                     uint32_t* const            voltage_uv)
{
	if (dev == NULL || voltage_uv == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;
	if (!dac8565IsValidChannel(channel))
		return DAC8565_ERROR_INVALID_PARAM;

	*voltage_uv = dac8565DecodeVoltage(dev, dev->last_codes[channel]);
	return DAC8565_ERROR_OK;
}

DAC8565Error dac8565GetEncoding(const DAC8565Device* const dev, DAC8565Encoding* const encoding)
{
	if (dev == NULL || encoding == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;

	*encoding = dev->encoding;
	return DAC8565_ERROR_OK;
}

/* ── 8. Hardware pin control ──────────────────────────────────────────────── */

DAC8565Error dac8565SetEnable(DAC8565Device* const dev, const bool level)
{
	if (dev == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;
	if (dev->hal.enableSet == NULL)
		return DAC8565_ERROR_NOT_SUPPORTED;

	dev->hal.enableSet(level);
	return DAC8565_ERROR_OK;
}

DAC8565Error dac8565PulseLDAC(DAC8565Device* const dev)
{
	if (dev == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return DAC8565_ERROR_NOT_INIT;
	if (dev->hal.ldacSet == NULL)
		return DAC8565_ERROR_NOT_SUPPORTED;

	/* Raw GPIO pulse only — does NOT update the staged-buffer cache.
	 * Use this when LDAC is being orchestrated outside the staging API
	 * (e.g., chip-internal sync via the LD=10 frame, or external triggering).
	 * For the typical Stage* → Commit flow, prefer ::dac8565CommitBuffers. */
	dev->hal.ldacSet(true);
	if (dev->hal.delayUs != NULL)
		dev->hal.delayUs(DAC8565_LDAC_PULSE_US);
	dev->hal.ldacSet(false);
	return DAC8565_ERROR_OK;
}

/* ── Internal helpers ─────────────────────────────────────────────────────── */

static DAC8565Error dac8565BuildDataFrame(uint8_t              frame[DAC8565_FRAME_BYTES],
                                          const uint8_t        ld_bits,
                                          const DAC8565Channel channel,
                                          const uint16_t       code)
{
	if (frame == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dac8565IsValidChannel(channel))
		return DAC8565_ERROR_INVALID_PARAM;
	if ((ld_bits & ~0x3U) != 0U)
		return DAC8565_ERROR_INVALID_PARAM;

	frame[0] = (uint8_t)((ld_bits << DAC8565_LD_SHIFT) | ((uint8_t)channel << DAC8565_DSEL_SHIFT));
	frame[1] = (uint8_t)(code >> 8);
	frame[2] = (uint8_t)(code & 0xFFU);
	return DAC8565_ERROR_OK;
}

static DAC8565Error dac8565BuildPdFrame(uint8_t                frame[DAC8565_FRAME_BYTES],
                                        const uint8_t          ld_bits,
                                        const DAC8565Channel   channel,
                                        const DAC8565PowerMode mode)
{
	if (frame == NULL)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dac8565IsValidChannel(channel))
		return DAC8565_ERROR_INVALID_PARAM;
	if ((ld_bits & ~0x3U) != 0U)
		return DAC8565_ERROR_INVALID_PARAM;
	if (!dac8565IsValidPowerMode(mode))
		return DAC8565_ERROR_INVALID_PARAM;

	const uint8_t pd0 = (mode == DAC8565_POWER_MODE_NORMAL) ? 0U : 1U;
	frame[0]          = (uint8_t)((ld_bits << DAC8565_LD_SHIFT) |
                         ((uint8_t)channel << DAC8565_DSEL_SHIFT) |
                         (pd0 << DAC8565_PD0_SHIFT));
	frame[1]          = (uint8_t)(((uint8_t)mode & 0x3U) << DAC8565_PD_BITS_SHIFT);
	frame[2]          = 0U;
	return DAC8565_ERROR_OK;
}

static DAC8565Error dac8565BuildReferenceFrame(uint8_t                    frame[DAC8565_FRAME_BYTES],
                                               const DAC8565ReferenceMode mode)
{
	if (frame == NULL)
		return DAC8565_ERROR_INVALID_PARAM;

	uint8_t byte1;
	switch (mode)
	{
	case DAC8565_REF_MODE_DEFAULT:
		byte1 = DAC8565_REF_BYTE1_DEFAULT;
		break;
	case DAC8565_REF_MODE_ALWAYS_ON:
		byte1 = DAC8565_REF_BYTE1_ALWAYS;
		break;
	case DAC8565_REF_MODE_DISABLED:
		byte1 = DAC8565_REF_BYTE1_DISABLE;
		break;
	default:
		return DAC8565_ERROR_INVALID_PARAM;
	}

	frame[0] = DAC8565_REF_BYTE0;
	frame[1] = byte1;
	frame[2] = 0U;
	return DAC8565_ERROR_OK;
}

static DAC8565Error dac8565BuildBroadcastFrame(uint8_t        frame[DAC8565_FRAME_BYTES],
                                               const uint16_t code)
{
	if (frame == NULL)
		return DAC8565_ERROR_INVALID_PARAM;

	/* Broadcast update with SR data: LD=11 (DAC8565_LD_BROADCAST), DB18=1, PD0=0.
	 * byte 0 = 0011 0100 = (LD=11)<<4 | (DB18=1)<<2 = 0x30 | 0x04 = 0x34. */
	frame[0] = (uint8_t)((DAC8565_LD_BROADCAST << DAC8565_LD_SHIFT) | (1U << 2));
	frame[1] = (uint8_t)(code >> 8);
	frame[2] = (uint8_t)(code & 0xFFU);
	return DAC8565_ERROR_OK;
}

static DAC8565Error dac8565Transmit(const DAC8565Device* const dev,
                                    const uint8_t              frame[DAC8565_FRAME_BYTES])
{
	if (dev->hal.spiWrite(frame, DAC8565_FRAME_BYTES) != 0)
		return DAC8565_ERROR_COMM_FAIL;
	return DAC8565_ERROR_OK;
}

static DAC8565Error dac8565CommitReferenceMode(DAC8565Device* const dev, const DAC8565ReferenceMode mode)
{
	uint8_t            frame[DAC8565_FRAME_BYTES];
	const DAC8565Error build_err = dac8565BuildReferenceFrame(frame, mode);
	if (build_err != DAC8565_ERROR_OK)
		return build_err;

	const DAC8565Error tx_err = dac8565Transmit(dev, frame);
	if (tx_err != DAC8565_ERROR_OK)
		return tx_err;

	dev->reference_mode = mode;
	if (mode != DAC8565_REF_MODE_DISABLED)
		dev->reference_voltage_uv = DAC8565_INTERNAL_REFERENCE_UV;
	/* If switching to DISABLED, the application should have already supplied
	 * an external reference via dac8565SetExternalReferenceVoltageUv (or it
	 * was set at init). Conservatively, do not overwrite the cached value. */
	return DAC8565_ERROR_OK;
}

/**
 * @brief Convert a microvolt request into a chip code, applying the
 *        configured RSTSEL encoding.
 *
 * Performs `code = round(voltage_uv × 65536 / V_REF)` in 64-bit math (binary).
 * For two's-complement coding, XORs the MSB to map binary → TC.
 */
static uint16_t dac8565EncodeVoltage(const DAC8565Device* const dev, const uint32_t voltage_uv)
{
	const uint64_t scaled    = (uint64_t)voltage_uv * (uint64_t)DAC8565_FULL_SCALE;
	const uint64_t half_lsb  = (uint64_t)dev->reference_voltage_uv / 2U;
	const uint32_t code_full = (uint32_t)((scaled + half_lsb) / (uint64_t)dev->reference_voltage_uv);
	const uint16_t code_bin  = (code_full > DAC8565_CODE_MAX) ? (uint16_t)DAC8565_CODE_MAX : (uint16_t)code_full;

	return (dev->encoding == DAC8565_ENCODING_TWOS_COMPLEMENT)
	           ? (uint16_t)(code_bin ^ DAC8565_TC_MSB_FLIP)
	           : code_bin;
}

/**
 * @brief Convert a stored chip code back into microvolts.
 */
static uint32_t dac8565DecodeVoltage(const DAC8565Device* const dev, const uint16_t code)
{
	const uint16_t code_bin = (dev->encoding == DAC8565_ENCODING_TWOS_COMPLEMENT)
	                              ? (uint16_t)(code ^ DAC8565_TC_MSB_FLIP)
	                              : code;
	const uint64_t scaled   = (uint64_t)code_bin * (uint64_t)dev->reference_voltage_uv;
	return (uint32_t)(scaled / (uint64_t)DAC8565_FULL_SCALE);
}

static bool dac8565IsValidChannel(const DAC8565Channel channel)
{
	return ((uint8_t)channel < DAC8565_CHANNEL_COUNT);
}

static bool dac8565IsValidPowerMode(const DAC8565PowerMode mode)
{
	return ((uint8_t)mode <= (uint8_t)DAC8565_POWER_MODE_PD_HIZ);
}
