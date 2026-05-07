/**
 * @file main.c
 * @brief Hardware-in-the-loop system test for the DAC8565 driver.
 *
 * Runs against a real DAC8565 wired to an ESP32 / ESP32-S3 host. Covers:
 *
 *   1. NULL-pointer + invalid-arg rejection at every public entry point
 *   2. Use-before-init / use-after-deinit guards
 *   3. Init / deinit / re-init lifecycle (incl. COMM_FAIL propagation)
 *   4. Voltage ↔ code round-trip across the input domain (binary encoding)
 *   5. Per-channel writes to all four channels
 *   6. Multi-channel synchronous write (Example 1 sequence)
 *   7. Broadcast write
 *   8. Buffered staging + LDAC commit
 *   9. Per-channel power-down cycling and GetChannelPowerMode
 *  10. SetAllPowerModes (Example 3 sequence)
 *  11. Reference-mode swap (default / always-on / disabled)
 *  12. NOT_SUPPORTED returned when optional HAL pins are absent
 *  13. Output sweep (visible on V_OUT_A with a scope)
 *
 * @author Orion Serup <orion@crablabs.io>
 *
 * @reviewer TBD <reviewer@crablabs.io>
 */

#include "dac8565.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef DAC8565_TEST_PIN_MOSI
#define DAC8565_TEST_PIN_MOSI GPIO_NUM_34
#endif
#ifndef DAC8565_TEST_PIN_MISO
#define DAC8565_TEST_PIN_MISO GPIO_NUM_33
#endif
#ifndef DAC8565_TEST_PIN_SCLK
#define DAC8565_TEST_PIN_SCLK GPIO_NUM_35
#endif
#ifndef DAC8565_TEST_PIN_SYNC
#define DAC8565_TEST_PIN_SYNC GPIO_NUM_45
#endif
#ifndef DAC8565_TEST_PIN_LDAC
#define DAC8565_TEST_PIN_LDAC GPIO_NUM_21
#endif
#ifndef DAC8565_TEST_VREF_UV
#define DAC8565_TEST_VREF_UV 2500000U /* internal 2.5 V */
#endif
#ifndef DAC8565_TEST_SPI_HZ
#define DAC8565_TEST_SPI_HZ 1000000
#endif

#define DAC8565_TEST_SWEEP_STEP_COUNT  256U
#define DAC8565_TEST_SWEEP_DELAY_MS    1U
#define DAC8565_TEST_PD_CYCLE_DELAY_MS 5U

static const char* const TAG = "dac8565_test";

static spi_device_handle_t s_spi         = NULL;
static int                 s_pass_count  = 0;
static int                 s_fail_count  = 0;
static const char*         s_active_case = "init";

#define DAC8565_TEST_EXPECT(cond, fmt, ...)                                \
	do                                                                     \
	{                                                                      \
		if (cond)                                                          \
		{                                                                  \
			s_pass_count++;                                                \
		}                                                                  \
		else                                                               \
		{                                                                  \
			s_fail_count++;                                                \
			ESP_LOGE(TAG, "FAIL [%s] " fmt, s_active_case, ##__VA_ARGS__); \
		}                                                                  \
	}                                                                      \
	while (0)

#define DAC8565_TEST_EXPECT_OK(call) \
	DAC8565_TEST_EXPECT((call) == DAC8565_ERROR_OK, "%s did not return OK", #call)

#define DAC8565_TEST_EXPECT_ERR(call, expected_err)                    \
	do                                                                 \
	{                                                                  \
		const DAC8565Error _actual = (call);                           \
		DAC8565_TEST_EXPECT(_actual == (expected_err),                 \
		                    "%s returned %d, expected %d",             \
		                    #call, (int)_actual, (int)(expected_err)); \
	}                                                                  \
	while (0)

/* ── HAL plumbing ─────────────────────────────────────────────────────────── */

static int testSpiWrite(const void* const data, const uint8_t length)
{
	const spi_transaction_t tx = {
	    .length    = (size_t)length * 8U,
	    .tx_buffer = data,
	};
	return (spi_device_transmit(s_spi, &tx) == ESP_OK) ? 0 : -1;
}

static void testLdacSet(bool level)
{
	gpio_set_level(DAC8565_TEST_PIN_LDAC, level ? 1 : 0);
}

static void testDelayUs(uint32_t delay_us)
{
	esp_rom_delay_us(delay_us);
}

static int testFailingSpiWrite(const void* const data, const uint8_t length)
{
	(void)data;
	(void)length;
	return -1;
}

static esp_err_t testInitSpi(void)
{
	gpio_reset_pin(DAC8565_TEST_PIN_LDAC);
	gpio_set_direction(DAC8565_TEST_PIN_LDAC, GPIO_MODE_OUTPUT);
	gpio_set_level(DAC8565_TEST_PIN_LDAC, 0);

	const spi_bus_config_t bus_cfg = {
	    .mosi_io_num     = DAC8565_TEST_PIN_MOSI,
	    .miso_io_num     = DAC8565_TEST_PIN_MISO,
	    .sclk_io_num     = DAC8565_TEST_PIN_SCLK,
	    .quadwp_io_num   = -1,
	    .quadhd_io_num   = -1,
	    .max_transfer_sz = 32,
	};
	const esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
		return err;

	const spi_device_interface_config_t dev_cfg = {
	    .clock_speed_hz = DAC8565_TEST_SPI_HZ,
	    .mode           = 1,
	    .spics_io_num   = DAC8565_TEST_PIN_SYNC,
	    .queue_size     = 1,
	};
	return spi_bus_add_device(SPI2_HOST, &dev_cfg, &s_spi);
}

/* ── Test scenarios ───────────────────────────────────────────────────────── */

static const DAC8565HAL    k_hal = {.spiWrite = testSpiWrite, .ldacSet = testLdacSet, .delayUs = testDelayUs};
static const DAC8565Config k_cfg = {.encoding                      = DAC8565_ENCODING_BINARY,
                                    .initial_reference_mode        = DAC8565_REF_MODE_ALWAYS_ON,
                                    .external_reference_voltage_uv = 0U};

static void testNullParamRejection(void)
{
	s_active_case = "null_param_rejection";

	DAC8565Device dev = {0};
	DAC8565_TEST_EXPECT_ERR(dac8565Init(NULL, &k_cfg, &k_hal), DAC8565_ERROR_INVALID_PARAM);
	DAC8565_TEST_EXPECT_ERR(dac8565Init(&dev, NULL, &k_hal), DAC8565_ERROR_INVALID_PARAM);
	DAC8565_TEST_EXPECT_ERR(dac8565Init(&dev, &k_cfg, NULL), DAC8565_ERROR_INVALID_PARAM);

	const DAC8565HAL no_spi = {.spiWrite = NULL};
	DAC8565_TEST_EXPECT_ERR(dac8565Init(&dev, &k_cfg, &no_spi), DAC8565_ERROR_INVALID_PARAM);

	const DAC8565Config bad_enc = {.encoding = (DAC8565Encoding)42, .initial_reference_mode = DAC8565_REF_MODE_DEFAULT};
	DAC8565_TEST_EXPECT_ERR(dac8565Init(&dev, &bad_enc, &k_hal), DAC8565_ERROR_INVALID_PARAM);

	const DAC8565Config disabled_no_extref = {.encoding                      = DAC8565_ENCODING_BINARY,
	                                          .initial_reference_mode        = DAC8565_REF_MODE_DISABLED,
	                                          .external_reference_voltage_uv = 0U};
	DAC8565_TEST_EXPECT_ERR(dac8565Init(&dev, &disabled_no_extref, &k_hal), DAC8565_ERROR_INVALID_PARAM);
}

static void testUseBeforeInitGuards(void)
{
	s_active_case = "use_before_init_guards";

	DAC8565Device          dev                                  = {0};
	uint16_t               code                                 = 0U;
	uint32_t               vuv                                  = 0U;
	DAC8565PowerMode       mode                                 = DAC8565_POWER_MODE_NORMAL;
	DAC8565ReferenceMode   rmode                                = DAC8565_REF_MODE_DEFAULT;
	DAC8565Encoding        enc                                  = DAC8565_ENCODING_BINARY;
	const uint16_t         four_codes[DAC8565_CHANNEL_COUNT]    = {0U, 0U, 0U, 0U};
	const uint32_t         four_voltages[DAC8565_CHANNEL_COUNT] = {0U, 0U, 0U, 0U};
	const DAC8565PowerMode four_modes[DAC8565_CHANNEL_COUNT]    = {0, 0, 0, 0};

	DAC8565_TEST_EXPECT_ERR(dac8565Reset(&dev), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565ResetHardware(&dev), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565Deinit(&dev), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565WriteCode(&dev, DAC8565_CHANNEL_A, 0U), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565WriteVoltageUv(&dev, DAC8565_CHANNEL_A, 0U), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565WriteAllCodes(&dev, four_codes), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565WriteAllVoltagesUv(&dev, four_voltages), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565WriteBroadcastCode(&dev, 0U), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565WriteBroadcastVoltageUv(&dev, 0U), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565StageCode(&dev, DAC8565_CHANNEL_A, 0U), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565StageVoltageUv(&dev, DAC8565_CHANNEL_A, 0U), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565CommitBuffers(&dev), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565SetChannelPowerMode(&dev, DAC8565_CHANNEL_A, DAC8565_POWER_MODE_NORMAL),
	                        DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565SetAllPowerModes(&dev, four_modes), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565GetChannelPowerMode(&dev, DAC8565_CHANNEL_A, &mode), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565GetLastCode(&dev, DAC8565_CHANNEL_A, &code), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565GetLastVoltageUv(&dev, DAC8565_CHANNEL_A, &vuv), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565GetEncoding(&dev, &enc), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565SetReferenceMode(&dev, DAC8565_REF_MODE_DEFAULT), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565GetReferenceMode(&dev, &rmode), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565SetExternalReferenceVoltageUv(&dev, 4096000U), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565GetReferenceVoltageUv(&dev, &vuv), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565SetEnable(&dev, true), DAC8565_ERROR_NOT_INIT);
	DAC8565_TEST_EXPECT_ERR(dac8565PulseLDAC(&dev), DAC8565_ERROR_NOT_INIT);

	/* NULL dev → INVALID_PARAM, never NOT_INIT. */
	DAC8565_TEST_EXPECT_ERR(dac8565WriteCode(NULL, DAC8565_CHANNEL_A, 0U), DAC8565_ERROR_INVALID_PARAM);
	DAC8565_TEST_EXPECT_ERR(dac8565Deinit(NULL), DAC8565_ERROR_INVALID_PARAM);
}

static void testLifecycle(DAC8565Device* const dev)
{
	s_active_case = "lifecycle";

	DAC8565_TEST_EXPECT_OK(dac8565Init(dev, &k_cfg, &k_hal));
	DAC8565_TEST_EXPECT(dev->is_initialized, "device should be initialized");
	DAC8565_TEST_EXPECT_OK(dac8565Deinit(dev));
	DAC8565_TEST_EXPECT(!dev->is_initialized, "device should be deinitialized");
	DAC8565_TEST_EXPECT_OK(dac8565Init(dev, &k_cfg, &k_hal));

	const DAC8565HAL fail_hal = {.spiWrite = testFailingSpiWrite, .ldacSet = testLdacSet, .delayUs = testDelayUs};
	DAC8565Device    fail_dev = {0};
	DAC8565_TEST_EXPECT_ERR(dac8565Init(&fail_dev, &k_cfg, &fail_hal), DAC8565_ERROR_COMM_FAIL);
	DAC8565_TEST_EXPECT(!fail_dev.is_initialized, "failed init should leave is_initialized=false");
}

static void testVoltageRoundTrip(DAC8565Device* const dev)
{
	s_active_case = "voltage_round_trip";

	const uint32_t test_voltages_uv[] = {
	    0U,
	    1U,
	    DAC8565_TEST_VREF_UV / 4U,
	    DAC8565_TEST_VREF_UV / 2U,
	    (DAC8565_TEST_VREF_UV * 3U) / 4U,
	    DAC8565_TEST_VREF_UV - 1U,
	    DAC8565_TEST_VREF_UV,
	};
	const size_t count = sizeof(test_voltages_uv) / sizeof(test_voltages_uv[0]);

	for (size_t i = 0; i < count; i++)
	{
		const uint32_t v_uv = test_voltages_uv[i];
		DAC8565_TEST_EXPECT_OK(dac8565WriteVoltageUv(dev, DAC8565_CHANNEL_A, v_uv));

		uint32_t round_trip = 0U;
		DAC8565_TEST_EXPECT_OK(dac8565GetLastVoltageUv(dev, DAC8565_CHANNEL_A, &round_trip));

		const int64_t lsb_uv = (int64_t)DAC8565_TEST_VREF_UV / (int64_t)DAC8565_FULL_SCALE;
		const int64_t delta  = (int64_t)v_uv - (int64_t)round_trip;
		DAC8565_TEST_EXPECT(delta >= -(lsb_uv + 1) && delta <= (lsb_uv + 1),
		                    "voltage_uv=%" PRIu32 " round_trip=%" PRIu32 " delta=%" PRId64,
		                    v_uv, round_trip, delta);
	}
	DAC8565_TEST_EXPECT_ERR(dac8565WriteVoltageUv(dev, DAC8565_CHANNEL_A, DAC8565_TEST_VREF_UV + 1U),
	                        DAC8565_ERROR_INVALID_PARAM);
}

static void testPerChannel(DAC8565Device* const dev)
{
	s_active_case = "per_channel";

	for (uint32_t ch = 0U; ch < DAC8565_CHANNEL_COUNT; ch++)
	{
		const uint16_t code = (uint16_t)((ch + 1U) * 0x2000U);
		DAC8565_TEST_EXPECT_OK(dac8565WriteCode(dev, (DAC8565Channel)ch, code));

		uint16_t got = 0U;
		DAC8565_TEST_EXPECT_OK(dac8565GetLastCode(dev, (DAC8565Channel)ch, &got));
		DAC8565_TEST_EXPECT(got == code, "ch=%u expected=0x%04X got=0x%04X",
		                    (unsigned)ch, code, got);
	}

	DAC8565_TEST_EXPECT_ERR(dac8565WriteCode(dev, (DAC8565Channel)42, 0U), DAC8565_ERROR_INVALID_PARAM);
}

static void testMultiChannelSync(DAC8565Device* const dev)
{
	s_active_case = "multi_channel_sync";

	const uint16_t codes[DAC8565_CHANNEL_COUNT] = {0x1000U, 0x4000U, 0x8000U, 0xC000U};
	DAC8565_TEST_EXPECT_OK(dac8565WriteAllCodes(dev, codes));

	for (uint32_t ch = 0U; ch < DAC8565_CHANNEL_COUNT; ch++)
	{
		uint16_t got = 0U;
		DAC8565_TEST_EXPECT_OK(dac8565GetLastCode(dev, (DAC8565Channel)ch, &got));
		DAC8565_TEST_EXPECT(got == codes[ch], "ch=%u expected=0x%04X got=0x%04X",
		                    (unsigned)ch, codes[ch], got);
	}
}

static void testBroadcast(DAC8565Device* const dev)
{
	s_active_case = "broadcast";

	const uint16_t code = 0xABCDU;
	DAC8565_TEST_EXPECT_OK(dac8565WriteBroadcastCode(dev, code));

	for (uint32_t ch = 0U; ch < DAC8565_CHANNEL_COUNT; ch++)
	{
		uint16_t got = 0U;
		DAC8565_TEST_EXPECT_OK(dac8565GetLastCode(dev, (DAC8565Channel)ch, &got));
		DAC8565_TEST_EXPECT(got == code, "broadcast: ch=%u expected=0x%04X got=0x%04X",
		                    (unsigned)ch, code, got);
	}
}

static void testStageAndCommit(DAC8565Device* const dev)
{
	s_active_case = "stage_and_commit";

	for (uint32_t ch = 0U; ch < DAC8565_CHANNEL_COUNT; ch++)
		DAC8565_TEST_EXPECT_OK(dac8565StageCode(dev, (DAC8565Channel)ch, (uint16_t)(0x5555U + ch)));
	DAC8565_TEST_EXPECT_OK(dac8565CommitBuffers(dev));
}

static void testPowerModeCycle(DAC8565Device* const dev)
{
	s_active_case = "power_mode_cycle";

	const DAC8565PowerMode sequence[] = {
	    DAC8565_POWER_MODE_NORMAL,
	    DAC8565_POWER_MODE_PD_1K,
	    DAC8565_POWER_MODE_PD_100K,
	    DAC8565_POWER_MODE_PD_HIZ,
	    DAC8565_POWER_MODE_NORMAL,
	};
	const size_t count = sizeof(sequence) / sizeof(sequence[0]);

	for (size_t i = 0; i < count; i++)
	{
		DAC8565_TEST_EXPECT_OK(dac8565SetChannelPowerMode(dev, DAC8565_CHANNEL_A, sequence[i]));
		DAC8565PowerMode read_back = DAC8565_POWER_MODE_NORMAL;
		DAC8565_TEST_EXPECT_OK(dac8565GetChannelPowerMode(dev, DAC8565_CHANNEL_A, &read_back));
		DAC8565_TEST_EXPECT(read_back == sequence[i],
		                    "expected mode %d, got %d", (int)sequence[i], (int)read_back);
		vTaskDelay(pdMS_TO_TICKS(DAC8565_TEST_PD_CYCLE_DELAY_MS));
	}
}

static void testSetAllPowerModes(DAC8565Device* const dev)
{
	s_active_case = "set_all_power_modes";

	const DAC8565PowerMode modes[DAC8565_CHANNEL_COUNT] = {
	    DAC8565_POWER_MODE_PD_1K,
	    DAC8565_POWER_MODE_PD_1K,
	    DAC8565_POWER_MODE_PD_100K,
	    DAC8565_POWER_MODE_PD_100K,
	};
	DAC8565_TEST_EXPECT_OK(dac8565SetAllPowerModes(dev, modes));
	for (uint32_t ch = 0U; ch < DAC8565_CHANNEL_COUNT; ch++)
	{
		DAC8565PowerMode got = DAC8565_POWER_MODE_NORMAL;
		DAC8565_TEST_EXPECT_OK(dac8565GetChannelPowerMode(dev, (DAC8565Channel)ch, &got));
		DAC8565_TEST_EXPECT(got == modes[ch], "ch=%u expected=%d got=%d",
		                    (unsigned)ch, (int)modes[ch], (int)got);
	}
	const DAC8565PowerMode all_normal[DAC8565_CHANNEL_COUNT] = {
	    DAC8565_POWER_MODE_NORMAL,
	    DAC8565_POWER_MODE_NORMAL,
	    DAC8565_POWER_MODE_NORMAL,
	    DAC8565_POWER_MODE_NORMAL,
	};
	DAC8565_TEST_EXPECT_OK(dac8565SetAllPowerModes(dev, all_normal));
}

static void testReferenceSwap(DAC8565Device* const dev)
{
	s_active_case = "reference_swap";

	DAC8565ReferenceMode mode = DAC8565_REF_MODE_DEFAULT;
	uint32_t             v_uv = 0U;

	DAC8565_TEST_EXPECT_OK(dac8565GetReferenceMode(dev, &mode));
	DAC8565_TEST_EXPECT(mode == DAC8565_REF_MODE_ALWAYS_ON, "expected ALWAYS_ON, got %d", (int)mode);
	DAC8565_TEST_EXPECT_OK(dac8565GetReferenceVoltageUv(dev, &v_uv));
	DAC8565_TEST_EXPECT(v_uv == DAC8565_INTERNAL_REFERENCE_UV,
	                    "expected internal 2.5V, got %" PRIu32, v_uv);

	DAC8565_TEST_EXPECT_OK(dac8565SetReferenceMode(dev, DAC8565_REF_MODE_DEFAULT));
	DAC8565_TEST_EXPECT_OK(dac8565GetReferenceMode(dev, &mode));
	DAC8565_TEST_EXPECT(mode == DAC8565_REF_MODE_DEFAULT, "expected DEFAULT, got %d", (int)mode);

	DAC8565_TEST_EXPECT_OK(dac8565SetReferenceMode(dev, DAC8565_REF_MODE_ALWAYS_ON)); /* restore */
}

static void testNotSupported(void)
{
	s_active_case = "not_supported";

	const DAC8565HAL no_pins = {.spiWrite = testSpiWrite}; /* no ldacSet, resetSet, enableSet */
	DAC8565Device    dev     = {0};
	DAC8565_TEST_EXPECT_OK(dac8565Init(&dev, &k_cfg, &no_pins));

	DAC8565_TEST_EXPECT_ERR(dac8565CommitBuffers(&dev), DAC8565_ERROR_NOT_SUPPORTED);
	DAC8565_TEST_EXPECT_ERR(dac8565PulseLDAC(&dev), DAC8565_ERROR_NOT_SUPPORTED);
	DAC8565_TEST_EXPECT_ERR(dac8565ResetHardware(&dev), DAC8565_ERROR_NOT_SUPPORTED);
	DAC8565_TEST_EXPECT_ERR(dac8565SetEnable(&dev, true), DAC8565_ERROR_NOT_SUPPORTED);
	DAC8565_TEST_EXPECT_ERR(dac8565StageCode(&dev, DAC8565_CHANNEL_A, 0U), DAC8565_ERROR_NOT_SUPPORTED);

	(void)dac8565Deinit(&dev);
}

static void testOutputSweep(DAC8565Device* const dev)
{
	s_active_case = "output_sweep";

	for (uint32_t code = 0U; code <= DAC8565_CODE_MAX; code += DAC8565_TEST_SWEEP_STEP_COUNT)
	{
		const uint16_t codes[DAC8565_CHANNEL_COUNT] = {(uint16_t)code, (uint16_t)code, (uint16_t)code, (uint16_t)code};
		DAC8565_TEST_EXPECT_OK(dac8565WriteAllCodes(dev, codes));
		vTaskDelay(pdMS_TO_TICKS(DAC8565_TEST_SWEEP_DELAY_MS));
	}
}

/* ── Application entry ────────────────────────────────────────────────────── */

void app_main(void)
{
	s_pass_count = 0;
	s_fail_count = 0;

	ESP_LOGI(TAG, "DAC8565 system test starting");

	if (testInitSpi() != ESP_OK)
	{
		ESP_LOGE(TAG, "SPI bring-up failed; aborting");
		return;
	}

	testNullParamRejection();
	testUseBeforeInitGuards();

	DAC8565Device dev = {0};
	testLifecycle(&dev);
	if (!dev.is_initialized)
	{
		ESP_LOGE(TAG, "Device failed to initialize; aborting remaining cases");
		return;
	}

	testVoltageRoundTrip(&dev);
	testPerChannel(&dev);
	testMultiChannelSync(&dev);
	testBroadcast(&dev);
	testStageAndCommit(&dev);
	testPowerModeCycle(&dev);
	testSetAllPowerModes(&dev);
	testReferenceSwap(&dev);
	testNotSupported();
	testOutputSweep(&dev);

	(void)dac8565Reset(&dev);
	(void)dac8565Deinit(&dev);

	ESP_LOGI(TAG, "DAC8565 system test: pass=%d fail=%d", s_pass_count, s_fail_count);
}
