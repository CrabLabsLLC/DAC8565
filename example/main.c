/**
 * @file main.c
 * @brief Minimal ESP-IDF example: ramp all four DAC8565 channels in sync.
 *
 * @author Orion Serup <orion@crablabs.io>
 *
 * @reviewer TBD <reviewer@crablabs.io>
 */

#include "dac8565.h"

#include <stdint.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define EXAMPLE_PIN_MOSI GPIO_NUM_34
#define EXAMPLE_PIN_MISO GPIO_NUM_33
#define EXAMPLE_PIN_SCLK GPIO_NUM_35
#define EXAMPLE_PIN_SYNC GPIO_NUM_45
#define EXAMPLE_PIN_LDAC GPIO_NUM_21
#define EXAMPLE_VREF_UV  2500000U /* internal 2.5 V reference */
#define EXAMPLE_SPI_HZ   1000000

static spi_device_handle_t s_spi = NULL;

static int exampleSpiWrite(const void* const data, const uint8_t length)
{
	const spi_transaction_t tx = {
	    .length    = (size_t)length * 8U,
	    .tx_buffer = data,
	};
	return (spi_device_transmit(s_spi, &tx) == ESP_OK) ? 0 : -1;
}

static void exampleLdacSet(bool level)
{
	gpio_set_level(EXAMPLE_PIN_LDAC, level ? 1 : 0);
}

static void exampleDelayUs(uint32_t delay_us)
{
	esp_rom_delay_us(delay_us);
}

void app_main(void)
{
	gpio_reset_pin(EXAMPLE_PIN_LDAC);
	gpio_set_direction(EXAMPLE_PIN_LDAC, GPIO_MODE_OUTPUT);
	gpio_set_level(EXAMPLE_PIN_LDAC, 0);

	const spi_bus_config_t bus_cfg = {
	    .mosi_io_num     = EXAMPLE_PIN_MOSI,
	    .miso_io_num     = EXAMPLE_PIN_MISO,
	    .sclk_io_num     = EXAMPLE_PIN_SCLK,
	    .quadwp_io_num   = -1,
	    .quadhd_io_num   = -1,
	    .max_transfer_sz = 32,
	};
	spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);

	const spi_device_interface_config_t dev_cfg = {
	    .clock_speed_hz = EXAMPLE_SPI_HZ,
	    .mode           = 1,
	    .spics_io_num   = EXAMPLE_PIN_SYNC,
	    .queue_size     = 1,
	};
	spi_bus_add_device(SPI2_HOST, &dev_cfg, &s_spi);

	const DAC8565HAL hal = {
	    .spiWrite = exampleSpiWrite,
	    .ldacSet  = exampleLdacSet,
	    .delayUs  = exampleDelayUs,
	};
	const DAC8565Config config = {
	    .encoding                      = DAC8565_ENCODING_BINARY,
	    .initial_reference_mode        = DAC8565_REF_MODE_ALWAYS_ON,
	    .external_reference_voltage_uv = 0U,
	};
	DAC8565Device dev;

	if (dac8565Init(&dev, &config, &hal) != DAC8565_ERROR_OK)
		return;

	while (1)
	{
		for (uint32_t mv = 0U; mv < EXAMPLE_VREF_UV / 1000U; mv += 100U)
		{
			const uint32_t voltages_uv[DAC8565_CHANNEL_COUNT] = {
			    mv * 1000U,
			    mv * 1000U,
			    mv * 1000U,
			    mv * 1000U,
			};
			dac8565WriteAllVoltagesUv(&dev, voltages_uv);
			vTaskDelay(pdMS_TO_TICKS(50));
		}
	}
}
