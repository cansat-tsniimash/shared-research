#include <stdio.h>

#include <stm32f4xx_hal.h>

#include "bme280.h"


// Простой проект с BME280 на SPI
/* К сожалению проект не на black pill, а на большую черную плату с f405ve.
 * Что в целом не страшно. Все настройки идентичные.
 *
 * В этом проекте были проблемы с зависанием датчика. Поэтому если встретите их - сообщите
 */


static void bme_spi_cs_down(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
}


static void bme_spi_cs_up(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
}



static BME280_INTF_RET_TYPE bme_spi_read(
		uint8_t reg_addr, uint8_t * data, uint32_t data_len, void *intf_ptr
)
{
	extern SPI_HandleTypeDef hspi1;

	bme_spi_cs_down();
	reg_addr |= (1 << 7);
	HAL_SPI_Transmit(&hspi1, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, data, data_len, HAL_MAX_DELAY);
	bme_spi_cs_up();

	return 0;
}


static BME280_INTF_RET_TYPE bme_spi_write(
		uint8_t reg_addr, const uint8_t * data, uint32_t data_len, void *intf_ptr
)
{
	extern SPI_HandleTypeDef hspi1;

	bme_spi_cs_down();
	reg_addr &= ~(1 << 7);
	HAL_SPI_Transmit(&hspi1, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)data, data_len, HAL_MAX_DELAY);
	bme_spi_cs_up();

	return 0;
}


static void bme_delay_us(uint32_t period, void *intf_ptr)
{
	if (period < 1000)
		period = 1;
	else
		period = period / 1000;

	HAL_Delay(period);
}



int app_main(void)
{
	// Настройка bme280 =-=-=-=-=-=-=-=-=-=-=-=-
	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	struct bme280_dev bme = {0};
	bme.intf = BME280_SPI_INTF;
	bme.intf_ptr = NULL;
	bme.read = bme_spi_read;
	bme.write = bme_spi_write;
	bme.delay_us = bme_delay_us;

	int rc = bme280_soft_reset(&bme);
	printf("bme280 reset rc = %d\n", (int)rc);

	rc = bme280_init(&bme);
	printf("bme280 init rc = %d\n", (int)rc);

	bme.settings.osr_h = BME280_OVERSAMPLING_1X;
	bme.settings.osr_p = BME280_OVERSAMPLING_16X;
	bme.settings.osr_t = BME280_OVERSAMPLING_2X;
	bme.settings.filter = BME280_FILTER_COEFF_16;
	bme.settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

	uint8_t settings_sel;
	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_FILTER_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	rc = bme280_set_sensor_settings(settings_sel, &bme);
	printf("bme280 settings set rc = %d\n", rc);
	rc = bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme);
	printf("bme280 set sensor mode rc = %d\n", rc);

	while(1)
	{
		// Чтение данные из bme280
		// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

		HAL_Delay(10);

		struct bme280_data comp_data;
		rc = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme);

		// Печать
		printf(
			"temp = %8.4f; pressure = %10.4f; hum = %10.4f\n",
			(float)comp_data.temperature,
			(float)comp_data.pressure,
			(float)comp_data.humidity
		);

		//HAL_Delay(100);
	}

	return 0;
}
