#include "one_wire.h"

#include <inttypes.h>
#include <stdio.h>

#include <stm32f4xx_hal.h>


void print_1w_slave_rom(void)
{
	int reset_status = onewire_reset();

	uint8_t rom[8] = { 0 };
	onewire_read_rom(rom);

	printf("reset status = %d, rom == ", reset_status);

	for (int i = 0; i < 8; i++)
		printf("%02x", (int)rom[i]);

	printf("\n");
}


int app_main(void)
{
	// Инициализируем шину
	onewire_init();

	// Напечатаем ROM адрес этого ведомого
	print_1w_slave_rom();

	// Настроим датчик
	int rc = ds18b20_set_config(0, 0, DS18B20_RESOLUTION_12_BIT);
	printf("set config status = %d\n", rc);

	// Будем в цикле начинать замеры и запрашивать их результаты
	while(1)
	{
		// Начинаем замер
		int rc = ds18b20_start_conversion();
		printf("start conversion status = %d\n", rc);

		// ждем секунду пока он завершится
		// так то он завершится должен за 750 мс, но мы подождем чтоб точно
		HAL_Delay(1000);

		// Читаем температуру и пишем в консоль
		uint16_t raw_temperature;
		bool crc_ok;
		rc = ds18b20_read_raw_temperature(&raw_temperature, &crc_ok);
		printf(
				"read temperature status = %d, crc_ok = %d, raw_value = %d, value = %f\n",
				rc,
				(int)crc_ok,
				(int)raw_temperature,
				raw_temperature / 16.f
		);
	}

	return 0;
}
