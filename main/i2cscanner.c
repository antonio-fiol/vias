#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "config_vias.h"

static char tag[] = "i2cscanner";

uint16_t i2cscan(uint16_t start) {
	ESP_LOGD(tag, ">> i2cScanner");
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = I2C_SCANNER_SDA_IO;
	conf.scl_io_num = I2C_SCANNER_SCL_IO;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	i2c_param_config(I2C_SCANNER_NUM, &conf);

	i2c_driver_install(I2C_SCANNER_NUM, I2C_MODE_MASTER, 0, 0, 0);

	uint16_t i,ret=0;
	esp_err_t espRc;
	printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
	printf("00:         ");
	for (i=(start>3?3:start); i< 0x78; i++) {
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
		i2c_master_stop(cmd);

		espRc = i2c_master_cmd_begin(I2C_SCANNER_NUM, cmd, 10/portTICK_PERIOD_MS);
		if (i%16 == 0) {
			printf("\n%.2x:", i);
		}
                /*
                        ESP_OK Success
                        ESP_ERR_INVALID_ARG Parameter error
                        ESP_FAIL Sending command error, slave doesn’t ACK the transfer.
                        ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
                        ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
                */
		switch(espRc) {
                case ESP_OK:
			printf(" %.2x", i); break;
		case ESP_ERR_INVALID_ARG:
			printf(" IA"); break;
		case ESP_FAIL:
			if(i>=start && ret==0) {
				printf(" **");
				ret=i;
			} else {
				printf(" --");
			}
			break;
		case ESP_ERR_INVALID_STATE:
			printf(" IS"); break;
		case ESP_ERR_TIMEOUT:
			printf(" TO"); break;
		default:
			printf(" XX"); break;
		}
		//ESP_LOGD(tag, "i=%d, rc=%d (0x%x)", i, espRc, espRc);
		i2c_cmd_link_delete(cmd);
	}
	printf("\n");
        i2c_driver_delete(I2C_SCANNER_NUM);
	return ret;
}
void task_i2cscanner(void *ignore) {
	i2cscan(0x10);
	vTaskDelete(NULL);
}
