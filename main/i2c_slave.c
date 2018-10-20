#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include <driver/adc.h>
#include "esp_err.h"
#include "config_vias.h"
#include "generic_i2c_rw.h"
#include "math.h"
#include "types_vias.h"

void do_ota();
void initialise_wifi();

SemaphoreHandle_t print_mux = NULL;
static TaskHandle_t i2c_slave_task_handle = NULL;
static i2c_slave_state_t global_slave_state;


    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */
    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
        {
            .channel    = LEDC_CHANNEL_0,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH0_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_CHANNEL_1,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH1_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_CHANNEL_2,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH2_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_CHANNEL_3,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH3_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_CHANNEL_4,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH4_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_CHANNEL_5,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH5_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_CHANNEL_6,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH6_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_CHANNEL_7,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH7_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .timer_sel  = LEDC_HS_TIMER
        },
    };
    int dir_channel[LEDC_TEST_CH_NUM] = { CH0_DIR, CH1_DIR, CH2_DIR, CH3_DIR, CH4_DIR, CH5_DIR, CH6_DIR, CH7_DIR };

static void led_init()
{
    int ch;

    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_12_BIT, // resolution of PWM duty
        .freq_hz = 60,                        // frequency of PWM signal
        .speed_mode = LEDC_HS_MODE,           // timer mode
        .timer_num = LEDC_HS_TIMER            // timer index
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    // Set LED Controller with previously prepared configuration
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }
    gpio_config_t gpio = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE 
    };
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        gpio.pin_bit_mask |= (1ULL<<dir_channel[ch]);
    }
    gpio_config(&gpio);
}

static void set_digital(int ch)
{
                                xSemaphoreTake(print_mux, portMAX_DELAY);
                                printf("====Ch: %d digital ====\n", ch);
                                xSemaphoreGive(print_mux);

    // TODO: Digital
}

static void reset_digital(int ch)
{
                                xSemaphoreTake(print_mux, portMAX_DELAY);
                                printf("====Ch: %d analog ====\n", ch);
                                xSemaphoreGive(print_mux);

    // TODO: Digital
}

static void set_frequency(int freq_hz)
{
                            xSemaphoreTake(print_mux, portMAX_DELAY);
                            printf("==== SET_FREQ f: %d ====\n", freq_hz);
                            xSemaphoreGive(print_mux);

   if (freq_hz > 3 && freq_hz < 1000) {
       // TODO: Frequency
   }
}


/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t* buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if (( i + 1 ) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}
void i2c_slave_ready();

/**
 * @brief i2c slave initialization
 */
static void i2c_slave_init(uint16_t addr)
{
    int i2c_slave_port = I2C_SLAVE_NUM;
    i2c_config_t conf_slave;
    conf_slave.sda_io_num = I2C_SLAVE_SDA_IO;
    conf_slave.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf_slave.scl_io_num = I2C_SLAVE_SCL_IO;
    conf_slave.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf_slave.mode = I2C_MODE_SLAVE;
    conf_slave.slave.addr_10bit_en = 0;
    conf_slave.slave.slave_addr = addr;
    i2c_param_config(i2c_slave_port, &conf_slave);
printf("-----");
    esp_err_t out = i2c_driver_install(i2c_slave_port, conf_slave.mode,
                                       I2C_SLAVE_BUF_LEN,
                                       I2C_SLAVE_RO_LEN, 0);
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("==== slave@%02x: i2c_driver_install-->%d ====\n", addr, out);
    xSemaphoreGive(print_mux);
    i2c_slave_ready();
}
void set_polarity(int ch, bool neg) {
    gpio_set_level(dir_channel[ch], !neg);
}
void set_duty(int ch, int16_t duty) {
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("= Ch: %d duty: %d\n", ch, duty);
    xSemaphoreGive(print_mux);

    bool neg = (duty<0);
    if(neg) duty*=(-1);
    set_polarity(ch, neg);
    ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, duty);
    ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
}

//uint8_t* get_presence();                                                       
//int* get_readings();
//void read_presence(uint8_t* out);
//void read_readings(int adc, int* out);


//void ret_readings(int ch) {
//    int* r = get_readings(ch);
//    disp_buf((uint8_t*)r, 8);
//    i2c_slave_write_buffer(I2C_SLAVE_NUM, (uint8_t*)r, 8, 100 / portTICK_RATE_MS);
//}
//void ret_val(uint8_t* p) {
//    disp_buf(p, 2);
//    i2c_slave_write_buffer(I2C_SLAVE_NUM, p, 2, 100 / portTICK_RATE_MS);
//}
//void ret_presence() {
//    uint8_t* p = get_presence();
//    ret_val(p);
//}
int cnt=0;
static void process_data(int size_in, uint8_t* data_in)
{
	uint8_t* data = data_in;
        int size = size_in;
        bool data_returned = false;
        while (size > 0 && size <= 14 && !data_returned) {
            if(size==size_in) {
                xSemaphoreTake(print_mux, portMAX_DELAY);
                //printf("====TASK[%d] Slave packet %d size: %d ====\n",  cnt++, size);
                disp_buf(data, size);
                xSemaphoreGive(print_mux);
            }
            int ch = data[0];
            int processed = 1;
            if (ch < LEDC_TEST_CH_NUM) {
                if(size>=3) {
                    int16_t duty = (int16_t)(((uint16_t)data[2])<<8 | data[1]);
                    set_duty( ch, duty);
                    processed += 2;
                }
            } else {
                switch (ch) {
                    case CH_RESET:
                        xSemaphoreTake(print_mux, portMAX_DELAY);
                        printf("==== Reset timer ====\n");
                        xSemaphoreGive(print_mux);
                        ledc_timer_rst(LEDC_HS_MODE, LEDC_HS_TIMER);
                    break;
                    case CH_SET_DIGITAL:
                        if (size>=2) {
                            ch = data[1];
                            if (ch < LEDC_TEST_CH_NUM) {
                                set_digital( ch);
                            }
	                    processed += 1;
                        }
                    break;
                    case CH_RESET_DIGITAL:
                        if (size>=2) {
                            ch = data[1];
                            if (ch < LEDC_TEST_CH_NUM) {
                                reset_digital( ch);
                            }
                            processed += 1;
                        }
                    break;
                    case CH_SET_FREQ:
                        if (size>=3) {
                            set_frequency( ((uint16_t)data[2])<<8 | data[1]);
                            processed += 2;
                        }
                    break;
//                    case CH_READINGS:
//                        if (size>=2) {
//                            ch = data[1];
//                            xSemaphoreTake(print_mux, portMAX_DELAY);
//                            printf("==== READINGS %d ====\n",  ch);
//                            xSemaphoreGive(print_mux);
//                            ret_readings(ch);
//                            processed += 1;
//                            data_returned = true;
//                        }
//                    break;
//                    case CH_PRESENCE:
//                            xSemaphoreTake(print_mux, portMAX_DELAY);
//                            printf("==== PRESENCE ====\n");
//                            xSemaphoreGive(print_mux);
//                            ret_presence();
//                            data_returned = true;
//                    break;
                    case CH_OTA:
                            xSemaphoreTake(print_mux, portMAX_DELAY);
                            printf("==== OTA ====\n");
                            xSemaphoreGive(print_mux);
                            do_ota();
                    break;
//                    case CH_ADC_CONFIG:
//                            {
//                            int val = adc1_get_raw(CONFIG_ADC_CHANNEL);
//                            xSemaphoreTake(print_mux, portMAX_DELAY);
//                            printf("==== ADC_CONFIG %d ====\n",  val);
//                            xSemaphoreGive(print_mux);
//                            ret_val((uint8_t*)&val);
//                            data_returned = true;
//                            }
//                    break;
                }
            }
            size -= processed;
            data += processed;
        }
        if (size > 14) {
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("==== size too big: %d\n",  size);

            xSemaphoreGive(print_mux);
        }
}

//static void i2c_slave_task(void* arg)
//{
//    uint32_t task_idx = (uint32_t) arg;
//    uint8_t* data = (uint8_t*) malloc(DATA_LENGTH);
//
//    while (1) {
//        int size = i2c_slave_read_buffer( I2C_SLAVE_NUM, data, DATA_LENGTH, 0 / portTICK_RATE_MS); // 0-->return immediately if no data??
//        process_data(task_idx, size, data);
//        vTaskDelay(10 / portTICK_RATE_MS);
//    }
//}
static void i2c_slave_task(void* arg)
{
    uint32_t notify_val = 0;
    int notify_res = 0;
    int rdlen;
    uint8_t *data;
    i2c_slave_state_t slave_state;

    printf("[SLV_TASK] Started\n");
    if (i2c_slave_add_task(I2C_SLAVE_NUM, &i2c_slave_task_handle, &global_slave_state) != ESP_OK) {
        printf("[SLV_TASK] Error registering slave task\n");
        goto exit;
    }

    while (1) {
        // === Wait for notification from I2C interrupt routine ===
        notify_val = 0;
        notify_res = xTaskNotifyWait(0, ULONG_MAX, &notify_val, 1000 / portTICK_RATE_MS);
        if (notify_res != pdPASS) continue;

        // notification received
        // Check if task exit requested
        if (notify_val == I2C_SLAVE_DRIVER_DELETED) {
            printf("[SLV_TASK] i2c driver deleted\n");
            break;
        }

        slave_state = global_slave_state;
        data = NULL;
        rdlen = 0;
        if ((slave_state.rxptr == 0) && (slave_state.txptr == 0)) {
            // address received from master
            printf("[SLV_TASK] Addr: %d\n", slave_state.rxaddr);
        }
        else if (slave_state.status & 0x02) {
            // read transaction, data sent to master
            if (slave_state.txptr) {
                data = malloc(slave_state.txptr+1);
                if (data) {
                    rdlen = i2c_slave_read_buffer(I2C_SLAVE_NUM, data, slave_state.txaddr, slave_state.txptr, 200 / portTICK_RATE_MS);
                    data[slave_state.txptr] = 0;
                    if (rdlen != slave_state.txptr) {
                        free(data);
                        data = NULL;
                    }
                }
            }
            printf("[SLV_TASK] To master: addr=%d, len=%d, ovf=%d\n", slave_state.txaddr, slave_state.txptr, slave_state.txovf);
            if (data) {
                disp_buf(data, slave_state.txptr);
                free(data);
            }
            else printf(" [No data]\n");
        }
        else {
            // write transaction, data received from master
            /*
            if (slave_state.rxptr) {
                data = malloc(slave_state.rxptr+1);
                if (data) {
                    rdlen = i2c_slave_read_buffer(I2C_SLAVE_NUM, data, slave_state.rxaddr, slave_state.rxptr, 200 / portTICK_RATE_MS);
                    if (rdlen != slave_state.rxptr) {
                        free(data);
                        data = NULL;
                    }
                    data[slave_state.rxptr] = 0;
                }
            }
            printf("[SLV_TASK] From master: addr=%d, len=%d, ovf=%d\n", slave_state.rxaddr, slave_state.rxptr, slave_state.rxovf);
            if (data) {
                disp_buf(data, slave_state.rxptr);
                free(data);
            }
            else printf(" [No data]\n");
            */
            {
            int ch = slave_state.rxaddr >> 1;
            int base = ch << 1;
            uint8_t data[2];
            rdlen = i2c_slave_read_buffer(I2C_SLAVE_NUM, data, base, 2, 200 / portTICK_RATE_MS);
            if(rdlen==2) {
               uint16_t data16 = (int16_t)(((uint16_t)data[1])<<8 | data[0]);
               switch(base) {
                   case CH_SET_FREQ:
                       set_frequency(data16);
                       break;
                   case CH_SET_DIGITAL:
                       set_digital(ch);
                       break;
                   case CH_RESET_DIGITAL:
                       reset_digital(ch);
                       break;
                   case CH_OTA:
                       xSemaphoreTake(print_mux, portMAX_DELAY);
                       printf("==== OTA ====\n");
                       xSemaphoreGive(print_mux);
                       do_ota();
                       break;
                   case CH_RESET:
                       xSemaphoreTake(print_mux, portMAX_DELAY);
                       printf("==== Reset timer ====\n");
                       xSemaphoreGive(print_mux);
                       ledc_timer_rst(LEDC_HS_MODE, LEDC_HS_TIMER);
                       break;
                   default:
                       if (ch < LEDC_TEST_CH_NUM) {
                           set_duty(ch, data16);
                       }
                       break;
               }
            }
            }
        }
    }

exit:
    i2c_slave_remove_task(I2C_SLAVE_NUM);
    printf("[SLV_TASK] Deleted\n");
    i2c_slave_task_handle = NULL;
    vTaskDelete(NULL);
}

static void demo_task(void* arg)
{
    int sign=-1;
    while (1) {
        sign=-sign;
        for(uint16_t duty = 1000; duty < 3500; duty+=200) {
           for(int ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
               set_duty( ch, duty*sign);
           }
           vTaskDelay(1000 / portTICK_RATE_MS);
        }
        for(int ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
           set_duty( ch, 2);
        }
        vTaskDelay(5000 / portTICK_RATE_MS);
    }

}


static void i2c_fast_blink_task(void* arg)
{
    uint32_t task_idx = (uint32_t) arg;
    int cntblink = 0;
    long t = xTaskGetTickCount();

    while (1) {
        cntblink++;
        set_polarity(0,0);
            taskYIELD();
        if(cntblink==1000) {
            long tt = xTaskGetTickCount();

            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("====TASK[%d] %ld ms for 1000 iter\n", task_idx, (tt-t) * portTICK_PERIOD_MS);
            xSemaphoreGive(print_mux);
            cntblink=0;
            t = xTaskGetTickCount();
        }
    }
}
int val_to_delta(int val) {
    float f_delta = (CONFIG_DIR0_VAL - val) / (float)CONFIG_DELTA_VAL;
    int i_delta = round( f_delta );
    printf("val_to_delta(%d) f_delta=%f i_delta=%d\n", val, f_delta, i_delta);
    return i_delta;
}
report_t report;
int config_delta()
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(CONFIG_ADC_CHANNEL,ADC_ATTEN_DB_11);
    int avg = 0;
    for(int n=0; n<8; n++) {
        int val = adc1_get_raw(CONFIG_ADC_CHANNEL);
        printf("%d\n",val);
        avg+=val;
        vTaskDelay(100 / portTICK_RATE_MS);
    }
    avg = avg >> 3;
    report.val = avg;
    return val_to_delta(avg);
}

uint16_t i2cscan(uint16_t addr);

void ads_task(void *args);
void report_address_task(void *args);

void start_i2c_slave(int addr)
{
    // Do an initialization of the i2c driver
    i2c_slave_init(addr);
    // Create the slave task
    xTaskCreate(i2c_slave_task, "i2c_slave_task", 1024 * 2, (void* ) 0, 12, &i2c_slave_task_handle);
}
void app_main()
{
    printf("app_main: enter\n");
    generic_i2c_master_init (I2C_MASTER_NUM, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, I2C_MASTER_FREQ_HZ);
    printf("app_main: generic_i2c_master_init done\n");
    led_init();
    printf("app_main: led_init done\n");

    print_mux = xSemaphoreCreateMutex();

    int delta = config_delta();

    printf("app_main: delta done\n");
    initialise_wifi();
    printf("app_main: initialise_wifi done\n");

    uint16_t addr = I2C_SLAVE_START_ADDRESS + delta;
    report.addr = addr;
    xTaskCreate(ads_task, "ads_task_1", 1024 * 2, (void* ) 1, 10, NULL);
//    xTaskCreate(i2c_fast_blink_task, "i2c_fast_blink_task_1", 1024 * 2, (void* ) 1, 10, NULL);
    //xTaskCreate(demo_task, "demo", 1024 * 2, (void* ) 2, 10, NULL);
    xTaskCreate(report_address_task, "report_task", 1024 * 8, (void* ) &report, 10, NULL);
}

