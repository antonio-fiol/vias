//#define SIG_EN1 5
#define SIG_EN1 17
#define SIG_EN2 21
#define SIG_EN3 2
#define SIG_EN4 16
#define SIG_EN5 26
#define SIG_EN6 23
#define SIG_EN7 13
//#define SIG_EN8 14
#define SIG_EN8 12
//#define SIG_F1 17
#define SIG_F1 5
#define SIG_F2 19
#define SIG_F3 15
#define SIG_F4 4
#define SIG_F5 27
#define SIG_F6 25
#define SIG_F7 22
//#define SIG_F8 12
#define SIG_F8 14

#define CONFIG_ADC_CHANNEL ADC1_GPIO35_CHANNEL
#define CONFIG_DIR0_VAL 2608
#define CONFIG_DELTA_VAL 78

/* GPIO outputs for DIR signal */
#define CH0_DIR    (SIG_F3)
#define CH1_DIR    (SIG_F4)
#define CH2_DIR    (SIG_F1)
#define CH3_DIR    (SIG_F2)
#define CH4_DIR    (SIG_F7)
#define CH5_DIR    (SIG_F8)
#define CH6_DIR    (SIG_F5)
#define CH7_DIR    (SIG_F6)


/* GPIO outputs for PWM EN signal */
#define LEDC_HS_CH0_GPIO       (SIG_EN3)
#define LEDC_HS_CH1_GPIO       (SIG_EN4)
#define LEDC_HS_CH2_GPIO       (SIG_EN1)
#define LEDC_HS_CH3_GPIO       (SIG_EN2)
#define LEDC_HS_CH4_GPIO       (SIG_EN7)
#define LEDC_HS_CH5_GPIO       (SIG_EN8)
#define LEDC_HS_CH6_GPIO       (SIG_EN5)
#define LEDC_HS_CH7_GPIO       (SIG_EN6)

/* GPIO ADC inputs for current sensing */
#define CH0_SENS  ()
#define CH1_SENS  ()
#define CH2_SENS  ()
#define CH3_SENS  ()
#define CH4_SENS  ()
#define CH5_SENS  ()
#define CH6_SENS  ()
#define CH7_SENS  ()

/* PWM Configuration */
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_TEST_CH_NUM       (8)

/* I2C slave configuration */
//#define ESP_SLAVE_ADDR             0x28             /*!< ESP32 slave address, you can set any 7bit value */
#define I2C_SLAVE_START_ADDRESS    0x10
//#define DATA_LENGTH                128              /*!<Data buffer length for test buffer*/
#define I2C_SLAVE_NUM              I2C_NUM_1        /*!<I2C port number for slave dev */
// New driver buffer info
#define I2C_SLAVE_BUF_LEN          256
#define I2C_SLAVE_RO_LEN           0

//#define I2C_SLAVE_TX_BUF_LEN       (2*DATA_LENGTH)  /*!<I2C slave tx buffer size */
//#define I2C_SLAVE_RX_BUF_LEN       (2*DATA_LENGTH)  /*!<I2C slave rx buffer size */
#define I2C_SLAVE_SCL_IO           (34)               /*!<gpio number for i2c slave clock  */
#define I2C_SLAVE_SDA_IO           (18)               /*!<gpio number for i2c slave data */

#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_SCL_IO          (32)
#define I2C_MASTER_SDA_IO          (33)
#define I2C_MASTER_FREQ_HZ         800000

#define I2C_SCANNER_NUM    I2C_MASTER_NUM
#define I2C_SCANNER_SCL_IO I2C_MASTER_SCL_IO
#define I2C_SCANNER_SDA_IO I2C_MASTER_SDA_IO

#define ADS Adafruit_ADS1115 /*!<Adafruit_ADS1115 or Adafruit_ADS1105 */
#define ADS_CNT 2            /*!<Number of ADS chips per board */
#define CH_CNT 4             /*!<Number of channels per ADS chip */
#define THR_CNT 2            /*!<Number of thresholds to check */

/* I2C slave non-channel registers */
#define CH_DUTY_BASE      0
// 8 int = 16 bytes duty

#define CH_SET_FREQ       0x10
// 1 int = 2 bytes freq

#define CH_RESET          0x12
#define CH_SET_DIGITAL    0x14
#define CH_RESET_DIGITAL  0x16
#define CH_OTA            0x18
// Leaving 1 int for each but none is needed or ever read

// Space to align at lower nibble=0

#define CH_READINGS_BASE  0x20
// 8 int = 16 bytes readings
#define CH_PRESENCE_BASE  0x30
// 2x8x1bit = 2 bytes presence
// Space to duplicate presence (2 bytes)
#define CH_ADC_CONFIG     0x34
// 1 int = 2 bytes config

