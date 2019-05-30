#define PIN_BLE_CS A2       // Bluetooth CS Arduino pin number
#define BLE_CS_PORT PORTF   // Bluetooth CS port
#define BLE_CS_BIT PORTF5   // Bluetooth CS physical bit number

#define PIN_BLE_IRQ 0       // Bluetooth IRQ Arduino pin number
#define BLE_IRQ_PORT PORTD  // Bluetooth IRQ port
#define BLE_IRQ_BIT PORTD2  // Bluetooth IRQ physical bit number

#define PIN_BLE_RST A1      // Bluetooth reset Arduino pin number
#define BLE_RST_PORT PORTF  // Bluetooth reset port
#define BLE_RST_BIT PORTF6  // Bluetooth reset physical bit number

#define PIN_BATT_LVL 12       // Battery Level Arduino pin number
#define BATT_LVL_PORT PORTD   // Battery Level port
#define BATT_LVL_BIT PORTD6   // Battery Level physical bit number

#define PIN_BATT_EN 4         // Battery Enable Arduino pin number
#define BATT_EN_PORT PORTD    // Battery Enable port
#define BATT_EN_BIT PORTD4    // Battery Enable physical bit number

#define PIN_BATT_STAT 5       // Battery State Arduino pin number
#define BATT_STAT_PORT PORTC  // Battery State port
#define BATT_STAT_BIT PORTC6  // Battery State physical bit number

#define PIN_MPU_INT 7       // MPU6050 interrupt Arduino pin number
#define MPU_INT_PORT PORTE  // MPU6050 interrupt port
#define MPU_INT_BIT PORTE6  // MPU6050 interrupt physical bit number

#define PIN_RTC_INT 1       // Real Time Clock interrupt Arduino pin number
#define RTC_INT_PORT PORTD  // Real Time Clock interrupt port
#define RTC_INT_BIT PORTD3  // Real Time Clock interrupt physical bit number

#define PIN_I2C_SDA 2       // I2C SDA Arduino pin number
#define I2C_SDA_PORT PORTD  // I2C SDA port
#define I2C_SDA_BIT PORTD1  // I2C SDA physical bit number

#define PIN_I2C_SCL 3       // I2C SCL Arduino pin number
#define I2C_SCL_PORT PORTD  // I2C SCL port
#define I2C_SCL_BIT PORTD0  // I2C SCL physical bit number

#define PIN_CS A5       // Display CS Arduino pin number
#define CS_PORT PORTF   // Display CS port
#define CS_BIT PORTF0   // Display CS physical bit number

#define PIN_DC A3       // Display D/C Arduino pin number
#define DC_PORT PORTF   // Display D/C port
#define DC_BIT PORTF4   // Display D/C physical bit number

#define PIN_RST A4      // Display reset Arduino pin number
#define RST_PORT PORTF  // Display reset port
#define RST_BIT PORTF1  // Display reset physical bit number

#define LEDL 13   /**< The pin number for LEDL. */
#define LEDL_PORT PORTC
#define LEDL_BIT PORTC7

#define LEDR 6    /**< The pin number for LEDR. */
#define LEDR_PORT PORTD
#define LEDR_BIT PORTD7

// bit values for button states
// these are determined by the buttonsState() function
#define LEFT_BUTTON _BV(0)  /**< The Left button value for functions requiring a bitmask */
#define RIGHT_BUTTON _BV(0) /**< The Right button value for functions requiring a bitmask */
#define UP_BUTTON _BV(7)    /**< The Up button value for functions requiring a bitmask */
#define DOWN_BUTTON _BV(6)  /**< The Down button value for functions requiring a bitmask */
#define A_BUTTON _BV(0)     /**< The A button value for functions requiring a bitmask */
#define B_BUTTON _BV(4)     /**< The B button value for functions requiring a bitmask */

#define PIN_UP_BUTTON 11
#define UP_BUTTON_PORT PORTB
#define UP_BUTTON_PORTIN PINB
#define UP_BUTTON_DDR DDRB
#define UP_BUTTON_BIT PORTB7

#define PIN_DOWN_BUTTON 10
#define DOWN_BUTTON_PORT PORTB
#define DOWN_BUTTON_PORTIN PINB
#define DOWN_BUTTON_DDR DDRB
#define DOWN_BUTTON_BIT PORTB6

#define PIN_B_BUTTON 8
#define B_BUTTON_PORT PORTB
#define B_BUTTON_PORTIN PINB
#define B_BUTTON_DDR DDRB
#define B_BUTTON_BIT PORTB4

#define PIN_SPEAKER 9  /**< The pin number of the speaker */
#define SPEAKER_PORT PORTB
#define SPEAKER_DDR DDRB
#define SPEAKER_BIT PORTB5

// Unconnected analog input used for noise by initRandomSeed()
#define RAND_SEED_IN A0
#define RAND_SEED_IN_PORT PORTF
#define RAND_SEED_IN_BIT PORTF7
// Value for ADMUX to read the random seed pin: 2.56V reference, ADC7
#define RAND_SEED_IN_ADMUX (_BV(REFS1) | _BV(REFS0) | _BV(MUX2) | _BV(MUX1) | _BV(MUX0))

// SPI interface
#define SPI_MISO_PORT PORTB
#define SPI_MISO_BIT PORTB3

#define SPI_MOSI_PORT PORTB
#define SPI_MOSI_BIT PORTB2

#define SPI_SCK_PORT PORTB
#define SPI_SCK_BIT PORTB1

#define SPI_SS_PORT PORTB
#define SPI_SS_BIT PORTB0
// --------------------
