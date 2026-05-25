#ifndef CONFIG_H
#define CONFIG_H

#define SERIAL_DEBUG    1
#if SERIAL_DEBUG == 1
  #define debugInit(x)  Serial.begin(x)
  #define debug(x)      Serial.print(x)
  #define debugln(x)    Serial.println(x)
  #define debugBin(x)   Serial.print(x, BIN)
  #define debugHex(x)   Serial.print(x, HEX)
  #define debugf(...)   Serial.printf(__VA_ARGS__)
#else
  #define debugInit(x)
  #define debug(x)
  #define debugln(x)
  #define debugBin(x)
  #define debugHex(x)
  #define debugf(...)
#endif

#define BV16(x)         (uint16_t(1u) << (x))
#ifndef max
  #define               max(a, b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
  #define               min(a, b) (((a) < (b)) ? (a) : (b))
#endif








// Definicje pinów SPI4
#define SPI4_SCK           PE_12
#define SPI4_MISO          PE_13
#define SPI4_MOSI          PE_14

// Piny CS dla urządzeń na SPI4
#define CS_BMP2            PE_8
#define CS_BMP1            PE_9
#define CS_ADXL            PE_10
#define CS_LSM             PE_11

// SPI1 (Flash)
#define CS_FLASH           PA_4
#define SPI1_SCK           PA_5
#define SPI1_MISO          PA_6
#define SPI1_MOSI          PA_7

// Piny dla LoRa (osobny SPI)
#define SPI_LORA_SCK       PC_10
#define SPI_LORA_MISO      PC_11
#define SPI_LORA_MOSI      PC_12
#define BUSY_LORA          PD_0
#define CS_LORA            PD_3
#define RST_LORA           -1      // brak podłączenia
#define DIO1_LORA          -1      // brak podłączenia

// Piny UART dla GPS (opcjonalnie)
#define RX_MAX             PB_10
#define TX_MAX             PB_11

// Piny DIP switch (wejścia)
#define MODE1              PD_13
#define MODE2              PD_12
#define MODE3              PD_11
#define MODE4              PD_10

// Piny GPIO
#define LED_1              PE_5                      // info
#define LED_2              PE_6                      // do errorów
#define BUZZER             PE_15
#define SOLENOID_1         PE_7
#define SOLENOID_2         PB_1
#define BATTERY            PC_0


// LoRa CONFIG
#define FREQUENCY                           868.0f   // MHz
#define BANDWIDTH                           125.0f   // kHz
#define SF                                  9        // 7-12
#define CODING_RATE                         5        // 5-8
#define POWER                               20       // dBm (do 17-22 dBm)
#define PREAMBLE_LENGTH                     15       // 6-30 symbols, the longer the symbols, the better the synchronization and range?, but the slower the transmission.
#define LORA_TX_TIMEOUT                     500      // ms

// bufory
#define FLUSH_AFTER_WRITES                  5
#define FUSION_ALPHA                        0.8f

#define LAUNCH_ACCEL_THRESHOLD             30.0f     // >3G
#define BURNOUT_ACCEL_THRESHOLD            5.0f      // <0.5G
#define APOGEE_VELOCITY_THRESHOLD          0.5f      // m/s
#define LANDING_VELOCITY_THRESHOLD         0.5f      // m/s

#define LANDING_STABLE_TIME                10000     // 10 sekund
#define DEPLOY_DROGUE_MAX_SPEED            15.0f     // m/s
#define DEPLOY_MAIN_MAX_SPEED              30.0f     // m/s
#define DEPLOY_DROGUE_MIN_ALTITUDE         400.0f    // metry
#define DEPLOY_MAIN_MIN_ALTITUDE           200.0f    // metry
#define SOLENOID_DELAY                     200       //ms
#define WATCHDOG_REFRESH_INTERVAL          4000

#define LAUNCH_DEBOUNCE_MS                 50
#define BURNOUT_DEBOUNCE_MS                300
#define APOGEE_DEBOUNCE_MS                 500
#define LANDING_DEBOUNCE_MS                10000

#define DROGUE_PARASHUTE_TIMEOUT           20000     // ms









// TIMING CONFIG
#define INTERVAL_DEBUG                      500
#define INTERVAL_FLIGHT                     50
#define INTERVAL_DUMP                       500
#define INTERVAL_SLEEP                      5000



#define INTERVAL_IDLE                       5000
#define INTERVAL_READY                      1000
#define INTERVAL_BURN                       100
#define INTERVAL_RISING                     100
#define INTERVAL_FALLING                    500
#define INTERVAL_TOUCHDOWN                  10000


// WATCHDOG / BUZZER
#define WATCHDOG_INTERVAL                   2500
#define BUZZER_INTERVAL_DEBUG               5000
#define BUZZER_INTERVAL_IDLE                10000
#define BUZZER_INTERVAL_READY               1000
#define BUZZER_INTERVAL_BURN                65535
#define BUZZER_INTERVAL_RISING              65535
#define BUZZER_INTERVAL_FALLING             65535
#define BUZZER_INTERVAL_TOUCHDOWN           10000

// SENSORS / MEMORY
#define GPS_BAUDRATE                        9600
#define SEA_LEVEL_PRESSURE_HPA              1013.25f
#define ADXL375_MG2G_MULTIPLIER             (0.049f)  // trzeba sprawdzić czy to dodać → / ADXL343_MG2G_MULTIPLIER)

// STATUS UPGRADE CONST      /// tu trzeba wpisać dane z symulacji
#define BURN_ACCEL_THRESHOLD                5.0f
#define MAX_FREEFALLING_SPEED               (-20.0f)
#define BURN_ACCEL_CHECK_TIME               300
#define RISING_ACCEL_CHECK_TIME             3000
#define APOGEE_CHECK_TIME                   1000
#define MAX_RISING_TIME                     45000
#define MIN_PARACHUTE_TIME                  (MAX_RISING_TIME + 5000)
#define TOUCHDOWN_CHECK_TIME                2000
#define MAIN_PARASHUTE_TIMEOUT              (MAX_RISING_TIME + 15000)

// ERROR CODES
#define LORA_ERROR                          BV16(0)
#define LSM_ERROR                           BV16(1)
#define BMP1_ERROR                          BV16(2)
#define BMP2_ERROR                          BV16(3)
#define ADXL_ERROR                          BV16(4)
#define GPS_ERROR                           BV16(5)
#define FLASH_ERROR                         BV16(6)
#define FLASH_FILE_ERROR                    BV16(7)
////////////////////////////////////////////////////////////////////////////////

// DATA LEN AND POSITIONS
// HEADER for 16 bits
#define timePos                             16
#define timeLen                             22
#define packetPos                           (timePos + timeLen)
#define packetLen                           16
#define errorPos                            (packetPos + packetLen)
#define errorLen                            16
#define statusPos                           (errorPos + errorLen)
#define statusLen                           4
#define handlePos                           (statusPos + statusLen)
#define handleLen                           1

#define gpsLatPos                           (handlePos + handleLen)
#define gpsLatLen                           (17+1)
#define gpsLngPos                           (gpsLatPos + gpsLatLen)
#define gpsLngLen                           (17+1)
#define gpsAltiPos                          (gpsLngPos + gpsLngLen)
#define gpsAltiLen                          14
#define gpsHourPos                          (gpsAltiPos + gpsAltiLen)
#define gpsHourLen                          5
#define gpsMinPos                           (gpsHourPos + gpsHourLen)
#define gpsMinLen                           6
#define gpsSecPos                           (gpsMinPos + gpsMinLen)
#define gpsSecLen                           6
#define gpsCentisecPos                      (gpsSecPos + gpsSecLen)
#define gpsCentisecLen                      7
#define gpsSpeedPos                         (gpsCentisecPos + gpsCentisecLen)
#define gpsSpeedLen                         (14+1)
#define gpsCoursePos                        (gpsSpeedPos + gpsSpeedLen)
#define gpsCourseLen                        9
#define gpsSatNumPos                        (gpsCoursePos + gpsCourseLen)
#define gpsSatNumLen                        3
#define gpsHdopPos                          (gpsSatNumPos + gpsSatNumLen)
#define gpsHdopLen                          5

#define lsmAccelXPos                        (gpsHdopPos + gpsHdopLen)
#define lsmAccelXLen                        (14+1)
#define lsmAccelYPos                        (lsmAccelXPos + lsmAccelXLen)
#define lsmAccelYLen                        (14+1)
#define lsmAccelZPos                        (lsmAccelYPos + lsmAccelYLen)
#define lsmAccelZLen                        (14+1)
#define lsmGyroXPos                         (lsmAccelZPos + lsmAccelZLen)
#define lsmGyroXLen                         (14+1)
#define lsmGyroYPos                         (lsmGyroXPos + lsmGyroXLen)
#define lsmGyroYLen                         (14+1)
#define lsmGyroZPos                         (lsmGyroYPos + lsmGyroYLen)
#define lsmGyroZLen                         (14+1)
#define lsmTempPos                          (lsmGyroZPos + lsmGyroZLen)
#define lsmTempLen                          (9+1)
#define lsmSpeedPos                         (lsmTempPos + lsmTempLen)
#define lsmSpeedLen                         (10+1)

#define adxlAccelXPos                       (lsmSpeedPos + lsmSpeedLen)
#define adxlAccelXLen                       (15+1)
#define adxlAccelYPos                       (adxlAccelXPos + adxlAccelXLen)
#define adxlAccelYLen                       (15+1)
#define adxlAccelZPos                       (adxlAccelYPos + adxlAccelYLen)
#define adxlAccelZLen                       (15+1)
#define adxlSpeedPos                        (adxlAccelZPos + adxlAccelZLen)
#define adxlSpeedLen                        (14+1)

#define bmp1TempPos                         (adxlSpeedPos + adxlSpeedLen)
#define bmp1TempLen                         (9+1)
#define bmp1PressPos                        (bmp1TempPos + bmp1TempLen)
#define bmp1PressLen                        14
#define bmp1AltiPos                         (bmp1PressPos + bmp1PressLen)
#define bmp1AltiLen                         14
#define bmp1SpeedPos                        (bmp1AltiPos + bmp1AltiLen)
#define bmp1SpeedLen                        (14+1)

#define bmp2TempPos                         (bmp1SpeedPos + bmp1SpeedLen)
#define bmp2TempLen                         (9+1)
#define bmp2PressPos                        (bmp2TempPos + bmp2TempLen)
#define bmp2PressLen                        14
#define bmp2AltiPos                         (bmp2PressPos + bmp2PressLen)
#define bmp2AltiLen                         14
#define bmp2SpeedPos                        (bmp2AltiPos + bmp2AltiLen)
#define bmp2SpeedLen                        (14+1)

#define batteryPos                          (bmp2SpeedPos + bmp2SpeedLen)
#define batteryLen                          7

// CHECKSUM for 8 bits

#endif  // CONFIG_H