
// =====================================================================
// config.h - CONFIGURAZIONE GLOBALE FIRMWARE v7.2
// =====================================================================
// MYRUMINET - Sensore Gas Stalla
// RAK4630 (nRF52840 + SX1262) / RAK19007 Rev.C
// BUILD INCREMENTALE — Step 1: Base
// =====================================================================

#ifndef CONFIG_H
#define CONFIG_H

// =====================================================================
// VERSIONE FIRMWARE
// =====================================================================
#define FW_VERSION        "7.2"
#define FW_NAME           "MYRUMINET - SENSORE GAS STALLA"
#define FW_ARCH           "EVENT-DRIVEN (pattern RAK)"
#define FW_BOARD          "RAK4630 (nRF52840) / RAK19007 Rev.C"
#define FW_MODULES        "BME680 + NH3 + H2S + RTC + FLASH"
#define FW_DRIVERS        "Bare-metal (zero heap)"


// =====================================================================
// MODIFICA QUI PER DEBUG/PRODUZIONE
// =====================================================================
#define ENABLE_PREHEAT          true
#define PREHEAT_MINUTES         3           // Produzione: 3, Debug: 1
#define SLEEP_MINUTES           27          // Produzione: 27, Debug: 3


// =====================================================================
// PIN HARDWARE - RAK4630 su RAK19007 Rev.C
// =====================================================================
#define SENSOR_POWER_PIN        WB_IO2      // 3V3_S switch

// I2C pin — SOLO per recovery bit-bang
#define I2C_SDA_PIN             13          // P0.13
#define I2C_SCL_PIN             14          // P0.14

// Batteria
#define BATTERY_PIN             WB_A0       // AIN2 (P0.04)
#define BATTERY_DIVIDER         2.265f      // Calibrato su RAK19007 Rev.C

// SPI Flash RAK15001 - Slot B
#define FLASH_CS_PIN            SS
#define FLASH_WP_PIN            WB_IO1
#define FLASH_HOLD_PIN          WB_IO2


// =====================================================================
// INDIRIZZI I2C
// =====================================================================
#define BME680_I2C_ADDR         0x76
#define NH3_I2C_ADDR            0x77
#define H2S_I2C_ADDR            0x74
#define RTC_I2C_ADDR            0x52


// =====================================================================
// I2C BUS
// =====================================================================
#define I2C_TIMEOUT_MS              3000
#define I2C_RECOVERY_MAX_RETRIES    3
#define SENSOR_POWERUP_DELAY_MS     500


// =====================================================================
// LORAWAN
// =====================================================================
#define LORAWAN_BAND            RAK_REGION_EU868
#define LORAWAN_CLASS           CLASS_A
#define LORAWAN_FPORT           77
#define LORAWAN_DR_JOIN         0       // SF12
#define LORAWAN_DR_TX           5       // SF7
#define LORAWAN_RX1DL           5000
#define LORAWAN_RX2DL           6000
#define JOIN_POLL_INTERVAL      5000
#define JOIN_MAX_POLLS          30


// =====================================================================
// CREDENZIALI OTAA
// =====================================================================
static uint8_t node_dev_eui[8]  = {0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x0A, 0x70, 0xF4};
static uint8_t node_app_eui[8]  = {0xAC, 0x1F, 0x09, 0xFF, 0xF8, 0x68, 0x31, 0x72};
static uint8_t node_app_key[16] = {0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x0A, 0x70, 0xF4,
                                    0xAC, 0x1F, 0x09, 0xFF, 0xF8, 0x68, 0x31, 0x72};


// =====================================================================
// PROTEZIONE BATTERIA
// =====================================================================
#define BATTERY_MIN_MV          3000
#define LOW_BATTERY_SLEEP_MS    (60UL * 60UL * 1000UL)  // 60 minuti


// =====================================================================
// PROTEZIONE MEMORIA
// =====================================================================
#define MEM_CRITICAL_THRESHOLD  1024


// =====================================================================
// WATCHDOG
// =====================================================================
#define WDT_TIMEOUT_MS          120000UL    // 120 secondi
#define WDT_TIMER_ID            RAK_TIMER_4


// =====================================================================
// TX FAILURE / REBOOT
// =====================================================================
#define MAX_TX_FAILURES         3
#define REBOOT_EVERY_N_CYCLES   336     // ~7 giorni


// =====================================================================
// FLASH LOG - GD25Q16C (2 MB)
// =====================================================================
#define FLASH_PAGE_SIZE         256
#define FLASH_SECTOR_SIZE       4096
#define FLASH_TOTAL_SIZE        (2UL * 1024UL * 1024UL)
#define FLASH_LOG_HEADER_ADDR   0x000000
#define FLASH_LOG_START_ADDR    0x001000
#define FLASH_LOG_END_ADDR      FLASH_TOTAL_SIZE
#define FLASH_RECORD_SIZE       256


// =====================================================================
// FORMULE AUTOMATICHE (NON MODIFICARE)
// =====================================================================
#define SLEEP_INTERVAL          ((unsigned long)SLEEP_MINUTES * 60UL * 1000UL)

#if ENABLE_PREHEAT
  #define PREHEAT_INTERVAL      ((unsigned long)PREHEAT_MINUTES * 60UL * 1000UL)
  #define PREHEAT_SECONDS       (PREHEAT_MINUTES * 60)
  #define CYCLE_TOTAL_MIN       (SLEEP_MINUTES + PREHEAT_MINUTES)
#else
  #define CYCLE_TOTAL_MIN       SLEEP_MINUTES
#endif


#endif // CONFIG_H

