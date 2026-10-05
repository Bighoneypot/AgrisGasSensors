
// =====================================================================
// config.h - CONFIGURAZIONE GLOBALE FIRMWARE v6.6
// =====================================================================
// MYRUMINET - Sensore Gas Stalla
// RAK3172-E / RAK19007 Rev.C
//
// NOTA: Le variabili globali (joined, flash_paused, i2c_bus_ok,
//       tx_fail_count) e le funzioni utility (readBatteryMV,
//       estimateSOC) sono in diagnostics.h per evitare duplicati.
// =====================================================================

#ifndef CONFIG_H
#define CONFIG_H

// =====================================================================
// VERSIONE FIRMWARE
// =====================================================================
#define FW_VERSION        "6.7.1"
#define FW_NAME           "MYRUMINET - SENSORE GAS STALLA"
#define FW_BOARD          "RAK3172-E / RAK19007 Rev.C"
#define FW_MODULES        "BME680 + NH3 + H2S + RTC + FLASH"
#define RECOVERY_MODE     false
#define I2C_TIMEOUT_MS    3000
#define IWDG_TIMEOUT_MS   60000
#define REBOOT_EVERY_N_CYCLES     336     // 336 * 30 min = 7 giorni
#define SWWDT_TIMEOUT_MS  30000   // 30 secondi timeout SW watchdog


// =====================================================================
// MODIFICA QUI PER DEBUG/PRODUZIONE
// =====================================================================

// Produzione: true    Debug: true/false
#define ENABLE_PREHEAT          true

// Produzione: 5       Debug: 1
#define PREHEAT_MINUTES         3

// Produzione: 25      Debug: 3
#define SLEEP_MINUTES           27


// =====================================================================
// PIN HARDWARE - RAK19007 Rev.C
// =====================================================================

// Alimentazione sensori (3V3_S switch)
// NOTA: WB_IO2 e' anche HOLD della RAK15001 su Slot B!
#define SENSOR_POWER_PIN        WB_IO2

// Batteria
#define BATTERY_PIN             WB_A0
#define BATTERY_DIVIDER         2.265f

// I2C (bus condiviso BME680 + NH3 + H2S + RTC)
#define I2C_SDA_PIN             PA11
#define I2C_SCL_PIN             PA12

// SPI Flash RAK15001 - Slot B
#define FLASH_CS_PIN            SS          // PA4 - SPI1_NSS hardware
#define FLASH_WP_PIN            WB_IO1      // Write Protect per Slot B
#define FLASH_HOLD_PIN          WB_IO2      // HOLD per Slot B (= 3V3_S!)


// =====================================================================
// INDIRIZZI I2C
// =====================================================================

#define BME680_I2C_ADDR         0x76
#define NH3_I2C_ADDR            0x77
#define H2S_I2C_ADDR            0x74
#define RTC_I2C_ADDR            0x52    // RV-3028 su RAK12002


// =====================================================================
// LORAWAN
// =====================================================================

#define LORAWAN_BAND            RAK_REGION_EU868
#define LORAWAN_CLASS           CLASS_A
#define LORAWAN_FPORT           77
#define LORAWAN_DR_JOIN         0
#define LORAWAN_DR_TX           5
#define LORAWAN_RX1DL           5000
#define LORAWAN_RX2DL           6000


// =====================================================================
// CREDENZIALI OTAA - Agris_Lifely_GAS_Sensors_01
// =====================================================================
// IDENTICHE AL v6.5.5

static uint8_t node_dev_eui[8]  = {0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x0A, 0x70, 0xF4};
static uint8_t node_app_eui[8]  = {0xAC, 0x1F, 0x09, 0xFF, 0xF8, 0x68, 0x31, 0x72};
static uint8_t node_app_key[16] = {0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x0A, 0x70, 0xF4,
                                    0xAC, 0x1F, 0x09, 0xFF, 0xF8, 0x68, 0x31, 0x72};


// =====================================================================
// WATCHDOG TIMER
// =====================================================================

#define WDT_TIMEOUT_MS          120000UL    // 120 secondi (2 min)
#define WDT_TIMER_ID            RAK_TIMER_4


// =====================================================================
// TX FAILURE SAFETY
// =====================================================================

#define MAX_TX_FAILURES         3


// =====================================================================
// I2C RECOVERY
// =====================================================================

#define I2C_RECOVERY_MAX_RETRIES    3
#define SENSOR_POWERUP_DELAY_MS     500     // 500 ms stabilizzazione dopo power on


// =====================================================================
// PROTEZIONE BATTERIA
// =====================================================================

#define BATTERY_MIN_MV          3000
#define LOW_BATTERY_SLEEP_MS    (60UL * 60UL * 1000UL)   // 60 minuti


// =====================================================================
// FLASH LOG - GD25Q16C (2 MB)
// =====================================================================

#define FLASH_PAGE_SIZE         256
#define FLASH_SECTOR_SIZE       4096
#define FLASH_TOTAL_SIZE        (2UL * 1024UL * 1024UL)  // 2 MB

// Settore 0 = header (puntatore scrittura, contatore)
// Settori 1-511 = log entries (ring buffer)
#define FLASH_LOG_HEADER_ADDR   0x000000
#define FLASH_LOG_START_ADDR    0x001000    // settore 1
#define FLASH_LOG_END_ADDR      FLASH_TOTAL_SIZE
#define FLASH_RECORD_SIZE       256         // 1 record = 1 pagina = 256 byte max


// =====================================================================
// JOIN POLLING
// =====================================================================
// EU868 con DR0 (SF12) ha bisogno di piu' tempo per il join.
// RX1 = 5s, RX2 = 6s, quindi ogni tentativo richiede ~7-8 secondi.
// Con 30 polls * 5s = 150 secondi di attesa massima.

#define JOIN_POLL_INTERVAL      5000        // 5 secondi
#define JOIN_MAX_POLLS          30          // 30 * 5s = 150s max


// =====================================================================
// FORMULE AUTOMATICHE (NON MODIFICARE)
// =====================================================================

#define SLEEP_INTERVAL          ((unsigned long)SLEEP_MINUTES * 60UL * 1000UL)

#if ENABLE_PREHEAT
  #define PREHEAT_INTERVAL      ((unsigned long)PREHEAT_MINUTES * 60UL * 1000UL)
  #define PREHEAT_BLOCK         30000UL     // 30 secondi per blocco
  #define PREHEAT_BLOCKS        ((PREHEAT_INTERVAL) / (PREHEAT_BLOCK))
  #define PREHEAT_SECONDS       (PREHEAT_MINUTES * 60)
  #define CYCLE_TOTAL_MIN       (SLEEP_MINUTES + PREHEAT_MINUTES)
#else
  #define CYCLE_TOTAL_MIN       SLEEP_MINUTES
#endif


#endif // CONFIG_H

