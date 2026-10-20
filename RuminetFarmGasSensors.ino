
// =====================================================================
// GAS_Sensor_v7.2a.ino - EVENT-DRIVEN FIRMWARE COMPLETO
// =====================================================================
// MYRUMINET - Sensore Gas Stalla
// RAK4630 (nRF52840 + SX1262) / RAK19007 Rev.C
//
// v7.2a CHANGELOG:
//   - Time Sync via downlink LoRaWAN (fPort 10, 7 bytes)
//   - Preheat con MCU SVEGLIA (no sleep durante preheat)
//   - Flash write SOLO dopo TX_DONE (no conflitti SPI)
//   - Timing GENEROSI su tutte le operazioni SPI/I2C
//   - NO HW Watchdog (causa reboot in sleep su nRF52840)
//   - Reboot preventivo ogni 336 cicli come safety net
//
// Board Arduino IDE: WisBlock Core RAK4631 | Baud: 115200
// =====================================================================

#include <Wire.h>
#include "config.h"
#include "diagnostics.h"
#include "memory_monitor.h"
#include "rtc_time.h"
#include "flash_log.h"
#include "sleep_diag.h"
#include "bme680_sensor.h"
#include "gas_sensors.h"
#include "lorawan_manager.h"

// =====================================================================
// FALLBACK DEFINES
// =====================================================================
#ifndef RECOVERY_MODE
#define RECOVERY_MODE           false
#endif
#ifndef REBOOT_EVERY_N_CYCLES
#define REBOOT_EVERY_N_CYCLES   336
#endif

// =====================================================================
// INTERVALLI TIMER
// =====================================================================
#define CYCLE_INTERVAL_MS       ((uint32_t)SLEEP_MINUTES * 60UL * 1000UL)
#define PREHEAT_MS              ((uint32_t)PREHEAT_MINUTES * 60UL * 1000UL)
#define JOIN_RETRY_MS           120000UL
#define MAX_JOIN_RETRIES        5
#define MAX_TX_FAILURES_CONSEC  3

// =====================================================================
// TIMING GENEROSI
// =====================================================================
#define SPI_BUS_SWITCH_MS       50
#define SPI_POST_WAKE_MS        30
#define SPI_POST_WRITE_MS       20
#define SPI_PRE_SLEEP_MS        20
#define SPI_POST_ERASE_MS       200
#define SPI_POST_INIT_MS        100
#define I2C_POST_BEGIN_MS       200
#define I2C_POST_RECOVERY_MS    100
#define I2C_BETWEEN_DEVICES_MS  50
#define I2C_POST_SENSOR_READ_MS 30
#define RADIO_POST_TX_MS        200
#define PREHEAT_LOG_INTERVAL_MS 30000
#define POWER_ON_SETTLE_MS      500
#define POWER_CYCLE_OFF_MS      1000
#define POWER_CYCLE_ON_MS       500
#define BOOT_USB_SETTLE_MS      3000
#define BOOT_SERIAL_INIT_MS     2000

// Time Sync
#define TIMESYNC_FPORT          10
#define TIMESYNC_PAYLOAD_LEN    7

// =====================================================================
// FLAG VOLATILI
// =====================================================================
volatile bool flag_startCycle   = false;
volatile bool flag_txDone       = false;
volatile bool flag_joinDone     = false;
volatile int  flag_txStatus     = 0;

// =====================================================================
// DATI CICLO
// =====================================================================
static BmeData  bmeData         = {0.0f, 0.0f, 0.0f, 0};
static GasData  gasData         = {0.0f, 0.0f};
static uint16_t battery_mv      = 0;
static uint8_t  battery_soc     = 0;
static char     jsonLogBuf[FLASH_RECORD_SIZE];
static const char *cycleTimestamp = "";
static uint32_t flashWriteCount = 0;
static uint8_t  joinRetryCount  = 0;
static uint8_t  tx_fail_count_consec = 0;
static bool     pendingFlashWrite = false;
static char     pendingPhase[16] = "";
static bool     rtcSynced = false;
static uint32_t rtcSyncCount = 0;

// =====================================================================
// CALLBACK LoRaWAN
// =====================================================================
void cb_joinResult(int32_t status) {
    flag_joinDone = true;
    Serial.print("[CB] Join result: "); Serial.println(status);
}

void cb_sendResult(int32_t status) {
    flag_txDone   = true;
    flag_txStatus = status;
    Serial.print("[CB] TX result: "); Serial.println(status);
}

void cb_recvData(SERVICE_LORA_RECEIVE_T *data) {
    if (data->BufferSize == 0) return;

    Serial.print("[CB] Downlink: "); Serial.print(data->BufferSize);
    Serial.print(" bytes, fPort "); Serial.println(data->Port);

    // --- TIME SYNC (fPort 10, 7 bytes) ---
    if (data->Port == TIMESYNC_FPORT && data->BufferSize == TIMESYNC_PAYLOAD_LEN) {
        uint8_t yearShort = data->Buffer[0];
        uint8_t month     = data->Buffer[1];
        uint8_t day       = data->Buffer[2];
        uint8_t hourUTC   = data->Buffer[3];
        uint8_t minute    = data->Buffer[4];
        uint8_t second    = data->Buffer[5];
        uint8_t tzQH      = data->Buffer[6];

        uint16_t year = 2000 + yearShort;
        uint8_t tzHours   = tzQH / 4;
        uint8_t tzMinutes = (tzQH % 4) * 15;
        uint8_t hourLocal = hourUTC + tzHours;
        uint8_t minLocal  = minute + tzMinutes;

        if (minLocal >= 60) { minLocal -= 60; hourLocal++; }
        if (hourLocal >= 24) { hourLocal -= 24; day++; }

        Serial.println();
        Serial.println("[TIMESYNC] === Downlink Time Sync ===");
        Serial.print("[TIMESYNC] UTC:   "); Serial.print(year); Serial.print("-");
        if (month < 10) Serial.print("0"); Serial.print(month); Serial.print("-");
        if (day < 10) Serial.print("0"); Serial.print(day); Serial.print(" ");
        if (hourUTC < 10) Serial.print("0"); Serial.print(hourUTC); Serial.print(":");
        if (minute < 10) Serial.print("0"); Serial.print(minute); Serial.print(":");
        if (second < 10) Serial.print("0"); Serial.println(second);
        Serial.print("[TIMESYNC] TZ:    +"); Serial.print(tzHours); Serial.print("h");
        if (tzMinutes > 0) { Serial.print(tzMinutes); Serial.print("m"); }
        Serial.println();
        Serial.print("[TIMESYNC] Local: "); Serial.print(year); Serial.print("-");
        if (month < 10) Serial.print("0"); Serial.print(month); Serial.print("-");
        if (day < 10) Serial.print("0"); Serial.print(day); Serial.print(" ");
        if (hourLocal < 10) Serial.print("0"); Serial.print(hourLocal); Serial.print(":");
        if (minLocal < 10) Serial.print("0"); Serial.print(minLocal); Serial.print(":");
        if (second < 10) Serial.print("0"); Serial.println(second);

        if (year < 2026 || year > 2035 || month < 1 || month > 12 ||
            day < 1 || day > 31 || hourLocal > 23 || minLocal > 59 || second > 59) {
            Serial.println("[TIMESYNC] ERRORE: valori fuori range!");
            return;
        }

        rtcSetTime(year, month, day, hourLocal, minLocal, second);
        rtcSynced = true;
        rtcSyncCount++;
        Serial.println("[TIMESYNC] RTC aggiornato!");
        Serial.print("[TIMESYNC] Verifica: "); Serial.println(rtcGetTimestamp());
        Serial.print("[TIMESYNC] Sync totali: "); Serial.println(rtcSyncCount);
        Serial.println("[TIMESYNC] =============================");
        Serial.println();
    } else {
        Serial.print("[CB] Payload HEX: ");
        for (int i = 0; i < data->BufferSize; i++) {
            if (data->Buffer[i] < 16) Serial.print("0");
            Serial.print(data->Buffer[i], HEX); Serial.print(" ");
        }
        Serial.println();
    }
}

// =====================================================================
// CALLBACK TIMER
// =====================================================================
void cb_timerCycle(void *) {
    flag_startCycle = true;
    Serial.println("[CB] TIMER_0: avvio ciclo");
}

void cb_timerJoinRetry(void *) {
    Serial.println("[CB] TIMER_2: retry join");
    api.lorawan.join();
}

// =====================================================================
// I2C CLOCK RECOVERY
// =====================================================================
void i2cClockRecovery(void) {
    Wire.end();
    delay(50);
    pinMode(I2C_SCL_PIN, OUTPUT);
    pinMode(I2C_SDA_PIN, INPUT_PULLUP);
    delay(10);

    if (digitalRead(I2C_SDA_PIN) == HIGH) {
        Serial.println("[I2C] Bus OK (SDA HIGH)");
        pinMode(I2C_SCL_PIN, INPUT);
        pinMode(I2C_SDA_PIN, INPUT);
        delay(I2C_POST_RECOVERY_MS);
        return;
    }

    Serial.print("[I2C] SDA basso, recovery... ");
    for (int i = 0; i < 9; i++) {
        digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(10);
        digitalWrite(I2C_SCL_PIN, LOW);  delayMicroseconds(10);
        if (digitalRead(I2C_SDA_PIN) == HIGH) {
            Serial.print("OK dopo "); Serial.print(i + 1); Serial.println(" impulsi");
            break;
        }
    }
    pinMode(I2C_SDA_PIN, OUTPUT);
    digitalWrite(I2C_SDA_PIN, LOW);  delayMicroseconds(10);
    digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(I2C_SDA_PIN, HIGH); delayMicroseconds(10);
    pinMode(I2C_SDA_PIN, INPUT);
    pinMode(I2C_SCL_PIN, INPUT);
    delay(I2C_POST_RECOVERY_MS);
}

// =====================================================================
// I2C SCAN
// =====================================================================
uint8_t i2cScan(void) {
    uint8_t found = 0;
    Serial.println("[I2C] Scanning bus...");
    for (uint8_t addr = 0x01; addr < 0x7F; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            found++;
            Serial.print("[I2C]   0x");
            if (addr < 16) Serial.print("0");
            Serial.print(addr, HEX); Serial.print(" -> ");
            if      (addr == RTC_I2C_ADDR)    Serial.println("RV-3028 RTC");
            else if (addr == BME680_I2C_ADDR) Serial.println("BME680");
            else if (addr == NH3_I2C_ADDR)    Serial.println("DFRobot NH3");
            else if (addr == H2S_I2C_ADDR)    Serial.println("DFRobot H2S");
            else                               Serial.println("SCONOSCIUTO");
        }
        delay(5);
    }
    Serial.print("[I2C] Trovati: "); Serial.print(found); Serial.println(" device");
    return found;
}

// =====================================================================
// I2C INIT SICURO
// =====================================================================
bool i2cInitSafe(void) {
    i2cClockRecovery();
    Wire.begin();
    delay(I2C_POST_BEGIN_MS);
    uint8_t devCount = i2cScan();
    delay(I2C_BETWEEN_DEVICES_MS);
    return (devCount > 0);
}

// =====================================================================
// SPI INIT PER FLASH
// =====================================================================
void spiInitForFlash(void) {
    SPI.begin();
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    delay(SPI_POST_INIT_MS);
}

// =====================================================================
// POWER CONTROL
// =====================================================================
void sensorPowerOn(void) {
    pinMode(SENSOR_POWER_PIN, OUTPUT);
    digitalWrite(SENSOR_POWER_PIN, HIGH);
    delay(POWER_ON_SETTLE_MS);
    Serial.println("[PWR] 3V3_S ON");
}

void sensorPowerOff(void) {
    bmeReset();
    gasReset();
    delay(50);
    digitalWrite(SENSOR_POWER_PIN, LOW);
    delay(50);
    Serial.println("[PWR] 3V3_S OFF");
}

void sensorPowerCycle(void) {
    Serial.println("[PWR] Power cycle 3V3_S...");
    digitalWrite(SENSOR_POWER_PIN, LOW);
    delay(POWER_CYCLE_OFF_MS);
    digitalWrite(SENSOR_POWER_PIN, HIGH);
    delay(POWER_CYCLE_ON_MS);
    Serial.println("[PWR] Power cycle completato");
}

// =====================================================================
// FLASH WRITE SICURA
// =====================================================================
void flashWriteSafe(const char *record) {
    Serial.println("[FLASH] === Inizio scrittura sicura ===");
    delay(SPI_BUS_SWITCH_MS);
    spiInitForFlash();
    flashLogWake();
    delay(SPI_POST_WAKE_MS);
    flashLogWrite(record);
    flashWriteCount++;
    delay(SPI_POST_WRITE_MS);
    delay(SPI_PRE_SLEEP_MS);
    flashLogSleep();
    delay(SPI_POST_WAKE_MS);
    Serial.println("[FLASH] === Scrittura completata ===");
}

// =====================================================================
// PREHEAT ATTIVO
// =====================================================================
void doPreheatActive(void) {
    uint32_t preheatStart = millis();
    uint32_t lastLog  = 0;
    uint32_t elapsed  = 0;

    Serial.println();
    Serial.println("=============================================");
    Serial.print("  PREHEAT ATTIVO: "); Serial.print(PREHEAT_MS / 1000); Serial.println(" secondi");
    Serial.println("  MCU sveglia -- AT disponibili");
    Serial.println("=============================================");
    Serial.println();

    while (true) {
        elapsed = millis() - preheatStart;
        if (elapsed >= PREHEAT_MS) {
            Serial.println();
            Serial.println("[PREHEAT] === COMPLETATO ===");
            Serial.println();
            break;
        }
        if (elapsed - lastLog >= PREHEAT_LOG_INTERVAL_MS) {
            uint32_t remaining = (PREHEAT_MS - elapsed) / 1000;
            Serial.print("[PREHEAT] "); Serial.print(elapsed / 1000);
            Serial.print("s / "); Serial.print(PREHEAT_MS / 1000);
            Serial.print("s -- rimanenti: "); Serial.print(remaining); Serial.println("s");
            lastLog = elapsed;
        }
        if (flash_paused) {
            Serial.println("[PREHEAT] Pausa AT attiva");
            while (flash_paused) { delay(100); }
            Serial.println("[PREHEAT] Ripreso");
            preheatStart = millis() - elapsed;
        }
        delay(100);
    }
}

// =====================================================================
// JSON LOG
// =====================================================================
void buildJsonLog(const char *timestamp, const char *phase) {
    snprintf(jsonLogBuf, sizeof(jsonLogBuf),
        "{\"c\":%lu,\"ts\":\"%s\",\"st\":\"%s\","
        "\"bat\":{\"mv\":%u,\"soc\":%u},"
        "\"bme\":{\"t\":%.1f,\"h\":%.1f,\"p\":%.1f,\"v\":%lu},"
        "\"nh3\":%.2f,\"h2s\":%.2f,"
        "\"mem\":{\"free\":%lu,\"min\":%lu},"
        "\"tx\":%s,\"rtcSync\":%s,"
        "\"sd\":{\"e\":%lu,\"x\":%lu}}",
        sdiag_getCycleCount(), timestamp, phase,
        battery_mv, battery_soc,
        bmeData.temperature, bmeData.humidity, bmeData.pressure,
        (unsigned long)bmeData.voc_resistance,
        gasData.nh3_ppm, gasData.h2s_ppm,
        (unsigned long)memGetFreeHeapForLog(), (unsigned long)memGetMinHeapForLog(),
        (flag_txStatus == 0) ? "true" : "false",
        rtcSynced ? "true" : "false",
        (unsigned long)sdiag_getSleepEnter(),
        (unsigned long)sdiag_getSleepExit());
}

// =====================================================================
// REBOOT PREVENTIVO
// =====================================================================
void logRebootAndReset(void) {
    Serial.println();
    Serial.println("=============================================");
    Serial.print("  REBOOT PREVENTIVO -- ciclo #"); Serial.println(sdiag_getCycleCount());
    Serial.println("=============================================");

    snprintf(jsonLogBuf, sizeof(jsonLogBuf),
        "{\"c\":%lu,\"ts\":\"%s\",\"st\":\"SCHEDULED_REBOOT\","
        "\"bat\":{\"mv\":%u,\"soc\":%u},"
        "\"mem\":{\"free\":%lu,\"min\":%lu},"
        "\"flash_writes\":%lu,\"rtcSyncCount\":%lu,"
        "\"sd\":{\"e\":%lu,\"x\":%lu}}",
        sdiag_getCycleCount(), rtcGetTimestamp(),
        battery_mv, battery_soc,
        (unsigned long)memGetFreeHeapForLog(), (unsigned long)memGetMinHeapForLog(),
        (unsigned long)flashWriteCount, (unsigned long)rtcSyncCount,
        (unsigned long)sdiag_getSleepEnter(), (unsigned long)sdiag_getSleepExit());
    flashWriteSafe(jsonLogBuf);

    sensorPowerOff();
    Serial.flush();
    delay(2000);
    NVIC_SystemReset();
}

// =====================================================================
// AT COMMAND HANDLERS
// =====================================================================
int infoGasHandler(SERIAL_PORT port, char *cmd, stParam *param) {
    (void)port; (void)cmd; (void)param;

    Serial.println();
    Serial.println("=============================================");
    Serial.println("  ATC+INFO_GAS - DIAGNOSTICA v7.2a");
    Serial.println("=============================================");
    Serial.println();

    Serial.println("{\"firmware\":{");
    Serial.print("  \"version\":\""); Serial.print(FW_VERSION); Serial.println("a\",");
    Serial.print("  \"arch\":\""); Serial.print(FW_ARCH); Serial.println("\",");
    Serial.print("  \"board\":\""); Serial.print(FW_BOARD); Serial.println("\",");
    Serial.print("  \"modules\":\""); Serial.print(FW_MODULES); Serial.println("\",");
    Serial.print("  \"drivers\":\""); Serial.print(FW_DRIVERS); Serial.println("\",");
    Serial.print("  \"cycle_count\":"); Serial.print(sdiag_getCycleCount()); Serial.println(",");
    Serial.print("  \"successful_tx\":"); Serial.print(diag.successfulTx); Serial.println(",");
    Serial.print("  \"tx_fail_consec\":"); Serial.print(tx_fail_count_consec); Serial.println(",");
    Serial.print("  \"wdt_triggered\":"); Serial.print(diag.wdtTriggered); Serial.println(",");
    Serial.print("  \"flash_writes\":"); Serial.print(flashWriteCount); Serial.println(",");
    Serial.print("  \"rtc_synced\":"); Serial.print(rtcSynced ? "true" : "false"); Serial.println(",");
    Serial.print("  \"rtc_sync_count\":"); Serial.print(rtcSyncCount); Serial.println(",");
    Serial.print("  \"reboot_every\":"); Serial.print(REBOOT_EVERY_N_CYCLES); Serial.println(",");
    Serial.print("  \"cycle_ms\":"); Serial.print(CYCLE_INTERVAL_MS); Serial.println(",");
    Serial.print("  \"preheat_ms\":"); Serial.println(PREHEAT_MS);
    Serial.println("}}");
    Serial.println();

    sdiag_printStatus();
    Serial.println();

    Serial.print("{\"rtc\":\""); Serial.print(rtcGetTimestamp()); Serial.println("\"}");
    Serial.println();

    uint16_t bat = readBatteryMV();
    Serial.print("{\"battery\":{\"mv\":"); Serial.print(bat);
    Serial.print(",\"soc\":"); Serial.print(estimateSOC(bat)); Serial.println("}}");
    Serial.println();

    memPrintInfo("ATC+INFO_GAS");

    Serial.print("{\"lorawan\":{\"joined\":"); Serial.print(api.lorawan.njs.get() ? "true" : "false");
    Serial.print(",\"dr\":"); Serial.print(api.lorawan.dr.get());
    Serial.print(",\"adr\":"); Serial.print(api.lorawan.adr.get() ? "true" : "false");
    Serial.println("}}");
    Serial.println();

    Serial.println("[FLASH] Dump in corso...");
    delay(SPI_BUS_SWITCH_MS);
    spiInitForFlash();
    flashLogWake();
    delay(SPI_POST_WAKE_MS);
    flashLogDump();
    delay(SPI_PRE_SLEEP_MS);
    flashLogSleep();
    delay(SPI_POST_WAKE_MS);

    flash_paused = true;
    Serial.println();
    Serial.println("  *** CICLO IN PAUSA ***");
    Serial.println("  ATC+RESUME_GAS per riprendere");
    Serial.println("  ATC+CLEARLOG_GAS per cancellare log");
    Serial.println();
    return AT_OK;
}

int resumeGasHandler(SERIAL_PORT port, char *cmd, stParam *param) {
    (void)port; (void)cmd; (void)param;
    flash_paused = false;
    Serial.println("[AT] Ciclo ripreso!");
    return AT_OK;
}

int clearlogGasHandler(SERIAL_PORT port, char *cmd, stParam *param) {
    (void)port; (void)cmd; (void)param;
    Serial.println("[AT] Cancellazione flash log...");
    Serial.println("[AT] Chip erase in corso (attendere 20-40s)...");
    delay(SPI_BUS_SWITCH_MS);
    spiInitForFlash();
    flashLogWake();
    delay(SPI_POST_WAKE_MS);
    flashLogClear();
    delay(SPI_POST_ERASE_MS);
    flashLogSleep();
    delay(SPI_PRE_SLEEP_MS);
    flashWriteCount = 0;

    delay(SPI_BUS_SWITCH_MS);
    spiInitForFlash();
    flashLogWake();
    delay(SPI_POST_WAKE_MS);
    sdiag_clear();
    delay(SPI_POST_WRITE_MS);
    flashLogSleep();
    delay(SPI_PRE_SLEEP_MS);
    Serial.println("[AT] Log e diagnostica cancellati!");
    return AT_OK;
}

int settimeGasHandler(SERIAL_PORT port, char *cmd, stParam *param) {
    (void)port; (void)cmd;
    if (param->argc != 6) {
        Serial.println("[AT] Formato: ATC+SETTIME_GAS=YYYY,MM,DD,HH,MM,SS");
        return AT_PARAM_ERROR;
    }
    rtcSetTime(atoi(param->argv[0]), atoi(param->argv[1]), atoi(param->argv[2]),
               atoi(param->argv[3]), atoi(param->argv[4]), atoi(param->argv[5]));
    Serial.println("[AT] RTC impostato!");
    return AT_OK;
}

void registerATCommands(void) {
    bool ok;
    ok = api.system.atMode.add("INFO_GAS",
        "Diagnostica completa v7.2a", "INFO_GAS", infoGasHandler, RAK_ATCMD_PERM_READ);
    Serial.print("[AT] ATC+INFO_GAS: "); Serial.println(ok ? "OK" : "FAIL");

    ok = api.system.atMode.add("RESUME_GAS",
        "Riprende il ciclo", "RESUME_GAS", resumeGasHandler, RAK_ATCMD_PERM_READ);
    Serial.print("[AT] ATC+RESUME_GAS: "); Serial.println(ok ? "OK" : "FAIL");

    ok = api.system.atMode.add("CLEARLOG_GAS",
        "Cancella flash log", "CLEARLOG_GAS", clearlogGasHandler, RAK_ATCMD_PERM_READ);
    Serial.print("[AT] ATC+CLEARLOG_GAS: "); Serial.println(ok ? "OK" : "FAIL");

    ok = api.system.atMode.add("SETTIME_GAS",
        "Imposta RTC", "SETTIME_GAS", settimeGasHandler, RAK_ATCMD_PERM_WRITE);
    Serial.print("[AT] ATC+SETTIME_GAS: "); Serial.println(ok ? "OK" : "FAIL");
    Serial.println();
}

// =====================================================================
// SETUP
// =====================================================================
void setup()
{
    Serial.begin(115200);
    api.system.lpm.set(1);
    delay(BOOT_SERIAL_INIT_MS);

    Serial.println();
    Serial.println("=============================================");
    Serial.print("  "); Serial.println(FW_NAME);
#if RECOVERY_MODE
    Serial.println("  *** RECOVERY MODE v7.2a ***");
#else
    Serial.print("  FIRMWARE v"); Serial.print(FW_VERSION); Serial.println("a -- EVENT-DRIVEN");
#endif
    Serial.print("  "); Serial.println(FW_BOARD);
    Serial.print("  "); Serial.println(FW_MODULES);
    Serial.print("  Drivers: "); Serial.println(FW_DRIVERS);
    Serial.println("  LoRaWAN OTAA - EU868 - fPort 77");
    Serial.print("  Ciclo: "); Serial.print(SLEEP_MINUTES);
    Serial.print(" min sleep + "); Serial.print(PREHEAT_MINUTES); Serial.println(" min preheat");
    Serial.print("  Architettura: "); Serial.println(FW_ARCH);
    Serial.println("  Preheat: MCU SVEGLIA (no sleep)");
    Serial.println("  Flash write: SOLO dopo TX_DONE");
    Serial.println("  Time Sync: downlink LoRaWAN fPort 10");
    Serial.println("  NO HW Watchdog (reboot preventivo come safety)");
    Serial.print("  Reboot ogni "); Serial.print(REBOOT_EVERY_N_CYCLES);
    Serial.print(" cicli (~"); Serial.print((uint32_t)REBOOT_EVERY_N_CYCLES * CYCLE_TOTAL_MIN / 60 / 24);
    Serial.println(" giorni)");
    Serial.println("=============================================");
    Serial.println();

    printBootDiagnostic();

    registerATCommands();

    memSetBaseline();
    memPrintInfo("BOOT");

    uint16_t bootBat = readBatteryMV();
    Serial.print("[BOOT] Alimentazione: "); Serial.print(bootBat); Serial.println(" mV");
    if (bootBat < 3300 || bootBat > 4500) {
        Serial.println("[BOOT] Solo USB -- delay stabilizzazione");
        delay(BOOT_USB_SETTLE_MS);
    }

    sensorPowerOn();
    bool i2c_ok = i2cInitSafe();

    if (!i2c_ok) {
        Serial.println("[BOOT] Nessun device I2C -- power cycle e retry");
        sensorPowerCycle();
        i2c_ok = i2cInitSafe();
    }

#if RECOVERY_MODE
    Serial.println("[RECOVERY] Modalita' recovery attiva");
    Serial.println("[RECOVERY] Init flash SPI...");
    delay(SPI_BUS_SWITCH_MS);
    spiInitForFlash();
    flashLogInit();
    Serial.println("[RECOVERY] === DUMP FLASH LOG ===");
    flashLogWake();
    delay(SPI_POST_WAKE_MS);
    flashLogDump();
    delay(SPI_PRE_SLEEP_MS);
    flashLogSleep();
    Serial.println("[RECOVERY] === FINE DUMP ===");
    flash_paused = true;
    Serial.println("[RECOVERY] In attesa comandi AT...");
    return;
#endif

    rtcInit();
    delay(I2C_BETWEEN_DEVICES_MS);
    rtcAutoSetFromCompileTime();
    rtcPrintTime();
    Serial.println("[RTC] Time sync via LoRaWAN downlink attivo (fPort 10)");
    Serial.println();

    Serial.println("[FLASH] Init SPI flash...");
    delay(SPI_BUS_SWITCH_MS);
    spiInitForFlash();
    flashLogInit();
    delay(SPI_POST_INIT_MS);

    sdiag_init();
    Serial.println();

    if (sdiag_hadSleepMismatch()) {
        snprintf(jsonLogBuf, sizeof(jsonLogBuf),
            "{\"c\":%lu,\"ts\":\"%s\",\"st\":\"SLEEP_MISMATCH\","
            "\"sd\":{\"e\":%lu,\"x\":%lu,\"missed\":%lu}}",
            sdiag_getCycleCount(), rtcGetTimestamp(),
            (unsigned long)sdiag_getSleepEnter(),
            (unsigned long)sdiag_getSleepExit(),
            (unsigned long)(sdiag_getSleepEnter() - sdiag_getSleepExit()));
        flashWriteSafe(jsonLogBuf);
        Serial.println("[FLASH] Loggato SLEEP_MISMATCH!");
    }

    delay(SPI_PRE_SLEEP_MS);
    flashLogSleep();
    delay(SPI_POST_WAKE_MS);
    Serial.println();

    sensorPowerOff();
    memPrintInfo("POST-INIT");

    api.lorawan.registerJoinCallback(cb_joinResult);
    api.lorawan.registerSendCallback(cb_sendResult);
    api.lorawan.registerRecvCallback(cb_recvData);
    Serial.println("[LORA] Callback registrate (join, send, recv + timesync)");

    bool t0 = api.system.timer.create(RAK_TIMER_0, cb_timerCycle, RAK_TIMER_PERIODIC);
    bool t2 = api.system.timer.create(RAK_TIMER_2, cb_timerJoinRetry, RAK_TIMER_ONESHOT);
    Serial.print("[TIMER] RAK_TIMER_0 (ciclo "); Serial.print(CYCLE_INTERVAL_MS/1000); Serial.print("s): "); Serial.println(t0 ? "OK" : "FAIL");
    Serial.print("[TIMER] RAK_TIMER_2 (join retry): "); Serial.println(t2 ? "OK" : "FAIL");
    Serial.println();

    loraPrintDeviceInfo();
    loraInit();

    joinRetryCount       = 0;
    tx_fail_count_consec = 0;
    flashWriteCount      = 0;
    pendingFlashWrite    = false;
    rtcSynced            = false;
    rtcSyncCount         = 0;

    Serial.println("[JOIN] Avvio join OTAA...");
    api.lorawan.join();

    Serial.println();
    Serial.println("[SETUP] Completato -- entro in event loop");
    Serial.println("=============================================");
    Serial.println();
}


// =====================================================================
// LOOP -- EVENT-DRIVEN v7.2a (NO WDT)
// =====================================================================
void loop()
{
    if (flash_paused) {
        delay(100);
        return;
    }

    // =================================================================
    // EVENTO 1: JOIN COMPLETATO
    // =================================================================
    if (flag_joinDone) {
        flag_joinDone = false;

        if (api.lorawan.njs.get()) {
            joined = true;
            joinRetryCount = 0;
            Serial.println("[JOIN] Connesso al network!");
            delay(200);
            api.lorawan.dr.set(LORAWAN_DR_TX);
            Serial.println("[JOIN] DR impostato a SF7");
            memPrintInfo("POST-JOIN");

            Serial.print("[TIMER] Avvio TIMER_0: ");
            Serial.print(CYCLE_INTERVAL_MS / 1000); Serial.println("s");
            api.system.timer.start(RAK_TIMER_0, CYCLE_INTERVAL_MS, NULL);

            flag_startCycle = true;
            Serial.println("[LOOP] Primo ciclo forzato");
        } else {
            joined = false;
            joinRetryCount++;
            Serial.print("[JOIN] FALLITO -- tentativo ");
            Serial.print(joinRetryCount); Serial.print("/"); Serial.println(MAX_JOIN_RETRIES);
            if (joinRetryCount >= MAX_JOIN_RETRIES) {
                Serial.println("[JOIN] Max tentativi -- riprovo tra 2 min");
                joinRetryCount = 0;
            }
            api.system.timer.start(RAK_TIMER_2, JOIN_RETRY_MS, NULL);
        }
    }

    // =================================================================
    // EVENTO 2: AVVIO CICLO
    // =================================================================
    if (flag_startCycle) {
        flag_startCycle = false;

        // FASE 1: Accensione e init
        sensorPowerOn();
        delay(100);
        i2cInitSafe();

        // Incrementa ciclo su flash
        delay(SPI_BUS_SWITCH_MS);
        spiInitForFlash();
        flashLogWake();
        delay(SPI_POST_WAKE_MS);
        sdiag_incrementCycle();
        delay(SPI_POST_WRITE_MS);
        flashLogSleep();
        delay(SPI_PRE_SLEEP_MS);

        Serial.println();
        Serial.println("=============================================");
        Serial.print("  CICLO #"); Serial.println(sdiag_getCycleCount());
        Serial.print("  "); Serial.println(rtcGetTimestamp());
        Serial.print("  RTC sync: "); Serial.println(rtcSynced ? "SI" : "NO (attesa downlink)");
        Serial.println("=============================================");
        Serial.println();

        // Reboot preventivo
        if (sdiag_getCycleCount() > 0 &&
            (sdiag_getCycleCount() % REBOOT_EVERY_N_CYCLES) == 0) {
            logRebootAndReset();
        }

        // Batteria
        battery_mv  = readBatteryMV();
        battery_soc = estimateSOC(battery_mv);
        Serial.print("[BAT] "); Serial.print(battery_mv);
        Serial.print(" mV ("); Serial.print(battery_soc); Serial.println("%)");

        if (battery_mv < BATTERY_MIN_MV && battery_mv > 0) {
            Serial.println("[BAT] TENSIONE CRITICA -- salto ciclo");
            snprintf(jsonLogBuf, sizeof(jsonLogBuf),
                "{\"c\":%lu,\"ts\":\"%s\",\"st\":\"LOW_BAT\",\"bat\":{\"mv\":%u}}",
                sdiag_getCycleCount(), rtcGetTimestamp(), battery_mv);
            flashWriteSafe(jsonLogBuf);
            sensorPowerOff();
            goto end_cycle;
        }

        // FASE 2: Preheat ATTIVO
        doPreheatActive();

        // FASE 3: Init sensori
        Serial.println("[SENSORI] Init dopo preheat...");
        delay(I2C_BETWEEN_DEVICES_MS);
        bmeInit();
        delay(I2C_BETWEEN_DEVICES_MS);
        gasInitAll();
        delay(I2C_BETWEEN_DEVICES_MS);

        // FASE 4: Lettura sensori
        Serial.println();
        Serial.println("--- LETTURA SENSORI ---");
        bmeData = bmeRead();
        delay(I2C_POST_SENSOR_READ_MS);
        gasData = gasRead();
        delay(I2C_POST_SENSOR_READ_MS);

        battery_mv  = readBatteryMV();
        battery_soc = estimateSOC(battery_mv);
        Serial.print("[BAT] "); Serial.print(battery_mv);
        Serial.print(" mV ("); Serial.print(battery_soc); Serial.println("%)");

        cycleTimestamp = rtcGetTimestamp();
        Serial.print("[RTC] "); Serial.println(cycleTimestamp);

        memUpdateMin();
        memPrintInfo("PRE-TX");

        // FASE 5: Payload
        loraBuildPayload(
            bmeData.temperature, bmeData.humidity, bmeData.pressure,
            bmeData.voc_resistance,
            gasData.nh3_ppm, gasData.h2s_ppm,
            battery_mv, battery_soc);

        // Sleep diag pre-TX
        delay(SPI_BUS_SWITCH_MS);
        spiInitForFlash();
        flashLogWake();
        delay(SPI_POST_WAKE_MS);
        sdiag_incrementSleepEnter();
        delay(SPI_POST_WRITE_MS);
        flashLogSleep();
        delay(SPI_PRE_SLEEP_MS);

        // FASE 6: TX LoRaWAN
        delay(SPI_BUS_SWITCH_MS);
        Serial.print("[LORA] TX 19 bytes fPort "); Serial.println(LORAWAN_FPORT);

        if (!api.lorawan.send(sizeof(payload), payload, LORAWAN_FPORT, false)) {
            Serial.println("[LORA] send() fallito");
            delay(SPI_BUS_SWITCH_MS);
            spiInitForFlash();
            flashLogWake();
            delay(SPI_POST_WAKE_MS);
            sdiag_incrementSleepExit();
            delay(SPI_POST_WRITE_MS);
            flashLogSleep();
            delay(SPI_PRE_SLEEP_MS);

            tx_fail_count_consec++;
            buildJsonLog(cycleTimestamp, "TX_FAIL_SEND");
            flashWriteSafe(jsonLogBuf);
            sensorPowerOff();
            pendingFlashWrite = false;

            if (!api.lorawan.njs.get()) {
                Serial.println("[LORA] Sessione persa -> rejoin");
                joined = false;
                joinRetryCount = 0;
                api.lorawan.join();
            }
        } else {
            Serial.println("[LORA] TX avviata (attendo TX_DONE + downlink timesync)");
            pendingFlashWrite = true;
            strncpy(pendingPhase, "TX_OK", sizeof(pendingPhase));
        }
    }

    // =================================================================
    // EVENTO 3: TX COMPLETATA
    // =================================================================
    if (flag_txDone) {
        flag_txDone = false;

        delay(RADIO_POST_TX_MS);

        // Sleep diag post-TX
        delay(SPI_BUS_SWITCH_MS);
        spiInitForFlash();
        flashLogWake();
        delay(SPI_POST_WAKE_MS);
        sdiag_incrementSleepExit();
        delay(SPI_POST_WRITE_MS);
        flashLogSleep();
        delay(SPI_PRE_SLEEP_MS);

        if (flag_txStatus == 0) {
            Serial.println("[LORA] TX OK!");
            diag.successfulTx++;
            diag.lastTxResult = 1;
            tx_fail_count_consec = 0;
            strncpy(pendingPhase, "TX_OK", sizeof(pendingPhase));
        } else {
            Serial.print("[LORA] TX FAIL (status="); Serial.print(flag_txStatus); Serial.println(")");
            diag.lastTxResult = 0;
            tx_fail_count_consec++;
            diag.lastTxFailCount = tx_fail_count_consec;
            strncpy(pendingPhase, "TX_FAIL", sizeof(pendingPhase));

            if (tx_fail_count_consec >= MAX_TX_FAILURES_CONSEC) {
                Serial.println("[LORA] TX fail consecutivi -> rejoin");
                if (!api.lorawan.njs.get()) {
                    joined = false;
                    joinRetryCount = 0;
                    api.lorawan.join();
                }
                tx_fail_count_consec = 0;
            }
        }

        // Flash write post-TX (radio idle)
        if (pendingFlashWrite) {
            buildJsonLog(cycleTimestamp, pendingPhase);
            Serial.print("[FLASH] Scrittura post-TX: "); Serial.println(pendingPhase);
            flashWriteSafe(jsonLogBuf);
            Serial.print("[FLASH] "); Serial.println(jsonLogBuf);
            pendingFlashWrite = false;
        }

        // Spegni sensori
        sensorPowerOff();

        // Diagnostica
        memUpdateMin();
        Serial.print("[MEM] Free: "); Serial.print(memGetFreeHeap());
        Serial.print(" | Min: "); Serial.println(memGetMinHeapForLog());

        Serial.println();
        Serial.println("-------------------------------------------");
        Serial.print("[SLEEP] Prossimo ciclo tra ~");
        Serial.print(CYCLE_INTERVAL_MS / 60000); Serial.println(" min");
        Serial.println("-------------------------------------------");
        Serial.flush();
        delay(100);
    }

    // =================================================================
    // end_cycle label (per LOW_BAT skip)
    // =================================================================
    end_cycle:

    // =================================================================
    // TORNA A DORMIRE
    // =================================================================
    api.system.sleep.all();
}


