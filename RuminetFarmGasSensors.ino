
// =====================================================================
// GAS_Sensor_v7.0.ino - EVENT-DRIVEN + REBOOT PREVENTIVO + DIAG SLEEP
// =====================================================================
// MYRUMINET - Sensore Gas Stalla
// RAK3172-E / RAK19007 Rev.C
//
// Moduli:
//   Slot A: RAK1906 (BME680) - I2C 0x76
//   Slot B: RAK15001 (GD25Q16C Flash 2MB) - SPI CS=SS(PA4)
//   Slot C: RAK12002 (RV-3028 RTC) - I2C 0x52
//   RAK1920: DFRobot NH3 (0x77) + H2S (0x74)
//
// LoRaWAN OTAA - EU868 - Class A - fPort 77
//
// ARCHITETTURA v7.0: 100% EVENT-DRIVEN (pattern RAK ufficiale)
//   - Callback settano SOLO flag volatile
//   - Lavoro pesante (I2C, SPI, sensori) nel loop()
//   - api.system.sleep.all() SEMPRE ultima istruzione del loop
//   - ZERO race condition tra TX_DONE e sleep
//   - MCU dorme anche durante il preheat (3 min)
//
// TIMER RUI3:
//   RAK_TIMER_0: PERIODIC - ciclo principale (30 min)
//   RAK_TIMER_1: ONESHOT  - fine preheat (3 min)
//   RAK_TIMER_2: ONESHOT  - retry join (2 min)
//
// SAFETY NET:
//   - Reboot preventivo ogni 120 cicli (~60h = 2.5 giorni)
//   - Diagnostica sleep persistente su flash (sleep_diag.h)
//   - SW watchdog per operazioni I2C
//   - NVIC_SystemReset() per reboot pulito
//
// Comandi AT custom (prefisso ATC+):
//   ATC+INFO_GAS       -> diagnostica + dump flash + pausa
//   ATC+RESUME_GAS     -> riprende il ciclo
//   ATC+CLEARLOG_GAS   -> cancella flash log + diagnostica sleep
//   ATC+SETTIME_GAS=YYYY,MM,DD,HH,MM,SS -> imposta RTC
//
// Board Arduino IDE: RAK3172-E
// Baud: 115200
// =====================================================================

#include <Wire.h>
#include "config.h"
#include "diagnostics.h"
#include "memory_monitor.h"
#include "watchdog_hw.h"
#include "rtc_time.h"
#include "flash_log.h"
#include "sleep_diag.h"
#include "bme680_sensor.h"
#include "gas_sensors.h"
#include "lorawan_manager.h"


// =====================================================================
// CONFIGURAZIONE v7.0
// =====================================================================

#ifndef RECOVERY_MODE
#define RECOVERY_MODE           false
#endif

#ifndef I2C_TIMEOUT_MS
#define I2C_TIMEOUT_MS          3000
#endif

#ifndef REBOOT_EVERY_N_CYCLES
#define REBOOT_EVERY_N_CYCLES   120       // ~60h = 2.5 giorni
#endif

#ifndef SWWDT_TIMEOUT_MS
#define SWWDT_TIMEOUT_MS        30000     // 30s timeout per operazioni I2C
#endif

#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN             PA11
#endif
#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN             PA12
#endif

// Intervalli timer (in millisecondi)
#define CYCLE_INTERVAL_MS       ((uint32_t)SLEEP_MINUTES * 60UL * 1000UL)    // 30 min = 1800000 ms
#define PREHEAT_MS              ((uint32_t)PREHEAT_MINUTES * 60UL * 1000UL)  // 3 min = 180000 ms
#define JOIN_RETRY_MS           120000UL  // 2 min tra retry join
#define LONG_SLEEP_MS           120000UL  // 2 min sleep dopo errori gravi
#define JOIN_TIMEOUT_MS         90000UL   // 90s timeout join

#define MAX_JOIN_RETRIES        5
#define MAX_TX_FAILURES_CONSEC  3
#define MAX_ERROR_COUNT         3


// =====================================================================
// FLAG VOLATILI — settati dalle callback, letti nel loop()
// =====================================================================

volatile bool flag_startCycle   = false;
volatile bool flag_preheatDone  = false;
volatile bool flag_txDone       = false;
volatile bool flag_joinDone     = false;
volatile int  flag_txStatus     = 0;      // 0 = RAK_LORAMAC_STATUS_OK


// =====================================================================
// DATI CICLO — condivisi tra le fasi del loop()
// =====================================================================

static BmeData  bmeData         = {0.0f, 0.0f, 0.0f, 0};
static GasData  gasData         = {0.0f, 0.0f};
static uint16_t battery_mv      = 0;
static uint8_t  battery_soc     = 0;
static bool     sensors_ok      = false;
static char     jsonLogBuf[FLASH_RECORD_SIZE];
static const char *cycleTimestamp = "";
static uint32_t flashWriteCount = 0;
static uint8_t  joinRetryCount  = 0;
static uint8_t  tx_fail_count_consec = 0;


// =====================================================================
// CALLBACK LoRaWAN — SOLO FLAG, ZERO lavoro pesante
// =====================================================================

void cb_joinResult(int32_t status)
{
    flag_joinDone = true;
    Serial.print("[CB] Join result: ");
    Serial.println(status);
}

void cb_sendResult(int32_t status)
{
    flag_txDone   = true;
    flag_txStatus = status;
    Serial.print("[CB] TX result: ");
    Serial.println(status);
}

void cb_recvData(SERVICE_LORA_RECEIVE_T *data)
{
    if (data->BufferSize > 0) {
        Serial.print("[CB] Downlink ricevuto, ");
        Serial.print(data->BufferSize);
        Serial.print(" bytes, fPort ");
        Serial.println(data->Port);
        // Futuro: gestione comandi remoti
    }
}


// =====================================================================
// CALLBACK TIMER — SOLO FLAG, ZERO lavoro pesante
// =====================================================================

void cb_timerCycle(void *)
{
    flag_startCycle = true;
    Serial.println("[CB] TIMER_0: avvio ciclo");
}

void cb_timerPreheat(void *)
{
    flag_preheatDone = true;
    Serial.println("[CB] TIMER_1: preheat completato");
}

void cb_timerJoinRetry(void *)
{
    Serial.println("[CB] TIMER_2: retry join");
    api.lorawan.join();
}


// =====================================================================
// SOFTWARE WATCHDOG
// =====================================================================

static uint32_t swWdtStartMs   = 0;
static uint32_t swWdtTimeoutMs = 0;
static bool     swWdtActive    = false;

void swWatchdogStart(uint32_t timeout_ms)
{
    swWdtStartMs   = millis();
    swWdtTimeoutMs = timeout_ms;
    swWdtActive    = true;
}

void swWatchdogStop(void)
{
    swWdtActive = false;
}

void swWatchdogCheck(void)
{
    if (!swWdtActive) return;

    uint32_t elapsed = millis() - swWdtStartMs;
    if (elapsed >= swWdtTimeoutMs) {
        Serial.println();
        Serial.println("[SWWDT] !!! TIMEOUT SCADUTO !!!");
        Serial.print("[SWWDT] Fase durata ");
        Serial.print(elapsed / 1000);
        Serial.println("s -- REBOOT FORZATO");

        snprintf(jsonLogBuf, sizeof(jsonLogBuf),
            "{\"c\":%lu,\"ts\":\"%s\",\"st\":\"SWWDT_REBOOT\"}",
            sdiag_getCycleCount(), rtcGetTimestamp());
        flashLogWriteSafe(jsonLogBuf);

        Serial.flush();
        delay(100);
        NVIC_SystemReset();
    }
}


// =====================================================================
// I2C CLOCK RECOVERY
// =====================================================================

void i2cClockRecovery(void)
{
    Serial.print("[I2C] Clock recovery (9 impulsi SCL)... ");

    Wire.end();
    delay(10);

    pinMode(I2C_SCL_PIN, OUTPUT);
    pinMode(I2C_SDA_PIN, INPUT_PULLUP);

    if (digitalRead(I2C_SDA_PIN) == HIGH) {
        Serial.println("SDA gia' HIGH, bus OK");
        return;
    }

    Serial.print("SDA basso, recovery... ");
    for (int i = 0; i < 9; i++) {
        digitalWrite(I2C_SCL_PIN, HIGH);
        delayMicroseconds(5);
        digitalWrite(I2C_SCL_PIN, LOW);
        delayMicroseconds(5);

        if (digitalRead(I2C_SDA_PIN) == HIGH) {
            Serial.print("OK dopo ");
            Serial.print(i + 1);
            Serial.println(" impulsi");
            break;
        }
    }

    // STOP condition
    pinMode(I2C_SDA_PIN, OUTPUT);
    digitalWrite(I2C_SDA_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(I2C_SDA_PIN, HIGH);
    delayMicroseconds(5);

    pinMode(I2C_SDA_PIN, INPUT_PULLUP);
    delay(1);
    if (digitalRead(I2C_SDA_PIN) == HIGH) {
        Serial.println("[I2C] Recovery OK -- SDA rilasciato");
    } else {
        Serial.println("[I2C] Recovery FALLITA -- SDA ancora basso!");
    }
}


// =====================================================================
// I2C SCAN DIAGNOSTICO
// =====================================================================

uint8_t i2cScan(void)
{
    uint8_t found = 0;
    Serial.println("[I2C] Scanning bus...");

    for (uint8_t addr = 0x01; addr < 0x7F; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            found++;
            Serial.print("[I2C]   0x");
            if (addr < 16) Serial.print("0");
            Serial.print(addr, HEX);
            Serial.print(" -> ");
            if      (addr == RTC_I2C_ADDR)    Serial.println("RV-3028 RTC");
            else if (addr == BME680_I2C_ADDR) Serial.println("BME680");
            else if (addr == NH3_I2C_ADDR)    Serial.println("DFRobot NH3");
            else if (addr == H2S_I2C_ADDR)    Serial.println("DFRobot H2S");
            else                               Serial.println("SCONOSCIUTO");
        }
    }

    Serial.print("[I2C] Trovati: ");
    Serial.print(found);
    Serial.println(" device");
    return found;
}


// =====================================================================
// I2C INIT SICURO
// =====================================================================

bool i2cInitSafe(void)
{
    i2cClockRecovery();

    uint32_t startMs = millis();
    Wire.begin();
    uint32_t elapsed = millis() - startMs;

    Serial.print("[I2C] Wire.begin() completato in ");
    Serial.print(elapsed);
    Serial.println(" ms");

    if (elapsed > 1000) {
        Serial.println("[I2C] WARNING: Wire.begin() lento!");
    }

    delay(100);
    Serial.println("[I2C] Bus OK");

    uint8_t devCount = i2cScan();
    Serial.println();

    return (devCount > 0);
}


// =====================================================================
// INIT CON TIMEOUT DIAGNOSTICO
// =====================================================================

bool rtcInitWithTimeout(void)
{
    Serial.print("[RTC] Init... ");
    uint32_t startMs = millis();

    rtcInit();

    uint32_t elapsed = millis() - startMs;
    Serial.print("completato in ");
    Serial.print(elapsed);
    Serial.println(" ms");

    if (elapsed > I2C_TIMEOUT_MS) {
        Serial.println("[RTC] WARNING: init lento, possibile problema I2C");
        return false;
    }
    return true;
}

bool flashLogInitWithTimeout(void)
{
    Serial.print("[FLASH] Init GD25Q16C... ");
    uint32_t startMs = millis();

    flashLogInit();

    uint32_t elapsed = millis() - startMs;
    Serial.print("completato in ");
    Serial.print(elapsed);
    Serial.println(" ms");

    if (elapsed > I2C_TIMEOUT_MS) {
        Serial.println("[FLASH] WARNING: init lento");
        return false;
    }
    return true;
}


// =====================================================================
// POWER CONTROL SENSORI
// =====================================================================

void sensorPowerOn(void)
{
    pinMode(SENSOR_POWER_PIN, OUTPUT);
    digitalWrite(SENSOR_POWER_PIN, HIGH);
    delay(SENSOR_POWERUP_DELAY_MS);
    Serial.println("[PWR] 3V3_S ON");
}

void sensorPowerOff(void)
{
    bmeReset();
    gasReset();
    digitalWrite(SENSOR_POWER_PIN, LOW);
    Serial.println("[PWR] 3V3_S OFF");
}

void sensorPowerCycle(void)
{
    Serial.println("[PWR] Power cycle 3V3_S...");
    digitalWrite(SENSOR_POWER_PIN, LOW);
    delay(500);
    digitalWrite(SENSOR_POWER_PIN, HIGH);
    delay(SENSOR_POWERUP_DELAY_MS);
    Serial.println("[PWR] Power cycle completato");
}


// =====================================================================
// FLASH LOG — SCRITTURA SICURA
// =====================================================================

void flashLogWriteSafe(const char *record)
{
    flashLogWake();
    flashLogWrite(record);
    flashWriteCount++;

    if (flashWriteCount % 1000 == 0) {
        Serial.print("[FLASH] Scritture da boot: ");
        Serial.println(flashWriteCount);
    }

    flashLogSleep();
}


// =====================================================================
// JSON LOG PER FLASH
// =====================================================================

void buildJsonLog(const char *timestamp, const char *phase)
{
    snprintf(jsonLogBuf, sizeof(jsonLogBuf),
        "{\"c\":%lu,\"ts\":\"%s\",\"st\":\"%s\","
        "\"bat\":{\"mv\":%u,\"soc\":%u},"
        "\"bme\":{\"t\":%.1f,\"h\":%.1f,\"p\":%.1f,\"v\":%lu},"
        "\"nh3\":%.2f,\"h2s\":%.2f,"
        "\"mem\":{\"free\":%lu,\"min\":%lu},"
        "\"tx\":%s,"
        "\"sd\":{\"e\":%lu,\"x\":%lu}}",
        sdiag_getCycleCount(), timestamp, phase,
        battery_mv, battery_soc,
        bmeData.temperature, bmeData.humidity, bmeData.pressure,
        (unsigned long)bmeData.voc_resistance,
        gasData.nh3_ppm, gasData.h2s_ppm,
        (unsigned long)memGetFreeHeapForLog(), (unsigned long)memGetMinHeapForLog(),
        (flag_txStatus == 0) ? "true" : "false",
        (unsigned long)sdiag_getSleepEnter(),
        (unsigned long)sdiag_getSleepExit()
    );
}


// =====================================================================
// REBOOT PREVENTIVO — Log + Reset
// =====================================================================

void logRebootAndReset(void)
{
    Serial.println();
    Serial.println("=============================================");
    Serial.print("  REBOOT PREVENTIVO -- ciclo #");
    Serial.println(sdiag_getCycleCount());
    Serial.print("  Sleep enter/exit: ");
    Serial.print(sdiag_getSleepEnter());
    Serial.print("/");
    Serial.println(sdiag_getSleepExit());
    Serial.print("  Prossimo reboot al ciclo #");
    Serial.println(sdiag_getCycleCount() + REBOOT_EVERY_N_CYCLES);
    Serial.println("=============================================");

    snprintf(jsonLogBuf, sizeof(jsonLogBuf),
        "{\"c\":%lu,\"ts\":\"%s\",\"st\":\"SCHEDULED_REBOOT\","
        "\"bat\":{\"mv\":%u,\"soc\":%u},"
        "\"mem\":{\"free\":%lu,\"min\":%lu},"
        "\"flash_writes\":%lu,"
        "\"sd\":{\"e\":%lu,\"x\":%lu}}",
        sdiag_getCycleCount(), rtcGetTimestamp(),
        battery_mv, battery_soc,
        (unsigned long)memGetFreeHeapForLog(),
        (unsigned long)memGetMinHeapForLog(),
        (unsigned long)flashWriteCount,
        (unsigned long)sdiag_getSleepEnter(),
        (unsigned long)sdiag_getSleepExit());
    flashLogWriteSafe(jsonLogBuf);

    sensorPowerOff();
    Serial.flush();
    delay(2000);
    NVIC_SystemReset();
}


// =====================================================================
// AT COMMAND HANDLERS
// =====================================================================

int infoGasHandler(SERIAL_PORT port, char *cmd, stParam *param)
{
    (void)port; (void)cmd; (void)param;

    Serial.println();
    Serial.println("=============================================");
    Serial.println("  ATC+INFO_GAS - DIAGNOSTICA v7.0 EVENT-DRIVEN");
    Serial.println("=============================================");
    Serial.println();

    Serial.println("{\"firmware\":{");
    Serial.print("  \"version\":\""); Serial.print(FW_VERSION); Serial.println("\",");
    Serial.print("  \"arch\":\"EVENT-DRIVEN v7.0\",");
    Serial.println();
    Serial.print("  \"board\":\""); Serial.print(FW_BOARD); Serial.println("\",");
    Serial.print("  \"modules\":\""); Serial.print(FW_MODULES); Serial.println("\",");
    Serial.print("  \"cycle_count\":"); Serial.print(sdiag_getCycleCount()); Serial.println(",");
    Serial.print("  \"successful_tx\":"); Serial.print(diag.successfulTx); Serial.println(",");
    Serial.print("  \"tx_fail_consec\":"); Serial.print(tx_fail_count_consec); Serial.println(",");
    Serial.print("  \"wdt_triggered\":"); Serial.print(diag.wdtTriggered); Serial.println(",");
    Serial.print("  \"flash_writes\":"); Serial.print(flashWriteCount); Serial.println(",");
    Serial.print("  \"reboot_every\":"); Serial.print(REBOOT_EVERY_N_CYCLES); Serial.println(",");
    Serial.print("  \"cycle_ms\":"); Serial.print(CYCLE_INTERVAL_MS); Serial.println(",");
    Serial.print("  \"preheat_ms\":"); Serial.println(PREHEAT_MS);
    Serial.println("}}");
    Serial.println();

    Serial.print("{\"flags\":{");
    Serial.print("\"startCycle\":"); Serial.print(flag_startCycle ? "true" : "false");
    Serial.print(",\"preheatDone\":"); Serial.print(flag_preheatDone ? "true" : "false");
    Serial.print(",\"txDone\":"); Serial.print(flag_txDone ? "true" : "false");
    Serial.print(",\"joinDone\":"); Serial.print(flag_joinDone ? "true" : "false");
    Serial.println("}}");
    Serial.println();

    sdiag_printStatus();
    Serial.println();

    Serial.print("{\"rtc\":\""); Serial.print(rtcGetTimestamp()); Serial.println("\"}");
    Serial.println();

    uint16_t bat = readBatteryMV();
    Serial.print("{\"battery\":{\"mv\":"); Serial.print(bat);
    Serial.print(",\"soc\":"); Serial.print(estimateSOC(bat));
    Serial.println("}}");
    Serial.println();

    memPrintInfo("ATC+INFO_GAS");

    Serial.println("{\"sensors\":{");
    if (digitalRead(SENSOR_POWER_PIN) == HIGH) {
        BmeData bme = bmeRead();
        GasData gas = gasRead();
        Serial.print("  \"power\":\"ON\",");
        Serial.print("  \"temp\":"); Serial.print(bme.temperature, 2); Serial.print(",");
        Serial.print("  \"hum\":"); Serial.print(bme.humidity, 2); Serial.print(",");
        Serial.print("  \"pres\":"); Serial.print(bme.pressure, 2); Serial.print(",");
        Serial.print("  \"voc\":"); Serial.print(bme.voc_resistance); Serial.print(",");
        Serial.print("  \"nh3\":"); Serial.print(gas.nh3_ppm, 2); Serial.print(",");
        Serial.print("  \"h2s\":"); Serial.println(gas.h2s_ppm, 2);
    } else {
        Serial.println("  \"power\":\"OFF\"");
    }
    Serial.println("}}");
    Serial.println();

    Serial.print("{\"lorawan\":{\"joined\":"); Serial.print(api.lorawan.njs.get() ? "true" : "false");
    Serial.print(",\"dr\":"); Serial.print(api.lorawan.dr.get());
    Serial.print(",\"adr\":"); Serial.print(api.lorawan.adr.get() ? "true" : "false");
    Serial.println("}}");
    Serial.println();

    flashLogWake();
    flashLogDump();
    flashLogSleep();

    flash_paused = true;
    Serial.println();
    Serial.println("  *** CICLO IN PAUSA ***");
    Serial.println("  ATC+RESUME_GAS per riprendere");
    Serial.println("  ATC+CLEARLOG_GAS per cancellare log");
    Serial.println();

    return AT_OK;
}

int resumeGasHandler(SERIAL_PORT port, char *cmd, stParam *param)
{
    (void)port; (void)cmd; (void)param;
    flash_paused = false;
    Serial.println("[AT] Ciclo ripreso!");
    return AT_OK;
}

int clearlogGasHandler(SERIAL_PORT port, char *cmd, stParam *param)
{
    (void)port; (void)cmd; (void)param;
    Serial.println("[AT] Cancellazione flash log...");
    flashLogWake();
    flashLogClear();
    flashLogSleep();
    flashWriteCount = 0;

    sdiag_clear();

    Serial.println("[AT] Log e diagnostica cancellati!");
    return AT_OK;
}

int settimeGasHandler(SERIAL_PORT port, char *cmd, stParam *param)
{
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

void registerATCommands(void)
{
    bool ok;

    ok = api.system.atMode.add("INFO_GAS",
        "Diagnostica completa v7.0 + dump flash log + pausa ciclo",
        "INFO_GAS", infoGasHandler, RAK_ATCMD_PERM_READ);
    Serial.print("[AT] ATC+INFO_GAS: "); Serial.println(ok ? "OK" : "FAIL");

    ok = api.system.atMode.add("RESUME_GAS",
        "Riprende il ciclo", "RESUME_GAS",
        resumeGasHandler, RAK_ATCMD_PERM_READ);
    Serial.print("[AT] ATC+RESUME_GAS: "); Serial.println(ok ? "OK" : "FAIL");

    ok = api.system.atMode.add("CLEARLOG_GAS",
        "Cancella flash log + diagnostica sleep", "CLEARLOG_GAS",
        clearlogGasHandler, RAK_ATCMD_PERM_READ);
    Serial.print("[AT] ATC+CLEARLOG_GAS: "); Serial.println(ok ? "OK" : "FAIL");

    ok = api.system.atMode.add("SETTIME_GAS",
        "Imposta RTC", "SETTIME_GAS",
        settimeGasHandler, RAK_ATCMD_PERM_WRITE);
    Serial.print("[AT] ATC+SETTIME_GAS: "); Serial.println(ok ? "OK" : "FAIL");

    Serial.println();
}


// =====================================================================
// SETUP — Init hardware + registra callback + avvia join
// =====================================================================

void setup()
{
    Serial.begin(115200);
    api.system.lpm.set(1);       // Low Power Mode attivo
    delay(2000);

    hwWatchdogCheckResetReason();

    // --- Banner ---
    Serial.println();
    Serial.println("=============================================");
    Serial.println("  " FW_NAME);
#if RECOVERY_MODE
    Serial.println("  *** RECOVERY MODE v7.0 ***");
#else
    Serial.println("  FIRMWARE v7.0 -- EVENT-DRIVEN");
#endif
    Serial.println("  " FW_BOARD);
    Serial.println("  " FW_MODULES);
    Serial.println("  LoRaWAN OTAA - EU868 - fPort 77");
    Serial.print("  Ciclo: "); Serial.print(SLEEP_MINUTES);
    Serial.print(" min sleep + "); Serial.print(PREHEAT_MINUTES);
    Serial.println(" min preheat");
    Serial.println("  Architettura: EVENT-DRIVEN (pattern RAK)");
    Serial.println("  Callback -> flag -> loop() -> sleep.all()");
    Serial.print("  Reboot preventivo ogni "); Serial.print(REBOOT_EVERY_N_CYCLES);
    Serial.print(" cicli (~"); Serial.print((uint32_t)REBOOT_EVERY_N_CYCLES * CYCLE_TOTAL_MIN / 60 / 24);
    Serial.println(" giorni)");
    Serial.println("  Diagnostica sleep persistente su flash");
    if (hwWatchdogWasReset()) {
        Serial.println("  *** RECOVERY DA RESET ***");
    }
    Serial.println("=============================================");
    Serial.println();

    printBootDiagnostic();
    if (hwWatchdogWasReset()) {
        diag.wdtTriggered++;
    }

    // --- HW Watchdog ---
    hwWatchdogInit(0);

    // --- AT Commands ---
    registerATCommands();

    // --- Memory baseline ---
    memSetBaseline();
    memPrintInfo("BOOT");

    // --- Alimentazione ---
    uint16_t bootBat = readBatteryMV();
    Serial.print("[BOOT] Alimentazione: "); Serial.print(bootBat); Serial.println(" mV");
    if (bootBat < 3300 || bootBat > 4500) {
        Serial.println("[BOOT] Solo USB rilevato -- delay stabilizzazione 3s");
        delay(3000);
    }

    // --- Sensori + I2C ---
    sensorPowerOn();

    swWatchdogStart(SWWDT_TIMEOUT_MS);
    bool i2c_ok = i2cInitSafe();
    swWatchdogCheck();

    if (!i2c_ok) {
        Serial.println("[BOOT] Nessun device I2C trovato -- power cycle e retry");
        sensorPowerCycle();
        i2c_ok = i2cInitSafe();
        swWatchdogCheck();
    }
    swWatchdogStop();

#if RECOVERY_MODE
    Serial.println("[RECOVERY] Modalita' recovery attiva");
    Serial.println("[RECOVERY] Salto init RTC e sensori");
    Serial.println();

    Serial.println("[RECOVERY] Init flash SPI...");
    flashLogInitWithTimeout();
    Serial.println();

    Serial.println("[RECOVERY] === DUMP FLASH LOG ===");
    flashLogWake();
    flashLogDump();
    flashLogSleep();
    Serial.println("[RECOVERY] === FINE DUMP ===");
    Serial.println();

    flash_paused = true;
    Serial.println("[RECOVERY] In attesa comandi AT...");
    Serial.println("  ATC+INFO_GAS     -> diagnostica");
    Serial.println("  ATC+CLEARLOG_GAS -> cancella log");
    Serial.println("  ATC+RESUME_GAS   -> avvia ciclo normale");
    Serial.println();
    return;
#endif

    // --- RTC ---
    rtcInitWithTimeout();
    rtcAutoSetFromCompileTime();
    rtcPrintTime();
    Serial.println();

    // --- Flash Log ---
    flashLogInitWithTimeout();

    // --- Sleep Diagnostica (DOPO flash_log!) ---
    sdiag_init();
    Serial.println();

    // --- Log boot/recovery ---
    if (hwWatchdogWasReset()) {
        snprintf(jsonLogBuf, sizeof(jsonLogBuf),
            "{\"c\":%lu,\"ts\":\"%s\",\"st\":\"RESET_RECOVERY\","
            "\"mem\":{\"free\":%lu},"
            "\"sd\":{\"e\":%lu,\"x\":%lu}}",
            sdiag_getCycleCount(), rtcGetTimestamp(),
            (unsigned long)memGetFreeHeap(),
            (unsigned long)sdiag_getSleepEnter(),
            (unsigned long)sdiag_getSleepExit());
        flashLogWriteSafe(jsonLogBuf);
        Serial.println("[FLASH] Loggato RESET_RECOVERY con sleep diag");
    }

    if (sdiag_hadSleepMismatch()) {
        snprintf(jsonLogBuf, sizeof(jsonLogBuf),
            "{\"c\":%lu,\"ts\":\"%s\",\"st\":\"SLEEP_MISMATCH\","
            "\"sd\":{\"e\":%lu,\"x\":%lu,\"missed\":%lu}}",
            sdiag_getCycleCount(), rtcGetTimestamp(),
            (unsigned long)sdiag_getSleepEnter(),
            (unsigned long)sdiag_getSleepExit(),
            (unsigned long)(sdiag_getSleepEnter() - sdiag_getSleepExit()));
        flashLogWriteSafe(jsonLogBuf);
        Serial.println("[FLASH] Loggato SLEEP_MISMATCH!");
    }

    Serial.println();

    // --- Spegni sensori (li riaccende TIMER_0) ---
    sensorPowerOff();

    memPrintInfo("POST-INIT");

    // --- Registra callback LoRaWAN ---
    api.lorawan.registerJoinCallback(cb_joinResult);
    api.lorawan.registerSendCallback(cb_sendResult);
    api.lorawan.registerRecvCallback(cb_recvData);
    Serial.println("[LORA] Callback registrate (join, send, recv)");

    // --- Crea timer ---
    bool t0 = api.system.timer.create(RAK_TIMER_0, cb_timerCycle, RAK_TIMER_PERIODIC);
    bool t1 = api.system.timer.create(RAK_TIMER_1, cb_timerPreheat, RAK_TIMER_ONESHOT);
    bool t2 = api.system.timer.create(RAK_TIMER_2, cb_timerJoinRetry, RAK_TIMER_ONESHOT);

    Serial.print("[TIMER] RAK_TIMER_0 (ciclo): "); Serial.println(t0 ? "OK" : "FAIL");
    Serial.print("[TIMER] RAK_TIMER_1 (preheat): "); Serial.println(t1 ? "OK" : "FAIL");
    Serial.print("[TIMER] RAK_TIMER_2 (join retry): "); Serial.println(t2 ? "OK" : "FAIL");
    Serial.println();

    // --- Init + Avvia Join LoRaWAN ---
    loraPrintDeviceInfo();
    loraInit();

    joinRetryCount       = 0;
    tx_fail_count_consec = 0;
    flashWriteCount      = 0;

    Serial.println("[JOIN] Avvio join OTAA (asincrono)...");
    api.lorawan.join();

    // NON avviare TIMER_0 qui — lo avvia il join callback quando OK
    Serial.println();
    Serial.println("[SETUP] Completato — entro in event loop");
    Serial.println("=============================================");
    Serial.println();
}

// =====================================================================
// LOOP — EVENT-DRIVEN (pattern RAK ufficiale)
//
// Regola d'oro: api.system.sleep.all() è SEMPRE l'ultima istruzione.
// Il MCU si sveglia SOLO quando un evento setta un flag.
// Dopo aver gestito il flag, torna a dormire.
// =====================================================================

// =====================================================================
// LOOP — EVENT-DRIVEN v7.0.1 (FIX: periferiche dopo sleep)
//
// Regola d'oro: api.system.sleep.all() è SEMPRE l'ultima istruzione.
// Il MCU si sveglia SOLO quando un evento setta un flag.
// Dopo aver gestito il flag, torna a dormire.
//
// FIX v7.0.1:
//   - SPI re-init in flashLogWake() dopo STOP2 wakeup
//   - NO accesso I2C/RTC quando 3V3_S è OFF
//   - Riordinate fasi per accendere sensori PRIMA di leggere RTC
// =====================================================================

void loop()
{
    // --- PAUSA AT COMMAND ---
    if (flash_paused) {
        delay(100);
        return;            // NON chiamare sleep.all() durante pausa AT
    }

    // =================================================================
    // EVENTO 1: JOIN COMPLETATO (callback LoRaWAN)
    //
    // ATTENZIONE: a questo punto 3V3_S e' OFF!
    // NON accedere a RTC (I2C) ne' a flash (SPI potrebbe non essere
    // pronta dopo STOP2). Solo flag e timer.
    // =================================================================
    if (flag_joinDone) {
        flag_joinDone = false;

        if (api.lorawan.njs.get()) {
            joined = true;
            joinRetryCount = 0;
            Serial.println("[JOIN] Connesso al network!");

            api.lorawan.dr.set(LORAWAN_DR_TX);
            Serial.println("[JOIN] DR impostato a SF7");

            memPrintInfo("POST-JOIN");

            // NON loggare su flash qui — SPI potrebbe non essere pronta
            // Il log JOIN_OK viene scritto nel primo ciclo utile
            Serial.println("[JOIN] Log JOIN_OK rimandato al primo ciclo");

            // Avvia timer ciclo principale
            Serial.print("[TIMER] Avvio TIMER_0 periodico: ");
            Serial.print(CYCLE_INTERVAL_MS / 1000);
            Serial.println("s");
            api.system.timer.start(RAK_TIMER_0, CYCLE_INTERVAL_MS, NULL);

            // Forza il primo ciclo subito dopo il join
            flag_startCycle = true;
            Serial.println("[LOOP] Primo ciclo forzato subito dopo join");

        } else {
            joined = false;
            joinRetryCount++;
            Serial.print("[JOIN] FALLITO -- tentativo ");
            Serial.print(joinRetryCount);
            Serial.print("/");
            Serial.println(MAX_JOIN_RETRIES);

            if (joinRetryCount >= MAX_JOIN_RETRIES) {
                Serial.println("[JOIN] Max tentativi -- riprovo tra 2 min");
                joinRetryCount = 0;
            }

            // Retry join via TIMER_2 oneshot
            api.system.timer.start(RAK_TIMER_2, JOIN_RETRY_MS, NULL);
        }
    }

    // =================================================================
    // EVENTO 2: AVVIO CICLO (TIMER_0 periodico)
    //
    // ORDINE CRITICO:
    //   1. sensorPowerOn() — alimenta 3V3_S (RTC, sensori)
    //   2. Re-init SPI (per flash dopo STOP2)
    //   3. Re-init I2C (per RTC e sensori dopo STOP2)
    //   4. Solo ORA possiamo accedere a RTC, flash, sensori
    // =================================================================
    if (flag_startCycle) {
        flag_startCycle = false;

        // --- STEP 1: Accendi 3V3_S PRIMA DI TUTTO ---
        sensorPowerOn();

        // --- STEP 2: Re-init SPI (potrebbe essere persa dopo STOP2) ---
        SPI.begin();
        SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
        Serial.println("[SPI] Re-init dopo sleep OK");

        // --- STEP 3: Re-init I2C ---
        i2cInitSafe();

        // --- STEP 4: Ora possiamo accedere a flash e RTC ---
        sdiag_incrementCycle();

        Serial.println();
        Serial.println("=============================================");
        Serial.print("  CICLO #"); Serial.println(sdiag_getCycleCount());
        Serial.print("  "); Serial.println(rtcGetTimestamp());
        Serial.println("=============================================");
        Serial.println();

        // --- Log JOIN_OK se è il primo ciclo dopo join ---
        if (sdiag_getCycleCount() == 1) {
            snprintf(jsonLogBuf, sizeof(jsonLogBuf),
                "{\"c\":%lu,\"ts\":\"%s\",\"st\":\"JOIN_OK\"}",
                sdiag_getCycleCount(), rtcGetTimestamp());
            flashLogWriteSafe(jsonLogBuf);
            Serial.println("[FLASH] Log JOIN_OK scritto");
        }

        // --- Reboot preventivo ---
        if (sdiag_getCycleCount() > 0 &&
            (sdiag_getCycleCount() % REBOOT_EVERY_N_CYCLES) == 0) {
            logRebootAndReset();
            // Non ritorna — NVIC_SystemReset()
        }

        // --- Controlla batteria ---
        battery_mv  = readBatteryMV();
        battery_soc = estimateSOC(battery_mv);
        Serial.print("[BAT] "); Serial.print(battery_mv);
        Serial.print(" mV ("); Serial.print(battery_soc); Serial.println("%)");

        if (battery_mv < BATTERY_MIN_MV && battery_mv > 0) {
            Serial.println("[BAT] TENSIONE CRITICA -- salto ciclo");

            snprintf(jsonLogBuf, sizeof(jsonLogBuf),
                "{\"c\":%lu,\"ts\":\"%s\",\"st\":\"LOW_BAT\",\"bat\":{\"mv\":%u}}",
                sdiag_getCycleCount(), rtcGetTimestamp(), battery_mv);
            flashLogWriteSafe(jsonLogBuf);

            // Spegni sensori e aspetta prossimo TIMER_0
            sensorPowerOff();
        } else {
            // --- Avvia preheat (sensori già accesi al punto STEP 1) ---
            Serial.print("[PREHEAT] Avvio timer oneshot: ");
            Serial.print(PREHEAT_MS / 1000);
            Serial.println("s (MCU dormira' durante preheat)");

            api.system.timer.start(RAK_TIMER_1, PREHEAT_MS, NULL);
        }
    }

    // =================================================================
    // EVENTO 3: PREHEAT COMPLETATO (TIMER_1 oneshot)
    //
    // Sensori gia' accesi da EVENTO 2.
    // I2C gia' inizializzato da EVENTO 2.
    // Ma dopo STOP2 (preheat = MCU dormiva) serve re-init!
    // =================================================================
    if (flag_preheatDone) {
        flag_preheatDone = false;

        Serial.println("[PREHEAT] Completato — lettura sensori");

        // --- Re-init periferiche dopo STOP2 del preheat ---
        SPI.begin();
        SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

        swWatchdogStart(SWWDT_TIMEOUT_MS);

        // --- I2C Recovery + re-init ---
        i2cClockRecovery();
        Wire.begin();
        delay(100);
        swWatchdogCheck();

        // --- Init sensori ---
        bmeInit();
        swWatchdogCheck();
        gasInitAll();
        swWatchdogCheck();

        // --- Lettura sensori ---
        Serial.println("--- LETTURA SENSORI ---");

        bmeData = bmeRead();
        swWatchdogCheck();
        Serial.print("[BME680] Temp: "); Serial.print(bmeData.temperature, 2); Serial.println(" C");
        Serial.print("[BME680] Hum:  "); Serial.print(bmeData.humidity, 2); Serial.println(" %");
        Serial.print("[BME680] Pres: "); Serial.print(bmeData.pressure, 2); Serial.println(" hPa");
        Serial.print("[BME680] VOC:  "); Serial.print(bmeData.voc_resistance); Serial.println(" Ohm");

        gasData = gasRead();
        swWatchdogCheck();
        Serial.print("[NH3] Conc: "); Serial.print(gasData.nh3_ppm, 2); Serial.println(" ppm");
        Serial.print("[H2S] Conc: "); Serial.print(gasData.h2s_ppm, 2); Serial.println(" ppm");

        sensors_ok = true;

        swWatchdogStop();

        // --- Batteria aggiornata ---
        battery_mv  = readBatteryMV();
        battery_soc = estimateSOC(battery_mv);
        Serial.print("[BAT] "); Serial.print(battery_mv);
        Serial.print(" mV ("); Serial.print(battery_soc); Serial.println("%)");

        cycleTimestamp = rtcGetTimestamp();
        Serial.print("[RTC] "); Serial.println(cycleTimestamp);

        // --- Memory check ---
        memUpdateMin();
        memPrintInfo("PRE-TX");

        if (memCheckCritical()) {
            Serial.println("[MEM] CRITICA -- loggo e proseguo");
            snprintf(jsonLogBuf, sizeof(jsonLogBuf),
                "{\"c\":%lu,\"ts\":\"%s\",\"st\":\"MEM_CRITICAL\",\"mem\":{\"free\":%lu}}",
                sdiag_getCycleCount(), cycleTimestamp, (unsigned long)memGetFreeHeap());
            flashLogWriteSafe(jsonLogBuf);
        }

        // --- Costruisci payload ---
        loraBuildPayload(
            bmeData.temperature, bmeData.humidity, bmeData.pressure,
            bmeData.voc_resistance,
            gasData.nh3_ppm, gasData.h2s_ppm,
            battery_mv, battery_soc
        );

        // --- Diagnostica pre-TX ---
        sdiag_incrementSleepEnter();

        // --- TX asincrona ---
        Serial.print("[LORA] TX 19 bytes fPort "); Serial.println(LORAWAN_FPORT);

        if (!api.lorawan.send(sizeof(payload), payload, LORAWAN_FPORT, false)) {
            Serial.println("[LORA] send() ritornato false -- sessione persa?");

            sdiag_incrementSleepExit();     // Bilancia il counter
            tx_fail_count_consec++;

            buildJsonLog(cycleTimestamp, "TX_FAIL_SEND");
            flashLogWriteSafe(jsonLogBuf);

            sensorPowerOff();

            if (!api.lorawan.njs.get()) {
                Serial.println("[LORA] Sessione persa -> rejoin");
                joined = false;
                joinRetryCount = 0;
                api.lorawan.join();
            }
        } else {
            Serial.println("[LORA] TX avviata (attendo TX_DONE callback)");
            // TX_DONE arriverà via cb_sendResult → flag_txDone
            // MCU dormirà fino a quel momento
        }
    }

    // =================================================================
    // EVENTO 4: TX COMPLETATA (callback LoRaWAN)
    //
    // Sensori ancora accesi, SPI/I2C dovrebbero essere OK
    // (poco tempo passato da EVENTO 3, no STOP2 in mezzo)
    // Ma per sicurezza re-init SPI prima di scrivere flash
    // =================================================================
    if (flag_txDone) {
        flag_txDone = false;

        // Re-init SPI per sicurezza
        SPI.begin();
        SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

        sdiag_incrementSleepExit();

        if (flag_txStatus == 0) {   // RAK_LORAMAC_STATUS_OK
            Serial.println("[LORA] TX OK!");
            diag.successfulTx++;
            diag.lastTxResult = 1;
            tx_fail_count_consec = 0;
        } else {
            Serial.print("[LORA] TX FAIL (status="); Serial.print(flag_txStatus); Serial.println(")");
            diag.lastTxResult = 0;
            tx_fail_count_consec++;
            diag.lastTxFailCount = tx_fail_count_consec;

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

        // --- Log su flash ---
        const char *phase;
        if (flag_txStatus == 0)   phase = "TX_OK";
        else                      phase = "TX_FAIL";

        buildJsonLog(cycleTimestamp, phase);
        flashLogWriteSafe(jsonLogBuf);
        Serial.print("[FLASH] "); Serial.println(jsonLogBuf);

        // --- Spegni sensori ---
        sensorPowerOff();

        // --- Diagnostica memoria ---
        memUpdateMin();
        Serial.print("[MEM] Free: "); Serial.print(memGetFreeHeap());
        Serial.print(" | Min: "); Serial.print(memGetMinHeapForLog());
        Serial.print(" | Delta: "); Serial.println((int32_t)memGetFreeHeap() - (int32_t)mem_heap_free_boot);

        Serial.println();
        Serial.println("-------------------------------------------");
        Serial.print("[SLEEP] Attendo prossimo TIMER_0 tra ~");
        Serial.print(CYCLE_INTERVAL_MS / 60000);
        Serial.println(" min");
        Serial.println("-------------------------------------------");
        Serial.flush();
    }

    // =================================================================
    // SEMPRE: TORNA A DORMIRE
    // Questa è l'ultima istruzione — MAI codice dopo questa riga.
    // Il MCU si sveglia SOLO quando un evento (timer, TX_DONE, AT, etc.)
    // setta un flag. RUI3 gestisce internamente lo sleep/wakeup.
    // =================================================================
    api.system.sleep.all();
}


