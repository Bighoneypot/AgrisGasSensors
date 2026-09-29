
// =====================================================================
// GAS_Sensor_v6.7.ino - FIRMWARE FSM PULITA
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
// ARCHITETTURA: FSM 100% sequenziale
//   - ZERO timer paralleli
//   - ZERO ATZ automatico
//   - ZERO callback da interrupt
//   - Ogni stato si completa prima del successivo
//   - Se il join fallisce: sleep 2 min + retry (max 5), poi sleep lungo
//   - Se il firmware si blocca: solo pyocd puo' recuperare
//     (ma non si blocchera' perche' non ci sono timer/ATZ)
//
// Comandi AT custom (prefisso ATC+):
//   ATC+INFO_GAS       -> diagnostica + dump flash + pausa
//   ATC+RESUME_GAS     -> riprende il ciclo
//   ATC+CLEARLOG_GAS   -> cancella flash log
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
#include "bme680_sensor.h"
#include "gas_sensors.h"
#include "lorawan_manager.h"


// =====================================================================
// FSM — DEFINIZIONE STATI
// =====================================================================

typedef enum {
    STATE_BOOT = 0,
    STATE_JOIN,
    STATE_JOIN_RETRY,
    STATE_PREHEAT,
    STATE_I2C_RECOVERY,
    STATE_READ_SENSORS,
    STATE_BATTERY_CHECK,
    STATE_BUILD_PAYLOAD,
    STATE_TX,
    STATE_LOG_FLASH,
    STATE_SLEEP,
    STATE_WAKEUP,
    STATE_JOIN_CHECK,
    STATE_ERROR,
    STATE_COUNT
} FsmState;

static const char* stateNames[] = {
    "BOOT", "JOIN", "JOIN_RETRY", "PREHEAT",
    "I2C_RECOVERY", "READ_SENSORS", "BATTERY_CHECK",
    "BUILD_PAYLOAD", "TX", "LOG_FLASH", "SLEEP",
    "WAKEUP", "JOIN_CHECK", "ERROR"
};


// =====================================================================
// FSM — VARIABILI
// =====================================================================

static FsmState currentState  = STATE_BOOT;
static FsmState previousState = STATE_BOOT;
static uint8_t  errorCount     = 0;
static uint8_t  joinRetryCount = 0;

#define MAX_JOIN_RETRIES     5
#define MAX_ERROR_COUNT      3
#define ERROR_SLEEP_MS       60000UL    // 1 minuto
#define JOIN_RETRY_SLEEP_MS  120000UL   // 2 minuti
#define LONG_SLEEP_MS        120000UL   // 2 minuti (dopo max retry)
#define JOIN_TIMEOUT_MS      90000UL    // 90 secondi


// =====================================================================
// DATI CICLO — Condivisi tra stati
// =====================================================================

static BmeData  bmeData     = {0.0f, 0.0f, 0.0f, 0};
static GasData  gasData     = {0.0f, 0.0f};
static uint16_t battery_mv  = 0;
static uint8_t  battery_soc = 0;
static bool     tx_ok       = false;
static bool     sensors_ok  = false;
static char     jsonLogBuf[FLASH_RECORD_SIZE];
static const char *cycleTimestamp = "";


// =====================================================================
// FSM — TRANSIZIONE
// =====================================================================

void fsmTransition(FsmState newState)
{
    previousState = currentState;
    currentState  = newState;

    Serial.print("[FSM] ");
    Serial.print(stateNames[previousState]);
    Serial.print(" -> ");
    Serial.println(stateNames[currentState]);
}

void fsmTransitionError(const char* reason)
{
    Serial.print("[FSM] ERRORE in ");
    Serial.print(stateNames[currentState]);
    Serial.print(": ");
    Serial.println(reason);

    previousState = currentState;
    currentState  = STATE_ERROR;
    errorCount++;
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
        "\"tx\":%s}",
        diag.cycleCount, timestamp, phase,
        battery_mv, battery_soc,
        bmeData.temperature, bmeData.humidity, bmeData.pressure,
        (unsigned long)bmeData.voc_resistance,
        gasData.nh3_ppm, gasData.h2s_ppm,
        (unsigned long)memGetFreeHeapForLog(), (unsigned long)memGetMinHeapForLog(),
        tx_ok ? "true" : "false"
    );
}


// =====================================================================
// AT COMMAND HANDLERS
// =====================================================================

int infoGasHandler(SERIAL_PORT port, char *cmd, stParam *param)
{
    (void)port; (void)cmd; (void)param;

    Serial.println();
    Serial.println("=============================================");
    Serial.println("  ATC+INFO_GAS - DIAGNOSTICA v6.7 FSM");
    Serial.println("=============================================");
    Serial.println();

    // Stato FSM
    Serial.print("{\"fsm\":{\"state\":\""); Serial.print(stateNames[currentState]);
    Serial.print("\",\"prev\":\""); Serial.print(stateNames[previousState]);
    Serial.print("\",\"errors\":"); Serial.print(errorCount);
    Serial.print(",\"join_retries\":"); Serial.print(joinRetryCount);
    Serial.println("}}");
    Serial.println();

    // Firmware
    Serial.println("{\"firmware\":{");
    Serial.print("  \"version\":\""); Serial.print(FW_VERSION); Serial.println("\",");
    Serial.print("  \"board\":\""); Serial.print(FW_BOARD); Serial.println("\",");
    Serial.print("  \"modules\":\""); Serial.print(FW_MODULES); Serial.println("\",");
    Serial.print("  \"cycle_count\":"); Serial.print(diag.cycleCount); Serial.println(",");
    Serial.print("  \"successful_tx\":"); Serial.print(diag.successfulTx); Serial.println(",");
    Serial.print("  \"tx_fail_count\":"); Serial.print(diag.lastTxFailCount); Serial.println(",");
    Serial.print("  \"wdt_triggered\":"); Serial.println(diag.wdtTriggered);
    Serial.println("}}");
    Serial.println();

    // RTC
    Serial.print("{\"rtc\":\""); Serial.print(rtcGetTimestamp()); Serial.println("\"}");
    Serial.println();

    // Batteria
    uint16_t bat = readBatteryMV();
    Serial.print("{\"battery\":{\"mv\":"); Serial.print(bat);
    Serial.print(",\"soc\":"); Serial.print(estimateSOC(bat));
    Serial.println("}}");
    Serial.println();

    // Memoria
    memPrintInfo("ATC+INFO_GAS");

    // Sensori live
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

    // LoRaWAN
    Serial.print("{\"lorawan\":{\"joined\":"); Serial.print(api.lorawan.njs.get() ? "true" : "false");
    Serial.print(",\"dr\":"); Serial.print(api.lorawan.dr.get());
    Serial.print(",\"adr\":"); Serial.print(api.lorawan.adr.get() ? "true" : "false");
    Serial.println("}}");
    Serial.println();

    // Flash log dump
    flashLogWake();
    flashLogDump();
    flashLogSleep();

    // Pausa
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
    Serial.println("[AT] Log cancellato!");
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
        "Diagnostica completa + dump flash log + pausa ciclo",
        "INFO_GAS", infoGasHandler, RAK_ATCMD_PERM_READ);
    Serial.print("[AT] ATC+INFO_GAS: "); Serial.println(ok ? "OK" : "FAIL");

    ok = api.system.atMode.add("RESUME_GAS",
        "Riprende il ciclo", "RESUME_GAS",
        resumeGasHandler, RAK_ATCMD_PERM_READ);
    Serial.print("[AT] ATC+RESUME_GAS: "); Serial.println(ok ? "OK" : "FAIL");

    ok = api.system.atMode.add("CLEARLOG_GAS",
        "Cancella flash log", "CLEARLOG_GAS",
        clearlogGasHandler, RAK_ATCMD_PERM_READ);
    Serial.print("[AT] ATC+CLEARLOG_GAS: "); Serial.println(ok ? "OK" : "FAIL");

    ok = api.system.atMode.add("SETTIME_GAS",
        "Imposta RTC", "SETTIME_GAS",
        settimeGasHandler, RAK_ATCMD_PERM_WRITE);
    Serial.print("[AT] ATC+SETTIME_GAS: "); Serial.println(ok ? "OK" : "FAIL");

    Serial.println();
}


// =====================================================================
// FSM — HANDLER PER OGNI STATO
// =====================================================================

// -----------------------------------------------------------------
// STATE_BOOT
// -----------------------------------------------------------------
void stateBoot(void)
{
    Serial.begin(115200);
    api.system.lpm.set(1);
    delay(2000);

    // Reset reason
    hwWatchdogCheckResetReason();

    // Banner
    Serial.println();
    Serial.println("=============================================");
    Serial.println("  " FW_NAME);
    Serial.println("  FIRMWARE v" FW_VERSION " -- FSM PULITA");
    Serial.println("  " FW_BOARD);
    Serial.println("  " FW_MODULES);
    Serial.println("  LoRaWAN OTAA - EU868 - fPort 77");
    Serial.print("  Ciclo: "); Serial.print(SLEEP_MINUTES);
    Serial.print(" min sleep + "); Serial.print(PREHEAT_MINUTES);
    Serial.println(" min preheat");
    Serial.print("  Join timeout: "); Serial.print(JOIN_TIMEOUT_MS / 1000); Serial.println("s");
    Serial.println("  No timer, no ATZ, no callback");
    if (hwWatchdogWasReset()) {
        Serial.println("  *** RECOVERY DA RESET ***");
    }
    Serial.println("=============================================");
    Serial.println();

    // Diagnostica boot
    printBootDiagnostic();
    if (hwWatchdogWasReset()) {
        diag.wdtTriggered++;
    }

    // Init (stub, nessun timer)
    hwWatchdogInit(0);

    // Comandi AT
    registerATCommands();

    // Memory baseline
    memSetBaseline();
    memPrintInfo("BOOT");

    // Stabilizzazione alimentazione prima di accendere sensori
    uint16_t bootBat = readBatteryMV();
    Serial.print("[BOOT] Alimentazione: "); Serial.print(bootBat); Serial.println(" mV");
    if (bootBat < 3300 || bootBat > 4500) {
        Serial.println("[BOOT] Solo USB rilevato -- delay stabilizzazione 3s");
        delay(3000);
    }

    // Power on sensori (per RTC + Flash init)
    sensorPowerOn();

    // I2C
    Wire.begin();
    delay(100);
    Serial.println("[I2C] Bus OK");
    Serial.println();

    // RTC
    rtcInit();
    rtcAutoSetFromCompileTime();
    rtcPrintTime();
    Serial.println();

    // Flash log
    flashLogInit();

    if (hwWatchdogWasReset()) {
        snprintf(jsonLogBuf, sizeof(jsonLogBuf),
            "{\"c\":%lu,\"ts\":\"%s\",\"st\":\"RESET_RECOVERY\",\"mem\":{\"free\":%lu}}",
            diag.cycleCount, rtcGetTimestamp(), (unsigned long)memGetFreeHeap());
        flashLogWrite(jsonLogBuf);
        Serial.println("[FLASH] Loggato RESET_RECOVERY");
    }

    flashLogSleep();
    Serial.println();

    // Spegni sensori (risparmio durante join)
    sensorPowerOff();

    // Memory post-init
    memPrintInfo("POST-INIT");

    // LoRaWAN config
    loraPrintDeviceInfo();
    loraInit();

    // Reset contatori
    errorCount     = 0;
    joinRetryCount = 0;

    // -> JOIN
    fsmTransition(STATE_JOIN);
}


// -----------------------------------------------------------------
// STATE_JOIN — Join BLOCCANTE, nessun timer parallelo
// -----------------------------------------------------------------
void stateJoin(void)
{
    Serial.println("[JOIN] Avvio join OTAA (bloccante, no timer)...");

    if (!api.lorawan.join()) {
        Serial.println("[JOIN] api.lorawan.join() restituito false");
        fsmTransition(STATE_JOIN_RETRY);
        return;
    }

    // Polling bloccante
    uint32_t startMs = millis();
    uint32_t elapsed = 0;

    while (elapsed < JOIN_TIMEOUT_MS) {
        if (api.lorawan.njs.get()) {
            joined = true;
            joinRetryCount = 0;
            Serial.println("[JOIN] JOIN OK!");

            api.lorawan.dr.set(LORAWAN_DR_TX);
            Serial.println("[JOIN] DR impostato a SF7");

            memPrintInfo("POST-JOIN");

            fsmTransition(STATE_PREHEAT);
            return;
        }

        delay(5000);
        elapsed = millis() - startMs;

        Serial.print("[JOIN] Attendo join... ");
        Serial.print(elapsed / 1000);
        Serial.println("s");
    }

    // Timeout
    Serial.println("[JOIN] TIMEOUT 90s -- join fallito");
    joined = false;
    fsmTransition(STATE_JOIN_RETRY);
}


// -----------------------------------------------------------------
// STATE_JOIN_RETRY — Sleep 2 min + retry, max 5 tentativi
// -----------------------------------------------------------------
void stateJoinRetry(void)
{
    joinRetryCount++;

    Serial.print("[JOIN-RETRY] Tentativo ");
    Serial.print(joinRetryCount);
    Serial.print("/");
    Serial.println(MAX_JOIN_RETRIES);

    if (joinRetryCount >= MAX_JOIN_RETRIES) {
        Serial.println("[JOIN-RETRY] Max tentativi raggiunto");
        Serial.print("[JOIN-RETRY] Sleep lungo "); Serial.print(LONG_SLEEP_MS / 1000);
        Serial.println("s poi riprovo da zero");

        // Reset contatore e sleep lungo
        joinRetryCount = 0;
        sleepSimple(LONG_SLEEP_MS);

        // Riprova da JOIN
        fsmTransition(STATE_JOIN);
        return;
    }

    // Sleep tra retry
    Serial.print("[JOIN-RETRY] Sleep "); Serial.print(JOIN_RETRY_SLEEP_MS / 1000);
    Serial.println("s prima del prossimo tentativo");

    sleepSimple(JOIN_RETRY_SLEEP_MS);

    fsmTransition(STATE_JOIN);
}


// -----------------------------------------------------------------
// STATE_PREHEAT — Riscaldamento sensori (SOLO dopo join OK)
// -----------------------------------------------------------------
void statePreheat(void)
{
    diag.cycleCount++;
    Serial.println();
    Serial.println("=============================================");
    Serial.print("  CICLO #"); Serial.println(diag.cycleCount);
    Serial.println("=============================================");
    Serial.println();

    // Accendi sensori
    sensorPowerOn();

    #if ENABLE_PREHEAT
    uint32_t totalSec = (uint32_t)PREHEAT_MINUTES * 60UL;
    uint32_t elapsed  = 0;
    uint32_t blockSec = 5;

    Serial.println("[PREHEAT] Preriscaldo sensori...");

    while (elapsed < totalSec) {
        // Pausa AT
        while (flash_paused) {
            delay(100);
        }

        delay(blockSec * 1000UL);
        elapsed += blockSec;

        Serial.print("[PREHEAT] ");
        Serial.print(elapsed);
        Serial.print(" sec / ");
        Serial.print(totalSec);
        Serial.println(" sec");
    }

    Serial.println("[PREHEAT] Completato");
    Serial.println();
    #endif

    fsmTransition(STATE_I2C_RECOVERY);
}


// -----------------------------------------------------------------
// STATE_I2C_RECOVERY
// -----------------------------------------------------------------
void stateI2cRecovery(void)
{
    Wire.begin();
    delay(100);
    i2c_bus_ok = i2cBusRecoveryWithRetry();

    if (!i2c_bus_ok) {
        Serial.println("[I2C] Bus fail! Power cycle...");
        sensorPowerOff();
        delay(2000);
        sensorPowerOn();
        Wire.begin();
        delay(500);
        i2c_bus_ok = i2cBusRecoveryWithRetry();
    }

    if (i2c_bus_ok) {
        sensors_ok = true;
        fsmTransition(STATE_READ_SENSORS);
    } else {
        Serial.println("[I2C] Bus irrecuperabile!");
        sensors_ok = false;
        bmeData = {0.0f, 0.0f, 0.0f, 0};
        gasData = {0.0f, 0.0f};
        fsmTransition(STATE_BATTERY_CHECK);
    }
}


// -----------------------------------------------------------------
// STATE_READ_SENSORS
// -----------------------------------------------------------------
void stateReadSensors(void)
{
    bmeInit();
    gasInitAll();

    Serial.println("--- LETTURA SENSORI ---");

    bmeData = bmeRead();
    Serial.print("[BME680] Temp: "); Serial.print(bmeData.temperature, 2); Serial.println(" C");
    Serial.print("[BME680] Hum:  "); Serial.print(bmeData.humidity, 2); Serial.println(" %");
    Serial.print("[BME680] Pres: "); Serial.print(bmeData.pressure, 2); Serial.println(" hPa");
    Serial.print("[BME680] VOC:  "); Serial.print(bmeData.voc_resistance); Serial.println(" Ohm");

    gasData = gasRead();
    Serial.print("[NH3] Conc: "); Serial.print(gasData.nh3_ppm, 2); Serial.println(" ppm");
    Serial.print("[H2S] Conc: "); Serial.print(gasData.h2s_ppm, 2); Serial.println(" ppm");

    fsmTransition(STATE_BATTERY_CHECK);
}


// -----------------------------------------------------------------
// STATE_BATTERY_CHECK
// -----------------------------------------------------------------
void stateBatteryCheck(void)
{
    battery_mv  = readBatteryMV();
    battery_soc = estimateSOC(battery_mv);
    diag.lastBatteryMv = battery_mv;

    Serial.print("[BAT] "); Serial.print(battery_mv);
    Serial.print(" mV ("); Serial.print(battery_soc); Serial.println("%)");
    Serial.println();

    if (battery_mv < BATTERY_MIN_MV && battery_mv > 0) {
        Serial.println("[BAT] TENSIONE CRITICA -- sleep lungo");

        flashLogWake();
        snprintf(jsonLogBuf, sizeof(jsonLogBuf),
            "{\"c\":%lu,\"ts\":\"%s\",\"st\":\"LOW_BAT\",\"bat\":{\"mv\":%u}}",
            diag.cycleCount, rtcGetTimestamp(), battery_mv);
        flashLogWrite(jsonLogBuf);
        flashLogSleep();

        sensorPowerOff();
        sleepSimple(LOW_BATTERY_SLEEP_MS);

        fsmTransition(STATE_WAKEUP);
        return;
    }

    cycleTimestamp = rtcGetTimestamp();
    Serial.print("[RTC] "); Serial.println(cycleTimestamp);
    Serial.println();

    memUpdateMin();
    memPrintInfo("PRE-TX");

    if (memCheckCritical()) {
        Serial.println("[MEM] CRITICA -- loggo e proseguo comunque");
        flashLogWake();
        snprintf(jsonLogBuf, sizeof(jsonLogBuf),
            "{\"c\":%lu,\"ts\":\"%s\",\"st\":\"MEM_CRITICAL\",\"mem\":{\"free\":%lu}}",
            diag.cycleCount, cycleTimestamp, (unsigned long)memGetFreeHeap());
        flashLogWrite(jsonLogBuf);
        flashLogSleep();
        // NON resetto, proseguo -- il ciclo e' piu' importante
    }

    fsmTransition(STATE_BUILD_PAYLOAD);
}


// -----------------------------------------------------------------
// STATE_BUILD_PAYLOAD
// -----------------------------------------------------------------
void stateBuildPayload(void)
{
    loraBuildPayload(
        bmeData.temperature, bmeData.humidity, bmeData.pressure,
        bmeData.voc_resistance,
        gasData.nh3_ppm, gasData.h2s_ppm,
        battery_mv, battery_soc
    );

    fsmTransition(STATE_TX);
}


// -----------------------------------------------------------------
// STATE_TX
// -----------------------------------------------------------------
void stateTx(void)
{
    Serial.print("[LORA] TX 19 bytes fPort "); Serial.println(LORAWAN_FPORT);

    tx_ok = api.lorawan.send(sizeof(payload), payload, LORAWAN_FPORT, false);

    if (tx_ok) {
        Serial.println("[LORA] TX OK!");
        diag.successfulTx++;
        diag.lastTxResult = 1;
        tx_fail_count = 0;
        errorCount = 0;
    } else {
        Serial.println("[LORA] TX FAIL!");
        diag.lastTxResult = 0;
        tx_fail_count++;
        diag.lastTxFailCount = tx_fail_count;

        if (!api.lorawan.njs.get()) {
            Serial.println("[LORA] Sessione persa");
            joined = false;
        }

        if (tx_fail_count >= MAX_TX_FAILURES) {
            Serial.println("[LORA] TX fail consecutivi -> ERROR");
            fsmTransitionError("TX falliti consecutivi");
            return;
        }
    }

    // Radio wait (RX1 + RX2)
    delay(3000);
    delay(3000);

    fsmTransition(STATE_LOG_FLASH);
}


// -----------------------------------------------------------------
// STATE_LOG_FLASH
// -----------------------------------------------------------------
void stateLogFlash(void)
{
    flashLogWake();

    const char *phase;
    if (!sensors_ok)    phase = "I2C_FAIL";
    else if (tx_ok)     phase = "TX_OK";
    else                phase = "TX_FAIL";

    buildJsonLog(cycleTimestamp, phase);
    flashLogWrite(jsonLogBuf);
    Serial.print("[FLASH] "); Serial.println(jsonLogBuf);
    flashLogSleep();

    memUpdateMin();
    Serial.print("[MEM] Free: "); Serial.print(memGetFreeHeap());
    Serial.print(" | Min: "); Serial.print(memGetMinHeapForLog());
    Serial.print(" | Delta: "); Serial.println((int32_t)memGetFreeHeap() - (int32_t)mem_heap_free_boot);

    fsmTransition(STATE_SLEEP);
}


// -----------------------------------------------------------------
// STATE_SLEEP
// -----------------------------------------------------------------
void stateSleep(void)
{
    sensorPowerOff();

    Serial.println();
    Serial.println("-------------------------------------------");
    Serial.print("[SLEEP] "); Serial.print(SLEEP_MINUTES);
    Serial.println(" min");
    Serial.println("-------------------------------------------");
    Serial.flush();

    sleepSimple(SLEEP_INTERVAL);

    fsmTransition(STATE_WAKEUP);
}


// -----------------------------------------------------------------
// STATE_WAKEUP
// -----------------------------------------------------------------
void stateWakeup(void)
{
    Serial.println();
    Serial.print("[WAKE] Risveglio -- cicli completati: ");
    Serial.println(diag.cycleCount);
    Serial.println();

    fsmTransition(STATE_JOIN_CHECK);
}


// -----------------------------------------------------------------
// STATE_JOIN_CHECK
// -----------------------------------------------------------------
void stateJoinCheck(void)
{
    if (api.lorawan.njs.get()) {
        joined = true;
        Serial.println("[JOIN-CHECK] Sessione attiva -> PREHEAT");
        fsmTransition(STATE_PREHEAT);
    } else {
        Serial.println("[JOIN-CHECK] Sessione persa -> rejoin");
        joined = false;
        joinRetryCount = 0;
        fsmTransition(STATE_JOIN);
    }
}


// -----------------------------------------------------------------
// STATE_ERROR
// -----------------------------------------------------------------
void stateError(void)
{
    Serial.println();
    Serial.println("=============================================");
    Serial.print("  ERROR HANDLER -- errore #"); Serial.println(errorCount);
    Serial.print("  Stato precedente: "); Serial.println(stateNames[previousState]);
    Serial.println("=============================================");
    Serial.println();

    // Log su flash
    flashLogWake();
    snprintf(jsonLogBuf, sizeof(jsonLogBuf),
        "{\"c\":%lu,\"ts\":\"%s\",\"st\":\"ERROR\",\"prev\":\"%s\",\"err\":%u}",
        diag.cycleCount, rtcGetTimestamp(), stateNames[previousState], errorCount);
    flashLogWrite(jsonLogBuf);
    flashLogSleep();

    sensorPowerOff();

    if (errorCount >= MAX_ERROR_COUNT) {
        Serial.println("[ERROR] Max errori -- sleep lungo e riprovo");
        errorCount = 0;
        sleepSimple(LONG_SLEEP_MS);
        fsmTransition(STATE_JOIN_CHECK);
        return;
    }

    // Sleep breve e riprova
    Serial.print("[ERROR] Sleep "); Serial.print(ERROR_SLEEP_MS / 1000);
    Serial.println("s poi riprovo");

    sleepSimple(ERROR_SLEEP_MS);

    switch (previousState) {
        case STATE_JOIN:
        case STATE_JOIN_RETRY:
            fsmTransition(STATE_JOIN);
            break;
        case STATE_I2C_RECOVERY:
        case STATE_READ_SENSORS:
            fsmTransition(STATE_PREHEAT);
            break;
        case STATE_TX:
            fsmTransition(STATE_TX);
            break;
        default:
            fsmTransition(STATE_JOIN_CHECK);
            break;
    }
}


// =====================================================================
// FSM — ENGINE
// =====================================================================

void fsmRun(void)
{
    switch (currentState) {
        case STATE_BOOT:          stateBoot();          break;
        case STATE_JOIN:          stateJoin();          break;
        case STATE_JOIN_RETRY:    stateJoinRetry();     break;
        case STATE_PREHEAT:       statePreheat();       break;
        case STATE_I2C_RECOVERY:  stateI2cRecovery();   break;
        case STATE_READ_SENSORS:  stateReadSensors();   break;
        case STATE_BATTERY_CHECK: stateBatteryCheck();  break;
        case STATE_BUILD_PAYLOAD: stateBuildPayload();  break;
        case STATE_TX:            stateTx();            break;
        case STATE_LOG_FLASH:     stateLogFlash();      break;
        case STATE_SLEEP:         stateSleep();         break;
        case STATE_WAKEUP:        stateWakeup();        break;
        case STATE_JOIN_CHECK:    stateJoinCheck();     break;
        case STATE_ERROR:         stateError();         break;
        default:
            Serial.println("[FSM] Stato sconosciuto -> BOOT");
            currentState = STATE_BOOT;
            break;
    }
}


// =====================================================================
// SETUP + LOOP
// =====================================================================

void setup()
{
    currentState = STATE_BOOT;
    fsmRun();
}

void loop()
{
    if (flash_paused) {
        delay(100);
        return;
    }

    fsmRun();
}

