
// =====================================================================
// diagnostics.h - WDT, FASI FIRMWARE, I2C RECOVERY, BATTERIA
// =====================================================================
// MYRUMINET - Sensore Gas Stalla v6.6
// =====================================================================

#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include "config.h"
#include <Wire.h>


// =====================================================================
// FASI FIRMWARE (ordine di esecuzione)
// =====================================================================

enum FirmwarePhase : uint8_t {
    PHASE_BOOT            = 0,
    PHASE_BATTERY_CHECK   = 1,
    PHASE_JOIN_CHECK      = 2,
    PHASE_SENSOR_POWER_ON = 3,
    PHASE_PREHEAT         = 4,
    PHASE_I2C_RECOVERY    = 5,
    PHASE_BME680_INIT     = 6,
    PHASE_NH3_INIT        = 7,
    PHASE_H2S_INIT        = 8,
    PHASE_BME680_READ     = 9,
    PHASE_NH3_READ        = 10,
    PHASE_H2S_READ        = 11,
    PHASE_BATTERY_READ    = 12,
    PHASE_FLASH_LOG       = 13,
    PHASE_PAYLOAD_BUILD   = 14,
    PHASE_LORAWAN_SEND    = 15,
    PHASE_RADIO_WAIT      = 16,
    PHASE_SLEEP_PREP      = 17,
    PHASE_SLEEPING        = 18,
    PHASE_WAKE            = 19
};

static const char* phaseNames[] = {
    "BOOT",
    "BATTERY_CHECK",
    "JOIN_CHECK",
    "SENSOR_POWER_ON",
    "PREHEAT",
    "I2C_RECOVERY",
    "BME680_INIT",
    "NH3_INIT",
    "H2S_INIT",
    "BME680_READ",
    "NH3_READ",
    "H2S_READ",
    "BATTERY_READ",
    "FLASH_LOG",
    "PAYLOAD_BUILD",
    "LORAWAN_SEND",
    "RADIO_WAIT",
    "SLEEP_PREP",
    "SLEEPING",
    "WAKE"
};


// =====================================================================
// STRUTTURA DIAGNOSTICA (.noinit - sopravvive a software reboot)
// =====================================================================

struct DiagnosticState {
    uint32_t magic;
    FirmwarePhase lastPhase;
    uint32_t cycleCount;
    uint32_t successfulTx;
    uint8_t lastTxResult;
    uint16_t lastBatteryMv;
    uint8_t lastTxFailCount;
    uint8_t wdtTriggered;
} __attribute__((section(".noinit")));

static DiagnosticState diag __attribute__((section(".noinit")));


// =====================================================================
// MACRO
// =====================================================================

#define SET_PHASE(p) do { \
    diag.lastPhase = (p); \
    Serial.print("[PHASE] -> "); \
    Serial.println(phaseNames[(p)]); \
} while(0)

#define LOG_ENTER(op) do { Serial.print("[" op "] Ingresso..."); Serial.println(); } while(0)
#define LOG_EXIT_OK(op) Serial.println("[" op "] Uscita OK")
#define LOG_EXIT_FAIL(op) Serial.println("[" op "] Uscita FAIL")


// =====================================================================
// STATO GLOBALE
// =====================================================================

static volatile bool wdt_active = false;
static bool i2c_bus_ok = false;
static bool joined = false;
static uint8_t tx_fail_count = 0;


// =====================================================================
// WATCHDOG TIMER
// =====================================================================

void wdtCallback(void *data)
{
    (void)data;
    Serial.println();
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("  [WDT] TIMEOUT! Sistema bloccato!");
    Serial.print("  [WDT] FASE: ");
    Serial.println(phaseNames[diag.lastPhase]);
    Serial.print("  [WDT] CICLO: ");
    Serial.println(diag.cycleCount);

    diag.wdtTriggered = 1;

    #if ENABLE_PREHEAT
        digitalWrite(SENSOR_POWER_PIN, LOW);
        Serial.println("  [WDT] 3V3_S SPENTO (safety)");
    #endif

    Serial.println("  [WDT] Reboot automatico in corso...");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.flush();
    delay(100);
    api.system.reboot();
}

void wdtStart(void)
{
    api.system.timer.start(WDT_TIMER_ID, WDT_TIMEOUT_MS, NULL);
    wdt_active = true;
    Serial.print("[WDT] Avviato (timeout ");
    Serial.print(WDT_TIMEOUT_MS / 1000);
    Serial.println(" sec)");
}

void wdtStop(void)
{
    if (wdt_active) {
        api.system.timer.stop(WDT_TIMER_ID);
        wdt_active = false;
        Serial.println("[WDT] Fermato");
    }
}

void wdtKick(void)
{
    if (wdt_active) {
        api.system.timer.stop(WDT_TIMER_ID);
    }
    api.system.timer.start(WDT_TIMER_ID, WDT_TIMEOUT_MS, NULL);
    wdt_active = true;
}

bool wdtInit(void)
{
    if (!api.system.timer.create(WDT_TIMER_ID, wdtCallback, RAK_TIMER_ONESHOT)) {
        Serial.println("[WDT] ERRORE creazione timer!");
        return false;
    }
    Serial.println("[WDT] Timer creato (one-shot, pronto)");
    return true;
}


// =====================================================================
// I2C BUS RECOVERY
// =====================================================================

bool i2cBusRecovery(void)
{
    Wire.end();
    delay(10);

    pinMode(I2C_SDA_PIN, INPUT_PULLUP);
    pinMode(I2C_SCL_PIN, OUTPUT);

    if (digitalRead(I2C_SDA_PIN) == LOW) {
        Serial.println("[I2C] SDA bloccata LOW! Recovery in corso...");

        for (int i = 0; i < 9; i++) {
            digitalWrite(I2C_SCL_PIN, HIGH);
            delayMicroseconds(5);
            digitalWrite(I2C_SCL_PIN, LOW);
            delayMicroseconds(5);
        }

        pinMode(I2C_SDA_PIN, OUTPUT);
        digitalWrite(I2C_SDA_PIN, LOW);
        delayMicroseconds(5);
        digitalWrite(I2C_SCL_PIN, HIGH);
        delayMicroseconds(5);
        digitalWrite(I2C_SDA_PIN, HIGH);
        delayMicroseconds(5);

        pinMode(I2C_SDA_PIN, INPUT_PULLUP);
        if (digitalRead(I2C_SDA_PIN) == HIGH) {
            Serial.println("[I2C] Recovery OK - SDA rilasciata");
            pinMode(I2C_SDA_PIN, INPUT_PULLUP);
            pinMode(I2C_SCL_PIN, INPUT_PULLUP);
            delay(10);
            return true;
        } else {
            Serial.println("[I2C] Recovery FALLITA - SDA ancora LOW");
            pinMode(I2C_SDA_PIN, INPUT_PULLUP);
            pinMode(I2C_SCL_PIN, INPUT_PULLUP);
            delay(10);
            return false;
        }
    } else {
        Serial.println("[I2C] Bus OK (SDA HIGH)");
        pinMode(I2C_SDA_PIN, INPUT_PULLUP);
        pinMode(I2C_SCL_PIN, INPUT_PULLUP);
        delay(10);
        return true;
    }
}

bool i2cBusRecoveryWithRetry(void)
{
    const unsigned int retryDelays[I2C_RECOVERY_MAX_RETRIES] = {200, 500, 1000};

    for (int attempt = 0; attempt < I2C_RECOVERY_MAX_RETRIES; attempt++) {
        Serial.print("[I2C] Tentativo recovery ");
        Serial.print(attempt + 1);
        Serial.print("/");
        Serial.println(I2C_RECOVERY_MAX_RETRIES);

        if (i2cBusRecovery()) {
            return true;
        }

        if (attempt < I2C_RECOVERY_MAX_RETRIES - 1) {
            Serial.print("[I2C] Attendo ");
            Serial.print(retryDelays[attempt]);
            Serial.println(" ms prima di riprovare...");
            delay(retryDelays[attempt]);
        }
    }

    Serial.println("[I2C] !!! Recovery FALLITA dopo tutti i tentativi !!!");
    return false;
}


// =====================================================================
// BATTERIA
// =====================================================================

uint16_t readBatteryMV(void)
{
    analogReadResolution(12);
    uint32_t sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += analogRead(BATTERY_PIN);
        delay(5);
    }
    float raw = (float)sum / 10.0f;
    float voltage_mv = (raw / 4096.0f) * 3000.0f * BATTERY_DIVIDER;
    return (uint16_t)voltage_mv;
}

uint8_t estimateSOC(uint16_t mv)
{
    if (mv >= 4200) return 100;
    if (mv >= 4060) return 90;
    if (mv >= 3980) return 80;
    if (mv >= 3920) return 70;
    if (mv >= 3870) return 60;
    if (mv >= 3820) return 50;
    if (mv >= 3790) return 40;
    if (mv >= 3770) return 30;
    if (mv >= 3740) return 20;
    if (mv >= 3680) return 10;
    if (mv >= 3400) return 5;
    return 0;
}


// =====================================================================
// BOOT DIAGNOSTIC
// =====================================================================

void printBootDiagnostic(void)
{
    Serial.println();
    Serial.println("=============================================");
    Serial.println("  DIAGNOSTICA BOOT v6.6");
    Serial.println("=============================================");

    if (diag.magic == 0xDEADBEEF) {
        Serial.print("  [BOOT] LAST_PHASE   = ");
        Serial.println(phaseNames[diag.lastPhase]);
        Serial.print("  [BOOT] LAST_CYCLE   = ");
        Serial.println(diag.cycleCount);
        Serial.print("  [BOOT] TOTAL_TX_OK  = ");
        Serial.println(diag.successfulTx);
        Serial.print("  [BOOT] LAST_TX      = ");
        Serial.println(diag.lastTxResult ? "OK" : "FAIL");
        Serial.print("  [BOOT] TX_FAIL_CNT  = ");
        Serial.println(diag.lastTxFailCount);
        Serial.print("  [BOOT] LAST_BAT     = ");
        Serial.print(diag.lastBatteryMv);
        Serial.println(" mV");
        Serial.print("  [BOOT] RESET_REASON = ");

        if (diag.wdtTriggered) {
            Serial.println("WDT TIMEOUT");
            Serial.print("  [BOOT] WDT scattato durante: ");
            Serial.println(phaseNames[diag.lastPhase]);
        } else if (diag.lastPhase == PHASE_SLEEPING) {
            Serial.println("WAKE FROM SLEEP (normale)");
        } else {
            Serial.println("SCONOSCIUTO (power cycle o crash)");
        }
    } else {
        Serial.println("  [BOOT] Prima accensione o power cycle");
        Serial.println("  [BOOT] Nessun dato diagnostico precedente");
    }

    Serial.println("=============================================");
    Serial.println();

    // Inizializza per questa sessione
    diag.magic = 0xDEADBEEF;
    diag.lastPhase = PHASE_BOOT;
    diag.cycleCount = 0;
    diag.successfulTx = 0;
    diag.lastTxResult = 0;
    diag.lastBatteryMv = 0;
    diag.lastTxFailCount = 0;
    diag.wdtTriggered = 0;
}


#endif // DIAGNOSTICS_H

