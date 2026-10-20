
// =====================================================================
// diagnostics.h - FASI FIRMWARE, I2C RECOVERY, BATTERIA
// =====================================================================
// MYRUMINET v7.2 — RAK4630 (nRF52840) / RAK19007 Rev.C
// Rimosso .noinit (non supportato su nRF52840/RUI3)
// Rimosso WDT timer-based (usa watchdog_hw.h)
// =====================================================================

#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include "config.h"
#include <Wire.h>


// =====================================================================
// FASI FIRMWARE
// =====================================================================

enum FirmwarePhase : uint8_t {
    PHASE_BOOT = 0, PHASE_BATTERY_CHECK, PHASE_JOIN_CHECK,
    PHASE_SENSOR_POWER_ON, PHASE_PREHEAT, PHASE_I2C_RECOVERY,
    PHASE_BME680_INIT, PHASE_NH3_INIT, PHASE_H2S_INIT,
    PHASE_BME680_READ, PHASE_NH3_READ, PHASE_H2S_READ,
    PHASE_BATTERY_READ, PHASE_FLASH_LOG, PHASE_PAYLOAD_BUILD,
    PHASE_LORAWAN_SEND, PHASE_RADIO_WAIT, PHASE_SLEEP_PREP,
    PHASE_SLEEPING, PHASE_WAKE
};

static const char* phaseNames[] = {
    "BOOT", "BATTERY_CHECK", "JOIN_CHECK", "SENSOR_POWER_ON",
    "PREHEAT", "I2C_RECOVERY", "BME680_INIT", "NH3_INIT", "H2S_INIT",
    "BME680_READ", "NH3_READ", "H2S_READ", "BATTERY_READ",
    "FLASH_LOG", "PAYLOAD_BUILD", "LORAWAN_SEND", "RADIO_WAIT",
    "SLEEP_PREP", "SLEEPING", "WAKE"
};


// =====================================================================
// STRUTTURA DIAGNOSTICA (RAM normale — persa al reboot)
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
};

static DiagnosticState diag;


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


// =====================================================================
// I2C BUS RECOVERY (SDA=P0.13, SCL=P0.14)
// =====================================================================

bool i2cBusRecovery(void)
{
    Wire.end();
    delay(10);

    pinMode(I2C_SDA_PIN, INPUT_PULLUP);
    pinMode(I2C_SCL_PIN, OUTPUT);

    if (digitalRead(I2C_SDA_PIN) == LOW) {
        Serial.println("[I2C] SDA bloccata LOW! Recovery...");

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
            Serial.println("[I2C] Recovery OK");
        } else {
            Serial.println("[I2C] Recovery FALLITA");
        }
    } else {
        Serial.println("[I2C] Bus OK (SDA HIGH)");
    }

    pinMode(I2C_SDA_PIN, INPUT);
    pinMode(I2C_SCL_PIN, INPUT);
    delay(10);
    return (digitalRead(I2C_SDA_PIN) == HIGH);
}

bool i2cBusRecoveryWithRetry(void)
{
    const unsigned int retryDelays[I2C_RECOVERY_MAX_RETRIES] = {200, 500, 1000};
    for (int attempt = 0; attempt < I2C_RECOVERY_MAX_RETRIES; attempt++) {
        Serial.print("[I2C] Tentativo "); Serial.print(attempt + 1);
        Serial.print("/"); Serial.println(I2C_RECOVERY_MAX_RETRIES);
        if (i2cBusRecovery()) return true;
        if (attempt < I2C_RECOVERY_MAX_RETRIES - 1) delay(retryDelays[attempt]);
    }
    Serial.println("[I2C] Recovery FALLITA dopo tutti i tentativi!");
    return false;
}


// =====================================================================
// BATTERIA (nRF52840 ADC 12-bit, Vref 3.0V)
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
    Serial.println("  DIAGNOSTICA BOOT v7.2 (nRF52840)");
    Serial.println("=============================================");

    if (diag.magic == 0xDEADBEEF) {
        Serial.print("  [BOOT] LAST_PHASE   = "); Serial.println(phaseNames[diag.lastPhase]);
        Serial.print("  [BOOT] LAST_CYCLE   = "); Serial.println(diag.cycleCount);
        Serial.print("  [BOOT] TOTAL_TX_OK  = "); Serial.println(diag.successfulTx);
        Serial.print("  [BOOT] LAST_TX      = "); Serial.println(diag.lastTxResult ? "OK" : "FAIL");
        Serial.print("  [BOOT] TX_FAIL_CNT  = "); Serial.println(diag.lastTxFailCount);
        Serial.print("  [BOOT] LAST_BAT     = "); Serial.print(diag.lastBatteryMv); Serial.println(" mV");
        if (diag.wdtTriggered) {
            Serial.println("  [BOOT] RESET = WDT TIMEOUT");
        } else if (diag.lastPhase == PHASE_SLEEPING) {
            Serial.println("  [BOOT] RESET = WAKE FROM SLEEP");
        } else {
            Serial.println("  [BOOT] RESET = SCONOSCIUTO");
        }
    } else {
        Serial.println("  [BOOT] Prima accensione o power cycle");
    }

    Serial.println("=============================================");
    Serial.println();

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

