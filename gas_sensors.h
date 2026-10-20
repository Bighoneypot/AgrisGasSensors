
// =====================================================================
// gas_sensors.h - DRIVER SENSORI GAS NH3 + H2S BARE-METAL (ZERO HEAP)
// =====================================================================
// MYRUMINET - Sensore Gas Stalla v7.2
// RAK4630 (nRF52840) / RAK19007 Rev.C
// NH3: DFRobot SEN0467 - I2C 0x77
// H2S: DFRobot SEN0469 - I2C 0x74
// v7.2: debug raw bytes per diagnosi gas type 0x0
// =====================================================================

#ifndef GAS_SENSORS_H
#define GAS_SENSORS_H

#include "config.h"
#include <Wire.h>


#define DFGAS_CMD_READ_CONC     0x86
#define DFGAS_CMD_CHANGE_MODE   0x78
#define DFGAS_CMD_READ_GASTYPE  0x7E

#define DFGAS_MODE_PASSIVE      0x04

#define DFGAS_PKT_SIZE          9


struct GasData {
    float nh3_ppm;
    float h2s_ppm;
    bool nh3_valid;
    bool h2s_valid;
};

static bool nh3_ok = false;
static bool h2s_ok = false;


// =====================================================================
// CHECKSUM
// =====================================================================

static uint8_t dfgas_checksum(const uint8_t *buf)
{
    uint8_t sum = 0;
    for (uint8_t i = 1; i < 8; i++) {
        sum += buf[i];
    }
    return (~sum) + 1;
}


// =====================================================================
// DEBUG: dump raw 9 bytes
// =====================================================================

static void dfgas_dumpRaw(const char *label, const char *dir, const uint8_t *buf)
{
    Serial.print("["); Serial.print(label); Serial.print("] ");
    Serial.print(dir); Serial.print(": ");
    for (uint8_t i = 0; i < DFGAS_PKT_SIZE; i++) {
        if (buf[i] < 0x10) Serial.print("0");
        Serial.print(buf[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}


// =====================================================================
// INVIO COMANDO + LETTURA RISPOSTA (9 bytes ciascuno)
// =====================================================================

static bool dfgas_sendCmd(uint8_t i2c_addr, uint8_t cmd, uint8_t param,
                          uint8_t *resp, const char *label)
{
    uint8_t txBuf[DFGAS_PKT_SIZE];
    txBuf[0] = 0xFF;
    txBuf[1] = 0x01;
    txBuf[2] = cmd;
    txBuf[3] = param;
    txBuf[4] = 0x00;
    txBuf[5] = 0x00;
    txBuf[6] = 0x00;
    txBuf[7] = 0x00;
    txBuf[8] = dfgas_checksum(txBuf);

    dfgas_dumpRaw(label, "TX", txBuf);

    Wire.beginTransmission(i2c_addr);
    for (uint8_t i = 0; i < DFGAS_PKT_SIZE; i++) {
        Wire.write(txBuf[i]);
    }
    if (Wire.endTransmission() != 0) {
        Serial.print("["); Serial.print(label); Serial.println("] I2C TX error");
        return false;
    }

    delay(100);

    Wire.requestFrom(i2c_addr, (uint8_t)DFGAS_PKT_SIZE);

    uint8_t count = 0;
    uint32_t t0 = millis();
    while (count < DFGAS_PKT_SIZE && (millis() - t0) < 200) {
        if (Wire.available()) {
            resp[count++] = Wire.read();
        }
    }

    if (count != DFGAS_PKT_SIZE) {
        Serial.print("["); Serial.print(label);
        Serial.print("] RX incompleto: "); Serial.print(count);
        Serial.println(" bytes");
        return false;
    }

    dfgas_dumpRaw(label, "RX", resp);

    if (resp[0] != 0xFF) {
        Serial.print("["); Serial.print(label); Serial.println("] Header RX errato");
        return false;
    }

    uint8_t expected_cs = dfgas_checksum(resp);
    if (resp[8] != expected_cs) {
        Serial.print("["); Serial.print(label);
        Serial.print("] Checksum RX errato: atteso 0x");
        Serial.print(expected_cs, HEX);
        Serial.print(" ricevuto 0x");
        Serial.println(resp[8], HEX);
        return false;
    }

    return true;
}


// =====================================================================
// PASSIVE MODE
// =====================================================================

static bool dfgas_setPassiveMode(uint8_t i2c_addr, const char *label)
{
    uint8_t resp[DFGAS_PKT_SIZE];
    return dfgas_sendCmd(i2c_addr, DFGAS_CMD_CHANGE_MODE, DFGAS_MODE_PASSIVE, resp, label);
}


// =====================================================================
// PROBE (leggi gas type)
// =====================================================================

static bool dfgas_probe(uint8_t i2c_addr, const char *label)
{
    uint8_t resp[DFGAS_PKT_SIZE];

    if (!dfgas_sendCmd(i2c_addr, DFGAS_CMD_READ_GASTYPE, 0x00, resp, label)) {
        Serial.print("["); Serial.print(label); Serial.println("] Nessuna risposta");
        return false;
    }

    Serial.print("["); Serial.print(label);
    Serial.print("] Gas type code: 0x"); Serial.println(resp[2], HEX);

    return true;
}


// =====================================================================
// LETTURA CONCENTRAZIONE GAS (PPM)
// =====================================================================

static float dfgas_readPPM(uint8_t i2c_addr, bool *valid, const char *label)
{
    *valid = false;
    uint8_t resp[DFGAS_PKT_SIZE];

    if (!dfgas_sendCmd(i2c_addr, DFGAS_CMD_READ_CONC, 0x00, resp, label)) {
        return 0.0f;
    }

    uint16_t raw_conc = ((uint16_t)resp[2] << 8) | resp[3];
    uint8_t decimal_info = resp[5];

    Serial.print("["); Serial.print(label);
    Serial.print("] raw_conc="); Serial.print(raw_conc);
    Serial.print(" decimal_info="); Serial.println(decimal_info);

    float ppm;
    switch (decimal_info) {
        case 1:  ppm = (float)raw_conc / 10.0f;  break;
        case 2:  ppm = (float)raw_conc / 100.0f;  break;
        default: ppm = (float)raw_conc;            break;
    }

    if (resp[2] & 0x80) {
        ppm = 0.0f;
    }

    *valid = true;
    return ppm;
}


// =====================================================================
// INIT
// =====================================================================

void gasInitNH3(void)
{
    Serial.println("[NH3] Init bare-metal driver...");
    delay(1000);

    if (dfgas_probe(NH3_I2C_ADDR, "NH3")) {
        if (dfgas_setPassiveMode(NH3_I2C_ADDR, "NH3")) {
            nh3_ok = true;
            Serial.println("[NH3] Passive mode OK");
            Serial.println("[NH3] Init OK (bare-metal, zero heap)");
        } else {
            nh3_ok = false;
            Serial.println("[NH3] Errore set passive mode");
        }
    } else {
        nh3_ok = false;
        Serial.println("[NH3] Init FALLITA");
    }
}

void gasInitH2S(void)
{
    Serial.println("[H2S] Init bare-metal driver...");
    delay(1000);

    if (dfgas_probe(H2S_I2C_ADDR, "H2S")) {
        if (dfgas_setPassiveMode(H2S_I2C_ADDR, "H2S")) {
            h2s_ok = true;
            Serial.println("[H2S] Passive mode OK");
            Serial.println("[H2S] Init OK (bare-metal, zero heap)");
        } else {
            h2s_ok = false;
            Serial.println("[H2S] Errore set passive mode");
        }
    } else {
        h2s_ok = false;
        Serial.println("[H2S] Init FALLITA");
    }
}

void gasInitAll(void)
{
    gasInitNH3();
    gasInitH2S();
}


// =====================================================================
// LETTURA
// =====================================================================

GasData gasRead(void)
{
    GasData data;
    data.nh3_ppm = 0.0f;
    data.h2s_ppm = 0.0f;
    data.nh3_valid = false;
    data.h2s_valid = false;

    if (nh3_ok) {
        data.nh3_ppm = dfgas_readPPM(NH3_I2C_ADDR, &data.nh3_valid, "NH3");
        Serial.print("[NH3] Conc: "); Serial.print(data.nh3_ppm, 2);
        Serial.print(" ppm");
        if (!data.nh3_valid) Serial.print(" [INVALID]");
        Serial.println();
    } else {
        Serial.println("[NH3] Non inizializzato, skip");
    }

    if (h2s_ok) {
        data.h2s_ppm = dfgas_readPPM(H2S_I2C_ADDR, &data.h2s_valid, "H2S");
        Serial.print("[H2S] Conc: "); Serial.print(data.h2s_ppm, 2);
        Serial.print(" ppm");
        if (!data.h2s_valid) Serial.print(" [INVALID]");
        Serial.println();
    } else {
        Serial.println("[H2S] Non inizializzato, skip");
    }

    return data;
}


// =====================================================================
// RESET
// =====================================================================

void gasReset(void)
{
    nh3_ok = false;
    h2s_ok = false;
}


#endif // GAS_SENSORS_H

