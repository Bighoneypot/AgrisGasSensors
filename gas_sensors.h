
// =====================================================================
// gas_sensors.h - DRIVER SENSORI GAS NH3 + H2S (DFRobot)
// =====================================================================
// MYRUMINET - Sensore Gas Stalla v6.6
// NH3: DFRobot SEN0467 - I2C 0x77
// H2S: DFRobot SEN0469 - I2C 0x74
// Collegati via RAK1920 (Sensor Adapter)
// Alimentazione controllata via WB_IO2 (3V3_S)
// =====================================================================

#ifndef GAS_SENSORS_H
#define GAS_SENSORS_H

#include "config.h"
#include <Wire.h>
#include "DFRobot_MultiGasSensor.h"


// =====================================================================
// STRUTTURA DATI GAS
// =====================================================================

struct GasData {
    float nh3_ppm;
    float h2s_ppm;
    bool nh3_valid;
    bool h2s_valid;
};

static DFRobot_GAS_I2C gasSensorNH3(&Wire, NH3_I2C_ADDR);
static DFRobot_GAS_I2C gasSensorH2S(&Wire, H2S_I2C_ADDR);
static bool nh3_ok = false;
static bool h2s_ok = false;


// =====================================================================
// INIT SENSORI GAS
// =====================================================================

void gasInitNH3(void)
{
    Serial.println("[NH3] Init...");
    delay(1000);

    if (gasSensorNH3.begin()) {
        nh3_ok = true;
        Serial.print("[NH3] Gas type: ");
        Serial.println(gasSensorNH3.queryGasType());
        gasSensorNH3.changeAcquireMode(gasSensorNH3.PASSIVITY);
        Serial.println("[NH3] Init OK");
    } else {
        nh3_ok = false;
        Serial.println("[NH3] Init FALLITA");
    }
}

void gasInitH2S(void)
{
    Serial.println("[H2S] Init...");
    delay(1000);

    if (gasSensorH2S.begin()) {
        h2s_ok = true;
        Serial.print("[H2S] Gas type: ");
        Serial.println(gasSensorH2S.queryGasType());
        gasSensorH2S.changeAcquireMode(gasSensorH2S.PASSIVITY);
        Serial.println("[H2S] Init OK");
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
// LETTURA SENSORI GAS
// =====================================================================

GasData gasRead(void)
{
    GasData data;
    data.nh3_ppm = 0.0f;
    data.h2s_ppm = 0.0f;
    data.nh3_valid = false;
    data.h2s_valid = false;

    if (nh3_ok) {
        data.nh3_ppm = gasSensorNH3.readGasConcentrationPPM();
        data.nh3_valid = true;
        Serial.print("[NH3] Conc: ");
        Serial.print(data.nh3_ppm, 2);
        Serial.println(" ppm");
    } else {
        Serial.println("[NH3] Non inizializzato, skip");
    }

    if (h2s_ok) {
        data.h2s_ppm = gasSensorH2S.readGasConcentrationPPM();
        data.h2s_valid = true;
        Serial.print("[H2S] Conc: ");
        Serial.print(data.h2s_ppm, 2);
        Serial.println(" ppm");
    } else {
        Serial.println("[H2S] Non inizializzato, skip");
    }

    return data;
}


// =====================================================================
// RESET FLAG (per nuovo ciclo dopo power off sensori)
// =====================================================================

void gasReset(void)
{
    nh3_ok = false;
    h2s_ok = false;
}


#endif // GAS_SENSORS_H

