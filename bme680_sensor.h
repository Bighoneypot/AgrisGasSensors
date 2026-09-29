
// =====================================================================
// bme680_sensor.h - DRIVER BME680 (RAK1906)
// =====================================================================
// MYRUMINET - Sensore Gas Stalla v6.6
// Chip: Bosch BME680
// Indirizzo I2C: 0x76
// =====================================================================

#ifndef BME680_SENSOR_H
#define BME680_SENSOR_H

#include "config.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>


// =====================================================================
// STRUTTURA DATI BME680
// =====================================================================

struct BmeData {
    float temperature;
    float humidity;
    float pressure;
    uint32_t voc_resistance;
    bool valid;
};

static Adafruit_BME680 bme;
static bool bme680_ok = false;


// =====================================================================
// INIT BME680
// =====================================================================

bool bmeInit(void)
{
    Serial.println("[BME680] Init...");

    if (bme.begin(BME680_I2C_ADDR, &Wire)) {
        bme680_ok = true;
        bme.setTemperatureOversampling(BME680_OS_8X);
        bme.setHumidityOversampling(BME680_OS_2X);
        bme.setPressureOversampling(BME680_OS_4X);
        bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
        bme.setGasHeater(320, 150);
        Serial.println("[BME680] Init OK");
        return true;
    }

    bme680_ok = false;
    Serial.println("[BME680] Init FALLITA");
    return false;
}


// =====================================================================
// LETTURA BME680
// =====================================================================

BmeData bmeRead(void)
{
    BmeData data;
    data.temperature = 0.0f;
    data.humidity = 0.0f;
    data.pressure = 0.0f;
    data.voc_resistance = 0;
    data.valid = false;

    if (!bme680_ok) {
        Serial.println("[BME680] Non inizializzato");
        return data;
    }

    if (bme.performReading()) {
        data.temperature = bme.temperature;
        data.humidity = bme.humidity;
        data.pressure = bme.pressure / 100.0f;
        data.voc_resistance = (uint32_t)bme.gas_resistance;
        data.valid = true;

        Serial.print("[BME680] Temp: ");
        Serial.print(data.temperature, 2);
        Serial.println(" C");
        Serial.print("[BME680] Hum:  ");
        Serial.print(data.humidity, 2);
        Serial.println(" %");
        Serial.print("[BME680] Pres: ");
        Serial.print(data.pressure, 2);
        Serial.println(" hPa");
        Serial.print("[BME680] VOC:  ");
        Serial.print(data.voc_resistance);
        Serial.println(" Ohm");
    } else {
        Serial.println("[BME680] Errore lettura!");
    }

    return data;
}


// =====================================================================
// RESET FLAG (per nuovo ciclo dopo power off sensori)
// =====================================================================

void bmeReset(void)
{
    bme680_ok = false;
}


#endif // BME680_SENSOR_H

