
// =====================================================================
// bme680_sensor.h - DRIVER BME680 BARE-METAL (ZERO HEAP)
// =====================================================================
// MYRUMINET - Sensore Gas Stalla v7.2
// RAK4630 (nRF52840) / RAK19007 Rev.C
// Chip: Bosch BME680 — I2C 0x76
//
// FIX v7.2a:
//   1. Compensazione umidita' corretta (Bosch API reference)
//   2. Heater abilitato esplicitamente (ctrl_gas_0 = 0x00)
//   3. Lettura warmup a fine init per pre-riscaldare heater
//   4. gas_wait portato a 100ms (da 150ms) per match Adafruit default
// =====================================================================

#ifndef BME680_SENSOR_H
#define BME680_SENSOR_H

#include "config.h"
#include <Wire.h>

// =====================================================================
// REGISTRI BME680
// =====================================================================

#define BME680_CHIP_ID_REG      0xD0
#define BME680_CHIP_ID_VAL      0x61
#define BME680_RESET_REG        0xE0
#define BME680_RESET_CMD        0xB6
#define BME680_STATUS_REG       0x73
#define BME680_EAS_STATUS_REG   0x1D
#define BME680_CTRL_MEAS_REG    0x74
#define BME680_CTRL_HUM_REG     0x72
#define BME680_CTRL_GAS0_REG    0x70
#define BME680_CTRL_GAS1_REG    0x71
#define BME680_CONFIG_REG       0x75
#define BME680_GAS_WAIT_0_REG   0x64
#define BME680_RES_HEAT_0_REG   0x5A

#define BME680_OS_NONE          0x00
#define BME680_OS_1X            0x01
#define BME680_OS_2X            0x02
#define BME680_OS_4X            0x03
#define BME680_OS_8X            0x04
#define BME680_OS_16X           0x05

#define BME680_FILTER_OFF       0x00
#define BME680_FILTER_1         0x01
#define BME680_FILTER_3         0x02
#define BME680_FILTER_7         0x03
#define BME680_FILTER_15        0x04

#define BME680_MODE_SLEEP       0x00
#define BME680_MODE_FORCED      0x01

// Configurazione heater
#define BME680_HEATER_TEMP_C    320
#define BME680_HEATER_DUR_MS    100


// =====================================================================
// STRUTTURA DATI
// =====================================================================

struct BmeData {
    float temperature;
    float humidity;
    float pressure;
    uint32_t voc_resistance;
    bool valid;
};


// =====================================================================
// CALIBRAZIONE (statica, ~60 bytes)
// =====================================================================

static struct {
    uint16_t par_t1;
    int16_t  par_t2;
    int8_t   par_t3;

    uint16_t par_p1;
    int16_t  par_p2;
    int8_t   par_p3;
    int16_t  par_p4;
    int16_t  par_p5;
    int8_t   par_p6;
    int8_t   par_p7;
    int16_t  par_p8;
    int16_t  par_p9;
    uint8_t  par_p10;

    uint16_t par_h1;
    uint16_t par_h2;
    int8_t   par_h3;
    int8_t   par_h4;
    int8_t   par_h5;
    uint8_t  par_h6;
    int8_t   par_h7;

    int8_t   par_gh1;
    int16_t  par_gh2;
    int8_t   par_gh3;
    uint8_t  res_heat_range;
    int8_t   res_heat_val;
    int8_t   range_sw_err;

    int32_t  t_fine;
    bool     valid;
} bme680_cal = {0};

static bool bme680_ok = false;


// =====================================================================
// I2C LOW-LEVEL
// =====================================================================

static bool bme680_write8(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(BME680_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

static uint8_t bme680_read8(uint8_t reg)
{
    Wire.beginTransmission(BME680_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)BME680_I2C_ADDR, (uint8_t)1);
    return Wire.read();
}

static bool bme680_readBurst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission(BME680_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom((uint8_t)BME680_I2C_ADDR, len);
    for (uint8_t i = 0; i < len; i++) {
        if (!Wire.available()) return false;
        buf[i] = Wire.read();
    }
    return true;
}


// =====================================================================
// LETTURA CALIBRAZIONE
// =====================================================================

static bool bme680_readCalibration(void)
{
    uint8_t coeff1[25];
    uint8_t coeff2[16];

    if (!bme680_readBurst(0x89, coeff1, 25)) return false;
    if (!bme680_readBurst(0xE1, coeff2, 16)) return false;

    // Temperatura
    bme680_cal.par_t1 = (uint16_t)(coeff2[8]) | ((uint16_t)(coeff2[9]) << 8);
    bme680_cal.par_t2 = (int16_t)((uint16_t)(coeff1[1]) | ((uint16_t)(coeff1[2]) << 8));
    bme680_cal.par_t3 = (int8_t)(coeff1[3]);

    // Pressione
    bme680_cal.par_p1  = (uint16_t)(coeff1[5])  | ((uint16_t)(coeff1[6]) << 8);
    bme680_cal.par_p2  = (int16_t)((uint16_t)(coeff1[7])  | ((uint16_t)(coeff1[8]) << 8));
    bme680_cal.par_p3  = (int8_t)(coeff1[9]);
    bme680_cal.par_p4  = (int16_t)((uint16_t)(coeff1[11]) | ((uint16_t)(coeff1[12]) << 8));
    bme680_cal.par_p5  = (int16_t)((uint16_t)(coeff1[13]) | ((uint16_t)(coeff1[14]) << 8));
    bme680_cal.par_p6  = (int8_t)(coeff1[16]);
    bme680_cal.par_p7  = (int8_t)(coeff1[15]);
    bme680_cal.par_p8  = (int16_t)((uint16_t)(coeff1[19]) | ((uint16_t)(coeff1[20]) << 8));
    bme680_cal.par_p9  = (int16_t)((uint16_t)(coeff1[21]) | ((uint16_t)(coeff1[22]) << 8));
    bme680_cal.par_p10 = coeff1[23];

    // Umidita'
    bme680_cal.par_h1 = (uint16_t)((uint16_t)(coeff2[2]) << 4) | (coeff2[1] & 0x0F);
    bme680_cal.par_h2 = (uint16_t)((uint16_t)(coeff2[0]) << 4) | (coeff2[1] >> 4);
    bme680_cal.par_h3 = (int8_t)(coeff2[3]);
    bme680_cal.par_h4 = (int8_t)(coeff2[4]);
    bme680_cal.par_h5 = (int8_t)(coeff2[5]);
    bme680_cal.par_h6 = coeff2[6];
    bme680_cal.par_h7 = (int8_t)(coeff2[7]);

    // Gas
    bme680_cal.par_gh1 = (int8_t)(coeff2[12]);
    bme680_cal.par_gh2 = (int16_t)((uint16_t)(coeff2[10]) | ((uint16_t)(coeff2[11]) << 8));
    bme680_cal.par_gh3 = (int8_t)(coeff2[13]);
    bme680_cal.res_heat_range = (bme680_read8(0x02) >> 4) & 0x03;
    bme680_cal.res_heat_val = (int8_t)bme680_read8(0x00);
    bme680_cal.range_sw_err = ((int8_t)(bme680_read8(0x04) & 0xF0)) >> 4;

    bme680_cal.valid = true;

    Serial.print("[BME680] Cal H: h1="); Serial.print(bme680_cal.par_h1);
    Serial.print(" h2="); Serial.print(bme680_cal.par_h2);
    Serial.print(" h3="); Serial.print(bme680_cal.par_h3);
    Serial.print(" h4="); Serial.print(bme680_cal.par_h4);
    Serial.print(" h5="); Serial.print(bme680_cal.par_h5);
    Serial.print(" h6="); Serial.print(bme680_cal.par_h6);
    Serial.print(" h7="); Serial.println(bme680_cal.par_h7);

    return true;
}


// =====================================================================
// COMPENSAZIONE TEMPERATURA
// =====================================================================

static float bme680_compensateTemp(uint32_t adc_temp)
{
    int64_t var1, var2, var3;

    var1 = ((int32_t)adc_temp >> 3) - ((int32_t)bme680_cal.par_t1 << 1);
    var2 = (var1 * (int32_t)bme680_cal.par_t2) >> 11;
    var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
    var3 = (var3 * ((int32_t)bme680_cal.par_t3 << 4)) >> 14;

    bme680_cal.t_fine = (int32_t)(var2 + var3);

    float temp = ((bme680_cal.t_fine * 5) + 128) >> 8;
    return temp / 100.0f;
}


// =====================================================================
// COMPENSAZIONE PRESSIONE
// =====================================================================

static float bme680_compensatePres(uint32_t adc_pres)
{
    int32_t var1, var2, var3, press_comp;

    var1 = (bme680_cal.t_fine >> 1) - 64000;
    var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)bme680_cal.par_p6) >> 2;
    var2 = var2 + ((var1 * (int32_t)bme680_cal.par_p5) << 1);
    var2 = (var2 >> 2) + ((int32_t)bme680_cal.par_p4 << 16);

    var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) *
             ((int32_t)bme680_cal.par_p3 << 5)) >> 3) +
            (((int32_t)bme680_cal.par_p2 * var1) >> 1);
    var1 = var1 >> 18;
    var1 = ((32768 + var1) * (int32_t)bme680_cal.par_p1) >> 15;

    press_comp = 1048576 - (int32_t)adc_pres;
    press_comp = (int32_t)((press_comp - (var2 >> 12)) * ((int32_t)3125));

    if (press_comp >= (int32_t)0x40000000)
        press_comp = ((press_comp / var1) << 1);
    else
        press_comp = ((press_comp << 1) / var1);

    var1 = ((int32_t)bme680_cal.par_p9 *
            (int32_t)(((press_comp >> 3) * (press_comp >> 3)) >> 13)) >> 12;
    var2 = ((int32_t)(press_comp >> 2) * (int32_t)bme680_cal.par_p8) >> 13;
    var3 = ((int32_t)(press_comp >> 8) * (int32_t)(press_comp >> 8) *
            (int32_t)(press_comp >> 8) * (int32_t)bme680_cal.par_p10) >> 17;

    press_comp = (int32_t)(press_comp) + ((var1 + var2 + var3 +
                 ((int32_t)bme680_cal.par_p7 << 7)) >> 4);

    return (float)press_comp / 100.0f;
}


// =====================================================================
// COMPENSAZIONE UMIDITA' — CORRETTO (Bosch BME680 API reference)
// =====================================================================

static float bme680_compensateHum(uint16_t adc_hum)
{
    int32_t var1, var2, var3, var4, var5, var6;
    int32_t temp_scaled = ((bme680_cal.t_fine * 5) + 128) >> 8;

    var1 = (int32_t)adc_hum - (int32_t)((int32_t)bme680_cal.par_h1 << 4) -
           (((temp_scaled * (int32_t)bme680_cal.par_h3) / 100) >> 1);

    var2 = ((int32_t)bme680_cal.par_h2 *
            (((temp_scaled * (int32_t)bme680_cal.par_h4) / 100) +
             (((temp_scaled * ((temp_scaled * (int32_t)bme680_cal.par_h5) / 100)) >> 6) / 100) +
             (int32_t)(1 << 14))) >> 10;

    var3 = var1 * var2;

    var4 = (int32_t)bme680_cal.par_h6 << 7;
    var4 = ((var4) + ((temp_scaled * (int32_t)bme680_cal.par_h7) / 100)) >> 4;

    var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
    var6 = (var4 * var5) >> 1;

    int32_t calc_hum = (((var3 + var6) >> 10) * ((int32_t)1000)) >> 12;

    if (calc_hum > 100000) calc_hum = 100000;
    if (calc_hum < 0)      calc_hum = 0;

    return (float)calc_hum / 1000.0f;
}


// =====================================================================
// COMPENSAZIONE GAS RESISTANCE
// =====================================================================

static uint32_t bme680_compensateGas(uint16_t adc_gas, uint8_t gas_range)
{
    static const uint32_t lookupK1[16] = {
        2147483647UL, 2147483647UL, 2147483647UL, 2147483647UL,
        2147483647UL, 2126008810UL, 2147483647UL, 2130303777UL,
        2147483647UL, 2147483647UL, 2143188679UL, 2136746228UL,
        2147483647UL, 2126008810UL, 2147483647UL, 2147483647UL
    };

    static const uint32_t lookupK2[16] = {
        4096000000UL, 2048000000UL, 1024000000UL, 512000000UL,
        255744255UL,  127110228UL,  64000000UL,   32258064UL,
        16016016UL,   8000000UL,    4000000UL,    2000000UL,
        1000000UL,    500000UL,     250000UL,     125000UL
    };

    int64_t var1 = (int64_t)((1340 + (5 * (int64_t)bme680_cal.range_sw_err)) *
                   ((int64_t)lookupK1[gas_range])) >> 16;
    int64_t var2 = (int64_t)((int64_t)adc_gas << 15) - (int64_t)(1 << 24) + var1;
    int64_t var3 = ((int64_t)lookupK2[gas_range] * (int64_t)var1) >> 9;

    uint32_t gas_res = (uint32_t)((var3 + ((int64_t)var2 >> 1)) / (int64_t)var2);
    return gas_res;
}


// =====================================================================
// HEATER RESISTANCE + GAS WAIT
// =====================================================================

static uint8_t bme680_calcHeaterRes(uint16_t target_temp)
{
    int32_t amb_temp = 25;
    if (bme680_cal.t_fine != 0) {
        amb_temp = (((bme680_cal.t_fine * 5) + 128) >> 8) / 100;
    }
    if (target_temp > 400) target_temp = 400;

    int32_t var1 = (((int32_t)amb_temp * bme680_cal.par_gh3) / 1000) * 256;
    int32_t var2 = (bme680_cal.par_gh1 + 784) *
           (((((bme680_cal.par_gh2 + 154009) * (int32_t)target_temp * 5) / 100) + 3276800) / 10);
    int32_t var3 = var1 + (var2 / 2);
    int32_t var4 = (var3 / (bme680_cal.res_heat_range + 4));
    int32_t var5 = (131 * bme680_cal.res_heat_val) + 65536;

    int32_t heatr_res_x100 = (int32_t)(((var4 / var5) - 250) * 34);
    return (uint8_t)((heatr_res_x100 + 50) / 100);
}

static uint8_t bme680_calcGasWait(uint16_t dur_ms)
{
    uint8_t factor = 0;
    if (dur_ms >= 0xFC0) return 0xFF;
    while (dur_ms > 0x3F) {
        dur_ms = dur_ms >> 2;
        factor++;
    }
    return (uint8_t)(dur_ms + (factor * 64));
}


// =====================================================================
// INIT BME680
// =====================================================================
BmeData bmeRead(void);

bool bmeInit(void)
{
    Serial.println("[BME680] Init bare-metal driver...");

    uint8_t chip_id = bme680_read8(BME680_CHIP_ID_REG);
    if (chip_id != BME680_CHIP_ID_VAL) {
        Serial.print("[BME680] Chip ID errato: 0x");
        Serial.println(chip_id, HEX);
        bme680_ok = false;
        return false;
    }
    Serial.println("[BME680] Chip ID OK (0x61)");

    // Soft reset
    bme680_write8(BME680_RESET_REG, BME680_RESET_CMD);
    delay(10);

    // Leggi calibrazione
    if (!bme680_readCalibration()) {
        Serial.println("[BME680] Errore lettura calibrazione!");
        bme680_ok = false;
        return false;
    }
    Serial.println("[BME680] Calibrazione OK");

    // --- CONFIGURAZIONE REGISTRI ---
    // Ordine critico: ctrl_hum -> config -> gas -> ctrl_meas

    // 1. Humidity oversampling (DEVE essere scritto PRIMA di ctrl_meas)
    bme680_write8(BME680_CTRL_HUM_REG, BME680_OS_2X);

    // 2. IIR Filter = 3
    bme680_write8(BME680_CONFIG_REG, (BME680_FILTER_3 << 2));

    // 3. Gas heater target e durata
    bme680_write8(BME680_RES_HEAT_0_REG, bme680_calcHeaterRes(BME680_HEATER_TEMP_C));
    bme680_write8(BME680_GAS_WAIT_0_REG, bme680_calcGasWait(BME680_HEATER_DUR_MS));

    // 4. FIX: Abilitare l'heater esplicitamente!
    //    Dopo soft reset, ctrl_gas_0 (0x70) = 0x08 -> heat_off=1 (SPENTO!)
    //    Dobbiamo mettere heat_off=0 per accendere l'heater
    bme680_write8(BME680_CTRL_GAS0_REG, 0x00);  // heat_off=0 -> heater ATTIVO

    // Verifica registro
    uint8_t ctrl_gas0_check = bme680_read8(BME680_CTRL_GAS0_REG);
    Serial.print("[BME680] ctrl_gas_0 (0x70) = 0x");
    Serial.print(ctrl_gas0_check, HEX);
    Serial.println(ctrl_gas0_check == 0x00 ? " [HEATER ON]" : " [HEATER OFF!]");

    // 5. Abilita gas measurement: run_gas=1, nb_conv=0 (heater profile 0)
    bme680_write8(BME680_CTRL_GAS1_REG, 0x10);

    // Verifica registri critici
    uint8_t ctrl_hum_check  = bme680_read8(BME680_CTRL_HUM_REG);
    uint8_t ctrl_gas1_check = bme680_read8(BME680_CTRL_GAS1_REG);
    Serial.print("[BME680] ctrl_hum  (0x72) = 0x"); Serial.println(ctrl_hum_check, HEX);
    Serial.print("[BME680] ctrl_gas1 (0x71) = 0x"); Serial.println(ctrl_gas1_check, HEX);

    bme680_ok = true;
    Serial.println("[BME680] Init OK (bare-metal, zero heap)");

    // 6. Lettura warmup: la prima misura pre-riscalda l'heater
    //    Il risultato VOC sara' probabilmente ancora instabile, ma
    //    alla seconda lettura (quella vera del ciclo) sara' stabile
    Serial.println("[BME680] Warmup read (pre-riscaldamento heater)...");
    BmeData warmup = bmeRead();
    Serial.print("[BME680] Warmup: T="); Serial.print(warmup.temperature, 1);
    Serial.print(" H="); Serial.print(warmup.humidity, 1);
    Serial.print(" VOC="); Serial.print(warmup.voc_resistance);
    Serial.println(" (scartato)");

    return true;
}


// =====================================================================
// LETTURA BME680 (forced mode)
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

    // ctrl_hum va riscritto PRIMA di ogni forced mode (datasheet)
    bme680_write8(BME680_CTRL_HUM_REG, BME680_OS_2X);

    // Heater: ricarica target e durata prima di ogni misura
    bme680_write8(BME680_RES_HEAT_0_REG, bme680_calcHeaterRes(BME680_HEATER_TEMP_C));
    bme680_write8(BME680_GAS_WAIT_0_REG, bme680_calcGasWait(BME680_HEATER_DUR_MS));
    bme680_write8(BME680_CTRL_GAS0_REG, 0x00);   // heat_off=0
    bme680_write8(BME680_CTRL_GAS1_REG, 0x10);    // run_gas=1

    // Trigger forced mode: osrs_t=8x, osrs_p=4x, mode=forced
    uint8_t ctrl_meas = (BME680_OS_8X << 5) | (BME680_OS_4X << 2) | BME680_MODE_FORCED;
    bme680_write8(BME680_CTRL_MEAS_REG, ctrl_meas);

    // Attendi completamento (polling new_data_0, bit 7 di 0x1D)
    uint8_t status;
    uint16_t timeout = 500;
    do {
        delay(10);
        status = bme680_read8(BME680_EAS_STATUS_REG);
        timeout -= 10;
    } while (!(status & 0x80) && timeout > 0);

    if (!(status & 0x80)) {
        Serial.println("[BME680] Timeout lettura!");
        return data;
    }

    // Leggi dati raw (0x1D..0x2B = 15 bytes)
    uint8_t raw[15];
    if (!bme680_readBurst(0x1D, raw, 15)) {
        Serial.println("[BME680] Errore I2C burst read!");
        return data;
    }

    // Parse ADC
    uint32_t adc_pres = ((uint32_t)raw[2] << 12) | ((uint32_t)raw[3] << 4) | (raw[4] >> 4);
    uint32_t adc_temp = ((uint32_t)raw[5] << 12) | ((uint32_t)raw[6] << 4) | (raw[7] >> 4);
    uint16_t adc_hum  = ((uint16_t)raw[8] << 8) | raw[9];
    uint16_t adc_gas  = ((uint16_t)raw[13] << 2) | (raw[14] >> 6);
    uint8_t gas_range = raw[14] & 0x0F;
    bool gas_valid    = (raw[14] & 0x20) != 0;
    bool heat_stab    = (raw[14] & 0x10) != 0;

    // Compensazione (T PRIMA, poi P e H)
    data.temperature = bme680_compensateTemp(adc_temp);
    data.pressure = bme680_compensatePres(adc_pres);
    data.humidity = bme680_compensateHum(adc_hum);

    if (gas_valid && heat_stab) {
        data.voc_resistance = bme680_compensateGas(adc_gas, gas_range);
    } else if (gas_valid && !heat_stab) {
        // Heater non ancora stabile, ma il gas ADC e' disponibile
        // Lo riportiamo comunque per diagnostica, con flag
        data.voc_resistance = bme680_compensateGas(adc_gas, gas_range);
        Serial.print("[BME680] VOC (heater non stabile): ");
        Serial.print(data.voc_resistance);
        Serial.println(" Ohm [APPROX]");
    }

    data.valid = true;

    // Log seriale
    Serial.print("[BME680] Temp: "); Serial.print(data.temperature, 2); Serial.println(" C");
    Serial.print("[BME680] Hum:  "); Serial.print(data.humidity, 2); Serial.println(" %");
    Serial.print("[BME680] Pres: "); Serial.print(data.pressure, 2); Serial.println(" hPa");
    Serial.print("[BME680] VOC:  "); Serial.print(data.voc_resistance); Serial.print(" Ohm");
    if (!gas_valid) Serial.print(" [GAS_INVALID]");
    else if (!heat_stab) Serial.print(" [HEATER_UNSTABLE]");
    else Serial.print(" [STABLE]");
    Serial.println();

    return data;
}


// =====================================================================
// RESET
// =====================================================================

void bmeReset(void)
{
    bme680_ok = false;
}


#endif // BME680_SENSOR_H

