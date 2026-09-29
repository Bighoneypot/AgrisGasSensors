
// =====================================================================
// rtc_time.h - DRIVER RV-3028 RTC (RAK12002)
// =====================================================================
// MYRUMINET - Sensore Gas Stalla v6.6
// Chip: Micro Crystal RV-3028-C7
// Indirizzo I2C: 0x52
// =====================================================================

#ifndef RTC_TIME_H
#define RTC_TIME_H

#include "config.h"
#include <Wire.h>


// =====================================================================
// RV-3028 REGISTRI
// =====================================================================

#define RV3028_SECONDS      0x00
#define RV3028_MINUTES      0x01
#define RV3028_HOURS        0x02
#define RV3028_WEEKDAY      0x03
#define RV3028_DATE         0x04
#define RV3028_MONTH        0x05
#define RV3028_YEAR         0x06
#define RV3028_STATUS       0x0E
#define RV3028_CONTROL1     0x0F
#define RV3028_CONTROL2     0x10


// =====================================================================
// STRUTTURA TEMPO
// =====================================================================

struct RtcTime {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    bool valid;
};

static RtcTime currentTime;
static bool rtc_ok = false;


// =====================================================================
// FUNZIONI HELPER BCD
// =====================================================================

static uint8_t bcdToDec(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

static uint8_t decToBcd(uint8_t dec)
{
    return ((dec / 10) << 4) | (dec % 10);
}


// =====================================================================
// LETTURA/SCRITTURA REGISTRO I2C
// =====================================================================

static uint8_t rtcReadReg(uint8_t reg)
{
    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)RTC_I2C_ADDR, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0xFF;
}

static void rtcWriteReg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}


// =====================================================================
// INIT RTC
// =====================================================================

bool rtcInit(void)
{
    Wire.beginTransmission(RTC_I2C_ADDR);
    uint8_t err = Wire.endTransmission();

    if (err != 0) {
        Serial.print("[RTC] RV-3028 non trovato su 0x");
        Serial.println(RTC_I2C_ADDR, HEX);
        rtc_ok = false;
        return false;
    }

    uint8_t status = rtcReadReg(RV3028_STATUS);
    Serial.print("[RTC] RV-3028 trovato su 0x");
    Serial.print(RTC_I2C_ADDR, HEX);
    Serial.print(" | Status: 0x");
    Serial.println(status, HEX);

    if (status & 0x01) {
        Serial.println("[RTC] Power-On Reset rilevato, ora non valida");
    }

    rtc_ok = true;
    Serial.println("[RTC] Init OK");
    return true;
}


// =====================================================================
// LETTURA TEMPO
// =====================================================================

RtcTime rtcGetTime(void)
{
    RtcTime t;
    t.valid = false;

    if (!rtc_ok) {
        Serial.println("[RTC] Non inizializzato");
        return t;
    }

    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(RV3028_SECONDS);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)RTC_I2C_ADDR, (uint8_t)7);

    if (Wire.available() < 7) {
        Serial.println("[RTC] Errore lettura tempo");
        return t;
    }

    t.second = bcdToDec(Wire.read() & 0x7F);
    t.minute = bcdToDec(Wire.read() & 0x7F);
    t.hour   = bcdToDec(Wire.read() & 0x3F);
    Wire.read(); // weekday (skip)
    t.day    = bcdToDec(Wire.read() & 0x3F);
    t.month  = bcdToDec(Wire.read() & 0x1F);
    t.year   = 2000 + bcdToDec(Wire.read());
    t.valid  = true;

    return t;
}


// =====================================================================
// SET TEMPO
// =====================================================================

bool rtcSetTime(uint16_t year, uint8_t month, uint8_t day,
                uint8_t hour, uint8_t minute, uint8_t second)
{
    if (!rtc_ok) {
        Serial.println("[RTC] Non inizializzato");
        return false;
    }

    rtcWriteReg(RV3028_SECONDS, decToBcd(second));
    rtcWriteReg(RV3028_MINUTES, decToBcd(minute));
    rtcWriteReg(RV3028_HOURS,   decToBcd(hour));
    rtcWriteReg(RV3028_DATE,    decToBcd(day));
    rtcWriteReg(RV3028_MONTH,   decToBcd(month));
    rtcWriteReg(RV3028_YEAR,    decToBcd(year - 2000));

    Serial.print("[RTC] Ora impostata: ");
    Serial.print(year); Serial.print("-");
    if (month < 10) Serial.print("0"); Serial.print(month); Serial.print("-");
    if (day < 10) Serial.print("0"); Serial.print(day); Serial.print("T");
    if (hour < 10) Serial.print("0"); Serial.print(hour); Serial.print(":");
    if (minute < 10) Serial.print("0"); Serial.print(minute); Serial.print(":");
    if (second < 10) Serial.print("0"); Serial.println(second);

    return true;
}


// =====================================================================
// FORMATTAZIONE TIMESTAMP ISO 8601
// =====================================================================

static char timestampBuf[24];

const char* rtcGetTimestamp(void)
{
    RtcTime t = rtcGetTime();

    if (!t.valid) {
        snprintf(timestampBuf, sizeof(timestampBuf), "0000-00-00T00:00:00");
        return timestampBuf;
    }

    snprintf(timestampBuf, sizeof(timestampBuf),
        "%04d-%02d-%02dT%02d:%02d:%02d",
        t.year, t.month, t.day,
        t.hour, t.minute, t.second);

    return timestampBuf;
}


// =====================================================================
// AUTO-SET RTC DA ORA DI COMPILAZIONE (primo boot)
// =====================================================================
// Usa __DATE__ e __TIME__ del compilatore per impostare l'RTC.
// Attivo SOLO se il Power-On Reset flag e' attivo (RTC vergine o
// ha perso alimentazione). Al prossimo reboot non sovrascrive.
// Precisione: ~1 minuto (tempo compilazione + upload)

static int parseMonth(const char *monthStr)
{
    // __DATE__ formato: "Sep 28 2026"
    if (monthStr[0] == 'J' && monthStr[1] == 'a') return 1;   // Jan
    if (monthStr[0] == 'F') return 2;                          // Feb
    if (monthStr[0] == 'M' && monthStr[2] == 'r') return 3;   // Mar
    if (monthStr[0] == 'A' && monthStr[1] == 'p') return 4;   // Apr
    if (monthStr[0] == 'M' && monthStr[2] == 'y') return 5;   // May
    if (monthStr[0] == 'J' && monthStr[2] == 'n') return 6;   // Jun
    if (monthStr[0] == 'J' && monthStr[2] == 'l') return 7;   // Jul
    if (monthStr[0] == 'A' && monthStr[1] == 'u') return 8;   // Aug
    if (monthStr[0] == 'S') return 9;                          // Sep
    if (monthStr[0] == 'O') return 10;                         // Oct
    if (monthStr[0] == 'N') return 11;                         // Nov
    if (monthStr[0] == 'D') return 12;                         // Dec
    return 1;
}

bool rtcAutoSetFromCompileTime(void)
{
    if (!rtc_ok) {
        Serial.println("[RTC] Non inizializzato, impossibile auto-set");
        return false;
    }

    // Controlla il Power-On Reset flag (bit 0 del registro STATUS)
    uint8_t status = rtcReadReg(RV3028_STATUS);

    if (!(status & 0x01)) {
        // POR flag non attivo = RTC ha gia' un'ora valida
        Serial.println("[RTC] Ora gia' impostata, skip auto-set");
        return false;
    }

    Serial.println("[RTC] Power-On Reset rilevato -> auto-set da ora compilazione");

    // Parse __DATE__ = "Sep 28 2026"
    const char *dateStr = __DATE__;
    int month = parseMonth(dateStr);
    int day   = atoi(&dateStr[4]);
    int year  = atoi(&dateStr[7]);

    // Parse __TIME__ = "11:12:30"
    const char *timeStr = __TIME__;
    int hour   = (timeStr[0] - '0') * 10 + (timeStr[1] - '0');
    int minute = (timeStr[3] - '0') * 10 + (timeStr[4] - '0');
    int second = (timeStr[6] - '0') * 10 + (timeStr[7] - '0');

    Serial.print("[RTC] Ora compilazione: ");
    Serial.print(year); Serial.print("-");
    if (month < 10) Serial.print("0"); Serial.print(month); Serial.print("-");
    if (day < 10) Serial.print("0"); Serial.print(day); Serial.print("T");
    if (hour < 10) Serial.print("0"); Serial.print(hour); Serial.print(":");
    if (minute < 10) Serial.print("0"); Serial.print(minute); Serial.print(":");
    if (second < 10) Serial.print("0"); Serial.println(second);

    // Imposta l'ora
    rtcSetTime(year, month, day, hour, minute, second);

    // Pulisci il POR flag cosi' al prossimo reboot non sovrascrive
    rtcWriteReg(RV3028_STATUS, status & ~0x01);

    Serial.println("[RTC] Auto-set completato! POR flag pulito.");
    Serial.println("[RTC] NOTA: precisione ~1 min (tempo compilazione + upload)");
    Serial.println("[RTC] Per ora esatta usare: AT+SETTIME=YYYY,MM,DD,HH,MM,SS");

    return true;
}


// =====================================================================
// STAMPA TEMPO SU SERIALE
// =====================================================================

void rtcPrintTime(void)
{
    RtcTime t = rtcGetTime();

    if (!t.valid) {
        Serial.println("[RTC] Tempo non valido");
        return;
    }

    Serial.print("[RTC] ");
    Serial.print(t.year); Serial.print("-");
    if (t.month < 10) Serial.print("0"); Serial.print(t.month); Serial.print("-");
    if (t.day < 10) Serial.print("0"); Serial.print(t.day); Serial.print(" ");
    if (t.hour < 10) Serial.print("0"); Serial.print(t.hour); Serial.print(":");
    if (t.minute < 10) Serial.print("0"); Serial.print(t.minute); Serial.print(":");
    if (t.second < 10) Serial.print("0"); Serial.println(t.second);
}


#endif // RTC_TIME_H

