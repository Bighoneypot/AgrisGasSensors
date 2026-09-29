
// =====================================================================
// lorawan_manager.h - CONFIGURAZIONE, JOIN, TX, PAYLOAD LORAWAN
// =====================================================================
// MYRUMINET - Sensore Gas Stalla v6.6
// LoRaWAN OTAA - EU868 - Class A - fPort 77
//
// CHECK PREVENTIVI NVM:
//   Prima di ogni set(), verifica se il valore e' gia' corretto
//   in NVM. RUI3 puo' ritornare FAIL se il valore e' gia' impostato.
//   Questo evita i "FAIL" intermittenti al boot.
//
// PAYLOAD IDENTICO AL v6.5.5 (19 bytes):
//   Byte 0-1:   Temperatura (int16, *100)
//   Byte 2-3:   Umidita (uint16, *100)
//   Byte 4-7:   Pressione (uint32, *100)
//   Byte 8-11:  VOC Resistance (uint32, Ohm)
//   Byte 12-13: NH3 (uint16, *10)
//   Byte 14-15: H2S (uint16, *10)
//   Byte 16-17: Batteria mV (uint16)
//   Byte 18:    Batteria SOC (uint8, %)
// =====================================================================

#ifndef LORAWAN_MANAGER_H
#define LORAWAN_MANAGER_H

#include "config.h"


// =====================================================================
// PAYLOAD BUFFER
// =====================================================================

static uint8_t payload[19];


// =====================================================================
// HELPER: confronto array di bytes
// =====================================================================

static bool arrayEqual(const uint8_t *a, const uint8_t *b, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        if (a[i] != b[i]) return false;
    }
    return true;
}


// =====================================================================
// INIT LORAWAN (con check preventivi NVM)
// =====================================================================

void loraInit(void)
{
    Serial.println("--- CONFIGURAZIONE LORAWAN ---");

    bool ok;

    // -----------------------------------------------------------------
    // CREDENZIALI (check se gia' impostate in NVM)
    // -----------------------------------------------------------------

    // DevEUI
    uint8_t currentDevEui[8];
    api.lorawan.deui.get(currentDevEui, 8);
    if (arrayEqual(currentDevEui, node_dev_eui, 8)) {
        Serial.println("[LORA] Set DevEUI: GIA' OK");
    } else {
        ok = api.lorawan.deui.set(node_dev_eui, 8);
        Serial.print("[LORA] Set DevEUI: ");
        Serial.println(ok ? "OK" : "FAIL");
    }

    // AppEUI
    uint8_t currentAppEui[8];
    api.lorawan.appeui.get(currentAppEui, 8);
    if (arrayEqual(currentAppEui, node_app_eui, 8)) {
        Serial.println("[LORA] Set AppEUI: GIA' OK");
    } else {
        ok = api.lorawan.appeui.set(node_app_eui, 8);
        Serial.print("[LORA] Set AppEUI: ");
        Serial.println(ok ? "OK" : "FAIL");
    }

    // AppKey
    uint8_t currentAppKey[16];
    api.lorawan.appkey.get(currentAppKey, 16);
    if (arrayEqual(currentAppKey, node_app_key, 16)) {
        Serial.println("[LORA] Set AppKey: GIA' OK");
    } else {
        ok = api.lorawan.appkey.set(node_app_key, 16);
        Serial.print("[LORA] Set AppKey: ");
        Serial.println(ok ? "OK" : "FAIL");
    }

    Serial.println();

    // -----------------------------------------------------------------
    // PARAMETRI RADIO (check se gia' impostati in NVM)
    // -----------------------------------------------------------------

    // Banda EU868
    if (api.lorawan.band.get() == LORAWAN_BAND) {
        Serial.println("[LORA] Banda EU868: GIA' OK");
    } else {
        ok = api.lorawan.band.set(LORAWAN_BAND);
        Serial.print("[LORA] Banda EU868: ");
        Serial.println(ok ? "OK" : "FAIL");
        if (!ok) {
            Serial.println("[LORA] WARN: Banda non impostata! Retry...");
            delay(500);
            ok = api.lorawan.band.set(LORAWAN_BAND);
            Serial.print("[LORA] Banda EU868 retry: ");
            Serial.println(ok ? "OK" : "FAIL");
        }
    }

    // Classe A
    if (api.lorawan.deviceClass.get() == LORAWAN_CLASS) {
        Serial.println("[LORA] Classe A: GIA' OK");
    } else {
        ok = api.lorawan.deviceClass.set(LORAWAN_CLASS);
        Serial.print("[LORA] Classe A: ");
        Serial.println(ok ? "OK" : "FAIL");
    }

    // OTAA
    if (api.lorawan.njm.get() == RAK_LORA_OTAA) {
        Serial.println("[LORA] OTAA: GIA' OK");
    } else {
        ok = api.lorawan.njm.set(RAK_LORA_OTAA);
        Serial.print("[LORA] OTAA: ");
        Serial.println(ok ? "OK" : "FAIL");
    }

    // ADR
    if (api.lorawan.adr.get() == true) {
        Serial.println("[LORA] ADR ON: GIA' OK");
    } else {
        ok = api.lorawan.adr.set(true);
        Serial.print("[LORA] ADR ON: ");
        Serial.println(ok ? "OK" : "FAIL");
    }

    // DR iniziale (DR0 per join)
    if (api.lorawan.dr.get() == LORAWAN_DR_JOIN) {
        Serial.println("[LORA] DR0 iniziale: GIA' OK");
    } else {
        ok = api.lorawan.dr.set(LORAWAN_DR_JOIN);
        Serial.print("[LORA] DR0 iniziale: ");
        Serial.println(ok ? "OK" : "FAIL");
    }

    // RX2 DR
    if (api.lorawan.rx2dr.get() == 0) {
        Serial.println("[LORA] RX2 DR0: GIA' OK");
    } else {
        ok = api.lorawan.rx2dr.set(0);
        Serial.print("[LORA] RX2 DR0: ");
        Serial.println(ok ? "OK" : "FAIL");
    }

    // RX1 Delay
    if (api.lorawan.rx1dl.get() == LORAWAN_RX1DL) {
        Serial.println("[LORA] RX1 DL 5000ms: GIA' OK");
    } else {
        ok = api.lorawan.rx1dl.set(LORAWAN_RX1DL);
        Serial.print("[LORA] RX1 DL 5000ms: ");
        Serial.println(ok ? "OK" : "FAIL");
    }

    // RX2 Delay
    if (api.lorawan.rx2dl.get() == LORAWAN_RX2DL) {
        Serial.println("[LORA] RX2 DL 6000ms: GIA' OK");
    } else {
        ok = api.lorawan.rx2dl.set(LORAWAN_RX2DL);
        Serial.print("[LORA] RX2 DL 6000ms: ");
        Serial.println(ok ? "OK" : "FAIL");
    }

    Serial.println();

    // -----------------------------------------------------------------
    // STAMPA STATO FINALE
    // -----------------------------------------------------------------

    Serial.println("--- STATO LORAWAN ---");
    Serial.print("  NWM:   "); Serial.println(api.lorawan.nwm.get());
    Serial.print("  NJM:   "); Serial.println(api.lorawan.njm.get());
    Serial.print("  BAND:  "); Serial.println(api.lorawan.band.get());
    Serial.print("  DR:    "); Serial.println(api.lorawan.dr.get());
    Serial.print("  ADR:   "); Serial.println(api.lorawan.adr.get());
    Serial.print("  TXP:   "); Serial.println(api.lorawan.txp.get());
    Serial.print("  RX1DL: "); Serial.println(api.lorawan.rx1dl.get());
    Serial.print("  RX2DL: "); Serial.println(api.lorawan.rx2dl.get());
    Serial.print("  RX2DR: "); Serial.println(api.lorawan.rx2dr.get());
    Serial.println();
}


// =====================================================================
// STAMPA INFO DISPOSITIVO
// =====================================================================

void loraPrintDeviceInfo(void)
{
    Serial.println("--- INFO DISPOSITIVO ---");
    Serial.print("  Firmware RUI3: ");
    Serial.println(api.system.firmwareVersion.get().c_str());
    Serial.print("  Modello:       ");
    Serial.println(api.system.modelId.get().c_str());
    Serial.println();

    Serial.println("--- CREDENZIALI LORAWAN ---");

    uint8_t devEui[8];
    api.lorawan.deui.get(devEui, 8);
    Serial.print("  DevEUI: ");
    for (int i = 0; i < 8; i++) {
        if (devEui[i] < 0x10) Serial.print("0");
        Serial.print(devEui[i], HEX);
    }
    Serial.println();

    uint8_t appEui[8];
    api.lorawan.appeui.get(appEui, 8);
    Serial.print("  AppEUI: ");
    for (int i = 0; i < 8; i++) {
        if (appEui[i] < 0x10) Serial.print("0");
        Serial.print(appEui[i], HEX);
    }
    Serial.println();

    uint8_t appKey[16];
    api.lorawan.appkey.get(appKey, 16);
    Serial.print("  AppKey: ");
    for (int i = 0; i < 16; i++) {
        if (appKey[i] < 0x10) Serial.print("0");
        Serial.print(appKey[i], HEX);
    }
    Serial.println();
    Serial.println();
}


// =====================================================================
// JOIN NETWORK
// =====================================================================

bool loraJoin(void)
{
    Serial.println("[LORA] Avvio Join OTAA...");

    api.lorawan.join();

    int polls = 0;
    while (polls < JOIN_MAX_POLLS) {
        delay(JOIN_POLL_INTERVAL);

        if (api.lorawan.njs.get()) {
            joined = true;
            Serial.println("[LORA] JOIN RIUSCITO!");

            api.lorawan.dr.set(LORAWAN_DR_TX);
            Serial.println("[LORA] DR impostato a 5 (SF7)");
            Serial.println();

            return true;
        }

        polls++;
        Serial.print("[LORA] Attendo join... ");
        Serial.print(polls * 5);
        Serial.println("s");
    }

    Serial.println("[LORA] JOIN FALLITO dopo 150 secondi");
    joined = false;
    return false;
}


// =====================================================================
// BUILD PAYLOAD (19 bytes) - IDENTICO AL v6.5.5
// =====================================================================
//
// Byte 0-1:   Temperatura (int16, *100)     es. 25.30 -> 2530
// Byte 2-3:   Umidita (uint16, *100)        es. 68.20 -> 6820
// Byte 4-7:   Pressione (uint32, *100)      es. 1013.25 -> 101325
// Byte 8-11:  VOC Resistance (uint32, Ohm)  es. 50000
// Byte 12-13: NH3 (uint16, *10)             es. 12.5 -> 125
// Byte 14-15: H2S (uint16, *10)             es. 0.8 -> 8
// Byte 16-17: Batteria mV (uint16)          es. 4007
// Byte 18:    Batteria SOC (uint8, %)       es. 90

void loraBuildPayload(float temp, float hum, float pres, uint32_t voc,
                      float nh3, float h2s, uint16_t bat_mv, uint8_t bat_soc)
{
    memset(payload, 0, sizeof(payload));

    // Byte 0-1: Temperatura (int16, *100)
    int16_t temp_encoded = (int16_t)(temp * 100);
    payload[0] = (temp_encoded >> 8) & 0xFF;
    payload[1] = temp_encoded & 0xFF;

    // Byte 2-3: Umidita (uint16, *100)
    uint16_t hum_encoded = (uint16_t)(hum * 100);
    payload[2] = (hum_encoded >> 8) & 0xFF;
    payload[3] = hum_encoded & 0xFF;

    // Byte 4-7: Pressione (uint32, *100)
    uint32_t pres_encoded = (uint32_t)(pres * 100);
    payload[4] = (pres_encoded >> 24) & 0xFF;
    payload[5] = (pres_encoded >> 16) & 0xFF;
    payload[6] = (pres_encoded >> 8) & 0xFF;
    payload[7] = pres_encoded & 0xFF;

    // Byte 8-11: VOC Resistance (uint32)
    payload[8] = (voc >> 24) & 0xFF;
    payload[9] = (voc >> 16) & 0xFF;
    payload[10] = (voc >> 8) & 0xFF;
    payload[11] = voc & 0xFF;

    // Byte 12-13: NH3 (uint16, *10)
    uint16_t nh3_encoded = (uint16_t)(nh3 * 10);
    payload[12] = (nh3_encoded >> 8) & 0xFF;
    payload[13] = nh3_encoded & 0xFF;

    // Byte 14-15: H2S (uint16, *10)
    uint16_t h2s_encoded = (uint16_t)(h2s * 10);
    payload[14] = (h2s_encoded >> 8) & 0xFF;
    payload[15] = h2s_encoded & 0xFF;

    // Byte 16-17: Batteria mV (uint16)
    payload[16] = (bat_mv >> 8) & 0xFF;
    payload[17] = bat_mv & 0xFF;

    // Byte 18: Batteria SOC (uint8)
    payload[18] = bat_soc;

    // Stampa payload hex
    Serial.print("[PAYLOAD] HEX: ");
    for (int i = 0; i < 19; i++) {
        if (payload[i] < 0x10) Serial.print("0");
        Serial.print(payload[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}


#endif // LORAWAN_MANAGER_H

