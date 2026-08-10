/// =====================================================================
// MYRUMINET - Sensore Gas Stalla
// FIRMWARE v4 - LOW POWER
// =====================================================================
// RAK3172-E / RAK19007 Rev.C
// RAK1920 Sensor Adapter
// BME680 + DFRobot NH3 + DFRobot H2S
//
// LoRaWAN OTAA - EU868
// Class A
// fPort 77
// Invio ogni 30 minuti
//
// RUI3 / RUI 4.2.4
//
// FIX v4:
// - Vero low-power con api.system.sleep.all()
// - Disabilitare RX1/RX2 prima della sleep
// - SEN0469/H2S rimangono alimentati tramite 3V3_S
// - Nessun Wire.end() prima dello sleep
// - I2C riinizializzato dopo il wake-up
// - Nessun nuovo JOIN ad ogni ciclo
// - ADR ABILITATO
// - Nessun falso warm-up per NH3/H2S
// - BME680 configurato correttamente
// =====================================================================


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include "DFRobot_MultiGasSensor.h"


// =====================================================================
// CONFIGURAZIONE
// =====================================================================

// ---------------------------------------------------------------------
// LoRaWAN
// ---------------------------------------------------------------------

#define LORAWAN_BAND       RAK_REGION_EU868
#define LORAWAN_CLASS      CLASS_A
#define LORAWAN_FPORT      77

// DR utilizzato inizialmente per il JOIN
#define LORAWAN_DR_JOIN    0

// DR iniziale dopo il JOIN.
// Con ADR attivo questo valore potrà essere modificato dalla rete.
#define LORAWAN_DR_TX      5

// RX timing
#define LORAWAN_RX1DL      5000
#define LORAWAN_RX2DL      6000


// ---------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------

// 30 minuti
#define SEND_INTERVAL      1800000UL

// Polling JOIN
#define JOIN_POLL_INTERVAL 5000
#define JOIN_MAX_POLLS     30


// ---------------------------------------------------------------------
// Indirizzi I2C
// ---------------------------------------------------------------------

#define NH3_I2C_ADDR       0x77
#define H2S_I2C_ADDR       0x74
#define BME680_I2C_ADDR    0x76


// ---------------------------------------------------------------------
// Batteria
// ---------------------------------------------------------------------

#define BATTERY_PIN        WB_A0


// =====================================================================
// OGGETTI GLOBALI
// =====================================================================

Adafruit_BME680 bme;

DFRobot_GAS_I2C gasSensorNH3(&Wire, NH3_I2C_ADDR);
DFRobot_GAS_I2C gasSensorH2S(&Wire, H2S_I2C_ADDR);


// =====================================================================
// STATO
// =====================================================================

bool nh3_ok = false;
bool h2s_ok = false;
bool bme680_ok = false;

bool joined = false;


// =====================================================================
// PAYLOAD
// =====================================================================

uint8_t payload[19];


// =====================================================================
// SETUP
// =====================================================================

void setup() {

  Serial.begin(115200);
  api.system.lpm.set(1);
  delay(3000);

  Serial.println();
  Serial.println("=============================================");
  Serial.println("  MYRUMINET - SENSORE GAS STALLA v4");
  Serial.println("  RAK3172-E / RAK19007 Rev.C");
  Serial.println("  RAK1920 + BME680 + NH3 + H2S");
  Serial.println("  LoRaWAN OTAA - EU868 - fPort 77");
  Serial.println("  LOW POWER - 30 MIN");
  Serial.println("=============================================");
  Serial.println();


  // -------------------------------------------------------------------
  // INFO DISPOSITIVO
  // -------------------------------------------------------------------

  printDeviceInfo();


  // -------------------------------------------------------------------
  // CONFIGURAZIONE LORAWAN
  // -------------------------------------------------------------------

  initLoRaWAN();


  // -------------------------------------------------------------------
  // JOIN
  // -------------------------------------------------------------------

  joinNetwork();


  // -------------------------------------------------------------------
  // I2C
  // -------------------------------------------------------------------

  Wire.begin();

  delay(100);

  Serial.println("[I2C] Bus inizializzato");
  Serial.println();


  // -------------------------------------------------------------------
  // SENSORI
  // -------------------------------------------------------------------

  initSensors();


  Serial.println();
  Serial.println("=============================================");
  Serial.println("  SETUP COMPLETATO");
  Serial.println("  Intervallo invio: 30 minuti");
  Serial.println("  SEN0469/H2S: SEMPRE ALIMENTATI");
  Serial.println("  RAK3172: LOW POWER TRA I CICLI");
  Serial.println("=============================================");
  Serial.println();
}


// =====================================================================
// LOOP
// =====================================================================

void loop() {

  // -------------------------------------------------------------------
  // I2C
  // -------------------------------------------------------------------

  // Dopo il wake-up assicuriamoci che il bus sia inizializzato.
  Wire.begin();
  delay(100);


  // -------------------------------------------------------------------
  // VERIFICA JOIN
  // -------------------------------------------------------------------

  if (!api.lorawan.njs.get()) {

    Serial.println("[LORA] Sessione LoRaWAN non presente.");

    joined = false;

    joinNetwork();


    if (!joined) {

      Serial.println();
      Serial.println("[LORA] Join fallito.");
      Serial.println("[SLEEP] Riprovo tra 5 minuti...");
      Serial.println();

      api.system.sleep.all(300000UL);

      return;
    }
  }

  else {

    joined = true;
  }


  // -------------------------------------------------------------------
  // VARIABILI SENSORI
  // -------------------------------------------------------------------

  float temperature = 0.0;
  float humidity = 0.0;
  float pressure = 0.0;

  uint32_t voc_resistance = 0;

  float nh3_ppm = 0.0;
  float h2s_ppm = 0.0;

  uint16_t battery_mv = 0;
  uint8_t battery_soc = 0;


  // -------------------------------------------------------------------
  // BME680
  // -------------------------------------------------------------------

  if (bme680_ok) {

    Serial.println("--- LETTURA SENSORI ---");

    // Il BME680 usa il proprio gas heater.
    // Non serve un warm-up di 3 secondi ad ogni ciclo.

    if (bme.performReading()) {

      temperature = bme.temperature;
      humidity = bme.humidity;
      pressure = bme.pressure / 100.0;
      voc_resistance = (uint32_t)bme.gas_resistance;


      Serial.print("[BME680] Temp: ");
      Serial.print(temperature, 2);
      Serial.println(" C");


      Serial.print("[BME680] Hum:  ");
      Serial.print(humidity, 2);
      Serial.println(" %");


      Serial.print("[BME680] Pres: ");
      Serial.print(pressure, 2);
      Serial.println(" hPa");


      Serial.print("[BME680] VOC:  ");
      Serial.print(voc_resistance);
      Serial.println(" Ohm");
    }

    else {

      Serial.println("[BME680] Errore lettura!");
    }
  }


  // -------------------------------------------------------------------
  // NH3
  // -------------------------------------------------------------------

  if (nh3_ok) {

    nh3_ppm = gasSensorNH3.readGasConcentrationPPM();

    Serial.print("[NH3]   Conc: ");
    Serial.print(nh3_ppm, 2);
    Serial.println(" ppm");
  }


  // -------------------------------------------------------------------
  // H2S
  // -------------------------------------------------------------------

  if (h2s_ok) {

    h2s_ppm = gasSensorH2S.readGasConcentrationPPM();

    Serial.print("[H2S]   Conc: ");
    Serial.print(h2s_ppm, 2);
    Serial.println(" ppm");
  }


  // -------------------------------------------------------------------
  // BATTERIA
  // -------------------------------------------------------------------

  battery_mv = readBatteryMV();
  battery_soc = estimateSOC(battery_mv);

  Serial.print("[BAT]   Volt: ");
  Serial.print(battery_mv);
  Serial.println(" mV");


  Serial.print("[BAT]   SOC:  ");
  Serial.print(battery_soc);
  Serial.println(" %");


  Serial.println();


  // -------------------------------------------------------------------
  // PAYLOAD
  // -------------------------------------------------------------------

  buildPayload(
    temperature,
    humidity,
    pressure,
    voc_resistance,
    nh3_ppm,
    h2s_ppm,
    battery_mv,
    battery_soc
  );


  // -------------------------------------------------------------------
  // LORAWAN TX
  // -------------------------------------------------------------------

  Serial.print("[LORA] Invio payload (");
  Serial.print(sizeof(payload));
  Serial.print(" bytes) su fPort ");
  Serial.print(LORAWAN_FPORT);
  Serial.println("...");


  bool tx_ok =
    api.lorawan.send(
      sizeof(payload),
      payload,
      LORAWAN_FPORT,
      false
    );


  if (tx_ok) {

    Serial.println("[LORA] Invio riuscito!");
  }

  else {

    Serial.println("[LORA] Invio fallito!");


    // Verifica se la sessione è ancora presente.
    if (!api.lorawan.njs.get()) {

      Serial.println("[LORA] Sessione persa.");
      joined = false;
    }
  }


  // -------------------------------------------------------------------
  // ATTESA CHE LA RADIO COMPLETI LE OPERAZIONI
  // -------------------------------------------------------------------
  // Aspettiamo che le finestre RX1/RX2 siano trascorse.
  delay(6000);


  // ===================================================================
  // ★★★ LOW POWER - MODIFICA PRINCIPALE ★★★
  // ===================================================================
  // Per far dormire il modulo, disabilitiamo le finestre di ricezione
  // (RX1 e RX2) impostandole a 0, poi dormiamo per 30 minuti.
  // Al risveglio le ripristiniamo ai valori originali.
  // ===================================================================

  Serial.println();
  Serial.println("-------------------------------------------");
  Serial.println("[SLEEP] RAK3172 -> LOW POWER");
  Serial.println("[SLEEP] SEN0469/H2S rimangono alimentati");
  Serial.println("[SLEEP] Prossimo ciclo tra 30 minuti");
  Serial.println("-------------------------------------------");
  Serial.println();

  // 1️⃣ Disabilita le finestre di ricezione (così la radio si spegne)
  api.lorawan.rx1dl.set(0);
  api.lorawan.rx2dl.set(0);

  // 2️⃣ Ora il sistema può entrare in deep sleep per 30 minuti
  Serial.println("[SLEEP] Entro in deep sleep per 30 minuti...");
  Serial.flush();

  api.system.sleep.all(SEND_INTERVAL);

  // 3️⃣ Al risveglio, ripristina i valori originali per il prossimo ciclo
  api.lorawan.rx1dl.set(LORAWAN_RX1DL);
  api.lorawan.rx2dl.set(LORAWAN_RX2DL);

  // 4️⃣ Wake-up
  Serial.println();
  Serial.println("=============================================");
  Serial.println("[WAKE] RAK3172 risvegliato");
  Serial.println("=============================================");
  Serial.println();
}


// =====================================================================
// STAMPA INFO DISPOSITIVO
// =====================================================================

void printDeviceInfo() {

  Serial.println("--- INFO DISPOSITIVO ---");


  Serial.print("  Firmware RUI3: ");
  Serial.println(
    api.system.firmwareVersion.get().c_str()
  );


  Serial.print("  Modello:        ");
  Serial.println(
    api.system.modelId.get().c_str()
  );


  Serial.println();


  Serial.println("--- CREDENZIALI LORAWAN ---");


  uint8_t devEui[8];

  api.lorawan.deui.get(
    devEui,
    8
  );


  Serial.print("  DevEUI: ");

  for (int i = 0; i < 8; i++) {

    if (devEui[i] < 0x10)
      Serial.print("0");

    Serial.print(
      devEui[i],
      HEX
    );
  }

  Serial.println();


  uint8_t appEui[8];

  api.lorawan.appeui.get(
    appEui,
    8
  );


  Serial.print("  AppEUI: ");

  for (int i = 0; i < 8; i++) {

    if (appEui[i] < 0x10)
      Serial.print("0");

    Serial.print(
      appEui[i],
      HEX
    );
  }

  Serial.println();


  uint8_t appKey[16];

  api.lorawan.appkey.get(
    appKey,
    16
  );


  Serial.print("  AppKey: ");

  for (int i = 0; i < 16; i++) {

    if (appKey[i] < 0x10)
      Serial.print("0");

    Serial.print(
      appKey[i],
      HEX
    );
  }

  Serial.println();

  Serial.println();
}


// =====================================================================
// INIT SENSORI
// =====================================================================

void initSensors() {

  Serial.println(
    "--- INIZIALIZZAZIONE SENSORI ---"
  );


  // -------------------------------------------------------------------
  // BME680
  // -------------------------------------------------------------------

  Serial.print(
    "[BME680] Init (0x76)... "
  );


  if (bme.begin(
        BME680_I2C_ADDR,
        &Wire
      )) {

    bme680_ok = true;


    bme.setTemperatureOversampling(
      BME680_OS_8X
    );


    bme.setHumidityOversampling(
      BME680_OS_2X
    );


    bme.setPressureOversampling(
      BME680_OS_4X
    );


    bme.setIIRFilterSize(
      BME680_FILTER_SIZE_3
    );


    bme.setGasHeater(
      320,
      150
    );


    Serial.println("OK");
  }

  else {

    Serial.println("FALLITO");
  }


  // -------------------------------------------------------------------
  // NH3
  // -------------------------------------------------------------------

  delay(1000);


  Serial.print(
    "[NH3]   Init (0x77)... "
  );


  if (gasSensorNH3.begin()) {

    nh3_ok = true;


    Serial.print(
      "OK - Gas type: "
    );


    Serial.println(
      gasSensorNH3.queryGasType()
    );


    // Modalità passiva.
    gasSensorNH3.changeAcquireMode(
      gasSensorNH3.PASSIVITY
    );
  }

  else {

    Serial.println("FALLITO");
  }


  // -------------------------------------------------------------------
  // H2S
  // -------------------------------------------------------------------

  delay(1000);


  Serial.print(
    "[H2S]   Init (0x74)... "
  );


  if (gasSensorH2S.begin()) {

    h2s_ok = true;


    Serial.print(
      "OK - Gas type: "
    );


    Serial.println(
      gasSensorH2S.queryGasType()
    );


    // Modalità passiva.
    gasSensorH2S.changeAcquireMode(
      gasSensorH2S.PASSIVITY
    );
  }

  else {

    Serial.println("FALLITO");
  }


  Serial.println();
}


// =====================================================================
// INIT LORAWAN
// =====================================================================

void initLoRaWAN() {

  Serial.println(
    "--- CONFIGURAZIONE LORAWAN ---"
  );


  bool ok;


  // -------------------------------------------------------------------
  // EU868
  // -------------------------------------------------------------------

  ok =
    api.lorawan.band.set(
      LORAWAN_BAND
    );


  Serial.print(
    "[LORA] Banda EU868: "
  );

  Serial.println(
    ok ? "OK" : "FAIL"
  );


  // -------------------------------------------------------------------
  // CLASS A
  // -------------------------------------------------------------------

  ok =
    api.lorawan.deviceClass.set(
      LORAWAN_CLASS
    );


  Serial.print(
    "[LORA] Classe A: "
  );

  Serial.println(
    ok ? "OK" : "FAIL"
  );


  // -------------------------------------------------------------------
  // OTAA
  // -------------------------------------------------------------------

  ok =
    api.lorawan.njm.set(
      RAK_LORA_OTAA
    );


  Serial.print(
    "[LORA] OTAA: "
  );

  Serial.println(
    ok ? "OK" : "FAIL"
  );


  // -------------------------------------------------------------------
  // ADR
  // -------------------------------------------------------------------

  ok =
    api.lorawan.adr.set(true);


  Serial.print(
    "[LORA] ADR ON: "
  );

  Serial.println(
    ok ? "OK" : "FAIL"
  );


  // -------------------------------------------------------------------
  // DR iniziale
  // -------------------------------------------------------------------

  ok =
    api.lorawan.dr.set(
      LORAWAN_DR_JOIN
    );


  Serial.print(
    "[LORA] DR0 iniziale: "
  );

  Serial.println(
    ok ? "OK" : "FAIL"
  );


  // -------------------------------------------------------------------
  // RX2
  // -------------------------------------------------------------------

  ok =
    api.lorawan.rx2dr.set(0);


  Serial.print(
    "[LORA] RX2 DR0: "
  );

  Serial.println(
    ok ? "OK" : "FAIL"
  );


  // -------------------------------------------------------------------
  // RX1 DELAY
  // -------------------------------------------------------------------

  ok =
    api.lorawan.rx1dl.set(
      LORAWAN_RX1DL
    );


  Serial.print(
    "[LORA] RX1 DL 5000ms: "
  );

  Serial.println(
    ok ? "OK" : "FAIL"
  );


  // -------------------------------------------------------------------
  // RX2 DELAY
  // -------------------------------------------------------------------

  ok =
    api.lorawan.rx2dl.set(
      LORAWAN_RX2DL
    );


  Serial.print(
    "[LORA] RX2 DL 6000ms: "
  );

  Serial.println(
    ok ? "OK" : "FAIL"
  );


  Serial.println();


  // -------------------------------------------------------------------
  // STATO
  // -------------------------------------------------------------------

  Serial.println(
    "--- STATO LORAWAN ---"
  );


  Serial.print("  NWM:   ");
  Serial.println(
    api.lorawan.nwm.get()
  );


  Serial.print("  NJM:   ");
  Serial.println(
    api.lorawan.njm.get()
  );


  Serial.print("  BAND:  ");
  Serial.println(
    api.lorawan.band.get()
  );


  Serial.print("  DR:    ");
  Serial.println(
    api.lorawan.dr.get()
  );


  Serial.print("  ADR:   ");
  Serial.println(
    api.lorawan.adr.get()
  );


  Serial.print("  TXP:   ");
  Serial.println(
    api.lorawan.txp.get()
  );


  Serial.print("  RX1DL: ");
  Serial.println(
    api.lorawan.rx1dl.get()
  );


  Serial.print("  RX2DL: ");
  Serial.println(
    api.lorawan.rx2dl.get()
  );


  Serial.print("  RX2DR: ");
  Serial.println(
    api.lorawan.rx2dr.get()
  );


  Serial.println();
}


// =====================================================================
// JOIN OTAA
// =====================================================================

void joinNetwork() {

  Serial.println(
    "[LORA] Avvio Join OTAA..."
  );


  api.lorawan.join();


  int polls = 0;


  while (
    polls < JOIN_MAX_POLLS
  ) {

    delay(
      JOIN_POLL_INTERVAL
    );


    if (
      api.lorawan.njs.get()
    ) {

      joined = true;


      Serial.println(
        "[LORA] JOIN RIUSCITO!"
      );


      // DR iniziale.
      // Con ADR attivo la rete potrà successivamente
      // modificarlo.

      api.lorawan.dr.set(
        LORAWAN_DR_TX
      );


      Serial.println(
        "[LORA] DR iniziale impostato a 5 (SF7)"
      );


      Serial.println();

      return;
    }


    polls++;


    Serial.print(
      "[LORA] Attendo join... "
    );

    Serial.print(
      polls * 5
    );

    Serial.println("s");
  }


  Serial.println(
    "[LORA] JOIN FALLITO dopo 150 secondi"
  );


  joined = false;
}


// =====================================================================
// COSTRUZIONE PAYLOAD
// =====================================================================

void buildPayload(
  float temp,
  float hum,
  float pres,
  uint32_t voc,
  float nh3,
  float h2s,
  uint16_t bat_mv,
  uint8_t bat_soc
) {


  // -------------------------------------------------------------------
  // Byte 0-1
  // Temperatura x100
  // -------------------------------------------------------------------

  int16_t temp_encoded =
    (int16_t)(temp * 100);


  payload[0] =
    (temp_encoded >> 8) & 0xFF;


  payload[1] =
    temp_encoded & 0xFF;


  // -------------------------------------------------------------------
  // Byte 2-3
  // Umidità x100
  // -------------------------------------------------------------------

  uint16_t hum_encoded =
    (uint16_t)(hum * 100);


  payload[2] =
    (hum_encoded >> 8) & 0xFF;


  payload[3] =
    hum_encoded & 0xFF;


  // -------------------------------------------------------------------
  // Byte 4-7
  // Pressione Pa
  // -------------------------------------------------------------------

  uint32_t pres_encoded =
    (uint32_t)(pres * 100);


  payload[4] =
    (pres_encoded >> 24) & 0xFF;


  payload[5] =
    (pres_encoded >> 16) & 0xFF;


  payload[6] =
    (pres_encoded >> 8) & 0xFF;


  payload[7] =
    pres_encoded & 0xFF;


  // -------------------------------------------------------------------
  // Byte 8-11
  // VOC resistance
  // -------------------------------------------------------------------

  payload[8] =
    (voc >> 24) & 0xFF;


  payload[9] =
    (voc >> 16) & 0xFF;


  payload[10] =
    (voc >> 8) & 0xFF;


  payload[11] =
    voc & 0xFF;


  // -------------------------------------------------------------------
  // Byte 12-13
  // NH3 ppm x10
  // -------------------------------------------------------------------

  uint16_t nh3_encoded =
    (uint16_t)(nh3 * 10);


  payload[12] =
    (nh3_encoded >> 8) & 0xFF;


  payload[13] =
    nh3_encoded & 0xFF;


  // -------------------------------------------------------------------
  // Byte 14-15
  // H2S ppm x10
  // -------------------------------------------------------------------

  uint16_t h2s_encoded =
    (uint16_t)(h2s * 10);


  payload[14] =
    (h2s_encoded >> 8) & 0xFF;


  payload[15] =
    h2s_encoded & 0xFF;


  // -------------------------------------------------------------------
  // Byte 16-17
  // Batteria mV
  // -------------------------------------------------------------------

  payload[16] =
    (bat_mv >> 8) & 0xFF;


  payload[17] =
    bat_mv & 0xFF;


  // -------------------------------------------------------------------
  // Byte 18
  // SOC
  // -------------------------------------------------------------------

  payload[18] =
    bat_soc;


  // -------------------------------------------------------------------
  // DEBUG
  // -------------------------------------------------------------------

  Serial.print(
    "[PAYLOAD] HEX: "
  );


  for (
    int i = 0;
    i < 19;
    i++
  ) {

    if (
      payload[i] < 0x10
    )
      Serial.print("0");


    Serial.print(
      payload[i],
      HEX
    );


    Serial.print(" ");
  }


  Serial.println();
}


// =====================================================================
// LETTURA BATTERIA
// =====================================================================

uint16_t readBatteryMV() {

  analogReadResolution(12);

  uint32_t sum = 0;

  for (int i = 0; i < 10; i++) {

    sum += analogRead(BATTERY_PIN);

    delay(5);
  }

  float raw = (float)sum / 10.0;

  // Divisore 1.5:1
  // ADC reference ~3.0V
  // 12 bit

  float voltage_mv =
    (raw / 4096.0) *
    3000.0 *
    1.5;

  return (uint16_t)voltage_mv;
}


// =====================================================================
// STIMA SOC
// =====================================================================

uint8_t estimateSOC(
  uint16_t mv
) {

  if (mv >= 4200)
    return 100;

  if (mv >= 4060)
    return 90;

  if (mv >= 3980)
    return 80;

  if (mv >= 3920)
    return 70;

  if (mv >= 3870)
    return 60;

  if (mv >= 3820)
    return 50;

  if (mv >= 3790)
    return 40;

  if (mv >= 3770)
    return 30;

  if (mv >= 3740)
    return 20;

  if (mv >= 3680)
    return 10;

  if (mv >= 3400)
    return 5;

  return 0;
}