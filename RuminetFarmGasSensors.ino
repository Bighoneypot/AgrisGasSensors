
// =====================================================================
// MYRUMINET - Sensore Gas Stalla
// FIRMWARE v6.3 FINAL - SENSORI SEMPRE ON + PREHEAT CONDIZIONALE
// =====================================================================
// RAK3172-E / RAK19007 Rev.C
// RAK1920 Sensor Adapter (3V3 sempre alimentato - bug schematico)
// BME680 + DFRobot NH3 + DFRobot H2S
//
// LoRaWAN OTAA - EU868
// Class A
// fPort 77
//
// RUI 4.2.4
//
// NOTE:
// - La RAK1920 ha 3V3 e 3V3_S unite: sensori sempre alimentati
// - ENABLE_PREHEAT = false -> sensori sempre caldi, no preheat
// - ENABLE_PREHEAT = true  -> quando si risolve il power-off (RAK13009/MOSFET)
// - Divisore batteria RAK19007 Rev.C: 2.265
// - Con pannello solare 6W + batteria 10Ah = autonomia infinita
// =====================================================================


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include "DFRobot_MultiGasSensor.h"


// =====================================================================
// CONFIGURAZIONE - MODIFICA QUI PER DEBUG/PRODUZIONE
// =====================================================================

// Abilitare il preriscaldamento sensori?
// false: sensori sempre alimentati (RAK1920 bug, no power-off)
// true:  sensori vengono spenti e riaccesi (RAK13009 o MOSFET esterno)
#define ENABLE_PREHEAT     false

// Preriscaldamento sensori gas in MINUTI (usato solo se ENABLE_PREHEAT = true)
// Produzione: 5    Debug: 1
#define PREHEAT_MINUTES    5

// Sleep in MINUTI tra un invio e l'altro
// Produzione: 25   Debug: 3
#define SLEEP_MINUTES      30

// Retry join in MINUTI
#define JOIN_RETRY_MINUTES 5


// --- FORMULE AUTOMATICHE (NON MODIFICARE) ---
#define SLEEP_INTERVAL       ((unsigned long)SLEEP_MINUTES * 60UL * 1000UL)
#define JOIN_RETRY_INTERVAL  ((unsigned long)JOIN_RETRY_MINUTES * 60UL * 1000UL)

#if ENABLE_PREHEAT
  #define PREHEAT_INTERVAL   ((unsigned long)PREHEAT_MINUTES * 60UL * 1000UL)
  #define PREHEAT_BLOCK      30000UL
  #define PREHEAT_BLOCKS     ((PREHEAT_INTERVAL) / (PREHEAT_BLOCK))
  #define PREHEAT_SECONDS    (PREHEAT_MINUTES * 60)
  #define CYCLE_TOTAL_MIN    (SLEEP_MINUTES + PREHEAT_MINUTES)
#else
  #define CYCLE_TOTAL_MIN    SLEEP_MINUTES
#endif


// =====================================================================
// CONFIGURAZIONE LORAWAN
// =====================================================================

#define LORAWAN_BAND       RAK_REGION_EU868
#define LORAWAN_CLASS      CLASS_A
#define LORAWAN_FPORT      77
#define LORAWAN_DR_JOIN    0
#define LORAWAN_DR_TX      5
#define LORAWAN_RX1DL      5000
#define LORAWAN_RX2DL      6000


// =====================================================================
// CREDENZIALI OTAA - Agris_Lifely_GAS_Sensors_01
// =====================================================================

uint8_t node_dev_eui[8] = {0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x0A, 0x70, 0xF4};
uint8_t node_app_eui[8] = {0xAC, 0x1F, 0x09, 0xFF, 0xF8, 0x68, 0x31, 0x72};
uint8_t node_app_key[16] = {0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x0A, 0x70, 0xF4,
                             0xAC, 0x1F, 0x09, 0xFF, 0xF8, 0x68, 0x31, 0x72};


// =====================================================================
// CONFIGURAZIONE HARDWARE
// =====================================================================

#if ENABLE_PREHEAT
  #define SENSOR_POWER_PIN   WB_IO2
#endif

#define NH3_I2C_ADDR       0x77
#define H2S_I2C_ADDR       0x74
#define BME680_I2C_ADDR    0x76
#define BATTERY_PIN        WB_A0
#define BATTERY_DIVIDER    2.265
#define JOIN_POLL_INTERVAL 5000
#define JOIN_MAX_POLLS     30


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
// FORWARD DECLARATIONS
// =====================================================================

#if ENABLE_PREHEAT
  void preheatWait(void);
  void sensorPowerOn(void);
  void sensorPowerOff(void);
#endif
void printDeviceInfo(void);
void doInitSensors(void);
void initLoRaWAN(void);
void joinNetwork(void);
void buildPayload(float temp, float hum, float pres, uint32_t voc, float nh3, float h2s, uint16_t bat_mv, uint8_t bat_soc);
uint16_t readBatteryMV(void);
uint8_t estimateSOC(uint16_t mv);


// =====================================================================
// SETUP
// =====================================================================

void setup() {

  Serial.begin(115200);
  api.system.lpm.set(1);
  delay(3000);

  Serial.println();
  Serial.println("=============================================");
  Serial.println("  MYRUMINET - SENSORE GAS STALLA v6.3");
  Serial.println("  RAK3172-E / RAK19007 Rev.C / RAK1920");
  Serial.println("  BME680 + NH3 + H2S");
  Serial.println("  LoRaWAN OTAA - EU868 - fPort 77");
  Serial.println("  RUI 4.2.4");
  #if ENABLE_PREHEAT
    Serial.println("  MODALITA: PREHEAT ATTIVO (power-off sensori)");
    Serial.print("  PREHEAT: ");
    Serial.print(PREHEAT_MINUTES);
    Serial.println(" min");
  #else
    Serial.println("  MODALITA: SENSORI SEMPRE ON (no power-off)");
  #endif
  Serial.print("  SLEEP:   ");
  Serial.print(SLEEP_MINUTES);
  Serial.println(" min");
  Serial.print("  CICLO:   ");
  Serial.print(CYCLE_TOTAL_MIN);
  Serial.println(" min totale");
  Serial.print("  JOIN RETRY: ");
  Serial.print(JOIN_RETRY_MINUTES);
  Serial.println(" min");
  Serial.println("=============================================");
  Serial.println();

  #if ENABLE_PREHEAT
    // Sensori SPENTI fino a join riuscito
    pinMode(SENSOR_POWER_PIN, OUTPUT);
    digitalWrite(SENSOR_POWER_PIN, LOW);
    Serial.println("[PWR] 3V3_S SPENTO (sensori OFF fino a join)");
    Serial.println();
  #endif

  // Info dispositivo
  printDeviceInfo();

  // Configurazione LoRaWAN
  initLoRaWAN();

  // Join
  joinNetwork();

  // Se join fallito: sleep e lascia che il loop riprovi
  if (!joined) {
    Serial.println("[SETUP] Join fallito. Sleep e retry nel loop.");
    Serial.flush();
    delay(10);
    api.system.sleep.all(JOIN_RETRY_INTERVAL);
    return;
  }

  // Join riuscito
  #if ENABLE_PREHEAT
    sensorPowerOn();
  #endif

  // I2C
  Wire.begin();
  delay(100);
  Serial.println("[I2C] Bus inizializzato");
  Serial.println();

  #if ENABLE_PREHEAT
    Serial.print("[PWR] Preriscaldo iniziale sensori (");
    Serial.print(PREHEAT_MINUTES);
    Serial.println(" min)...");
    preheatWait();
    Serial.println("[PWR] Preriscaldo completato");
    Serial.println();
  #endif

  // Init sensori
  doInitSensors();

  Serial.println();
  Serial.println("=============================================");
  Serial.println("  SETUP COMPLETATO - v6.3");
  Serial.println("=============================================");
  Serial.println();
}


// =====================================================================
// LOOP
// =====================================================================

void loop() {

  // ----- VERIFICA JOIN -----
  if (!api.lorawan.njs.get()) {
    Serial.println("[LORA] Sessione LoRaWAN non presente.");
    joined = false;
    joinNetwork();

    if (!joined) {
      Serial.print("[LORA] Join fallito. Sleep ");
      Serial.print(JOIN_RETRY_MINUTES);
      Serial.println(" min e riprovo.");
      Serial.flush();
      delay(10);
      api.system.sleep.all(JOIN_RETRY_INTERVAL);
      return;
    }
  } else {
    joined = true;
  }

  // ----- SENSORI: ACCENSIONE + PREHEAT (solo se abilitato) -----
  #if ENABLE_PREHEAT
    sensorPowerOn();
    Serial.print("[PWR] Preriscaldo sensori (");
    Serial.print(PREHEAT_MINUTES);
    Serial.println(" min)...");
    preheatWait();
    Serial.println("[PWR] Preriscaldo completato");
    Serial.println();
  #endif

  // ----- I2C + INIT SENSORI -----
  Wire.begin();
  delay(100);
  doInitSensors();

  // ----- VARIABILI -----
  float temperature = 0.0;
  float humidity = 0.0;
  float pressure = 0.0;
  uint32_t voc_resistance = 0;
  float nh3_ppm = 0.0;
  float h2s_ppm = 0.0;
  uint16_t battery_mv = 0;
  uint8_t battery_soc = 0;

  // ----- BME680 -----
  if (bme680_ok) {
    Serial.println("--- LETTURA SENSORI ---");
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
    } else {
      Serial.println("[BME680] Errore lettura!");
    }
  }

  // ----- NH3 -----
  if (nh3_ok) {
    nh3_ppm = gasSensorNH3.readGasConcentrationPPM();
    Serial.print("[NH3]   Conc: ");
    Serial.print(nh3_ppm, 2);
    Serial.println(" ppm");
  }

  // ----- H2S -----
  if (h2s_ok) {
    h2s_ppm = gasSensorH2S.readGasConcentrationPPM();
    Serial.print("[H2S]   Conc: ");
    Serial.print(h2s_ppm, 2);
    Serial.println(" ppm");
  }

  // ----- BATTERIA -----
  battery_mv = readBatteryMV();
  battery_soc = estimateSOC(battery_mv);
  Serial.print("[BAT]   Volt: ");
  Serial.print(battery_mv);
  Serial.println(" mV");
  Serial.print("[BAT]   SOC:  ");
  Serial.print(battery_soc);
  Serial.println(" %");
  Serial.println();

  // ----- PAYLOAD -----
  buildPayload(temperature, humidity, pressure, voc_resistance, nh3_ppm, h2s_ppm, battery_mv, battery_soc);

  // ----- LORAWAN TX -----
  Serial.print("[LORA] Invio payload (");
  Serial.print(sizeof(payload));
  Serial.print(" bytes) su fPort ");
  Serial.print(LORAWAN_FPORT);
  Serial.println("...");

  bool tx_ok = api.lorawan.send(sizeof(payload), payload, LORAWAN_FPORT, false);

  if (tx_ok) {
    Serial.println("[LORA] Invio riuscito!");
  } else {
    Serial.println("[LORA] Invio fallito!");
    if (!api.lorawan.njs.get()) {
      Serial.println("[LORA] Sessione persa.");
      joined = false;
    }
  }

  // ----- ATTESA RADIO -----
  delay(6000);

  // =================================================================
  // SLEEP
  // =================================================================

  Serial.println();
  Serial.print("[SLEEP] Deep sleep ");
  Serial.print(SLEEP_MINUTES);
  Serial.println(" min...");

  // Rilascia I2C
  Wire.end();

  #if ENABLE_PREHEAT
    // Spegni sensori (solo se abbiamo power control)
    sensorPowerOff();
  #endif

  // Flush seriale
  Serial.flush();
  delay(10);

  // Deep sleep MCU
  api.system.sleep.all(SLEEP_INTERVAL);

  // --- RISVEGLIO ---
  Serial.println("[WAKE] RAK3172 risvegliato");
  Serial.println();
}


// =====================================================================
// IMPLEMENTAZIONE FUNZIONI
// =====================================================================


#if ENABLE_PREHEAT

void preheatWait(void) {
  for (unsigned int i = 1; i <= PREHEAT_BLOCKS; i++) {
    delay(PREHEAT_BLOCK);
    Serial.print("[PWR] Preheat: ");
    Serial.print(i * 30);
    Serial.print(" sec / ");
    Serial.print(PREHEAT_SECONDS);
    Serial.println(" sec");
  }
}

void sensorPowerOn(void) {
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  Serial.println("[PWR] 3V3_S ACCESO (WB_IO2 HIGH)");
}

void sensorPowerOff(void) {
  nh3_ok = false;
  h2s_ok = false;
  bme680_ok = false;
  digitalWrite(SENSOR_POWER_PIN, LOW);
  Serial.println("[PWR] 3V3_S SPENTO (WB_IO2 LOW)");
}

#endif


void printDeviceInfo(void) {
  Serial.println("--- INFO DISPOSITIVO ---");
  Serial.print("  Firmware RUI3: ");
  Serial.println(api.system.firmwareVersion.get().c_str());
  Serial.print("  Modello:        ");
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


void doInitSensors(void) {
  Serial.println("--- INIZIALIZZAZIONE SENSORI ---");

  // BME680
  Serial.print("[BME680] Init (0x76)... ");
  if (bme.begin(BME680_I2C_ADDR, &Wire)) {
    bme680_ok = true;
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);
    Serial.println("OK");
  } else {
    Serial.println("FALLITO");
  }

  // NH3
  delay(1000);
  Serial.print("[NH3]   Init (0x77)... ");
  if (gasSensorNH3.begin()) {
    nh3_ok = true;
    Serial.print("OK - Gas type: ");
    Serial.println(gasSensorNH3.queryGasType());
    gasSensorNH3.changeAcquireMode(gasSensorNH3.PASSIVITY);
  } else {
    Serial.println("FALLITO");
  }

  // H2S
  delay(1000);
  Serial.print("[H2S]   Init (0x74)... ");
  if (gasSensorH2S.begin()) {
    h2s_ok = true;
    Serial.print("OK - Gas type: ");
    Serial.println(gasSensorH2S.queryGasType());
    gasSensorH2S.changeAcquireMode(gasSensorH2S.PASSIVITY);
  } else {
    Serial.println("FALLITO");
  }

  Serial.println();
}


void initLoRaWAN(void) {
  Serial.println("--- CONFIGURAZIONE LORAWAN ---");

  bool ok;

  // Credenziali
  ok = api.lorawan.deui.set(node_dev_eui, 8);
  Serial.print("[LORA] Set DevEUI: ");
  Serial.println(ok ? "OK" : "FAIL");

  ok = api.lorawan.appeui.set(node_app_eui, 8);
  Serial.print("[LORA] Set AppEUI: ");
  Serial.println(ok ? "OK" : "FAIL");

  ok = api.lorawan.appkey.set(node_app_key, 16);
  Serial.print("[LORA] Set AppKey: ");
  Serial.println(ok ? "OK" : "FAIL");

  Serial.println();

  // Parametri
  ok = api.lorawan.band.set(LORAWAN_BAND);
  Serial.print("[LORA] Banda EU868: ");
  Serial.println(ok ? "OK" : "FAIL");

  ok = api.lorawan.deviceClass.set(LORAWAN_CLASS);
  Serial.print("[LORA] Classe A: ");
  Serial.println(ok ? "OK" : "FAIL");

  ok = api.lorawan.njm.set(RAK_LORA_OTAA);
  Serial.print("[LORA] OTAA: ");
  Serial.println(ok ? "OK" : "FAIL");

  ok = api.lorawan.adr.set(true);
  Serial.print("[LORA] ADR ON: ");
  Serial.println(ok ? "OK" : "FAIL");

  ok = api.lorawan.dr.set(LORAWAN_DR_JOIN);
  Serial.print("[LORA] DR0 iniziale: ");
  Serial.println(ok ? "OK" : "FAIL");

  ok = api.lorawan.rx2dr.set(0);
  Serial.print("[LORA] RX2 DR0: ");
  Serial.println(ok ? "OK" : "FAIL");

  ok = api.lorawan.rx1dl.set(LORAWAN_RX1DL);
  Serial.print("[LORA] RX1 DL 5000ms: ");
  Serial.println(ok ? "OK" : "FAIL");

  ok = api.lorawan.rx2dl.set(LORAWAN_RX2DL);
  Serial.print("[LORA] RX2 DL 6000ms: ");
  Serial.println(ok ? "OK" : "FAIL");

  Serial.println();

  // Stato
  Serial.println("--- STATO LORAWAN ---");
  Serial.print("  NWM:   ");
  Serial.println(api.lorawan.nwm.get());
  Serial.print("  NJM:   ");
  Serial.println(api.lorawan.njm.get());
  Serial.print("  BAND:  ");
  Serial.println(api.lorawan.band.get());
  Serial.print("  DR:    ");
  Serial.println(api.lorawan.dr.get());
  Serial.print("  ADR:   ");
  Serial.println(api.lorawan.adr.get());
  Serial.print("  TXP:   ");
  Serial.println(api.lorawan.txp.get());
  Serial.print("  RX1DL: ");
  Serial.println(api.lorawan.rx1dl.get());
  Serial.print("  RX2DL: ");
  Serial.println(api.lorawan.rx2dl.get());
  Serial.print("  RX2DR: ");
  Serial.println(api.lorawan.rx2dr.get());
  Serial.println();
}


void joinNetwork(void) {
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
      return;
    }

    polls++;
    Serial.print("[LORA] Attendo join... ");
    Serial.print(polls * 5);
    Serial.println("s");
  }

  Serial.println("[LORA] JOIN FALLITO dopo 150 secondi");
  joined = false;
}


void buildPayload(float temp, float hum, float pres, uint32_t voc, float nh3, float h2s, uint16_t bat_mv, uint8_t bat_soc) {

  int16_t temp_encoded = (int16_t)(temp * 100);
  payload[0] = (temp_encoded >> 8) & 0xFF;
  payload[1] = temp_encoded & 0xFF;

  uint16_t hum_encoded = (uint16_t)(hum * 100);
  payload[2] = (hum_encoded >> 8) & 0xFF;
  payload[3] = hum_encoded & 0xFF;

  uint32_t pres_encoded = (uint32_t)(pres * 100);
  payload[4] = (pres_encoded >> 24) & 0xFF;
  payload[5] = (pres_encoded >> 16) & 0xFF;
  payload[6] = (pres_encoded >> 8) & 0xFF;
  payload[7] = pres_encoded & 0xFF;

  payload[8] = (voc >> 24) & 0xFF;
  payload[9] = (voc >> 16) & 0xFF;
  payload[10] = (voc >> 8) & 0xFF;
  payload[11] = voc & 0xFF;

  uint16_t nh3_encoded = (uint16_t)(nh3 * 10);
  payload[12] = (nh3_encoded >> 8) & 0xFF;
  payload[13] = nh3_encoded & 0xFF;

  uint16_t h2s_encoded = (uint16_t)(h2s * 10);
  payload[14] = (h2s_encoded >> 8) & 0xFF;
  payload[15] = h2s_encoded & 0xFF;

  payload[16] = (bat_mv >> 8) & 0xFF;
  payload[17] = bat_mv & 0xFF;

  payload[18] = bat_soc;

  Serial.print("[PAYLOAD] HEX: ");
  for (int i = 0; i < 19; i++) {
    if (payload[i] < 0x10) Serial.print("0");
    Serial.print(payload[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}


uint16_t readBatteryMV(void) {
  analogReadResolution(12);
  uint32_t sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(BATTERY_PIN);
    delay(5);
  }
  float raw = (float)sum / 10.0;
  float voltage_mv = (raw / 4096.0) * 3000.0 * BATTERY_DIVIDER;
  return (uint16_t)voltage_mv;
}


uint8_t estimateSOC(uint16_t mv) {
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

