
// =====================================================================
// MYRUMINET - Sensore Gas Stalla
// FIRMWARE v6.5.5 - DIAGNOSTICA - TROVA IL BUG
// =====================================================================
// RAK3172-E / RAK19007 Rev.C
// RAK13009 QWIIC Module (3V3_S controllato via WB_IO2)
// BME680 + DFRobot NH3 + DFRobot H2S
//
// LoRaWAN OTAA - EU868
// Class A
// fPort 77
//
// RUI 4.2.4
//
// CHANGELOG v6.5.5 DIAGNOSTICA:
// - DISABILITATO reboot preventivo ogni 12 cicli (per test)
// - AGGIUNTO sistema LAST_PHASE persistente:
//   Salva in RAM la fase corrente del firmware.
//   Al boot, stampa l'ultima fase raggiunta prima del crash/reboot.
//   Fasi tracciate: BOOT, BATTERY_CHECK, JOIN_CHECK, SENSOR_POWER_ON,
//   PREHEAT, I2C_RECOVERY, BME680_INIT, NH3_INIT, H2S_INIT,
//   BME680_READ, NH3_READ, H2S_READ, PAYLOAD_BUILD, LORAWAN_SEND,
//   RADIO_WAIT, SLEEP_PREP, SLEEPING, WAKE
// - AGGIUNTO contatore cicli completati e esito ultimo TX
// - AGGIUNTO log ingresso/uscita per ogni operazione critica
// - AGGIUNTO gestione esplicita fallimento I2C recovery:
//   Se la recovery fallisce, salta init e lettura sensori I2C
// - MANTENUTO WDT fase attiva (120 sec)
// - MANTENUTO TX safety (3 fallimenti -> reboot)
// - MANTENUTO protezione batteria (3.0V)
// - RIMOSSO sleep watchdog (da verificare se funziona con sleep.all)
//
// OBIETTIVO: Al prossimo blocco, il boot stampa:
//   [BOOT] LAST_PHASE = H2S_READ (o qualsiasi fase)
//   [BOOT] LAST_CYCLE = 37
//   [BOOT] LAST_TX = OK
//   [BOOT] BAT = 3780 mV
//   Cosi' sappiamo ESATTAMENTE dove si blocca.
// =====================================================================


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include "DFRobot_MultiGasSensor.h"


// =====================================================================
// MODIFICA QUI PER DEBUG/PRODUZIONE
// =====================================================================

#define ENABLE_PREHEAT     true

// Produzione: 5    Debug: 1
#define PREHEAT_MINUTES    5

// Produzione: 25   Debug: 3
#define SLEEP_MINUTES      25

#define JOIN_RETRY_MINUTES 5


// =====================================================================
// WATCHDOG TIMER - SOFTWARE WDT (fase attiva)
// =====================================================================

#define WDT_TIMEOUT_MS     120000UL   // 120 secondi (2 min)
#define WDT_TIMER_ID       RAK_TIMER_4


// =====================================================================
// TX FAILURE SAFETY
// =====================================================================

#define MAX_TX_FAILURES    3


// =====================================================================
// REBOOT PREVENTIVO - DISABILITATO PER DIAGNOSTICA
// =====================================================================
// #define REBOOT_EVERY_N_CYCLES    12
// Commentato: vogliamo vedere dopo quanti cicli si blocca senza reboot


// =====================================================================
// I2C RECOVERY
// =====================================================================

#define I2C_RECOVERY_MAX_RETRIES   3
#define SENSOR_POWERUP_DELAY_MS    500


// =====================================================================
// PROTEZIONE BATTERIA
// =====================================================================

#define BATTERY_MIN_MV             3000
#define LOW_BATTERY_SLEEP_MS       (60UL * 60UL * 1000UL)


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
// SISTEMA DIAGNOSTICO - LAST_PHASE (v6.5.5)
// =====================================================================
// Ogni fase del firmware aggiorna questa variabile.
// Al boot, se il valore non e' BOOT, significa che il firmware
// si e' bloccato/resettato durante quella fase.
//
// Usiamo __attribute__((section(".noinit"))) per sopravvivere
// a un software reboot (api.system.reboot) ma NON a un power cycle.
// Su STM32WLE5 la sezione .noinit non viene azzerata al reboot.
// =====================================================================

// Fasi del firmware (ordine di esecuzione)
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
  PHASE_PAYLOAD_BUILD   = 13,
  PHASE_LORAWAN_SEND    = 14,
  PHASE_RADIO_WAIT      = 15,
  PHASE_SLEEP_PREP      = 16,
  PHASE_SLEEPING        = 17,
  PHASE_WAKE            = 18
};

// Nomi leggibili per il log
const char* phaseNames[] = {
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
  "PAYLOAD_BUILD",
  "LORAWAN_SEND",
  "RADIO_WAIT",
  "SLEEP_PREP",
  "SLEEPING",
  "WAKE"
};

// Struttura diagnostica che sopravvive al software reboot
// __attribute__((section(".noinit"))) evita l'azzeramento al reboot
struct DiagnosticState {
  uint32_t magic;            // 0xDEADBEEF = dati validi
  FirmwarePhase lastPhase;   // Ultima fase raggiunta
  uint32_t cycleCount;       // Contatore cicli totali
  uint32_t successfulTx;     // TX riusciti totali
  uint8_t lastTxResult;      // 0 = fail, 1 = ok
  uint16_t lastBatteryMv;    // Ultima lettura batteria
  uint8_t lastTxFailCount;   // Contatore TX falliti consecutivi
  uint8_t wdtTriggered;      // 1 = ultimo reboot causato da WDT
} __attribute__((section(".noinit")));

DiagnosticState diag __attribute__((section(".noinit")));

// Macro per aggiornare la fase corrente con log
#define SET_PHASE(p) do { \
  diag.lastPhase = (p); \
  Serial.print("[PHASE] -> "); \
  Serial.println(phaseNames[(p)]); \
} while(0)

// Macro per log ingresso operazione critica
#define LOG_ENTER(op) Serial.print("[" op "] Ingresso...")
#define LOG_EXIT_OK(op) Serial.println("[" op "] Uscita OK")
#define LOG_EXIT_FAIL(op) Serial.println("[" op "] Uscita FAIL")


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

#define I2C_SDA_PIN        PA11
#define I2C_SCL_PIN        PA12


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
volatile bool wdt_active = false;
bool i2c_bus_ok = false;   // v6.5.5: flag recovery I2C

uint8_t tx_fail_count = 0;


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

void wdtStart(void);
void wdtStop(void);
void wdtKick(void);
void wdtCallback(void *data);

bool i2cBusRecovery(void);
bool i2cBusRecoveryWithRetry(void);

void printBootDiagnostic(void);


// =====================================================================
// WATCHDOG TIMER
// =====================================================================

void wdtCallback(void *data) {
  (void)data;
  Serial.println();
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  Serial.println("  [WDT] TIMEOUT! Sistema bloccato!");
  Serial.print("  [WDT] FASE: ");
  Serial.println(phaseNames[diag.lastPhase]);
  Serial.print("  [WDT] CICLO: ");
  Serial.println(diag.cycleCount);

  // Marca il WDT come causa del reboot
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

void wdtStart(void) {
  api.system.timer.start(WDT_TIMER_ID, WDT_TIMEOUT_MS, NULL);
  wdt_active = true;
  Serial.print("[WDT] Avviato (timeout ");
  Serial.print(WDT_TIMEOUT_MS / 1000);
  Serial.println(" sec)");
}

void wdtStop(void) {
  if (wdt_active) {
    api.system.timer.stop(WDT_TIMER_ID);
    wdt_active = false;
    Serial.println("[WDT] Fermato");
  }
}

void wdtKick(void) {
  if (wdt_active) {
    api.system.timer.stop(WDT_TIMER_ID);
  }
  api.system.timer.start(WDT_TIMER_ID, WDT_TIMEOUT_MS, NULL);
  wdt_active = true;
}


// =====================================================================
// I2C BUS RECOVERY
// =====================================================================

bool i2cBusRecovery(void) {
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

bool i2cBusRecoveryWithRetry(void) {
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
// BOOT DIAGNOSTIC - v6.5.5
// =====================================================================

void printBootDiagnostic(void) {
  Serial.println();
  Serial.println("=============================================");
  Serial.println("  DIAGNOSTICA BOOT v6.5.5");
  Serial.println("=============================================");

  if (diag.magic == 0xDEADBEEF) {
    // Dati validi da sessione precedente
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

  // Inizializza/resetta la struttura per questa sessione
  diag.magic = 0xDEADBEEF;
  diag.lastPhase = PHASE_BOOT;
  diag.cycleCount = 0;
  diag.successfulTx = 0;
  diag.lastTxResult = 0;
  diag.lastBatteryMv = 0;
  diag.lastTxFailCount = 0;
  diag.wdtTriggered = 0;
}


// =====================================================================
// SETUP
// =====================================================================

void setup() {

  Serial.begin(115200);
  api.system.lpm.set(1);
  delay(3000);

  Serial.println();
  Serial.println("=============================================");
  Serial.println("  MYRUMINET - SENSORE GAS STALLA v6.5.5");
  Serial.println("  *** VERSIONE DIAGNOSTICA ***");
  Serial.println("  RAK3172-E / RAK19007 Rev.C / RAK13009");
  Serial.println("  BME680 + NH3 + H2S");
  Serial.println("  LoRaWAN OTAA - EU868 - fPort 77");
  Serial.println("  RUI 4.2.4");
  Serial.println("  WDT ATTIVO | TX SAFETY ATTIVO");
  Serial.println("  REBOOT PREVENTIVO: DISABILITATO (test)");
  #if ENABLE_PREHEAT
    Serial.println("  MODALITA: PREHEAT ATTIVO");
    Serial.print("  PREHEAT: ");
    Serial.print(PREHEAT_MINUTES);
    Serial.println(" min");
  #else
    Serial.println("  MODALITA: SENSORI SEMPRE ON");
  #endif
  Serial.print("  SLEEP:   ");
  Serial.print(SLEEP_MINUTES);
  Serial.println(" min");
  Serial.print("  CICLO:   ");
  Serial.print(CYCLE_TOTAL_MIN);
  Serial.println(" min totale");
  Serial.print("  WDT:     ");
  Serial.print(WDT_TIMEOUT_MS / 1000);
  Serial.println(" sec timeout");
  Serial.print("  TX FAIL MAX: ");
  Serial.println(MAX_TX_FAILURES);
  Serial.print("  BAT MIN: ");
  Serial.print(BATTERY_MIN_MV);
  Serial.println(" mV");
  Serial.println("=============================================");
  Serial.println();

  // ---- DIAGNOSTICA BOOT ----
  printBootDiagnostic();

  // ---- Creazione timer WDT (one-shot) ----
  if (!api.system.timer.create(WDT_TIMER_ID, wdtCallback, RAK_TIMER_ONESHOT)) {
    Serial.println("[WDT] ERRORE creazione timer!");
  } else {
    Serial.println("[WDT] Timer creato (one-shot, pronto)");
  }
  Serial.println();

  #if ENABLE_PREHEAT
    pinMode(SENSOR_POWER_PIN, OUTPUT);
    digitalWrite(SENSOR_POWER_PIN, LOW);
    Serial.println("[PWR] 3V3_S SPENTO (sensori OFF fino al primo ciclo)");
    Serial.println();
  #endif

  printDeviceInfo();
  initLoRaWAN();
  joinNetwork();

  if (!joined) {
    Serial.println("[SETUP] Join fallito. Sleep e retry nel loop.");
    SET_PHASE(PHASE_SLEEPING);
    Serial.flush();
    delay(10);
    api.system.sleep.all(JOIN_RETRY_INTERVAL);
    return;
  }

  Serial.println();
  Serial.println("=============================================");
  Serial.println("  SETUP COMPLETATO - v6.5.5 DIAGNOSTICA");
  Serial.println("  Join OK. Primo ciclo sensori nel loop.");
  Serial.println("=============================================");
  Serial.println();
}


// =====================================================================
// LOOP
// =====================================================================

void loop() {

  // --- WAKE ---
  SET_PHASE(PHASE_WAKE);

  // Incrementa contatore cicli
  diag.cycleCount++;
  Serial.println();
  Serial.println("=============================================");
  Serial.print("[CICLO] #");
  Serial.print(diag.cycleCount);
  Serial.print(" | TX OK totali: ");
  Serial.print(diag.successfulTx);
  Serial.print(" | TX fail consec: ");
  Serial.print(tx_fail_count);
  Serial.print("/");
  Serial.println(MAX_TX_FAILURES);
  Serial.println("=============================================");
  Serial.println();

  // ===== CHECK BATTERIA =====
  SET_PHASE(PHASE_BATTERY_CHECK);
  uint16_t bat_check = readBatteryMV();
  diag.lastBatteryMv = bat_check;
  Serial.print("[BAT] Check: ");
  Serial.print(bat_check);
  Serial.println(" mV");

  if (bat_check < BATTERY_MIN_MV) {
    Serial.print("[BAT] !!! Tensione critica: ");
    Serial.print(bat_check);
    Serial.println(" mV");
    Serial.println("[BAT] Sleep lungo 60 min...");

    #if ENABLE_PREHEAT
      digitalWrite(SENSOR_POWER_PIN, LOW);
    #endif

    SET_PHASE(PHASE_SLEEPING);
    Serial.flush();
    delay(10);
    api.system.sleep.all(LOW_BATTERY_SLEEP_MS);
    return;
  }

  // ===== AVVIO WDT =====
  wdtStart();

  // ===== VERIFICA JOIN =====
  SET_PHASE(PHASE_JOIN_CHECK);
  Serial.println("[JOIN] Verifica sessione...");

  if (!api.lorawan.njs.get()) {
    Serial.println("[JOIN] Sessione NON presente. Rejoin...");
    joined = false;
    wdtStop();
    joinNetwork();

    if (!joined) {
      Serial.println("[JOIN] Fallito. Sleep e riprovo.");
      SET_PHASE(PHASE_SLEEPING);
      Serial.flush();
      delay(10);
      api.system.sleep.all(JOIN_RETRY_INTERVAL);
      return;
    }
    wdtStart();
  } else {
    joined = true;
    Serial.println("[JOIN] Sessione OK");
  }

  // ===== SENSORI: POWER ON + PREHEAT =====
  #if ENABLE_PREHEAT
    SET_PHASE(PHASE_SENSOR_POWER_ON);
    LOG_ENTER("PWR-ON");
    sensorPowerOn();
    Serial.print(" stabilizzazione ");
    Serial.print(SENSOR_POWERUP_DELAY_MS);
    Serial.println(" ms...");
    delay(SENSOR_POWERUP_DELAY_MS);
    LOG_EXIT_OK("PWR-ON");

    SET_PHASE(PHASE_PREHEAT);
    Serial.print("[PREHEAT] Inizio (");
    Serial.print(PREHEAT_MINUTES);
    Serial.println(" min)...");
    preheatWait();
    Serial.println("[PREHEAT] Completato");
    Serial.println();
    wdtKick();
  #endif

  // ===== I2C RECOVERY =====
  SET_PHASE(PHASE_I2C_RECOVERY);
  LOG_ENTER("I2C-REC");
  Serial.println();
  i2c_bus_ok = i2cBusRecoveryWithRetry();

  if (!i2c_bus_ok) {
    Serial.println("[I2C] !!! BUS NON RECUPERATO !!!");
    Serial.println("[I2C] Salto init e lettura sensori I2C.");
    Serial.println("[I2C] Invio payload con valori zero.");
    LOG_EXIT_FAIL("I2C-REC");
  } else {
    LOG_EXIT_OK("I2C-REC");
    Wire.begin();
    delay(100);

    // ===== INIT SENSORI (solo se I2C OK) =====
    doInitSensors();
  }

  // ===== VARIABILI =====
  float temperature = 0.0;
  float humidity = 0.0;
  float pressure = 0.0;
  uint32_t voc_resistance = 0;
  float nh3_ppm = 0.0;
  float h2s_ppm = 0.0;
  uint16_t battery_mv = 0;
  uint8_t battery_soc = 0;

  // ===== LETTURE (solo se I2C OK) =====
  if (i2c_bus_ok) {

    Serial.println("--- LETTURA SENSORI ---");

    // ----- BME680 -----
    if (bme680_ok) {
      SET_PHASE(PHASE_BME680_READ);
      LOG_ENTER("BME680-READ");
      Serial.println();
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
        LOG_EXIT_OK("BME680-READ");
      } else {
        Serial.println("[BME680] Errore lettura!");
        LOG_EXIT_FAIL("BME680-READ");
      }
    }

    // ----- NH3 -----
    if (nh3_ok) {
      SET_PHASE(PHASE_NH3_READ);
      LOG_ENTER("NH3-READ");
      Serial.println();
      nh3_ppm = gasSensorNH3.readGasConcentrationPPM();
      Serial.print("[NH3] Conc: ");
      Serial.print(nh3_ppm, 2);
      Serial.println(" ppm");
      LOG_EXIT_OK("NH3-READ");
    }

    // ----- H2S -----
    if (h2s_ok) {
      SET_PHASE(PHASE_H2S_READ);
      LOG_ENTER("H2S-READ");
      Serial.println();
      h2s_ppm = gasSensorH2S.readGasConcentrationPPM();
      Serial.print("[H2S] Conc: ");
      Serial.print(h2s_ppm, 2);
      Serial.println(" ppm");
      LOG_EXIT_OK("H2S-READ");
    }

  } else {
    Serial.println("[SENSORI] Saltati (I2C bus non disponibile)");
  }

  // ===== BATTERIA =====
  SET_PHASE(PHASE_BATTERY_READ);
  battery_mv = readBatteryMV();
  battery_soc = estimateSOC(battery_mv);
  diag.lastBatteryMv = battery_mv;
  Serial.print("[BAT] Volt: ");
  Serial.print(battery_mv);
  Serial.println(" mV");
  Serial.print("[BAT] SOC:  ");
  Serial.print(battery_soc);
  Serial.println(" %");
  Serial.println();

  // ===== PAYLOAD =====
  SET_PHASE(PHASE_PAYLOAD_BUILD);
  buildPayload(temperature, humidity, pressure, voc_resistance, nh3_ppm, h2s_ppm, battery_mv, battery_soc);

  // ===== LORAWAN TX =====
  SET_PHASE(PHASE_LORAWAN_SEND);
  Serial.print("[LORA] Invio payload (");
  Serial.print(sizeof(payload));
  Serial.print(" bytes) su fPort ");
  Serial.print(LORAWAN_FPORT);
  Serial.println("...");

  bool tx_ok = api.lorawan.send(sizeof(payload), payload, LORAWAN_FPORT, false);

  if (tx_ok) {
    Serial.println("[LORA] Invio riuscito!");
    tx_fail_count = 0;
    diag.successfulTx++;
    diag.lastTxResult = 1;
    diag.lastTxFailCount = 0;
  } else {
    Serial.println("[LORA] Invio fallito!");
    tx_fail_count++;
    diag.lastTxResult = 0;
    diag.lastTxFailCount = tx_fail_count;

    Serial.print("[TX-SAFETY] Fallimenti consecutivi: ");
    Serial.print(tx_fail_count);
    Serial.print("/");
    Serial.println(MAX_TX_FAILURES);

    if (tx_fail_count >= MAX_TX_FAILURES) {
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      Serial.print("  [TX-SAFETY] ");
      Serial.print(MAX_TX_FAILURES);
      Serial.println(" TX falliti consecutivi!");
      Serial.println("  [TX-SAFETY] Reboot forzato...");
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

      #if ENABLE_PREHEAT
        digitalWrite(SENSOR_POWER_PIN, LOW);
      #endif

      Serial.flush();
      delay(100);
      api.system.reboot();
    }

    if (!api.lorawan.njs.get()) {
      Serial.println("[LORA] Sessione persa.");
      joined = false;
    }
  }

  // ===== ATTESA RADIO =====
  SET_PHASE(PHASE_RADIO_WAIT);
  delay(6000);

  // ===== SLEEP =====
  SET_PHASE(PHASE_SLEEP_PREP);
  wdtStop();

  Serial.println();
  Serial.println("--- RIEPILOGO CICLO ---");
  Serial.print("  Ciclo:    #");
  Serial.println(diag.cycleCount);
  Serial.print("  TX:       ");
  Serial.println(tx_ok ? "OK" : "FAIL");
  Serial.print("  TX OK tot: ");
  Serial.println(diag.successfulTx);
  Serial.print("  Batteria: ");
  Serial.print(battery_mv);
  Serial.println(" mV");
  Serial.print("  I2C bus:  ");
  Serial.println(i2c_bus_ok ? "OK" : "FAIL");
  Serial.println("------------------------");
  Serial.println();

  Serial.print("[SLEEP] Deep sleep ");
  Serial.print(SLEEP_MINUTES);
  Serial.println(" min...");

  Wire.end();

  #if ENABLE_PREHEAT
    sensorPowerOff();
  #endif

  SET_PHASE(PHASE_SLEEPING);

  Serial.flush();
  delay(10);

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
    wdtKick();
    delay(PREHEAT_BLOCK);

    Serial.print("[PREHEAT] ");
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
  SET_PHASE(PHASE_BME680_INIT);
  LOG_ENTER("BME680-INIT");
  Serial.println();
  if (bme.begin(BME680_I2C_ADDR, &Wire)) {
    bme680_ok = true;
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);
    LOG_EXIT_OK("BME680-INIT");
  } else {
    LOG_EXIT_FAIL("BME680-INIT");
  }

  // NH3
  SET_PHASE(PHASE_NH3_INIT);
  delay(1000);
  LOG_ENTER("NH3-INIT");
  Serial.println();
  if (gasSensorNH3.begin()) {
    nh3_ok = true;
    Serial.print("[NH3] Gas type: ");
    Serial.println(gasSensorNH3.queryGasType());
    gasSensorNH3.changeAcquireMode(gasSensorNH3.PASSIVITY);
    LOG_EXIT_OK("NH3-INIT");
  } else {
    LOG_EXIT_FAIL("NH3-INIT");
  }

  // H2S
  SET_PHASE(PHASE_H2S_INIT);
  delay(1000);
  LOG_ENTER("H2S-INIT");
  Serial.println();
  if (gasSensorH2S.begin()) {
    h2s_ok = true;
    Serial.print("[H2S] Gas type: ");
    Serial.println(gasSensorH2S.queryGasType());
    gasSensorH2S.changeAcquireMode(gasSensorH2S.PASSIVITY);
    LOG_EXIT_OK("H2S-INIT");
  } else {
    LOG_EXIT_FAIL("H2S-INIT");
  }

  Serial.println();
}


void initLoRaWAN(void) {
  Serial.println("--- CONFIGURAZIONE LORAWAN ---");

  bool ok;

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

