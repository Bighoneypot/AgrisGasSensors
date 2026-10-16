
// =================================================================
//  sleep_diag.h — Diagnostica Sleep Persistente su Flash v1.1
// =================================================================
//  Salva contatori SLEEP_ENTER / SLEEP_EXIT su flash GD25Q16C.
//  Al boot, confronta i due contatori:
//    sleepEnter > sleepExit -> il codice NON e' tornato dallo sleep
//    sleepEnter == sleepExit -> il blocco e' DOPO lo sleep
//
//  Usa l'ultimo settore della GD25Q16C (0x1FF000 - 0x1FFFFF)
//  con buffer circolare per minimizzare gli erase.
//
//  Struttura record: 16 bytes
//    magic(4) + cycleCount(4) + sleepEnter(4) + sleepExit(4)
//
//  Buffer circolare: 4096 / 16 = 256 record per settore
//  Con 48 cicli/giorno -> erase ogni ~5.3 giorni
//  Endurance: 100K erase -> ~731 anni (nessun problema)
//
//  IMPORTANTE: Questo header DEVE essere incluso DOPO flash_log.h
//  perche' usa direttamente le funzioni static di flash_log.h:
//    flashReadData(), flashPageProgram(), flashSectorErase(),
//    flashReleasePowerDown(), flashPowerDown()
//
//  Target: GD25Q16C 2MB su RAK15001
// =================================================================
#ifndef SLEEP_DIAG_H
#define SLEEP_DIAG_H

#include <Arduino.h>

// --- Configurazione settore diagnostica ---
#define SDIAG_SECTOR_ADDR       0x1FF000UL  // Ultimo settore 4KB
#define SDIAG_SECTOR_SIZE       4096
#define SDIAG_MAGIC             0xDEAD5EE0UL
#define SDIAG_RECORD_SIZE       16
#define SDIAG_MAX_RECORDS       (SDIAG_SECTOR_SIZE / SDIAG_RECORD_SIZE)  // 256

// --- Struttura record (16 bytes, packed) ---
struct __attribute__((packed)) SleepDiagRecord {
    uint32_t magic;         // 0xDEAD5EE0 = valido
    uint32_t cycleCount;    // Cicli completati (persistente al reboot)
    uint32_t sleepEnter;    // Incrementato PRIMA di sleep
    uint32_t sleepExit;     // Incrementato DOPO ritorno da sleep
};

// --- Stato modulo ---
static SleepDiagRecord sdiag_current = {0, 0, 0, 0};
static uint32_t sdiag_writeIndex = 0;
static bool sdiag_initialized = false;
static bool sdiag_sleepMismatch = false;

// =================================================================
// WRITE RECORD — Scrive nel buffer circolare
// Usa flashPageProgram() e flashSectorErase() da flash_log.h
// =================================================================
static void sdiag_writeRecord(void) {
    if (!sdiag_initialized) return;

    if (sdiag_writeIndex >= SDIAG_MAX_RECORDS) {
        flashSectorErase(SDIAG_SECTOR_ADDR);
        sdiag_writeIndex = 0;
    }

    sdiag_current.magic = SDIAG_MAGIC;
    uint32_t addr = SDIAG_SECTOR_ADDR + (sdiag_writeIndex * SDIAG_RECORD_SIZE);

    flashReleasePowerDown();
    delay(5);
    flashPageProgram(addr, (const uint8_t *)&sdiag_current, SDIAG_RECORD_SIZE);
    flashPowerDown();

    sdiag_writeIndex++;
}

// =================================================================
// INIT — Legge l'ultimo record valido dal buffer circolare
// =================================================================
static void sdiag_init(void) {
    Serial.println("[SDIAG] Init diagnostica sleep persistente...");

    flashReleasePowerDown();
    delay(5);

    SleepDiagRecord rec;
    int32_t lastValid = -1;

    for (uint32_t i = 0; i < SDIAG_MAX_RECORDS; i++) {
        uint32_t addr = SDIAG_SECTOR_ADDR + (i * SDIAG_RECORD_SIZE);
        flashReadData(addr, (uint8_t *)&rec, SDIAG_RECORD_SIZE);

        if (rec.magic == SDIAG_MAGIC) {
            lastValid = (int32_t)i;
        } else {
            break;
        }
    }

    if (lastValid >= 0) {
        uint32_t addr = SDIAG_SECTOR_ADDR + (lastValid * SDIAG_RECORD_SIZE);
        flashReadData(addr, (uint8_t *)&sdiag_current, SDIAG_RECORD_SIZE);
        sdiag_writeIndex = (uint32_t)(lastValid + 1);

        Serial.print("[SDIAG] Ultimo record trovato @ slot ");
        Serial.println(lastValid);
        Serial.print("[SDIAG] cycleCount=");
        Serial.print(sdiag_current.cycleCount);
        Serial.print(" sleepEnter=");
        Serial.print(sdiag_current.sleepEnter);
        Serial.print(" sleepExit=");
        Serial.println(sdiag_current.sleepExit);

        if (sdiag_current.sleepEnter > sdiag_current.sleepExit) {
            sdiag_sleepMismatch = true;
            uint32_t missed = sdiag_current.sleepEnter - sdiag_current.sleepExit;
            Serial.println();
            Serial.println("!!! ================================================ !!!");
            Serial.print("!!! SLEEP NON TORNATO! Enter=");
            Serial.print(sdiag_current.sleepEnter);
            Serial.print(" Exit=");
            Serial.print(sdiag_current.sleepExit);
            Serial.print(" Mancati=");
            Serial.println(missed);
            Serial.println("!!! Il device si e' bloccato DENTRO sleep !!!");
            Serial.println("!!! ================================================ !!!");
            Serial.println();
        } else {
            sdiag_sleepMismatch = false;
            Serial.println("[SDIAG] Sleep OK -- Enter == Exit (nessun blocco rilevato)");
        }
    } else {
        Serial.println("[SDIAG] Nessun record precedente -- prima accensione");
        sdiag_current.magic = SDIAG_MAGIC;
        sdiag_current.cycleCount = 0;
        sdiag_current.sleepEnter = 0;
        sdiag_current.sleepExit = 0;
        sdiag_writeIndex = 0;
    }

    flashPowerDown();
    sdiag_initialized = true;
    Serial.println("[SDIAG] Init OK");
}

// =================================================================
// INCREMENT CYCLE
// =================================================================
static void sdiag_incrementCycle(void) {
    sdiag_current.cycleCount++;
    sdiag_writeRecord();
}

// =================================================================
// INCREMENT SLEEP ENTER
// =================================================================
static void sdiag_incrementSleepEnter(void) {
    sdiag_current.sleepEnter++;
    sdiag_writeRecord();
}

// =================================================================
// INCREMENT SLEEP EXIT
// =================================================================
static void sdiag_incrementSleepExit(void) {
    sdiag_current.sleepExit++;
    sdiag_writeRecord();
}

// =================================================================
// GETTERS
// =================================================================
static uint32_t sdiag_getCycleCount(void) {
    return sdiag_current.cycleCount;
}

static uint32_t sdiag_getSleepEnter(void) {
    return sdiag_current.sleepEnter;
}

static uint32_t sdiag_getSleepExit(void) {
    return sdiag_current.sleepExit;
}

static bool sdiag_hadSleepMismatch(void) {
    return sdiag_sleepMismatch;
}

// =================================================================
// CLEAR
// =================================================================
static void sdiag_clear(void) {
    Serial.println("[SDIAG] Cancellazione settore diagnostica...");
    flashReleasePowerDown();
    delay(5);
    flashSectorErase(SDIAG_SECTOR_ADDR);
    flashPowerDown();

    sdiag_current.magic = SDIAG_MAGIC;
    sdiag_current.cycleCount = 0;
    sdiag_current.sleepEnter = 0;
    sdiag_current.sleepExit = 0;
    sdiag_writeIndex = 0;

    Serial.println("[SDIAG] Reset completato");
}

// =================================================================
// PRINT STATUS
// =================================================================
static void sdiag_printStatus(void) {
    Serial.print("{\"sleep_diag\":{");
    Serial.print("\"cycle_count\":"); Serial.print(sdiag_current.cycleCount);
    Serial.print(",\"sleep_enter\":"); Serial.print(sdiag_current.sleepEnter);
    Serial.print(",\"sleep_exit\":"); Serial.print(sdiag_current.sleepExit);
    Serial.print(",\"mismatch\":"); Serial.print(sdiag_sleepMismatch ? "true" : "false");
    Serial.print(",\"flash_slot\":"); Serial.print(sdiag_writeIndex);
    Serial.print(",\"flash_slots_free\":"); Serial.print(SDIAG_MAX_RECORDS - sdiag_writeIndex);
    Serial.println("}}");
}

#endif // SLEEP_DIAG_H

