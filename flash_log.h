
// =====================================================================
// flash_log.h - DRIVER GD25Q16C + RING BUFFER + AT+INFO / AT+RESUME
// =====================================================================
// MYRUMINET - Sensore Gas Stalla v6.6
// Chip: GigaDevice GD25Q16C (16 Mbit = 2 MB)
// Interfaccia: SPI - Slot B RAK19007
// CS = SS (PA4), WP = WB_IO1, HOLD = WB_IO2
// =====================================================================

#ifndef FLASH_LOG_H
#define FLASH_LOG_H

#include "config.h"
#include <SPI.h>


// =====================================================================
// COMANDI SPI FLASH GD25Q16C
// =====================================================================

#define CMD_WRITE_ENABLE      0x06
#define CMD_WRITE_DISABLE     0x04
#define CMD_READ_STATUS_REG1  0x05
#define CMD_READ_DATA         0x03
#define CMD_PAGE_PROGRAM      0x02
#define CMD_SECTOR_ERASE      0x20
#define CMD_CHIP_ERASE        0xC7
#define CMD_READ_JEDEC_ID     0x9F
#define CMD_POWER_DOWN        0xB9
#define CMD_RELEASE_PD        0xAB

#define STATUS_BUSY           0x01


// =====================================================================
// STATO FLASH LOG
// =====================================================================

struct FlashLogHeader {
    uint32_t magic;
    uint32_t writeAddr;
    uint32_t totalRecords;
    uint32_t wrapCount;
};

static FlashLogHeader logHeader;
static bool flash_ok = false;
static bool flash_paused = false;


// =====================================================================
// FORWARD DECLARATIONS
// =====================================================================

void flashLogSaveHeader(void);


// =====================================================================
// FUNZIONI SPI BASSO LIVELLO
// =====================================================================

static void flashSelect(void)   { digitalWrite(FLASH_CS_PIN, LOW);  }
static void flashDeselect(void) { digitalWrite(FLASH_CS_PIN, HIGH); }

static void flashWriteEnable(void)
{
    flashSelect();
    SPI.transfer(CMD_WRITE_ENABLE);
    flashDeselect();
}

static uint8_t flashReadStatus(void)
{
    flashSelect();
    SPI.transfer(CMD_READ_STATUS_REG1);
    uint8_t status = SPI.transfer(0x00);
    flashDeselect();
    return status;
}

static void flashWaitBusy(void)
{
    uint32_t start = millis();
    while (flashReadStatus() & STATUS_BUSY) {
        delay(1);
        if (millis() - start > 10000) {
            Serial.println("[FLASH] Timeout attesa!");
            return;
        }
    }
}

static void flashReadData(uint32_t addr, uint8_t *buf, uint16_t len)
{
    flashSelect();
    SPI.transfer(CMD_READ_DATA);
    SPI.transfer((addr >> 16) & 0xFF);
    SPI.transfer((addr >> 8) & 0xFF);
    SPI.transfer(addr & 0xFF);
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = SPI.transfer(0x00);
    }
    flashDeselect();
}

static void flashPageProgram(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    if (len > FLASH_PAGE_SIZE) len = FLASH_PAGE_SIZE;

    flashWriteEnable();
    flashSelect();
    SPI.transfer(CMD_PAGE_PROGRAM);
    SPI.transfer((addr >> 16) & 0xFF);
    SPI.transfer((addr >> 8) & 0xFF);
    SPI.transfer(addr & 0xFF);
    for (uint16_t i = 0; i < len; i++) {
        SPI.transfer(buf[i]);
    }
    flashDeselect();
    flashWaitBusy();
}

static void flashSectorErase(uint32_t addr)
{
    flashWriteEnable();
    flashSelect();
    SPI.transfer(CMD_SECTOR_ERASE);
    SPI.transfer((addr >> 16) & 0xFF);
    SPI.transfer((addr >> 8) & 0xFF);
    SPI.transfer(addr & 0xFF);
    flashDeselect();
    flashWaitBusy();
}

static void flashChipErase(void)
{
    flashWriteEnable();
    flashSelect();
    SPI.transfer(CMD_CHIP_ERASE);
    flashDeselect();
    flashWaitBusy();
}

static void flashPowerDown(void)
{
    flashSelect();
    SPI.transfer(CMD_POWER_DOWN);
    flashDeselect();
}

static void flashReleasePowerDown(void)
{
    flashSelect();
    SPI.transfer(CMD_RELEASE_PD);
    flashDeselect();
    delay(1);
}


// =====================================================================
// INIT FLASH + HEADER
// =====================================================================

bool flashLogInit(void)
{
    Serial.println("[FLASH] Init GD25Q16C...");

    // Setup pin
    pinMode(FLASH_WP_PIN, OUTPUT);
    digitalWrite(FLASH_WP_PIN, HIGH);

    pinMode(FLASH_CS_PIN, OUTPUT);
    flashDeselect();

    SPI.begin();
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    delay(50);

    // Risveglia flash
    flashReleasePowerDown();
    delay(10);

    // Verifica JEDEC ID
    flashSelect();
    SPI.transfer(CMD_READ_JEDEC_ID);
    uint8_t mfr  = SPI.transfer(0x00);
    uint8_t type = SPI.transfer(0x00);
    uint8_t cap  = SPI.transfer(0x00);
    flashDeselect();

    Serial.print("[FLASH] JEDEC: 0x");
    if (mfr < 0x10) Serial.print("0"); Serial.print(mfr, HEX);
    Serial.print(" 0x");
    if (type < 0x10) Serial.print("0"); Serial.print(type, HEX);
    Serial.print(" 0x");
    if (cap < 0x10) Serial.print("0"); Serial.println(cap, HEX);

    if (mfr != 0xC8 || cap != 0x15) {
        Serial.println("[FLASH] ERRORE: GD25Q16C non rilevata!");
        flash_ok = false;
        return false;
    }

    Serial.println("[FLASH] GD25Q16C rilevata (2 MB)");

    // Leggi header dal settore 0
    uint8_t headerBuf[16];
    flashReadData(FLASH_LOG_HEADER_ADDR, headerBuf, 16);

    logHeader.magic        = ((uint32_t)headerBuf[0] << 24) | ((uint32_t)headerBuf[1] << 16) |
                             ((uint32_t)headerBuf[2] << 8)  | headerBuf[3];
    logHeader.writeAddr    = ((uint32_t)headerBuf[4] << 24) | ((uint32_t)headerBuf[5] << 16) |
                             ((uint32_t)headerBuf[6] << 8)  | headerBuf[7];
    logHeader.totalRecords = ((uint32_t)headerBuf[8] << 24) | ((uint32_t)headerBuf[9] << 16) |
                             ((uint32_t)headerBuf[10] << 8) | headerBuf[11];
    logHeader.wrapCount    = ((uint32_t)headerBuf[12] << 24) | ((uint32_t)headerBuf[13] << 16) |
                             ((uint32_t)headerBuf[14] << 8) | headerBuf[15];

    if (logHeader.magic != 0xF1A5F1A5) {
        Serial.println("[FLASH] Header non valido, inizializzazione flash...");
        logHeader.magic = 0xF1A5F1A5;
        logHeader.writeAddr = FLASH_LOG_START_ADDR;
        logHeader.totalRecords = 0;
        logHeader.wrapCount = 0;
        flashLogSaveHeader();
    } else {
        Serial.print("[FLASH] Header OK | Record: ");
        Serial.print(logHeader.totalRecords);
        Serial.print(" | WriteAddr: 0x");
        Serial.print(logHeader.writeAddr, HEX);
        Serial.print(" | Wraps: ");
        Serial.println(logHeader.wrapCount);
    }

    flash_ok = true;
    flash_paused = false;
    return true;
}


// =====================================================================
// SALVA HEADER SU FLASH
// =====================================================================

void flashLogSaveHeader(void)
{
    uint8_t headerBuf[16];

    headerBuf[0]  = (logHeader.magic >> 24) & 0xFF;
    headerBuf[1]  = (logHeader.magic >> 16) & 0xFF;
    headerBuf[2]  = (logHeader.magic >> 8) & 0xFF;
    headerBuf[3]  = logHeader.magic & 0xFF;
    headerBuf[4]  = (logHeader.writeAddr >> 24) & 0xFF;
    headerBuf[5]  = (logHeader.writeAddr >> 16) & 0xFF;
    headerBuf[6]  = (logHeader.writeAddr >> 8) & 0xFF;
    headerBuf[7]  = logHeader.writeAddr & 0xFF;
    headerBuf[8]  = (logHeader.totalRecords >> 24) & 0xFF;
    headerBuf[9]  = (logHeader.totalRecords >> 16) & 0xFF;
    headerBuf[10] = (logHeader.totalRecords >> 8) & 0xFF;
    headerBuf[11] = logHeader.totalRecords & 0xFF;
    headerBuf[12] = (logHeader.wrapCount >> 24) & 0xFF;
    headerBuf[13] = (logHeader.wrapCount >> 16) & 0xFF;
    headerBuf[14] = (logHeader.wrapCount >> 8) & 0xFF;
    headerBuf[15] = logHeader.wrapCount & 0xFF;

    flashSectorErase(FLASH_LOG_HEADER_ADDR);
    flashPageProgram(FLASH_LOG_HEADER_ADDR, headerBuf, 16);
}


// =====================================================================
// SCRIVI RECORD JSON SU FLASH (RING BUFFER)
// =====================================================================

bool flashLogWrite(const char *jsonRecord)
{
    if (!flash_ok) {
        Serial.println("[FLASH] Non inizializzata");
        return false;
    }

    if (flash_paused) {
        Serial.println("[FLASH] Log in pausa (AT+INFO attivo)");
        return false;
    }

    uint16_t len = strlen(jsonRecord);
    if (len >= FLASH_RECORD_SIZE) {
        Serial.println("[FLASH] Record troppo lungo! Troncato.");
        len = FLASH_RECORD_SIZE - 1;
    }

    uint32_t currentSector = logHeader.writeAddr & ~(FLASH_SECTOR_SIZE - 1);
    uint32_t offsetInSector = logHeader.writeAddr - currentSector;

    if (offsetInSector == 0) {
        Serial.print("[FLASH] Erase settore 0x");
        Serial.println(currentSector, HEX);
        flashSectorErase(currentSector);
    }

    uint8_t pageBuf[FLASH_RECORD_SIZE];
    memset(pageBuf, 0xFF, FLASH_RECORD_SIZE);
    memcpy(pageBuf, jsonRecord, len);
    pageBuf[len] = 0x00;

    flashPageProgram(logHeader.writeAddr, pageBuf, FLASH_RECORD_SIZE);

    Serial.print("[FLASH] Record #");
    Serial.print(logHeader.totalRecords + 1);
    Serial.print(" scritto @ 0x");
    Serial.println(logHeader.writeAddr, HEX);

    logHeader.writeAddr += FLASH_RECORD_SIZE;
    logHeader.totalRecords++;

    if (logHeader.writeAddr >= FLASH_LOG_END_ADDR) {
        logHeader.writeAddr = FLASH_LOG_START_ADDR;
        logHeader.wrapCount++;
        Serial.println("[FLASH] Ring buffer wrap!");
    }

    flashLogSaveHeader();

    return true;
}


// =====================================================================
// DUMP COMPLETO LOG (per AT+INFO)
// =====================================================================

void flashLogDump(void)
{
    if (!flash_ok) {
        Serial.println("[FLASH] Non inizializzata");
        return;
    }

    Serial.println();
    Serial.println("=============================================");
    Serial.println("  FLASH LOG DUMP - AT+INFO");
    Serial.println("=============================================");
    Serial.print("  Record totali:   ");
    Serial.println(logHeader.totalRecords);
    Serial.print("  Indirizzo write: 0x");
    Serial.println(logHeader.writeAddr, HEX);
    Serial.print("  Wrap count:      ");
    Serial.println(logHeader.wrapCount);
    Serial.println("=============================================");
    Serial.println();

    if (logHeader.totalRecords == 0) {
        Serial.println("[FLASH] Nessun record salvato.");
        return;
    }

    uint32_t maxRecords = (FLASH_LOG_END_ADDR - FLASH_LOG_START_ADDR) / FLASH_RECORD_SIZE;
    uint32_t recordsToRead = logHeader.totalRecords;
    if (recordsToRead > maxRecords) {
        recordsToRead = maxRecords;
    }

    uint32_t readAddr;
    if (logHeader.wrapCount > 0) {
        readAddr = logHeader.writeAddr;
    } else {
        readAddr = FLASH_LOG_START_ADDR;
    }

    uint8_t recordBuf[FLASH_RECORD_SIZE];

    Serial.println("[");

    for (uint32_t i = 0; i < recordsToRead; i++) {
        if (readAddr >= FLASH_LOG_END_ADDR) {
            readAddr = FLASH_LOG_START_ADDR;
        }

        flashReadData(readAddr, recordBuf, FLASH_RECORD_SIZE);

        if (recordBuf[0] != 0xFF && recordBuf[0] != 0x00) {
            if (i > 0) Serial.println(",");
            for (uint16_t j = 0; j < FLASH_RECORD_SIZE; j++) {
                if (recordBuf[j] == 0x00 || recordBuf[j] == 0xFF) break;
                Serial.print((char)recordBuf[j]);
            }
        }

        readAddr += FLASH_RECORD_SIZE;
    }

    Serial.println();
    Serial.println("]");
    Serial.println();
    Serial.println("=============================================");
    Serial.println("  DUMP COMPLETATO");
    Serial.print("  Record letti: ");
    Serial.println(recordsToRead);
    Serial.println("  Invia AT+RESUME per riprendere il ciclo");
    Serial.println("  Invia AT+CLEARLOG per cancellare il log");
    Serial.println("=============================================");
}


// =====================================================================
// CANCELLA LOG
// =====================================================================

void flashLogClear(void)
{
    if (!flash_ok) {
        Serial.println("[FLASH] Non inizializzata");
        return;
    }

    Serial.println("[FLASH] Cancellazione completa in corso...");
    Serial.println("[FLASH] Attendere (puo' richiedere 20-40 secondi)...");

    uint32_t t0 = millis();
    flashChipErase();
    uint32_t t1 = millis();

    Serial.print("[FLASH] Chip erase completato in ");
    Serial.print((t1 - t0) / 1000);
    Serial.println(" secondi");

    logHeader.magic = 0xF1A5F1A5;
    logHeader.writeAddr = FLASH_LOG_START_ADDR;
    logHeader.totalRecords = 0;
    logHeader.wrapCount = 0;
    flashLogSaveHeader();

    Serial.println("[FLASH] Log cancellato e header reinizializzato");
}


// =====================================================================
// POWER DOWN / WAKE (risparmio energetico)
// =====================================================================

void flashLogSleep(void)
{
    if (flash_ok) {
        flashPowerDown();
    }
}

void flashLogWake(void)
{
    if (flash_ok) {
        flashReleasePowerDown();
        delay(5);
    }
}


#endif // FLASH_LOG_H

