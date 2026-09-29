
// =====================================================================
// watchdog_hw.h — Diagnostica Reset Reason (SOLO LETTURA)
// Firmware v6.7.0 — Sensore Gas Stalla
// =====================================================================
//
// Questo file contiene SOLO la lettura del registro RCC_CSR
// per capire perche' la board si e' resettata.
//
// NESSUN timer, NESSUN ATZ, NESSUN callback.
// La FSM e' 100% sequenziale.
//
// =====================================================================
#ifndef WATCHDOG_HW_H
#define WATCHDOG_HW_H

#include <Arduino.h>

// =====================================================================
// Registro RCC_CSR per causa reset
// =====================================================================
#define RCC_BASE_ADDR   0x58000000UL
#define RCC_CSR_REG     (*(volatile uint32_t *)(RCC_BASE_ADDR + 0x094))

// =====================================================================
// Variabili modulo
// =====================================================================
static bool _wasWatchdogReset = false;

// =====================================================================
// hwWatchdogWasReset()
// =====================================================================
static bool hwWatchdogWasReset(void)
{
    return _wasWatchdogReset;
}

// =====================================================================
// hwWatchdogCheckResetReason()
// =====================================================================
static void hwWatchdogCheckResetReason(void)
{
    uint32_t csr = RCC_CSR_REG;

    _wasWatchdogReset = (csr & (1UL << 29)) != 0;
    bool swReset = (csr & (1UL << 28)) != 0;

    Serial.print("[RESET] Reason (RCC_CSR): 0x");
    Serial.println(csr >> 24, HEX);

    if (_wasWatchdogReset) Serial.println("[RESET] -> IWDG Reset");
    if (swReset) {
        Serial.println("[RESET] -> Software Reset");
        _wasWatchdogReset = true;
    }
    if (csr & (1UL << 26)) Serial.println("[RESET] -> Pin Reset");
    if (csr & (1UL << 27)) Serial.println("[RESET] -> BOR Reset");

    // Pulisci i flag
    RCC_CSR_REG |= (1UL << 23);
}

// =====================================================================
// hwWatchdogInit() — Stub (nessun timer da inizializzare)
// =====================================================================
static bool hwWatchdogInit(uint8_t timeout_sec)
{
    (void)timeout_sec;
    Serial.println("[RESET] Diagnostica reset attiva (no timer, no ATZ)");
    return true;
}

// =====================================================================
// hwWatchdogFeed() — Stub (nessun timer da alimentare)
// =====================================================================
static inline void hwWatchdogFeed(void)
{
    // Noop — nessun timer attivo
}

// =====================================================================
// hwWatchdogStarve() — Stub (nessun watchdog da far scattare)
// =====================================================================
static void hwWatchdogStarve(const char* reason)
{
    Serial.print("[RESET] Richiesto reset per: ");
    Serial.println(reason);
    Serial.println("[RESET] Nessun meccanismo di reset automatico.");
    Serial.println("[RESET] Il dispositivo continuera' a funzionare.");
}

// =====================================================================
// sleepSimple() — Sleep diretto senza timer
// =====================================================================
static void sleepSimple(uint32_t totalMs)
{
    // Sleep a blocchi da 60s per evitare overflow
    uint32_t blockMs = 60000UL;
    uint32_t remaining = totalMs;

    while (remaining > 0) {
        uint32_t thisBlock = (remaining > blockMs) ? blockMs : remaining;
        api.system.sleep.all(thisBlock);
        remaining -= thisBlock;
    }
}

#endif // WATCHDOG_HW_H

