
#ifndef MEMORY_MONITOR_H
#define MEMORY_MONITOR_H

// =====================================================================
// memory_monitor.h - MONITORAGGIO HEAP E STACK
// =====================================================================
// RAK3172-E (STM32WLE5CCU6)
// RAM: 64 KB (0x20000000 - 0x20010000)
//
// NOTA IMPORTANTE:
//   Su RUI3/STM32WLE5, mallinfo() riporta solo l'heap dinamico
//   che e' MOLTO piccolo (~248 bytes). La maggior parte della RAM
//   e' usata come stack (~64 KB). Questo e' NORMALE.
//   La soglia critica deve essere basata su ENTRAMBI heap + stack.
//
// Fornisce:
//   - memGetFreeHeap()   -> bytes liberi nell'heap
//   - memGetUsedHeap()   -> bytes usati nell'heap
//   - memGetFreeStack()  -> bytes liberi nello stack
//   - memPrintInfo()     -> stampa completa su seriale
//   - memGetFreeHeapStr() -> stringa per JSON log
// =====================================================================

#include <malloc.h>

// Indirizzo fine RAM STM32WLE5 (64 KB)
#define RAM_START   0x20000000
#define RAM_END     0x20010000
#define RAM_SIZE    (RAM_END - RAM_START)   // 65536 bytes

// Soglie critiche
// HEAP: su RUI3 l'heap dinamico e' ~248 bytes totali,
//       quindi 16 bytes liberi e' il minimo accettabile
// STACK: 2 KB e' il minimo per operazioni LoRaWAN + sensori
#define MEM_HEAP_CRITICAL    16      // 16 bytes heap minimo
#define MEM_STACK_CRITICAL   2048    // 2 KB stack minimo


// =====================================================================
// VARIABILI PER TRACKING
// =====================================================================

static uint32_t mem_heap_free_min = 0xFFFFFFFF;   // Minimo heap libero mai visto
static uint32_t mem_heap_free_boot = 0;            // Heap libero al boot (baseline)
static bool     mem_baseline_set = false;


// =====================================================================
// FUNZIONI HEAP (via mallinfo - standard C)
// =====================================================================

// Heap libero in bytes
static inline uint32_t memGetFreeHeap(void)
{
    struct mallinfo mi = mallinfo();
    return mi.fordblks;
}

// Heap in uso in bytes
static inline uint32_t memGetUsedHeap(void)
{
    struct mallinfo mi = mallinfo();
    return mi.uordblks;
}

// Heap totale arena in bytes
static inline uint32_t memGetTotalHeap(void)
{
    struct mallinfo mi = mallinfo();
    return mi.arena;
}


// =====================================================================
// FUNZIONE STACK (via stack pointer)
// =====================================================================

// Stima bytes liberi nello stack
static inline uint32_t memGetFreeStack(void)
{
    volatile uint32_t sp;
    __asm__ volatile("mov %0, sp" : "=r"(sp));

    // Lo stack cresce verso il basso su ARM Cortex-M
    // Il "free" reale e' tra la fine dell'heap e lo stack pointer
    struct mallinfo mi = mallinfo();
    uint32_t heap_end = RAM_START + mi.arena;

    if (sp > heap_end) {
        return sp - heap_end;
    }
    return 0;  // Stack overflow imminente!
}


// =====================================================================
// BASELINE (chiamare una volta nel setup)
// =====================================================================

void memSetBaseline(void)
{
    mem_heap_free_boot = memGetFreeHeap();
    mem_heap_free_min = mem_heap_free_boot;
    mem_baseline_set = true;

    Serial.print("[MEM] Baseline heap libero: ");
    Serial.print(mem_heap_free_boot);
    Serial.println(" bytes");
}


// =====================================================================
// UPDATE MINIMO (chiamare ad ogni ciclo)
// =====================================================================

void memUpdateMin(void)
{
    uint32_t free_now = memGetFreeHeap();
    if (free_now < mem_heap_free_min) {
        mem_heap_free_min = free_now;
    }
}


// =====================================================================
// STAMPA COMPLETA SU SERIALE
// =====================================================================

void memPrintInfo(const char *label)
{
    struct mallinfo mi = mallinfo();
    uint32_t free_stack = memGetFreeStack();
    uint32_t free_heap = mi.fordblks;
    uint32_t used_heap = mi.uordblks;

    Serial.println();
    Serial.print("[MEM] --- ");
    Serial.print(label);
    Serial.println(" ---");
    Serial.print("[MEM] Heap totale (arena): ");
    Serial.print(mi.arena);
    Serial.println(" bytes");
    Serial.print("[MEM] Heap in uso:         ");
    Serial.print(used_heap);
    Serial.println(" bytes");
    Serial.print("[MEM] Heap libero:         ");
    Serial.print(free_heap);
    Serial.println(" bytes");
    Serial.print("[MEM] Heap libero minimo:  ");
    Serial.print(mem_heap_free_min);
    Serial.println(" bytes");

    if (mem_baseline_set) {
        int32_t delta = (int32_t)free_heap - (int32_t)mem_heap_free_boot;
        Serial.print("[MEM] Delta dal boot:      ");
        Serial.print(delta);
        Serial.print(" bytes (");
        if (delta < 0) {
            Serial.print("LEAK? -");
            Serial.print(-delta);
        } else if (delta > 0) {
            Serial.print("OK +");
            Serial.print(delta);
        } else {
            Serial.print("stabile");
        }
        Serial.println(")");
    }

    Serial.print("[MEM] Stack libero:        ");
    Serial.print(free_stack);
    Serial.println(" bytes");
    Serial.print("[MEM] RAM totale:          ");
    Serial.print(RAM_SIZE);
    Serial.println(" bytes (64 KB)");
    Serial.println();
}


// =====================================================================
// VALORI PER JSON LOG
// =====================================================================

// Ritorna heap libero corrente (per inserimento nel JSON)
static inline uint32_t memGetFreeHeapForLog(void)
{
    memUpdateMin();
    return memGetFreeHeap();
}

// Ritorna heap libero minimo (per inserimento nel JSON)
static inline uint32_t memGetMinHeapForLog(void)
{
    return mem_heap_free_min;
}


// =====================================================================
// CHECK CRITICO (basato su heap + stack)
// =====================================================================
// Su RUI3/STM32WLE5:
//   - Heap dinamico e' ~248 bytes totali (NORMALE, non e' un leak)
//   - Stack e' ~64 KB (qui risiede la maggior parte dei dati)
//   - Il check critico deve guardare ENTRAMBI

bool memCheckCritical(void)
{
    uint32_t free_heap = memGetFreeHeap();
    uint32_t free_stack = memGetFreeStack();
    bool critical = false;

    if (free_heap < MEM_HEAP_CRITICAL) {
        Serial.println("[MEM] !!! HEAP CRITICO !!!");
        Serial.print("[MEM] Solo ");
        Serial.print(free_heap);
        Serial.print(" / ");
        Serial.print(MEM_HEAP_CRITICAL);
        Serial.println(" bytes minimi!");
        critical = true;
    }

    if (free_stack < MEM_STACK_CRITICAL) {
        Serial.println("[MEM] !!! STACK CRITICO !!!");
        Serial.print("[MEM] Solo ");
        Serial.print(free_stack);
        Serial.print(" / ");
        Serial.print(MEM_STACK_CRITICAL);
        Serial.println(" bytes minimi!");
        critical = true;
    }

    if (!critical) {
        Serial.print("[MEM] OK - Heap: ");
        Serial.print(free_heap);
        Serial.print("B | Stack: ");
        Serial.print(free_stack);
        Serial.println("B");
    }

    return critical;
}


#endif // MEMORY_MONITOR_H

