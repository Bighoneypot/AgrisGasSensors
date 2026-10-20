
// =====================================================================
// memory_monitor.h - STUB MEMORY MONITOR per nRF52840 (256 KB SRAM)
// =====================================================================
// MYRUMINET - Sensore Gas Stalla v7.2
// RAK4630 (nRF52840) / RAK19007 Rev.C
//
// Sul STM32WLE5 (64 KB) il memory monitor era CRITICO perche' il
// LoRaWAN stack lasciava solo ~68-92 byte liberi di heap.
// Sul nRF52840 (256 KB) il margine e' enormemente piu' ampio.
//
// Questo stub fornisce le stesse API del vecchio memory_monitor.h
// ma usa il gap heap-stack per stimare la RAM libera.
// =====================================================================

#ifndef MEMORY_MONITOR_H
#define MEMORY_MONITOR_H

// =====================================================================
// STIMA RAM LIBERA (heap-stack gap)
// =====================================================================

extern "C" char *sbrk(int incr);

static uint32_t mem_heap_free_boot = 0;
static uint32_t mem_heap_min       = 0xFFFFFFFF;

static uint32_t memGetFreeHeap(void)
{
    char stackVar;
    char *heapEnd = (char *)sbrk(0);
    if (heapEnd == (char *)-1) return 0;
    uint32_t freeBytes = (uint32_t)(&stackVar - heapEnd);
    return freeBytes;
}

static uint32_t memGetFreeStack(void)
{
    return memGetFreeHeap();  // Sul nRF52840 e' la stessa metrica
}

static uint32_t memGetUsedHeap(void)
{
    return 0;  // Non disponibile senza mallinfo
}

static void memUpdateMin(void)
{
    uint32_t current = memGetFreeHeap();
    if (current < mem_heap_min) {
        mem_heap_min = current;
    }
}

static void memSetBaseline(void)
{
    mem_heap_free_boot = memGetFreeHeap();
    mem_heap_min       = mem_heap_free_boot;
}

static uint32_t memGetFreeHeapForLog(void)
{
    return memGetFreeHeap();
}

static uint32_t memGetMinHeapForLog(void)
{
    return mem_heap_min;
}

static bool memCheckCritical(void)
{
    uint32_t free = memGetFreeHeap();
    return (free < 1024);  // 1 KB threshold (generoso su 256 KB)
}

static void memPrintInfo(const char *label)
{
    uint32_t free = memGetFreeHeap();
    memUpdateMin();

    Serial.print("[MEM] ");
    Serial.print(label);
    Serial.print(" | Free: ");
    Serial.print(free);
    Serial.print(" bytes | Min: ");
    Serial.print(mem_heap_min);
    Serial.print(" bytes | Boot: ");
    Serial.print(mem_heap_free_boot);
    Serial.print(" | Delta: ");
    int32_t delta = (int32_t)free - (int32_t)mem_heap_free_boot;
    Serial.println(delta);
}


#endif // MEMORY_MONITOR_H

