#if !defined(UNIT_RAM_H)
#define UNIT_RAM_H 1

#include <stdint.h>

#include "hardware/structs/systick.h"
#include "pico.h"

// The co-processor's code and data (FEATURE_MAP_UNIT: the map unit, the snoop), in the last SRAM bank
// (memmap_firmware.ld's UNIT_RAM), copied there from flash by map_unit_init. Its interrupts run from
// there and call nothing outside it: no flash, no library (no division, no switch tables, no memset:
// their files are built without loop patterns).
#define UNIT_CODE(name) __attribute__((section(".unit_ram.text." #name), noinline)) name
#define UNIT_DATA __attribute__((section(".unit_ram.data")))

// Each core's own SysTick, on the processor's clock: times in cycles.
static __force_inline void systick_start()
{
    systick_hw->rvr = 0xFFFFFF;
    systick_hw->cvr = 0;
    systick_hw->csr = 5; // on, the processor's clock, no interrupt
}

static __force_inline uint32_t since(uint32_t then)
{
    return (then - systick_hw->cvr) & 0xFFFFFF; // SysTick counts down, 24 bits
}

#endif
