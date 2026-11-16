#if !defined(ROM_H)
#define ROM_H 1

#include "hardware/pio.h"

void rom_init_programs();

// Core 1 started (the ROM loop, or a bench) takes the SDK's start from flash, under a microsecond at
// 270 MHz: its starter waits this long before it goes on, so that a flash erase or program after it
// (the map unit's store) never finds core 1 there.
static constexpr uint32_t CORE1_STARTED_US = 10;

// The ROM loop on core 1 (it runs from RAM); returns once core 1 is in it.
void rom_service_start();
void rom_service_stop();

uint8_t *rom_get_buffer();

bool rom_check_oe();

#endif // ROM_H
