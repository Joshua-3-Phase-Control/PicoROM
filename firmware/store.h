#if !defined(STORE_H)
#define STORE_H 1

#include <stddef.h>
#include <stdint.h>

// The store (FEATURE_MAP_UNIT; DME-Sim's docs/Paired_Fuel_Learning.md, step 1): what the P28 learns,
// kept in its flash across power cycles as records, a 256-byte page each, in a ring of sectors below
// the image. Its code and data are in bank 2 (memmap_firmware.ld's STORE_RAM: .store_code, copied
// there from flash by map_unit_init, beside .store, what it keeps): bank 3, the unit's, has no room.
// L2 posts a kind's bytes; thread mode programs them, holding off only the interrupts whose code is in
// flash; the board's start loads the newest whole record of each kind.
#define STORE_CODE(name) __attribute__((section(".store_code.text." #name), noinline)) name
#define STORE_DATA __attribute__((section(".store_code.data")))

// The store's kinds (a record's first byte).
static constexpr uint32_t STORE_KNOCK = 1; // format 24's knock maps: each pair's lower, a byte a point

// The board's start, from map_unit_init (flash code: the board's alone): the flash's JEDEC ID.
void store_board_start();

// The unit's start (map_unit_start, which DME-Sim calls too): the store's sectors read, the newest
// whole record of each kind whose key matches the image loaded, the next record's place found.
void store_start();

// L2, once a tick after the knock and film work: a clear the PC asked for, and a kind changed since its
// last record (at most once a minute) posted for thread mode.
void store_tick();

// Thread mode (map_unit_update): a posted record programmed, or the next sector erased ahead.
void store_update();

// Thread mode, after store_update (flash code: the board's alone): store_bench's next operation on the
// sector below the store's, erased and programmed n times, each timed (store.cpp).
void store_bench_update();

// The CRC-32 (reflected, polynomial EDB88320h) of one more byte.
uint32_t store_crc(uint32_t crc, uint32_t byte);

// The PC's clear of kinds (bit k: kind k), which L2 does (the knock maps once the engine has stopped),
// and the kinds asked for and not yet cleared.
void store_ask_clear(uint32_t kinds);
uint32_t store_clears_pending();

// store, store_flash, store_clear, store_bench (its counts: cycles, faults, the erase's longest and mean,
// the program's, us) and store_irqs (core 0's enabled interrupts, those whose handler is in flash,
// SysTick's own); store_clear and store_bench set.
bool store_get_parameter(const char *name, char *value, size_t value_size);
bool store_set_parameter(const char *name, const char *value);

#endif // STORE_H
