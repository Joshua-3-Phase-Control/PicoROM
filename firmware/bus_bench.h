#if !defined(BUS_BENCH_H)
#define BUS_BENCH_H 1

#include <stdint.h>

#include "system.h"

// The bus bench (FEATURE_MAP_UNIT): core 1 plays the 8051 on the board itself, through the GPIO input
// overrides of the address, /OE, /CE and data pins. No pin is driven and the data buffer is held off,
// but the board serves no 8051 meanwhile. The 8051's part runs from flash (it fits the XIP cache), so
// it doesn't hold up the unit's bank as no 8051 would. Times are in cycles of the system clock.
#if defined(FEATURE_MAP_UNIT)

static constexpr uint32_t CYCLES_PER_US = SYS_CLK_HZ / 1000000;
static constexpr uint32_t MACHINE_CYCLE = 2 * CYCLES_PER_US;        // the 8051's: 12 clocks at 6 MHz
static constexpr uint32_t READ_CYCLES = 465 * CYCLES_PER_US / 1000; // /PSEN low: 3 TCLCL - 35 ns

// The bus taken (the ROM service stopped, the buffer off, /CE low and /OE high), then run on core 1;
// false if a bench is running.
bool bench_launch(void (*run)());
bool bench_running();

// Core 1's, its run over: the board is given back by bench_update, from the main loop.
[[noreturn]] void bench_finished();
void bench_update();

// The bus taken and given back on core 0 (a single read: map_probe), the ROM service left running.
void bench_take();
void bench_give();

// The pins the 8051 would drive: the address (A13 and up, which addr_mask leaves out, read 0 in a DME
// and are set from the address here), /OE (its /PSEN) and the data pins (a MOVX's byte on P0).
void bench_address(uint32_t address);
void bench_oe(bool low);
void bench_data(uint32_t byte);

// One read as the 8051 makes it: the address, then /OE low for 465 ns; the byte the image holds there
// (through the mask), as the 8051 latches it as the read ends. Returns when /OE fell; end, when it rose.
uint32_t bench_read(uint32_t address, const uint8_t *rom, uint32_t mask, uint32_t *value, uint32_t *end);

#endif

#endif
