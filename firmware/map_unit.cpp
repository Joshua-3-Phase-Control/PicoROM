// The map unit (FEATURE_MAP_UNIT): the board looks maps up for the 8051, as DME-Sim's MapUnit models
// it (experiments/coproc_maps there; docs/map_unit.md here). The board serves the 8051 the image's
// first 8 KiB (addr_mask 0x1fff: the DME ties the socket's A13 high) and keeps its own memory after
// them, which the 8051 never reads: from 2000h the mark ("MAPU", format 2, the window's page, the
// number of maps, the index's address, high byte first), the index (each map's address and its axes)
// and the maps. The window is one page of the 8 KiB, which the mark names. The socket has no write
// line, so the 8051 asks by the addresses it reads there, and the unit fills the page with each
// answer:
//
//   window + id   picks map id; the byte read is the page as written: by id, each map's first
//                 input's RAM address
//   window + v    sends v, the map's next input; the byte read is the next input's address, or 0
//   window + any  after the last: the map's value at the inputs sent, rom_lean's Map_Lookup and
//                 Map_Lookup_2D's bit for bit; then the page is as written again
//
// map_window (map_unit.pio) pushes A0-A7 of each read of the page once the read is over, and FFFFFFFFh
// for a read of 0000h, where the 8051 starts after a reset: the page as written again, and the unit
// waiting for a pick. The 8051's routine reads each answer 8 us or more after it asks (map_bus times
// the board). The PC reads and writes the image as it wrote it: the page is kept for it apart (the
// shadow), and goes to flash as written.

#include "map_unit.h"

#if defined(FEATURE_MAP_UNIT)

#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/structs/io_bank0.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/systick.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico/multicore.h"

#include "pio_programs.h"
#include "rom.h"
#include "str_util.h"
#include "system.h"

// The unit's code and data, in the last SRAM bank (memmap_firmware.ld's UNIT_RAM), copied there from
// flash by map_unit_init. Its interrupt runs from there and calls nothing outside it: no flash, no
// library (no division, no switch tables, no memset: this file is built without loop patterns).
#define UNIT_CODE(name) __attribute__((section(".unit_ram.text." #name), noinline)) name
#define UNIT_DATA __attribute__((section(".unit_ram.data")))

extern "C"
{
    extern uint8_t __unit_ram_start__[], __unit_ram_end__[], __unit_ram_source__[];
    extern uint32_t __StackBottom[];
}

static_assert(ROM_SIZE <= 0x30000, "the map unit's bank is the last one");
static_assert(N_OE_PINS == 2, "the bus bench reads with /OE and /CE");

static constexpr uint32_t MARK = 0x2000;          // the unit's own memory, after the 8051's 8 KiB
static constexpr uint32_t EIGHT_KIB = 0x1FFF;     // the address mask the unit works with
static constexpr uint32_t PAGE = 0x100;
static constexpr uint32_t STARTED = 0xFFFFFFFF;   // map_window's word for a read of 0000h

enum : uint32_t
{
    PICK_DUE,  // waiting for a pick; the page as written
    INPUT_DUE, // the map's inputs due
    VALUE_DUE  // the page holds the value
};

// Cycles the interrupt took, from its start (map_cycles).
struct Cycles
{
    uint32_t count, most;
    uint64_t sum;
};

struct Unit
{
    uint8_t *rom;
    uint32_t mask;    // the address mask the board serves through, within the 8051's 64 KiB
    volatile bool on; // the image has the mark, and the board serves the 8051 its 8 KiB
    uint32_t window;  // the page's address
    uint32_t phase;
    uint32_t map, axes, sent;
    uint8_t inputs[2];
    volatile uint32_t lookups;
    volatile uint32_t starts; // reads of 0000h
    volatile uint32_t served; // reads answered (the bus bench waits on it)
    Cycles answers;           // a pick's, or a send's before the last
    Cycles values;            // the last send's: the lookup, and the page filled with its value
    Cycles restores;          // the value's read, or a start: the page as written again
};

static Unit unit UNIT_DATA = {};
static uint32_t shadow[PAGE / 4] UNIT_DATA = {}; // the page as the PC wrote it
static uint32_t reciprocal[256] UNIT_DATA = {};  // ceil(2^16 / d): x / d = x * it >> 16, exact below 256

// ---------------------------------------------------------------------------------------------------
// The lookup, format 2 (format 1's maps): as MapUnit.Lookup and rom_lean's routines make it, from the
// unit's own memory. An axis is 9 bytes: its input's RAM address, an offset, then each of two
// segments' span, step and factor, and the index where the second starts. A one-axis map's values
// follow its axis; a two-axis map's second axis follows the first, then its column count, then the
// values a row at a time.

static __force_inline uint32_t at(uint32_t address)
{
    return unit.rom[address & 0xFFFF];
}

// Even_Axis: the point at or below the input (low byte) and the Q8 fraction to the next (high byte).
static __force_inline uint32_t place(uint32_t axis, uint32_t input)
{
    uint32_t offset = at(axis + 1), x = input >= offset ? input - offset : 0, span = at(axis + 2);
    uint32_t step = at(axis + 3), factor = at(axis + 4), start = 0;
    if (x >= span)
    {
        x -= span;
        uint32_t limit = at(axis + 5);
        if (x > limit)
            x = limit;
        step = at(axis + 6);
        factor = at(axis + 7);
        start = at(axis + 8);
    }
    uint32_t index = x, remainder = 0; // DIV AB by 0 leaves A as it was, and B (0)
    if (step != 0)
    {
        index = x * reciprocal[step] >> 16;
        remainder = x - index * step;
    }
    return ((index + start) & 0xFF) | ((remainder * factor) & 0xFF) << 8;
}

// Lerp: from + (to - from) x fraction / 256, rounded to nearest, for either sign.
static __force_inline uint32_t lerp(uint32_t from, uint32_t to, uint32_t fraction)
{
    if (to >= from)
        return (from + (((to - from) * fraction + 0x80) >> 8)) & 0xFF;
    return (from - (((from - to) * fraction + 0x80) >> 8)) & 0xFF;
}

// Map_Cell: the value at base + offset, towards the next (the offset a byte, as A is) by the fraction.
static __force_inline uint32_t cell(uint32_t base, uint32_t offset, uint32_t fraction)
{
    uint32_t value = at(base + offset);
    return fraction == 0 ? value : lerp(value, at(base + ((offset + 1) & 0xFF)), fraction);
}

static uint32_t UNIT_CODE(map_unit_lookup)(uint32_t map, uint32_t axes, uint32_t first, uint32_t second)
{
    uint32_t row = place(map, first), row_fraction = row >> 8;
    row &= 0xFF;
    if (axes < 2)
        return cell(map, (row + 9) & 0xFF, row_fraction);
    uint32_t axis = (map + 9) & 0xFFFF;
    uint32_t column = place(axis, second), column_fraction = column >> 8;
    column &= 0xFF;
    uint32_t columns = at(axis + 9);
    uint32_t cells = (axis + row * columns + column) & 0xFFFF; // DPTR: the cell - 10
    uint32_t here = cell(cells, 0x0A, column_fraction);
    if (row_fraction == 0)
        return here;
    return lerp(here, cell(cells, (columns + 0x0A) & 0xFF, column_fraction), row_fraction);
}

// ---------------------------------------------------------------------------------------------------
// The window: the interrupt and the page.

// The 256 bytes at p set to the word's, 16 at a store: about 105 cycles (single stores, as GCC
// unrolls them, take about 256).
__attribute__((naked, noinline, section(".unit_ram.text.fill_page"))) static void fill_page(uint32_t *p,
                                                                                          uint32_t word)
{
    // Divided syntax, as GCC's inline assembly here is (rom.cpp's too): sub and mov set the flags.
    __asm volatile("push {r4, r5}                 \n\t"
                   "mov r2, r1                    \n\t"
                   "mov r3, r1                    \n\t"
                   "mov r4, r1                    \n\t"
                   "mov r5, r1                    \n\t"
                   "mov r1, #4                    \n\t"
                   "1:                            \n\t"
                   "stmia r0!, {r2, r3, r4, r5}   \n\t"
                   "stmia r0!, {r2, r3, r4, r5}   \n\t"
                   "stmia r0!, {r2, r3, r4, r5}   \n\t"
                   "stmia r0!, {r2, r3, r4, r5}   \n\t"
                   "sub r1, #1                    \n\t"
                   "bne 1b                        \n\t"
                   "pop {r4, r5}                  \n\t"
                   "bx lr                         \n\t");
}

// The 256 bytes at to set to those at from, 16 at a load and a store.
__attribute__((naked, noinline, section(".unit_ram.text.copy_page"))) static void copy_page(uint32_t *to,
                                                                                          const uint32_t *from)
{
    __asm volatile("push {r4, r5, r6}             \n\t"
                   "mov r2, #8                    \n\t"
                   "1:                            \n\t"
                   "ldmia r1!, {r3, r4, r5, r6}   \n\t"
                   "stmia r0!, {r3, r4, r5, r6}   \n\t"
                   "ldmia r1!, {r3, r4, r5, r6}   \n\t"
                   "stmia r0!, {r3, r4, r5, r6}   \n\t"
                   "sub r2, #1                    \n\t"
                   "bne 1b                        \n\t"
                   "pop {r4, r5, r6}              \n\t"
                   "bx lr                         \n\t");
}

static __force_inline uint32_t *live_page()
{
    return (uint32_t *)(unit.rom + unit.window);
}

static __force_inline void fill(uint32_t value)
{
    fill_page(live_page(), value * 0x01010101u);
}

// The page as written, the unit waiting for a pick.
static __force_inline void as_written()
{
    copy_page(live_page(), shadow);
    unit.phase = PICK_DUE;
}

static __force_inline void pick(uint32_t id)
{
    if (id >= at(MARK + 6))
        return; // no map: still waiting for a pick
    uint32_t entry = (at(MARK + 7) << 8 | at(MARK + 8)) + 3 * id;
    unit.map = at(entry) << 8 | at(entry + 1);
    unit.axes = at(entry + 2);
    unit.sent = 0;
    unit.phase = INPUT_DUE;
    fill(unit.axes > 1 ? at(unit.map + 9) : 0); // the second input's address, or none
}

// The input sent; true when it was the last, and the page holds the map's value.
static __force_inline bool send(uint32_t input)
{
    unit.inputs[unit.sent++] = input;
    if (unit.sent < unit.axes)
    {
        fill(0); // no input after the next
        return false;
    }
    fill(map_unit_lookup(unit.map, unit.axes, unit.inputs[0], unit.inputs[1]));
    unit.phase = VALUE_DUE;
    unit.lookups++;
    return true;
}

// Each core's own SysTick, on the processor's clock: the unit's times, in cycles.
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

static __force_inline void note(Cycles *cycles, uint32_t taken)
{
    cycles->count++;
    cycles->sum += taken;
    if (taken > cycles->most)
        cycles->most = taken;
}

// Above USB and the timers (priority 0): a lookup waits for nothing but the one before it.
static void UNIT_CODE(map_unit_irq)()
{
    while (!(MAP_UNIT_PIO->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + MAP_UNIT_SM))))
    {
        uint32_t start = systick_hw->cvr;
        uint32_t read = MAP_UNIT_PIO->rxf[MAP_UNIT_SM];
        Cycles *cycles = nullptr;
        if (read == STARTED)
        {
            unit.starts++;
            if (unit.phase != PICK_DUE)
            {
                as_written();
                cycles = &unit.restores;
            }
        }
        else if (unit.phase == PICK_DUE)
        {
            pick(read);
            cycles = &unit.answers;
        }
        else if (unit.phase == INPUT_DUE)
            cycles = send(read) ? &unit.values : &unit.answers;
        else
        {
            as_written(); // the value read
            cycles = &unit.restores;
        }
        unit.served++;
        if (cycles != nullptr)
            note(cycles, since(start));
    }
}

static void window_start()
{
    PIO p = MAP_UNIT_PIO;
    uint sm = MAP_UNIT_SM;
    pio_sm_set_enabled(p, sm, false);
    pio_sm_clear_fifos(p, sm);
    pio_sm_restart(p, sm);
    pio_sm_exec(p, sm, pio_encode_jmp(prg_map_window.offset));
    pio_sm_exec(p, sm, pio_encode_set(pio_y, unit.window >> 8)); // 1 to 1Fh: set's 5 bits
    pio_set_irq0_source_enabled(p, (pio_interrupt_source_t)(pis_sm0_rx_fifo_not_empty + sm), true);
    irq_set_enabled(PIO1_IRQ_0, true);
    pio_sm_set_enabled(p, sm, true);
}

static void window_stop()
{
    PIO p = MAP_UNIT_PIO;
    uint sm = MAP_UNIT_SM;
    irq_set_enabled(PIO1_IRQ_0, false);
    pio_set_irq0_source_enabled(p, (pio_interrupt_source_t)(pis_sm0_rx_fifo_not_empty + sm), false);
    pio_sm_set_enabled(p, sm, false);
    pio_sm_clear_fifos(p, sm);
}

// The window's page from the mark (1 to 1Fh), or 0: the image unmarked, or the board serving more
// than the 8051's 8 KiB (in a DME, A13 high would then take the 8051 to the unit's memory).
static uint32_t marked_page()
{
    const uint8_t *m = unit.rom + MARK;
    if (unit.mask != EIGHT_KIB || m[0] != 'M' || m[1] != 'A' || m[2] != 'P' || m[3] != 'U' || m[4] != 2)
        return 0;
    return m[5] < 0x20 ? m[5] : 0;
}

static void turn_on(uint32_t page)
{
    unit.window = page << 8;
    memcpy(shadow, unit.rom + unit.window, PAGE);
    unit.phase = PICK_DUE;
    unit.on = true;
    window_start();
}

static void turn_off()
{
    window_stop();
    unit.on = false;
    memcpy(unit.rom + unit.window, shadow, PAGE);
}

// After the image or the mask changed: on with the mark and the 8 KiB served, off without.
static void refresh()
{
    uint32_t page = marked_page();
    if (unit.on && page != unit.window >> 8)
        turn_off();
    if (page != 0 && !unit.on)
        turn_on(page);
}

// The main stack below this call painted, so map_stack can say how much of it was never used.
static constexpr uint32_t PAINT = 0x5A5A5A5A;

static void paint_stack()
{
    uint32_t *sp;
    __asm volatile("mov %0, sp" : "=r"(sp));
    for (uint32_t *p = __StackBottom; p < sp - 16; p++)
        *p = PAINT;
}

static uint32_t stack_unused()
{
    uint32_t bytes = 0;
    for (uint32_t *p = __StackBottom; *p == PAINT && bytes < PICO_STACK_SIZE; p++)
        bytes += 4;
    return bytes;
}

void map_unit_init(uint8_t *rom, uint32_t addr_mask)
{
    paint_stack();
    memcpy(__unit_ram_start__, __unit_ram_source__, __unit_ram_end__ - __unit_ram_start__);
    for (uint32_t d = 1; d < 256; d++)
        reciprocal[d] = (0x10000 + d - 1) / d;
    unit.rom = rom;
    unit.mask = addr_mask & 0xFFFF;
    systick_start(); // core 0's, for map_cycles
    if (!prg_map_window.valid())
        return;
    PRG_LOCAL(prg_map_window, p, sm, offset, cfg);
    sm_config_set_in_pins(&cfg, BASE_OE_PIN);
    pio_sm_init(p, sm, offset, &cfg);
    irq_set_exclusive_handler(PIO1_IRQ_0, map_unit_irq);
    irq_set_priority(PIO1_IRQ_0, 0);
    refresh();
}

void map_unit_set_mask(uint32_t addr_mask)
{
    unit.mask = addr_mask & 0xFFFF;
    refresh();
}

// While the unit is on, the PC's bytes for its page go to the shadow; the 8051 sees them once the page
// is as written again (after the next lookup, or the 8051's start).
void map_unit_write(uint32_t offset, const uint8_t *data, uint32_t size)
{
    uint8_t *page = (uint8_t *)shadow;
    for (uint32_t i = 0; i < size; i++)
    {
        uint32_t address = offset + i;
        if (unit.on && address - unit.window < PAGE)
            page[address - unit.window] = data[i];
        else
            unit.rom[address] = data[i];
    }
    refresh();
}

void map_unit_read(uint32_t offset, uint8_t *into, uint32_t size)
{
    const uint8_t *page = (const uint8_t *)shadow;
    for (uint32_t i = 0; i < size; i++)
    {
        uint32_t address = offset + i;
        into[i] = unit.on && address - unit.window < PAGE ? page[address - unit.window] : unit.rom[address];
    }
}

void map_unit_hide()
{
    if (!unit.on)
        return;
    window_stop();
    memcpy(unit.rom + unit.window, shadow, PAGE);
}

void map_unit_show()
{
    if (!unit.on)
        return;
    unit.on = false;
    turn_on(unit.window >> 8);
}

// ---------------------------------------------------------------------------------------------------
// The benches. map_bench: every input of a map looked up, each timed, with core 1 serving the 8051's
// ROM as in the car. map_bus: core 1 plays the 8051 instead, reading the window as coproc_maps'
// routine does (through the GPIO input overrides: no pin is driven, and the data buffer is held off),
// while core 0 answers and serves USB as it would in the car. The 8051's part runs from flash (it
// fits the XIP cache), so it doesn't hold up the unit's bank as no 8051 would; the ROM's bank goes
// unread meanwhile, though core 1 reads it every 7 cycles in the car. Times are in cycles of the
// system clock.

static constexpr uint32_t CYCLES_PER_US = SYS_CLK_HZ / 1000000;
static constexpr uint32_t MACHINE_CYCLE = 2 * CYCLES_PER_US;            // the 8051's: 12 clocks at 6 MHz
static constexpr uint32_t READ_CYCLES = 465 * CYCLES_PER_US / 1000;     // /PSEN low: 3 TCLCL - 35 ns
static constexpr uint32_t GAP = 4 * MACHINE_CYCLE;                      // a read to the next, at least
static constexpr uint32_t PAUSE = 10 * MACHINE_CYCLE;                   // from a lookup to the next
static constexpr uint32_t LOST = 50 * CYCLES_PER_US;                    // no answer by then: lost
static constexpr uint32_t INOVER_LOW = 2u << IO_BANK0_GPIO0_CTRL_INOVER_LSB;
static constexpr uint32_t INOVER_HIGH = 3u << IO_BANK0_GPIO0_CTRL_INOVER_LSB;

enum : uint32_t
{
    BUS_IDLE,
    BUS_RUNNING,
    BUS_DONE, // core 1 has finished; core 0 gives the board back
    BUS_ENDED
};

enum : uint32_t
{
    ANSWER, // a pick's or a send's before the last: the next read GAP after
    VALUE,  // the last send's: the lookup, the next read GAP after
    RESTORE // the value's read, or a start: the page as written, the next lookup PAUSE after
};

struct Times
{
    uint32_t count, sum, most; // from the end of the read to the unit's answer
};

struct Bus
{
    volatile uint32_t state;
    uint32_t want;
    volatile uint32_t done;
    uint32_t wrong, late, lost;
    Times times[3]; // by kind
};

static Bus bus UNIT_DATA = {};
static uint32_t bus_stack[128] UNIT_DATA = {};
static uint32_t gpio_ctrl[N_ADDR_PINS + N_OE_PINS] UNIT_DATA = {};  // the pins' CTRL, no input override
static uint32_t gpio_saved[N_ADDR_PINS + N_OE_PINS] UNIT_DATA = {}; // as they were (addr_mask's overrides)
static char bench_text[40] UNIT_DATA = {};
static char lookup_text[40] UNIT_DATA = {};

static __force_inline uint32_t crc32_byte(uint32_t crc, uint32_t byte)
{
    crc ^= byte;
    for (uint32_t k = 0; k < 8; k++)
        crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1)));
    return crc;
}

// Every input of the map, the second inner on two axes: the values' CRC-32, as DME-Sim makes it from
// MapUnit.Lookup, and each lookup's cycles with the interrupts held off, less the clock's own reads.
static void UNIT_CODE(bench_map)(uint32_t map, uint32_t axes, uint32_t *crc, uint32_t *least, uint32_t *sum,
                                 uint32_t *most)
{
    systick_start();
    uint32_t t = systick_hw->cvr, empty = since(t);
    *crc = 0xFFFFFFFF;
    *least = 0xFFFFFFFF;
    *sum = 0;
    *most = 0;
    for (uint32_t first = 0; first < 256; first++)
        for (uint32_t second = 0; second < (axes > 1 ? 256u : 1u); second++)
        {
            uint32_t ints = save_and_disable_interrupts();
            uint32_t start = systick_hw->cvr;
            uint32_t value = map_unit_lookup(map, axes, first, second);
            uint32_t cycles = since(start) - empty;
            restore_interrupts(ints);
            *crc = crc32_byte(*crc, value);
            *sum += cycles;
            if (cycles < *least)
                *least = cycles;
            if (cycles > *most)
                *most = cycles;
        }
    *crc = ~*crc;
}

static __force_inline void bus_pin(uint32_t pin, uint32_t ctrl, uint32_t over)
{
    io_bank0_hw->io[pin].ctrl = ctrl | over;
}

// The bench's bus at rest: /CE low throughout, as the DME grounds it (951digit.pdf), and /OE (the
// 8051's /PSEN) high.
static void bus_rest()
{
    for (uint32_t i = 0; i < N_ADDR_PINS + N_OE_PINS; i++)
    {
        uint32_t pin = i < N_ADDR_PINS ? BASE_ADDR_PIN + i : BASE_OE_PIN + i - N_ADDR_PINS;
        gpio_saved[i] = io_bank0_hw->io[pin].ctrl;
        gpio_ctrl[i] = gpio_saved[i] & ~IO_BANK0_GPIO0_CTRL_INOVER_BITS;
    }
    bus_pin(BASE_OE_PIN + 1, gpio_ctrl[N_ADDR_PINS + 1], INOVER_LOW);
    bus_pin(BASE_OE_PIN, gpio_ctrl[N_ADDR_PINS], INOVER_HIGH);
}

// The pins as they were, and the data buffer back under the output enable program.
static void bus_restore()
{
    for (uint32_t i = 0; i < N_ADDR_PINS + N_OE_PINS; i++)
    {
        uint32_t pin = i < N_ADDR_PINS ? BASE_ADDR_PIN + i : BASE_OE_PIN + i - N_ADDR_PINS;
        io_bank0_hw->io[pin].ctrl = gpio_saved[i];
    }
    gpio_set_outover(BUF_OE_PIN, GPIO_OVERRIDE_NORMAL);
}

// One read as the 8051 makes it: the address, then /PSEN (the EPROM's /OE) low for 465 ns. The byte
// is taken from the image as the read ends, as the 8051 latches it.
static uint32_t bus_read(uint32_t address, uint32_t *value, uint32_t *end)
{
    for (uint32_t i = 0; i < N_ADDR_PINS; i++)
        bus_pin(BASE_ADDR_PIN + i, gpio_ctrl[i], (address >> i) & 1 ? INOVER_HIGH : INOVER_LOW);
    uint32_t start = systick_hw->cvr;
    bus_pin(BASE_OE_PIN, gpio_ctrl[N_ADDR_PINS], INOVER_LOW);
    while (since(start) < READ_CYCLES)
    {
    }
    *value = unit.rom[address & unit.mask];
    bus_pin(BASE_OE_PIN, gpio_ctrl[N_ADDR_PINS], INOVER_HIGH);
    *end = systick_hw->cvr;
    return start;
}

// A read wait after the one that started at after: checked against what the 8051 should read, then
// the unit's answer timed from the read's end. Returns when the read started.
static uint32_t bus_step(uint32_t after, uint32_t wait, uint32_t address, uint32_t expect, uint32_t kind)
{
    while (since(after) < wait)
    {
    }
    uint32_t served = unit.served, value, end;
    uint32_t start = bus_read(address, &value, &end);
    if (value != expect)
        bus.wrong++;
    while (unit.served == served && since(end) < LOST)
        __asm volatile("nop\n nop\n nop\n nop\n nop\n nop"); // gently: served is in the unit's bank
    uint32_t cycles = since(end);
    if (unit.served == served)
    {
        bus.lost++;
        return start;
    }
    if (cycles > (kind == RESTORE ? PAUSE : GAP) - READ_CYCLES)
        bus.late++; // after the next read would have begun
    Times *times = &bus.times[kind];
    times->count++;
    times->sum += cycles;
    if (cycles > times->most)
        times->most = cycles;
    return start;
}

// Core 1: each map in turn at random inputs, as coproc_maps' Map_Lookup reads them. One lookup in 64
// is broken off after its pick (and, one time in two, an input) by a read of 0000h, as the 8051's
// start after a reset would; the lookup after it must be right.
static void bus_run()
{
    systick_start();
    uint32_t count = at(MARK + 6), index = at(MARK + 7) << 8 | at(MARK + 8), window = unit.window;
    uint32_t seed = 0x2545F491, id = 0, t = systick_hw->cvr;
    for (uint32_t n = 0; n < bus.want; n++)
    {
        uint32_t entry = index + 3 * id;
        uint32_t map = at(entry) << 8 | at(entry + 1), axes = at(entry + 2);
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        uint32_t first = seed & 0xFF, second = seed >> 8 & 0xFF;
        uint32_t next = axes > 1 ? at(map + 9) : 0; // the answer to the first input
        if ((n & 63) == 63)
        {
            t = bus_step(t, PAUSE, window + id, at(map), ANSWER);
            if (seed & 0x10000)
                t = bus_step(t, GAP, window + (second ^ 0x5A), next, axes > 1 ? ANSWER : VALUE);
            t = bus_step(t, GAP, 0, at(0), RESTORE);
        }
        uint32_t value = map_unit_lookup(map, axes, first, second);
        t = bus_step(t, PAUSE, window + id, at(map), ANSWER);
        t = bus_step(t, GAP, window + first, next, axes > 1 ? ANSWER : VALUE);
        if (axes > 1)
            t = bus_step(t, GAP, window + second, 0, VALUE);
        t = bus_step(t, GAP, window, value, RESTORE);
        bus.done = n + 1;
        if (++id == count)
            id = 0;
    }
    bus.state = BUS_DONE;
    while (true)
        __wfe();
}

static bool bus_start(uint32_t want)
{
    if (!unit.on || bus.state == BUS_RUNNING || bus.state == BUS_DONE || want == 0)
        return false;
    memset(&bus, 0, sizeof(bus));
    bus.want = want;
    rom_service_stop();
    gpio_set_outover(BUF_OE_PIN, GPIO_OVERRIDE_HIGH); // the socket's data buffer off: nothing driven
    bus_rest();
    bus.state = BUS_RUNNING;
    multicore_launch_core1_with_stack(bus_run, bus_stack, sizeof(bus_stack));
    return true;
}

void map_unit_update()
{
    if (bus.state != BUS_DONE)
        return;
    multicore_reset_core1();
    bus_restore();
    rom_service_start();
    bus.state = BUS_ENDED;
}

// map_probe: one read made from core 0, as the bus bench makes them: what the pins read during it,
// where map_window was during and after it, and the reads the unit answered.
static char probe_text[40] UNIT_DATA = {};

static void probe(uint32_t address)
{
    gpio_set_outover(BUF_OE_PIN, GPIO_OVERRIDE_HIGH);
    bus_rest();
    busy_wait_us_32(2); // map_window back at its start, whatever the pins said before
    uint32_t served = unit.served, before = pio_sm_get_pc(MAP_UNIT_PIO, MAP_UNIT_SM);
    for (uint32_t i = 0; i < N_ADDR_PINS; i++)
        bus_pin(BASE_ADDR_PIN + i, gpio_ctrl[i], (address >> i) & 1 ? INOVER_HIGH : INOVER_LOW);
    bus_pin(BASE_OE_PIN, gpio_ctrl[N_ADDR_PINS], INOVER_LOW);
    busy_wait_us_32(2);
    uint32_t pins = sio_hw->gpio_in, during = pio_sm_get_pc(MAP_UNIT_PIO, MAP_UNIT_SM);
    bus_pin(BASE_OE_PIN, gpio_ctrl[N_ADDR_PINS], INOVER_HIGH);
    busy_wait_us_32(20);
    uint32_t after = pio_sm_get_pc(MAP_UNIT_PIO, MAP_UNIT_SM);
    bus_restore();
    snprintf(probe_text, sizeof(probe_text), "pins %08x pc %u %u %u served %u", (unsigned)pins, (unsigned)before,
             (unsigned)during, (unsigned)after, (unsigned)(unit.served - served));
}

// ---------------------------------------------------------------------------------------------------
// Parameters: map_unit (its state), map_counts (lookups made and starts seen), map_lookup
// (id,first[,second]: one lookup, timed), map_bench (id: every input of the map), map_bus (n: n
// lookups through the window, core 1 the 8051; its answer comes as it runs), map_bus_cycles (their
// times by kind, mean and most: cycles from the read's end), map_cycles (the interrupt's own by kind,
// from its start, for every read answered since start, the 8051's in a car's included) and map_stack
// (how much of the main stack has never been used since start). The kinds: a pick or a send before
// the last; the last send (the lookup); the page as written again (the value's read, or a start). An
// answer is 35 characters at most (a packet's payload).

// Up to three numbers, separated by commas.
static uint32_t numbers(const char *text, uint32_t *values)
{
    uint32_t n = 0;
    while (n < 3)
    {
        values[n++] = strtoul(text);
        text = strchr(text, ',');
        if (text == nullptr)
            break;
        text++;
    }
    return n;
}

static bool map_of(uint32_t id, uint32_t *map, uint32_t *axes)
{
    if (!unit.on || id >= at(MARK + 6))
        return false;
    uint32_t entry = (at(MARK + 7) << 8 | at(MARK + 8)) + 3 * id;
    *map = at(entry) << 8 | at(entry + 1);
    *axes = at(entry + 2);
    return true;
}

static uint32_t mean(uint64_t sum, uint32_t count)
{
    return count ? (uint32_t)(sum / count) : 0;
}

bool map_unit_set_parameter(const char *name, const char *value)
{
    uint32_t args[3] = {}, map, axes;
    uint32_t n = numbers(value, args);
    if (streq(name, "map_lookup"))
    {
        if (n < 2 || !map_of(args[0], &map, &axes))
            return false;
        systick_start();
        uint32_t ints = save_and_disable_interrupts();
        uint32_t start = systick_hw->cvr;
        uint32_t result = map_unit_lookup(map, axes, args[1] & 0xFF, args[2] & 0xFF);
        uint32_t cycles = since(start);
        restore_interrupts(ints);
        snprintf(lookup_text, sizeof(lookup_text), "value %u, %u cycles", (unsigned)result, (unsigned)cycles);
        return true;
    }
    if (streq(name, "map_bench"))
    {
        if (!map_of(args[0], &map, &axes))
            return false;
        uint32_t crc, least, sum, most;
        bench_map(map, axes, &crc, &least, &sum, &most);
        uint32_t count = axes > 1 ? 65536 : 256;
        snprintf(bench_text, sizeof(bench_text), "crc %08x cycles %u %u %u", (unsigned)crc, (unsigned)least,
                 (unsigned)((sum + count / 2) / count), (unsigned)most);
        return true;
    }
    if (streq(name, "map_bus"))
        return bus_start(args[0]);
    if (streq(name, "map_probe"))
    {
        if (!unit.on || bus.state == BUS_RUNNING || bus.state == BUS_DONE)
            return false;
        probe(args[0]);
        return true;
    }
    return false;
}

bool map_unit_get_parameter(const char *name, char *value, size_t value_size)
{
    if (streq(name, "map_unit"))
    {
        if (unit.on)
            snprintf(value, value_size, "%u maps, window %04x", (unsigned)at(MARK + 6), (unsigned)unit.window);
        else
            strcpyz(value, value_size, "off");
        return true;
    }
    if (streq(name, "map_counts"))
    {
        snprintf(value, value_size, "%u lookups %u starts", (unsigned)unit.lookups, (unsigned)unit.starts);
        return true;
    }
    if (streq(name, "map_lookup"))
    {
        strcpyz(value, value_size, lookup_text[0] ? lookup_text : "none");
        return true;
    }
    if (streq(name, "map_bench"))
    {
        strcpyz(value, value_size, bench_text[0] ? bench_text : "none");
        return true;
    }
    if (streq(name, "map_bus"))
    {
        if (bus.state == BUS_IDLE)
            strcpyz(value, value_size, "none");
        else if (bus.state != BUS_ENDED)
            snprintf(value, value_size, "running %u of %u", (unsigned)bus.done, (unsigned)bus.want);
        else
            snprintf(value, value_size, "%u done %u wrong %u late %u lost", (unsigned)bus.done, (unsigned)bus.wrong,
                     (unsigned)bus.late, (unsigned)bus.lost);
        return true;
    }
    if (streq(name, "map_probe"))
    {
        strcpyz(value, value_size, probe_text[0] ? probe_text : "none");
        return true;
    }
    if (streq(name, "map_cycles"))
    {
        uint32_t ints = save_and_disable_interrupts(); // the interrupt's sums, whole
        Cycles answers = unit.answers, values = unit.values, restores = unit.restores;
        restore_interrupts(ints);
        snprintf(value, value_size, "%u %u %u %u %u %u", (unsigned)mean(answers.sum, answers.count),
                 (unsigned)answers.most, (unsigned)mean(values.sum, values.count), (unsigned)values.most,
                 (unsigned)mean(restores.sum, restores.count), (unsigned)restores.most);
        return true;
    }
    if (streq(name, "map_stack"))
    {
        snprintf(value, value_size, "%u of %u bytes never used", (unsigned)stack_unused(), (unsigned)PICO_STACK_SIZE);
        return true;
    }
    if (streq(name, "map_bus_cycles"))
    {
        if (bus.state != BUS_ENDED)
        {
            strcpyz(value, value_size, "none");
            return true;
        }
        const Times *t = bus.times;
        snprintf(value, value_size, "%u %u %u %u %u %u", (unsigned)mean(t[0].sum, t[0].count), (unsigned)t[0].most,
                 (unsigned)mean(t[1].sum, t[1].count), (unsigned)t[1].most, (unsigned)mean(t[2].sum, t[2].count),
                 (unsigned)t[2].most);
        return true;
    }
    return false;
}

#endif // FEATURE_MAP_UNIT
