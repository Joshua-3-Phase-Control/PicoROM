// The map unit (FEATURE_MAP_UNIT): the board looks maps up for the 8051, as DME-Sim's MapUnit models
// it (experiments/coproc_maps there; docs/map_unit.md here). With the board's A13 wire the 8051 reads
// 3C00h-3FFFh of the image, and since the socket has no write line it asks by the addresses it reads:
//
//   3C00h + id   picks map id; the byte read is the RAM address of its first input
//   3D00h + v    sends v, the map's next input; the byte read is the next input's address, or 0
//   3E00h        the map's value at the inputs sent: rom_lean's Map_Lookup and Map_Lookup_2D, bit
//                for bit (format 1)
//   3F00h        the mark: "MAPU", the format, the number of maps, the index's address (high, low);
//                the index: each map's address (high, low) and its axes
//
// map_window (map_unit.pio) pushes A0-A9 of each read of that 1 KiB once the read is over, and the
// interrupt here writes the answer into the image core 1 serves: the select page holds every map's
// answer from the start, a pick or a send fills the input page with the next answer, the last send
// fills the value page. The 8051's routine reads each answer 8 us or more after it asks (map_bus
// times the board). The PC reads and writes the image as it wrote it: the three pages are kept for it
// apart (the shadow), and go to flash as written.

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

static constexpr uint32_t SELECT = 0x3C00, INPUT = 0x3D00, RESULT = 0x3E00, MARK = 0x3F00;
static constexpr uint32_t PAGES = MARK - SELECT; // the three the unit answers in
static constexpr uint32_t WINDOW = SELECT >> 10; // map_window's y: A10 and up

// Cycles the interrupt took to answer, from its start (map_cycles).
struct Cycles
{
    uint32_t count, most;
    uint64_t sum;
};

struct Unit
{
    uint8_t *rom;
    uint32_t mask;      // the address mask the board serves through, within the 8051's 64 KiB
    volatile bool on;   // the image has the mark, and the 8051 reaches it
    int32_t selected;   // the map picked, or -1
    uint32_t map, axes, sent;
    uint8_t inputs[2];
    uint32_t input_value; // the byte the whole input page holds
    volatile uint32_t lookups;
    volatile uint32_t served; // reads of the window answered (the bus bench waits on it)
    Cycles answers;           // a pick's, or a send's before the last
    Cycles values;            // the last send's: the lookup and the value page
};

static Unit unit UNIT_DATA = {};
static uint8_t shadow[PAGES] UNIT_DATA = {};     // the image's bytes under the pages, as the PC wrote them
static uint32_t reciprocal[256] UNIT_DATA = {}; // ceil(2^16 / d): x / d = x * it >> 16, exact below 256

// ---------------------------------------------------------------------------------------------------
// The lookup, format 1: as MapUnit.Lookup and rom_lean's routines make it, reading the maps as the
// 8051 does, through the address mask. An axis is 9 bytes: its input's RAM address, an offset, then
// each of two segments' span, step and factor, and the index where the second starts. A one-axis
// map's values follow its axis; a two-axis map's second axis follows the first, then its column
// count, then the values a row at a time.

static __force_inline uint32_t at(uint32_t address)
{
    return unit.rom[address & unit.mask];
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
// The window: the interrupt and the pages.

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

static __force_inline void fill(uint32_t page, uint32_t value)
{
    fill_page((uint32_t *)(unit.rom + page), value * 0x01010101u);
}

static __force_inline void set_input(uint32_t value)
{
    if (unit.input_value == value)
        return;
    fill(INPUT, value);
    unit.input_value = value;
}

static __force_inline void pick(uint32_t id)
{
    unit.selected = -1;
    if (id >= at(MARK + 5))
        return;
    uint32_t entry = (at(MARK + 6) << 8 | at(MARK + 7)) + 3 * id;
    unit.map = at(entry) << 8 | at(entry + 1);
    unit.axes = at(entry + 2);
    unit.sent = 0;
    unit.selected = id;
    set_input(unit.axes > 1 ? at(unit.map + 9) : 0); // the second input's address, or none
}

// The input sent; true when it was the last, and the value page holds the map's value.
static __force_inline bool send(uint32_t input)
{
    if (unit.selected < 0 || unit.sent >= unit.axes)
        return false;
    unit.inputs[unit.sent++] = input;
    bool last = unit.sent >= unit.axes;
    if (last)
    {
        fill(RESULT, map_unit_lookup(unit.map, unit.axes, unit.inputs[0], unit.inputs[1]));
        unit.lookups++;
    }
    set_input(0); // no input after this one
    return last;
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
        uint32_t page = read >> 8;
        Cycles *cycles = nullptr;
        if (page == 0)
        {
            pick(read & 0xFF);
            cycles = &unit.answers;
        }
        else if (page == 1)
            cycles = send(read & 0xFF) ? &unit.values : &unit.answers;
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
    pio_sm_exec(p, sm, pio_encode_set(pio_y, WINDOW));
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

// The select page: each map's first byte, its first input's address. The rest keep the image's.
static void fill_select()
{
    uint32_t count = at(MARK + 5), index = at(MARK + 6) << 8 | at(MARK + 7);
    for (uint32_t id = 0; id < count; id++)
    {
        uint32_t entry = index + 3 * id;
        unit.rom[SELECT + id] = at(at(entry) << 8 | at(entry + 1));
    }
}

static bool marked()
{
    const uint8_t *m = unit.rom + MARK;
    return (unit.mask & 0x3FFF) == 0x3FFF && m[0] == 'M' && m[1] == 'A' && m[2] == 'P' && m[3] == 'U' && m[4] == 1;
}

static void turn_on()
{
    fill(INPUT, 0);
    fill(RESULT, 0);
    unit.input_value = 0;
    unit.selected = -1;
    fill_select();
    unit.on = true;
    window_start();
}

static void turn_off()
{
    window_stop();
    unit.on = false;
    memcpy(unit.rom + SELECT, shadow, PAGES);
}

// After the image or the mask changed: on with the mark (and A13), off without.
static void refresh()
{
    bool marked_now = marked();
    if (marked_now && !unit.on)
        turn_on();
    else if (!marked_now && unit.on)
        turn_off();
    else if (unit.on)
        fill_select();
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
    unit.selected = -1;
    memcpy(shadow, rom + SELECT, PAGES);
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

void map_unit_write(uint32_t offset, const uint8_t *data, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++)
    {
        uint32_t address = offset + i, page = address - SELECT;
        if (page < PAGES)
        {
            shadow[page] = data[i];
            if (unit.on)
                continue; // the unit's answer stays
        }
        unit.rom[address] = data[i];
    }
    refresh();
}

void map_unit_read(uint32_t offset, uint8_t *into, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++)
    {
        uint32_t address = offset + i, page = address - SELECT;
        into[i] = page < PAGES ? shadow[page] : unit.rom[address];
    }
}

void map_unit_hide()
{
    if (!unit.on)
        return;
    window_stop();
    memcpy(unit.rom + SELECT, shadow, PAGES);
}

void map_unit_show()
{
    if (!unit.on)
        return;
    unit.on = false;
    turn_on();
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
    ANSWER, // a pick's or a send's (the input page)
    VALUE,  // the last send's: the lookup (the value page)
    READ    // a read of the value: nothing to do
};

struct Bus
{
    volatile uint32_t state;
    uint32_t want;
    volatile uint32_t done;
    uint32_t wrong, late, lost;
    uint32_t answers, answer_sum, answer_max; // from the end of the read to the unit's answer
    uint32_t values, value_sum, value_max;
};

static Bus bus UNIT_DATA = {};
static uint32_t bus_stack[128] UNIT_DATA = {};
static uint32_t gpio_ctrl[N_ADDR_PINS + N_OE_PINS] UNIT_DATA = {}; // the pins' CTRL, no input override
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
    for (uint32_t i = 0; i < N_ADDR_PINS; i++)
        gpio_ctrl[i] = io_bank0_hw->io[BASE_ADDR_PIN + i].ctrl & ~IO_BANK0_GPIO0_CTRL_INOVER_BITS;
    for (uint32_t i = 0; i < N_OE_PINS; i++)
        gpio_ctrl[N_ADDR_PINS + i] = io_bank0_hw->io[BASE_OE_PIN + i].ctrl & ~IO_BANK0_GPIO0_CTRL_INOVER_BITS;
    bus_pin(BASE_OE_PIN + 1, gpio_ctrl[N_ADDR_PINS + 1], INOVER_LOW);
    bus_pin(BASE_OE_PIN, gpio_ctrl[N_ADDR_PINS], INOVER_HIGH);
}

// The pins as they were, and the data buffer back under the output enable program.
static void bus_restore()
{
    for (uint32_t i = 0; i < N_ADDR_PINS; i++)
        io_bank0_hw->io[BASE_ADDR_PIN + i].ctrl = gpio_ctrl[i];
    for (uint32_t i = 0; i < N_OE_PINS; i++)
        io_bank0_hw->io[BASE_OE_PIN + i].ctrl = gpio_ctrl[N_ADDR_PINS + i];
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

// A read of the window GAP (or PAUSE) after the one that started at after: checked against what the
// 8051 should read, then the unit's answer timed from the read's end. Returns when the read started.
static uint32_t bus_step(uint32_t after, uint32_t wait, uint32_t address, uint32_t expect, uint32_t kind)
{
    while (since(after) < wait)
    {
    }
    uint32_t served = unit.served, value, end;
    uint32_t start = bus_read(address, &value, &end);
    if (value != expect)
        bus.wrong++;
    uint32_t cycles;
    while (unit.served == served && since(end) < LOST)
        __asm volatile("nop\n nop\n nop\n nop\n nop\n nop"); // gently: served is in the unit's bank
    cycles = since(end);
    if (unit.served == served)
        bus.lost++;
    else if (kind != READ)
    {
        if (cycles > GAP - READ_CYCLES)
            bus.late++; // after the next read would have begun
        if (kind == ANSWER)
        {
            bus.answers++;
            bus.answer_sum += cycles;
            if (cycles > bus.answer_max)
                bus.answer_max = cycles;
        }
        else
        {
            bus.values++;
            bus.value_sum += cycles;
            if (cycles > bus.value_max)
                bus.value_max = cycles;
        }
    }
    return start;
}

// Core 1: each map in turn at random inputs, as coproc_maps' Map_Lookup reads them.
static void bus_run()
{
    systick_start();
    uint32_t count = at(MARK + 5), index = at(MARK + 6) << 8 | at(MARK + 7);
    uint32_t seed = 0x2545F491, id = 0, t = systick_hw->cvr;
    for (uint32_t n = 0; n < bus.want; n++)
    {
        uint32_t entry = index + 3 * id;
        uint32_t map = at(entry) << 8 | at(entry + 1), axes = at(entry + 2);
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        uint32_t first = seed & 0xFF, second = seed >> 8 & 0xFF;
        uint32_t value = map_unit_lookup(map, axes, first, second);
        t = bus_step(t, PAUSE, SELECT + id, at(map), ANSWER);
        t = bus_step(t, GAP, INPUT + first, axes > 1 ? at(map + 9) : 0, axes > 1 ? ANSWER : VALUE);
        if (axes > 1)
            t = bus_step(t, GAP, INPUT + second, 0, VALUE);
        t = bus_step(t, GAP, RESULT, value, READ);
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

// map_probe: one read of the window made from core 0, as the bus bench makes them: what the pins
// read during it, where map_window was during and after it, and the reads the unit answered.
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
// Parameters: map_unit (its state), map_lookup (id,first[,second]: one lookup, timed), map_bench (id:
// every input of the map), map_bus (n: n lookups through the window, core 1 the 8051; its answer
// comes as it runs), map_bus_cycles (their times: mean and most, cycles from the read's end),
// map_cycles (the interrupt's own, from its start, for every read answered since start, the 8051's
// in a car's included) and map_stack (how much of the main stack has never been used since start).

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
    if (!unit.on || id >= at(MARK + 5))
        return false;
    uint32_t entry = (at(MARK + 6) << 8 | at(MARK + 7)) + 3 * id;
    *map = at(entry) << 8 | at(entry + 1);
    *axes = at(entry + 2);
    return true;
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
            snprintf(value, value_size, "%u maps, %u lookups", (unsigned)at(MARK + 5), (unsigned)unit.lookups);
        else
            strcpyz(value, value_size, "off");
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
        Cycles answers = unit.answers, values = unit.values;
        restore_interrupts(ints);
        snprintf(value, value_size, "answer %u %u value %u %u",
                 (unsigned)(answers.count ? answers.sum / answers.count : 0), (unsigned)answers.most,
                 (unsigned)(values.count ? values.sum / values.count : 0), (unsigned)values.most);
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
            strcpyz(value, value_size, "none");
        else
            snprintf(value, value_size, "answer %u %u value %u %u",
                     (unsigned)(bus.answers ? bus.answer_sum / bus.answers : 0), (unsigned)bus.answer_max,
                     (unsigned)(bus.values ? bus.value_sum / bus.values : 0), (unsigned)bus.value_max);
        return true;
    }
    return false;
}

#endif // FEATURE_MAP_UNIT
