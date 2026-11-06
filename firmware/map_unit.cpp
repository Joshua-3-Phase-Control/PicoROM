// The map unit (FEATURE_MAP_UNIT): the board looks maps up for the 8051, as DME-Sim's MapUnit models
// it (experiments/coproc_maps there; docs/map_unit.md here), and works out tasks for it (format 3:
// rom_lean's slow work, unit_tasks.cpp; experiments/coproc_slow). The board serves the 8051 the
// image's first 8 KiB (addr_mask 0x1fff: the DME ties the socket's A13 high) and keeps its own memory
// after them, which the 8051 never reads: from 2000h the mark ("MAPU", format 2 or 3, the window's
// page, the number of maps and tasks, the index's address, high byte first), the index (by id each
// map's address and its axes, or a task's description's address and 80h + its number) and the maps
// and descriptions. The window is one page of the 8 KiB, which the mark names. The socket has no
// write line, so the 8051 asks by the addresses it reads there, and the unit fills the page with each
// answer:
//
//   window + id   picks map or task id; the byte read is the page as written: by id, its first
//                 input's RAM address
//   window + v    sends v, the next input; the byte read is the address of the input after it, or 0
//   window + any  after a map's last: its value at the inputs sent, rom_lean's Map_Lookup and
//                 Map_Lookup_2D's bit for bit; then the page is as written again
//   window + any  after a task's last: the RAM address of its first output, then its value, then
//                 the next output's address, and so on; after 0 (no more) the page is as written again
//
// map_window (map_unit.pio) pushes A0-A7 of each read of the page once the read is over (shifted up a
// bit), and of each read of page 0 (with a 1 below): 0000h, where the 8051 starts after a reset, puts
// the page as written again and the unit waiting for a pick; the rest go to the snoop (its vectors).
// The 8051's routine reads each answer 8 us or more after it asks (map_bus times the board). The PC
// reads and writes the image as it wrote it: the page is kept for it apart (the shadow), and goes to
// flash as written.

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

#include "bus_bench.h"
#include "pio_programs.h"
#include "rom.h"
#include "snoop.h"
#include "str_util.h"
#include "system.h"
#include "unit_ram.h"
#include "unit_tasks.h"

extern "C"
{
    extern uint8_t __unit_ram_start__[], __unit_ram_end__[], __unit_ram_source__[];
    extern uint32_t __StackBottom[];
}

static_assert(ROM_SIZE <= 0x30000, "the map unit's bank is the last one");

static constexpr uint32_t MARK = 0x2000;          // the unit's own memory, after the 8051's 8 KiB
static constexpr uint32_t EIGHT_KIB = 0x1FFF;     // the address mask the unit works with
static constexpr uint32_t PAGE = 0x100;
static constexpr uint32_t TASK = 0x80; // an index entry's kind: a task, its number below (format 3)
static constexpr uint32_t MAP = ~0u;   // unit.task: a map picked

enum : uint32_t
{
    PICK_DUE,    // waiting for a pick; the page as written
    INPUT_DUE,   // the map's or the task's inputs due
    VALUE_DUE,   // the page holds the map's value
    ADDRESS_DUE, // the page holds the task's next output's RAM address, or 0: none
    OUTPUT_DUE   // the page holds that output's value
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
    volatile uint32_t starts; // reads of 0000h, the 8051's start
    volatile uint32_t served; // reads answered (the bus bench waits on it)
    Cycles answers;           // a pick's, a send's before the last, a task's output's
    Cycles values;            // the last send's: the lookup or the task, and the page filled after
    Cycles restores;          // the value's read, the end of a task's outputs, or a start: the page
                              //   as written again
    uint32_t task;            // the task picked (its number), or MAP (after the fields DME-Sim reads)
    uint32_t output;          // the task's next output
    volatile uint32_t tasks;  // tasks worked out
};

static Unit unit UNIT_DATA = {};
static uint32_t shadow[PAGE / 4] UNIT_DATA = {}; // the page as the PC wrote it
static uint8_t ram[0x80] UNIT_DATA = {};         // the 8051's RAM bytes a task was sent, and it made

// ceil(2^16 / d): x / d = x * it >> 16, exact below 256. Made as the firmware is built, so the image of
// the unit's bank holds it (DME-Sim runs that image).
struct Reciprocals
{
    uint32_t of[256];
};

static constexpr Reciprocals make_reciprocals()
{
    Reciprocals r = {};
    for (uint32_t d = 1; d < 256; d++)
        r.of[d] = (0x10000 + d - 1) / d;
    return r;
}

static const Reciprocals reciprocals __attribute__((section(".unit_ram.rodata"))) = make_reciprocals();

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
        index = x * reciprocals.of[step] >> 16;
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

// A map's entry in the index, by id.
static __force_inline uint32_t entry_of(uint32_t id)
{
    return (at(MARK + 7) << 8 | at(MARK + 8)) + 3 * id;
}

uint32_t UNIT_CODE(unit_map_value)(uint32_t id, const uint8_t *bytes)
{
    uint32_t entry = entry_of(id), map = at(entry) << 8 | at(entry + 1), axes = at(entry + 2);
    return map_unit_lookup(map, axes, bytes[at(map) & 0x7F], axes > 1 ? bytes[at(map + 9) & 0x7F] : 0);
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

// The RAM address of the picked map's or task's input k (from 1), or 0 past its last. A map's are its
// axes' first bytes; a task's description lists them after their count.
static __force_inline uint32_t input_address(uint32_t k)
{
    if (k > unit.axes)
        return 0;
    return unit.task == MAP ? at(unit.map + 9 * (k - 1)) : at(unit.map + k);
}

// The RAM address of the picked task's output j (from 0), or 0 past its last: the description lists
// them, after their count, after the inputs.
static __force_inline uint32_t output_address(uint32_t j)
{
    uint32_t outputs = unit.map + 1 + unit.axes;
    return j < at(outputs) ? at(outputs + 1 + j) : 0;
}

static __force_inline void pick(uint32_t id)
{
    if (id >= at(MARK + 6))
        return; // nothing by that id: still waiting for a pick
    uint32_t entry = entry_of(id), kind = at(entry + 2);
    unit.map = at(entry) << 8 | at(entry + 1);
    unit.task = kind & TASK ? kind & ~TASK : MAP;
    unit.axes = unit.task == MAP ? kind : at(unit.map); // the inputs
    unit.sent = 0;
    unit.phase = INPUT_DUE;
    fill(input_address(2)); // the second input's address, or none
}

// The input sent; true when it was the last, and the page holds the map's value or the task's first
// output's address.
static __force_inline bool send(uint32_t input)
{
    if (unit.task == MAP)
        unit.inputs[unit.sent] = input;
    else
        ram[input_address(unit.sent + 1) & 0x7F] = input;
    unit.sent++;
    if (unit.sent < unit.axes)
    {
        fill(input_address(unit.sent + 2)); // the input's after the next, or none
        return false;
    }
    if (unit.task == MAP)
    {
        fill(map_unit_lookup(unit.map, unit.axes, unit.inputs[0], unit.inputs[1]));
        unit.phase = VALUE_DUE;
        unit.lookups++;
        return true;
    }
    unit_task(unit.task, unit.map, unit.rom, ram);
    unit.tasks++;
    unit.output = 0;
    unit.phase = ADDRESS_DUE;
    fill(output_address(0));
    return true;
}

// A read of a task's output's address (0: the end) or of its value: the page holds what comes next.
static __force_inline Cycles *output()
{
    if (unit.phase == OUTPUT_DUE)
    {
        unit.output++;
        unit.phase = ADDRESS_DUE;
        fill(output_address(unit.output));
        return &unit.answers;
    }
    uint32_t address = output_address(unit.output);
    if (address == 0)
    {
        as_written();
        return &unit.restores;
    }
    unit.phase = OUTPUT_DUE;
    fill(ram[address & 0x7F]);
    return &unit.answers;
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
        uint32_t word = MAP_UNIT_PIO->rxf[MAP_UNIT_SM], read = word >> 1;
        Cycles *cycles = nullptr;
        if (word & 1) // page 0
        {
            snoop_page_read(read);
            if (read != 0)
                continue; // the snoop's alone
            unit.starts++;
            if (unit.phase != PICK_DUE)
            {
                as_written();
                cycles = &unit.restores;
            }
        }
        else if (!unit.on)
            continue; // a window's read pushed before the unit went off
        else if (unit.phase == PICK_DUE)
        {
            pick(read);
            cycles = &unit.answers;
        }
        else if (unit.phase == INPUT_DUE)
            cycles = send(read) ? &unit.values : &unit.answers;
        else if (unit.phase != VALUE_DUE)
            cycles = output();
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

// map_window runs from init, so page 0's reads reach the snoop whether the unit is on or not; the
// window's are pushed while y holds its page (1 to 1Fh: set's 5 bits), none while it holds ~0.
static void window_start()
{
    pio_sm_exec(MAP_UNIT_PIO, MAP_UNIT_SM, pio_encode_set(pio_y, unit.window >> 8));
}

// The page as written first, the interrupt held off meanwhile (a window's read pushed before is
// dropped once the unit is off).
static void window_stop()
{
    irq_set_enabled(PIO1_IRQ_0, false);
    pio_sm_exec(MAP_UNIT_PIO, MAP_UNIT_SM, pio_encode_mov_not(pio_y, pio_null));
    memcpy(unit.rom + unit.window, shadow, PAGE);
    irq_set_enabled(PIO1_IRQ_0, true);
}

// The window's page from the mark (1 to 1Fh), or 0: the image unmarked, or the board serving more
// than the 8051's 8 KiB (in a DME, A13 high would then take the 8051 to the unit's memory).
static uint32_t UNIT_CODE(marked_page)()
{
    const uint8_t *m = unit.rom + MARK;
    if (unit.mask != EIGHT_KIB || m[0] != 'M' || m[1] != 'A' || m[2] != 'P' || m[3] != 'U' || m[4] < 2 || m[4] > 3)
        return 0;
    return m[5] < 0x20 ? m[5] : 0;
}

// The unit on at the page, waiting for a pick: its state alone (turn_on starts the window's PIO).
static void UNIT_CODE(state_on)(uint32_t page)
{
    unit.window = page << 8;
    copy_page(shadow, (const uint32_t *)(unit.rom + unit.window));
    unit.phase = PICK_DUE;
    unit.on = true;
}

static void turn_on(uint32_t page)
{
    state_on(page);
    window_start();
}

static void turn_off()
{
    window_stop();
    unit.on = false;
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

// The unit's state as the board starts it: on, waiting for a pick, if the image has the mark and the
// board serves the 8051 its 8 KiB. It touches no hardware, so DME-Sim starts the unit with it too.
void UNIT_CODE(map_unit_start)(uint8_t *rom, uint32_t addr_mask)
{
    unit.rom = rom;
    unit.mask = addr_mask & 0xFFFF;
    uint32_t page = marked_page();
    if (page != 0)
        state_on(page);
}

void map_unit_init(uint8_t *rom, uint32_t addr_mask)
{
    paint_stack();
    memcpy(__unit_ram_start__, __unit_ram_source__, __unit_ram_end__ - __unit_ram_start__);
    unit.rom = rom;
    unit.mask = addr_mask & 0xFFFF;
    systick_start(); // core 0's, for map_cycles
    if (!prg_map_window.valid())
        return;
    PRG_LOCAL(prg_map_window, p, sm, offset, cfg);
    sm_config_set_in_pins(&cfg, BASE_OE_PIN);
    sm_config_set_in_shift(&cfg, false, false, 32); // ISR left: the tag below A0-A7
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_RX);
    pio_sm_init(p, sm, offset, &cfg);
    pio_sm_exec(p, sm, pio_encode_mov_not(pio_y, pio_null)); // no window till the unit is on
    irq_set_exclusive_handler(PIO1_IRQ_0, map_unit_irq);
    irq_set_priority(PIO1_IRQ_0, 0);
    pio_set_irq0_source_enabled(p, (pio_interrupt_source_t)(pis_sm0_rx_fifo_not_empty + sm), true);
    irq_set_enabled(PIO1_IRQ_0, true);
    pio_sm_set_enabled(p, sm, true);
    map_unit_start(rom, addr_mask);
    if (unit.on)
        window_start();
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
    if (unit.on)
        window_stop();
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
// ROM as in the car. map_bus: core 1 plays the 8051 instead (bus_bench.h), reading the window as
// coproc_maps' routine does, while core 0 answers and serves USB as it would in the car; the ROM's
// bank goes unread meanwhile, though core 1 reads it every 7 cycles in the car. Times are in cycles
// of the system clock.

static constexpr uint32_t GAP = 4 * MACHINE_CYCLE;                      // a read to the next, at least
static constexpr uint32_t TASK_GAP = 6 * MACHINE_CYCLE;                 // a task's last input to its first
                                                                        //   output (coproc_slow's Co_Task)
static constexpr uint32_t PAUSE = 10 * MACHINE_CYCLE;                   // from a lookup to the next
static constexpr uint32_t LOST = 50 * CYCLES_PER_US;                    // no answer by then: lost

enum : uint32_t
{
    BUS_IDLE,
    BUS_RUNNING,
    BUS_ENDED // core 1 has finished; bench_update gives the board back
};

enum : uint32_t
{
    ANSWER, // a pick's, a send's before the last, a task's output's
    VALUE,  // the last send's: the lookup or the task
    RESTORE // the value's read, a task's last, or a start: the page as written again
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
static char bench_text[40] UNIT_DATA = {};
static char lookup_text[40] UNIT_DATA = {};
static char task_text[40] UNIT_DATA = {};
static uint8_t bench_ram[0x80] UNIT_DATA = {}; // the 8051's RAM bytes for map_task's runs
static uint8_t bus_ram[0x80] = {};             // core 1's for the bus bench's tasks

static __force_inline uint32_t crc32_byte(uint32_t crc, uint32_t byte)
{
    crc ^= byte;
    for (uint32_t k = 0; k < 8; k++)
        crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1)));
    return crc;
}

static __force_inline uint32_t xorshift(uint32_t seed)
{
    seed ^= seed << 13;
    seed ^= seed >> 17;
    return seed ^ seed << 5;
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

// One of map_task's runs, from the inputs in bench_ram: timed as bench_map times a lookup, its outputs
// into the CRC.
static __force_inline void bench_run(uint32_t task, uint32_t description, uint32_t outputs, uint32_t empty,
                                     uint32_t *crc, uint32_t *least, uint32_t *sum, uint32_t *most)
{
    uint32_t ints = save_and_disable_interrupts();
    uint32_t start = systick_hw->cvr;
    unit_task(task, description, unit.rom, bench_ram);
    uint32_t cycles = since(start) - empty;
    restore_interrupts(ints);
    for (uint32_t j = 0; j < at(outputs); j++)
        *crc = crc32_byte(*crc, bench_ram[at(outputs + 1 + j) & 0x7F]);
    *sum += cycles;
    if (cycles < *least)
        *least = cycles;
    if (cycles > *most)
        *most = cycles;
}

static constexpr uint32_t INPUTS = 8;          // a task's inputs, at most
static constexpr uint32_t CASES = 1u << 18;    // a task's exhaustive cases, at most; beyond, as many at random
static constexpr uint32_t SEED = 0x2545F491;   // the random cases' (UnitTasks.Seed)

// The task at each case of its inputs as DME-Sim's UnitTasks.Cases takes them: every value of each
// input byte, the last fastest, a byte of bits (20h-2Fh) 00h and FFh; then each of its bits alone (the
// other inputs 80h) over the first other input's range. More than CASES of those: CASES cases at
// random instead, each input a byte of xorshift's in turn. The outputs' CRC-32, and the cases.
static void UNIT_CODE(bench_task)(uint32_t task, uint32_t description, uint32_t *crc, uint32_t *least, uint32_t *sum,
                                  uint32_t *most, uint32_t *cases)
{
    uint32_t n = at(description), outputs = description + 1 + n, address[INPUTS], count[INPUTS], place[INPUTS] = {};
    uint32_t flags = INPUTS, other = INPUTS, product = 1;
    if (n > INPUTS)
        n = INPUTS; // (none has more)
    for (uint32_t i = 0; i < n; i++)
    {
        address[i] = at(description + 1 + i) & 0x7F;
        bool bits = address[i] >= 0x20 && address[i] < 0x30;
        count[i] = bits ? 2 : 256;
        if (bits && flags == INPUTS)
            flags = i;
        if (!bits && other == INPUTS)
            other = i;
        if (product <= CASES)
            product *= count[i]; // past CASES it stays past it
    }
    systick_start();
    uint32_t t = systick_hw->cvr, empty = since(t);
    *crc = 0xFFFFFFFF;
    *least = 0xFFFFFFFF;
    *sum = 0;
    *most = 0;
    *cases = 0;
    if (product + (flags < INPUTS && other < INPUTS ? 8 * 256 : 0) > CASES)
    {
        uint32_t seed = SEED;
        for (*cases = 0; *cases < CASES; ++*cases)
        {
            for (uint32_t i = 0; i < n; i++)
            {
                seed = xorshift(seed);
                bench_ram[address[i]] = seed & 0xFF;
            }
            bench_run(task, description, outputs, empty, crc, least, sum, most);
        }
        *crc = ~*crc;
        return;
    }
    for (;;)
    {
        for (uint32_t i = 0; i < n; i++)
            bench_ram[address[i]] = count[i] == 2 ? (place[i] ? 0xFF : 0x00) : place[i];
        bench_run(task, description, outputs, empty, crc, least, sum, most);
        ++*cases;
        uint32_t i = n;
        while (i > 0 && ++place[i - 1] == count[i - 1])
            place[--i] = 0;
        if (i == 0)
            break;
    }
    if (flags < INPUTS && other < INPUTS)
        for (uint32_t bit = 0; bit < 8; bit++)
            for (uint32_t v = 0; v < 256; v++)
            {
                for (uint32_t i = 0; i < n; i++)
                    bench_ram[address[i]] = 0x80;
                bench_ram[address[flags]] = 1u << bit;
                bench_ram[address[other]] = v;
                bench_run(task, description, outputs, empty, crc, least, sum, most);
                ++*cases;
            }
    *crc = ~*crc;
}

// A read wait after the one that started at after: checked against what the 8051 should read, then
// the unit's answer timed from the read's end, late if not in place due after the read's start (the
// next read's). Returns when the read started.
static uint32_t bus_step(uint32_t after, uint32_t wait, uint32_t address, uint32_t expect, uint32_t kind,
                         uint32_t due)
{
    while (since(after) < wait)
    {
    }
    uint32_t served = unit.served, value, end;
    uint32_t start = bench_read(address, unit.rom, unit.mask, &value, &end);
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
    if (cycles > due - READ_CYCLES)
        bus.late++; // after the next read would have begun
    Times *times = &bus.times[kind];
    times->count++;
    times->sum += cycles;
    if (cycles > times->most)
        times->most = cycles;
    return start;
}

// A task as coproc_slow's Co_Task asks for it, its inputs at random: each output checked against the
// task worked out here (core 1's own RAM bytes). The first output read 6 machine cycles after the last
// send, the rest 4 or more apart. Returns when the last read started.
static uint32_t bus_task(uint32_t t, uint32_t window, uint32_t id, uint32_t task, uint32_t description, uint32_t *seed)
{
    uint32_t n = at(description), outputs = description + 1 + n;
    for (uint32_t k = 1; k <= n; k++)
    {
        *seed = xorshift(*seed);
        bus_ram[at(description + k) & 0x7F] = *seed & 0xFF;
    }
    unit_task(task, description, unit.rom, bus_ram);
    t = bus_step(t, PAUSE, window + id, at(description + 1), ANSWER, GAP);
    for (uint32_t k = 1; k <= n; k++) // send k: the byte read is input k + 1's address, or 0
        t = bus_step(t, GAP, window + bus_ram[at(description + k) & 0x7F], k < n ? at(description + 1 + k) : 0,
                     k < n ? ANSWER : VALUE, k < n ? GAP : TASK_GAP);
    uint32_t wait = TASK_GAP;
    for (uint32_t j = 0; j < at(outputs); j++)
    {
        uint32_t address = at(outputs + 1 + j) & 0x7F;
        t = bus_step(t, wait, window, address, ANSWER, GAP);
        t = bus_step(t, GAP, window + address, bus_ram[address], ANSWER, GAP);
        wait = GAP;
    }
    return bus_step(t, wait, window, 0, RESTORE, PAUSE); // the 0 after the last output: the page as written
}

// Core 1: each map and task in turn at random inputs, as coproc_maps' Map_Lookup and coproc_slow's
// Co_Task read them. One lookup in 64 is broken off after its pick (and, one time in two, an input) by
// a read of 0000h, as the 8051's start after a reset would; the lookup after it must be right.
static void bus_run()
{
    systick_start();
    uint32_t count = at(MARK + 6), index = at(MARK + 7) << 8 | at(MARK + 8), window = unit.window;
    uint32_t seed = 0x2545F491, id = 0, t = systick_hw->cvr;
    for (uint32_t n = 0; n < bus.want; n++)
    {
        uint32_t entry = index + 3 * id;
        uint32_t map = at(entry) << 8 | at(entry + 1), axes = at(entry + 2);
        if (axes & TASK)
            t = bus_task(t, window, id, axes & ~TASK, map, &seed);
        else
        {
            seed = xorshift(seed);
            uint32_t first = seed & 0xFF, second = seed >> 8 & 0xFF;
            uint32_t next = axes > 1 ? at(map + 9) : 0; // the answer to the first input
            if ((n & 63) == 63)
            {
                t = bus_step(t, PAUSE, window + id, at(map), ANSWER, GAP);
                if (seed & 0x10000)
                    t = bus_step(t, GAP, window + (second ^ 0x5A), next, axes > 1 ? ANSWER : VALUE, GAP);
                t = bus_step(t, GAP, 0, at(0), RESTORE, PAUSE);
            }
            uint32_t value = map_unit_lookup(map, axes, first, second);
            t = bus_step(t, PAUSE, window + id, at(map), ANSWER, GAP);
            t = bus_step(t, GAP, window + first, next, axes > 1 ? ANSWER : VALUE, GAP);
            if (axes > 1)
                t = bus_step(t, GAP, window + second, 0, VALUE, GAP);
            t = bus_step(t, GAP, window, value, RESTORE, PAUSE);
        }
        bus.done = n + 1;
        if (++id == count)
            id = 0;
    }
    bus.state = BUS_ENDED;
    bench_finished();
}

static bool bus_start(uint32_t want)
{
    bench_update(); // a run over, the board not yet given back: given back now
    if (!unit.on || bench_running() || want == 0)
        return false;
    memset(&bus, 0, sizeof(bus));
    bus.want = want;
    bus.state = BUS_RUNNING;
    if (bench_launch(bus_run))
        return true;
    bus.state = BUS_IDLE;
    return false;
}

void map_unit_update()
{
    bench_update();
}

// map_probe: one read made from core 0, as the bus bench makes them: what the pins read during it,
// where map_window was during and after it, and the reads the unit answered.
static char probe_text[40] UNIT_DATA = {};

static void probe(uint32_t address)
{
    bench_take();
    busy_wait_us_32(2); // map_window back at its start, whatever the pins said before
    uint32_t served = unit.served, before = pio_sm_get_pc(MAP_UNIT_PIO, MAP_UNIT_SM);
    bench_address(address);
    bench_oe(true);
    busy_wait_us_32(2);
    uint32_t pins = sio_hw->gpio_in, during = pio_sm_get_pc(MAP_UNIT_PIO, MAP_UNIT_SM);
    bench_oe(false);
    busy_wait_us_32(20);
    uint32_t after = pio_sm_get_pc(MAP_UNIT_PIO, MAP_UNIT_SM);
    bench_give();
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
    uint32_t entry = entry_of(id);
    *map = at(entry) << 8 | at(entry + 1);
    *axes = at(entry + 2);
    return !(*axes & TASK); // a map's
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
    if (streq(name, "map_task"))
    {
        if (!unit.on || args[0] >= at(MARK + 6))
            return false;
        uint32_t entry = entry_of(args[0]), kind = at(entry + 2);
        if (!(kind & TASK))
            return false; // a map's id
        uint32_t crc, least, sum, most, cases;
        bench_task(kind & ~TASK, at(entry) << 8 | at(entry + 1), &crc, &least, &sum, &most, &cases);
        snprintf(task_text, sizeof(task_text), "crc %08x cycles %u %u %u", (unsigned)crc, (unsigned)least,
                 (unsigned)((sum + cases / 2) / cases), (unsigned)most);
        return true;
    }
    if (streq(name, "map_bus"))
        return bus_start(args[0]);
    if (streq(name, "map_probe"))
    {
        if (!unit.on || bench_running())
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
        if (!unit.on)
            strcpyz(value, value_size, "off");
        else
        {
            uint32_t count = at(MARK + 6), maps = 0;
            while (maps < count && !(at(entry_of(maps) + 2) & TASK))
                maps++; // the tasks come after the maps
            snprintf(value, value_size, "%u maps, %u tasks, window %04x", (unsigned)maps, (unsigned)(count - maps),
                     (unsigned)unit.window);
        }
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
    if (streq(name, "map_task"))
    {
        strcpyz(value, value_size, task_text[0] ? task_text : "none");
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
