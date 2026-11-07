// The snoop (FEATURE_MAP_UNIT; docs/snoop.md): what the board learns of the 8051 from the socket alone,
// as DME-Sim's P28Snoop models it (its docs/Socket_View.md):
//
//   each MOVX: the program snoop (map_unit.pio) finds /OE high for longer than a read's gap after a
//   read, and pushes all the pins 1.5 us after that read's end (A0-A12 the data address, D0-D7 the
//   byte), then those of the read itself, of the byte after the MOVX's opcode: the image says read
//   or write. Every read is the ADC's (its OE, START and ALE are all NOT /RD: DME-Sim's
//   docs/Hardware.md): A8-A10 the channel it starts, and its byte the result of the conversion the
//   read before started, on that read's channel (done by then: a conversion takes 64 us, and
//   rom_lean's reads are five teeth or more apart).
//   each interrupt: map_window pushes every read of page 0 (map_unit.cpp); a vector the firmware uses
//   read, then the byte after it, is its routine starting.
//
// The byte needs the P28's data buffer turned towards the RP2040 for the sample (not yet: docs/snoop.md);
// the bench (core 1 playing the 8051 through the input overrides, the data pins' too) checks the rest.

#include "snoop.h"

#if defined(FEATURE_MAP_UNIT)

#include <stdio.h>
#include <string.h>

#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/timer.h"

#include "bus_bench.h"
#include "crank.h"
#include "map_unit.h"
#include "pio_programs.h"
#include "str_util.h"
#include "system.h"
#include "unit_ram.h"

static constexpr uint32_t CODE_MASK = 0x1FFF; // the 8051's 8 KiB, as the socket shows them
static constexpr uint32_t NONE = 0x100;       // no read of page 0 before
static constexpr uint32_t NO_CHANNEL = 8;     // no ADC read before

enum : uint32_t
{
    OTHER,
    READ,
    WRITE
};

struct Movx
{
    uint32_t kind, address, data;
};

struct Snoop
{
    const uint8_t *rom;
    uint32_t vectors;                        // the vectors watched: bit k for 0003h + 8k
    uint32_t previous;                       // the last read of page 0's A0-A7, or NONE
    volatile uint32_t activity;              // reads seen (the activity LED)
    volatile uint32_t reads, writes, others; // MOVX reads and writes; gaps with no MOVX before them
    volatile uint32_t movx;                  // MOVX seen
    Movx log[4];                             // the last four, by movx
    uint8_t adc[8];                          // each channel's last result
    uint32_t adc_count[8];
    volatile uint32_t entries[5]; // each vector's routines begun
    uint32_t entry_time[5];       // when the last began (us)
    uint32_t channel;             // the last ADC read's channel (the conversion it started), or NO_CHANNEL
    uint32_t pins;                // A0's GPIO, D0's << 8, /OE's << 16 (system.h): for DME-Sim's model of the pushes
    uint8_t adc_before[8];        // each channel's result before its last
};

static Snoop snoop UNIT_DATA = {};

// ---------------------------------------------------------------------------------------------------
// The interrupts: the snoop's own (a MOVX), and the map unit's for page 0.

static __force_inline uint32_t kind_of(uint32_t opcode)
{
    if (opcode == 0xE0 || opcode == 0xE2 || opcode == 0xE3)
        return READ;
    if (opcode == 0xF0 || opcode == 0xF2 || opcode == 0xF3)
        return WRITE;
    return OTHER;
}

// A gap: the pins in the strobe, then those of the read before it (the byte after a MOVX's opcode).
static __force_inline void gap(uint32_t strobe, uint32_t before)
{
    uint32_t opcode = snoop.rom[(((before >> BASE_ADDR_PIN) & CODE_MASK) - 1) & CODE_MASK];
    uint32_t kind = kind_of(opcode);
    if (kind == OTHER)
    {
        snoop.others++;
        return;
    }
    uint32_t address = (strobe >> BASE_ADDR_PIN) & CODE_MASK, data = (strobe >> BASE_DATA_PIN) & 0xFF;
    if (kind == READ)
    {
        uint32_t channel = snoop.channel; // the conversion this read collects
        snoop.channel = (address >> 8) & 7;
        if (channel != NO_CHANNEL)
        {
            snoop.adc_before[channel] = snoop.adc[channel];
            snoop.adc[channel] = data;
            snoop.adc_count[channel]++;
            map_unit_adc(channel);
        }
        snoop.reads++;
        crank_read((before >> BASE_ADDR_PIN) & CODE_MASK, timer_hw->timerawl);
    }
    else
        snoop.writes++;
    snoop.log[snoop.movx & 3] = {kind, address, data};
    snoop.movx++;
}

static __force_inline uint32_t rx_level()
{
    return (SNOOP_PIO->flevel >> (SNOOP_SM * 8 + 4)) & 0xF; // FLEVEL: RXn at bits 8n + 4
}

// Below the map unit, above USB and the timers.
static void UNIT_CODE(snoop_irq)()
{
    while (rx_level() != 0)
    {
        uint32_t strobe = SNOOP_PIO->rxf[SNOOP_SM];
        for (uint32_t spin = 0; rx_level() == 0 && spin < 64; spin++) // its pair, a cycle behind
        {
        }
        uint32_t before = SNOOP_PIO->rxf[SNOOP_SM];
        snoop.activity++;
        gap(strobe, before);
    }
}

void UNIT_CODE(snoop_page_read)(uint32_t address)
{
    uint32_t previous = snoop.previous, k = previous >> 3;
    snoop.previous = address;
    snoop.activity++;
    if (address != previous + 1 || (previous & 7) != 3 || k > 4 || !((snoop.vectors >> k) & 1))
        return;
    snoop.entries[k]++;
    snoop.entry_time[k] = timer_hw->timerawl;
    if (k == 2) // INT1: a tooth
        crank_tooth(snoop.entry_time[k]);
}

uint32_t UNIT_CODE(snoop_result)(uint32_t channel, uint32_t back)
{
    return back ? snoop.adc_before[channel & 7] : snoop.adc[channel & 7];
}

uint32_t UNIT_CODE(snoop_results)(uint32_t channel)
{
    return snoop.adc_count[channel & 7];
}

void UNIT_CODE(snoop_start)(const uint8_t *rom)
{
    snoop.rom = rom;
    snoop.vectors = 0x1F;
    snoop.previous = NONE;
    snoop.channel = NO_CHANNEL;
    snoop.pins = BASE_ADDR_PIN | BASE_DATA_PIN << 8 | BASE_OE_PIN << 16;
    crank_start(rom);
}

void snoop_init(const uint8_t *rom)
{
    snoop_start(rom);
    if (!prg_snoop.valid())
        return;
    PRG_LOCAL(prg_snoop, p, sm, offset, cfg);
    sm_config_set_in_pins(&cfg, 0); // GPIO0 up: all the pins
    sm_config_set_jmp_pin(&cfg, BASE_OE_PIN);
    sm_config_set_in_shift(&cfg, false, true, 32); // autopush
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_RX);
    pio_sm_init(p, sm, offset, &cfg);
    pio_set_irq1_source_enabled(p, (pio_interrupt_source_t)(pis_sm0_rx_fifo_not_empty + sm), true);
    irq_set_exclusive_handler(PIO1_IRQ_1, snoop_irq);
    irq_set_priority(PIO1_IRQ_1, 0x40);
    irq_set_enabled(PIO1_IRQ_1, true);
    pio_sm_set_enabled(p, sm, true);
}

bool snoop_activity()
{
    static uint32_t seen;
    uint32_t now = snoop.activity;
    bool active = now != seen;
    seen = now;
    return active;
}

// ---------------------------------------------------------------------------------------------------
// The bench: core 1 plays rounds of the 8051's reads as DME-Sim's SocketBus makes them (bus_bench.h),
// each with a MOVX read (an ADC read) and a MOVX write, an interrupt's routine (INT1's, its RETI's
// dropped reads of its vector after it) and another's (INT0's, an AJMP's dropped reads of Timer 0's
// vector in it). In a MOVX's gap the byte is on the data pins only from P + on to P + off (its
// inverse before and after), P the end of the read before. The snoop must find each MOVX with its
// byte and each routine, and nothing else.

static constexpr uint32_t PERIOD = MACHINE_CYCLE / 2;            // a /PSEN read every 1 us
static constexpr uint32_t NEXT_AT = 2500 * CYCLES_PER_US / 1000;   // the next opcode's read
static constexpr uint32_t LOST = 50 * CYCLES_PER_US;               // no MOVX seen by then: lost
static constexpr uint32_t CODE = 0x0123;                           // where the interrupts come

enum : uint32_t
{
    BENCH_IDLE,
    BENCH_RUNNING,
    BENCH_ENDED
};

struct Bench
{
    volatile uint32_t state;
    uint32_t want, read_at, write_at;
    volatile uint32_t done;
    uint32_t wrong, lost, irq;
    uint32_t on, off;            // the byte's window, cycles from P
    uint32_t write;              // cycles a pin's control takes to write, mean of 64
    uint32_t lag;                // from an edge asked to where it lands, cycles, mean of 64
    uint32_t on_least, on_most;  // where the byte's first edge landed, cycles from P
    uint32_t off_least, off_most; // and its second
};

static Bench bench UNIT_DATA = {BENCH_IDLE, 0, 0, 0, 0, 0, 0, 0, 1360 * CYCLES_PER_US / 1000,
                                1800 * CYCLES_PER_US / 1000, 0, 0, 0, 0, 0, 0};

// A pin as the processor reads it, through the input overrides (the PIO reads the same).
static __force_inline uint32_t pin(uint32_t gpio)
{
    return (sio_hw->gpio_in >> gpio) & 1;
}

// /OE (its control written) as it lands: a control's write takes effect some cycles on.
static void UNIT_CODE(oe)(bool low)
{
    bench_oe(low);
    while (pin(BASE_OE_PIN) == (low ? 1u : 0u))
    {
    }
}

// One read, its address put out as the read before ends (as the 8051 does), its /OE fall PERIOD after
// the one before's (at after). Returns when /OE fell; end, when it rose.
static uint32_t UNIT_CODE(play)(uint32_t after, uint32_t address, uint32_t *end = nullptr)
{
    bench_address(address);
    while (since(after) < PERIOD)
    {
    }
    oe(true);
    uint32_t start = systick_hw->cvr;
    while (since(start) < READ_CYCLES)
    {
    }
    oe(false);
    if (end != nullptr)
        *end = systick_hw->cvr;
    return start;
}

static uint32_t UNIT_CODE(run)(uint32_t t, uint32_t from, uint32_t count)
{
    for (uint32_t i = 0; i < count; i++)
        t = play(t, from + i);
    return t;
}

// One of the byte's edges (bit 0's pin) and where it landed, cycles from P (at end).
static uint32_t UNIT_CODE(edge)(uint32_t byte, uint32_t end)
{
    bench_data(byte);
    while (pin(BASE_DATA_PIN) != (byte & 1))
    {
    }
    return since(end);
}

static __force_inline void note(uint32_t at, uint32_t *least, uint32_t *most)
{
    if (at < *least)
        *least = at;
    if (at > *most)
        *most = at;
}

// A MOVX at m: its opcode's read and the byte after's, then the gap: the data address (from P: the
// snoop takes it with the byte), the byte from P + on to P + off (the caller has put it on the data
// pins ahead with bit 0 the other way, which turns at P + on and back at P + off, each asked for its
// lag early: a sample outside reads it wrong), and the next opcode's read (m + 1) at P + 2.5 us.
static uint32_t UNIT_CODE(movx)(uint32_t t, uint32_t m, uint32_t address, uint32_t byte)
{
    uint32_t end;
    t = play(t, m);
    t = play(t, m + 1, &end);
    bench_address(address);
    while (since(end) + bench.lag < bench.on)
    {
    }
    note(edge(byte, end), &bench.on_least, &bench.on_most);
    while (since(end) + bench.lag < bench.off)
    {
    }
    note(edge(byte ^ 1, end), &bench.off_least, &bench.off_most);
    bench_address(m + 1);
    while (since(end) < NEXT_AT)
    {
    }
    oe(true); // the next opcode's read: its /OE fall at P + 2.5 us
    uint32_t start = systick_hw->cvr;
    while (since(start) < READ_CYCLES)
    {
    }
    oe(false);
    return start;
}

static bool UNIT_CODE(logged)(uint32_t movx, uint32_t kind, uint32_t address, uint32_t byte)
{
    const Movx &m = snoop.log[movx & 3];
    return m.kind == kind && m.address == (address & CODE_MASK) && m.data == (byte & 0xFF);
}

static void UNIT_CODE(bench_run)()
{
    systick_start();
    uint32_t t = systick_hw->cvr;
    for (uint32_t i = 0; i < 64; i++)
        bench_oe(false);
    bench.write = since(t) / 64;
    t = systick_hw->cvr;
    for (uint32_t i = 1; i <= 64; i++)
        edge(i & 1, t); // bit 0 alone, each way
    bench.lag = since(t) / 64;
    uint32_t seed = 0x2545F491;
    t = systick_hw->cvr;
    for (uint32_t n = 0; n < bench.want; n++)
    {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        // The data addresses differ from the read before the gap in A8-A10 (the ADC's channel) or A0-A2
        // only: a pin's control takes long to write (snoop_bench_data), the 8051's pins none.
        uint32_t read_address = ((bench.read_at + 1) & ~0x700u) | (n & 7) << 8, byte = seed >> 8 & 0xFF;
        uint32_t write_address = (bench.write_at + 1) ^ (seed & 7), written = seed >> 24;
        uint32_t movx0 = snoop.movx, int0 = snoop.entries[0], timer0 = snoop.entries[1], int1 = snoop.entries[2];

        t = run(t, bench.read_at - 4, 2);
        bench_data(byte ^ 1);                           // the byte but bit 0, ahead
        t = run(t, bench.read_at - 2, 2);
        t = movx(t, bench.read_at, read_address, byte); // an ADC read: channel n & 7
        t = run(t, bench.read_at + 2, 3);
        t = run(t, CODE, 1);                            // INT1: the LCALL's reads, the routine, its RETI
        t = run(t, CODE, 1);
        t = run(t, CODE, 1);
        t = run(t, CODE, 1);
        t = run(t, 0x0013, 4);
        t = run(t, 0x0012, 1);
        for (uint32_t i = 0; i < 3; i++)
            t = run(t, 0x0013, 1);
        t = run(t, CODE, 2);
        for (uint32_t i = 0; i < 4; i++)                // INT0's, an AJMP at 0009h in it
            t = run(t, CODE + 2, 1);
        t = run(t, 0x0003, 8);
        t = run(t, 0x000B, 1);
        t = run(t, 0x000B, 1);
        t = run(t, 0x0030, 2);
        t = run(t, bench.write_at - 4, 2);
        bench_data(written ^ 1);
        t = run(t, bench.write_at - 2, 2);
        t = movx(t, bench.write_at, write_address, written);
        t = run(t, bench.write_at + 2, 3);
        t = run(t, 0x0001, 1); // the round's end: a gap after it, its opcode (0000h's) no MOVX

        uint32_t start = systick_hw->cvr;
        while (snoop.movx - movx0 < 2 && since(start) < LOST)
            __asm volatile("nop\n nop\n nop\n nop\n nop\n nop"); // gently: the snoop is in the unit's bank
        uint32_t seen = snoop.movx - movx0;
        if (seen < 2)
            bench.lost += 2 - seen;
        else if (!logged(movx0, READ, read_address, byte) || !logged(movx0 + 1, WRITE, write_address, written))
            bench.wrong++;
        if (snoop.entries[0] - int0 != 1 || snoop.entries[1] - timer0 != 0 || snoop.entries[2] - int1 != 1)
            bench.irq++;
        bench.done = n + 1;
        t = systick_hw->cvr;
    }
    bench.state = BENCH_ENDED;
    bench_finished();
}

static bool bench_start(uint32_t want, uint32_t read_at, uint32_t write_at)
{
    bench_update(); // a run over, the board not yet given back: given back now
    if (bench_running() || want == 0 || kind_of(snoop.rom[read_at & CODE_MASK]) != READ ||
        kind_of(snoop.rom[write_at & CODE_MASK]) != WRITE)
        return false;
    bench.want = want;
    bench.read_at = read_at & CODE_MASK;
    bench.write_at = write_at & CODE_MASK;
    bench.done = bench.wrong = bench.lost = bench.irq = bench.on_most = bench.off_most = 0;
    bench.on_least = bench.off_least = 0xFFFFFF;
    bench.state = BENCH_RUNNING;
    if (bench_launch(bench_run))
        return true;
    bench.state = BENCH_IDLE;
    return false;
}

// ---------------------------------------------------------------------------------------------------
// Parameters: snoop (the MOVX reads and writes found, and the gaps with no MOVX), snoop_adc (each
// channel's last result), snoop_last (the last MOVX), snoop_irq (k: vector 0003h + 8k's routines and
// when the last began), snoop_vectors (the vectors watched: bit k for 0003h + 8k), snoop_window
// (on,off: the bench's byte from P + on to P + off, ns), snoop_bench (n,read_at,write_at: n rounds,
// the MOVX read's and write's opcodes at those addresses; its answer comes as it runs) and
// snoop_bench_data (the cycles from P to where the byte's edges landed, least and most of each, a pin's
// write, and an edge's lag). An answer is 35 characters at most (a packet's payload).

static uint32_t irq_shown UNIT_DATA = 2;

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

bool snoop_set_parameter(const char *name, const char *value)
{
    uint32_t args[3] = {};
    uint32_t n = numbers(value, args);
    if (streq(name, "snoop_vectors"))
    {
        snoop.vectors = args[0] & 0x1F;
        return true;
    }
    if (streq(name, "snoop_irq"))
    {
        if (args[0] > 4)
            return false;
        irq_shown = args[0];
        return true;
    }
    if (streq(name, "snoop_window"))
    {
        if (n < 2 || args[0] >= args[1] || args[1] >= 2400 || bench_running())
            return false;
        bench.on = args[0] * CYCLES_PER_US / 1000;
        bench.off = args[1] * CYCLES_PER_US / 1000;
        return true;
    }
    if (streq(name, "snoop_bench"))
        return n == 3 && bench_start(args[0], args[1], args[2]);
    return false;
}

bool snoop_get_parameter(const char *name, char *value, size_t value_size)
{
    if (streq(name, "snoop"))
    {
        snprintf(value, value_size, "%u reads %u writes %u other", (unsigned)snoop.reads, (unsigned)snoop.writes,
                 (unsigned)snoop.others);
        return true;
    }
    if (streq(name, "snoop_adc"))
    {
        const uint8_t *a = snoop.adc;
        snprintf(value, value_size, "%02x %02x %02x %02x %02x %02x %02x %02x", a[0], a[1], a[2], a[3], a[4], a[5],
                 a[6], a[7]);
        return true;
    }
    if (streq(name, "snoop_last"))
    {
        const Movx &m = snoop.log[(snoop.movx - 1) & 3];
        if (snoop.movx == 0)
            strcpyz(value, value_size, "none");
        else
            snprintf(value, value_size, "%s %04x %02x", m.kind == READ ? "read" : "write", (unsigned)m.address,
                     (unsigned)m.data);
        return true;
    }
    if (streq(name, "snoop_irq"))
    {
        uint32_t k = irq_shown;
        snprintf(value, value_size, "%04x %u, the last at %u us", (unsigned)(3 + 8 * k), (unsigned)snoop.entries[k],
                 (unsigned)snoop.entry_time[k]);
        return true;
    }
    if (streq(name, "snoop_vectors"))
    {
        snprintf(value, value_size, "0x%02x", (unsigned)snoop.vectors);
        return true;
    }
    if (streq(name, "snoop_window"))
    {
        snprintf(value, value_size, "%u %u ns", (unsigned)(bench.on * 1000 / CYCLES_PER_US),
                 (unsigned)(bench.off * 1000 / CYCLES_PER_US));
        return true;
    }
    if (streq(name, "snoop_bench"))
    {
        if (bench.state == BENCH_IDLE)
            strcpyz(value, value_size, "none");
        else if (bench.state != BENCH_ENDED)
            snprintf(value, value_size, "running %u of %u", (unsigned)bench.done, (unsigned)bench.want);
        else
            snprintf(value, value_size, "%u done %u wrong %u lost %u irq", (unsigned)bench.done, (unsigned)bench.wrong,
                     (unsigned)bench.lost, (unsigned)bench.irq);
        return true;
    }
    if (streq(name, "snoop_bench_data"))
    {
        snprintf(value, value_size, "%u %u %u %u %u %u", (unsigned)bench.on_least, (unsigned)bench.on_most,
                 (unsigned)bench.off_least, (unsigned)bench.off_most, (unsigned)bench.write, (unsigned)bench.lag);
        return true;
    }
    return false;
}

#endif // FEATURE_MAP_UNIT
