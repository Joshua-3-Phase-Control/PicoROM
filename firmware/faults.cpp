// The fault codes (faults.h; DME-Sim's docs/Paired_Faults.md, format 31): the first build records only; the actions'
// first step (format 32) debounces them, and 1113 judges the full-load signal closed once it has stood
// Cal_FaultFullTicks (at a snap open the air of a firing passes its limit a tick or two after the switch). The
// second (format 33) acts on lambda and the trims: the trims learn by the faults present now (pair.cpp's
// trim_tick), and lambda's loop is held open for the drive once its integrator has stood at 0, the sensor reading
// rich, for 1123's confirm time (fault_loop_open, which task 16 reads). The fourth (format 35) gives the control the
// NTCs' substitutes for a code past the checks' limits (fault_substitutes, which map_unit_adc takes at once). The
// fifth (format 36) looks for 1112 above the overrun's cut-off speed at the contact's raw state, and while it stands
// takes the contact open above that speed (fault_idle_open, which task 15's switches take).
//
// Once a tick (L2, after the trims' work, before the frame) each check looks at what the P28 has already: the
// snoop's ADC codes (the NTCs before their linearisation), lambda's and the idle control's state in the tick's
// copy, and the counts L0, L1 and L2 keep, each its one writer's (this reads their change since the last tick).
// A check tells, cause by cause (its bits, the code's own), what it tested this tick and what it saw. Then each
// code seen or not absent is qualified: absent; pending (an up/down count of its ticks above 0: one up a tick
// seen, one down a tick tested clean, held by a tick not tested; so a fault seen in more than half the ticks that
// test it confirms, and a glitch leaves nothing); present (confirmed at its confirm time's count: its record
// written); gone (tested clean for its heal time: the record stays, read as 12xx; a cause not testable now holds
// it). Checks with times of their own (1124 on one side, 1132 at rest and at idle, the crank's leaky counts, the
// events) qualify themselves and are confirmed at once; a heal of FFFFh ticks is the next drive without it (a
// drive: the engine run 30 s after the 8051's start).
//
// The memory, the store's third kind (232 bytes): a head (the drives counted, 32 bits; the clears, 16; the
// faults lost to a full memory, 8; a spare), then seven records of 32 bytes: the code (its place in the table
// from 1; 0 an empty slot), its cause, its state (present, confirmed this drive), its count of confirmations, the
// drives since it was last seen, a spare, first and last seen (the drive's low 16 bits, then the seconds into
// it), and its frame at the first confirmation (Cal_FaultFrame's 18 bytes, the log's as it stood). Words high
// byte first. A confirmation asks the store for a record soon (at least 10 s after the kind's last); a fault
// gone and a drive counted wait for its minute; a clear is written at once.

#include "faults.h"

#if defined(FEATURE_MAP_UNIT)

#include <stdio.h>

#include "map_unit.h"
#include "store.h"
#include "str_util.h"
#include "unit_ram.h"
#include "unit_tasks.h"

static constexpr uint32_t MARK = 0x2000;
static constexpr uint32_t FAULTS_FORMAT = 31;              // the table's first (and the memory's layout's)
static constexpr uint32_t FULL_SETTLE = 32;                 // the format with Cal_FaultFullTicks after the limits
                                                            //   (33, faults.h's: the faults act on lambda and the
                                                            //   trims; 35, the substitutes' three words after it)
static constexpr uint32_t RECORDS = 7, RECORD = 32, HEAD = 8, FRAME = 18;
static constexpr uint32_t BYTES = HEAD + RECORDS * RECORD;  // 232: the store's record
static constexpr uint32_t LAYOUT = FAULTS_FORMAT << 24 | RECORDS << 16 | RECORD << 8 | FRAME;
static constexpr uint32_t BY_DRIVE = 0xFFFF;                // a heal time: the next drive without it
static constexpr uint32_t A_SECOND = 88;                    // ticks (ISV_PERIOD 5681 x 2 us: 11.36 ms)
static constexpr uint32_t NONE = 0xFF;
static constexpr uint32_t CONFIRMS = 2 * FAULT_CHECKS, HEALS = 4 * FAULT_CHECKS; // the lists after the codes'

// A record's bytes.
enum : uint32_t
{
    R_CODE = 0,
    R_CAUSE = 1,
    R_STATE = 2,
    R_COUNT = 3,
    R_DRIVES = 4,
    R_FIRST = 6,
    R_LAST = 10,
    R_FRAME = 14,
};

// A record's state: present (else gone), confirmed this drive.
static constexpr uint32_t PRESENT = 1, THIS_DRIVE = 2;

// A code's qualification.
enum : uint8_t
{
    ABSENT,
    PENDING,
    CONFIRMED,
    GONE,
};

// The causes, a code's own bits (the .mld names them).
enum : uint8_t
{
    C_HIGH = 1,       // over its upper limit (an NTC open, the AFM at 5 V, 1141 restarted ...)
    C_LOW = 2,        // under its lower limit (an NTC shorted, the AFM at 0 V ...)
    C_OTHER = 4,      // its other test: 1112's wiring, 1123's trims, 1131's knock, 1132 at rest, 1162 run again
    C_FOURTH = 8,     // 1124 no signal, 1132 at a warm idle
    C_FIFTH = 16,     // 1124 on one side
};

// The limits as the image has them (from Cal_FaultSettleTicks: its bytes, then its words high byte first; format
// 32 a word more, Cal_FaultFullTicks, else 0; format 35 three more, the substitutes', else 0), taken at the start
// into RAM: a load each.
struct Limits
{
    uint8_t settle, supply_low, supply_high, ntc_open, ntc_short, afm_low, afm_high, map_low, map_high;
    uint8_t map_rest_low, map_rest_high, map_idle, warm, in6[4], in6_band;
    uint16_t afm_rpm, switch_rpm, idle_air, full_air, idle_off, idle_flow, idle_ticks, map_rest_ticks;
    uint16_t map_idle_ticks, o2_stuck_ticks, no_start_ticks, rest_ticks, event_add, event_level, clean_turns;
    uint16_t again_add, again_level, restart_ms, drive_ticks, full_ticks;
    uint16_t sub_cold, sub_warm, sub_rise; // format 35: Cal_FaultSubCold, Cal_FaultSubWarm (bytes), Cal_FaultSubRise
                                           //   (1/65536 of a byte a tick)
};

static constexpr uint32_t LIMIT_BYTES = 18, LIMIT_WORDS = 19;

// Each code as the P28 keeps it (L2's alone).
struct Check
{
    uint8_t state;      // ABSENT, PENDING, CONFIRMED, GONE
    uint8_t cause;      // the causes seen while pending or present
    uint8_t this_drive; // seen since the 8051's start
    uint8_t slot;       // its record's slot (NONE: none)
    uint16_t timer;     // ticks: pending, its up/down count toward its confirm time; present, toward its heal time
    uint16_t code;      // its code (BCD; 0: none on this car)
    uint16_t confirm, heal; // ticks (a heal of FFFFh: the next drive without it)
};

struct Faults
{
    // Read by DME-Sim (its tests take these offsets): the counts, then the state.
    volatile uint32_t runs;        // 0: ticks worked
    volatile uint32_t present;     // 4: the faults present now, a bit by id
    volatile uint32_t confirmed;   // 8: confirmations
    volatile uint32_t pending;     // 12: the codes pending now, a bit by id
    uint32_t since_start;          // 16: ticks since the 8051's start (to FFFFh)
    uint32_t drive_ticks;          // 20: ticks running since then (to Cal_FaultDriveTicks: the drive counted)
    uint32_t seconds, sub;         // 24, 28: the drive's seconds (to FFFFh), the tick in its second
    uint32_t fuel_off;             // 32: format 33, the drive's up/down count of ticks at lambda's fuel-off limit
                                   //   (closed, LamInt 0: one up; closed above it: one down), toward 1123's confirm
                                   //   time, which holds lambda's loop open (fault_loop_open) for the drive
    // The table's.
    bool on;
    bool lambda;                   // format 33's action on lambda (a format 33 image, 1123 fitted)
    uint32_t fitted;               // the codes this car has, a bit by id
    const uint8_t *rom;
    uint32_t codes_at, frame_at, live_at;
    // L2's own: the counts at the last tick, and what the checks keep from tick to tick.
    uint32_t seen_starts, seen_teeth, seen_refs, seen_turns, seen_missing, seen_crank, seen_beats;
    uint32_t seen_over, seen_lost, seen_tick_lost, seen_tick_over, seen_again, seen_store;
    uint32_t refs_at_teeth;        // the references when the teeth last moved (1152)
    uint32_t no_tooth;             // ticks since the last tooth (at rest)
    uint32_t seen_results;         // the snoop's results at the last tick
    uint32_t read_ago, read_teeth; // ticks since the 8051 last read an input; the teeth then
    bool read_at_rest;             // that read with no tooth since the tick before it (Power_Up_Inputs')
    uint32_t no_reference;         // ticks cranking with no reference (1151's no-start)
    uint32_t idle_ticks;           // ticks the idle contact has been closed, running (1122)
    uint32_t full_ticks;           // ticks the full-load signal has been closed, running (1113)
    uint32_t map_rest, map_idle;   // ticks off at rest, over at a warm idle (1132)
    uint32_t one_side, mid_band;   // ticks the oxygen sensor has stayed on one side, in its middle band (1124)
    uint32_t ref_leak, ref_clean, crank_leak, crank_clean; // 1151's and 1153's leaky counts and clean turns
    uint32_t again_leak;           // 1162's
    uint32_t last_running;         // the last tick running: its ping (us), for 1141's restart
    bool was_rich, was_running, no_start; // the sensor's side; the last tick running; 1151's no-start held
    bool no_start_drive;           // a no-start this drive
    bool dirty;                    // the memory changed by what the minute's rule writes
    uint32_t frames_due;           // the records whose frame is to be filled this tick (a bit a slot)
    // Format 35: the substitutes' state (this power-up's): the coolant's base and its rise (1/65536 of a byte),
    // whether a plausible coolant code has come, the intake air's first plausible byte.
    uint32_t coolant_base, coolant_rise;
    bool coolant_seen, air_seen;
    uint8_t air_first;
    bool idle_raw;                 // format 36: 1112's check at the cut-off on IN6's raw state, its action on
};

Faults fault_state BANK2_DATA = {};
volatile uint8_t fault_loop_open BANK2_DATA = 0;
volatile FaultSubstitutes fault_substitutes BANK2_DATA = {};
volatile uint8_t fault_idle_open BANK2_DATA = 0;
Check fault_checks[FAULT_CHECKS] BANK2_DATA = {};
static Limits limits BANK2_DATA = {};
static FaultPlaces src BANK2_DATA = {};
alignas(4) uint8_t fault_memory[BYTES] __attribute__((section(".store")));

static __force_inline uint32_t at(uint32_t address)
{
    return fault_state.rom[address & 0xFFFF];
}

static __force_inline uint32_t word(uint32_t address)
{
    return at(address) << 8 | at(address + 1);
}

static __force_inline bool bit(const uint8_t *bytes, uint32_t place)
{
    return bytes[place >> 8] & place;
}

static __force_inline uint8_t *record(uint32_t slot)
{
    return fault_memory + HEAD + slot * RECORD;
}

static __force_inline void put16(uint8_t *at, uint32_t value)
{
    at[0] = value >> 8 & 0xFF;
    at[1] = value & 0xFF;
}

static __force_inline uint32_t get16(const uint8_t *at)
{
    return at[0] << 8 | at[1];
}

static __force_inline uint32_t up(uint32_t count, uint32_t most)
{
    return count < most ? count + 1 : most;
}

// The time now, the drive's low 16 bits and the seconds into it.
static __force_inline void stamp(uint8_t *at)
{
    put16(at, get16(fault_memory + 2));                         // (the head's drives, its low half)
    put16(at + 2, fault_state.seconds);
}

// A code's qualification, and the masks of the codes present and pending with it.
static void BANK2_CODE(set_state)(uint32_t id, uint32_t state)
{
    Faults &f = fault_state;
    uint32_t bit = 1u << id;
    fault_checks[id].state = state;
    f.present = state == CONFIRMED ? f.present | bit : f.present & ~bit;
    f.pending = state == PENDING ? f.pending | bit : f.pending & ~bit;
}

// ---- The memory ----

// A slot for a new record: an empty one, else the oldest gone one (by its last seen), else NONE (every slot
// present).
static uint32_t BANK2_CODE(free_slot)()
{
    uint32_t oldest = NONE, oldest_at = 0;
    for (uint32_t s = 0; s < RECORDS; s++)
    {
        const uint8_t *r = record(s);
        if (r[R_CODE] == 0)
            return s;
        uint32_t last = get16(r + R_LAST) << 16 | get16(r + R_LAST + 2);
        if (!(r[R_STATE] & PRESENT) && (oldest == NONE || last < oldest_at))
        {
            oldest = s;
            oldest_at = last;
        }
    }
    return oldest;
}

// A code confirmed: its record made (a new slot, its frame due) or moved on (its count, its causes, last seen).
static void BANK2_CODE(confirm)(uint32_t id)
{
    Check &c = fault_checks[id];
    set_state(id, CONFIRMED);
    c.timer = 0;
    fault_state.confirmed = fault_state.confirmed + 1;
    uint32_t slot = c.slot;
    if (slot == NONE)
    {
        slot = free_slot();
        if (slot == NONE)
        {
            uint8_t &lost = fault_memory[6];                    // every slot present: counted as lost
            lost = up(lost, 0xFF);
            return;
        }
        uint8_t *r = record(slot);
        if (r[R_CODE] != 0)
            fault_checks[r[R_CODE] - 1].slot = NONE;            // a gone record displaced
        r[R_CODE] = id + 1;
        r[R_CAUSE] = 0;
        r[R_COUNT] = 0;
        r[5] = 0;
        stamp(r + R_FIRST);
        fault_state.frames_due = fault_state.frames_due | 1u << slot;
        c.slot = slot;
    }
    uint8_t *r = record(slot);
    r[R_CAUSE] = r[R_CAUSE] | c.cause;
    r[R_STATE] = PRESENT | THIS_DRIVE;
    r[R_COUNT] = up(r[R_COUNT], 0xFF);
    r[R_DRIVES] = 0;
    stamp(r + R_LAST);
    store_soon(STORE_FAULTS);                                   // written at once, 10 s after the kind's last
}

// A code gone: its record so (last seen as it was).
static void BANK2_CODE(heal)(uint32_t id)
{
    Check &c = fault_checks[id];
    set_state(id, GONE);
    c.timer = 0;
    if (c.slot != NONE)
    {
        uint8_t *r = record(c.slot);
        r[R_STATE] = r[R_STATE] & ~PRESENT;
        fault_state.dirty = true;
    }
}

// The engine has run 30 s since the 8051's start: a drive. Each record's drives since it was last seen; a fault
// that heals at the next drive without it, gone; 1151's no-start held till a drive without one.
static void BANK2_CODE(drive_counted)()
{
    uint32_t drives = (get16(fault_memory) << 16 | get16(fault_memory + 2)) + 1;
    put16(fault_memory, drives >> 16);
    put16(fault_memory + 2, drives & 0xFFFF);
    bool any = false;
    for (uint32_t s = 0; s < RECORDS; s++)
    {
        uint8_t *r = record(s);
        if (r[R_CODE] == 0)
            continue;
        any = true;
        uint32_t id = r[R_CODE] - 1;
        const Check &c = fault_checks[id];
        r[R_DRIVES] = c.this_drive ? 0 : up(r[R_DRIVES], 0xFF);
        if (c.state == CONFIRMED && c.heal == BY_DRIVE && !c.this_drive)
            heal(id);
    }
    if (!fault_state.no_start_drive)
        fault_state.no_start = false;
    fault_state.dirty = fault_state.dirty || any;
}

// The 8051 started: a new drive (nothing seen yet in it); a pending code from nothing; lambda's loop closed again
// (format 33), so that 1123's check tests it again.
static void BANK2_CODE(drive_start)()
{
    Faults &f = fault_state;
    f.since_start = 0;
    f.drive_ticks = 0;
    f.seconds = f.sub = 0;
    f.no_start_drive = false;
    f.idle_ticks = f.map_rest = f.map_idle = f.one_side = f.mid_band = f.no_reference = 0;
    f.fuel_off = 0;
    fault_loop_open = 0;
    fault_idle_open = 0;                                        // (format 36: starting, not above the cut-off)
    for (uint32_t id = 0; id < FAULT_CHECKS; id++)
    {
        Check &c = fault_checks[id];
        c.this_drive = 0;
        if (c.state == PENDING)
        {
            set_state(id, ABSENT);
            c.cause = 0;
        }
    }
    for (uint32_t s = 0; s < RECORDS; s++)
    {
        uint8_t *r = record(s);
        r[R_STATE] = r[R_STATE] & ~THIS_DRIVE;
    }
}

// ---- The checks ----

// What the checks find this tick, by id: the causes tested and those seen (a byte a code), the codes seen, and
// those that qualified themselves (confirmed at once).
static constexpr uint32_t FOUND_WORDS = (FAULT_CHECKS + 3) / 4;

struct Found
{
    uint32_t tested[FOUND_WORDS];   // (words: cleared four codes at a time)
    uint32_t seen[FOUND_WORDS];
    uint32_t seen_mask, at_once;
};

static __force_inline uint32_t tested_of(const Found &f, uint32_t id)
{
    return f.tested[id >> 2] >> 8 * (id & 3) & 0xFF;
}

static __force_inline uint32_t seen_of(const Found &f, uint32_t id)
{
    return f.seen[id >> 2] >> 8 * (id & 3) & 0xFF;
}

static __force_inline void test(Found &f, uint32_t id, uint32_t causes)
{
    f.tested[id >> 2] |= causes << 8 * (id & 3);
}

static __force_inline void see(Found &f, uint32_t id, uint32_t cause)
{
    f.seen[id >> 2] |= cause << 8 * (id & 3);
    f.seen_mask |= 1u << id;
}

// A leaky count of events (each Cal_FaultEventAdd, each clean turn one less) and the clean turns since the last;
// whether it stands: confirmed at Cal_FaultEventLevel, then till Cal_FaultCleanTurns clean turns.
static bool BANK2_CODE(leaky)(uint32_t *leak, uint32_t *clean, uint32_t events, uint32_t turns, bool present)
{
    const Limits &l = limits;
    if (events != 0)
    {
        uint32_t n = *leak + events * l.event_add;
        *leak = n > 0xFFFF ? 0xFFFF : n;
        *clean = 0;
    }
    else
    {
        *leak = *leak > turns ? *leak - turns : 0;
        uint32_t c = *clean + turns;
        *clean = c > 0xFFFF ? 0xFFFF : c;
    }
    return *leak >= l.event_level || (present && *clean < l.clean_turns);
}

// A count of its own toward its time: seen past it, not tested while it runs (so a heal can't race it), tested
// clean once the condition goes.
static __force_inline void timed(Found &found, uint32_t id, uint32_t cause, uint32_t *count, bool off, uint32_t time)
{
    *count = off ? up(*count, 0xFFFF) : 0;
    if (*count >= time)
    {
        see(found, id, cause);
        found.at_once |= 1u << id;
    }
    else if (!off)
        test(found, id, cause);
}

// Format 36: IN6 at the level of both switches closed or of the idle contact (within Cal_FaultIn6Band): the idle
// contact closed as its wiring reads, whatever the control makes of it.
static bool BANK2_CODE(contact)(uint32_t code)
{
    const Limits &l = limits;
    uint32_t band = l.in6_band;
    for (uint32_t k = 0; k < 2; k++)
        if (code + band >= l.in6[k] && code <= l.in6[k] + band)
            return true;
    return false;
}

// IN6 between the throttle switches' levels: within Cal_FaultIn6Band of none of the four.
static bool BANK2_CODE(between)(uint32_t code)
{
    const Limits &l = limits;
    uint32_t band = l.in6_band;
    for (uint32_t k = 0; k < 4; k++)
        if (code + band >= l.in6[k] && code <= l.in6[k] + band)
            return false;
    return true;
}

// What the checks share this tick.
struct Now
{
    const uint8_t *bytes;
    const FaultCounts *counts;
    uint32_t speed, air;
    bool running, cranking, rest, warm, idle, full, starting, synced;
    bool fresh, rest_read;  // the codes read in the last Cal_FaultRestTicks, or at rest with no tooth since
    uint32_t cutoff;        // format 36: the overrun's cut-off speed (rpm: Overrun_CutSpeed by coolant)
};

static void BANK2_CODE(sensors)(const Now &v, Found &found)
{
    Faults &f = fault_state;
    const Limits &l = limits;
    const uint8_t *adc = v.counts->adc;
    bool running = v.running, cranking = v.cranking, rest = v.rest, warm = v.warm;
    if (!v.fresh)                                               // codes the 8051 has not read lately (no
    {                                                           //   input read once the engine stops but
        f.map_rest = f.map_idle = 0;                            //   Power_Up_Inputs'): none tested
        return;
    }
    if (running)                                                // 1111 the supply, running
    {
        test(found, FAULT_SUPPLY, C_HIGH | C_LOW);
        uint32_t b = adc[1];
        if (b > l.supply_high)
            see(found, FAULT_SUPPLY, C_HIGH);
        else if (b < l.supply_low)
            see(found, FAULT_SUPPLY, C_LOW);
    }
    if (!cranking)                                              // 1114, 1125 the NTCs, at rest too
    {
        test(found, FAULT_COOLANT, C_HIGH | C_LOW);
        test(found, FAULT_AIR_TEMP, C_HIGH | C_LOW);
        uint32_t coolant = adc[3], air = adc[2];
        if (coolant >= l.ntc_open)
            see(found, FAULT_COOLANT, C_HIGH);
        else if (coolant <= l.ntc_short)
            see(found, FAULT_COOLANT, C_LOW);
        if (air >= l.ntc_open)
            see(found, FAULT_AIR_TEMP, C_HIGH);
        else if (air <= l.ntc_short)
            see(found, FAULT_AIR_TEMP, C_LOW);
    }
    uint32_t afm = adc[0];
    if (running && v.speed > l.afm_rpm)                         // 1121 the AFM: 0 V over 600 rpm, 5 V at part
    {                                                           //   load
        test(found, FAULT_AFM, C_LOW);
        if (afm <= l.afm_low)
            see(found, FAULT_AFM, C_LOW);
    }
    if (running && !v.idle && !v.full)
    {
        test(found, FAULT_AFM, C_HIGH);
        if (afm >= l.afm_high)
            see(found, FAULT_AFM, C_HIGH);
    }
    if (f.fitted & 1u << FAULT_MAP)                             // 1132 the MAP: at a rail; at rest off the
    {                                                           //   air's pressure; at a warm idle its hose
        uint32_t map = adc[5];
        if (!cranking)
        {
            test(found, FAULT_MAP, C_HIGH | C_LOW);
            if (map >= l.map_high)
                see(found, FAULT_MAP, C_HIGH);
            else if (map <= l.map_low)
                see(found, FAULT_MAP, C_LOW);
        }
        if (rest && v.rest_read)                                // (the code read at rest: Power_Up_Inputs')
            timed(found, FAULT_MAP, C_OTHER, &f.map_rest, map < l.map_rest_low || map > l.map_rest_high, l.map_rest_ticks);
        else
            f.map_rest = 0;
        if (running && warm && v.idle)
            timed(found, FAULT_MAP, C_FOURTH, &f.map_idle, map > l.map_idle, l.map_idle_ticks);
        else
            f.map_idle = 0;
    }
    uint32_t gate = f.idle_raw ? v.cutoff : l.switch_rpm;       // (format 36: over the overrun's cut-off, the contact
    bool idle = f.idle_raw ? contact(adc[6]) : v.idle;          //   as IN6 reads, which its action leaves)
    if (running && v.speed > gate && v.air > l.idle_air)        // 1112 the idle contact: closed with the air over a
    {                                                           //   limit; IN6 between the levels
        test(found, FAULT_IDLE_SW, C_HIGH);
        if (idle)
            see(found, FAULT_IDLE_SW, C_HIGH);
    }
    if (!cranking)
    {
        test(found, FAULT_IDLE_SW, C_OTHER);
        if (between(adc[6]))
            see(found, FAULT_IDLE_SW, C_OTHER);
    }
    if (running && v.speed > l.switch_rpm && v.air < l.full_air && !v.idle // 1113 full load: closed with the air
        && (!v.full || f.full_ticks >= l.full_ticks))                     //   under a limit, once it has stood its
    {                                                                     //   settle (untested till then)
        test(found, FAULT_FULL_SW, C_HIGH);
        if (v.full)
            see(found, FAULT_FULL_SW, C_HIGH);
    }
}

static void BANK2_CODE(control)(const Now &v, Found &found)
{
    Faults &f = fault_state;
    const Limits &l = limits;
    const uint8_t *b = v.bytes;
    bool running = v.running, warm = v.warm;
    bool cut = bit(b, src.fuel_cut) || src.rev_limit < v.speed, boost_cut = bit(b, src.boost_cut);
    f.idle_ticks = running && v.idle && !v.full ? up(f.idle_ticks, 0xFFFF) : 0;
    uint32_t target = map_unit_owns.own[MAP_UNIT_IDLE_TARGET - MAP_UNIT_SPEED];
    if (warm && !cut && f.idle_ticks >= l.idle_ticks)           // 1122 idle: warm, the contact closed 5 s, no cut: off
    {                                                           //   its target with the valve's integrator at a limit,
        test(found, FAULT_IDLE, C_HIGH | C_LOW);                //   over it only with the air flow over a limit (a car
        uint32_t integrator = b[src.idle_int];                  //   rolling in gear holds it up on the valve's least)
        if (v.speed > target + l.idle_off && integrator <= src.idle_least && v.air * v.speed > l.idle_flow * 30000u)
            see(found, FAULT_IDLE, C_HIGH);                     //   (the flow: 0.1 mg x rpm / 30,000 in 0.1 g/s)
        else if (v.speed + l.idle_off < target && integrator >= src.idle_most)
            see(found, FAULT_IDLE, C_LOW);
    }
    bool closed = running && bit(b, src.closed);
    if (closed)                                                 // 1123 the mixture: lambda's integrator (or the
    {                                                           //   trims) at a limit; lean adds fuel
        test(found, FAULT_MIXTURE, C_HIGH | C_LOW | C_OTHER);
        uint32_t lam = b[src.lam_int];
        if (lam >= src.lam_most)
            see(found, FAULT_MIXTURE, C_HIGH);
        else if (lam == 0)
            see(found, FAULT_MIXTURE, C_LOW);
        int32_t trim = (int32_t)v.counts->trim_factor - 0x8000, most = (int32_t)src.trim_most;
        if (trim >= most || -trim >= most)
            see(found, FAULT_MIXTURE, C_OTHER | (trim > 0 ? C_HIGH : C_LOW));
        if (f.lambda)                                           // format 33: the integrator at its fuel-off limit
        {                                                       //   (0, the sensor rich) one up, above it one down;
            if (lam == 0)                                       //   at 1123's confirm time the loop held open this
            {                                                   //   drive (the fuel-on limit keeps it)
                f.fuel_off = up(f.fuel_off, 0xFFFF);
                if (f.fuel_off >= fault_checks[FAULT_MIXTURE].confirm)
                    fault_loop_open = 1;
            }
            else if (f.fuel_off != 0)
                f.fuel_off = f.fuel_off - 1;
        }
    }
    bool past = v.full || src.rpm_max < v.speed || src.load_max < v.air || src.map_max < b[src.in5];
    if (running && warm && !past && !cut && !boost_cut)         // 1124 the oxygen sensor: no signal (its middle
        timed(found, FAULT_OXYGEN, C_FOURTH, &f.mid_band, bit(b, src.mid_band), //   band) its confirm time on end with
              fault_checks[FAULT_OXYGEN].confirm);              //   the loop's other conditions (a switch crosses it:
    else                                                        //   neither seen nor clean)
        f.mid_band = 0;
    bool rich = !(b[src.pins] & 2);                             // ... or on one side 15 s, closed and steady (O2_LEAN
    bool same = rich == f.was_rich;                             //   low, the pins' bit 1)
    f.was_rich = rich;
    if (closed && v.counts->trim_steady >= src.trim_steady_min)
    {
        f.one_side = same ? up(f.one_side, 0xFFFF) : 0;
        if (f.one_side >= l.o2_stuck_ticks)
        {
            see(found, FAULT_OXYGEN, C_FIFTH);
            found.at_once |= 1u << FAULT_OXYGEN;
        }
        else
            test(found, FAULT_OXYGEN, C_FIFTH);
    }
    else
        f.one_side = 0;
    if (f.fitted & 1u << FAULT_KNOCK)                           // 1131 the knock line, as format 24 marks it
    {
        test(found, FAULT_KNOCK, C_OTHER);
        if (v.counts->knock_faulty)
            see(found, FAULT_KNOCK, C_OTHER);
    }
    if (f.fitted & 1u << FAULT_BOOST)                           // 1137 the overboost guard's cut, while it cuts
    {
        test(found, FAULT_BOOST, C_HIGH);
        if (boost_cut)
            see(found, FAULT_BOOST, C_HIGH);
    }
}

// The crank's and the P28's: from the counts' changes since the last tick, each at any time.
static void BANK2_CODE(events)(const Now &v, Found &found, bool turning, uint32_t references)
{
    Faults &f = fault_state;
    const Limits &l = limits;
    const FaultCounts &c = *v.counts;
    uint32_t now_turns = c.turns, now_missing = c.missing, now_crank = c.crank_faults;
    uint32_t turns = now_turns - f.seen_turns, refs = references - f.seen_refs;
    uint32_t missing = now_missing - f.seen_missing, crank = now_crank - f.seen_crank;
    f.seen_turns = now_turns;
    f.seen_refs = references;
    f.seen_missing = now_missing;
    f.seen_crank = now_crank;
    static_assert(FAULT_DME % 4 == 0 && FAULT_STORE / 4 + 1 == FOUND_WORDS, "1141 on: the last two words");
    found.tested[FAULT_DME / 4] = found.tested[FAULT_DME / 4 + 1] = 0x07070707; // (1141 on: each tested at any
                                                                // time, all its causes)
    found.at_once |= 1u << FAULT_REFERENCE | 1u << FAULT_SPEED | 1u << FAULT_CRANK | 1u << FAULT_EVENTS
                     | 1u << FAULT_TICK | 1u << FAULT_STORE | 1u << FAULT_DME;
    if ((missing | f.ref_leak) != 0 || f.ref_clean < l.clean_turns) // 1151: references missing, close together
        if (leaky(&f.ref_leak, &f.ref_clean, missing, turns, fault_checks[FAULT_REFERENCE].state == CONFIRMED))
            see(found, FAULT_REFERENCE, C_HIGH);
    f.no_reference = v.starting && turning && !v.synced && refs == 0 ? up(f.no_reference, 0xFFFF) : 0;
    if (f.no_reference >= l.no_start_ticks)                     // ... or cranking 2 s with none (a no-start),
    {                                                           //   held till a drive without one
        f.no_start = f.no_start_drive = true;
        f.no_reference = 0;
    }
    if (f.no_start)
        see(found, FAULT_REFERENCE, C_LOW);
    if (turning)                                                // 1152: two references with no tooth between
        f.refs_at_teeth = references;
    else if (references - f.refs_at_teeth >= 2)
    {
        see(found, FAULT_SPEED, C_HIGH);
        f.refs_at_teeth = references;
    }
    if ((crank | f.crank_leak) != 0 || f.crank_clean < l.clean_turns) // 1153: references off, close together
        if (leaky(&f.crank_leak, &f.crank_clean, crank, turns, fault_checks[FAULT_CRANK].state == CONFIRMED))
            see(found, FAULT_CRANK, C_HIGH);
    uint32_t beats = c.beat_stops, over = c.event_over, lost = c.event_lost, tick_lost = c.tick_lost;
    uint32_t tick_over = c.tick_over, again = c.again, store = store_faults();
    if (((beats ^ f.seen_beats) | (over ^ f.seen_over) | (lost ^ f.seen_lost) | (tick_lost ^ f.seen_tick_lost)
         | (tick_over ^ f.seen_tick_over) | (store ^ f.seen_store)) != 0)
    {
        if (beats != f.seen_beats)                              // 1141: stopped by the heartbeat
            see(found, FAULT_DME, C_LOW);
        if (over != f.seen_over)                                // 1161: L1's jobs over their bound, into a full
            see(found, FAULT_EVENTS, C_HIGH);                   //   queue
        if (lost != f.seen_lost)
            see(found, FAULT_EVENTS, C_LOW);
        if (tick_lost != f.seen_tick_lost)                      // 1162: pings into a full queue, ticks over
            see(found, FAULT_TICK, C_HIGH);                     //   their bound (run again too often below)
        if (tick_over != f.seen_tick_over)
            see(found, FAULT_TICK, C_LOW);
        if (store != f.seen_store)                              // 1163: a page read back wrong
            see(found, FAULT_STORE, C_HIGH);
        f.seen_beats = beats;
        f.seen_over = over;
        f.seen_lost = lost;
        f.seen_tick_lost = tick_lost;
        f.seen_tick_over = tick_over;
        f.seen_store = store;
    }
    if (again != f.seen_again || f.again_leak != 0)
    {
        uint32_t a = again != f.seen_again ? f.again_leak + (again - f.seen_again) * l.again_add : f.again_leak - 1;
        f.again_leak = a > 0xFFFF ? 0xFFFF : a;
        f.seen_again = again;
        if (f.again_leak >= l.again_level)
            see(found, FAULT_TICK, C_OTHER);
    }
}

// Each code seen, pending or present qualified (the others stay absent or gone). Not present, an up/down count of
// its ticks (its timer; the actions' first step, format 32): one up a tick seen, one down a tick that tests every
// cause it has clean, held by a tick that leaves one untested; pending above 0, absent again at 0 (nothing left),
// confirmed at its confirm time's count, or at once if it qualified itself. Present: seen again, its heal time from
// nothing; every cause it has tested clean, toward its heal time (FFFFh: at the next drive).
static void BANK2_CODE(qualify)(const Found &found)
{
    Faults &f = fault_state;
    uint32_t mask = (found.seen_mask | f.pending | f.present) & f.fitted;
    for (uint32_t id = 0; mask != 0; id++, mask >>= 1)
    {
        if (!(mask & 1))
            continue;
        Check &c = fault_checks[id];
        uint32_t seen = seen_of(found, id);
        if (seen != 0)
        {
            c.this_drive = 1;
            if (c.state == CONFIRMED)
            {
                c.timer = 0;                                    // present: seen now (a new cause, its record's
                c.cause = c.cause | seen;                       //   too)
                if (c.slot != NONE)
                {
                    uint8_t *r = record(c.slot);
                    r[R_CAUSE] = r[R_CAUSE] | seen;
                    stamp(r + R_LAST);
                }
                continue;
            }
            if (c.state != PENDING)
            {
                set_state(id, PENDING);
                c.timer = 0;
                c.cause = 0;
            }
            c.cause = c.cause | seen;
            c.timer = up(c.timer, 0xFFFF);
            if ((found.at_once >> id & 1) || c.timer >= c.confirm)
                confirm(id);
            continue;
        }
        if (c.state != PENDING && (c.state != CONFIRMED || c.heal == BY_DRIVE))
            continue;
        uint32_t tested = tested_of(found, id);
        if (tested == 0 || (c.cause & ~tested) != 0)            // a cause it has untested: held
            continue;
        if (c.state == PENDING)                                 // every cause it has, tested clean: one down, to
        {                                                       //   nothing at 0
            uint32_t count = c.timer;
            c.timer = count - 1;
            if (count <= 1)
            {
                set_state(id, ABSENT);
                c.cause = 0;
                c.timer = 0;
            }
        }
        else
        {
            c.timer = up(c.timer, 0xFFFF);                      // present: toward its heal time
            if (c.timer >= c.heal)
                heal(id);
        }
    }
}

// Format 35: the NTCs' substitutes, made each tick as the control's bytes (map_unit_adc gives them for a code past
// the checks' limits). A plausible code's byte is its own: the tick's copy has the snoop's byte made from it. The
// coolant's base is its last plausible byte this power-up, else the intake air's first plausible byte, else
// Cal_FaultSubCold; while 1114 is present and the engine runs it rises Cal_FaultSubRise a tick, to Cal_FaultSubWarm
// (a base over it kept as it is), and never falls back. The intake air's: its last plausible byte, else
// Cal_FaultSubCold.
static void BANK2_CODE(substitutes)(const Now &v)
{
    Faults &f = fault_state;
    const Limits &l = limits;
    volatile FaultSubstitutes &s = fault_substitutes;
    uint32_t air = v.counts->adc[2], coolant = v.counts->adc[3];
    if (air < l.ntc_open && air > l.ntc_short)
    {
        s.air = v.bytes[src.air];
        if (!f.air_seen)
            f.air_first = v.bytes[src.air];
        f.air_seen = true;
    }
    if (coolant < l.ntc_open && coolant > l.ntc_short)
    {
        f.coolant_seen = true;
        f.coolant_base = (uint32_t)v.bytes[src.coolant] << 16;
        f.coolant_rise = 0;
    }
    else
    {
        if (!f.coolant_seen)
            f.coolant_base = (uint32_t)(f.air_seen ? f.air_first : l.sub_cold) << 16;
        if (fault_checks[FAULT_COOLANT].state == CONFIRMED && v.running && f.coolant_rise < 0xFF0000)
            f.coolant_rise += l.sub_rise;
    }
    uint32_t warm = (uint32_t)l.sub_warm << 16, value = f.coolant_base;
    if (value < warm)
    {
        value += f.coolant_rise;
        if (value > warm)
            value = warm;
    }
    s.coolant = value >> 16 & 0xFF;
}

void BANK2_CODE(faults_tick)(const uint8_t *bytes, uint32_t now, const FaultCounts &counts)
{
    Faults &f = fault_state;
    if (!f.on)
        return;
    const Limits &l = limits;
    bool restarted = false;
    uint32_t starts = counts.starts, references = counts.references;
    if (starts != f.seen_starts)                                // the 8051 started: a new drive; 1141 if it ran
    {                                                           //   on till just then (a restart, not a key cycle)
        restarted = f.was_running && now - f.last_running < l.restart_ms * 1000u;
        f.seen_starts = starts;
        drive_start();
    }
    else
        f.since_start = up(f.since_start, 0xFFFF);
    if (++f.sub == A_SECOND)
    {
        f.sub = 0;
        f.seconds = up(f.seconds, 0xFFFF);
    }
    uint32_t teeth = counts.teeth;
    bool tooth = teeth != f.seen_teeth;
    f.no_tooth = tooth ? 0 : up(f.no_tooth, 0xFFFF);
    f.seen_teeth = teeth;
    if (counts.results != f.seen_results)                       // an input read since the last tick: at rest
    {                                                           //   if no tooth came with it
        f.seen_results = counts.results;
        f.read_ago = 0;
        f.read_teeth = teeth;
        f.read_at_rest = !tooth;
    }
    else
        f.read_ago = up(f.read_ago, 0xFFFF);
    Now v;
    v.bytes = bytes;
    v.counts = &counts;
    v.speed = map_unit_owns.own[0];
    v.air = map_unit_owns.own[MAP_UNIT_LOAD - MAP_UNIT_SPEED] & 0xFFFF;
    v.synced = counts.synced != 0;
    v.starting = bit(bytes, src.start);
    v.idle = bit(bytes, src.idle);
    v.full = bit(bytes, src.full);
    bool turning = f.no_tooth == 0;
    v.rest = f.no_tooth >= l.rest_ticks;
    v.running = v.synced && !v.starting;
    f.full_ticks = v.running && v.full ? up(f.full_ticks, 0xFFFF) : 0;
    v.cranking = v.starting && turning;
    v.warm = bytes[src.coolant] >= l.warm;
    v.rest_read = f.read_at_rest && teeth == f.read_teeth;
    v.fresh = f.read_ago < l.rest_ticks || v.rest_read;
    v.cutoff = f.idle_raw ? unit_map_value(src.cut_speed, bytes, map_unit_owns.own) : 0; // (format 36's)
    if (fault_substitutes.on)                                   // format 35: the NTCs' substitutes, each tick
        substitutes(v);
    bool running = v.running;
    Found found;
    for (uint32_t i = 0; i < FOUND_WORDS; i++)                  // (no initialiser: that would be a memset)
    {
        found.tested[i] = 0;
        found.seen[i] = 0;
    }
    found.seen_mask = found.at_once = 0;
    if (f.since_start >= l.settle)                              // the sensors and the control: 2 s after the
    {                                                           //   8051's start, none while it cranks
        sensors(v, found);
        control(v, found);
    }
    events(v, found, turning, references);
    if (restarted)                                              // 1141: restarted running
        see(found, FAULT_DME, C_HIGH);
    qualify(found);
    if (running && f.drive_ticks < l.drive_ticks && ++f.drive_ticks == l.drive_ticks)
        drive_counted();
    f.was_running = running;
    if (running)
        f.last_running = now;
    fault_idle_open = f.idle_raw && running && fault_checks[FAULT_IDLE_SW].state == CONFIRMED // format 36: the
                      && v.speed > v.cutoff;                                                 //   contact taken open
    uint32_t *own = map_unit_owns.own;                          // the faults present now, the log's
    own[MAP_UNIT_FAULTS - MAP_UNIT_SPEED] = f.present & 0xFFFF;
    own[MAP_UNIT_FAULTS + 1 - MAP_UNIT_SPEED] = f.present >> 16;
    f.runs = f.runs + 1;
}

uint32_t BANK2_CODE(faults_frames_due)()
{
    uint32_t due = fault_state.frames_due;
    fault_state.frames_due = 0;
    return due;
}

uint8_t *BANK2_CODE(faults_frame)(uint32_t slot)
{
    return record(slot) + R_FRAME;
}

uint32_t BANK2_CODE(faults_frame_list)()
{
    return fault_state.frame_at;
}

uint32_t BANK2_CODE(faults_live)()
{
    return fault_state.on ? fault_state.live_at : 0;
}

// ---- As the board starts ----

// The memory empty, every code absent, lambda's loop not held; the work on with format 31's table (u + 194 on: the
// code table's, the limits', the frame's list's and the live copy's addresses): each check's entry, the limits
// taken (format 32's Cal_FaultFullTicks after them), format 33's action on lambda if 1123 is fitted. Once: small.
void __attribute__((cold)) BANK2_CODE(faults_start)(const uint8_t *rom, uint32_t u, const FaultPlaces *places,
                                                    const FaultCounts &counts)
{
    Faults &f = fault_state;
    f.rom = rom;
    f.on = false;
    f.fitted = 0;
    f.runs = f.present = f.confirmed = f.pending = 0;
    f.since_start = f.drive_ticks = f.seconds = f.sub = 0;
    f.no_tooth = f.no_reference = f.idle_ticks = f.full_ticks = f.map_rest = f.map_idle = f.one_side = f.mid_band = 0;
    f.ref_leak = f.ref_clean = f.crank_leak = f.crank_clean = f.again_leak = 0;
    f.was_rich = f.was_running = f.no_start = f.no_start_drive = f.dirty = false;
    f.frames_due = 0;
    f.lambda = false;
    f.fuel_off = 0;
    fault_loop_open = 0;
    fault_substitutes.on = 0;                                   // (format 35: none till the table's taken)
    fault_idle_open = 0;                                        // (format 36: the contact as it reads)
    f.idle_raw = false;
    f.coolant_base = f.coolant_rise = 0;
    f.coolant_seen = f.air_seen = false;
    for (uint32_t i = 0; i < BYTES; i++)                        // (a loop: no memset in the bank)
        fault_memory[i] = 0;
    for (uint32_t id = 0; id < FAULT_CHECKS; id++)
    {
        Check &c = fault_checks[id];
        c.state = ABSENT;
        c.cause = c.this_drive = 0;
        c.slot = NONE;
        c.timer = c.code = c.confirm = c.heal = 0;
    }
    if (at(MARK + 4) < FAULTS_FORMAT || places == nullptr)
        return;
    const uint32_t *from = (const uint32_t *)places;            // (a loop: a struct's copy is a memcpy, in flash)
    uint32_t *to = (uint32_t *)&src;
    for (uint32_t i = 0; i < sizeof(FaultPlaces) / 4; i++)
        to[i] = from[i];
    f.seen_starts = counts.starts;
    f.seen_teeth = counts.teeth;
    f.seen_refs = f.refs_at_teeth = counts.references;
    f.seen_turns = counts.turns;
    f.seen_missing = counts.missing;
    f.seen_crank = counts.crank_faults;
    f.seen_beats = counts.beat_stops;
    f.seen_over = counts.event_over;
    f.seen_lost = counts.event_lost;
    f.seen_tick_lost = counts.tick_lost;
    f.seen_tick_over = counts.tick_over;
    f.seen_again = counts.again;
    f.seen_store = store_faults();
    f.codes_at = word(u + 194);
    uint32_t limits_at = word(u + 196);
    f.frame_at = word(u + 198);
    f.live_at = word(u + 200);
    if (f.codes_at == 0 || limits_at == 0 || f.frame_at == 0)
        return;
    for (uint32_t id = 0; id < FAULT_CHECKS; id++)              // the codes, the confirm times, the heal times
    {
        Check &c = fault_checks[id];
        c.code = word(f.codes_at + 2 * id);
        c.confirm = word(f.codes_at + CONFIRMS + 2 * id);
        c.heal = word(f.codes_at + HEALS + 2 * id);
        if (c.code != 0)
            f.fitted |= 1u << id;
    }
    uint8_t *bytes = (uint8_t *)&limits;                        // the bytes, then the words (the struct's order)
    for (uint32_t i = 0; i < LIMIT_BYTES; i++)
        bytes[i] = at(limits_at + i);
    uint16_t *words = &limits.afm_rpm;
    for (uint32_t i = 0; i < LIMIT_WORDS; i++)
        words[i] = word(limits_at + LIMIT_BYTES + 2 * i);
    limits.full_ticks = at(MARK + 4) >= FULL_SETTLE ? word(limits_at + LIMIT_BYTES + 2 * LIMIT_WORDS) : 0;
    f.lambda = at(MARK + 4) >= FAULTS_LAMBDA_FORMAT && (f.fitted >> FAULT_MIXTURE & 1);
    f.idle_raw = at(MARK + 4) >= FAULTS_IDLE_FORMAT && src.cut_speed != 0 && (f.fitted >> FAULT_IDLE_SW & 1);
    if (at(MARK + 4) >= FAULTS_SUBSTITUTE_FORMAT)               // format 35: the substitutes' words and their start
    {
        uint32_t w = limits_at + LIMIT_BYTES + 2 * LIMIT_WORDS + 2;
        limits.sub_cold = word(w);
        limits.sub_warm = word(w + 2);
        limits.sub_rise = word(w + 4);
        f.coolant_base = limits.sub_cold << 16;
        fault_substitutes.open = limits.ntc_open;
        fault_substitutes.shorted = limits.ntc_short;
        fault_substitutes.air = fault_substitutes.coolant = limits.sub_cold & 0xFF;
        fault_substitutes.air_at = src.air;
        fault_substitutes.coolant_at = src.coolant;
        fault_substitutes.on = 1;
    }
    f.on = true;
}

static_assert(sizeof(Limits) == 18 + 2 * 23, "Limits: Cal_FaultSettleTicks' 18 bytes, 19 words, format 32's word, "
                                             "format 35's three");
static_assert(sizeof(FaultPlaces) % 4 == 0, "FaultPlaces: copied a word at a time");

// ---- The store's third kind (store.h) ----

bool STORE_CODE(faults_differs)(uint32_t, const uint8_t *)
{
    return fault_state.dirty;
}

void STORE_CODE(faults_copy)(uint32_t, uint8_t *out)
{
    fault_state.dirty = false;
    const uint32_t *from = (const uint32_t *)fault_memory;
    uint32_t *to = (uint32_t *)out;                             // (words: the post's page is aligned)
    for (uint32_t i = 0; i < BYTES / 4; i++)
        to[i] = from[i];
}

// As the board starts (the store's load): the memory as stored, each record's code in its state again; a new drive,
// so none of them seen in it yet (the P28 powered with the 8051 sees no start of its own), the loaded bytes too
// (the live copy's).
void STORE_CODE(faults_restore)(uint8_t *in)
{
    for (uint32_t s = 0; s < RECORDS; s++)
        in[HEAD + RECORD * s + R_STATE] = in[HEAD + RECORD * s + R_STATE] & ~THIS_DRIVE;
    for (uint32_t i = 0; i < BYTES; i++)
        fault_memory[i] = in[i];
    for (uint32_t s = 0; s < RECORDS; s++)
    {
        const uint8_t *r = record(s);
        uint32_t code = r[R_CODE];
        if (code == 0 || code > FAULT_CHECKS || fault_checks[code - 1].code == 0)
            continue;
        Check &c = fault_checks[code - 1];
        c.slot = s;
        c.cause = r[R_CAUSE];
        set_state(code - 1, r[R_STATE] & PRESENT ? CONFIRMED : GONE);
        c.timer = 0;
        if (code - 1 == FAULT_REFERENCE && c.state == CONFIRMED && (c.cause & C_LOW))
            fault_state.no_start = true;                        // (a no-start: held till a drive without one)
    }
}

// L2's (the store's tick, at the PC's clear): the memory empty but its counts (the drives; the clears one more),
// every code from nothing (a fault still there comes back after its confirm time), lambda's loop closed again as at a
// drive's start (format 33).
void STORE_CODE(faults_clear)()
{
    uint32_t clears = get16(fault_memory + 4) + 1;
    for (uint32_t i = 4; i < BYTES; i++)
        fault_memory[i] = 0;
    put16(fault_memory + 4, clears > 0xFFFF ? 0xFFFF : clears);
    for (uint32_t id = 0; id < FAULT_CHECKS; id++)
    {
        Check &c = fault_checks[id];
        c.state = ABSENT;
        c.cause = 0;
        c.slot = NONE;
        c.timer = 0;
    }
    fault_state.present = fault_state.pending = 0;
    fault_state.no_start = false;
    fault_state.dirty = false;
    fault_state.fuel_off = 0;
    fault_loop_open = 0;
    fault_idle_open = 0;                                        // (format 36: 1112 absent, the contact as it reads)
}

// What the memory means: the CRC-32 of its layout word, the code table's codes (not their times) and the
// frame's list; 0 with the work off (never a key: a CRC of 0 is taken as 1).
uint32_t STORE_CODE(faults_key)()
{
    if (!fault_state.on)
        return 0;
    uint32_t crc = ~0u;
    for (uint32_t i = 0; i < 4; i++)
        crc = store_crc(crc, LAYOUT >> 8 * i);
    for (uint32_t i = 0; i < CONFIRMS; i++)                     // (the codes alone)
        crc = store_crc(crc, at(fault_state.codes_at + i));
    for (uint32_t i = 0; i < FRAME; i++)
        crc = store_crc(crc, at(fault_state.frame_at + i));
    crc = ~crc;
    return crc == 0 ? 1 : crc;
}

// pair_faults: ticks worked, the faults present now and pending (bits by id, hex), confirmations, faults lost to a
// full memory.
bool faults_get_parameter(const char *name, char *value, size_t value_size)
{
    if (!streq(name, "pair_faults"))
        return false;
    const Faults &f = fault_state;
    snprintf(value, value_size, "%u %05x %05x %u %u", (unsigned)f.runs, (unsigned)f.present, (unsigned)f.pending,
             (unsigned)f.confirmed, (unsigned)fault_memory[6]);
    return true;
}

#endif // FEATURE_MAP_UNIT
