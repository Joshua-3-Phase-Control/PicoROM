// The paired core's P28 side (FEATURE_MAP_UNIT, format 6; DME-Sim's docs/Paired_Core.md and its
// experiments/rom_pair). The 8051 keeps no tooth chain: every tooth's interrupt fetches INT1's vector
// (0013h), which this code writes before each tooth. A RETI for a plain tooth; for a tooth with an
// event an AJMP to the event's handler, its operand first, then its opcode; one event for another
// through a RETI. So the 8051 never fetches a torn instruction, whenever a write lands.
//
//   the crank: each fetch of the vector (0013h, then 0014h) is a tooth, a tooth the reference's
//   handler takes inline too (it jumps there). The reference's fetch (0003h, then 0004h) gives the
//   latch's tooth by the teeth's times, as rom_lean takes it by the speed input's level: the fall
//   before the latch, or the next one if it comes within TEETH_HIGH/256 of a tooth. Fall 1, the
//   turn's first tooth, is that tooth, or the one after it (the table's FALL1_AHEAD, from MARK_G:
//   the 944's). Each reference checks the count: a tooth off either way is the tie (a fall and the latch
//   at one angle come in either order); more, a fault, and the count starts again there.
//   the firings: from one fetch of slot 0's tooth to the next, the microsecond timer gives the
//   firing's length (P, in the 8051's cycles: halved), and from it the tooth period (T64) with its
//   trend, as rom_lean's Main_Capture and Period_T64 make them, bit for bit.
//   the events: the crank events (rom_lean's reads and the KLR strobe) on their teeth; the coil's
//   charge and its spark's arm on the teeth the plans give. After each spark (the 8051's fetch of
//   000Eh, then 000Fh) the next charge and arm are planned: rom_lean's Spark_Plan and Dwell_Plan bit
//   for bit, with this core's tooth rules (no coil event on a crank event's tooth or a firing's teeth
//   0 and 1, each moved a tooth earlier) and Spark_Set's rule for a read the spark would meet (a
//   tooth or two late). The advance is what the 8051 sent last (task 8: its byte in the unit's RAM),
//   the charge's time task 3's output there. A tooth before the arm, its load again from the latest
//   tooth period (Spark_Fresh), then its two operands, then the vector.
//   a late run: one that finds more than one new tooth. The teeth after the first ran what the
//   vector held: an event again, counted, or a RETI where one was due, which goes on the next tooth
//   (the arm with its load from there, or not at all if that is too near its spark; a charge only
//   before its arm: else its spark fires not, the 8051's arm refusing an uncharged coil).
//   the heartbeat: each ping (0016h, then 0017h) moves the operand at 0017h on. A mark at 0006h
//   says the count is lost (the engine stopped, or the P28 back after its heartbeat stood still): a
//   RETI in the vector, and no event till the next reference. The 8051's start (0000h) is the same,
//   with the flags' opcodes as built.
//   TH0's loads (format 7, step 3.2): the pulse is the unit's task 9 (unit_tasks.cpp: rom_lean's
//   Fuel_Maps or Fuel_Crank, Fuel_Time and Fuel_Mode), and its load (TH0's start and its overflows)
//   goes into the 8051's code, where TH0 takes it: at a pulse's end (Inj_Next) and while TH0 is
//   stopped (Inj_Apply). Each place has two copies of its two loads and a switch, an AJMP's operand:
//   the copies not in use are written, then the switches turned to them, so the 8051 takes a whole
//   load, the old one or the new. A pulse made again (task 10) after its reference is not written.
//   the injection's phase (format 8, step 3.3): which reference injects (rom_lean's Inj720Odd, the
//   unit's now): the reference's AJMP's operand (0004h) to the injection's handler or past it, and
//   at the turn's first firing Main_Turn's SJMP to its pulse, none, or a cut. The counts the unit's
//   tasks keep for the 8051 (SlewCount, InjCount, CrankDecay: their own, never sent) change at the
//   8051's marks, where rom_lean's 8051 changed them: Pair_Stopped (the engine stopped: the count
//   lost as at 0006h, InjCount 0, CrankDecay cleared), Mark_Firing (each firing: SlewCount one less,
//   to 0) and Mark_Counted (an injection counted, starting or cut: InjCount one more, to 80h). The
//   8051's start clears the unit's RAM, as its Power_Up its own.
//
// The live bytes, all it writes of the 8 KiB (DME-Sim checks that): 0013h and 0014h, the arm's two
// operands, 0017h, and the opcodes at Pair_Synced (SETB NotSynced till in step, then CLR) and at
// Pair_Fault (CLR ToothFault; SETB after a fault); format 7, the copies' loads and the switches;
// format 8, 0004h, Main_Turn's SJMP's offset and the phase's opcode at Main_Reference.
// Format 6's table (Co_Pair), which the mark's 13th and 14th bytes point to, gives their places, the
// crank events and the coil's numbers; format 7's adds the switches and the copies; format 8's the
// phase's places and what its cut reads, then the counts' marks and places.
//
// Format 11 (DME-Sim's docs/Paired_Lambda_Idle.md): the whole tick is the unit's, from the 8051's
// ping, and the 8051 does no serial (the log is the unit's: pair_log_read). The 8051 tells its pins
// (the oxygen sensor's comparators, the A/C request) by calling one of eight marks in page 0, two
// bytes apart; their byte goes into the unit's copy of the RAM. Its flags' opcodes at the reference go
// (NotSynced, ToothFault and the phase's copy are the unit's, in its copy, for the log), and the starting
// injection's SJMP at Main_Firing is live: the unit points it at the injection or past it at slot 0's
// tooth, by its start mode.

#include "pair.h"

#if defined(FEATURE_MAP_UNIT)

#include <stdio.h>

#include "map_unit.h"
#include "str_util.h"
#include "unit_ram.h"
#include "unit_tasks.h"

static constexpr uint32_t MARK = 0x2000;
static constexpr uint32_t FORMAT = 6;
static constexpr uint32_t INJECTION = 7; // the format with TH0's loads (pair_pulse)
static constexpr uint32_t PHASE = 8;     // the format with the injection's phase (the reference's switch)
static constexpr uint32_t PIPELINE = 9;  // the format with the pipeline on the unit (pair_control)
static constexpr uint32_t TICKS = 10;    // the format with the tick's work too (its take's layout)
static constexpr uint32_t WHOLE = 11;    // the format with the whole tick at the ping, and the log
static constexpr uint32_t RETI = 0x32, AJMP = 0x01, SETB_BIT = 0xD2, CLR_BIT = 0xC2, MOV_C_BIT = 0xA2;
static constexpr uint32_t REFERENCE = 0x03, VECTOR = 0x13;
static constexpr uint32_t MOST_EVENTS = 8;
static constexpr uint32_t MOST_MISSED = 4;
static constexpr uint32_t MOVES = 2; // reads moved off a spark at once: the last spark's, the next's
static constexpr int32_t NONE = -0x40000000;
static constexpr uint32_t LOOPS = 600; // a plan's loop's turns at most (rom_lean's end long before)

// A tooth's event: nothing (a RETI), a crank event (its index in the table), the charge, the arm.
enum : uint32_t
{
    NOTHING = 0xFF,
    CHARGE = 0xFE,
    ARM = 0xFD,
};

// The spark's place (Spark_Place): its tooth in its firing (R1, signed: 1 its fall 1) and the time
// from that tooth's fall to the spark (R3:R5:R4, 1/64 cycle).
struct Place
{
    int32_t tooth;
    uint32_t time;
};

// The spark's arm's load (Spark_Load): TL0's start and the overflows to the spark.
struct Load
{
    uint32_t start, overflows;
};

// A read moved off a spark: its tooth's count, the count it goes on, the read (NONE: none).
struct Move
{
    int32_t from, to;
    uint32_t event;
};

struct Pair
{
    // Read by DME-Sim (its tests take these offsets): the counts, then the coil's plan.
    volatile uint32_t teeth_seen;   // 0: the vector's fetches
    volatile uint32_t references;   // 4
    volatile uint32_t syncs;        // 8: counts taken at a latch (the first, after a fault or a loss)
    volatile uint32_t faults;       // 12: references that found the count off by more than a tooth
    volatile uint32_t sparks;       // 16
    volatile uint32_t plans;        // 20: sparks planned
    volatile uint32_t refreshes;    // 24: arms whose load was made again a tooth before
    volatile uint32_t late_runs;    // 28: runs that found more than one new tooth
    volatile uint32_t repeats;      // 32: teeth that ran the event of the tooth before them again
    volatile uint32_t missed;       // 36: events whose tooth passed before the vector held them
    volatile uint32_t lost_sparks;  // 40: planned sparks that did not come (the coil started again)
    volatile uint32_t beat;         // 44: the heartbeat
    uint32_t plan_adv;              // 48: the advance the planned spark takes (half-teeth, a byte)
    uint32_t sload, shi;            // 52, 56: the arm's operands as planned, or refreshed
    int32_t arm_at, charge_at;      // 60, 64: the counts of the arm's and the charge's teeth, or NONE
    int32_t spark_at;               // 68: the count of the planned spark's tooth (its fall), or NONE
    uint32_t count;                 // 72: the count of the latest tooth (teeth seen)
    uint32_t t64, p;                // 76, 80: the tooth period (1/64 cycle), the firing (cycles)
    uint32_t synced;                // 84: in step (0 or 1)
    uint32_t most_loops;            // 88: a plan's loop's most turns (LOOPS ends one)
    uint32_t dj;                    // 92: the charge's teeth before its spark's (Dwell_Plan's, kept)
    int32_t sarm;                   // 96: the arm's tooth in its spark's firing
    uint32_t pos;                   // 100: the latest tooth's place in its turn (0: fall 1)
    volatile uint32_t pulses;       // 104: TH0's loads written (format 7)
    volatile uint32_t late_pulses;  // 108: pulses made again after their reference: none written
    uint32_t load;                  // 112: the last load written: TH0's start, its overflows << 8
    uint32_t next_skips;            // 116: the 720-degree phase: the next reference injects not (format 8)
    volatile uint32_t turn_cuts;    // 120: turns whose injection a cut left out
    volatile uint32_t turns;        // 124: the turns' first firings decided
    // The image and its table (Co_Pair).
    volatile uint8_t *rom;
    bool on;
    uint32_t teeth, firings, turn, events;
    uint32_t event_tooth[MOST_EVENTS], event_handler[MOST_EVENTS];
    uint32_t charge_handler, arm_handler, arm_start, arm_overflows, synced_op, fault_op;
    uint32_t ping, lost, spark;
    uint32_t spark_b, arm_ofs, arm_min, margin, floor_teeth, recip32, teeth_high, fall1_ahead;
    uint32_t charge_l, charge_h, adv_at;
    // The crank.
    uint32_t previous;              // the last page-0 read's A0-A7
    uint32_t t_last, t_prev, timed; // the last two teeth's times (us), and how many of them hold
    uint32_t tif, firing;           // the latest tooth in its firing (1 on), its firing (0 on)
    bool fault;                     // ToothFault's opcode SETB (kept till the 8051's start)
    // The firings.
    bool stamped;
    uint32_t stamp_count, stamp_time;
    uint32_t p_old;
    bool fire_valid;
    // The coil.
    bool coil_on, spark_due;
    Move moves[MOVES];              // reads moved off a spark, till they have run: the plan after a
                                    //   spark comes before its moved read
    uint32_t missed_events[MOST_MISSED], missed_count; // crank events a late run passed, to place
    // The vector.
    uint32_t vector;                // what it holds: an event (NOTHING: a RETI)
    uint32_t steered;               // the count the vector was last written after
    // TH0's loads (format 7): each switch (an AJMP's operand) and its two copies (MOV TH0,#start; MOV
    // InjHi,#overflows), at the pulse's end (Inj_Next) and while TH0 is stopped (Inj_Apply).
    bool inject;
    uint32_t next_switch, next_copy[2], apply_switch, apply_copy[2];
    uint32_t side;                  // the copies in use
    uint32_t built;                 // TH0's load as built (the copies' first): start, overflows << 8
    uint32_t pulse_reference;       // the references by the turn's pulse (a pulse made again after: late)
    // The injection's phase (format 8): the reference's two handlers (0004h, its AJMP's operand, the
    // switch), Main_Turn's SJMP and its three targets, the phase's copy's opcode at Main_Reference, and
    // what the turn's cut reads: Cal_RevLimit, the speed byte's RAM address, the bits of StartMode,
    // FuelCut and BoostCut.
    bool phased;
    uint32_t ref_inject, ref_skip, turn_at, turn_due, turn_speed, turn_cut, phase_op;
    uint32_t rev_limit, rpm_at, start_bit, fuel_cut_bit, boost_cut_bit;
    // The counts the unit keeps for the 8051 (format 8): the marks (page 0: the engine stopped, a
    // firing, an injection counted) and the RAM bytes and the bit they change.
    uint32_t stop_mark, fire_mark, count_mark, slew_at, inj_at, decay_bit;
    // Format 11: the eight marks of the 8051's pins (the first's A0-A7) and their byte's address; the
    // bits the unit keeps for the log (NotSynced, ToothFault, Inj720Odd, RefMissing, InjHold); the
    // starting injection's SJMP and its two targets (the injection, past it); the references at the
    // last turn's first firing (the reference's check).
    bool whole;
    uint32_t bits_mark, pins_at, synced_bit, fault_bit, phase_bit, missing_bit, hold_bit;
    uint32_t inject_at, inject_to, skip_to, ref_mark;
};

static Pair pair UNIT_DATA = {};

// DME-Sim's checks of the routines (pair_test): their inputs and outputs.
struct Case
{
    uint32_t in[8];
    uint32_t out[8];
};

static Case pair_case UNIT_DATA = {};

static __force_inline uint32_t at(uint32_t address)
{
    return pair.rom[address & 0xFFFF];
}

static __force_inline uint32_t word(uint32_t address)
{
    return at(address) << 8 | at(address + 1);
}

// ---------------------------------------------------------------------------------------------------
// rom_lean's arithmetic, bit for bit (its routines' names), each step noted with its instructions.

// Period_T64: R5:R4 = 2 x R7:R6 x RECIP32 / 65536, rounded but for the low bytes' product.
static uint32_t UNIT_CODE(period_t64)(uint32_t p, uint32_t recip)
{
    uint32_t r7 = p >> 8 & 0xFF, r6 = p & 0xFF, lo = recip & 0xFF, hi = recip >> 8 & 0xFF;
    uint32_t x = r7 * lo;                                       // high x low
    uint32_t r0 = x & 0xFF, r2 = x >> 8;
    x = r6 * hi;                                                // low x high
    uint32_t a = (x & 0xFF) + r0;
    r0 = a & 0xFF;
    a = (x >> 8) + r2 + (a >> 8);
    r2 = a & 0xFF;
    uint32_t r3 = a >> 8;                                       // CLR A; RLC A
    x = r7 * hi;                                                // high x high
    a = (x & 0xFF) + r2;
    r2 = a & 0xFF;
    r3 = ((x >> 8) + r3 + (a >> 8)) & 0xFF;                     // R3:R2:R0, the top 24 bits
    a = r2 + ((r0 + 0x80) >> 8);                                // T32, rounded
    uint32_t t32 = ((r3 + (a >> 8)) & 0xFF) << 8 | (a & 0xFF);
    return t32 << 1 & 0xFFFF;                                   // doubled
}

// Spark_Place_A: from the advance (half-teeth, signed) and SPARK_B, the spark's tooth and the time
// from its fall (its fraction x T64 / 256).
static Place UNIT_CODE(spark_place)(uint32_t advance, uint32_t spark_b, uint32_t t64)
{
    advance &= 0xFF;
    uint32_t r2 = advance >> 1 | (advance & 0x80);              // MOV C,ACC.7; RRC A: its whole teeth
    uint32_t r0 = (advance & 1) << 7;                           // CLR A; RRC A: its half (C clear)
    uint32_t a = (spark_b & 0xFF) - r0;                         // the place's fraction ...
    uint32_t borrow = a >> 31;
    r0 = a & 0xFF;
    uint32_t r1 = ((spark_b >> 8 & 0xFF) + 1 - r2 - borrow) & 0xFF; // ... and its tooth
    uint32_t r4 = r0 * (t64 & 0xFF) >> 8;                       // MUL AB: B
    uint32_t x = r0 * (t64 >> 8 & 0xFF);
    a = (x & 0xFF) + r4;
    uint32_t r5 = ((x >> 8) + (a >> 8)) & 0xFF;
    return {(int32_t)(int8_t)r1, r5 << 8 | (a & 0xFF)};          // R3 = 0
}

// Lead_Short: the time (R3:R5:R4) under ARM_MIN_S: the spark too soon after its arm's tooth.
static __force_inline bool lead_short(uint32_t time, uint32_t arm_min)
{
    return (time >> 16) == 0 && (time & 0xFFFF) < (arm_min * 64 & 0xFFFF);
}

// Tooth_Back: a tooth earlier, its time a tooth period more (24 bits).
static __force_inline void tooth_back(Place *place, uint32_t t64)
{
    place->tooth--;
    place->time = (place->time + t64) & 0xFFFFFF;
}

// Spark_Load: L = time / 64 + 0.5 - ARM_OFS_S; TL0 from -L, (L + 255) / 256 overflows.
static Load UNIT_CODE(spark_load)(uint32_t time, uint32_t arm_ofs)
{
    time = (time + 32) & 0xFFFFFF;                              // R4 + 32, carried
    time = time << 2 & 0xFFFFFF;                                // two bits left
    uint32_t l = ((time >> 8) - arm_ofs) & 0xFFFF;              // R3:R5 less ARM_OFS_S
    return {(0x100 - (l & 0xFF)) & 0xFF, ((l >> 8) + ((l & 0xFF) != 0)) & 0xFF};
}

// Dwell_Plan to Dwell_Tooth: the charge's teeth before the spark's tooth (DJ), stepped from dj until
// the spark's fraction's time and DJ tooth periods cover the charge's time (4 cycles: Charge, with
// DW_OFS), no more than (P - MARGIN) x 64 before the spark (fewer teeth then).
static uint32_t UNIT_CODE(dwell_teeth)(Place place, uint32_t dj, uint32_t t64, uint32_t p, uint32_t margin,
                                       uint32_t charge)
{
    uint32_t r7 = dj & 0xFF;
    uint32_t time = (place.time + r7 * (t64 & 0xFF) + (r7 * (t64 >> 8 & 0xFF) << 8)) & 0xFFFFFF;
    uint32_t limit = ((p - margin) & 0xFFFF) * 64;              // (P - MARGIN) x 64: R6:R2:R0
    for (uint32_t loops = 0; loops < LOOPS; loops++)
    {
        if (loops > pair.most_loops)
            pair.most_loops = loops;
        uint32_t next;
        if ((time >> 8 & 0xFFFF) < charge)                      // Dwell_Up: short of the charge's time
        {
            next = (time + t64) & 0xFFFFFF;                     // a tooth more ...
            if (limit >= next)                                  // ... unless that passes the limit
            {
                r7 = (r7 + 1) & 0xFF;
                time = next;
                continue;
            }
        }
        if (r7 == 0)                                            // Dwell_Down
            break;
        next = (time - t64) & 0xFFFFFF;                         // a tooth less: within the limit and
        if (limit >= time && (next >> 8 & 0xFFFF) < charge)     //   short without it: this
            break;
        r7 = (r7 - 1) & 0xFF;                                   // Dwell_Less
        time = next;
    }
    return r7;
}

// Main_Capture's tooth period: T64 of this firing (P), on by its change from the last firing's if
// that is a whole 4 cycles a tooth or more (256 or more either way: the plans a firing and more
// ahead) and the last firing's length held; none if the last firing is within 128 cycles of this.
static uint32_t UNIT_CODE(capture_t64)(uint32_t p, uint32_t *p_old, bool valid, uint32_t recip)
{
    uint32_t t64 = period_t64(p, recip), old = *p_old;
    *p_old = p;
    if (((((p - old) & 0xFFFF) + 0x80) & 0xFFFF) < 0x100)      // within 128 cycles: no trend
        return t64;
    uint32_t change = (t64 - period_t64(old, recip)) & 0xFFFF;
    if (!valid || change >> 8 == 0 || change >> 8 == 0xFF)
        return t64;
    return (t64 + change) & 0xFFFF;
}

// ---------------------------------------------------------------------------------------------------
// The tooth rules and the plans.

// A tooth (in a firing, signed: 1 its fall 1) in 1 to the teeth a firing.
static uint32_t UNIT_CODE(in_firing)(int32_t tooth)
{
    while (tooth < 1)
        tooth += (int32_t)pair.teeth;
    while (tooth > (int32_t)pair.teeth)
        tooth -= (int32_t)pair.teeth;
    return (uint32_t)tooth;
}

// The crank event on a tooth of a firing (1 on), or NOTHING.
static uint32_t UNIT_CODE(event_on)(uint32_t tooth)
{
    for (uint32_t i = 0; i < pair.events; i++)
        if (pair.event_tooth[i] == tooth)
            return i;
    return NOTHING;
}

// No coil event on a crank event's tooth, nor on a firing's teeth 0 and 1 (the latch's: rom_lean's).
static bool UNIT_CODE(barred)(int32_t tooth)
{
    uint32_t t = in_firing(tooth);
    return t == pair.teeth || t == 1 || event_on(t) != NOTHING;
}

// The arm of the spark at the advance (Spark_Plan to Spark_Set): its tooth the last ARM_MIN_S or more
// before the spark that the rules allow, and its load. The read the spark would meet goes late
// (Spark_Set's rule: a tooth late if the spark is in its tooth, two if in its tooth's last quarter;
// one if in the last quarter of the tooth before): its tooth in late_tooth (NONE: none), the teeth in
// late. The strobe, the table's last event, never moves.
static Load UNIT_CODE(arm_plan)(uint32_t advance, int32_t *arm, int32_t *late_tooth, uint32_t *late)
{
    Place place = spark_place(advance, pair.spark_b, pair.t64);
    int32_t spark = place.tooth;
    for (uint32_t loops = 0; lead_short(place.time, pair.arm_min) && loops < LOOPS; loops++)
        tooth_back(&place, pair.t64);
    for (uint32_t loops = 0; barred(place.tooth) && loops < LOOPS; loops++)
        tooth_back(&place, pair.t64);
    *arm = place.tooth;
    *late_tooth = NONE;
    *late = 0;
    bool last_quarter = ((pair.spark_b & 0xFF) ^ (advance & 1) << 7) >= 0xC0;
    for (int32_t k = 0; k <= 1; k++)                            // a read on the spark's tooth, or the next
    {
        uint32_t event = event_on(in_firing(spark + k));
        if (event == NOTHING || event == pair.events - 1 || spark + k <= place.tooth)
            continue;
        uint32_t teeth = k == 0 ? (last_quarter ? 2 : 1) : (last_quarter ? 1 : 0);
        if (teeth != 0)
        {
            *late_tooth = spark + k;
            *late = teeth;
        }
        break;
    }
    return spark_load(place.time, pair.arm_ofs);
}

// The charge for the spark at the advance (Dwell_Plan), whose arm is on tooth arm: DJ teeth before
// the spark's tooth (DJ kept), at least FLOOR_TEETH, before the arm, earlier as the rules say.
static int32_t UNIT_CODE(charge_plan)(uint32_t advance, int32_t arm, uint32_t charge)
{
    Place place = spark_place(advance, pair.spark_b, pair.t64);
    pair.dj = dwell_teeth(place, pair.dj, pair.t64, pair.p, pair.margin, charge);
    uint32_t teeth = pair.dj < pair.floor_teeth ? pair.floor_teeth : pair.dj;
    int32_t tooth = (int32_t)(int8_t)((place.tooth - (int32_t)teeth) & 0xFF);
    if (tooth >= arm)
        tooth = arm - 1;
    for (uint32_t loops = 0; barred(tooth) && loops < LOOPS; loops++)
        tooth--;
    return tooth;
}

// Spark_Fresh: the arm's load again from the tooth period now, its tooth kept (three back at most),
// unless the spark then comes under ARM_MIN_S from it (the load as it was).
static void UNIT_CODE(refresh)()
{
    Place place = spark_place(pair.plan_adv, pair.spark_b, pair.t64);
    for (uint32_t back = 4;;)
    {
        if (place.tooth == pair.sarm)
        {
            if (lead_short(place.time, pair.arm_min))
                return;
            Load load = spark_load(place.time, pair.arm_ofs);
            pair.sload = load.start;
            pair.shi = load.overflows;
            pair.refreshes++;
            return;
        }
        tooth_back(&place, pair.t64);
        if (--back == 0)
            return;
    }
}

// The arm moved to tooth arm (a late run passed its own): its load from there, unless that is too
// near its spark or barred. False if none.
static bool UNIT_CODE(arm_from)(int32_t arm)
{
    Place place = spark_place(pair.plan_adv, pair.spark_b, pair.t64);
    if (arm > place.tooth || barred(arm))
        return false;
    while (place.tooth > arm)
        tooth_back(&place, pair.t64);
    if (lead_short(place.time, pair.arm_min))
        return false;
    Load load = spark_load(place.time, pair.arm_ofs);
    pair.sload = load.start;
    pair.shi = load.overflows;
    pair.sarm = arm;
    return true;
}

// ---------------------------------------------------------------------------------------------------
// The image's live bytes.

static __force_inline void write(uint32_t address, uint32_t value)
{
    pair.rom[address & 0xFFFF] = value;
}

// An 8051 bit (its bit address) set or cleared in the unit's copy of the RAM, one byte's write (the
// pipeline's level gives back only the bits its job changed, so this one stands).
static void UNIT_CODE(own_bit)(uint32_t address, bool on)
{
    uint32_t byte = 0x20 + (address >> 3 & 0x0F), mask = 1u << (address & 7), value = map_unit_ram(byte);
    map_unit_ram_write(byte, on ? value | mask : value & ~mask);
}

static void UNIT_CODE(own_sync)();

// The flags' opcodes at the 8051's reference: NotSynced's CLR in step, else SETB; ToothFault's SETB
// after a fault. Format 9: NotSynced in the unit's copy too (the pipeline's task 11 reads it, sent by
// no task now). Format 11: no opcodes; NotSynced and ToothFault in the unit's copy alone (the log's).
static void UNIT_CODE(write_flags)()
{
    if (!pair.whole)
    {
        write(pair.synced_op, pair.synced ? CLR_BIT : SETB_BIT);
        write(pair.fault_op, pair.fault ? SETB_BIT : CLR_BIT);
    }
    else
        own_bit(pair.fault_bit, pair.fault);
    own_sync();
}

static __force_inline uint32_t handler_of(uint32_t event)
{
    return event == CHARGE ? pair.charge_handler : event == ARM ? pair.arm_handler : pair.event_handler[event];
}

// The vector to hold event: a RETI (one byte), or an AJMP to its handler. From a RETI its operand,
// then its opcode; from another event in the same page (the image keeps the handlers in one) its
// operand alone, one byte; from one in another page through a RETI. Whenever the 8051 fetches it,
// it reads a whole instruction, the old one or the new.
static void UNIT_CODE(steer)(uint32_t event)
{
    if (event == pair.vector)
        return;
    uint32_t was = pair.vector;
    pair.vector = event;
    if (event == NOTHING)
    {
        write(VECTOR, RETI);
        return;
    }
    uint32_t handler = handler_of(event), opcode = AJMP | (handler >> 8 & 7) << 5;
    if (was != NOTHING && (handler_of(was) ^ handler) >> 8 == 0)
    {
        write(VECTOR + 1, handler & 0xFF);
        return;
    }
    if (was != NOTHING)
        write(VECTOR, RETI);
    write(VECTOR + 1, handler & 0xFF);
    write(VECTOR, opcode);
}

// TH0's load (format 7): a copy's two operands (MOV TH0,#start at +2, MOV InjHi,#overflows at +5).
static __force_inline void write_load(uint32_t copy, uint32_t load)
{
    write(copy + 2, load & 0xFF);
    write(copy + 5, load >> 8 & 0xFF);
}

// The load into the copies not in use, then each switch (one byte) turned to them: the 8051 loads a
// whole pair, the old or the new, whenever the writes land (a copy is written again only at the
// next pulse, long after the 8051 left it).
static void UNIT_CODE(load_pulse)(uint32_t load)
{
    uint32_t side = pair.side ^ 1;
    write_load(pair.next_copy[side], load);
    write_load(pair.apply_copy[side], load);
    write(pair.next_switch + 1, pair.next_copy[side] & 0xFF);
    write(pair.apply_switch + 1, pair.apply_copy[side] & 0xFF);
    pair.side = side;
    pair.load = load;
}

// As the 8051 starts: both copies' loads and the switches as built.
static void UNIT_CODE(loads_as_built)()
{
    for (uint32_t side = 0; side < 2; side++)
    {
        write_load(pair.next_copy[side], pair.built);
        write_load(pair.apply_copy[side], pair.built);
    }
    write(pair.next_switch + 1, pair.next_copy[0] & 0xFF);
    write(pair.apply_switch + 1, pair.apply_copy[0] & 0xFF);
    pair.side = 0;
    pair.load = pair.built;
    pair.pulse_reference = pair.references;
}

// ---------------------------------------------------------------------------------------------------
// The injection's 720-degree phase (format 8): which reference injects, as rom_lean's Inj720Odd said
// it: every other one; none while starting (each firing's own, the 8051's); after a turn's cut, the
// next one not; after start mode, the next one. The reference's AJMP goes to the injection's handler
// or to the one that leaves it out (0004h, one byte); the 8051's Main_Turn makes the pulse, or none,
// or counts a cut, as its SJMP's operand says (one byte).

// An 8051 bit as the unit's tasks keep it (sent, or their own: StartMode's task 12, the cuts' 11).
static __force_inline bool ram_bit(uint32_t address)
{
    return map_unit_ram(0x20 + (address >> 3 & 0x0F)) >> (address & 7) & 1;
}

// The reference's AJMP to the injection's handler, or past it.
static __force_inline void write_reference()
{
    write(REFERENCE + 1, (pair.next_skips ? pair.ref_skip : pair.ref_inject) & 0xFF);
}

// The phase's copy (Inj720Odd, the stream's): Main_Reference's SETB or CLR; format 11, the unit's
// copy of the bit (the log's).
static void UNIT_CODE(write_phase)()
{
    if (pair.whole)
        own_bit(pair.phase_bit, pair.next_skips != 0);
    else
        write(pair.phase_op, pair.next_skips ? SETB_BIT : CLR_BIT);
}

// At each reference (rom_lean's Main_Reference: CPL Inj720Odd; set while starting): the next one's,
// and its copy.
static void UNIT_CODE(phase_at_reference)()
{
    pair.next_skips = ram_bit(pair.start_bit) ? 1 : pair.next_skips ^ 1;
    write_reference();
    write_phase();
}

// The turn's first firing, at slot 0's tooth (before the 8051's Main_Turn, rom_lean's Turn_Cut there):
// starting, its pulse (each firing's); a turn 720 degrees leaves out, none; else a cut (over
// Cal_RevLimit, the overrun's or the overboost guard's) leaves the next reference out, counted at the
// 8051's mark there (Mark_Counted); else its pulse.
// Format 9: no SJMP; the decision goes with the pipeline's job (TURN_PULSE, TURN_NONE or TURN_CUT).
enum : uint32_t
{
    TURN_PULSE, // its pulse
    TURN_NONE,  // 720 degrees leave its reference out: none
    TURN_CUT,   // a cut: the next reference left out, the injection counted
};

static uint32_t UNIT_CODE(turn)()
{
    uint32_t target = pair.turn_due, decision = TURN_PULSE;
    if (!ram_bit(pair.start_bit))
    {
        if (pair.next_skips)
        {
            target = pair.turn_speed;
            decision = TURN_NONE;
        }
        else if (at(pair.rev_limit) < map_unit_ram(pair.rpm_at) || ram_bit(pair.fuel_cut_bit)
                 || ram_bit(pair.boost_cut_bit))                // SUBB A,Rpm; ORL C,FuelCut; ORL C,BoostCut
        {
            pair.next_skips = 1;
            pair.turn_cuts++;
            write_reference();
            if (pair.whole)
                own_bit(pair.phase_bit, true);                  // (format 10: the take's SETB)
            target = pair.turn_cut;
            decision = TURN_CUT;
        }
    }
    if (pair.turn_at != 0)
        write(pair.turn_at + 1, (target - pair.turn_at - 2) & 0xFF); // the SJMP's offset
    pair.turns++;
    return decision;
}

// Format 10: from the pipeline's level (task 12); format 11, from the tick's commit (L2, L1 held off).
// The reference's handler not between.
void UNIT_CODE(pair_start_over)()
{
    if (!pair.phased)
        return;
    __asm volatile("cpsid i" ::: "memory");
    pair.next_skips = 0;                                        // rom_lean's CLR Inj720Odd
    write_reference();
    if (pair.whole)
        own_bit(pair.phase_bit, false);
    __asm volatile("cpsie i" ::: "memory");
}

// The counts at the 8051's marks (format 8), as rom_lean's 8051 kept them there: a firing's
// (Main_Turn's Turn_Start), an injection counted (starting, there; a turn's cut, Main_Turn_Cut), the
// engine stopped (Tick_Stop).
static void UNIT_CODE(at_firing)()
{
    uint32_t slew = map_unit_ram(pair.slew_at);
    if (slew != 0)                                              // MOV A,SlewCount; JZ; DEC SlewCount
        map_unit_ram_write(pair.slew_at, slew - 1);
}

static void UNIT_CODE(at_counted)()
{
    uint32_t count = map_unit_ram(pair.inj_at);
    if (count < 0x80)                                           // MOV A,InjCount; JB ACC.7; INC InjCount
        map_unit_ram_write(pair.inj_at, count + 1);
}

static void UNIT_CODE(at_stopped)()
{
    map_unit_ram_write(pair.inj_at, 0);                         // MOV InjCount,#00h
    uint32_t byte = 0x20 + (pair.decay_bit >> 3 & 0x0F);        // CLR CrankDecay
    map_unit_ram_write(byte, map_unit_ram(byte) & ~(1u << (pair.decay_bit & 7)));
}

// As the 8051 starts: the next reference left out (Power_Up's SETB Inj720Odd), Main_Turn's pulse made.
static void UNIT_CODE(phase_as_built)()
{
    pair.next_skips = 1;
    write_reference();
    write_phase();
    if (pair.turn_at != 0)                                      // (format 9: no SJMP)
        write(pair.turn_at + 1, (pair.turn_due - pair.turn_at - 2) & 0xFF);
}

bool UNIT_CODE(pair_pulse)(uint32_t start, uint32_t overflows, bool again)
{
    if (!pair.inject)
        return true;
    if (again && pair.references != pair.pulse_reference)
    {
        pair.late_pulses++;                                     // its reference came: it started
        return false;
    }
    if (!again)
        pair.pulse_reference = pair.references;
    load_pulse((start & 0xFF) | (overflows & 0xFF) << 8);
    pair.pulses++;
    return true;
}

// ---------------------------------------------------------------------------------------------------
// The pipeline (format 9; DME-Sim's docs/Paired_Pipeline.md): rom_lean's per-firing work, run by the
// unit on an interrupt of its own below the bus's (SPARE_IRQ_0, L1: the bus's interrupts break into it).
// The bus level queues its jobs, which it does in turn, one a run (the next one pended again):
//   a firing (slot 0's tooth): the firing's length into FirePeriod (Main_Capture's; past 16 bits the
//   speed byte, Capture_Long's), the turn's start (Turn_Start's slew count; starting, the injection the
//   8051 makes counted; a turn's cut counted), then at the rate (each firing under Cal_PipeRpm, else the
//   turn's first) the stages: the speed's count and the idle spark (task 14), the advance's sum (4) and
//   rest (11), the plans' advance (Main_Advance's sum: task 8's byte), and at the turn's first firing,
//   unless cut or left out, the pulse (9);
//   a tick (the 8051's ping; format 10, format 11's is L2's): the speed byte at most the teeth a firing /
//   the ticks since slot 0's tooth (Main_Tick's, as the engine stops), and the heartbeat, written here so
//   that it stands still if this level does;
//   the tick's values sent (task 13; format 11, the tick's commit): the pulse waiting for its reference
//   made again (task 10), under Cal_RemakeRpm and not starting (Main_Byte_Again's);
//   start mode's guard period (task 12): BoostCount one less, to 0;
//   the engine stopped (Pair_Stopped; the 8051's Power_Up_Inputs ends with it too): InjCount 0,
//   CrankDecay cleared, then the stages with no firing (FirePeriod FFFFh: the count skipped) and the
//   pulse, as rom_lean's Tick_Stop and Power_Up_Inputs set its stages (the first after the 8051's start
//   the AFM's filter the AFM too, as Power_Up_Inputs set it: format 10);
//   format 10 (DME-Sim's docs/Paired_Tick.md), the tick's work (task 15, from task 12's end): the
//   switches, the enrichments, the dynamic spark's filter, the AFM's rise, the load and its byte, the
//   warm-up's weight, their results counted for the take (the 8051's lambda starts on the count).
// A job takes the unit's copy of the 8051's RAM at its start and gives back the bytes it changed at its
// end, each in one go with the interrupts held off, so that a window's task never tears them; a job the
// 8051's start or the engine's stop overtook gives nothing. Then its results go to the 8051's take:
// into the copy of Take_A or Take_B the switch doesn't point to, the switch turned to it, then the
// count at Pair_Count moved on, so that the 8051 takes a whole copy, the old or the new, whenever the
// writes land.
//
// The levels (format 11; DME-Sim's docs/Paired_Levels.md): the whole tick is L2's, on SPARE_IRQ_1 at C0h
// (USB's interrupt and the others that run from flash beside it), below L1's events (80h) and the bus
// level. It works on a copy of its own and commits with L1 held off (its enable bit); if an L1 job
// committed while it ran, it runs again on that job's results, so that every outcome is that of whole
// jobs in some order. Until it commits, nothing it did reaches anything but its copy: at the commit its
// bits given back, its take, the heartbeat (if L1 has taken every job queued before the tick began), its
// counts, start mode's end for the phase, and the pulse made again queued for L1, which runs it as it is
// let go; then the log's frame from the unit's copy.

static constexpr uint32_t PAIR_IRQ = 26;    // SPARE_IRQ_0: L1, the engine's events, at 80h
static constexpr uint32_t TICK_IRQ = 27;    // SPARE_IRQ_1: L2, the tick (format 11), at C0h
static constexpr uint32_t JOBS = 8;         // L1's queue's length (a power of two)
static constexpr uint32_t PINGS = 4;        // L2's (a power of two)
static constexpr uint32_t EVENT_BOUND = 65; // us: an L1 job's end from its push (a tooth at 7,000 rpm)
static constexpr uint32_t TICK_BOUND = 1000; // us: a tick's end from its ping

enum : uint32_t
{
    JOB_FIRING,
    JOB_TICK,
    JOB_SENT,
    JOB_GUARD,
    JOB_STOP,
    JOB_TICKWORK,
};

// A job as the bus level queued it: its kind, the 8051's starts by then; a firing's length (cycles; 0,
// none: a stamp alone), whether it is the turn's first and the turn's decision, and (format 11) start
// mode at its tooth, as the starting injection's branch took it; a tick's gap; when it was pushed (us).
struct Job
{
    uint32_t kind, generation, length, first, decision, starting, at;
};

struct Control
{
    volatile uint32_t runs;      // the stages run
    volatile uint32_t takes;     // results given to the 8051's take
    volatile uint32_t dropped;   // jobs the 8051's start or a stop overtook
    // L1's queue: the bus level pushes (and L2's commit, the pulse made again), L1 takes.
    Job queue[JOBS];
    volatile uint32_t in, out;
    volatile uint32_t generation; // the 8051's starts and the engine's stops
    volatile bool stop_due;       // the engine stopped: its run due (never queued, so never lost)
    volatile uint32_t stop_at;    // when it was asked for (us)
    // The image's places and numbers.
    bool pipelined;
    uint32_t count_at, switch_at, copy[2], pipe_rpm, remake_rpm;
    uint32_t fire_l, fire_h, advance_at, dyn_at, inj_l, inj_h, dead_at, flags_at, boost_at;
    uint32_t speed_task, advance_task, rest_task, pulse_task, again_task, speed_k;
    uint32_t tick_task, load_at, accel_at, after_at, idle_bit, full_bit, afm_filt_at, afm_at; // format 10
    uint32_t start_task, lambda_task, idle_task, tick_at, lam_at, high_l, high_h; // format 11
    uint32_t stream_ticks, stream_list;                         //   the log's: Cal_StreamTicks, its list
    // The state.
    uint32_t gap;                // pings since slot 0's tooth, to 255 (rom_lean's TickGap)
    uint32_t side, count;        // the take's copy in use and its count (0 as the 8051 starts)
    uint32_t turn_pulses, all_pulses;
    uint32_t ticks;              // the tick's work's results given (0 as the 8051 starts)
    uint32_t duties;             // format 11: the valve's duties made (0 as the 8051 starts)
    bool duty_due;               // format 11: the stop asked for one (Tick_Stop's IdleDuty)
    bool pulse_made;             // a turn's pulse made since the 8051's start
    bool pinged;                 // a ping since the 8051's start (its first tick's work after it)
    bool fresh;                  // no stop's run since the 8051's start (the AFM's filter at it)
};

// The levels' counts (docs/Paired_Levels.md), read by DME-Sim (P28MapUnit: these offsets) and by the PC
// (the parameters pair_events and pair_tick).
struct Levels
{
    volatile uint32_t event_jobs;   // 0: L1's jobs done (stale ones too, and the stop's runs)
    volatile uint32_t event_lost;   // 4: jobs pushed into its full queue
    volatile uint32_t event_most;   // 8: its queue's most jobs waiting at once
    volatile uint32_t event_us;     // 12: a job's longest time from its push to its end (us)
    volatile uint32_t event_over;   // 16: jobs that ended over EVENT_BOUND us after their push
    volatile uint32_t ticks;        // 20: L2's ticks committed
    volatile uint32_t again;        // 24: ticks run again (an L1 job committed while they ran)
    volatile uint32_t tick_late;    // 28: pings that found a tick not yet done
    volatile uint32_t tick_lost;    // 32: pings into its full queue
    volatile uint32_t tick_us;      // 36: a tick's longest time from its ping to its end (us)
    volatile uint32_t tick_over;    // 40: ticks that ended over TICK_BOUND us after their ping
    volatile uint32_t tick_dropped; // 44: ticks the 8051's start or a stop overtook
};

// L2's queue (format 11): the pings, which the bus level pushes and L2 takes: the ticks since slot 0's
// tooth, the 8051's starts by then, the ping's time (us).
struct Ping
{
    uint32_t gap, generation, at;
};

struct Pings
{
    Ping queue[PINGS];
    volatile uint32_t in, out;   // out moves on as a tick is done
};

static Control control UNIT_DATA = {};
static Levels levels UNIT_DATA = {};
static Pings pings UNIT_DATA = {};
alignas(4) static uint8_t work[0x80] UNIT_DATA = {};        // an L1 job's copy of the 8051's RAM
alignas(4) static uint8_t before[0x80] UNIT_DATA = {};      // ... as it took it
alignas(4) static uint8_t tick_work[0x80] UNIT_DATA = {};   // L2's: the tick's, then the frame's
alignas(4) static uint8_t tick_before[0x80] UNIT_DATA = {};

static __force_inline uint32_t now_us()
{
    return *(volatile uint32_t *)0x40054028;                            // the timer's TIMERAWL
}

static __force_inline void pend()
{
    *(volatile uint32_t *)0xE000E200 = 1u << PAIR_IRQ;                  // the NVIC's ISPR
}

static __force_inline void pend_tick()
{
    *(volatile uint32_t *)0xE000E200 = 1u << TICK_IRQ;
}

// L1 held off (its enable bit, the NVIC's ICER; the barriers so that none is taken after), then let go
// (ISER): a job pended meanwhile runs at once.
static __force_inline void hold_events()
{
    *(volatile uint32_t *)0xE000E180 = 1u << PAIR_IRQ;
    __asm volatile("dsb\n\tisb" ::: "memory");
}

static __force_inline void let_events_go()
{
    *(volatile uint32_t *)0xE000E100 = 1u << PAIR_IRQ;
}

static void UNIT_CODE(push)(uint32_t kind, uint32_t length, uint32_t first, uint32_t decision, uint32_t starting = 0)
{
    uint32_t waiting = control.in - control.out;
    if (waiting >= JOBS)
    {
        levels.event_lost++;
        return;
    }
    if (waiting + 1 > levels.event_most)
        levels.event_most = waiting + 1;
    Job &job = control.queue[control.in & (JOBS - 1)];
    job.kind = kind;
    job.generation = control.generation;
    job.length = length;
    job.first = first;
    job.decision = decision;
    job.starting = starting;
    job.at = now_us();
    control.in = control.in + 1;
    pend();
}

// The engine stopped: the jobs queued or under way are stale (they give nothing), and the stop's own
// run comes next, whether the queue has room or not (a level held up long enough to fill it).
static void UNIT_CODE(stop_pipeline)()
{
    control.generation = control.generation + 1;
    control.stop_due = true;
    control.stop_at = now_us();
    pend();
}

// Format 11: the ping's tick for L2.
static void UNIT_CODE(push_ping)(uint32_t gap)
{
    uint32_t waiting = pings.in - pings.out;
    if (waiting >= PINGS)
    {
        levels.tick_lost++;
        return;
    }
    if (waiting != 0)
        levels.tick_late++;
    Ping &ping = pings.queue[pings.in & (PINGS - 1)];
    ping.gap = gap;
    ping.generation = control.generation;
    ping.at = now_us();
    pings.in = pings.in + 1;
    pend_tick();
}

// An 8051 bit or byte in a job's copy of the RAM (L1's work, L2's tick_work).
static __force_inline bool copy_bit(const uint8_t *ram, uint32_t address)
{
    return ram[0x20 + (address >> 3 & 0x0F)] >> (address & 7) & 1;
}

static __force_inline uint8_t &copy_byte(uint8_t *ram, uint32_t address)
{
    return ram[address & 0x7F];
}

static __force_inline bool work_bit(uint32_t address)
{
    return copy_bit(work, address);
}

static __force_inline uint8_t &work_byte(uint32_t address)
{
    return copy_byte(work, address);
}

// 32 words from `from` to `to`, four a step (LDM, STM; DME-Sim maps no boot ROM, so no memcpy).
static __force_inline void copy_words(uint32_t *to, const uint32_t *from)
{
    const uint32_t *end = from + 0x20;
    __asm volatile("1:\n\t"
                   "ldmia %[f]!, {r3, r4, r5, r6}\n\t"
                   "stmia %[t]!, {r3, r4, r5, r6}\n\t"
                   "cmp %[f], %[e]\n\t"
                   "bne 1b"
                   : [f] "+l"(from), [t] "+l"(to)
                   : [e] "l"(end)
                   : "r3", "r4", "r5", "r6", "cc", "memory");
}

// The unit's copy of the 8051's RAM into `into` in one go (the interrupts held off), the snoop's bytes
// put over it, as a task's snooped inputs are; `kept` (unless null) keeps it as taken.
static void UNIT_CODE(take_inputs)(uint8_t *into, uint8_t *kept)
{
    __asm volatile("cpsid i" ::: "memory");
    copy_words((uint32_t *)into, map_unit_ram_words());
    __asm volatile("cpsie i" ::: "memory");
    map_unit_snooped_over(into);
    if (kept != nullptr)
        copy_words((uint32_t *)kept, (const uint32_t *)into);
}

// The words of a job's copy it changed (`now` against `was`), by their index, into `changed`; their
// number. A function of its own, two words a turn, so that it runs in the low registers.
static uint32_t UNIT_CODE(changed_words)(uint8_t *changed, const uint32_t *now, const uint32_t *was)
{
    uint8_t *c = changed;
    for (uint32_t i = 0; i < 0x20; i += 2)
    {
        uint32_t a = now[i], b = now[i + 1];
        if (a != was[i])
            *c++ = i;
        if (b != was[i + 1])
            *c++ = i + 1;
    }
    return c - changed;
}

// The bits a job changed (`job` against `kept`, as it took them) back into the unit's copy in one go
// (the interrupts held off), unless the 8051's start or a stop overtook it; true if given.
static bool UNIT_CODE(give_back)(uint32_t generation, const uint8_t *job, const uint8_t *kept)
{
    const uint32_t *now = (const uint32_t *)job, *was = (const uint32_t *)kept;
    uint8_t changed[0x20];
    uint32_t changes = changed_words(changed, now, was);
    uint32_t *ram = map_unit_ram_words();
    bool given = false;
    __asm volatile("cpsid i" ::: "memory");
    if (generation == control.generation)
    {
        for (uint32_t k = 0; k < changes; k++)
        {
            uint32_t i = changed[k], bits = now[i] ^ was[i];
            ram[i] = (ram[i] & ~bits) | (now[i] & bits);
        }
        given = true;
    }
    __asm volatile("cpsie i" ::: "memory");
    if (!given)
        control.dropped++;
    return given;
}

// The opcode that copies an 8051 bit of the unit's copy into the 8051's: SETB or CLR.
static __force_inline uint32_t bit_op(uint32_t address)
{
    return map_unit_ram(0x20 + (address >> 3 & 0x0F)) >> (address & 7) & 1 ? SETB_BIT : CLR_BIT;
}

// The results into the 8051's take: the copy the switch doesn't point to (dme.asm's Take_A; format 11:
// MOV Taken, DeadNow and PulseFlags,#n; SETB or CLR IdleSwitch and StartMode; MOV R2, R3, R4, R6 and
// R7,#n: the valve's duties made, the turn's pulses, all pulses, the high phase; format 10:
// MOV Taken, Rpm, Advance, FirePeriodL, FirePeriodH, InjTimeL, InjTimeH, DeadNow, PulseFlags, Load,
// AccelEnrich, AfterStart and AdvanceDyn,#n; SETB or CLR FuelCut and BoostCut; SETB Inj720Odd after a
// cut, else MOV C,Inj720Odd; SETB or CLR IdleSwitch and FullLoadSwitch; MOV R2, R3 and R4,#n: the
// tick's work's results' count, the turn's pulses' and all pulses'), then the switch turned to it and
// the count moved on (Pair_Count's operand), unless the 8051's start overtook them.
static void UNIT_CODE(give_take)(uint32_t generation, bool cut)
{
    uint32_t side = control.side ^ 1, copy = control.copy[side], count = (control.count + 1) & 0xFF;
    write(copy + 2, count);
    if (pair.whole)                                             // format 11: what the 8051 itself uses: the
    {                                                           //   MAP pulse's dead time, idle switch and
        write(copy + 5, map_unit_ram(control.dead_at));         //   start mode, the pulse's flags and counts,
        write(copy + 8, map_unit_ram(control.flags_at));        //   the valve's duties and high phase
        write(copy + 9, bit_op(control.idle_bit));
        write(copy + 11, bit_op(pair.start_bit));
        write(copy + 14, control.duties & 0xFF);
        write(copy + 16, control.turn_pulses & 0xFF);
        write(copy + 18, control.all_pulses & 0xFF);
        write(copy + 20, map_unit_ram(control.high_l));
        write(copy + 22, map_unit_ram(control.high_h));
    }
    else
    {
        write(copy + 5, map_unit_ram(pair.rpm_at));
        write(copy + 8, map_unit_ram(control.advance_at));
        write(copy + 11, map_unit_ram(control.fire_l));
        write(copy + 14, map_unit_ram(control.fire_h));
        write(copy + 17, map_unit_ram(control.inj_l));
        write(copy + 20, map_unit_ram(control.inj_h));
        write(copy + 23, map_unit_ram(control.dead_at));
        write(copy + 26, map_unit_ram(control.flags_at));
        write(copy + 29, map_unit_ram(control.load_at));
        write(copy + 32, map_unit_ram(control.accel_at));
        write(copy + 35, map_unit_ram(control.after_at));
        write(copy + 38, map_unit_ram(control.dyn_at));
        write(copy + 39, bit_op(pair.fuel_cut_bit));
        write(copy + 41, bit_op(pair.boost_cut_bit));
        write(copy + 43, cut ? SETB_BIT : MOV_C_BIT);
        write(copy + 45, bit_op(control.idle_bit));
        write(copy + 47, bit_op(control.full_bit));
        write(copy + 50, control.ticks & 0xFF);
        write(copy + 52, control.turn_pulses & 0xFF);
        write(copy + 54, control.all_pulses & 0xFF);
    }
    __asm volatile("cpsid i" ::: "memory");
    if (generation == control.generation)
    {
        write(control.switch_at + 1, copy & 0xFF);
        write(control.count_at + 1, count);
        control.side = side;
        control.count = count;
        control.takes++;
    }
    __asm volatile("cpsie i" ::: "memory");
}

static __force_inline void count_injection()
{
    uint8_t &count = work_byte(pair.inj_at);
    if (count < 0x80)                                                   // JB ACC.7; INC InjCount
        count++;
}

// Main_Capture: the firing's length into FirePeriod; past 16 bits (Capture_Long: the engine under about
// 229 rpm) FFFFh, and the speed byte the times the length goes into SPEED_K + half of it (Speed_Count
// then keeps it).
static void UNIT_CODE(firing_length)(uint32_t length)
{
    if (length <= 0xFFFF)
    {
        work_byte(control.fire_l) = length & 0xFF;
        work_byte(control.fire_h) = length >> 8;
        return;
    }
    if (length > 0xFFFFFF)
        length = 0xFFFFFF;
    uint32_t rest = (control.speed_k + (length >> 1)) & 0xFFFFFF, count = 0;  // R2:R1:R0
    while (rest >= length)                                              // SUBB x 3; JC Capture_Long_Set
    {
        rest -= length;
        count++;                                                        // INC R4
    }
    work_byte(pair.rpm_at) = count & 0xFF;
    work_byte(control.fire_l) = work_byte(control.fire_h) = 0xFF;
}

// A task, or a part of one, on a job's copy of the RAM (L1's work unless said).
static __force_inline void run_task(uint32_t task, uint32_t description, uint8_t *ram = work)
{
    unit_task(task, description, (const uint8_t *)pair.rom, ram);
}

static __force_inline bool run_part(uint32_t task, uint32_t part, uint32_t description, uint8_t *ram = work)
{
    return unit_task_part(task, part, description, (const uint8_t *)pair.rom, ram);
}

// An 8051 bit set or cleared in L1's copy of the RAM.
static __force_inline void work_set(uint32_t address, bool on)
{
    uint8_t &byte = work[0x20 + (address >> 3 & 0x0F)];
    uint32_t mask = 1u << (address & 7);
    byte = on ? byte | mask : byte & ~mask;
}

static void UNIT_CODE(make_pulse)(bool again)
{
    run_task(again ? TASK_AGAIN : TASK_PULSE, again ? control.again_task : control.pulse_task);
    if (work_byte(control.flags_at) & PULSE_LATE)
        return;                                                         // its reference came: none
    control.all_pulses++;
    if (!again)
    {
        control.turn_pulses++;
        control.pulse_made = true;
    }
    if (pair.whole)                                                     // format 11: InjHold as the 8051's take
        work_set(pair.hold_bit, work_byte(control.flags_at) & PULSE_HELD); //   applies it (the log's)
}

// The stages (Main_Stage's 2 to 5): the speed's count and the idle spark, the advance's sum and rest,
// the plans' advance (Main_Advance's sum), then the pulse if due.
static void UNIT_CODE(run_stages)(bool pulse)
{
    run_task(TASK_SPEED, control.speed_task);
    run_task(TASK_ADVANCE, control.advance_task);
    run_task(TASK_REST, control.rest_task);
    work_byte(pair.adv_at) = (work_byte(control.advance_at) + work_byte(control.dyn_at)) & 0xFF;
    if (pulse)
        make_pulse(false);
    control.runs++;
}

static void UNIT_CODE(run_firing)(const Job &job)
{
    take_inputs(work, before);
    bool starting = pair.whole ? job.starting != 0 : work_bit(pair.start_bit);   // format 11: as the
    bool stages = false, pulse = false, cut = false;                             //   branch took it
    if (job.length != 0)
    {
        firing_length(job.length);                                      // Main_Capture
        uint8_t &slew = work_byte(pair.slew_at);                        // Turn_Start: the slew's count
        if (slew != 0)
            slew--;
    }
    if (starting)
        count_injection();                                              // Turn_Start_Inject's
    if (job.length != 0 && job.first)                                   // Main_Turn: the turn's first firing
    {
        stages = true;
        pulse = starting || job.decision == TURN_PULSE;
        cut = !starting && job.decision == TURN_CUT;
        if (cut)
            count_injection();                                          // Main_Turn_Cut's
    }
    else if (job.length != 0)
        stages = work_byte(pair.rpm_at) < control.pipe_rpm;             // the rate: CJNE A,#PIPE_RPM; CPL C
    if (stages)
        run_stages(pulse);
    if (give_back(job.generation, work, before))
        give_take(job.generation, cut);
}

// The log (format 11): each frame's bytes into a ring, which the board's USB sends from while the PC
// has a log session open (comms.cpp), and DME-Sim reads in the car; written counts every byte put.
static constexpr uint32_t LOG_RING = 256;

struct Log
{
    volatile uint32_t written; // 0 (DME-Sim reads it)
    uint8_t ring[LOG_RING];    // 4
};

static Log pair_log UNIT_DATA = {};

static __force_inline void log_put(uint32_t b)
{
    pair_log.ring[pair_log.written & (LOG_RING - 1)] = b;
    pair_log.written = pair_log.written + 1;
}

uint32_t UNIT_CODE(pair_log_read)(uint32_t *from, uint8_t *bytes, uint32_t most)
{
    uint32_t written = pair_log.written, n = 0;
    if (written - *from > LOG_RING)
        *from = written - LOG_RING;                                     // the ring went round: those lost
    while (*from != written && n < most)
    {
        bytes[n++] = pair_log.ring[*from & (LOG_RING - 1)];
        *from = *from + 1;
    }
    return n;
}

// The log's frame, every Cal_StreamTicks ticks (none for 0), as rom_lean's stream had it: A5h 5Ah, its
// count (the tick counter / Cal_StreamTicks: DIV AB), the RAM bytes Cal_StreamAddresses lists (to FFh),
// and the sum of the count and those bytes. Made after the tick's commit and the pulse made again from
// the unit's copy as it is then (L2's copy again, the snoop's bytes over it), so the bus level's own
// bits (NotSynced, ToothFault, the phase, RefMissing) are as it has them.
static void UNIT_CODE(make_frame)(uint32_t tick)
{
    uint32_t every = at(control.stream_ticks);
    if (every == 0)
        return;
    uint32_t count = map_unit_divide(tick, every);
    if (tick != count * every)
        return;                                                         // not due
    take_inputs(tick_work, nullptr);
    log_put(0xA5);
    log_put(0x5A);
    log_put(count);
    uint32_t sum = count;
    for (uint32_t i = 0; i < 0x80; i++)
    {
        uint32_t address = at(control.stream_list + i);
        if (address == 0xFF)
            break;
        uint32_t b = copy_byte(tick_work, address);
        log_put(b);
        sum += b;
    }
    log_put(sum & 0xFF);
}

// Format 11 (DME-Sim's docs/Paired_Lambda_Idle.md, docs/Paired_Levels.md): L2's tick on its own copy,
// in rom_lean's order: the speed byte's bound by the ticks since slot 0's tooth and the throttle switches
// (Main_Tick's; task 15's first part), Tick_Stop's open loop (task 16), start mode and the guard's
// periods (task 12, its inputs the unit's own), the rest of the tick's work (task 15), the tick counter
// one on, lambda's timer and lambda (16), every second tick the idle control's state (task 17) and a duty
// if it or the stop asked for one. Then its commit with L1 held off: nothing if the 8051's start or a
// stop overtook it (false), all again from a new copy if an L1 job committed meanwhile; else its bits
// given back, start mode's end for the phase (task 12 leaves it to the commit), its counts, its take, the
// heartbeat (if L1 has taken every job queued before the tick began, so that a stuck L1 stops it too),
// and the pulse made again queued for L1 if it can be made (Main_Byte_Again's: a pulse waiting, not
// starting, under Cal_RemakeRpm), which runs it as it is let go. *tick: the tick counter given, for
// the frame.
static bool UNIT_CODE(tick_job)(const Ping &ping, uint32_t *counter)
{
    uint32_t queued = control.in;
    for (;;)
    {
        uint32_t jobs = levels.event_jobs;
        uint8_t *ram = tick_work;
        take_inputs(ram, tick_before);
        bool starting = copy_bit(ram, pair.start_bit), asked = control.duty_due, due = false, duty = false;
        if (ping.gap != 0)
        {
            uint32_t bound = map_unit_divide(pair.teeth, ping.gap);     // DIV AB
            if (bound < copy_byte(ram, pair.rpm_at))                    // CJNE A,Rpm; JNC
                copy_byte(ram, pair.rpm_at) = bound;
        }
        run_part(TASK_TICKWORK, TICK_SWITCHES, control.tick_task, ram); // Main_Tick's first: the switches
        run_part(TASK_LAMBDA, LAMBDA_OPEN, control.lambda_task, ram);
        run_task(TASK_START, control.start_task, ram);
        run_task(TASK_TICKWORK, control.tick_task, ram);
        uint8_t &tick = copy_byte(ram, control.tick_at);                // Main_Tick_Counter: 0 to 179
        tick = tick == 179 ? 0 : tick + 1;
        run_part(TASK_LAMBDA, LAMBDA_TIMER, control.lambda_task, ram);
        run_part(TASK_LAMBDA, LAMBDA_RUN, control.lambda_task, ram);
        if ((tick & 1) == 0)                                            // IdleDue: the counter even
            due = run_part(TASK_IDLE, IDLE_STATE, control.idle_task, ram);
        if (due || asked)                                               // IdleDuty
        {
            run_part(TASK_IDLE, IDLE_DUTY, control.idle_task, ram);
            duty = true;
        }
        bool over = starting && !copy_bit(ram, pair.start_bit);         // start mode over
        hold_events();
        if (ping.generation != control.generation)
        {
            let_events_go();
            levels.tick_dropped++;
            return false;
        }
        if (levels.event_jobs != jobs)
        {
            let_events_go();
            levels.again++;
            continue;
        }
        if (!give_back(ping.generation, ram, tick_before))
        {
            let_events_go();
            levels.tick_dropped++;
            return false;
        }
        if (over)
            pair_start_over();
        if (asked)
            control.duty_due = false;
        control.ticks++;
        if (duty)
            control.duties++;
        give_take(ping.generation, false);
        if ((int32_t)(control.out - queued) >= 0)
            write(pair.ping + 1, ++pair.beat & 0xFF);                   // the heartbeat
        bool waits = control.pulse_made && pair.references == pair.pulse_reference;  // PulseWaits
        if (waits && !copy_bit(ram, pair.start_bit) && copy_byte(ram, pair.rpm_at) < control.remake_rpm)
        {                                                               // the pulse made again (its job
            __asm volatile("cpsid i" ::: "memory");                     //   asks again: a reference may
            push(JOB_SENT, 0, 0, 0);                                    //   come first); the bus level
            __asm volatile("cpsie i" ::: "memory");                     //   pushes too
        }
        levels.ticks++;
        *counter = tick;
        let_events_go();
        return true;
    }
}

bool UNIT_CODE(pair_on_tick_copy)(const uint8_t *ram)
{
    return ram == tick_work;
}

// pair_test's: L2's tick at once, its job, then L1's jobs (as L1 breaks in when the commit lets it go:
// the pulse made again), then its frame.
static void UNIT_CODE(tick_at_once)(const Ping &ping)
{
    uint32_t tick;
    if (!tick_job(ping, &tick))
        return;
    while (control.stop_due || control.out != control.in)
        pair_control();
    make_frame(tick);
}

// L2's interrupt: the next ping's tick, then its frame; the next one pended again.
void UNIT_CODE(pair_tick)()
{
    if (pings.out == pings.in)
        return;
    const Ping &ping = pings.queue[pings.out & (PINGS - 1)];
    uint32_t tick;
    if (ping.generation != control.generation)
        levels.tick_dropped++;
    else if (tick_job(ping, &tick))
    {
        make_frame(tick);
        uint32_t span = now_us() - ping.at;
        if (span > levels.tick_us)
            levels.tick_us = span;
        if (span > TICK_BOUND)
            levels.tick_over++;
    }
    pings.out = pings.out + 1;
    if (pings.out != pings.in)
        pend_tick();
}

// Format 10's tick (L1's): the beat, and the speed byte's bound.
static void UNIT_CODE(run_tick)(const Job &job)
{
    write(pair.ping + 1, ++pair.beat & 0xFF);                           // the heartbeat, from this level
    uint32_t gap = job.length;
    if (gap == 0 || job.generation != control.generation)
        return;
    uint32_t bound = map_unit_divide(pair.teeth, gap);                  // DIV AB
    if (bound >= map_unit_ram(pair.rpm_at))                             // CJNE A,Rpm; JNC
        return;
    map_unit_ram_write(pair.rpm_at, bound);                             // (this level's byte alone)
    give_take(job.generation, false);
}

static void UNIT_CODE(run_sent)(const Job &job)
{
    take_inputs(work, before);
    bool waits = control.pulse_made && pair.references == pair.pulse_reference;  // PulseWaits
    if (work_bit(pair.start_bit) || work_byte(pair.rpm_at) >= control.remake_rpm || !waits)
        return;                                                         // Main_Byte_Again
    make_pulse(true);
    if (give_back(job.generation, work, before))
        give_take(job.generation, false);
}

static void UNIT_CODE(run_guard)(const Job &job)
{
    uint32_t count = map_unit_ram(control.boost_at);
    if (count != 0 && job.generation == control.generation)             // MOV A,BoostCount; JZ; DEC
        map_unit_ram_write(control.boost_at, count - 1);
}

static void UNIT_CODE(run_stop)(const Job &job)
{
    take_inputs(work, before);
    bool fresh = control.fresh, duty = false;
    if (control.fresh && control.afm_filt_at != 0)                      // format 10, after the 8051's start:
        work_byte(control.afm_filt_at) = work_byte(control.afm_at);     //   Power_Up_Inputs' MOV AfmFilt,A
    control.fresh = false;
    work_byte(pair.inj_at) = 0;                                         // MOV InjCount,#00h
    work[0x20 + (pair.decay_bit >> 3 & 0x0F)] &= ~(1u << (pair.decay_bit & 7)); // CLR CrankDecay
    work_byte(control.fire_l) = work_byte(control.fire_h) = 0xFF;       // no firing: the count skipped
    run_stages(true);
    if (pair.whole)                                                     // format 11: InjHold cleared (the
    {                                                                   //   8051's stop); the valve's duty
        work_set(pair.hold_bit, false);                                 //   (IdleDuty) at once after its start
        if (fresh)                                                      //   (Power_Up_Inputs'), else in the
        {                                                               //   tick's job that follows (Tick_Stop's)
            run_part(TASK_IDLE, IDLE_DUTY, control.idle_task);
            duty = true;
        }
        else
            control.duty_due = true;
    }
    if (give_back(job.generation, work, before))
    {
        if (duty)
            control.duties++;
        give_take(job.generation, false);
    }
}

// The tick's work (format 10: task 15), its results counted for the 8051's lambda.
static void UNIT_CODE(run_tickwork)(const Job &job)
{
    take_inputs(work, before);
    if (pair.whole)                                                     // (format 11, pair_test's alone:
        run_part(TASK_TICKWORK, TICK_SWITCHES, control.tick_task);      //   its switches first, as the tick)
    run_task(TASK_TICKWORK, control.tick_task);
    if (give_back(job.generation, work, before))
    {
        control.ticks++;
        give_take(job.generation, false);
    }
}

// A job's time from its push (or the stop's from its asking) to its end, counted (us).
static __force_inline void event_done(uint32_t at)
{
    uint32_t span = now_us() - at;
    if (span > levels.event_us)
        levels.event_us = span;
    if (span > EVENT_BOUND)
        levels.event_over++;
    levels.event_jobs = levels.event_jobs + 1;
}

// L1's interrupt: one job a run, the stop's first (the jobs queued before it are stale); the next one
// pended again (a tail-chain), so that a job's run is the level's.
void UNIT_CODE(pair_control)()
{
    if (control.stop_due)
    {
        control.stop_due = false;
        Job job;
        job.kind = JOB_STOP;
        job.generation = control.generation;
        job.length = job.first = job.decision = job.starting = 0;
        job.at = control.stop_at;
        run_stop(job);
        event_done(job.at);
    }
    else if (control.out != control.in)
    {
        const Job &job = control.queue[control.out & (JOBS - 1)];
        if (job.generation != control.generation)
            control.dropped++;
        else if (job.kind == JOB_FIRING)
            run_firing(job);
        else if (job.kind == JOB_TICK)
            run_tick(job);
        else if (job.kind == JOB_SENT)
            run_sent(job);
        else if (job.kind == JOB_GUARD)
            run_guard(job);
        else if (job.kind == JOB_TICKWORK)
            run_tickwork(job);
        uint32_t at = job.at;
        control.out = control.out + 1;
        event_done(at);
    }
    else
        return;
    if (control.stop_due || control.out != control.in)
        pend();
}

// NotSynced (its bit address the operand after the opcode at Pair_Synced) as the unit has the count, in
// its copy of the 8051's RAM (the bus level's write: the pipeline's level never changes that bit).
static void UNIT_CODE(own_sync)()
{
    if (!control.pipelined)
        return;
    own_bit(pair.whole ? pair.synced_bit : at(pair.synced_op + 1), !pair.synced);   // (format 11: the table's)
}

void UNIT_CODE(pair_guard_period)()
{
    if (control.pipelined)
        push(JOB_GUARD, 0, 0, 0);
}

void UNIT_CODE(pair_tick_sent)()
{
    if (control.pipelined)
        push(JOB_SENT, 0, 0, 0);
}

// Task 12's end: the tick's work, as rom_lean's Main_Tick went on after start mode; none for its call
// at power-up (rom_lean's first tick's work comes at its first tick: the first ping).
void UNIT_CODE(pair_tick_work)()
{
    if (control.pipelined && control.pinged)
        push(JOB_TICKWORK, 0, 0, 0);
}

// The 8051's start: its RAM cleared, so the take's count from 0 and the switch to Take_A as built, the
// pulses' and the tick's work's counts from 0; the jobs queued or under way give nothing.
static void UNIT_CODE(control_as_built)()
{
    control.generation = control.generation + 1;
    control.stop_due = false;                                   // (Power_Up_Inputs' stop comes next)
    control.gap = 0;
    control.side = control.count = 0;
    control.turn_pulses = control.all_pulses = 0;
    control.ticks = 0;
    control.duties = 0;
    control.duty_due = false;
    control.pinged = false;
    control.fresh = true;
    control.pulse_made = false;
    write(control.switch_at + 1, control.copy[0] & 0xFF);
    write(control.count_at + 1, 0);
    if (pair.whole)                                             // format 11: the starting injection's branch
    {                                                           //   as built (past it), the pins as built
        write(pair.inject_at + 1, (pair.skip_to - pair.inject_at - 2) & 0xFF);  // (all high), Power_Up's
        map_unit_ram_write(pair.pins_at, 0x07);                 //   MOV LamInt,#80h, the reference's check
        map_unit_ram_write(control.lam_at, 0x80);               //   from here
        pair.ref_mark = pair.references;
    }
    own_sync();                                                 // (the RAM's copy cleared before: not in step)
}

// ---------------------------------------------------------------------------------------------------
// The crank.

// The latest tooth's place in its turn (0: fall 1), and so its tooth in its firing and its firing.
static void UNIT_CODE(place_at)(uint32_t pos)
{
    pair.pos = pos;
    pair.firing = 0;
    while (pos >= pair.teeth)
    {
        pos -= pair.teeth;
        pair.firing++;
    }
    pair.tif = pos + 1;
}

// No read moved off a spark: the coil starts again.
static void UNIT_CODE(clear_moves)()
{
    for (uint32_t i = 0; i < MOVES; i++)                        // (field by field: no initializer in flash)
    {
        pair.moves[i].from = pair.moves[i].to = NONE;
        pair.moves[i].event = NOTHING;
    }
}

// Nothing planned, the coil's events off.
static void UNIT_CODE(drop_coil)()
{
    pair.coil_on = false;
    pair.spark_due = false;
    pair.arm_at = pair.charge_at = pair.spark_at = NONE;
    clear_moves();
    pair.missed_count = 0;
}

// No count: no firing timed, nothing planned, a RETI in the vector.
static void UNIT_CODE(unsync)()
{
    pair.synced = 0;
    pair.stamped = false;
    pair.fire_valid = false;
    pair.p_old = 0xFFFF;
    drop_coil();
    steer(NOTHING);
    write_flags();
}

// The firing's length at slot 0's tooth (the table's first event), and the tooth period from it; the
// length returned (cycles; 0: none, a stamp alone).
static uint32_t UNIT_CODE(slot_zero)(uint32_t now)
{
    uint32_t length = 0;
    if (pair.stamped && pair.count - pair.stamp_count == pair.teeth)
    {
        uint32_t cycles = (now - pair.stamp_time + 1) >> 1;
        length = cycles;
        pair.p = cycles > 0xFFFF ? 0xFFFF : cycles;
        pair.t64 = capture_t64(pair.p, &pair.p_old, pair.fire_valid, pair.recip32);
        pair.fire_valid = true;
    }
    pair.stamp_count = pair.count;
    pair.stamp_time = now;
    pair.stamped = true;
    return length;
}

static void UNIT_CODE(tooth)(uint32_t now)
{
    pair.teeth_seen++;
    pair.count++;
    pair.t_prev = pair.t_last;
    pair.t_last = now;
    if (pair.timed < 2)
        pair.timed++;
    if (!pair.synced)
        return;
    if (++pair.pos == pair.turn)
        pair.pos = 0;
    if (++pair.tif > pair.teeth)
    {
        pair.tif = 1;
        if (++pair.firing == pair.firings)
            pair.firing = 0;
    }
    if (pair.tif == pair.event_tooth[0])
    {
        uint32_t length = slot_zero(now), decision = TURN_PULSE;
        if (pair.firing == 0 && pair.phased)
            decision = turn();                                  // the turn's first firing
        bool starting = ram_bit(pair.start_bit);
        if (pair.whole)                                         // format 11: the starting injection's branch,
        {                                                       //   before the 8051's main level reaches it;
            uint32_t to = starting ? pair.inject_to : pair.skip_to;   //   at the turn's first firing the
            write(pair.inject_at + 1, (to - pair.inject_at - 2) & 0xFF); // reference's check (rom_lean's
            if (pair.firing == 0)                               //   Main_Firing: RefTurn, else RefMissing,
            {                                                   //   kept till the 8051's start)
                if (pair.references == pair.ref_mark)
                    own_bit(pair.missing_bit, true);
                pair.ref_mark = pair.references;
            }
        }
        if (control.pipelined)                                  // format 9: the pipeline, at once
        {
            control.gap = 0;
            push(JOB_FIRING, length, pair.firing == 0, decision, starting);
        }
    }
}

// The reference: the latch's tooth by the times (as rom_lean by the speed input), so fall 1's count;
// in step if that is the count's fall 1 or a tooth off it, else the count from there (a fault).
static void UNIT_CODE(reference)(uint32_t now)
{
    pair.references++;
    if (pair.timed < 2)
        return;                                                 // no tooth period yet
    uint32_t period = pair.t_last - pair.t_prev, since = now - pair.t_last, k = 0;
    while (k < 4 && since * 256 >= (256 - pair.teeth_high + 256 * k) * period)
        k++;
    uint32_t ahead = k + pair.fall1_ahead;                      // fall 1, this many teeth on
    if (pair.synced)
    {
        uint32_t place = pair.pos + ahead;                      // its place in the count's turn
        while (place >= pair.turn)
            place -= pair.turn;
        if (place == 0 || place == 1 || place == pair.turn - 1)
            return;                                             // in step, or the tie
        pair.faults++;
        pair.fault = true;
        pair.stamped = false;
        pair.fire_valid = false;
        drop_coil();
        steer(NOTHING);
    }
    place_at(ahead == 0 ? 0 : pair.turn - ahead);
    pair.synced = 1;
    pair.syncs++;
    pair.steered = pair.count;
    write_flags();
}

// ---------------------------------------------------------------------------------------------------
// The coil.

// The spark in the firing whose fall 1 has count base planned: its arm, the read it moves late, and
// its charge (NONE if its tooth has passed: the 8051's arm then fires no spark, as it is not
// charging).
static void UNIT_CODE(plan)(int32_t base)
{
    int32_t arm, late_tooth;
    uint32_t late;
    pair.plan_adv = map_unit_ram(pair.adv_at) & 0xFF;
    Load load = arm_plan(pair.plan_adv, &arm, &late_tooth, &late);
    pair.sload = load.start;
    pair.shi = load.overflows;
    pair.sarm = arm;
    pair.arm_at = base + arm - 1;
    pair.spark_at = base + spark_place(pair.plan_adv, pair.spark_b, pair.t64).tooth - 1;
    if (late_tooth != NONE)                                     // in a slot whose read has run
    {
        Move *move = &pair.moves[0];
        for (uint32_t i = 0; i < MOVES; i++)
            if (pair.moves[i].to <= (int32_t)pair.count)
                move = &pair.moves[i];
        move->from = base + late_tooth - 1;
        move->to = move->from + (int32_t)late;
        move->event = event_on(in_firing(late_tooth));
    }
    uint32_t charge = map_unit_ram(pair.charge_h) << 8 | map_unit_ram(pair.charge_l);
    pair.charge_at = base + charge_plan(pair.plan_adv, arm, charge) - 1;
    if (pair.charge_at <= (int32_t)pair.count)
        pair.charge_at = NONE;                                  // passed
    pair.coil_on = true;
    pair.plans++;
}

// The coil's events from now: the spark of this firing if its charge is still to come, else the
// next firing's (rom_lean's Coil_Start charged the first at its arm, short). A read the first plan
// moved stays where it was if that spark is not the one.
static void UNIT_CODE(start)()
{
    int32_t base = (int32_t)pair.count - (int32_t)pair.tif + 1;
    clear_moves();
    plan(base);
    if (pair.charge_at == NONE || pair.arm_at <= (int32_t)pair.count)
    {
        clear_moves();
        plan(base + (int32_t)pair.teeth);
    }
}

// A moved read's event on the tooth whose count is c (NOTHING: none), and whether a read moved from
// there.
static uint32_t UNIT_CODE(moved_to)(int32_t c)
{
    for (uint32_t i = 0; i < MOVES; i++)
        if (pair.moves[i].to == c)
            return pair.moves[i].event;
    return NOTHING;
}

static bool UNIT_CODE(moved_from)(int32_t c)
{
    for (uint32_t i = 0; i < MOVES; i++)
        if (pair.moves[i].from == c)
            return true;
    return false;
}

// The crank event of the tooth count back from the latest (a late run's teeth).
static uint32_t UNIT_CODE(event_back)(uint32_t back)
{
    return event_on(in_firing((int32_t)pair.tif - (int32_t)back));
}

// A late run: the teeth after the run's first new one ran what the vector held. An event due there
// goes on the next tooth: the arm with its load from there (none if too near its spark), the charge
// if before the arm, a crank event in turn.
static void UNIT_CODE(late_run)(uint32_t fresh)
{
    pair.late_runs++;
    if (pair.vector != NOTHING)
        pair.repeats += fresh - 1;
    int32_t next = (int32_t)pair.count + 1;
    for (uint32_t back = fresh - 1; back-- > 0;)                // counts steered + 2 to the latest
    {
        int32_t c = (int32_t)pair.count - (int32_t)back;
        if (c == pair.arm_at)
        {
            pair.missed++;
            int32_t arm = pair.sarm + (next - pair.arm_at);
            pair.arm_at = arm_from(arm) ? next : NONE;
        }
        else if (c == pair.charge_at)
        {
            pair.missed++;
            pair.charge_at = pair.arm_at != NONE && next < pair.arm_at ? next : NONE;
        }
        uint32_t event = moved_to(c);
        if (event == NOTHING && !moved_from(c))
            event = event_back(back);
        if (event != NOTHING && pair.missed_count < MOST_MISSED)
        {
            pair.missed++;
            pair.missed_events[pair.missed_count++] = event;
        }
    }
}

// The event of the next tooth (count + 1).
static uint32_t UNIT_CODE(next_event)()
{
    int32_t next = (int32_t)pair.count + 1;
    if (next == pair.arm_at)
        return ARM;
    if (next == pair.charge_at)
        return CHARGE;
    uint32_t event = moved_to(next);
    if (event != NOTHING)
        return event;
    event = moved_from(next) ? NOTHING : event_on(pair.tif == pair.teeth ? 1 : pair.tif + 1);
    if (event == NOTHING && pair.missed_count != 0)             // a passed crank event, on a free tooth
    {
        event = pair.missed_events[0];
        for (uint32_t i = 1; i < pair.missed_count; i++)
            pair.missed_events[i - 1] = pair.missed_events[i];
        pair.missed_count--;
    }
    return event;
}

// ---------------------------------------------------------------------------------------------------
// The interface.

void UNIT_CODE(pair_page_read)(uint32_t address, uint32_t now)
{
    uint32_t previous = pair.previous;
    pair.previous = address;
    if (!pair.on)
        return;
    if (address == 0)                                           // the 8051's start
    {
        pair.fault = false;
        unsync();
        if (pair.inject)
            loads_as_built();
        if (pair.phased)
        {
            map_unit_ram_clear(0x00, pair.whole ? 0x7D : 0x7F); // the counts from 0, as Power_Up's RAM
            phase_as_built();                                   //   (format 11: the idle integrator kept)
        }
        if (control.pipelined)
            control_as_built();
        return;
    }
    if (address != previous + 1)
        return;
    if (previous == VECTOR)
        tooth(now);
    else if (previous == REFERENCE)
    {
        if (pair.phased)
            phase_at_reference();
        reference(now);
    }
    else if (previous == pair.spark)
        pair.spark_due = pair.synced != 0;
    else if (previous == pair.ping)
    {
        if (control.pipelined)                                  // format 9: the tick's job, the beat with it
        {
            uint32_t gap = control.gap;
            if (control.gap < 0xFF)
                control.gap++;
            control.pinged = true;                              // (format 10: the tick's work from here)
            if (pair.whole)
                push_ping(gap);                                 // format 11: L2's, the beat at its commit
            else
                push(JOB_TICK, gap, 0, 0);
        }
        else
            write(pair.ping + 1, ++pair.beat & 0xFF);
    }
    else if (previous == pair.lost)
        unsync();
    else if (pair.whole && previous - pair.bits_mark < 16 && ((previous - pair.bits_mark) & 1) == 0)
        map_unit_ram_write(pair.pins_at, (previous - pair.bits_mark) >> 1);  // format 11: the 8051's pins
    else if (!pair.phased)
        return;
    else if (pair.fire_mark != 0 && previous == pair.fire_mark) // (format 9: no such marks)
        at_firing();
    else if (pair.count_mark != 0 && previous == pair.count_mark)
        at_counted();
    else if (previous == pair.stop_mark)
    {
        unsync();
        if (control.pipelined)
            stop_pipeline();                                    // the counts and the stages: the pipeline's
        else
            at_stopped();
    }
}

void UNIT_CODE(pair_after)()
{
    if (!pair.on || !pair.synced)
        return;
    if (pair.spark_due)                                         // the next spark's charge and arm
    {
        pair.spark_due = false;
        pair.sparks++;
        if (pair.fire_valid)
            plan((int32_t)pair.count - (int32_t)pair.tif + 1 + (int32_t)pair.teeth);
    }
    if (pair.count == pair.steered)
        return;                                                 // no new tooth
    uint32_t fresh = pair.count - pair.steered;
    if (fresh > 1)
        late_run(fresh);
    pair.steered = pair.count;
    if (pair.fire_valid && !pair.coil_on)
        start();
    else if (pair.coil_on && pair.spark_at != NONE && (int32_t)pair.count > pair.spark_at + 2)
    {
        pair.lost_sparks++;                                     // its spark did not come
        start();
    }
    uint32_t event = next_event();
    if (event == ARM)
    {
        refresh();
        write(pair.arm_start, pair.sload);
        write(pair.arm_overflows, pair.shi);
    }
    steer(event);
}

// As the board starts (the counts from 0, as built): no count, nothing planned.
void UNIT_CODE(pair_start)(uint8_t *rom)
{
    pair.on = false;
    pair.inject = false;
    pair.synced = 0;
    if (rom == nullptr)
        return;                                                 // no unit
    pair.rom = rom;
    pair.vector = NOTHING;
    pair.dj = 1;
    pair.p_old = 0xFFFF;
    drop_coil();
    bool marked = at(MARK) == 'M' && at(MARK + 1) == 'A' && at(MARK + 2) == 'P' && at(MARK + 3) == 'U';
    if (!marked || at(MARK + 4) < FORMAT)
        return;
    uint32_t t = word(MARK + 13);
    pair.teeth = at(t);
    pair.firings = at(t + 1);
    pair.turn = pair.teeth * pair.firings;
    pair.events = at(t + 2);
    if (pair.events == 0 || pair.events > MOST_EVENTS || pair.teeth == 0 || pair.firings == 0)
        return;
    for (uint32_t i = 0; i < pair.events; i++)
    {
        pair.event_tooth[i] = at(t + 3 + 3 * i);
        pair.event_handler[i] = word(t + 4 + 3 * i);
    }
    uint32_t u = t + 3 + 3 * pair.events;
    pair.charge_handler = word(u);
    pair.arm_handler = word(u + 2);
    pair.arm_start = word(u + 4);
    pair.arm_overflows = word(u + 6);
    pair.synced_op = word(u + 8);
    pair.fault_op = word(u + 10);
    pair.ping = at(u + 12);
    pair.lost = at(u + 13);
    pair.spark = at(u + 14);
    pair.spark_b = word(u + 15);
    pair.arm_ofs = at(u + 17);
    pair.arm_min = at(u + 18);
    pair.margin = word(u + 19);
    pair.floor_teeth = at(u + 21);
    pair.recip32 = word(u + 22);
    pair.teeth_high = at(u + 24);
    pair.fall1_ahead = at(u + 25);
    pair.charge_l = at(u + 26);
    pair.charge_h = at(u + 27);
    pair.adv_at = at(u + 28);
    bool reach = pair.charge_handler < 0x800 && pair.arm_handler < 0x800; // within the vector's AJMP
    for (uint32_t i = 0; i < pair.events; i++)
        reach = reach && pair.event_handler[i] < 0x800;
    pair.inject = false;
    if (at(MARK + 4) >= INJECTION)                              // TH0's loads: each switch and its copies
    {
        pair.next_switch = word(u + 29);
        pair.next_copy[0] = word(u + 31);
        pair.next_copy[1] = word(u + 33);
        pair.apply_switch = word(u + 35);
        pair.apply_copy[0] = word(u + 37);
        pair.apply_copy[1] = word(u + 39);
        pair.built = at(pair.next_copy[0] + 2) | at(pair.next_copy[0] + 5) << 8;
        pair.side = 0;
        pair.load = pair.built;
        pair.pulse_reference = 0;
        // Each switch's AJMP reaches its copies, both in one page (the operand alone tells them).
        bool pages = pair.next_copy[0] >> 8 == pair.next_copy[1] >> 8 && pair.apply_copy[0] >> 8 == pair.apply_copy[1] >> 8;
        bool near = ((pair.next_switch + 2) ^ pair.next_copy[0]) < 0x800 && ((pair.apply_switch + 2) ^ pair.apply_copy[0]) < 0x800;
        pair.inject = pages && near;
        reach = reach && pair.inject;
    }
    pair.phased = false;
    if (at(MARK + 4) >= PHASE)                                  // the injection's phase
    {
        pair.ref_inject = word(u + 41);
        pair.ref_skip = word(u + 43);
        pair.turn_at = word(u + 45);
        pair.turn_due = word(u + 47);
        pair.turn_speed = word(u + 49);
        pair.turn_cut = word(u + 51);
        pair.phase_op = word(u + 53);
        pair.rev_limit = word(u + 55);
        pair.rpm_at = at(u + 57);
        pair.start_bit = at(u + 58);
        pair.fuel_cut_bit = at(u + 59);
        pair.boost_cut_bit = at(u + 60);
        pair.stop_mark = at(u + 61);                            // the counts' marks and places
        pair.fire_mark = at(u + 62);
        pair.count_mark = at(u + 63);
        pair.slew_at = at(u + 64);
        pair.inj_at = at(u + 65);
        pair.decay_bit = at(u + 66);
        pair.next_skips = 1;
        pair.turn_cuts = pair.turns = 0;
        // The reference's handlers in one page within 0003h's AJMP; the SJMP's targets within its reach
        // (format 9: none, 0).
        bool page = pair.ref_inject >> 8 == pair.ref_skip >> 8 && pair.ref_inject < 0x800;
        uint32_t from = pair.turn_at + 2 - 0x80;                // an SJMP reaches -128 to 127 from its end
        bool sjmp = pair.turn_at == 0
                    || (pair.turn_due - from < 0x100 && pair.turn_speed - from < 0x100 && pair.turn_cut - from < 0x100);
        pair.phased = page && sjmp;
        reach = reach && pair.phased;
    }
    pair.whole = false;
    if (at(MARK + 4) >= WHOLE && pair.phased)                   // format 11: the marks of the 8051's pins and
    {                                                           //   their byte, the bits the unit keeps for the
        pair.bits_mark = at(u + 93);                            //   log, the starting injection's SJMP and its
        pair.pins_at = at(u + 94) & 0x7F;                       //   two targets (the injection, past it)
        pair.synced_bit = at(u + 95);
        pair.fault_bit = at(u + 96);
        pair.phase_bit = at(u + 97);
        pair.missing_bit = at(u + 98);
        pair.hold_bit = at(u + 99);
        pair.inject_at = word(u + 100);
        pair.inject_to = word(u + 102);
        pair.skip_to = word(u + 104);
        uint32_t from = pair.inject_at + 2;                     // an SJMP reaches -128 to 127 from its end
        pair.whole = pair.inject_at != 0 && pair.inject_to - from + 0x80 < 0x100 && pair.skip_to - from + 0x80 < 0x100;
        reach = reach && pair.whole;
    }
    control.pipelined = false;
    control.in = control.out = control.generation = 0;
    control.gap = control.side = control.count = control.turn_pulses = control.all_pulses = 0;
    control.ticks = 0;
    control.pulse_made = control.pinged = false;
    control.fresh = true;
    // The pipeline: the take's places and numbers (format 10's take: format 9's has gone).
    if (at(MARK + 4) >= TICKS && pair.phased)
    {
        control.count_at = word(u + 67);
        control.switch_at = word(u + 69);
        control.copy[0] = word(u + 71);
        control.copy[1] = word(u + 73);
        control.pipe_rpm = at(u + 75);
        control.remake_rpm = at(u + 76);
        control.fire_l = at(u + 77) & 0x7F;
        control.fire_h = at(u + 78) & 0x7F;
        control.advance_at = at(u + 79) & 0x7F;
        control.dyn_at = at(u + 80) & 0x7F;
        control.inj_l = at(u + 81) & 0x7F;
        control.inj_h = at(u + 82) & 0x7F;
        control.dead_at = at(u + 83) & 0x7F;
        control.flags_at = at(u + 84) & 0x7F;
        control.boost_at = at(u + 85) & 0x7F;
        control.load_at = at(u + 86) & 0x7F;                    // format 10: the tick's work's in the take,
        control.accel_at = at(u + 87) & 0x7F;                   //   the switches' bits, the AFM's filter
        control.after_at = at(u + 88) & 0x7F;                   //   and the AFM (Power_Up_Inputs')
        control.idle_bit = at(u + 89);
        control.full_bit = at(u + 90);
        control.afm_filt_at = at(u + 91) & 0x7F;
        control.afm_at = at(u + 92) & 0x7F;
        control.tick_task = map_unit_task_at(TASK_TICKWORK);
        if (pair.whole)                                         // format 11: the tick's places, its tasks
        {
            control.tick_at = at(u + 106) & 0x7F;
            control.lam_at = at(u + 107) & 0x7F;
            control.stream_ticks = word(u + 108);
            control.stream_list = word(u + 110);
            control.high_l = at(u + 112) & 0x7F;
            control.high_h = at(u + 113) & 0x7F;
            control.start_task = map_unit_task_at(TASK_START);
            control.lambda_task = map_unit_task_at(TASK_LAMBDA);
            control.idle_task = map_unit_task_at(TASK_IDLE);
        }
        control.speed_task = map_unit_task_at(TASK_SPEED);
        control.advance_task = map_unit_task_at(TASK_ADVANCE);
        control.rest_task = map_unit_task_at(TASK_REST);
        control.pulse_task = map_unit_task_at(TASK_PULSE);
        control.again_task = map_unit_task_at(TASK_AGAIN);
        uint32_t data = control.speed_task + 1 + at(control.speed_task); // past its inputs, then its outputs
        data += 1 + at(data);
        control.speed_k = at(data) | at(data + 1) << 8 | at(data + 2) << 16;   // its data: SPEED_K, low first
        // Both copies in one page, which the switch's AJMP reaches; every task the pipeline runs found.
        bool copies = control.copy[0] >> 8 == control.copy[1] >> 8 && ((control.switch_at + 2) ^ control.copy[0]) < 0x800;
        bool tasks = control.speed_task != 0 && control.advance_task != 0 && control.rest_task != 0
                     && control.pulse_task != 0 && control.again_task != 0 && control.tick_task != 0
                     && (!pair.whole || (control.start_task != 0 && control.lambda_task != 0 && control.idle_task != 0));
        control.pipelined = copies && tasks;
        reach = reach && control.pipelined;
    }
    pair.on = reach;
}

// The levels' counts for the PC (from the main loop: in flash).
bool pair_get_parameter(const char *name, char *value, size_t value_size)
{
    if (streq(name, "pair_events"))
    {
        snprintf(value, value_size, "%u %u %u %u %u", (unsigned)levels.event_jobs, (unsigned)levels.event_lost,
                 (unsigned)levels.event_most, (unsigned)levels.event_us, (unsigned)levels.event_over);
        return true;
    }
    if (streq(name, "pair_tick"))
    {
        snprintf(value, value_size, "%u %u %u %u %u %u", (unsigned)levels.ticks, (unsigned)levels.again,
                 (unsigned)levels.tick_late, (unsigned)levels.tick_lost, (unsigned)levels.tick_us,
                 (unsigned)levels.tick_over);
        return true;
    }
    return false;
}

uint32_t UNIT_CODE(pair_test)(uint32_t routine)
{
    const uint32_t *in = pair_case.in;
    uint32_t *out = pair_case.out;
    if (routine == 0)                                           // Period_T64: P, RECIP32
        out[0] = period_t64(in[0], in[1]);
    else if (routine == 1)                                      // Spark_Place: advance, SPARK_B, T64
    {
        Place place = spark_place(in[0], in[1], in[2]);
        out[0] = (uint32_t)place.tooth & 0xFF;
        out[1] = place.time;
    }
    else if (routine == 2)                                      // Spark_Load: the time, ARM_OFS_S
    {
        Load load = spark_load(in[0], in[1]);
        out[0] = load.start;
        out[1] = load.overflows;
    }
    else if (routine == 3)                                      // Dwell_Plan's DJ: advance, SPARK_B, T64,
    {                                                           //   DJ, P, MARGIN, the charge's time
        Place place = spark_place(in[0], in[1], in[2]);
        out[0] = dwell_teeth(place, in[3], in[2], in[4], in[5], in[6]);
    }
    else if (routine == 4)                                      // Main_Capture's T64: P, the last P,
    {                                                           //   FireValid, RECIP32
        uint32_t old = in[1];
        out[0] = capture_t64(in[0], &old, in[2] != 0, in[3]);
        out[1] = old;
    }
    else if (routine == 5)                                      // the arm's plan with the image's table:
    {                                                           //   advance, T64
        pair.t64 = in[1];
        int32_t arm, late_tooth;
        uint32_t late;
        Load load = arm_plan(in[0], &arm, &late_tooth, &late);
        out[0] = (uint32_t)arm & 0xFF;
        out[1] = load.start;
        out[2] = load.overflows;
        out[3] = late_tooth == NONE ? 0xFF : (uint32_t)late_tooth & 0xFF;
        out[4] = late;
    }
    else if (routine == 6)                                      // the charge's plan with the image's table:
    {                                                           //   advance, T64, DJ, P, the charge's
        pair.t64 = in[1];                                       //   time, the arm's tooth
        pair.dj = in[2];
        pair.p = in[3];
        out[0] = (uint32_t)charge_plan(in[0], (int32_t)(int8_t)in[5], in[4]) & 0xFF;
        out[1] = pair.dj;
    }
    else if (routine >= 7 && routine <= 18)                     // format 9: a pipeline's job at once on the
    {                                                           //   unit's RAM: 7 a firing (its length, the
        Job job;                                                //   turn's first, the phase before it), 8 the
        job.generation = control.generation;                    //   stop, 9 a tick (its gap; format 10), 10 the
        job.length = in[0];                                     //   pulse made again; format 10, 11 the tick's
        job.first = in[1];                                      //   work; format 11, 12 the whole tick (its
        job.decision = TURN_PULSE;                              //   gap, the pins, a duty the stop asked for,
        job.starting = ram_bit(pair.start_bit);                 //   the turn's pulse waiting): 13 its setting
        if (routine == 7 || routine == 16)                      //   alone, 14 its ping queued for L2 and pended,
        {                                                       //   15 L2's tick of the ping queued, at once;
            job.kind = JOB_FIRING;                              //   16 a firing queued for L1 and pended; 17
            if (in[1] != 0)                                     //   the stop asked for and pended; 18 L1's jobs
            {                                                   //   at once (no table of the kinds: the
                pair.next_skips = in[2];                        //   compiler's would be in flash)
                job.decision = turn();
            }
            if (routine == 7)
                run_firing(job);
            else
                push(JOB_FIRING, job.length, job.first, job.decision, job.starting);
        }
        else if (routine >= 12 && routine <= 15)
        {
            if (routine == 12 || routine == 13)
            {
                map_unit_ram_write(pair.pins_at, in[2]);
                control.duty_due = in[3] != 0;
                control.pulse_made = true;
                pair.pulse_reference = pair.references - (in[4] != 0 ? 0 : 1);
            }
            if (routine == 12)
            {
                Ping ping;
                ping.gap = in[0];
                ping.generation = control.generation;
                ping.at = now_us();
                tick_at_once(ping);
            }
            else if (routine == 14)
                push_ping(in[0]);
            else if (routine == 15 && pings.out != pings.in)
            {
                tick_at_once(pings.queue[pings.out & (PINGS - 1)]);
                pings.out = pings.out + 1;
            }
        }
        else if (routine == 8 || routine == 17)
        {
            job.kind = JOB_STOP;
            unsync();                                           // (the stop's mark: the count lost first;
            control.fresh = false;                              //   an engine's stop, not the 8051's start)
            if (routine == 8)
                run_stop(job);
            else
                stop_pipeline();                                // 17: asked for as the bus level does, pended
        }
        else if (routine == 18)
        {
            while (control.stop_due || control.out != control.in)  // 18: L1's jobs at once
                pair_control();
        }
        else if (routine == 9)
        {
            job.kind = JOB_TICK;
            run_tick(job);
        }
        else if (routine == 10)
        {
            job.kind = JOB_SENT;                                // in[0]: the turn's pulse waits (its
            control.pulse_made = true;                          //   reference to come), else it came
            pair.pulse_reference = pair.references - (in[0] != 0 ? 0 : 1);
            run_sent(job);
        }
        else
        {
            job.kind = JOB_TICKWORK;
            run_tickwork(job);
        }
        out[0] = job.decision;
        out[1] = control.runs;
        out[2] = control.takes;
        out[3] = control.ticks;
        out[4] = control.duties;
        out[5] = pair_log.written;
    }
    else
        return 0;
    return 1;
}

// pair_test is called by DME-Sim alone: this keeps it in the bank (the linker drops what nothing
// refers to).
static uint32_t (*volatile pair_entry)(uint32_t) UNIT_DATA = pair_test;

#endif // FEATURE_MAP_UNIT
