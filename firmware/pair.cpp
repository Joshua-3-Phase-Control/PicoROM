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
//   turn's first tooth, is that tooth, or the one after it where MARK_G is over TEETH_HIGH (the
//   944's). Each reference checks the count: a tooth off either way is the tie (a fall and the latch
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
//
// The live bytes, all it writes of the 8 KiB (DME-Sim checks that): 0013h and 0014h, the arm's two
// operands, 0017h, and the opcodes at Pair_Synced (SETB NotSynced till in step, then CLR) and at
// Pair_Fault (CLR ToothFault; SETB after a fault). Format 6's table (Co_Pair), which the mark's 13th
// and 14th bytes point to, gives their places, the crank events and the coil's numbers.

#include "pair.h"

#if defined(FEATURE_MAP_UNIT)

#include "map_unit.h"
#include "unit_ram.h"

static constexpr uint32_t MARK = 0x2000;
static constexpr uint32_t FORMAT = 6;
static constexpr uint32_t RETI = 0x32, AJMP = 0x01, SETB_BIT = 0xD2, CLR_BIT = 0xC2;
static constexpr uint32_t REFERENCE = 0x03, VECTOR = 0x13;
static constexpr uint32_t MOST_EVENTS = 8;
static constexpr uint32_t MOST_MISSED = 4;
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
    // The image and its table (Co_Pair).
    volatile uint8_t *rom;
    bool on;
    uint32_t teeth, firings, turn, events;
    uint32_t event_tooth[MOST_EVENTS], event_handler[MOST_EVENTS];
    uint32_t charge_handler, arm_handler, arm_start, arm_overflows, synced_op, fault_op;
    uint32_t ping, lost, spark;
    uint32_t spark_b, arm_ofs, arm_min, margin, floor_teeth, recip32, teeth_high, mark_g;
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
    int32_t late_from, late_to;     // a read moved off a spark: its tooth's count, the one it goes on
    uint32_t late_event;
    uint32_t missed_events[MOST_MISSED], missed_count; // crank events a late run passed, to place
    // The vector.
    uint32_t vector;                // what it holds: an event (NOTHING: a RETI)
    uint32_t steered;               // the count the vector was last written after
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

// The flags' opcodes at the 8051's reference: NotSynced's CLR in step, else SETB; ToothFault's SETB
// after a fault.
static void UNIT_CODE(write_flags)()
{
    write(pair.synced_op, pair.synced ? CLR_BIT : SETB_BIT);
    write(pair.fault_op, pair.fault ? SETB_BIT : CLR_BIT);
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

// Nothing planned, the coil's events off.
static void UNIT_CODE(drop_coil)()
{
    pair.coil_on = false;
    pair.spark_due = false;
    pair.arm_at = pair.charge_at = pair.spark_at = NONE;
    pair.late_from = pair.late_to = NONE;
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

// The firing's length at slot 0's tooth (the table's first event), and the tooth period from it.
static void UNIT_CODE(slot_zero)(uint32_t now)
{
    if (pair.stamped && pair.count - pair.stamp_count == pair.teeth)
    {
        uint32_t cycles = (now - pair.stamp_time + 1) >> 1;
        pair.p = cycles > 0xFFFF ? 0xFFFF : cycles;
        pair.t64 = capture_t64(pair.p, &pair.p_old, pair.fire_valid, pair.recip32);
        pair.fire_valid = true;
    }
    pair.stamp_count = pair.count;
    pair.stamp_time = now;
    pair.stamped = true;
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
        slot_zero(now);
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
    uint32_t ahead = k + ((pair.mark_g + 255 - pair.teeth_high) >> 8); // fall 1, this many teeth on
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
    pair.late_from = pair.late_to = NONE;
    if (late_tooth != NONE)
    {
        pair.late_from = base + late_tooth - 1;
        pair.late_to = pair.late_from + (int32_t)late;
        pair.late_event = event_on(in_firing(late_tooth));
    }
    uint32_t charge = map_unit_ram(pair.charge_h) << 8 | map_unit_ram(pair.charge_l);
    pair.charge_at = base + charge_plan(pair.plan_adv, arm, charge) - 1;
    if (pair.charge_at <= (int32_t)pair.count)
        pair.charge_at = NONE;                                  // passed
    pair.coil_on = true;
    pair.plans++;
}

// The coil's events from now: the spark of this firing if its charge is still to come, else the
// next firing's (rom_lean's Coil_Start charged the first at its arm, short).
static void UNIT_CODE(start)()
{
    int32_t base = (int32_t)pair.count - (int32_t)pair.tif + 1;
    plan(base);
    if (pair.charge_at == NONE || pair.arm_at <= (int32_t)pair.count)
        plan(base + (int32_t)pair.teeth);
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
        uint32_t event = c == pair.late_to ? pair.late_event : c == pair.late_from ? NOTHING : event_back(back);
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
    if (next == pair.late_to)
        return pair.late_event;
    uint32_t event = next == pair.late_from ? NOTHING : event_on(pair.tif == pair.teeth ? 1 : pair.tif + 1);
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
        return;
    }
    if (address != previous + 1)
        return;
    if (previous == VECTOR)
        tooth(now);
    else if (previous == REFERENCE)
        reference(now);
    else if (previous == pair.spark)
        pair.spark_due = pair.synced != 0;
    else if (previous == pair.ping)
        write(pair.ping + 1, ++pair.beat & 0xFF);
    else if (previous == pair.lost)
        unsync();
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
    pair.mark_g = at(u + 25);
    pair.charge_l = at(u + 26);
    pair.charge_h = at(u + 27);
    pair.adv_at = at(u + 28);
    bool reach = pair.charge_handler < 0x800 && pair.arm_handler < 0x800; // within the vector's AJMP
    for (uint32_t i = 0; i < pair.events; i++)
        reach = reach && pair.event_handler[i] < 0x800;
    pair.on = reach;
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
    else
        return 0;
    return 1;
}

// pair_test is called by DME-Sim alone: this keeps it in the bank (the linker drops what nothing
// refers to).
static uint32_t (*volatile pair_entry)(uint32_t) UNIT_DATA = pair_test;

#endif // FEATURE_MAP_UNIT
