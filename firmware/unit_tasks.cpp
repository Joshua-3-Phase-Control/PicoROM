// The map unit's tasks (unit_tasks.h): rom_lean's slow work as its routines make it, each step
// noted with the 8051's instructions it stands for. DME-Sim's tools/coproc_maps/make_coproc.py
// writes each task's description, and DME-Sim checks each task against rom_lean's routine at every
// input (P28 tests there), running this code on its emulated Cortex-M0+.

#include "unit_tasks.h"

#if defined(FEATURE_MAP_UNIT)

#include "map_unit.h"
#include "pair.h"
#include "unit_ram.h"

namespace
{

// A task at work: the image, the 8051's RAM bytes, the bytes of its description that list its inputs
// and its outputs, and its data (task_of keeps them within the image's 64 KiB, so that a byte of them
// is read with no mask), and (format 18) the own values that go with ram (map_unit_owns_on).
struct Task
{
    const uint8_t *rom;
    uint8_t *ram;
    const uint8_t *inputs, *outputs, *data;
    uint32_t *own;
};

__force_inline uint32_t at(const uint8_t *rom, uint32_t address)
{
    return rom[address & 0xFFFF];
}

__force_inline uint32_t get(const Task &t, uint32_t i)
{
    return t.data[i];
}

__force_inline uint32_t word(const Task &t, uint32_t i)
{
    return get(t, i) << 8 | get(t, i + 1);
}

// Output j (from 0) into the 8051's RAM byte the description names.
__force_inline void put(const Task &t, uint32_t j, uint32_t value)
{
    t.ram[t.outputs[j] & 0x7F] = value & 0xFF;
}

// An 8051 bit (bit address 00h-7Fh: RAM 20h-2Fh), as the bytes sent hold it.
__force_inline uint32_t bit(const Task &t, uint32_t address)
{
    return t.ram[0x20 + (address >> 3 & 0x0F)] >> (address & 7) & 1;
}

// An 8051 bit set to on, or cleared, in the RAM bytes kept (a task's own decision, for pair.cpp too).
__force_inline void set_bit(const Task &t, uint32_t address, bool on)
{
    uint8_t &byte = t.ram[0x20 + (address >> 3 & 0x0F)];
    uint32_t mask = 1u << (address & 7);
    byte = on ? byte | mask : byte & ~mask;
}

// Format 8: a RAM byte the unit keeps for the 8051 (its own, not sent), whose address is the task's
// data byte i, set (byte_of reads it).
__force_inline void own(const Task &t, uint32_t i, uint32_t value)
{
    t.ram[t.data[i] & 0x7F] = value & 0xFF;
}

// The 8051's RAM byte whose address is the task's data byte i, as sent.
__force_inline uint32_t byte_of(const Task &t, uint32_t i)
{
    return t.ram[get(t, i) & 0x7F];
}

// The task's input k (from 0, in its description's order), as sent.
__force_inline uint32_t input(const Task &t, uint32_t k)
{
    return t.ram[t.inputs[k] & 0x7F];
}

// Format 9: the bytes the pipeline keeps are the unit's own, never sent (the mark's fifth byte).
__force_inline bool kept(const Task &t)
{
    return at(t.rom, 0x2004) >= PIPELINE_FORMAT;
}

constexpr uint32_t WIDE_FORMAT = 18; // the format with 16-bit inputs (docs/Paired_Precision.md)

// Format 18 (step A): a speed compared in rpm, the P28's own (map_unit.h's 80h), its thresholds words
// in rpm where the bytes were, and the maps whose values are speeds 16 bits in rpm.
__force_inline bool wide(const Task &t)
{
    return at(t.rom, 0x2004) >= WIDE_FORMAT;
}

// The speed a threshold is compared with: the speed byte as given, or (format 18) the speed in rpm.
__force_inline uint32_t speed_of(const Task &t, uint32_t byte)
{
    return wide(t) ? t.own[0] : byte;                                   // MAP_UNIT_SPEED
}

// A speed constant at address: its byte, or (format 18) its word in rpm, high byte first.
__force_inline uint32_t speed_at(const Task &t, uint32_t address)
{
    return wide(t) ? at(t.rom, address) << 8 | at(t.rom, address + 1) : at(t.rom, address);
}

// Format 19 (step B): the spark in 1/64 tooth, signed 16 bits: the spark's maps signed words (0 none),
// the advance's sums, limits, steps and terms; the sums and the advance the P28's own values
// (map_unit.h's 83h to 87h), the bytes Advance and AdvanceDyn the nearest half-teeth beside them.
constexpr uint32_t SPARK_FORMAT = 19;

__force_inline bool fine_spark(const Task &t)
{
    return at(t.rom, 0x2004) >= SPARK_FORMAT;
}

// A value in 1/64 tooth held to 16 bits, signed.
__force_inline int32_t held16(int32_t v)
{
    return v > 32767 ? 32767 : v < -32768 ? -32768 : v;
}

// A signed word at address (the constants in 1/64 tooth), high byte first.
__force_inline int32_t word16(const Task &t, uint32_t address)
{
    return (int16_t)(at(t.rom, address) << 8 | at(t.rom, address + 1));
}

// An own value as this task's copy has it, signed; and set, held to 16 bits.
__force_inline int32_t own16(const Task &t, uint32_t input)
{
    return (int16_t)t.own[(input - MAP_UNIT_SPEED) & (MAP_UNIT_OWNS - 1)];
}

__force_inline void set_own16(const Task &t, uint32_t input, int32_t v)
{
    t.own[(input - MAP_UNIT_SPEED) & (MAP_UNIT_OWNS - 1)] = (uint32_t)held16(v) & 0xFFFF;
}

// The nearest half-teeth (a signed byte) of a value in 1/64 tooth: the bytes beside it.
__force_inline uint32_t half_teeth(int32_t v)
{
    int32_t h = (v + 16) >> 5;
    return (uint32_t)(h > 127 ? 127 : h < -128 ? -128 : h) & 0xFF;
}

// Format 20 (step C): the load in g a cylinder: the air of a firing in 0.1 mg, the P28's own (81h; its
// filter's fraction above its 16 bits), which the maps' load axes, the load thresholds (words) and
// Boost_MaxLoad's values (words) take in place of the load byte.
constexpr uint32_t AIR_FORMAT = 20;

__force_inline bool fine_air(const Task &t)
{
    return at(t.rom, 0x2004) >= AIR_FORMAT;
}

// The air of a firing, 0.1 mg: the own value 81h's 16 bits.
__force_inline uint32_t air(const Task &t)
{
    return t.own[MAP_UNIT_LOAD - MAP_UNIT_SPEED] & 0xFFFF;
}

// An unsigned word at address, high byte first.
__force_inline uint32_t word_at(const Task &t, uint32_t address)
{
    return at(t.rom, address) << 8 | at(t.rom, address + 1);
}

// Format 21 (step D: the owner's choices of 2026-11-12): the fuel's factors words, 8000h a whole fuel
// (the bytes' /128 x 256), their product in 16-bit mantissas with exponents (F16, m x 2^(e - 15), m
// 8000h to FFFFh or 0: none), on the air's base: the air of a firing x K (fuel_k) x the trim x the
// intake air's; the enrichments the P28's own words (map_unit.h's 88h to 8Bh), their bytes beside them
// the nearest counts (/128 of the fuel), fading in shares of a count each tick.
constexpr uint32_t FUEL_FORMAT = 21;

__force_inline bool fine_fuel(const Task &t)
{
    return at(t.rom, 0x2004) >= FUEL_FORMAT;
}

// Format 22 (step E: the owner's choices of 2026-11-12): the MAP pulse, the pulse cut and the dead time in
// machine cycles. The MAP pulse's gains by coolant are fuel a kPa (Accel_MapGain's and Decel_MapGain's
// words, ug of fuel a cylinder a kPa), which task 2 makes into 1/32 cycle a count of the MAP (MapGainNow,
// DecelGainNow) by the factor task 1 makes with K (map_gain_k, MapK); the dead time a word in cycles
// (Inj_DeadTime's words), task 3's at each slow result (DeadNow: the battery's read before the firing's,
// as the pulse looked it up); the cut owed and taken words (DecelOwed, DecelCut). Each word in the RAM's
// copy low byte first, at the address its description names and the next.
constexpr uint32_t CYCLES_FORMAT = 22;
constexpr uint32_t MAP_LONGEST = 255 * 64; // the MAP pulse and the cut owed at most (format 21's 255 units)
constexpr uint32_t MAP_STOP = 4;           // the cycles Map_On's stop of TH0 adds to a pulse it lengthens

__force_inline bool fine_map(const Task &t)
{
    return at(t.rom, 0x2004) >= CYCLES_FORMAT;
}

// The RAM's word at address (its low byte) and the next (its high).
__force_inline uint32_t ram_word(const Task &t, uint32_t address)
{
    return t.ram[address & 0x7F] | t.ram[(address + 1) & 0x7F] << 8;
}

__force_inline void set_ram_word(const Task &t, uint32_t address, uint32_t value)
{
    t.ram[address & 0x7F] = value & 0xFF;
    t.ram[(address + 1) & 0x7F] = value >> 8 & 0xFF;
}

struct F16
{
    uint32_t m;
    int32_t e;
};

// A word, 8000h one, as an F16.
__force_inline F16 f16_word(uint32_t v)
{
    v &= 0xFFFF;
    if (v == 0)
        return {0, 0};
    int32_t e = 0;
    while (v < 0x8000)
    {
        v <<= 1;
        e--;
    }
    return {v, e};
}

// A whole number as an F16 (bits past its top 16 dropped).
static F16 UNIT_CODE(f16_whole)(uint32_t x)
{
    if (x == 0)
        return {0, 0};
    int32_t e = 15;
    while (x >= 0x10000)
    {
        x >>= 1;
        e++;
    }
    while (x < 0x8000)
    {
        x <<= 1;
        e--;
    }
    return {x, e};
}

// 1 + x / 8000h (x a word) as an F16.
__force_inline F16 f16_one_plus(uint32_t x)
{
    uint32_t v = 0x8000 + (x & 0xFFFF);
    return v >= 0x10000 ? F16{v >> 1, 1} : F16{v, 0};
}

// a x b, the product's top 16 bits (truncated).
__force_inline F16 f16_mul(F16 a, F16 b)
{
    if (a.m == 0 || b.m == 0)
        return {0, 0};
    uint32_t p = a.m * b.m;
    return p >= 0x80000000u ? F16{p >> 16, a.e + b.e + 1} : F16{p >> 15, a.e + b.e};
}

// a / b (b 0: none), the SIO's divider's quotient.
static F16 UNIT_CODE(f16_div)(F16 a, F16 b)
{
    if (a.m == 0 || b.m == 0)
        return {0, 0};
    return a.m < b.m ? F16{map_unit_divide32(a.m << 16, b.m), a.e - b.e - 1}
                     : F16{map_unit_divide32(a.m << 15, b.m), a.e - b.e};
}

// The square root of x, rounded down, a bit at a time (the bank calls no library).
__force_inline uint32_t root32(uint32_t x)
{
    uint32_t r = 0, bit = 1u << 30;
    while (bit > x)
        bit >>= 2;
    while (bit != 0)
    {
        if (x >= r + bit)
        {
            x -= r + bit;
            r = (r >> 1) + bit;
        }
        else
            r >>= 1;
        bit >>= 2;
    }
    return r;
}

// The square root of a as an F16: its mantissa's (with an even exponent) to 16 bits.
static F16 UNIT_CODE(f16_root)(F16 a)
{
    if (a.m == 0)
        return {0, 0};
    int32_t p = a.e - 15;                                               // a = m x 2^p
    uint32_t m = a.m;
    if (p & 1)
    {
        m <<= 1;
        p -= 1;
    }
    F16 r = f16_whole(root32(m << 14));                                 // the root of m x 2^14: of m x 2^7
    r.e += p / 2 - 7;
    return r;
}

// K: the cycles of 2 us a turn takes for 0.1 mg of air at lambda 1 (half an injection's, one each two
// turns): 1.5 x 10^8 / (the stoichiometric ratio x 100 x the injectors' flow, cc/min, x the fuel's
// density, g/L) x the root of the flow's pressure over the rail's (kPa).
static F16 UNIT_CODE(fuel_k)(uint32_t stoich, uint32_t flow, uint32_t density, uint32_t rated, uint32_t rail)
{
    F16 below = f16_mul(f16_mul(f16_whole(stoich), f16_whole(flow)), f16_whole(density));
    return f16_mul(f16_div(f16_whole(150000000), below), f16_root(f16_div(f16_whole(rated), f16_whole(rail))));
}

// Format 22: the MAP pulse's gains' factor, 1/32 cycle a count of the MAP for 1 ug of fuel a kPa: the MAP
// sensor's Pa a count x 32 / 1,000 x the cycles 1 ug of fuel takes, 30,000 / (the injectors' flow x the
// fuel's density) x the root, which is K x the stoichiometric ratio (x 100) / 5,000,000: K x the ratio x
// Pa / 156,250 (no division or root of its own).
static F16 UNIT_CODE(map_gain_k)(F16 k, uint32_t stoich, uint32_t pa)
{
    return f16_div(f16_mul(k, f16_whole(stoich * pa)), f16_whole(156250));
}

// Format 22: the factor at a in the RAM's copy: its mantissa low byte first, then its exponent (signed).
__force_inline F16 map_k_at(const Task &t, uint32_t a)
{
    return {(uint32_t)t.ram[a & 0x7F] | (uint32_t)t.ram[(a + 1) & 0x7F] << 8, (int8_t)t.ram[(a + 2) & 0x7F]};
}

// The base (a whole number to 16 bits) x s in whole cycles, rounded down, held to 16 bits.
__force_inline uint32_t f16_apply(uint32_t base, F16 s)
{
    if (s.m == 0 || base == 0)
        return 0;
    int32_t shift = 15 - s.e;
    if (shift < 0)
        return 0xFFFF;                                                  // (s 2 or more: the base x 2 at least)
    if (shift >= 32)
        return 0;
    uint32_t v = (base & 0xFFFF) * s.m >> shift;
    return v > 0xFFFF ? 0xFFFF : v;
}

// An own word (map_unit.h's input), as this task's copy has it.
__force_inline uint32_t own_word(const Task &t, uint32_t input)
{
    return t.own[(input - MAP_UNIT_SPEED) & (MAP_UNIT_OWNS - 1)] & 0xFFFF;
}

// The nearest count of an enrichment's word (/128 of the fuel: the word / 256), held to FFh.
__force_inline uint32_t nearest_count(uint32_t w)
{
    uint32_t c = (w + 0x80) >> 8;
    return c > 0xFF ? 0xFF : c;
}

// The time a turn running: the base x the slow factor s x (1 + the warm-up's weight) x (1 + the
// after-start) x the main map x (1 + the acceleration's), held to 16 bits (fuel_time's, inline: a firing's
// L1 job; pair_test's through running_time).
__force_inline uint32_t fuel_running(F16 s, uint32_t base, uint32_t warm, uint32_t after, uint32_t map, uint32_t accel)
{
    s = f16_mul(s, f16_one_plus(warm));
    s = f16_mul(s, f16_one_plus(after));
    s = f16_mul(s, f16_word(map));
    return f16_apply(base, f16_mul(s, f16_one_plus(accel)));
}

static uint32_t UNIT_CODE(running_time)(F16 s, uint32_t base, uint32_t warm, uint32_t after, uint32_t map, uint32_t accel)
{
    return fuel_running(s, base, warm, after, map, accel);
}

// The cranking's factor from Crank_Fuel's word as decayed (2^-11 a whole factor): under 1, none.
__force_inline F16 crank_factor(uint32_t fuel)
{
    if (fuel < 0x800)
        return {0, 0};
    F16 f = f16_whole(fuel);
    f.e -= 11;
    return f;
}

// The time a turn starting: the base x s x Cal_StartFactor x the cranking's factor (if any) x (1 + the
// acceleration's), held to 16 bits.
static uint32_t UNIT_CODE(fuel_starting)(F16 s, uint32_t base, uint32_t factor, uint32_t crank, uint32_t accel)
{
    s = f16_mul(s, f16_word(factor));
    F16 c = crank_factor(crank);
    if (c.m != 0)
        s = f16_mul(s, c);
    return f16_apply(base, f16_mul(s, f16_one_plus(accel)));
}

// A fade's share of a count (256 a period) at the tick counter's remainder r by the period (0: a count).
__force_inline uint32_t fade_share(uint32_t period, uint32_t r)
{
    return period == 0 ? 0x100 : map_unit_divide32((r + 1) << 8, period) - map_unit_divide32(r << 8, period);
}

// Shift_Left: R7:R6:R5 left a bit at a time while the count lasts and R7's top bit is clear; the
// count left. A value not 0 reaches the top bit within 23 shifts; 0 shifts its whole count away (up
// to 255 turns on the 8051), so that ends here at once with the count 0, as there.
__force_inline uint32_t shift_left(uint32_t *wide, uint32_t count)
{
    if (*wide == 0)
        return 0;
    while (count != 0 && !(*wide & 0x800000))
    {
        *wide <<= 1;
        count--;
    }
    return count;
}

// Mul_Normalise: R7:R6 x a / 256 by Mul16x8 (R7:R6:R5), then Shift_Left by the exponent + 1 (INC A, 8
// bits), the count left the exponent (FuelShift).
__force_inline void normalise(uint32_t *r, uint32_t *shift, uint32_t a)
{
    uint32_t wide = *r * a;
    *shift = shift_left(&wide, (*shift + 1) & 0xFF);
    *r = wide >> 8 & 0xFFFF;
}

// Mul_OnePlus: R7:R6 x (1 + a / 128): none for 0 (JZ); 128 + a (ADD A,#80h), when it carries halved
// (RRC A, the carry in) with the exponent a step on (INC FuelShift); then Mul_Normalise.
__force_inline void one_plus(uint32_t *r, uint32_t *shift, uint32_t a)
{
    if (a == 0)
        return;
    a += 0x80;
    if (a > 0xFF)
    {
        a = 0x80 | (a & 0xFF) >> 1;
        *shift = (*shift + 1) & 0xFF;
    }
    normalise(r, shift, a);
}

__force_inline uint32_t value(const Task &t, uint32_t id)
{
    return unit_map_value(id, t.ram, t.own);
}

// A spark map's value: a signed word in 1/64 tooth.
__force_inline int32_t spark_map(const Task &t, uint32_t id)
{
    return (int16_t)value(t, id);
}

// Sum_Map: the sum and the map's value less 20, mod 256 (ADD A,B; CLR C; SUBB A,#14h).
__force_inline uint32_t sum_map(const Task &t, uint32_t sum, uint32_t id)
{
    return (value(t, id) + sum - 0x14) & 0xFF;
}

// R7:R6 = 50 / the speed as an 8.8 number: DIV AB's quotient, then Divide_Fraction's byte (the
// remainder x 256 / the speed, a bit at a time); together 3200h / the speed, made here by the same
// restoring division. The speed 0: DIV AB keeps A and B (32h and 0, as DME-Sim's 8051 does; the
// manual leaves them undefined) and each of Divide_Fraction's subtractions goes (CPL A: FFh).
__force_inline uint32_t per_turn(uint32_t rpm)
{
    if (rpm == 0)
        return 0x32FF;
    uint32_t quotient = 0, rest = 0;
    for (uint32_t bit = 0x2000; bit != 0; bit >>= 1)                    // 3200h's 14 bits, the top first
    {
        rest = rest << 1 | ((0x3200 & bit) != 0);
        quotient <<= 1;
        if (rest >= rpm)
        {
            rest -= rpm;
            quotient |= 1;
        }
    }
    return quotient;
}

// Afm_Airflow: the air flow (g/min) at the AFM's code from Afm_Flow's 65 points (high byte first, one
// every 4 codes): the lower point, plus the rise to the next x the code's quarters + 2 (R5:R4), its
// high byte's product kept to 8 bits (ADD A,R5), / 4; R1:R0, 16 bits.
__force_inline uint32_t airflow(const Task &t, uint32_t table, uint32_t code)
{
    table += (code & 0xFC) >> 1;                                        // ANL A,#0FCh; RR A
    uint32_t lower = at(t.rom, table) << 8 | at(t.rom, table + 1);
    uint32_t rise = (at(t.rom, table + 2) << 8 | at(t.rom, table + 3)) - lower;   // SUBB, SUBB
    uint32_t quarters = code & 3;
    uint32_t low = (rise & 0xFF) * quarters + 2;                        // MUL AB; ADD A,#02h; ADDC A,#00h
    uint32_t high = ((rise >> 8 & 0xFF) * quarters + (low >> 8)) & 0xFF;  // MUL AB (B dropped); ADD A,R5
    return (lower + ((high << 8 | (low & 0xFF)) >> 2)) & 0xFFFF;        // RRC x 4; ADD A,R0; ADDC A,R5
}

} // namespace

// Step_Table_Lookup: a one-axis table's value of the interval its input is in, as Map_Axis finds it.
// The table: its input's RAM address, the point count n, n points added to the input from the last
// until one carries (R1 that point, 1 if none does), then the values (R1 - 1's).
static uint32_t UNIT_CODE(step_table)(const uint8_t *rom, const uint8_t *ram, uint32_t table)
{
    uint32_t n = at(rom, table + 1), r1 = n, r0 = ram[at(rom, table) & 0x7F];
    for (;;)
    {
        uint32_t sum = at(rom, table + 1 + r1) + r0; // MOV A,R1; MOVC A,@A+DPTR (DPTR the table + 1); ADD A,R0
        r0 = sum & 0xFF;
        if (sum > 0xFF)
            break;                                   // JC Map_Axis_Found
        r1 = (r1 - 1) & 0xFF;                        // DJNZ R1,Map_Axis_Scan
        if (r1 == 0)
        {
            r1 = 1;                                  // INC R1
            break;
        }
    }
    return at(rom, table + 2 + n + ((r1 - 1) & 0xFF)); // DEC R1; MOV A,R1; MOVC A,@A+DPTR (DPTR past the axis)
}

// Slow_WarmUp. Data: Fuel_WarmUp's id, Fuel_WarmUp_Cold's, ColdStartFuel's bit. Out: WarmUp. Format 21:
// the warm-up a word, the own value 88h, WarmUp its nearest count.
static void UNIT_CODE(warm_up)(const Task &t)
{
    uint32_t v = value(t, get(t, bit(t, get(t, 2))));   // the cold map after a cold start
    if (fine_fuel(t))
    {
        t.own[MAP_UNIT_WARM_UP - MAP_UNIT_SPEED] = v & 0xFFFF;
        v = nearest_count(v);
    }
    put(t, 0, v);
}

// Slow_Fuel. Data: Fuel_Quality_Table's address, Constants', Cal_FuelTrim's and Cal_InjScale's places
// in Constants, Fuel_AirTemp's id. Out: FuelFactorL, FuelFactorH, FuelShift. Format 21: Cal_FuelTrim
// words, and in Cal_InjScale's place Cal_Stoich's, the four words after it Cal_InjBaseFlow,
// Cal_FuelDensity, Cal_InjFlowKpa and Cal_RailKpa: K x the trim x Fuel_AirTemp (words), its mantissa
// FuelFactorH:L and its exponent FuelShift (signed). Format 22: data Cal_MapPa's address and MapK's; the
// MAP pulse's gains' factor (map_gain_k) into MapK, for task 2.
static void UNIT_CODE(fuel)(const Task &t)
{
    uint32_t constants = word(t, 2);
    uint32_t step = step_table(t.rom, t.ram, word(t, 0));
    if (fine_fuel(t))
    {
        uint32_t c = constants + get(t, 5);
        F16 k = fuel_k(word_at(t, c), word_at(t, c + 2), word_at(t, c + 4), word_at(t, c + 6), word_at(t, c + 8));
        uint32_t trim = word_at(t, constants + get(t, 4) + 2 * step);
        F16 s = f16_mul(f16_mul(k, f16_word(trim)), f16_word(value(t, get(t, 6))));
        put(t, 0, s.m);
        put(t, 1, s.m >> 8);
        put(t, 2, (uint32_t)s.e);
        if (fine_map(t))
        {
            F16 g = map_gain_k(k, word_at(t, c), word_at(t, word(t, 7)));
            uint32_t a = get(t, 9);
            t.ram[a & 0x7F] = g.m & 0xFF;
            t.ram[(a + 1) & 0x7F] = g.m >> 8 & 0xFF;
            t.ram[(a + 2) & 0x7F] = (uint32_t)g.e & 0xFF;
        }
        return;
    }
    uint32_t trim = at(t.rom, constants + ((step + get(t, 4)) & 0xFF)); // ADD A,#(Cal_FuelTrim - Constants); MOVC
    uint32_t r = trim * value(t, get(t, 6)), shift = 2;                 // Fuel_AirTemp; MUL AB: R7:R6; FuelShift
    normalise(&r, &shift, at(t.rom, constants + get(t, 5)));            // x Cal_InjScale (Mul_Normalise)
    put(t, 0, r);                                                       // R6
    put(t, 1, r >> 8);                                                  // R7
    put(t, 2, shift);                                                   // the exponent
}

// Main_Ign. Data: Ign_Coolant's id, Ign_Coolant_Cold's, Ign_AirTemp's, Accel_MapGain's, ColdStartIgn's
// bit; format 15, Decel_MapGain's id; format 22, MapK's address (task 1's factor). Out: IgnSlow,
// MapGainNow; format 15, DecelGainNow (the pulse cut's gain by coolant, which task 18 takes, as it takes
// MapGainNow). Format 22: the two gains words, 1/32 cycle a count of the MAP: the maps' fuel a kPa x MapK,
// held to FFFFh.
static void UNIT_CODE(ign)(const Task &t)
{
    if (fine_spark(t))                                                  // format 19: in 1/64 tooth, the
        set_own16(t, MAP_UNIT_IGN_SLOW, spark_map(t, get(t, bit(t, get(t, 4)))) + spark_map(t, get(t, 2)));
    else                                                                //   P28's own
    {
        uint32_t sum = sum_map(t, 0, get(t, bit(t, get(t, 4))));        // the cold map after a cold start
        put(t, 0, sum_map(t, sum, get(t, 2)));
    }
    if (fine_map(t))
    {
        F16 k = map_k_at(t, get(t, 6));                                 // MapK
        set_ram_word(t, t.outputs[1], f16_apply(value(t, get(t, 3)), k));     // MapGainNow
        set_ram_word(t, t.outputs[2], f16_apply(value(t, get(t, 5)), k));     // DecelGainNow
        return;
    }
    put(t, 1, value(t, get(t, 3)));
    if (at(t.rom, 0x2004) >= DECEL_FORMAT)
        put(t, 2, value(t, get(t, 5)));
}

// Slow_Charge's time: (Dwell_Time x CHARGE_K + CHARGE_0) / 256, a byte at a time. Data: Dwell_Time's id,
// CHARGE_K's low byte and high byte, CHARGE_0's; format 22, Inj_DeadTime's id and DeadNow's address. Out:
// ChargeL, ChargeH; format 22, the dead time by battery into DeadNow (a word, cycles), which the pulse's
// task and the MAP pulse take (the battery as the pulse's task had it: the read before its firing's).
static void UNIT_CODE(charge)(const Task &t)
{
    uint32_t time = value(t, get(t, 0));
    uint32_t low = time * get(t, 1), sum = (low & 0xFF) + get(t, 3);    // MUL AB; ADD A,#LOW(CHARGE_0)
    uint32_t r6 = ((low >> 8) + get(t, 4) + (sum >> 8)) & 0xFF;         // ADDC A,#HIGH(CHARGE_0): its carry lost
    uint32_t high = time * get(t, 2);                                   //   (MUL AB clears C)
    sum = (high & 0xFF) + r6;                                           // ADD A,R6
    put(t, 0, sum);                                                     // ChargeL
    put(t, 1, (high >> 8) + (sum >> 8));                                // CLR A; ADDC A,B: ChargeH
    if (fine_map(t))
        set_ram_word(t, get(t, 6), value(t, get(t, 5)));                // DeadNow: Inj_DeadTime
}

// Ign_Advance: the advance's maps summed, each less 20 (Sum_Map). Starting: by speed, then by coolant;
// at idle: by coolant (the cold map after a cold start), then by speed; off idle: the slow sum and the
// main map (Sum_Add). Data: Ign_StartSpeed's id, Ign_StartCoolant's, Ign_IdleCoolant's,
// Ign_IdleCoolant_Cold's, Ign_IdleSpeed's, Ign_Main's; StartMode's bit, IdleSwitch's, ColdStartIgn's;
// IgnSlow's RAM address. Out: Handover.
static void UNIT_CODE(advance)(const Task &t)
{
    if (fine_spark(t))                                                  // format 19: the sum in 1/64 tooth
    {
        int32_t whole;
        if (bit(t, get(t, 6)))
            whole = spark_map(t, get(t, 0)) + spark_map(t, get(t, 1));
        else if (bit(t, get(t, 7)))
            whole = spark_map(t, get(t, bit(t, get(t, 8)) ? 3 : 2)) + spark_map(t, get(t, 4));
        else
            whole = spark_map(t, get(t, 5)) + own16(t, MAP_UNIT_IGN_SLOW);
        set_own16(t, MAP_UNIT_HANDOVER, whole);
        return;
    }
    uint32_t sum;
    if (bit(t, get(t, 6)))                                              // JNB StartMode,Ign_Running
        sum = sum_map(t, sum_map(t, 0, get(t, 0)), get(t, 1));
    else if (bit(t, get(t, 7)))                                         // JNB IdleSwitch,Ign_OffIdle
        sum = sum_map(t, sum_map(t, 0, get(t, bit(t, get(t, 8)) ? 3 : 2)), get(t, 4));
    else
        sum = (value(t, get(t, 5)) + byte_of(t, 9) - 0x14) & 0xFF;     // MOV B,IgnSlow; Map_Lookup_2D; Sum_Add
    put(t, 0, sum);                                                     // MOV Handover,B
}

// Ign_Offset to Ign_Halve: the sum with the offset by speed (Sum_Map), clamped by Cal_AdvanceMax, or
// when negative by the constant after it (Cal_AdvanceMin), each compared as unsigned bytes, then halved
// with its sign (RRC). Data: Ign_SpeedOffset's id; Constants' address; Cal_AdvanceMax's place in
// Constants; Handover's RAM address. Out: Handover.
static uint32_t UNIT_CODE(target_of)(const Task &t, uint32_t handover, uint32_t offset_id, uint32_t constants,
                                     uint32_t max_place)
{
    uint32_t sum = sum_map(t, handover, offset_id);                     // MOV B,Handover; Sum_Map
    uint32_t negative = sum >> 7;                                       // MOV C,ACC.7
    uint32_t limit = at(t.rom, constants + ((max_place + negative) & 0xFF)); // MOV A,#..; (INC A;) MOVC
    uint32_t kept = negative ? (sum >= limit ? sum : limit)             // the sum less the limit, no borrow
                             : (limit >= sum ? sum : limit);            // the limit less the sum, no borrow
    return kept >> 1 | (kept & 0x80);                                   // MOV C,ACC.7; RRC A
}

static void UNIT_CODE(offset)(const Task &t)
{
    put(t, 0, target_of(t, byte_of(t, 4), get(t, 0), word(t, 1), get(t, 3)));
}

// Fuel_Time's arithmetic: the injection time a turn (R7:R6) on the slow factor r (shift its exponent,
// kept as it was: PUSH and POP). Running (handover 80h): x (1 + warm / 128) x (1 + after / 128) x the
// main map / 128 (map), by Mul_OnePlus and Mul_Normalise. Starting: x the cranking fuel's factor (map;
// 0: none) / 256, shifted by the exponent and its own (handover). Then the base time x (1 + accel /
// 128), x the factor (Mul16x16: R7:R6:R5), shifted by the exponent.
static uint32_t UNIT_CODE(time_a_turn)(uint32_t r, uint32_t shift, uint32_t handover, uint32_t warm, uint32_t after,
                                       uint32_t map, uint32_t base, uint32_t accel)
{
    if (handover == 0x80)                                               // CJNE A,#80h,Fuel_Time_Crank
    {
        one_plus(&r, &shift, warm);                                     // WarmWeight
        one_plus(&r, &shift, after);                                    // AfterStart
        normalise(&r, &shift, map);                                     // the main map
    }
    else if (map != 0)                                                  // JZ Fuel_Time_Base
    {
        uint32_t wide = r * map;                                        // Mul16x8
        shift = shift_left(&wide, (shift + handover) & 0xFF);           // ADD A,Handover; Shift_Left
        r = wide >> 8 & 0xFFFF;
    }
    one_plus(&base, &shift, accel);                                     // AccelEnrich
    uint32_t wide = base * r >> 8;                                      // Mul16x16
    shift_left(&wide, shift);                                           // MOV A,FuelShift; Shift_Left
    return wide >> 8 & 0xFFFF;                                          // R7:R6
}

// Fuel_Time: time_a_turn, the base time LoadHi:LoadMid (starting Cal_StartLoad x 25: MOVC; MOV
// B,#19h; MUL AB). Inputs, in turn: FuelFactorL, FuelFactorH, FuelShift, Handover, WarmWeight,
// AfterStart, InjTimeL, LoadHi, LoadMid, AccelEnrich, StartMode's byte, Inj_DeadTime's input. Data:
// Inj_DeadTime's id; Constants' address; Cal_StartLoad's place there; StartMode's bit. Out: InjTimeL,
// InjTimeH; the dead time by battery in Handover and DeadNow.
static void UNIT_CODE(injection_time)(const Task &t)
{
    uint32_t base = input(t, 7) << 8 | input(t, 8);
    if (bit(t, get(t, 4)))
        base = at(t.rom, word(t, 1) + get(t, 3)) * 0x19;
    uint32_t time = time_a_turn(input(t, 1) << 8 | input(t, 0), input(t, 2), input(t, 3), input(t, 4), input(t, 5),
                                input(t, 6), base, input(t, 9));
    uint32_t dead = value(t, get(t, 0));                                // Inj_DeadTime
    put(t, 0, time);                                                    // InjTimeL: R6
    put(t, 1, time >> 8);                                               // InjTimeH: R7
    put(t, 2, dead);                                                    // Handover
    put(t, 3, dead);                                                    // DeadNow
}

// ---------------------------------------------------------------------------------------------------
// Tasks 9 and 10 (format 7): the pulse, rom_lean's pipeline stages 5 to 7 in one, the turn's or made
// again. Inputs, in turn: Rpm, the byte of StartMode's bit (23h), LamInt, FirePeriodL, FirePeriodH,
// FuelFactorL, FuelFactorH, FuelShift, WarmWeight, AfterStart, LoadHi, LoadMid, AccelEnrich, and
// Coolant, Battery and Fuel_Main's second input (the snoop's In5, or Load). Data: the ids of Fuel_Main,
// Fuel_Idle, Crank_Fuel, Crank_DecaySpeed, Crank_DecayCount, Crank_DecayRpm and Inj_DeadTime;
// Constants' address; the places there of Cal_StartLoad, Cal_CrankCount and Cal_InjMinTime;
// Resume_Ramp_Rpm's and Resume_Ramp_Throttle's addresses; the bits StartMode, IdleSwitch, CrankDecay,
// ResumeRamp, ResumeByRpm; the pulse's longest a turn (FIRINGS x 60: 15/16 of its two turns, x 32 /
// the firing) and starting (30: of its firing); INJ_OFS; then the bytes the unit keeps (format 8),
// InjCount's and SlewCount's addresses, and SlewResume's bit. Out: InjTimeL, InjTimeH (the injection
// time), DeadNow, the flags (PulseFlags, unit_tasks.h; the unit's own count and bits as they say);
// TH0's load for the pulse goes into the 8051's code (pair_pulse).

namespace
{

// TH0's load: its start and its overflows (InjHi).
struct Pulse
{
    uint32_t start, overflows;
};

__force_inline uint32_t rotate(uint32_t a)
{
    return (a << 1 | a >> 7) & 0xFF;                                    // RL A
}

} // namespace

// Fuel_Crank: the cranking fuel's factor (the byte returned) and its exponent. By coolant (Crank_Fuel);
// decaying (CrankDecay, or now: over Crank_DecaySpeed's speed or past Cal_CrankCount injections, then
// InjCount 0) x Crank_DecayCount by those injections x Crank_DecayRpm by speed / 65,536, the fuel
// normalised first (its exponent 5 a step less for each step, its floor 8 rotated with it); under its
// floor none.
static uint32_t UNIT_CODE(crank_fuel)(const Task &t, uint32_t *exponent, uint32_t *flags)
{
    uint32_t r7 = value(t, get(t, 2)), r6 = 8, r3 = 5;                  // Crank_Fuel; MOV R6,#08h; MOV R3,#05h
    bool decaying = bit(t, get(t, 18));                                 // JB CrankDecay,Fuel_Crank_Decaying
    if (!decaying && (value(t, get(t, 3)) < speed_of(t, input(t, 0))    // Crank_DecaySpeed; SUBB A,Rpm; JC
                      || at(t.rom, word(t, 7) + get(t, 10)) < byte_of(t, 24))) // Cal_CrankCount; InjCount
    {
        *flags |= PULSE_DECAY;                                          // SETB CrankDecay
        set_bit(t, get(t, 18), true);
        own(t, 24, 0);                                                  // MOV InjCount,#00h
        decaying = true;
    }
    if (decaying && r7 != 0)                                            // MOV A,R7; JZ Fuel_Crank_Test
    {
        uint32_t decay = value(t, get(t, 4)) * value(t, get(t, 5)) >> 8; // Crank_DecayCount x Crank_DecayRpm: B
        uint32_t a = r7;
        while (!(a & 0x80))                                             // JB ACC.7,Fuel_Crank_Scale
        {
            a = rotate(a);                                              // RL A
            r6 = rotate(r6);                                            // XCH A,R6; RL A; XCH A,R6
            r3 = (r3 - 1) & 0xFF;                                       // DEC R3
        }
        uint32_t scaled = a * decay;                                    // MUL AB
        r7 = (scaled >> 8) + (scaled >> 7 & 1);                         // MOV R7,B; JNB ACC.7; INC R7
    }
    *exponent = r3;
    return r7 < r6 ? 0 : r7;                                            // CJNE A,06h; JNC; CLR A
}

// Fuel_Mode: the injection time (*time: the time a turn x lambda, twice it unless starting and at most
// F800h, the shortest at least) and the pulse's TH0 load: the time ramped after an overrun cut (16
// injections; made again at the step it was made at; format 15, less the cut a fall owes), the dead time
// on (format 22: in cycles, and the cut too), held over 15/16 of its turns (then twice that), less INJ_OFS. The turn's pulse counted (InjCount,
// to 80h; not starting).
static Pulse UNIT_CODE(fuel_mode)(const Task &t, uint32_t *time, uint32_t dead, bool again, uint32_t *count,
                                  uint32_t *flags)
{
    bool starting = bit(t, get(t, 16));
    uint32_t r = *time, lamint = input(t, 2);
    if (lamint != 0x80)                                                 // CJNE A,#80h,Fuel_Mode_Lambda
    {
        uint32_t wide = r * (0x6000 + lamint * 0x40) >> 8;              // MUL AB; ADD A,#60h; Mul16x16
        shift_left(&wide, 1);                                           // MOV A,#01h; Shift_Left
        r = wide >> 8 & 0xFFFF;
    }
    if (!starting)                                                      // JB StartMode,Fuel_Mode_T
    {
        r *= 2;                                                         // ADD A,R6; ADDC A,R7
        if (r >= 0xF800)                                                // JC; CJNE A,#0F8h; JC
            r = 0xF800;
    }
    uint32_t least = at(t.rom, word(t, 7) + get(t, 11)) * 4;            // Cal_InjMinTime; MOV B,#04h; MUL AB
    if (r < least)
        r = least;
    *time = r;                                                          // InjTimeL, InjTimeH
    if (!starting && bit(t, get(t, 19)))                                // JNB ResumeRamp,Fuel_Mode_Counted
    {
        uint32_t step = ((again ? 0xFF : 0) + *count) & 0xFF;           // MOV A,#0FFh; ADD A,InjCount
        if (step >= 0x10)                                               // CJNE A,#10h; JC Fuel_Mode_Scale
            *flags |= PULSE_RAMP_END | PULSE_SLEW_END;                  // CLR ResumeRamp; Fuel_Mode_Slew_At
        else
        {
            uint32_t factor = at(t.rom, word(t, bit(t, get(t, 20)) ? 12 : 14) + step);
            if (factor == 0x40)                                         // CJNE A,#40h; ACALL Fuel_Mode_Slew_At
                *flags |= PULSE_SLEW_END;
            uint32_t wide = r * factor;                                 // Mul16x8
            shift_left(&wide, 2);                                       // MOV A,#02h; Shift_Left
            r = wide >> 8 & 0xFFFF;
        }
    }
    // Format 15 (DME-Sim's docs/Paired_Tip_Outs.md): the cut a fall of the MAP owes (task 18's units of 64
    // cycles, DecelOwed) taken off the turn's pulse, down to the shortest time; the units it took kept
    // (DecelCut) for the pulse made again, which takes as many. None starting.
    bool cycles = fine_map(t);
    if (cycles && !starting)                                            // format 22: in cycles, words, down
    {                                                                   //   to the shortest time exactly
        uint32_t owed = ram_word(t, get(t, 27)), room = r > least ? r - least : 0;
        uint32_t cut = again ? ram_word(t, get(t, 28)) : owed;
        if (cut > room)
            cut = room;
        r -= cut;
        if (!again)
        {
            set_ram_word(t, get(t, 27), owed - cut);                    // DecelOwed
            set_ram_word(t, get(t, 28), cut);                           // DecelCut
        }
    }
    else if (at(t.rom, 0x2004) >= DECEL_FORMAT && !starting)
    {
        uint8_t &owed = t.ram[get(t, 27) & 0x7F], &taken = t.ram[get(t, 28) & 0x7F];
        uint32_t room = r > least ? (r - least) >> 6 : 0, cut = again ? taken : owed;
        if (cut > room)
            cut = room;
        r -= cut << 6;
        if (!again)
        {
            owed = owed - cut;
            taken = cut;
        }
    }
    if (!again && !starting && *count < 0x80)                           // JB ACC.7,Fuel_Mode_Dead; INC InjCount
        (*count)++;
    uint32_t pulse = (cycles ? dead : dead * 5) + r;                    // MOV B,#05h; MUL AB; ADD A,R6; ADDC A,R7
    if (pulse > 0xFFFF)
        pulse = 0xFF00 | (pulse & 0xFF);                                // MOV R3,#0FFh
    uint32_t wide = (input(t, 4) << 8 | input(t, 3)) * get(t, starting ? 22 : 21); // FirePeriod; Mul16x8
    shift_left(&wide, 3);                                               // MOV A,#03h; Shift_Left
    if ((wide >> 8 & 0xFFFF) < pulse)                                   // SUBB, SUBB: longer than that
    {
        *flags |= PULSE_HELD;                                           // SETB HeldTmp
        shift_left(&wide, 1);                                           // twice that
        pulse = wide >> 8 & 0xFFFF;
    }
    uint32_t ofs = get(t, 23);
    uint32_t overflows = (pulse >> 8) + (((pulse & 0xFF) + 255 - ofs) >> 8); // ADD A,#(255 - INJ_OFS); ADDC A,R7
    if (overflows > 0xFF)
        return {0, 0xFF};                                               // the longest: 255 from 0
    return {(ofs - pulse) & 0xFF, overflows};                           // MOV A,#INJ_OFS; SUBB A,R6
}

// Format 21: the cranking fuel's word as Fuel_Crank decides it (its decay begun as there), Crank_Fuel (2^-11
// a whole factor), decaying x Crank_DecayCount x Crank_DecayRpm (words, 2^-16 a whole).
static uint32_t UNIT_CODE(crank_word)(const Task &t, uint32_t *flags)
{
    uint32_t fuel = value(t, get(t, 2));                                // Crank_Fuel
    bool decaying = bit(t, get(t, 18));
    if (!decaying && (value(t, get(t, 3)) < speed_of(t, input(t, 0))    // Crank_DecaySpeed
                      || at(t.rom, word(t, 7) + get(t, 10)) < byte_of(t, 24))) // Cal_CrankCount; InjCount
    {
        *flags |= PULSE_DECAY;
        set_bit(t, get(t, 18), true);
        own(t, 24, 0);
        decaying = true;
    }
    if (decaying && fuel != 0)
        fuel = fuel * (value(t, get(t, 4)) * value(t, get(t, 5)) >> 16) >> 16;
    return fuel;
}

// Format 21: the time a turn (cycles): the base, the air of a firing (starting Cal_StartAir), x the slow
// factor (FuelFactorH:L, FuelShift), running x (1 + the warm-up's weight) x (1 + the after-start) x the
// main map (Fuel_Idle at idle, Fuel_Main off it), starting x Cal_StartFactor x the cranking's factor (if
// any); then x (1 + the acceleration's), held to 16 bits.
static uint32_t UNIT_CODE(fuel_time)(const Task &t, uint32_t *flags)
{
    F16 s = f16_word(input(t, 6) << 8 | input(t, 5));                  // (as Slow_Fuel leaves it, normalised)
    s.e += (int8_t)input(t, 7);
    uint32_t accel = own_word(t, MAP_UNIT_ACCEL);
    if (bit(t, get(t, 16)))
    {
        uint32_t c = word(t, 7);                                        // Cal_StartAir, Cal_StartFactor
        uint32_t crank = crank_word(t, flags);
        return fuel_starting(s, word_at(t, c + get(t, 29)), word_at(t, c + get(t, 30)), crank, accel);
    }
    return fuel_running(s, air(t), own_word(t, MAP_UNIT_WARM_WEIGHT), own_word(t, MAP_UNIT_AFTER_START),
                        value(t, get(t, bit(t, get(t, 17)) ? 1 : 0)), accel);
}

// The pulse: the cranking fuel's factor starting, else the main map (Fuel_Idle at idle, Fuel_Main
// off it), then Fuel_Time's arithmetic (the base time starting Cal_StartLoad x 25), then Fuel_Mode;
// TH0's load into the 8051's code. Format 21: fuel_time, then Fuel_Mode.
static void UNIT_CODE(pulse)(const Task &t, bool again)
{
    uint32_t flags = 0, exponent = 0x80, map, base = input(t, 10) << 8 | input(t, 11), time;
    if (fine_fuel(t))
        time = fuel_time(t, &flags);
    else
    {
        if (bit(t, get(t, 16)))                                         // JB StartMode,Fuel_Crank
        {
            map = crank_fuel(t, &exponent, &flags);
            base = at(t.rom, word(t, 7) + get(t, 9)) * 0x19;            // Cal_StartLoad x 25
        }
        else
            map = value(t, get(t, bit(t, get(t, 17)) ? 1 : 0));         // JB IdleSwitch: Fuel_Idle, else Fuel_Main
        time = time_a_turn(input(t, 6) << 8 | input(t, 5), input(t, 7), exponent, input(t, 8), input(t, 9), map,
                           base, input(t, 12));
    }
    bool cycles = fine_map(t);
    uint32_t dead = cycles ? ram_word(t, t.outputs[2]) : value(t, get(t, 6));   // Inj_DeadTime (format 22:
    uint32_t count = byte_of(t, 24);                                    //   task 3's, DeadNow)                                    // InjCount (0 if the decay began)
    Pulse load = fuel_mode(t, &time, dead, again, &count, &flags);
    if (!pair_pulse(load.start, load.overflows, again))
        flags |= PULSE_LATE;
    own(t, 24, count);                                                  // the unit's own (format 8)
    if (flags & PULSE_RAMP_END)
        set_bit(t, get(t, 19), false);                                  // CLR ResumeRamp
    if ((flags & PULSE_SLEW_END) && bit(t, get(t, 26)))                 // Fuel_Mode_Slew_At
    {
        set_bit(t, get(t, 26), false);                                  // CLR SlewResume
        own(t, 25, 0);                                                  // MOV SlewCount,#00h
    }
    put(t, 0, time);                                                    // InjTimeL
    put(t, 1, time >> 8);                                               // InjTimeH
    if (!cycles)
        put(t, 2, dead);                                                // DeadNow (format 22: task 3's)
    put(t, 3, flags);                                                   // PulseFlags
}

// Format 20: the air of a firing (the flow x P / 3,000: g/min over P machine cycles of 2 us, in 0.1 mg,
// rounded, at most FFFFh; held where P isn't known) into the own value 81h, filtered as the load is by
// LoadK, its fraction kept above its 16 bits (rom_lean's filter drops its low byte; this keeps it).
static void UNIT_CODE(filter_air)(const Task &t, uint32_t flow, uint32_t p, uint32_t k)
{
    if (p == 0xFFFF || p == 0)
        return;
    uint32_t product = flow * p, fresh = map_unit_divide32(product, 3000);
    if (2 * (product - fresh * 3000) >= 3000)
        fresh++;
    if (fresh > 0xFFFF)
        fresh = 0xFFFF;
    uint32_t &own = t.own[MAP_UNIT_LOAD - MAP_UNIT_SPEED];
    int32_t state = (int32_t)((own & 0xFFFF) << 8 | (own >> 16 & 0xFF));
    int32_t diff = (int32_t)(fresh << 8) - state;
    state += diff >= 0 ? (int32_t)((uint32_t)diff * k >> 8) : -(int32_t)((uint32_t)-diff * k >> 8);
    own = (uint32_t)state >> 8 | ((uint32_t)state & 0xFF) << 16;
}

// Load_Tick's running path (the 8051 keeps the starting one): 50 / the speed x the air flow / 256
// (Mul16x16), held to 16 bits, x Cal_LoadScale / 256 (Mul16x8) and doubled unless its top bit is set:
// the new load; filtered by LoadK (Lag_Filter24, its low byte 0) into LoadHi:LoadMid. Inputs, in turn:
// Rpm, Afm, LoadK, LoadHi, LoadMid (format 9: Rpm not sent, the unit's own: its address the data's
// sixth). Data: Constants' address; Cal_LoadScale's place there; Afm_Flow's address; format 18,
// FirePeriodL's and FirePeriodH's addresses and SPEED_K (low byte first). Out: LoadHi, LoadMid.
// Format 18 (docs/Paired_Precision.md, step A): with the firing's length P in 16 bits, 50 x the air
// flow x P / SPEED_K in place of the first product: the air of a firing, none of the speed byte's
// steps (its 50 / the speed is 50 x P / SPEED_K, the speed byte SPEED_K / P rounded).
static void UNIT_CODE(load)(const Task &t)
{
    uint32_t afm = kept(t) ? 0 : 1, rpm = kept(t) ? byte_of(t, 5) : input(t, 0); // the inputs from the AFM's
    uint32_t flow = airflow(t, word(t, 3), input(t, afm)), wide;
    uint32_t p = at(t.rom, 0x2004) >= WIDE_FORMAT ? byte_of(t, 7) << 8 | byte_of(t, 6) : 0xFFFF;
    if (p != 0xFFFF && p != 0)                                          // format 18: the air a firing,
    {                                                                   //   50 x flow x P / SPEED_K in
        uint32_t k = get(t, 8) | get(t, 9) << 8 | get(t, 10) << 16;     //   two divisions (flow x P is
        uint32_t q = map_unit_divide32(flow * p, k), r = flow * p - q * k; // 32 bits, the rest under K)
        wide = 50 * q + map_unit_divide32(50 * r, k);
    }
    else
        wide = per_turn(rpm) * flow >> 8;                               // R7:R6:R5
    if (wide > 0xFFFF)                                                  // MOV A,R7; JZ Load_Tick_Fits
        wide = 0xFFFF;                                                  // MOV R6,#0FFh; MOV R5,#0FFh
    wide *= at(t.rom, word(t, 0) + get(t, 2));                          // x Cal_LoadScale: R7:R6:R5
    if (!(wide & 0x800000))                                             // JB ACC.7,Load_Tick_Twice
        wide <<= 1;                                                     // RLC A: R5's top bit, R6, R7
    uint32_t fresh = wide >> 8 & 0xFFFF, k = input(t, afm + 1);        // R4:R3; R5 = LoadK
    uint32_t old = input(t, afm + 2) << 8 | input(t, afm + 3), state = old << 8; // R2:R1:R0, R0 = 0
    if (fresh >= old)                                                   // SUBB, SUBB: no borrow
        state += (fresh - old) * k;                                     // Mul16x8; ADD, ADDC, ADDC
    else
        state -= (old - fresh) * k;                                     // negated; Mul16x8; SUBB x 3
    put(t, 0, state >> 16);                                             // LoadHi: R2
    put(t, 1, state >> 8);                                              // LoadMid: R1
    if (fine_air(t))                                                    // format 20: the air of a firing,
        filter_air(t, flow, p, k);                                      //   filtered alike
}

// ---------------------------------------------------------------------------------------------------
// Task 11 (format 8): the advance's rest, rom_lean's pipeline stage 4 from Ign_Offset to Ign_Slew, its
// state the unit's own (never sent): Handover (the maps' sum, task 4's), Advance (its output),
// SlewCount, BoostCount, InjCount and the guard's, the cut's and the slew's bits. Inputs, in turn: Rpm,
// Load, the byte of StartMode's bit (23h: IdleSwitch's, NotSynced's and FuelCut's too), and the snoop's
// In5 and Coolant. Data: Ign_SpeedOffset's id; Constants' address; the places there of Cal_AdvanceMax,
// then Boost_MaxLoad's, Boost_MaxMap's and Overrun_CutSpeed's ids, then the places of
// Cal_BoostGuardMap, Cal_BoostDelay, Cal_BoostDelayMap, Cal_BoostRelease, Cal_BoostHyst,
// Cal_BoostReleaseTime, Cal_OverrunHyst, Cal_ResumeStepThrottle, Cal_ResumeStepRpm, Cal_SlewResume,
// Cal_SlewOverrun, Cal_SlewHighLoad, Cal_SlewNormal, Cal_SlewLoad and Cal_SlewRpm; the bits StartMode,
// IdleSwitch, NotSynced, FuelCut, BoostOver, BoostCut, ResumeRamp, ResumeByRpm, SlewHighLoad,
// SlewOverrun, SlewResume; the addresses of Handover, Advance, SlewCount, BoostCount and InjCount.
// Out: Advance, and the flags (AdvFlags, below), of which the 8051 copies the cuts (its lambda and idle
// control read them, its stream FuelCut); all kept here (pair.cpp's turn reads the cuts).

namespace
{

// The advance's flags, bit by bit (AdvFlags): the 8051 copies the cuts.
enum : uint32_t
{
    ADV_BOOST_OVER = 1,
    ADV_BOOST_CUT = 2,
    ADV_FUEL_CUT = 4,
    ADV_RESUME_RAMP = 8,
    ADV_RESUME_BY_RPM = 0x10,
    ADV_SLEW_HIGH_LOAD = 0x20,
    ADV_SLEW_OVERRUN = 0x40,
    ADV_SLEW_RESUME = 0x80,
};

// The advance's state as stage 4 takes and leaves it.
struct AdvanceState
{
    int32_t advance;                                                    // a byte, or (format 19) 1/64 tooth
    uint32_t slew, boost, count;
    bool over, cut, fuel_cut, ramp, by_rpm, high, overrun, resume;
};

__force_inline uint32_t cal(const Task &t, uint32_t place)
{
    return at(t.rom, word(t, 1) + get(t, place));
}

} // namespace

// Ign_Boost: the overboost guard (with Cal_BoostGuardMap the snoop's In5 against Boost_MaxMap, else the
// load byte against Boost_MaxLoad, both by speed): over the limit for its delay of periods (BoostCount,
// counted down at the tick: task 12), the fuel cut till at or below the release level for its time, or
// the throttle closed. Starting: none.
static void UNIT_CODE(boost_guard)(const Task &t, AdvanceState &s, bool starting, bool idle)
{
    if (starting)
    {
        s.cut = s.over = false;                                         // Ign_Boost_Release
        return;
    }
    bool on_map = cal(t, 7) != 0;                                       // Cal_BoostGuardMap
    bool airy = !on_map && fine_air(t);                                 // format 20: on the air, words
    uint32_t b = on_map ? input(t, 3) : airy ? air(t) : input(t, 1);    // MOV B,In5 / MOV B,Load
    uint32_t limit = value(t, get(t, on_map ? 5 : 4));                  // Boost_MaxMap / Boost_MaxLoad: R3
    if (!s.cut)                                                         // JB BoostCut,Ign_Boost_Cut
    {
        if (limit >= b)                                                 // CJNE A,B; JC Ign_Boost_Above
            s.over = false;                                             // within it
        else if (!s.over)
        {
            s.over = true;                                              // the delay for what is guarded
            s.boost = cal(t, on_map ? 9 : 8);                           // Cal_BoostDelayMap / Cal_BoostDelay
        }
        else if (s.boost == 0)                                          // over it long enough
        {
            s.cut = true;
            s.over = false;
        }
        return;
    }
    if (idle)                                                           // the throttle closed
    {
        s.cut = s.over = false;
        return;
    }
    uint32_t hyst = airy ? word_at(t, word(t, 1) + get(t, 11)) : 0;    // format 20: Cal_BoostHyst a word
    uint32_t level = on_map ? cal(t, 10) : airy ? (limit > hyst ? limit - hyst : 0)
                                                : (limit - cal(t, 11)) & 0xFF; // Cal_BoostRelease; or CPL; INC; ADD A,R3
    if (level < b)                                                      // CJNE A,B; JNC Ign_Boost_Below
        s.over = false;                                                 // above it: the release's time again
    else if (!s.over)
    {
        s.over = true;
        s.boost = cal(t, 12);                                           // Cal_BoostReleaseTime
    }
    else if (s.boost == 0)
        s.cut = s.over = false;                                         // Ign_Boost_Release
}

// The resume after an overrun cut (Ign_Overrun_Step): the advance a step on, the pulses ramped
// (InjCount from 0), the cut over; the target below the advance a step at once, else the resume's slew
// (its firings less this one) with the target taken as it is. True if the slew took the target so
// (Ign_Slewed: past Ign_Target).
static bool UNIT_CODE(resume)(const Task &t, AdvanceState &s, uint32_t step, uint32_t target)
{
    s.advance = (s.advance + step) & 0xFF;                              // ADD A,Advance
    s.count = 0;
    s.ramp = true;
    s.fuel_cut = false;
    if ((target - s.advance) & 0x80)                                    // SUBB A,Advance; JNB ACC.7
    {
        s.slew = 0;
        return false;
    }
    s.slew = (cal(t, 16) - 1) & 0xFF;                                   // Cal_SlewResume's firings; DEC A
    s.resume = true;
    return true;
}

// Ign_Overrun: the idle switch closed above the cut-off speed (Overrun_CutSpeed by coolant, and
// Cal_OverrunHyst, its second after a resume by the speed): the advance to its target first
// (SlewOverrun), then the cut; the speed back at the cut-off or the throttle open: the resume. True if
// the slew takes the target as it is (Ign_Slewed).
static bool UNIT_CODE(overrun_cut)(const Task &t, AdvanceState &s, uint32_t target, bool idle)
{
    if (!idle)                                                          // Ign_Overrun_Open
    {
        s.by_rpm = s.overrun = false;
        return s.fuel_cut && resume(t, s, cal(t, 14), target);          // Cal_ResumeStepThrottle
    }
    uint32_t rpm = speed_of(t, input(t, 0)), cutoff = value(t, get(t, 6)); // Overrun_CutSpeed
    if (cutoff >= rpm)                                                  // SUBB A,Rpm; JNC Ign_Overrun_Below
    {
        s.overrun = false;
        if (!s.fuel_cut)
            return false;
        s.by_rpm = true;
        return resume(t, s, cal(t, 15), target);                        // Cal_ResumeStepRpm
    }
    if (wide(t))                                                        // format 18: Cal_OverrunHyst's two
    {                                                                   //   words in rpm: within it
        if (rpm - cutoff <= speed_at(t, word(t, 1) + get(t, 13) + (s.by_rpm ? 2 : 0)))
            return false;
    }
    else if (at(t.rom, word(t, 1) + get(t, 13) + (s.by_rpm ? 1 : 0)) + ((cutoff - rpm) & 0xFF) > 0xFF)
        return false;                                                   // Cal_OverrunHyst (+1); ADD A,R3; JC
    s.overrun = true;                                                   // the advance to its target first
    if (target != s.advance)                                            // CJNE A,Advance,Ign_Taken
        return false;
    s.fuel_cut = true;                                                  // ... then the cut
    s.overrun = false;
    s.advance = target;
    return true;
}

// Ign_Target: the slew's flag by load and speed (above Cal_SlewLoad, or at Cal_SlewRpm and below), a
// step due when it changes but toward or after a cut; not in step the advance at the target at once;
// after a cut the resume's slew over once the target is below the advance.
static void UNIT_CODE(slew_target)(const Task &t, AdvanceState &s, uint32_t target, bool not_synced)
{
    bool c = (fine_air(t) ? word_at(t, word(t, 1) + get(t, 20)) < air(t) : cal(t, 20) < input(t, 1)) // Cal_SlewLoad
             || speed_at(t, word(t, 1) + get(t, 21)) >= speed_of(t, input(t, 0)); // Cal_SlewRpm; SUBB A,Rpm
    if (s.high)                                                         // JNB SlewHighLoad; CPL C
        c = !c;
    if (c)                                                              // JNC Ign_Target_Sync
    {
        s.high = !s.high;
        if (!s.resume && !s.overrun)
            s.slew = 0;
    }
    if (not_synced)                                                     // JNB NotSynced,Ign_Target_Resume
    {
        s.advance = target;
        s.slew = 0;
    }
    if (s.resume && ((target - s.advance) & 0x80))                      // SUBB A,Advance; JNB ACC.7
    {
        s.resume = false;
        s.slew = 0;
    }
}

// Ign_Slew: a step due (SlewCount 0), the pair the flags pick (Cal_SlewResume, Cal_SlewOverrun,
// Cal_SlewHighLoad, else Cal_SlewNormal: its firings to the next step, its half-teeth), the advance a
// step toward the target, or the target if nearer.
static void UNIT_CODE(slew_step)(const Task &t, AdvanceState &s, uint32_t target)
{
    if (s.slew != 0)                                                    // JNZ Ign_Done
        return;
    uint32_t place = word(t, 1) + get(t, s.resume ? 16 : s.overrun ? 17 : s.high ? 18 : 19);
    s.slew = at(t.rom, place);                                          // its firings
    uint32_t step = at(t.rom, place + 1), diff = (target - s.advance) & 0xFF;   // its half-teeth; SUBB A,R7
    if (diff & 0x80)                                                    // below it
        s.advance = ((0x100 - diff) & 0xFF) < step ? target : (s.advance - step) & 0xFF; // CPL; INC; SUBB A,R4
    else
        s.advance = diff < step ? target : (s.advance + step) & 0xFF;   // SUBB A,R4; ADD A,R4
}

// Format 19: Ign_Offset to Ign_Limits in 1/64 tooth: the sum with the offset by speed, a positive sum
// at most Cal_AdvanceMax, a negative one at least Cal_AdvanceMin (the word after it); no halving.
static int32_t UNIT_CODE(target64)(const Task &t, int32_t handover)
{
    int32_t sum = handover + spark_map(t, get(t, 0));                   // Ign_SpeedOffset
    uint32_t c = word(t, 1) + get(t, 3);
    int32_t most = word16(t, c), least = word16(t, c + 2);
    return held16(sum < 0 ? (sum >= least ? sum : least) : (most >= sum ? sum : most));
}

// Format 19: the resume (as resume), its step a word in 1/64 tooth.
static bool UNIT_CODE(resume64)(const Task &t, AdvanceState &s, int32_t step, int32_t target)
{
    s.advance = held16(s.advance + step);
    s.count = 0;
    s.ramp = true;
    s.fuel_cut = false;
    if (target < s.advance)
    {
        s.slew = 0;
        return false;
    }
    s.slew = (cal(t, 16) - 1) & 0xFF;                                   // Cal_SlewResume's firings
    s.resume = true;
    return true;
}

// Format 19: Ign_Overrun (as overrun_cut), the resume's steps words in 1/64 tooth.
static bool UNIT_CODE(overrun64)(const Task &t, AdvanceState &s, int32_t target, bool idle)
{
    uint32_t c = word(t, 1);
    if (!idle)
    {
        s.by_rpm = s.overrun = false;
        return s.fuel_cut && resume64(t, s, word16(t, c + get(t, 14)), target);   // Cal_ResumeStepThrottle
    }
    uint32_t rpm = speed_of(t, input(t, 0)), cutoff = value(t, get(t, 6));
    if (cutoff >= rpm)
    {
        s.overrun = false;
        if (!s.fuel_cut)
            return false;
        s.by_rpm = true;
        return resume64(t, s, word16(t, c + get(t, 15)), target);       // Cal_ResumeStepRpm
    }
    if (rpm - cutoff <= speed_at(t, c + get(t, 13) + (s.by_rpm ? 2 : 0)))
        return false;
    s.overrun = true;
    if (target != s.advance)
        return false;
    s.fuel_cut = true;
    s.overrun = false;
    s.advance = target;
    return true;
}

// Format 19: Ign_Target's resume end and Ign_Slew (as slew_target, slew_step): a pair's firings a byte,
// its step a word in 1/64 tooth.
static void UNIT_CODE(slew64)(const Task &t, AdvanceState &s, int32_t target, bool not_synced, bool slewed)
{
    if (!slewed)
    {
        bool c = (fine_air(t) ? word_at(t, word(t, 1) + get(t, 20)) < air(t) : cal(t, 20) < input(t, 1)) // format 20
                 || speed_at(t, word(t, 1) + get(t, 21)) >= speed_of(t, input(t, 0));
        if (s.high)
            c = !c;
        if (c)
        {
            s.high = !s.high;
            if (!s.resume && !s.overrun)
                s.slew = 0;
        }
        if (not_synced)
        {
            s.advance = target;
            s.slew = 0;
        }
        if (s.resume && target < s.advance)
        {
            s.resume = false;
            s.slew = 0;
        }
    }
    if (s.slew != 0)
        return;
    uint32_t place = word(t, 1) + get(t, s.resume ? 16 : s.overrun ? 17 : s.high ? 18 : 19);
    s.slew = at(t.rom, place);                                          // its firings
    int32_t step = at(t.rom, place + 1) << 8 | at(t.rom, place + 2), diff = target - s.advance;
    if (diff < 0)
        s.advance = -diff < step ? target : s.advance - step;
    else
        s.advance = diff < step ? target : s.advance + step;
}

static void UNIT_CODE(advance_rest64)(const Task &t)
{
    AdvanceState s;
    s.advance = own16(t, MAP_UNIT_ADVANCE);
    s.slew = byte_of(t, 35);
    s.boost = byte_of(t, 36);
    s.count = byte_of(t, 37);
    s.over = bit(t, get(t, 26));
    s.cut = bit(t, get(t, 27));
    s.fuel_cut = bit(t, get(t, 25));
    s.ramp = bit(t, get(t, 28));
    s.by_rpm = bit(t, get(t, 29));
    s.high = bit(t, get(t, 30));
    s.overrun = bit(t, get(t, 31));
    s.resume = bit(t, get(t, 32));
    bool starting = bit(t, get(t, 22)), idle = bit(t, get(t, 23));
    int32_t target = target64(t, own16(t, MAP_UNIT_HANDOVER));
    boost_guard(t, s, starting, idle);
    bool slewed = !starting && overrun64(t, s, target, idle);
    slew64(t, s, target, bit(t, get(t, 24)), slewed);
    uint32_t flags = (s.over ? ADV_BOOST_OVER : 0) | (s.cut ? ADV_BOOST_CUT : 0) | (s.fuel_cut ? ADV_FUEL_CUT : 0)
                     | (s.ramp ? ADV_RESUME_RAMP : 0) | (s.by_rpm ? ADV_RESUME_BY_RPM : 0)
                     | (s.high ? ADV_SLEW_HIGH_LOAD : 0) | (s.overrun ? ADV_SLEW_OVERRUN : 0)
                     | (s.resume ? ADV_SLEW_RESUME : 0);
    set_bit(t, get(t, 26), s.over);
    set_bit(t, get(t, 27), s.cut);
    set_bit(t, get(t, 25), s.fuel_cut);
    set_bit(t, get(t, 28), s.ramp);
    set_bit(t, get(t, 29), s.by_rpm);
    set_bit(t, get(t, 30), s.high);
    set_bit(t, get(t, 31), s.overrun);
    set_bit(t, get(t, 32), s.resume);
    set_own16(t, MAP_UNIT_HANDOVER, target);                            // the target, as rom_lean's left it
    set_own16(t, MAP_UNIT_ADVANCE, s.advance);
    own(t, 35, s.slew);
    own(t, 36, s.boost);
    own(t, 37, s.count);
    put(t, 0, half_teeth(s.advance));                                   // Advance: the nearest half-teeth
    put(t, 1, flags);
}

static void UNIT_CODE(advance_rest)(const Task &t)
{
    if (fine_spark(t))
    {
        advance_rest64(t);
        return;
    }
    AdvanceState s;                                                     // (field by field: no initializer in flash)
    s.advance = byte_of(t, 34);
    s.slew = byte_of(t, 35);
    s.boost = byte_of(t, 36);
    s.count = byte_of(t, 37);
    s.over = bit(t, get(t, 26));
    s.cut = bit(t, get(t, 27));
    s.fuel_cut = bit(t, get(t, 25));
    s.ramp = bit(t, get(t, 28));
    s.by_rpm = bit(t, get(t, 29));
    s.high = bit(t, get(t, 30));
    s.overrun = bit(t, get(t, 31));
    s.resume = bit(t, get(t, 32));
    bool starting = bit(t, get(t, 22)), idle = bit(t, get(t, 23));
    uint32_t target = target_of(t, byte_of(t, 33), get(t, 0), word(t, 1), get(t, 3)); // Ign_Offset: R5, Handover
    boost_guard(t, s, starting, idle);
    bool slewed = !starting && overrun_cut(t, s, target, idle);         // JB StartMode,Ign_Taken
    if (!slewed)
        slew_target(t, s, target, bit(t, get(t, 24)));
    slew_step(t, s, target);
    uint32_t flags = (s.over ? ADV_BOOST_OVER : 0) | (s.cut ? ADV_BOOST_CUT : 0) | (s.fuel_cut ? ADV_FUEL_CUT : 0)
                     | (s.ramp ? ADV_RESUME_RAMP : 0) | (s.by_rpm ? ADV_RESUME_BY_RPM : 0)
                     | (s.high ? ADV_SLEW_HIGH_LOAD : 0) | (s.overrun ? ADV_SLEW_OVERRUN : 0)
                     | (s.resume ? ADV_SLEW_RESUME : 0);
    set_bit(t, get(t, 26), s.over);                                     // kept for pair.cpp's turn too
    set_bit(t, get(t, 27), s.cut);
    set_bit(t, get(t, 25), s.fuel_cut);
    set_bit(t, get(t, 28), s.ramp);
    set_bit(t, get(t, 29), s.by_rpm);
    set_bit(t, get(t, 30), s.high);
    set_bit(t, get(t, 31), s.overrun);
    set_bit(t, get(t, 32), s.resume);
    own(t, 33, target);                                                 // Handover, as rom_lean's left it
    own(t, 35, s.slew);
    own(t, 36, s.boost);
    own(t, 37, s.count);
    put(t, 0, s.advance);                                               // Advance (the unit's too)
    put(t, 1, flags);
}

// ---------------------------------------------------------------------------------------------------
// Task 12 (format 8): the tick's start mode, rom_lean's Main_Tick from Cal_StartRpmEnter on, with
// Start_Variants: at Cal_StartRpmEnter or under, start mode and the cold-start variants by coolant
// (at or under Cal_ColdFuel, Cal_ColdIgn); in it, till over Start_ExitSpeed (by coolant) or the
// valve's ramp done (IsvRamp 0): then over, and the next reference injects (pair.cpp). Then the
// overboost guard's periods: BoostCount (the unit's own) one less each Cal_BoostPeriod ticks, to 0, by
// the tick counter as Main_Tick moves it on after (Tick_Due). Inputs: Rpm, IsvRamp, the byte of
// StartMode's bit (23h), TickCount, the snoop's Coolant. Data: Constants' address; the places there
// of Cal_StartRpmEnter, Cal_ColdFuel and Cal_ColdIgn; Start_ExitSpeed's id; StartMode's bit;
// Cal_BoostPeriod's place; BoostCount's address. Out: StartFlags (below), which the 8051 applies;
// StartMode kept here too. Format 9: the speed byte and StartMode's byte not sent (the unit's own:
// the inputs IsvRamp, TickCount and Coolant), the speed byte's address, ColdStartFuel's and
// ColdStartIgn's bits after the data (the variants kept here too, which the pipeline's advance reads),
// and the guard's period handed to the pipeline's level, which keeps BoostCount (pair_guard_period).
// Format 10: the snoop's Afm a fourth input and AfmFilt's address after the data: at start mode's end
// the AFM's filter the AFM here (the unit's own); then the tick's work (task 15) queued for the
// pipeline's level (pair_tick_work). Format 11: in the tick's job (L2, on its own copy) the job's
// commit gives start mode's end to the phase (pair_start_over); through the window, the task at once.

namespace
{

enum : uint32_t
{
    START_MODE = 1,     // StartMode
    START_COLD_FUEL = 2, // ColdStartFuel ...
    START_COLD_IGN = 4, // ... and ColdStartIgn, made (START_VARIANTS)
    START_VARIANTS = 8, // starting: the variants made
    START_OVER = 0x10,  // start mode over: Inj720Odd cleared, the AFM's filter the AFM (the 8051's; format
                        //   10, the unit's)
};

} // namespace

static void UNIT_CODE(start_mode)(const Task &t)
{
    bool own_bytes = kept(t);
    uint32_t c = word(t, 0), rpm = own_bytes ? byte_of(t, 9) : input(t, 0), flags = 0;
    uint32_t ramp = input(t, own_bytes ? 0 : 1), ticks = input(t, own_bytes ? 1 : 3);
    uint32_t coolant = input(t, own_bytes ? 2 : 4);
    bool starting = bit(t, get(t, 6));
    uint32_t speed = speed_of(t, rpm);
    if (speed_at(t, c + get(t, 2)) >= speed)                            // Cal_StartRpmEnter; SUBB A,Rpm; JNC
    {
        starting = true;
        flags |= START_VARIANTS;
        if (at(t.rom, c + get(t, 3)) >= coolant)                        // Cal_ColdFuel; CJNE A,Coolant; CPL C
            flags |= START_COLD_FUEL;
        if (at(t.rom, c + get(t, 4)) >= coolant)                        // Cal_ColdIgn
            flags |= START_COLD_IGN;
        if (own_bytes)                                                  // Start_Variants, here too
        {
            set_bit(t, get(t, 10), flags & START_COLD_FUEL);
            set_bit(t, get(t, 11), flags & START_COLD_IGN);
        }
    }
    else if (starting && (ramp == 0 || value(t, get(t, 5)) < speed))   // IsvRamp 0; Start_ExitSpeed; JNC
    {
        starting = false;
        flags |= START_OVER;
        if (!pair_on_tick_copy(t.ram))                                  // (the tick's job, L2: at its commit,
            pair_start_over();                                          //   with its StartMode given back)
        if (at(t.rom, 0x2004) >= TICK_FORMAT)
            own(t, 12, input(t, 3));                                    // MOV AfmFilt,Afm (the unit's own)
    }
    if (starting)
        flags |= START_MODE;
    set_bit(t, get(t, 6), starting);
    uint32_t tick = (ticks + 1) & 0xFF;                                 // MOV A,TickCount; INC A
    if (tick == 180)                                                    // CJNE A,#180; CLR A
        tick = 0;
    uint32_t period = at(t.rom, c + get(t, 7)), left = 0;               // Cal_BoostPeriod; DIV AB: B
    if (period != 0)                                                    //   (by 0: B kept, 0)
        for (left = tick; left >= period;)                              // (no division in the bank)
            left -= period;
    bool tick_job = at(t.rom, 0x2004) >= LOG_FORMAT;                   // format 11: in the tick's job, on its
    if (left == 0 && own_bytes && !tick_job)                            //   copy (at power-up the count is 0)
        pair_guard_period();                                            // the pipeline's level keeps it
    else if (left == 0 && byte_of(t, 8) != 0)                           // JNZ; MOV A,BoostCount; JZ
        own(t, 8, byte_of(t, 8) - 1);                                   // DEC BoostCount
    put(t, 0, flags);
    if (at(t.rom, 0x2004) == TICK_FORMAT)
        pair_tick_work();                                               // the tick's work next (task 15)
}

// ---------------------------------------------------------------------------------------------------
// Task 13 (format 9): the tick's values the 8051 makes, sent once a tick after lambda (rom_lean's
// Main_Byte_Again's place) and at power-up: the load's byte, LoadHi, LoadMid, WarmWeight, AfterStart,
// AccelEnrich, LamInt, IdleTarget, the throttle switches (Throttle_Switches_Table's value: Switches) and
// AdvanceDyn, each into the unit's copy as it is sent. Here the switches into their bits (RRC A twice:
// the full-load switch bit 0, the idle switch bit 1), the unit's StartMode and FuelCut in that byte
// left as they are; then pair.cpp's pipeline makes the waiting pulse again. Data: IdleSwitch's bit,
// FullLoadSwitch's. No outputs. Format 10: LamInt and IdleTarget alone, the rest the unit's own (task
// 15), no data.
static void UNIT_CODE(tick_values)(const Task &t)
{
    if (at(t.rom, 0x2004) < TICK_FORMAT)
    {
        uint32_t switches = input(t, 8);
        set_bit(t, get(t, 1), switches & 1);                            // MOV FullLoadSwitch,C
        set_bit(t, get(t, 0), switches >> 1 & 1);                       // MOV IdleSwitch,C
    }
    pair_tick_sent();
}

// ---------------------------------------------------------------------------------------------------
// Task 14 (format 9): the pipeline's stage 2, Speed_Count and Idle_Spark, which pair.cpp's pipeline
// runs on its copy of the RAM (the 8051 never asks for it). Data: SPEED_K's three bytes (low first),
// SPEED_BAND; Constants' address and the places there of Cal_IdleSparkGain, Cal_IdleSparkTop,
// Cal_IdleSparkMax and Cal_IdleSparkMin; the bits IdleSwitch, FullLoadSwitch, StartMode and FuelCut;
// the addresses of Rpm, FirePeriodL, FirePeriodH, IdlePrev, IdleTarget, Advance and AdvanceDyn. No
// inputs or outputs.

static __force_inline uint32_t speed_k(const Task &t)
{
    return get(t, 0) | get(t, 1) << 8 | get(t, 2) << 16;
}

// Speed_Count: the speed byte (the teeth a tick) from the firing's length P: D = SPEED_K + P / 2 less
// the byte x P, a count on (D less P) while D is P or more, a count back (D on by P) while it is under
// 0, four steps at most, else the count by long division a bit at a time (8 bits); then the byte held
// within SPEED_BAND - 1 of the count. P FFFFh (a firing past 16 bits, its count made apart): none.
static void UNIT_CODE(speed_count)(const Task &t)
{
    uint32_t pl = byte_of(t, 15), ph = byte_of(t, 16), p = ph << 8 | pl;
    if ((pl & ph) == 0xFF)                                              // ANL A,FirePeriodH; CPL A; JNZ
        return;
    uint32_t dividend = (speed_k(t) + (p >> 1)) & 0xFFFFFF;             // R7:R6:R5
    uint32_t rpm = byte_of(t, 14), r3 = rpm, product = r3 * p;          // R3; R2:R1:R0
    bool down = dividend < product;                                     // SUBB x 3: C
    uint32_t d = (dividend - product) & 0xFFFFFF, count = 0;
    bool found = false;
    for (uint32_t steps = 4; steps != 0 && !found; steps--)             // MOV R4,#4; DJNZ R4
    {
        if (down)                                                       // Speed_Down
        {
            r3 = (r3 - 1) & 0xFF;                                       // DEC R3
            d += p;                                                     // ADD, ADDC, ADDC
            found = d > 0xFFFFFF;                                       // JC Speed_Found
            d &= 0xFFFFFF;
        }
        else                                                            // Speed_Up
        {
            found = d < p;                                              // SUBB x 3; JC Speed_Found
            d = (d - p) & 0xFFFFFF;
            if (!found)
                r3 = (r3 + 1) & 0xFF;                                   // INC R3
        }
    }
    if (found)
        count = r3;                                                     // Speed_Found: MOV R5,R3
    else                                                                // Speed_Divide
    {
        uint32_t rest = dividend >> 8, r5 = dividend & 0xFF;            // R7:R6, R5
        for (uint32_t bit = 0; bit < 8; bit++)                          // MOV R4,#8; DJNZ R4,Speed_Bit
        {
            rest = rest << 1 | r5 >> 7;                                 // ADD A,R5; RLC R6; RLC R7
            r5 = r5 << 1 & 0xFF;
            bool top = rest > 0xFFFF;                                   // SpeedTop
            rest &= 0xFFFF;
            if (top || rest >= p)                                       // JB SpeedTop; SUBB; JC Speed_Next
            {
                rest = (rest - p) & 0xFFFF;                             // Speed_Fits: R7:R6 less P
                r5 |= 1;                                                // INC R5
            }
        }
        count = r5;
    }
    uint32_t band = get(t, 3), a = (count + 1 - band) & 0xFF;           // ADD A,#LOW(1 - SPEED_BAND)
    if (a >= rpm)                                                       // CJNE A,Rpm; JC Speed_High
        rpm = a;                                                        // MOV Rpm,A
    a = (a + 2 * band - 2) & 0xFF;                                      // ADD A,#(2 * SPEED_BAND - 2)
    if (a < rpm)                                                        // CJNE A,Rpm; JNC Speed_Out
        rpm = a;
    own(t, 14, rpm);
}

// Idle_Error: the idle speed's fine error, (the target x P - SPEED_K) / 1024: the top two bytes of the
// 24-bit difference (the low byte's borrow kept), two right with their sign, within -128 and 127,
// else the range's end by the sign.
static __force_inline uint32_t error_of(uint32_t difference)
{
    difference &= 0xFFFFFF;                                             // MUL AB x 2; SUBB x 3
    int32_t v = (int32_t)(int16_t)(difference >> 8 & 0xFFFF) >> 2;     // MOV C,ACC.7; RRC A, twice
    if (v < -128 || v > 127)                                            // RLC A; CLR A; ADDC A,R2; JZ
        return v < 0 ? 0x80 : 0x7F;                                     // MOV A,#7Fh; ADDC A,#00h
    return (uint32_t)v & 0xFF;                                          // MOV A,R1
}

static uint32_t UNIT_CODE(fine_error)(uint32_t target, uint32_t p, uint32_t k)
{
    return error_of(target * p - k);
}

// Format 18: Idle_Error with the target in rpm: the difference (the target x P - 40 x SPEED_K) / 40,
// rounded half up, taken as the byte target's (the target x P - SPEED_K) is; at a target of whole 40
// rpm the byte's error, and finer between.
static uint32_t UNIT_CODE(fine_error_rpm)(uint32_t target, uint32_t p, uint32_t k)
{
    uint32_t n = target * p + 20, m = 40 * k;
    return error_of(n >= m ? map_unit_divide32(n - m, 40) : 0 - map_unit_divide32(m - n + 39, 40));
}

uint32_t UNIT_CODE(unit_fine_error_rpm)(uint32_t target, uint32_t p, uint32_t k)
{
    return fine_error_rpm(target, p, k);
}

static uint32_t UNIT_CODE(idle_error)(const Task &t, uint32_t p)
{
    if (wide(t))                                                        // format 18: the idle task's target
        return fine_error_rpm(t.own[MAP_UNIT_IDLE_TARGET - MAP_UNIT_SPEED], p, speed_k(t)); // in rpm
    return fine_error(byte_of(t, 18), p, speed_k(t));                   // IdleTarget
}

// Idle_Spark: the length / 16, its low byte, against the last firing's (IdlePrev): its growth less
// one, x 2, at most 7Fh (a rise none). Off idle nothing more. At idle (the idle switch closed): with
// the full-load switch closed, starting or in an overrun cut, AdvanceDyn none; else the error and the
// fall x Cal_IdleSparkGain / 16, at least one from half of one (the owner's floor), a lead below the
// target within Cal_IdleSparkMax and the advance with it at most Cal_IdleSparkTop, a lag above it
// within Cal_IdleSparkMin (stored negative), into AdvanceDyn.
static void UNIT_CODE(idle_spark)(const Task &t)
{
    uint32_t pl = byte_of(t, 15), ph = byte_of(t, 16);
    uint32_t sixteenth = (ph << 4 | pl >> 4) & 0xFF;                    // SWAP, ANL, ORL: R2
    uint32_t fall = (sixteenth - byte_of(t, 17)) & 0xFF;                // XCH A,IdlePrev; SUBB A,R2
    own(t, 17, sixteenth);
    if (fall & 0x80)
        fall = 0;                                                       // a rise: none
    else if (fall != 0)
    {
        fall = (fall - 1) * 2 & 0xFF;                                   // DEC A; ADD A,ACC
        if (fall & 0x80)
            fall = 0x7F;
    }
    if (!bit(t, get(t, 10)))                                            // JNB IdleSwitch,Idle_Spark_Out
        return;
    if (bit(t, get(t, 11)) || bit(t, get(t, 12)) || bit(t, get(t, 13))) // the full-load switch, starting, cut
    {
        own(t, 20, 0);
        if (fine_spark(t))
            set_own16(t, MAP_UNIT_ADVANCE_DYN, 0);
        return;
    }
    int32_t sum = (int32_t)(int8_t)idle_error(t, ph << 8 | pl) + (int32_t)fall;  // ADD A,R3; JNB OV
    uint32_t a = sum > 127 ? 0x7F : (uint32_t)sum & 0xFF;
    bool lag = a & 0x80;                                                // SignTmp
    if (lag)
        a = (0x100 - a) & 0xFF;                                         // CPL A; INC A
    uint32_t c = word(t, 4), product = at(t.rom, c + get(t, 6)) * a;    // x Cal_IdleSparkGain: B:A
    if (fine_spark(t))                                                  // format 19: the term in 1/64 tooth,
    {                                                                   //   the product / 16 half-teeth
        int32_t size = (int32_t)product * 2, limit;                     //   exactly (no floor), its limits
        if (!lag)                                                       //   words
        {
            int32_t room = word16(t, c + get(t, 7)) - own16(t, MAP_UNIT_ADVANCE);  // Cal_IdleSparkTop
            int32_t most = word16(t, c + get(t, 8));                    // Cal_IdleSparkMax
            limit = room < 0 ? 0 : most < room ? most : room;
        }
        else
            limit = -word16(t, c + get(t, 9));                          // Cal_IdleSparkMin (negative)
        int32_t v = limit < size ? limit : size;
        set_own16(t, MAP_UNIT_ADVANCE_DYN, lag ? -v : v);
        own(t, 20, half_teeth(lag ? -v : v));
        return;
    }
    uint32_t low = product & 0xFF, high = product >> 8, size;
    if (high >= 0x10)
        size = 0xFF;                                                    // Idle_Spark_Full
    else
    {
        size = (high << 4 | low >> 4) & 0xFF;                           // / 16
        if (size == 0)
            size = low >= 8 ? 1 : 0;                                    // ADD A,#0F8h; CLR A; RLC A
    }
    uint32_t limit;
    if (!lag)
    {
        uint32_t room = (at(t.rom, c + get(t, 7)) - byte_of(t, 19)) & 0xFF;  // Cal_IdleSparkTop less Advance
        if (room & 0x80)
            room = 0;
        uint32_t most = at(t.rom, c + get(t, 8));                       // Cal_IdleSparkMax
        limit = most < room ? most : room;
    }
    else
        limit = (0x100 - at(t.rom, c + get(t, 9))) & 0xFF;              // Cal_IdleSparkMin; CPL A; INC A
    uint32_t v = limit < size ? limit : size;                           // CJNE A,00h; JC
    own(t, 20, lag ? (0x100 - v) & 0xFF : v);
}

static void UNIT_CODE(speed)(const Task &t)
{
    speed_count(t);
    idle_spark(t);
}

// ---------------------------------------------------------------------------------------------------
// Task 15 (format 10): the tick's work, rom_lean's in its order, which pair.cpp's tick job runs on its
// copy of the RAM after task 12 (the 8051 never asks for it): the throttle switches (Main_Tick's), the
// after-start enrichment and the acceleration enrichment's decay (Main_Tick_After, Main_Tick_Accel),
// the dynamic spark's filter (Spark_Dyn), the AFM's rise (Accel_Tick), the load filter's coefficient
// (Main_Tick_Filter's LoadK: here always, rom_lean's waited while its last load did), the load
// (Main_Load: Load_Tick, its running path task 7), the tip-in retard and the enrichment's test
// (Accel_Raise, Accel_Enrich), the load's byte and the warm-up's weight (Main_Byte: Load_Byte,
// Load_Weight). The tick counter is task 12's input, one on (as Main_Tick moved it after start mode).
// Data: below (TW_). No inputs or outputs.

namespace
{

// Task 15's data, by place: Constants' address and Throttle_Switches_Table's (words); the places in
// Constants of Cal_Prescalers, Cal_AfterStartHot, Cal_SparkDynRpm, Cal_RpmFilterGain,
// Cal_SparkDynLoad, Cal_SparkAccel, Cal_SparkDecel, Cal_StartLoad, Cal_LoadMax and Cal_LoadMin;
// RISE_MIN; FIRINGS; the ids of AfterStart_Coolant, AfterStart_AirTemp, Load_Filter, Ign_TipIn,
// Accel_PulseRise, Accel_Coolant, Accel_SpeedLoad, Fuel_WarmUpWeight and Fuel_WarmUpWeightIdle; task
// 7's description's address (a word); the bits StartMode, IdleSwitch, FullLoadSwitch and NotSynced;
// the addresses of TickCount, AfterStart, AccelEnrich, Coolant, Rpm, RpmFilt, AdvanceDyn, AfmFilt, Afm,
// AccelPart, LoadK, Load, LoadHi, LoadMid, WarmUp and WarmWeight.
enum : uint32_t
{
    TW_CONSTANTS = 0,
    TW_SWITCHES = 2,
    TW_PRESCALERS = 4,
    TW_AFTER_HOT,
    TW_DYN_RPM,
    TW_FILTER_GAIN,
    TW_DYN_LOAD,
    TW_SPARK_ACCEL,
    TW_SPARK_DECEL,
    TW_START_LOAD,
    TW_LOAD_MAX,
    TW_LOAD_MIN,
    TW_RISE_MIN,
    TW_FIRINGS,
    TW_AFTER_COOLANT,
    TW_AFTER_AIR,
    TW_LOAD_FILTER,
    TW_TIP_IN,
    TW_PULSE_RISE,
    TW_ACCEL_COOLANT,
    TW_ACCEL_SPEED_LOAD,
    TW_WEIGHT,
    TW_WEIGHT_IDLE,
    TW_LOAD_TASK,       // a word
    TW_START = TW_LOAD_TASK + 2,
    TW_IDLE,
    TW_FULL,
    TW_NOT_SYNCED,
    TW_TICK_COUNT,
    TW_AFTER,
    TW_ACCEL,
    TW_COOLANT,
    TW_RPM,
    TW_RPM_FILT,
    TW_DYN,
    TW_AFM_FILT,
    TW_AFM,
    TW_ACCEL_PART,
    TW_LOAD_K,
    TW_LOAD,
    TW_LOAD_HI,
    TW_LOAD_MID,
    TW_WARM_UP,
    TW_WARM_WEIGHT,
    TW_START_AIR,       // format 20: Cal_StartAir's place in Constants
};

// A task as its description places it: the input count and the inputs, the output count and the
// outputs, then the data.
// (A description past FC00h, which no image has, is read from there: each of its three parts is at most
// 256 bytes on, so that none reaches past the 64 KiB.)
__force_inline Task task_of(const uint8_t *rom, uint8_t *ram, uint32_t description, uint32_t *own = nullptr)
{
    description &= 0xFFFF;
    if (description > 0xFC00)
        description = 0xFC00;
    uint32_t outputs = description + 1 + rom[description];
    uint32_t data = outputs + 1 + rom[outputs];
    return Task{rom, ram, rom + description + 1, rom + outputs + 1, rom + data, own != nullptr ? own : map_unit_owns_on(ram)};
}

// Tick_Due: the tick counter's remainder by the period at Constants + place (DIV AB's B; by 0, B
// kept: 0), by subtraction (the bank calls no library).
__force_inline uint32_t tick_due(const Task &t, uint32_t place, uint32_t tick)
{
    uint32_t period = at(t.rom, word(t, TW_CONSTANTS) + (place & 0xFF)), left = 0;
    if (period != 0)
        for (left = tick; left >= period;)
            left -= period;
    return left;
}

// Afm_Rise: the AFM's rise against its filter before this tick's step: d = Afm less AfmFilt, the step
// d x 85 / 256 rounded (Afm_Third), signed (R2); for a positive d the rise d and the step, at most FFh,
// else 0 (R3).
__force_inline void afm_rise(const Task &t, uint32_t *rise, uint32_t *step)
{
    uint32_t afm = byte_of(t, TW_AFM), filt = byte_of(t, TW_AFM_FILT);
    if (afm < filt)                                                     // SUBB A,AfmFilt; JNC Afm_Rise_Up
    {
        uint32_t third = ((filt - afm) * 0x55 + 0x80) >> 8;             // CPL A; INC A; Afm_Third
        *step = (0x100 - third) & 0xFF;                                 // CPL A; INC A
        *rise = 0;                                                      // MOV R3,#00h
        return;
    }
    uint32_t d = afm - filt, third = (d * 0x55 + 0x80) >> 8;            // MUL AB; ADD A,#80h; ADDC A,B
    *step = third;
    *rise = d + third > 0xFF ? 0xFF : d + third;                        // ADD A,R3; JNC; MOV A,#0FFh
}

} // namespace

// Spark_Dyn_Adj: under Cal_SparkDynLoad and Cal_SparkDynRpm, the speed over its filter Cal_SparkAccel,
// under it Cal_SparkDecel, at it none; over the load or the speed none.
// Format 19: Cal_SparkAccel and Cal_SparkDecel words in 1/64 tooth (signed). Format 20: Cal_SparkDynLoad a
// word against load_air (Accel_Raise's the air before the load task, as rom_lean's byte is there).
static int32_t UNIT_CODE(spark_dyn_adj)(const Task &t, uint32_t load_air)
{
    uint32_t c = word(t, TW_CONSTANTS), rpm = byte_of(t, TW_RPM), filt = byte_of(t, TW_RPM_FILT);
    if (fine_air(t) ? word_at(t, c + get(t, TW_DYN_LOAD)) < load_air    // format 20: the air
                    : at(t.rom, c + get(t, TW_DYN_LOAD)) < byte_of(t, TW_LOAD)) // CJNE A,Load; JC Spark_Dyn_Zero
        return 0;
    if (at(t.rom, c + get(t, TW_DYN_RPM)) < rpm)                        // CJNE A,Rpm; JC Spark_Dyn_Zero
        return 0;
    if (filt == rpm)                                                    // CJNE A,Rpm,Spark_Dyn_Moved
        return 0;
    uint32_t place = c + get(t, filt < rpm ? TW_SPARK_ACCEL : TW_SPARK_DECEL);
    return fine_spark(t) ? word16(t, place) : (int32_t)at(t.rom, place);
}

// AdvanceDyn: a byte, or (format 19) the own value in 1/64 tooth with the byte its nearest half-teeth.
static __force_inline void set_dyn(const Task &t, int32_t v)
{
    if (fine_spark(t))
    {
        set_own16(t, MAP_UNIT_ADVANCE_DYN, v);
        own(t, TW_DYN, half_teeth(v));
    }
    else
        own(t, TW_DYN, (uint32_t)v);
}

// Spark_Dyn: under Cal_SparkDynRpm and in step, the speed filter a count toward the speed byte on the
// ticks where the tick counter x the step, mod 128, is under the step (|the speed less the filter| x
// Cal_RpmFilterGain, at most 127); over it or out of step the filter the speed.
static void UNIT_CODE(spark_dyn)(const Task &t, uint32_t tick)
{
    uint32_t c = word(t, TW_CONSTANTS), rpm = byte_of(t, TW_RPM), filt = byte_of(t, TW_RPM_FILT);
    if (at(t.rom, c + get(t, TW_DYN_RPM)) < rpm || bit(t, get(t, TW_NOT_SYNCED)))
    {
        own(t, TW_RPM_FILT, rpm);                                       // Spark_Dyn_Follow
        return;
    }
    if (rpm == filt)                                                    // JZ Spark_Dyn_Out
        return;
    bool down = rpm < filt;                                             // MOV DynDown,C
    uint32_t d = down ? filt - rpm : rpm - filt;                        // CPL A; INC A
    uint32_t product = d * at(t.rom, c + get(t, TW_FILTER_GAIN));       // MUL AB
    uint32_t step = (product & 0x80) || product > 0xFF ? 0x7F : product; // JB ACC.7; XCH A,B; JZ
    if ((tick * step & 0x7F) >= step)                                   // MUL AB; ANL A,#7Fh; SUBB A,R1; JNC
        return;
    own(t, TW_RPM_FILT, down ? filt - 1 : filt + 1);                    // DEC A, or INC A
}

// Accel_Tick: at idle the filter's step alone; off idle, starting or with a rise under RISE_MIN,
// AdvanceDyn the dynamic spark's adjustment and the step; else the enrichment's test next (true:
// AccelRaise; the step its).
static bool UNIT_CODE(accel_tick)(const Task &t)
{
    uint32_t rise, step;
    afm_rise(t, &rise, &step);
    if (!bit(t, get(t, TW_IDLE)))                                       // JB IdleSwitch,Accel_Tick_Step
    {
        if (!bit(t, get(t, TW_START)) && rise >= get(t, TW_RISE_MIN))   // JB StartMode; CJNE A,#RISE_MIN; JC
            return true;                                                // SETB AccelRaise
        set_dyn(t, spark_dyn_adj(t, air(t)));                           // Accel_Tick_Dyn
    }
    own(t, TW_AFM_FILT, byte_of(t, TW_AFM_FILT) + step);                // MOV A,AfmFilt; ADD A,R2
    return false;
}

// Format 21: an enrichment (map_unit.h's input, an own word) set, its byte (data place) its nearest count.
__force_inline void set_enrichment(const Task &t, uint32_t input, uint32_t place, uint32_t w)
{
    t.own[(input - MAP_UNIT_SPEED) & (MAP_UNIT_OWNS - 1)] = w & 0xFFFF;
    own(t, place, nearest_count(w));
}

// Format 21: an enrichment less its share of a count this tick, to 0: a count (256) each period of the
// tick counter at Constants + Cal_Prescalers' place (the counter's remainder r by the period: (r + 1) x
// 256 / it less r x 256 / it); a period of 0, a count each tick. The fades keep today's rates.
static void UNIT_CODE(fade)(const Task &t, uint32_t input, uint32_t place, uint32_t prescaler, uint32_t tick)
{
    uint32_t w = own_word(t, input);
    if (w == 0)
        return;
    uint32_t period = at(t.rom, word(t, TW_CONSTANTS) + (prescaler & 0xFF));
    uint32_t share = fade_share(period, tick_due(t, prescaler, tick));
    set_enrichment(t, input, place, w > share ? w - share : 0);
}

// Accel_Raise: the tip-in retard, AdvanceDyn the dynamic spark's adjustment less Ign_TipIn by the rise
// (63 at most); the enrichment's first part by the rise (Accel_PulseRise: AccelPart, *part); the filter's
// step. Both maps take the rise from R3 (03h), as Map_Lookup leaves it. True: the next test
// (EnrichDue). Format 21: the part a word, AccelPart its nearest byte.
static bool UNIT_CODE(accel_raise)(const Task &t, uint32_t load_air, uint32_t *part_out)
{
    uint32_t rise, step;
    afm_rise(t, &rise, &step);
    t.ram[3] = rise;                                                    // R3
    uint32_t tip = value(t, get(t, TW_TIP_IN));
    if (fine_spark(t))                                                  // format 19: a word in 1/64 tooth,
        set_dyn(t, spark_dyn_adj(t, load_air) - (int32_t)(tip >= 0x800 ? 0x7E0 : tip)); // 63 half-teeth at most
    else
    {
        if (tip >= 0x40)                                                // CJNE A,#40h; JC; MOV A,#3Fh
            tip = 0x3F;
        own(t, TW_DYN, spark_dyn_adj(t, load_air) + ((0x100 - tip) & 0xFF)); // CPL A; INC A; ADD A,R1
    }
    uint32_t part = value(t, get(t, TW_PULSE_RISE));
    *part_out = part;
    own(t, TW_ACCEL_PART, fine_fuel(t) ? nearest_count(part) : part);
    own(t, TW_AFM_FILT, byte_of(t, TW_AFM_FILT) + step);                // Accel_Raise_Step
    return part != 0;                                                   // JZ; SETB EnrichDue
}

// Accel_Enrich: the first part x Accel_Coolant by coolant / 256 x Accel_SpeedLoad by speed and load /
// 256 (none: none), into AccelEnrich if over it. Format 21: words, each product / 65,536.
static void UNIT_CODE(accel_enrich)(const Task &t, uint32_t part)
{
    uint32_t by_coolant = value(t, get(t, TW_ACCEL_COOLANT));
    if (by_coolant == 0)                                                // JZ Accel_Enrich_Out
        return;
    if (fine_fuel(t))
    {
        uint32_t b = by_coolant * part >> 16;
        if (b == 0)
            return;
        uint32_t v = value(t, get(t, TW_ACCEL_SPEED_LOAD)) * b >> 16;
        if (v > own_word(t, MAP_UNIT_ACCEL))
            set_enrichment(t, MAP_UNIT_ACCEL, TW_ACCEL, v);
        return;
    }
    uint32_t b = by_coolant * part >> 8;                                // MUL AB; MOV A,B
    if (b == 0)
        return;
    uint32_t v = value(t, get(t, TW_ACCEL_SPEED_LOAD)) * b >> 8;        // Map_Lookup_2D (B kept); MUL AB
    if (v > byte_of(t, TW_ACCEL))                                       // CJNE A,AccelEnrich; JC
        own(t, TW_ACCEL, v);
}

// Load_Tick: starting, rom_dual's fixed load (Cal_StartLoad, the filter's load it x FIRINGS x 25, the
// first product's low byte); else task 7's running path, as its description says.
static void UNIT_CODE(load_tick)(const Task &t)
{
    if (!bit(t, get(t, TW_START)))                                      // JNB StartMode,Load_Tick_Run
    {
        load(task_of(t.rom, t.ram, word(t, TW_LOAD_TASK), t.own));
        return;
    }
    uint32_t start = at(t.rom, word(t, TW_CONSTANTS) + get(t, TW_START_LOAD));
    own(t, TW_LOAD, start);
    if (fine_air(t))                                                    // format 20: Cal_StartAir, the air
        t.own[MAP_UNIT_LOAD - MAP_UNIT_SPEED] = word_at(t, word(t, TW_CONSTANTS) + get(t, TW_START_AIR));
    uint32_t v = (start * get(t, TW_FIRINGS) & 0xFF) * 0x19;            // MUL AB; MOV B,#19h; MUL AB
    own(t, TW_LOAD_HI, v >> 8);
    own(t, TW_LOAD_MID, v);
}

// Load_Byte: not starting, LoadHi:LoadMid clamped to Cal_LoadMax and Cal_LoadMin (x 25 each), x 164 /
// 256 (Mul16x8), its nibbles as SWAP takes them (/ 16 while the high byte is under 16).
static void UNIT_CODE(load_byte)(const Task &t)
{
    if (bit(t, get(t, TW_START)))                                       // JB StartMode,Load_Byte_Out
        return;
    uint32_t c = word(t, TW_CONSTANTS), r = byte_of(t, TW_LOAD_HI) << 8 | byte_of(t, TW_LOAD_MID);
    uint32_t most = at(t.rom, c + get(t, TW_LOAD_MAX)) * 0x19;
    uint32_t least = at(t.rom, c + get(t, TW_LOAD_MIN)) * 0x19;
    if (most < r)                                                       // Clamp_Upper: C, the most less r
        r = most;
    if (least >= r)                                                     // Clamp_Lower: CPL C
        r = least;
    r = r * 0xA4 >> 8;                                                  // MOV A,#0A4h; Mul16x8
    uint32_t r7 = r >> 8, r6 = r & 0xFF;
    own(t, TW_LOAD, ((r7 << 4 | r7 >> 4) & 0xFF) | r6 >> 4);            // SWAP A; XCH A,R6; SWAP A; ANL; ORL
}

// Load_Weight: the warm-up x its weight / 256 (off idle by speed and load, at idle by speed; the
// warm-up 0: none). Format 21: words, the product / 65,536, the own value 89h.
static void UNIT_CODE(load_weight)(const Task &t)
{
    if (fine_fuel(t))
    {
        uint32_t warm = own_word(t, MAP_UNIT_WARM_UP), weight = 0;
        if (warm != 0)
            weight = value(t, get(t, bit(t, get(t, TW_IDLE)) ? TW_WEIGHT_IDLE : TW_WEIGHT)) * warm >> 16;
        set_enrichment(t, MAP_UNIT_WARM_WEIGHT, TW_WARM_WEIGHT, weight);
        return;
    }
    uint32_t warm = byte_of(t, TW_WARM_UP), weight = 0;
    if (warm != 0)                                                      // JZ Load_Weight_Out
        weight = value(t, get(t, bit(t, get(t, TW_IDLE)) ? TW_WEIGHT_IDLE : TW_WEIGHT)) * warm >> 8;
    own(t, TW_WARM_WEIGHT, weight);                                     // MUL AB; MOV A,B
}

// Main_Tick's throttle switches (format 11: the tick's job's first part, TICK_SWITCHES).
static void UNIT_CODE(tick_switches)(const Task &t)
{
    uint32_t switches = step_table(t.rom, t.ram, word(t, TW_SWITCHES)); // Step_Table_Lookup
    set_bit(t, get(t, TW_FULL), switches & 1);                          // RRC A; MOV FullLoadSwitch,C
    set_bit(t, get(t, TW_IDLE), switches >> 1 & 1);                     // RRC A; MOV IdleSwitch,C
}

static void UNIT_CODE(tick_work)(const Task &t)
{
    if (at(t.rom, 0x2004) < LOG_FORMAT)                                 // (format 11: made first)
        tick_switches(t);
    uint32_t tick = (byte_of(t, TW_TICK_COUNT) + 1) & 0xFF;             // the tick counter as moved on
    if (tick == 180)
        tick = 0;
    bool words = fine_fuel(t);                                          // (format 21: the enrichments' words)
    if (!bit(t, get(t, TW_START)))                                      // JB StartMode,Main_Tick_Start
    {
        uint32_t after = byte_of(t, TW_AFTER);                          // one less each Cal_Prescalers'
        if (words)                                                      //   first ticks, to 0
            fade(t, MAP_UNIT_AFTER_START, TW_AFTER, get(t, TW_PRESCALERS), tick);
        else if (tick_due(t, get(t, TW_PRESCALERS), tick) == 0 && after != 0)
            own(t, TW_AFTER, after - 1);
    }
    else                                                                // starting: by coolant, or over
    {                                                                   //   Cal_AfterStartHot by air
        uint32_t hot = at(t.rom, word(t, TW_CONSTANTS) + get(t, TW_AFTER_HOT));
        uint32_t id = get(t, hot < byte_of(t, TW_COOLANT) ? TW_AFTER_AIR : TW_AFTER_COOLANT);
        if (words)
            set_enrichment(t, MAP_UNIT_AFTER_START, TW_AFTER, value(t, id));
        else
            own(t, TW_AFTER, value(t, id));                             // CJNE A,Coolant; JNC; Map_Lookup
    }
    uint32_t accel = byte_of(t, TW_ACCEL);                              // one less each Cal_Prescalers'
    if (words)                                                          //   second ticks, to 0
        fade(t, MAP_UNIT_ACCEL, TW_ACCEL, get(t, TW_PRESCALERS) + 1, tick);
    else if (tick_due(t, get(t, TW_PRESCALERS) + 1, tick) == 0 && accel != 0)
        own(t, TW_ACCEL, accel - 1);
    spark_dyn(t, tick);
    bool raise = accel_tick(t);
    own(t, TW_LOAD_K, value(t, get(t, TW_LOAD_FILTER)));                // Main_Tick_Filter
    uint32_t before = air(t), part = 0;                                 // (format 20: Accel_Raise's load the
    load_tick(t);                                                       //   air before Main_Load, as its byte)
    if (raise && accel_raise(t, before, &part))                         // Accel_Raise, then Accel_Enrich
        accel_enrich(t, part);
    load_byte(t);                                                       // Main_Byte
    load_weight(t);
}

// ---------------------------------------------------------------------------------------------------
// Task 16 (format 11): lambda, which pair.cpp's tick runs in three parts in rom_lean's order (the 8051
// never asks for it): Tick_Stop's open loop at the tick's start, Main_Tick's sensor timer after the
// tick counter, Main_Lambda after the tick's work. The sensor's comparators come with the 8051's marks
// (their byte: bit 0 O2_NOT_RICH, P1.6; bit 1 O2_LEAN, P1.7). Data: below (LM_). No inputs or outputs.

namespace
{

// Task 16's data, by place: Constants' address (a word); the places there of Cal_O2Ready,
// Cal_O2NotReady, Cal_LamRpmMax, Cal_LamRpmClose, Cal_LamCoolantCold, Cal_LamCoolant (two bytes: off
// idle, at idle), Cal_LamCoolantHyst, Cal_LamJumpRich, Cal_LamIntMax, Cal_LamLoadMax and
// Cal_LamMapMax; Lambda_Ramp's and Lambda_Jump's ids; the bits StartMode, NotSynced, FuelCut,
// BoostCut, FullLoadSwitch, IdleSwitch, ColdStartIgn, O2MidBand, O2Active, ClosedLoop, O2WasRich and
// O2Ready; the addresses of the marks' byte, TickCount, Rpm, Coolant, Load, In5 and LamInt.
enum : uint32_t
{
    LM_CONSTANTS = 0,
    LM_O2_READY = 2,
    LM_O2_NOT_READY,
    LM_RPM_MAX,
    LM_RPM_CLOSE,
    LM_COOLANT_COLD,
    LM_COOLANT,
    LM_COOLANT_HYST,
    LM_JUMP_RICH,
    LM_INT_MAX,
    LM_LOAD_MAX,
    LM_MAP_MAX,
    LM_RAMP,
    LM_JUMP,
    LM_START,
    LM_NOT_SYNCED,
    LM_FUEL_CUT,
    LM_BOOST_CUT,
    LM_FULL,
    LM_IDLE,
    LM_COLD_IGN,
    LM_MID_BAND,
    LM_ACTIVE,
    LM_CLOSED,
    LM_WAS_RICH,
    LM_READY,
    LM_PINS,
    LM_TICK_COUNT,
    LM_RPM,
    LM_COOLANT_BYTE,
    LM_LOAD,
    LM_IN5,
    LM_LAM_INT,
};

// The marks' bits (pair.cpp: the 8051's pins as its Bits routine read them).
constexpr uint32_t PIN_NOT_RICH = 1, PIN_LEAN = 2, PIN_AC = 4;

__force_inline uint32_t constant(const Task &t, uint32_t place)
{
    return at(t.rom, word(t, 0) + get(t, place));
}

} // namespace

// Lambda_Load_Open: the full-load switch closed, the load over Cal_LamLoadMax or the MAP over
// Cal_LamMapMax.
static bool UNIT_CODE(lambda_load_open)(const Task &t)
{
    if (bit(t, get(t, LM_FULL)))                                        // MOV C,FullLoadSwitch; JC
        return true;
    if (fine_air(t) ? word_at(t, word(t, 0) + get(t, LM_LOAD_MAX)) < air(t)  // format 20: the air
                    : constant(t, LM_LOAD_MAX) < byte_of(t, LM_LOAD))   // CJNE A,Load: C; JC
        return true;
    return constant(t, LM_MAP_MAX) < byte_of(t, LM_IN5);                // CJNE A,In5: C
}

// Tick_Stop's: so, the loop open at once and the integrator x 1.0.
static void UNIT_CODE(lambda_open)(const Task &t)
{
    if (!lambda_load_open(t))                                           // ACALL Lambda_Load_Open; JNC
        return;
    set_bit(t, get(t, LM_CLOSED), false);                               // CLR ClosedLoop
    own(t, LM_LAM_INT, 0x80);                                           // MOV LamInt,#80h
}

// Main_Tick's timer: the sensor's period (Cal_O2NotReady in its middle band, else Cal_O2Ready) at its
// end on the tick counter (Tick_Due; by 0, B kept: due): ready if out of the band, from the period's
// second end since the state began (O2Active its first).
static void UNIT_CODE(lambda_timer)(const Task &t)
{
    bool mid = bit(t, get(t, LM_MID_BAND));
    uint32_t period = constant(t, mid ? LM_O2_NOT_READY : LM_O2_READY), left = 0;
    if (period != 0)                                                    // MOV A,TickCount; DIV AB; MOV A,B
        for (left = byte_of(t, LM_TICK_COUNT); left >= period;)
            left -= period;
    if (left != 0)                                                      // JNZ Main_Tick_Out
        return;
    if (bit(t, get(t, LM_ACTIVE)))                                      // JNB O2Active,Main_Tick_Seen
        set_bit(t, get(t, LM_READY), !mid);                             // MOV C,O2MidBand; CPL C; MOV O2Ready,C
    set_bit(t, get(t, LM_ACTIVE), true);                                // SETB O2Active
}

// Main_Lambda: the sensor rich (O2_LEAN low), a switch since the last run clearing O2Active; the loop
// closed but starting, not in step, in a cut, by Lambda_Load_Open, over the speed (Cal_LamRpmMax,
// closed again at Cal_LamRpmClose) or not over the coolant's threshold (Cal_LamCoolantCold after a cold
// start, else Cal_LamCoolant's, at idle its second; Cal_LamCoolantHyst more to close); the sensor's
// band; closed with the sensor ready, the integrator a step toward its other side (at a switch the
// jump, Lambda_Jump, toward rich less Cal_LamJumpRich; else the ramp, Lambda_Ramp: a step each 64 and
// one more on the rest's share of the ticks), within 0 and Cal_LamIntMax; open, x 1.0.
static void UNIT_CODE(lambda_run)(const Task &t)
{
    uint32_t pins = byte_of(t, LM_PINS);
    bool lean = pins & PIN_LEAN, rich = !lean, was_rich = bit(t, get(t, LM_WAS_RICH)); // MOV C,O2_LEAN; CPL C
    if (rich != was_rich)                                               // a switch: CLR O2Active
        set_bit(t, get(t, LM_ACTIVE), false);
    bool closed = bit(t, get(t, LM_CLOSED));
    bool open = bit(t, get(t, LM_START)) || bit(t, get(t, LM_NOT_SYNCED)) || bit(t, get(t, LM_FUEL_CUT))
                || bit(t, get(t, LM_BOOST_CUT)) || lambda_load_open(t);
    if (!open)                                                          // CJNE A,Rpm: C; JC Main_Lambda_Open
        open = speed_at(t, word(t, 0) + get(t, closed ? LM_RPM_MAX : LM_RPM_CLOSE)) < speed_of(t, byte_of(t, LM_RPM));
    if (open)
        closed = false;                                                 // CLR ClosedLoop
    else
    {
        uint32_t threshold = bit(t, get(t, LM_COLD_IGN)) ? constant(t, LM_COOLANT_COLD)
                             : at(t.rom, word(t, 0) + get(t, LM_COOLANT) + (bit(t, get(t, LM_IDLE)) ? 1 : 0));
        if (!closed)
            threshold = (threshold + constant(t, LM_COOLANT_HYST)) & 0xFF; // ADD A,R2
        closed = threshold < byte_of(t, LM_COOLANT_BYTE);               // CLR C; SUBB A,Coolant; MOV ClosedLoop,C
    }
    bool mid = bit(t, get(t, LM_MID_BAND));                             // the band: not rich and not lean
    if ((pins & PIN_NOT_RICH) && !lean ? !mid : mid)                    // JNB O2_NOT_RICH; JB O2_LEAN; JB O2MidBand
    {
        set_bit(t, get(t, LM_MID_BAND), !mid);                          // SETB, or CLR, O2MidBand
        set_bit(t, get(t, LM_ACTIVE), false);                           // Main_Lambda_Begun
    }
    closed = closed && bit(t, get(t, LM_READY));                        // ANL C,O2Ready
    set_bit(t, get(t, LM_CLOSED), closed);
    if (!closed)
        own(t, LM_LAM_INT, 0x80);                                       // Main_Lambda_One
    else
    {
        uint32_t step;
        if (rich != was_rich)                                           // Main_Lambda_Jump
        {
            uint32_t a = value(t, get(t, LM_JUMP));
            if (rich)                                                   // XCH A,B; CLR C; SUBB A,B
                a = (a - constant(t, LM_JUMP_RICH)) & 0xFF;
            step = ((a + 2) & 0xFF) >> 2;                               // ADD A,#02h; RR A; RR A; ANL A,#3Fh
        }
        else                                                            // the ramp: R2 the value, R3 its rest
        {
            uint32_t a = value(t, get(t, LM_RAMP)), rest = a & 0x3F;
            uint32_t share = byte_of(t, LM_TICK_COUNT) * rest & 0x3F;   // MUL AB; ANL A,#3Fh; SUBB A,R3: C
            step = (a >> 6) + (share < rest ? 1 : 0);                   // RL A; RL A; ANL A,#03h; ADDC A,#00h
        }
        uint32_t lam = byte_of(t, LM_LAM_INT);
        if (rich)                                                       // Main_Lambda_Lean: SUBB; JNC; CLR A
            lam = lam >= step ? lam - step : 0;
        else                                                            // ADD A,LamInt; JC; CJNE: at most the max
        {
            uint32_t most = constant(t, LM_INT_MAX);
            lam += step;
            if (lam > 0xFF || most < lam)
                lam = most;
        }
        own(t, LM_LAM_INT, lam);
    }
    set_bit(t, get(t, LM_WAS_RICH), rich);                              // MOV C,SignTmp; MOV O2WasRich,C
}

// ---------------------------------------------------------------------------------------------------
// Task 17 (format 11): the idle valve's control, rom_lean's R4.2b, which pair.cpp's tick runs every
// second tick (the 8051 never asks for it): Main_Idle, then, in step, Idle_Duty with Idle_Ramp. The
// valve's edges stay the 8051's: the high phase goes to it in the take, which moves an open valve's
// close by the change (Idle_Duty's end). The A/C request comes with the 8051's marks (bit 2: P3.5, low
// requested). Data: below (ID_). No inputs or outputs.

namespace
{

// Task 17's data, by place: Constants' address (a word); SPEED_K (three bytes, low first); ISV_PERIOD
// (a word); Idle_Ramp's two literals (rom_lean's Cal_IsvRampRpm and Cal_IsvRampStep); the places in
// Constants of Cal_IdleTargetAc, Cal_IdleOpen, Cal_IdleClose, Cal_IsvCorrMax, Cal_IsvCorrMin, Cal_IsvAc,
// Cal_IsvBasicSetting, Cal_IsvDutyGain and Cal_IsvDutyOffset; the ids of Idle_Target_Map, Idle_Crank,
// Isv_Base_Map and Isv_Overrun; the bits NotSynced, StartMode, FuelCut, IdleSwitch, FullLoadSwitch and
// IdleReturn; the addresses of the marks' byte, FirePeriodL, FirePeriodH, IdleTarget, AdvanceDyn,
// IdleIntLo, IdleIntHi, IsvRamp, Rpm, IsvHighL and IsvHighH; format 18, Idle_Ramp's speed in rpm (a word).
enum : uint32_t
{
    ID_CONSTANTS = 0,
    ID_SPEED_K = 2,
    ID_PERIOD = 5,
    ID_RAMP_RPM = 7,
    ID_RAMP_STEP,
    ID_TARGET_AC,
    ID_OPEN,
    ID_CLOSE,
    ID_CORR_MAX,
    ID_CORR_MIN,
    ID_ISV_AC,
    ID_BASIC,
    ID_DUTY_GAIN,
    ID_DUTY_OFFSET,
    ID_TARGET_MAP,
    ID_CRANK,
    ID_BASE,
    ID_OVERRUN,
    ID_NOT_SYNCED,
    ID_START,
    ID_FUEL_CUT,
    ID_IDLE,
    ID_FULL,
    ID_RETURN,
    ID_PINS,
    ID_FIRE_L,
    ID_FIRE_H,
    ID_TARGET,
    ID_DYN,
    ID_INT_LO,
    ID_INT_HI,
    ID_RAMP,
    ID_RPM,
    ID_HIGH_L,
    ID_HIGH_H,
    ID_RAMP_RPM_WIDE,   // format 18: Idle_Ramp's speed in rpm (a word)
};

// Add_Saturate: the sum, at most FFh.
__force_inline uint32_t add_saturate(uint32_t a, uint32_t b)
{
    return a + b > 0xFF ? 0xFF : a + b;
}

// Integrate: r + or - the gain x a (its high byte at most 7Fh, the low byte kept), held within 0 and
// FFFFh.
__force_inline uint32_t integrate(uint32_t r, uint32_t gain, uint32_t a, bool down)
{
    uint32_t product = gain * a, high = product >> 8;                   // MUL AB
    if (high & 0x80)                                                    // JNB B.7; MOV B,#7Fh
        high = 0x7F;
    uint32_t step = high << 8 | (product & 0xFF);
    if (down)                                                           // CPL C; CPL A; XRL B,#0FFh; ADDC
        return r >= step ? r - step : 0;
    return r + step > 0xFFFF ? 0xFFFF : r + step;                       // ADDC; CPL C; JC: else FFFFh
}

} // namespace

// Main_Idle: not in step, the integrator neutral and no duty; else the duty due, the target by coolant
// (with the A/C request at least Cal_IdleTargetAc), and at idle (not starting, no cut, the idle switch
// alone closed) the integrator on the spark's term (a lead opens at Cal_IdleOpen, a lag closes as much,
// on the way back to idle at Cal_IdleClose till the speed is at the target), within Cal_IsvCorrMin and
// Cal_IsvCorrMax; the throttle open or a cut, on the way back (IdleReturn).
static bool UNIT_CODE(idle_state)(const Task &t)
{
    if (bit(t, get(t, ID_NOT_SYNCED)))                                  // Idle_Reset: the duty kept
    {
        own(t, ID_INT_HI, 0x80);
        own(t, ID_INT_LO, 0);
        return false;
    }
    uint32_t target = value(t, get(t, ID_TARGET_MAP));                  // SETB IdleDuty; Idle_Target_Map
    if (!(byte_of(t, ID_PINS) & PIN_AC))                                // JB AC_REQUEST: requested, low
    {
        uint32_t ac = speed_at(t, word(t, 0) + get(t, ID_TARGET_AC));   // CJNE A,B; JNC: the larger
        if (ac >= target)
            target = ac;
    }
    if (wide(t))                                                        // format 18: the target in rpm the
    {                                                                   //   P28's own, IdleTarget the byte
        t.own[MAP_UNIT_IDLE_TARGET - MAP_UNIT_SPEED] = target & 0xFFFF; //   nearest it (the log's), at most
        uint32_t nearest = map_unit_divide32(target + 20, 40);          //   FFh
        own(t, ID_TARGET, nearest > 0xFF ? 0xFF : nearest);
    }
    else
        own(t, ID_TARGET, target);
    if (bit(t, get(t, ID_START)))                                       // starting: held
        return true;
    if (bit(t, get(t, ID_FUEL_CUT)) || !bit(t, get(t, ID_IDLE)))        // Idle_Held
    {
        set_bit(t, get(t, ID_RETURN), true);
        return true;
    }
    if (bit(t, get(t, ID_FULL)))                                        // the jumper
        return true;
    bool back = bit(t, get(t, ID_RETURN));
    if (back)                                                           // ACALL Idle_Error; JB ACC.7
    {
        uint32_t p = byte_of(t, ID_FIRE_H) << 8 | byte_of(t, ID_FIRE_L);
        uint32_t k = get(t, ID_SPEED_K) | get(t, ID_SPEED_K + 1) << 8 | get(t, ID_SPEED_K + 2) << 16;
        if (!((wide(t) ? fine_error_rpm(target, p, k) : fine_error(target, p, k)) & 0x80))
        {
            back = false;
            set_bit(t, get(t, ID_RETURN), false);                       // CLR IdleReturn
        }
    }
    uint32_t term = byte_of(t, ID_DYN), gain = ID_OPEN;                 // Idle_Back: AdvanceDyn
    bool lag = term & 0x80;
    if (lag)
    {
        if (back)
            gain = ID_CLOSE;
        term = (0x100 - term) & 0xFF;                                   // CPL A; INC A
    }
    uint32_t r = integrate(byte_of(t, ID_INT_HI) << 8 | byte_of(t, ID_INT_LO), constant(t, gain), term, lag);
    uint32_t r1 = r >> 8;
    if (r1 & 0x80)                                                      // within the limits: R1 alone
    {
        uint32_t most = (0x80 + (constant(t, ID_CORR_MAX) >> 1)) & 0xFF; // CLR C; RRC A; ADD A,#80h
        if (most < r1)                                                  // CJNE A,01h: C; JNC
            r1 = most;
    }
    else
    {
        uint32_t least = (0x80 + (~(constant(t, ID_CORR_MIN) >> 1) & 0xFF)) & 0xFF; // RRC A; CPL A; ADD A,#80h
        if (least >= r1)                                                // CJNE A,01h: C; JC
            r1 = least;
    }
    own(t, ID_INT_LO, r & 0xFF);
    own(t, ID_INT_HI, r1);
    return true;
}

// Idle_Ramp: under Cal_IsvRampRpm the command the cranking position by coolant (Idle_Crank) and the
// weight full; past it the weight Cal_IsvRampStep less (to 0), the loop's command moved toward the
// position by their difference x the weight / 256, rounded (none at 0: the loop's alone). R1:R0 = r.
static uint32_t UNIT_CODE(idle_ramp)(const Task &t, uint32_t r)
{
    uint32_t a;
    bool under = wide(t) ? speed_of(t, 0) < word(t, ID_RAMP_RPM_WIDE)   // format 18: in rpm
                         : byte_of(t, ID_RPM) < get(t, ID_RAMP_RPM);
    if (under)                                                          // CJNE A,#10: C; JNC Idle_Ramp_Step
    {
        own(t, ID_RAMP, 0xFF);
        a = value(t, get(t, ID_CRANK));
    }
    else
    {
        uint32_t weight = byte_of(t, ID_RAMP), step = get(t, ID_RAMP_STEP);
        if (weight == 0)                                                // JZ Idle_Ramp_Out
            return r;
        weight = weight >= step ? weight - step : 0;                    // SUBB A,#6; JNC; CLR A
        own(t, ID_RAMP, weight);
        uint32_t command = r >> 7 & 0xFF;                               // RLC A twice: / 128 (B)
        uint32_t position = value(t, get(t, ID_CRANK));
        bool down = position < command;                                 // SUBB A,B: RampNeg
        uint32_t size = down ? command - position : position - command; // CPL A; INC A
        uint32_t moved = (size * weight + 0x80) >> 8;                   // MUL AB; ADD A,#80h; ADDC A,B
        a = (command + (down ? 0x100 - moved : moved)) & 0xFF;          // CPL A; INC A; ADD A,R2
    }
    return a << 7;                                                      // CLR C; RRC A twice: x 128
}

// Idle_Duty: the base (Isv_Base_Map by speed and coolant; in a cut Isv_Overrun's dashpot by speed, with
// the A/C request Cal_IsvAc, each Add_Saturate) x 128 and the integrator's correction (its neutral
// 8000h), within 0 and 7FFFh, through Idle_Ramp; or, both switches closed, not starting and no cut,
// Cal_IsvBasicSetting alone (the jumper). Its duty: the command x 2 x Cal_IsvDutyGain / 256 (Mul16x8),
// Cal_IsvDutyOffset on its high byte; its high phase that x ISV_PERIOD / 65536 (Mul16x16).
static void UNIT_CODE(idle_duty)(const Task &t)
{
    uint32_t base = value(t, get(t, ID_BASE));
    bool cut = bit(t, get(t, ID_FUEL_CUT));
    if (cut)
        base = add_saturate(value(t, get(t, ID_OVERRUN)), base);
    if (!(byte_of(t, ID_PINS) & PIN_AC))
        base = add_saturate(constant(t, ID_ISV_AC), base);
    uint32_t r;
    if (!bit(t, get(t, ID_START)) && !cut && bit(t, get(t, ID_FULL)) && bit(t, get(t, ID_IDLE)))
        r = constant(t, ID_BASIC) >> 1 << 8;                            // Idle_Basic: RRC A; MOV R0,#00h
    else
    {
        uint32_t integrator = byte_of(t, ID_INT_HI) << 8 | byte_of(t, ID_INT_LO);
        r = (base * 0x80 + integrator - 0x8000) & 0xFFFF;               // MUL AB; ADD A,R0; ADDC A,R1
        if (r & 0x8000)                                                 // JNB ACC.7; JNB IsvCorrNeg: 7FFFh,
            r = integrator >= 0x8000 ? 0x7FFF : 0;                      //   else 0
        r = idle_ramp(t, r);
    }
    uint32_t duty = (r << 1 & 0xFFFF) * constant(t, ID_DUTY_GAIN) >> 8 & 0xFFFF; // Idle_Set; Mul16x8
    duty = ((duty >> 8) + constant(t, ID_DUTY_OFFSET)) << 8 & 0xFF00 | (duty & 0xFF);  // ADD A,R7
    uint32_t high = duty * (get(t, ID_PERIOD) << 8 | get(t, ID_PERIOD + 1)) >> 16; // Mul16x16
    own(t, ID_HIGH_L, high & 0xFF);
    own(t, ID_HIGH_H, high >> 8 & 0xFF);
}

// Task 18 (format 12; DME-Sim's docs/Paired_Map_Pulse.md): the MAP pulse, rom_lean's Main_Map for one
// read and Accel_Pulse's arithmetic, which pair.cpp's MAP job runs as the snoop sees each MAP read
// (slots 3 and 9, each its own filter: the cylinders' before at the same crank angle). Its data:
// Cal_MapDeadband's address (high byte first); INJ_OFS; the bits IdleSwitch and StartMode; the bytes
// MapPrev0 and MapPrev1 (the filters, the unit's own), MapGainNow (task 2's) and DeadNow (the pulse's
// task's). Format 14: Cal_MapCarry's address, and the bytes MapCarry0 and MapCarry1 (the rise each
// slot carries, the unit's own: 0 from the 8051's start, as the RAM's copy). Format 15: the bytes
// DecelGainNow (task 2's) and DecelOwed (the pulse cut a fall owes, the unit's own), and
// Cal_DecelDeadband's address. Format 22: the gains, the dead time and the cut owed words (fine_map).
namespace
{

enum : uint32_t
{
    MP_DEADBAND, // a word
    MP_OFS = 2,
    MP_IDLE,
    MP_START,
    MP_PREV,     // two: the first MAP read's filter, the second's
    MP_GAIN = MP_PREV + 2,
    MP_DEAD,
    MP_CARRY_ON, // format 14: a word
    MP_CARRY = MP_CARRY_ON + 2, // two: the first MAP read's carry, the second's
    MP_DECEL_GAIN = MP_CARRY + 2, // format 15: the pulse cut's gain by coolant (task 2's)
    MP_DECEL_OWED,               // the cut owed, in 64-cycle units (format 22: a word, cycles)
    MP_DECEL_DEAD,               // a word: the falls' deadband
};

// Format 15 (DME-Sim's docs/Paired_Tip_Outs.md: the owner, 2026-11-09): a read's fall below its slot's
// filter past Cal_DecelDeadband, each read alone (falls are not carried: steady reads sit a count or
// two under their filters), x DecelGainNow (Decel_MapGain by coolant, task 2's) / 32, owed to the pulse
// task's cut (DecelOwed, at most 255). Format 22: in cycles, DecelGainNow 1/32 cycle a count (at most
// MAP_LONGEST).
__force_inline void owe(const Task &t, uint32_t fall)
{
    uint32_t deadband = at(t.rom, word(t, MP_DECEL_DEAD));
    if (fall <= deadband)
        return;
    if (fine_map(t))
    {
        uint32_t at_owed = get(t, MP_DECEL_OWED);
        uint32_t sum = ram_word(t, at_owed) + (((fall - deadband) * ram_word(t, get(t, MP_DECEL_GAIN)) + 0x10) >> 5);
        set_ram_word(t, at_owed, sum > MAP_LONGEST ? MAP_LONGEST : sum);
        return;
    }
    uint8_t &owed = t.ram[get(t, MP_DECEL_OWED) & 0x7F];
    uint32_t sum = owed + (((fall - deadband) * byte_of(t, MP_DECEL_GAIN) + 0x10) >> 5);
    owed = sum > 0xFF ? 0xFF : sum;
}

// Format 22: a pulse on lengthened by longer cycles, R7:R6: it less the cycles Map_On's stop of TH0 adds,
// its low byte none or 32 at least (1 to 31 to the nearer of them). Map_On takes R6 off TH0 with TH0's
// handler held off; with an overflow waiting there (TH0 a few counts past FFh), TH0 less a byte under 32
// would overflow again before the handler took the first, and that overflow be lost (256 cycles more).
// Format 21's units of 64 cycles never gave such a byte.
__force_inline uint32_t map_lengthen(uint32_t longer)
{
    uint32_t r = longer - MAP_STOP, low = r & 0xFF;
    if (low != 0 && low < 32)
        r = low < 16 ? r - low : r + 32 - low;
    return r;
}

// Format 14 (DME-Sim's docs/Paired_Transients.md: the owner, 2026-11-09): with Cal_MapCarry on, a rise
// under the deadband is carried to its slot's next read and added to its rise, so the deadband is paid
// once on the sum; a read with no rise, the idle switch or start mode ends the carry. Off, each read
// pays the deadband (format 12's). Format 15: a fall owes the pulse cut (owe). Format 22: the pulse in
// cycles, the rise past the deadband x MapGainNow (1/32 cycle a count), from 32 cycles on (where format
// 21's rounding made a unit) and at most MAP_LONGEST, the dead time a word; a pulse on lengthened by it
// as map_lengthen gives it.
__force_inline bool map_pulse(const Task &t, uint32_t slot, uint32_t read, uint32_t *load)
{
    uint8_t &filter = t.ram[get(t, MP_PREV + slot) & 0x7F];
    uint32_t rise = read >= filter ? read - filter : 0;                 // SUBB A,MapPrev0; JC: none falling
    uint32_t fall = read < filter ? filter - read : 0;
    filter = (filter + read + 1) >> 1;                                  // SETB C; ADDC A,R3; RRC A: the
                                                                        //   filter half the way, rounded
    uint8_t none = 0;
    uint32_t format = at(t.rom, 0x2004);
    bool carried = format >= CARRY_FORMAT && at(t.rom, word(t, MP_CARRY_ON)) != 0;
    uint8_t &carry = carried ? t.ram[get(t, MP_CARRY + slot) & 0x7F] : none;
    bool off = bit(t, get(t, MP_IDLE)) || bit(t, get(t, MP_START));    // JB IdleSwitch; JB StartMode
    if (off || rise == 0)
    {
        carry = 0;
        if (!off && format >= DECEL_FORMAT)
            owe(t, fall);
        return false;
    }
    uint32_t deadband = at(t.rom, word(t, MP_DEADBAND));                // MOVC A,@A+DPTR (Cal_MapDeadband)
    uint32_t total = rise + carry;
    if (total <= deadband)                                              // SUBB A,R2; JC; JZ
    {
        carry = total;
        return false;
    }
    carry = 0;
    uint32_t whole, lengthen;
    if (format >= CYCLES_FORMAT)                                        // format 22: in cycles
    {
        uint32_t longer = ((total - deadband) * ram_word(t, get(t, MP_GAIN)) + 0x10) >> 5;
        if (longer < 32)
            return false;
        if (longer > MAP_LONGEST)
            longer = MAP_LONGEST;
        whole = ram_word(t, get(t, MP_DEAD)) + longer;
        lengthen = map_lengthen(longer);
    }
    else
    {
        uint32_t units = ((total - deadband) * byte_of(t, MP_GAIN) + 0x10) >> 5;  // MUL AB; ADD A,#10h ...; / 32
        if (units > 0xFF)                                               // CJNE A,#10h; JNC: at most 255
            units = 0xFF;
        if (units == 0)                                                 // JZ Main_Map_Out
            return false;
        lengthen = units * 64;                                          // Accel_Pulse: R7:R6 = A x 64
        whole = byte_of(t, MP_DEAD) * 5 + lengthen;                     // R3:R2: the dead time x 5 and it
    }
    uint32_t ofs = get(t, MP_OFS), low = whole & 0xFF;
    uint32_t start = (ofs - low) & 0xFF;                                // MOV A,#INJ_OFS; CLR C; SUBB A,R2
    uint32_t overflows = ((whole >> 8) + (low > ofs ? 1 : 0)) & 0xFF;   // ADD A,#(255 - INJ_OFS); ADDC
    *load = start | overflows << 8 | lengthen << 16;
    return true;
}

} // namespace

uint32_t UNIT_CODE(unit_filter_air)(uint32_t flow, uint32_t p, uint32_t k, uint32_t own)
{
    uint32_t owns[MAP_UNIT_OWNS];                                       // (no initializer: no memset in flash)
    owns[MAP_UNIT_LOAD - MAP_UNIT_SPEED] = own;
    filter_air(Task{nullptr, nullptr, nullptr, nullptr, nullptr, owns}, flow, p, k);
    return owns[MAP_UNIT_LOAD - MAP_UNIT_SPEED];
}

// Format 21 (pair_test's, for DME-Sim's twins): K's mantissa (bits 0-15) and exponent (16-31, signed).
uint32_t UNIT_CODE(unit_fuel_k)(uint32_t stoich, uint32_t flow, uint32_t density, uint32_t rated, uint32_t rail)
{
    F16 k = fuel_k(stoich, flow, density, rated, rail);
    return (k.m & 0xFFFF) | (uint32_t)k.e << 16;
}

// Format 21: the time a turn, in[0] and in[1] the slow factor's mantissa and exponent, in[2] the base;
// running in[3] to in[6] the warm-up's weight, the after-start, the main map, the acceleration's;
// starting in[3] to in[5] Cal_StartFactor, the cranking's word, the acceleration's.
uint32_t UNIT_CODE(unit_fuel_time)(const uint32_t *in, bool starting)
{
    F16 s = f16_word(in[0]);
    s.e += (int32_t)in[1];
    return starting ? fuel_starting(s, in[2], in[3], in[4], in[5]) : running_time(s, in[2], in[3], in[4], in[5], in[6]);
}

// Format 21: a fade's share at the period and the tick counter's remainder by it.
uint32_t UNIT_CODE(unit_fade_share)(uint32_t period, uint32_t r)
{
    return fade_share(period, r);
}

// Format 22: the MAP pulse's gains' factor's mantissa (bits 0-15) and exponent (16-31, signed), from K's
// constants (in[0] to in[4], as unit_fuel_k's) and the sensor's Pa a count (in[5]).
uint32_t UNIT_CODE(unit_map_gain_k)(const uint32_t *in)
{
    F16 k = map_gain_k(fuel_k(in[0], in[1], in[2], in[3], in[4]), in[0], in[5]);
    return (k.m & 0xFFFF) | (uint32_t)k.e << 16;
}

bool UNIT_CODE(unit_map_pulse)(uint32_t description, const uint8_t *rom, uint8_t *ram, uint32_t slot, uint32_t read,
                               uint32_t *load)
{
    return map_pulse(task_of(rom, ram, description), slot & 1, read & 0xFF, load);
}

bool UNIT_CODE(unit_task_part)(uint32_t task, uint32_t part, uint32_t description, const uint8_t *rom, uint8_t *ram,
                               uint32_t *own)
{
    Task t = task_of(rom, ram, description, own);
    if (task == TASK_TICKWORK && part == TICK_SWITCHES)
        tick_switches(t);
    else if (task == TASK_LAMBDA && part == LAMBDA_OPEN)
        lambda_open(t);
    else if (task == TASK_LAMBDA && part == LAMBDA_TIMER)
        lambda_timer(t);
    else if (task == TASK_LAMBDA && part == LAMBDA_RUN)
        lambda_run(t);
    else if (task == TASK_IDLE && part == IDLE_STATE)
        return idle_state(t);
    else if (task == TASK_IDLE && part == IDLE_DUTY)
        idle_duty(t);
    return true;
}

bool UNIT_CODE(unit_task)(uint32_t task, uint32_t description, const uint8_t *rom, uint8_t *ram, uint32_t *own)
{
    Task t = task_of(rom, ram, description, own);
    if (task == TASK_WARMUP)
        warm_up(t);
    else if (task == TASK_FUEL)
        fuel(t);
    else if (task == TASK_IGN)
        ign(t);
    else if (task == TASK_CHARGE)
        charge(t);
    else if (task == TASK_ADVANCE)
        advance(t);
    else if (task == TASK_OFFSET)
        offset(t);
    else if (task == TASK_TIME)
        injection_time(t);
    else if (task == TASK_LOAD)
        load(t);
    else if (task == TASK_COIL)
        ; // its input kept in ram, where pair.cpp's plans read it
    else if (task == TASK_PULSE || task == TASK_AGAIN)
        pulse(t, task == TASK_AGAIN);
    else if (task == TASK_REST)
        advance_rest(t);
    else if (task == TASK_START)
        start_mode(t);
    else if (task == TASK_TICK)
        tick_values(t);
    else if (task == TASK_SPEED)
        speed(t);
    else if (task == TASK_TICKWORK)
        tick_work(t);
    else
        return false;
    return true;
}

#endif // FEATURE_MAP_UNIT
