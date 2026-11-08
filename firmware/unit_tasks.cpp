// The map unit's tasks (unit_tasks.h): rom_lean's slow work as its routines make it, each step
// noted with the 8051's instructions it stands for. DME-Sim's tools/coproc_maps/make_coproc.py
// writes each task's description, and DME-Sim checks each task against rom_lean's routine at every
// input (P28 tests there), running this code on its emulated Cortex-M0+.

#include "unit_tasks.h"

#if defined(FEATURE_MAP_UNIT)

#include "pair.h"
#include "unit_ram.h"

namespace
{

// A task at work: the image, the 8051's RAM bytes, where its description lists its inputs and its
// outputs, and its data.
struct Task
{
    const uint8_t *rom;
    uint8_t *ram;
    uint32_t inputs, outputs, data;
};

__force_inline uint32_t at(const uint8_t *rom, uint32_t address)
{
    return rom[address & 0xFFFF];
}

__force_inline uint32_t get(const Task &t, uint32_t i)
{
    return at(t.rom, t.data + i);
}

__force_inline uint32_t word(const Task &t, uint32_t i)
{
    return get(t, i) << 8 | get(t, i + 1);
}

// Output j (from 0) into the 8051's RAM byte the description names.
__force_inline void put(const Task &t, uint32_t j, uint32_t value)
{
    t.ram[at(t.rom, t.outputs + j) & 0x7F] = value & 0xFF;
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

// The 8051's RAM byte whose address is the task's data byte i, as sent.
__force_inline uint32_t byte_of(const Task &t, uint32_t i)
{
    return t.ram[get(t, i) & 0x7F];
}

// The task's input k (from 0, in its description's order), as sent.
__force_inline uint32_t input(const Task &t, uint32_t k)
{
    return t.ram[at(t.rom, t.inputs + k) & 0x7F];
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
    return unit_map_value(id, t.ram);
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

// Slow_WarmUp. Data: Fuel_WarmUp's id, Fuel_WarmUp_Cold's, ColdStartFuel's bit. Out: WarmUp.
static void UNIT_CODE(warm_up)(const Task &t)
{
    put(t, 0, value(t, get(t, bit(t, get(t, 2)))));    // the cold map after a cold start
}

// Slow_Fuel. Data: Fuel_Quality_Table's address, Constants', Cal_FuelTrim's and Cal_InjScale's places
// in Constants, Fuel_AirTemp's id. Out: FuelFactorL, FuelFactorH, FuelShift.
static void UNIT_CODE(fuel)(const Task &t)
{
    uint32_t constants = word(t, 2);
    uint32_t step = step_table(t.rom, t.ram, word(t, 0));
    uint32_t trim = at(t.rom, constants + ((step + get(t, 4)) & 0xFF)); // ADD A,#(Cal_FuelTrim - Constants); MOVC
    uint32_t r = trim * value(t, get(t, 6)), shift = 2;                 // Fuel_AirTemp; MUL AB: R7:R6; FuelShift
    normalise(&r, &shift, at(t.rom, constants + get(t, 5)));            // x Cal_InjScale (Mul_Normalise)
    put(t, 0, r);                                                       // R6
    put(t, 1, r >> 8);                                                  // R7
    put(t, 2, shift);                                                   // the exponent
}

// Main_Ign. Data: Ign_Coolant's id, Ign_Coolant_Cold's, Ign_AirTemp's, Accel_MapGain's, ColdStartIgn's
// bit. Out: IgnSlow, MapGainNow.
static void UNIT_CODE(ign)(const Task &t)
{
    uint32_t sum = sum_map(t, 0, get(t, bit(t, get(t, 4))));            // the cold map after a cold start
    put(t, 0, sum_map(t, sum, get(t, 2)));
    put(t, 1, value(t, get(t, 3)));
}

// Slow_Charge's time: (Dwell_Time x CHARGE_K + CHARGE_0) / 256, a byte at a time. Data: Dwell_Time's id,
// CHARGE_K's low byte and high byte, CHARGE_0's. Out: ChargeL, ChargeH.
static void UNIT_CODE(charge)(const Task &t)
{
    uint32_t time = value(t, get(t, 0));
    uint32_t low = time * get(t, 1), sum = (low & 0xFF) + get(t, 3);    // MUL AB; ADD A,#LOW(CHARGE_0)
    uint32_t r6 = ((low >> 8) + get(t, 4) + (sum >> 8)) & 0xFF;         // ADDC A,#HIGH(CHARGE_0): its carry lost
    uint32_t high = time * get(t, 2);                                   //   (MUL AB clears C)
    sum = (high & 0xFF) + r6;                                           // ADD A,R6
    put(t, 0, sum);                                                     // ChargeL
    put(t, 1, (high >> 8) + (sum >> 8));                                // CLR A; ADDC A,B: ChargeH
}

// Ign_Advance: the advance's maps summed, each less 20 (Sum_Map). Starting: by speed, then by coolant;
// at idle: by coolant (the cold map after a cold start), then by speed; off idle: the slow sum and the
// main map (Sum_Add). Data: Ign_StartSpeed's id, Ign_StartCoolant's, Ign_IdleCoolant's,
// Ign_IdleCoolant_Cold's, Ign_IdleSpeed's, Ign_Main's; StartMode's bit, IdleSwitch's, ColdStartIgn's;
// IgnSlow's RAM address. Out: Handover.
static void UNIT_CODE(advance)(const Task &t)
{
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
// again. Inputs, in turn: Rpm, the bytes of CrankDecay's bits (21h) and StartMode's (23h), InjCount,
// LamInt, FirePeriodL, FirePeriodH, FuelFactorL, FuelFactorH, FuelShift, WarmWeight, AfterStart,
// LoadHi, LoadMid, AccelEnrich, and Coolant, Battery and Fuel_Main's second input (the snoop's In5, or
// Load). Data: the ids of Fuel_Main, Fuel_Idle, Crank_Fuel, Crank_DecaySpeed, Crank_DecayCount,
// Crank_DecayRpm and Inj_DeadTime; Constants' address; the places there of Cal_StartLoad,
// Cal_CrankCount and Cal_InjMinTime; Resume_Ramp_Rpm's and Resume_Ramp_Throttle's addresses; the bits
// StartMode, IdleSwitch, CrankDecay, ResumeRamp, ResumeByRpm; the pulse's longest a turn (FIRINGS x 60:
// 15/16 of its two turns, x 32 / the firing) and starting (30: of its firing); INJ_OFS. Out: InjTimeL,
// InjTimeH (the injection time), DeadNow, InjCount, the flags (PulseFlags, below); TH0's load for the
// pulse goes into the 8051's code (pair_pulse).

namespace
{

// The pulse's flags, for the 8051 to apply as rom_lean's routines did.
enum : uint32_t
{
    PULSE_DECAY = 1,    // Fuel_Crank's decay begun: CrankDecay set (InjCount 0)
    PULSE_RAMP_END = 2, // the resume's ramp over: ResumeRamp cleared
    PULSE_SLEW_END = 4, // its slew over (Fuel_Mode_Slew_At): SlewResume cleared, SlewCount 0, if set
    PULSE_HELD = 8,     // over its turns, held (InjHold)
    PULSE_LATE = 0x10,  // made again after its reference: its load not written
};

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
    if (!decaying && (value(t, get(t, 3)) < input(t, 0)                 // Crank_DecaySpeed; SUBB A,Rpm; JC
                      || at(t.rom, word(t, 7) + get(t, 10)) < input(t, 3))) // Cal_CrankCount; SUBB A,InjCount; JNC
    {
        *flags |= PULSE_DECAY;                                          // SETB CrankDecay
        t.ram[at(t.rom, t.inputs + 3) & 0x7F] = 0;                      // MOV InjCount,#00h
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
// injections; made again at the step it was made at), the dead time on, held over 15/16 of its turns
// (then twice that), less INJ_OFS. The turn's pulse counted (InjCount, to 80h; not starting).
static Pulse UNIT_CODE(fuel_mode)(const Task &t, uint32_t *time, uint32_t dead, bool again, uint32_t *count,
                                  uint32_t *flags)
{
    bool starting = bit(t, get(t, 16));
    uint32_t r = *time, lamint = input(t, 4);
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
    if (!again && !starting && *count < 0x80)                           // JB ACC.7,Fuel_Mode_Dead; INC InjCount
        (*count)++;
    uint32_t pulse = dead * 5 + r;                                      // MOV B,#05h; MUL AB; ADD A,R6; ADDC A,R7
    if (pulse > 0xFFFF)
        pulse = 0xFF00 | (pulse & 0xFF);                                // MOV R3,#0FFh
    uint32_t wide = (input(t, 6) << 8 | input(t, 5)) * get(t, starting ? 22 : 21); // FirePeriod; Mul16x8
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

// The pulse: the cranking fuel's factor starting, else the main map (Fuel_Idle at idle, Fuel_Main
// off it), then Fuel_Time's arithmetic (the base time starting Cal_StartLoad x 25), then Fuel_Mode;
// TH0's load into the 8051's code.
static void UNIT_CODE(pulse)(const Task &t, bool again)
{
    uint32_t flags = 0, exponent = 0x80, map, base = input(t, 12) << 8 | input(t, 13);
    if (bit(t, get(t, 16)))                                             // JB StartMode,Fuel_Crank
    {
        map = crank_fuel(t, &exponent, &flags);
        base = at(t.rom, word(t, 7) + get(t, 9)) * 0x19;                // Cal_StartLoad x 25
    }
    else
        map = value(t, get(t, bit(t, get(t, 17)) ? 1 : 0));             // JB IdleSwitch: Fuel_Idle, else Fuel_Main
    uint32_t time = time_a_turn(input(t, 8) << 8 | input(t, 7), input(t, 9), exponent, input(t, 10), input(t, 11),
                                map, base, input(t, 14));
    uint32_t dead = value(t, get(t, 6));                                // Inj_DeadTime
    uint32_t count = t.ram[at(t.rom, t.inputs + 3) & 0x7F];             // InjCount (0 if the decay began)
    Pulse load = fuel_mode(t, &time, dead, again, &count, &flags);
    if (!pair_pulse(load.start, load.overflows, again))
        flags |= PULSE_LATE;
    put(t, 0, time);                                                    // InjTimeL
    put(t, 1, time >> 8);                                               // InjTimeH
    put(t, 2, dead);                                                    // DeadNow
    put(t, 3, count);                                                   // InjCount
    put(t, 4, flags);                                                   // PulseFlags
}

// Load_Tick's running path (the 8051 keeps the starting one): 50 / the speed x the air flow / 256
// (Mul16x16), held to 16 bits, x Cal_LoadScale / 256 (Mul16x8) and doubled unless its top bit is set:
// the new load; filtered by LoadK (Lag_Filter24, its low byte 0) into LoadHi:LoadMid. Inputs, in turn:
// Rpm, Afm, LoadK, LoadHi, LoadMid. Data: Constants' address; Cal_LoadScale's place there; Afm_Flow's
// address. Out: LoadHi, LoadMid.
static void UNIT_CODE(load)(const Task &t)
{
    uint32_t wide = per_turn(input(t, 0)) * airflow(t, word(t, 3), input(t, 1)) >> 8;   // R7:R6:R5
    if (wide > 0xFFFF)                                                  // MOV A,R7; JZ Load_Tick_Fits
        wide = 0xFFFF;                                                  // MOV R6,#0FFh; MOV R5,#0FFh
    wide *= at(t.rom, word(t, 0) + get(t, 2));                          // x Cal_LoadScale: R7:R6:R5
    if (!(wide & 0x800000))                                             // JB ACC.7,Load_Tick_Twice
        wide <<= 1;                                                     // RLC A: R5's top bit, R6, R7
    uint32_t fresh = wide >> 8 & 0xFFFF, k = input(t, 2);              // R4:R3; R5 = LoadK
    uint32_t old = input(t, 3) << 8 | input(t, 4), state = old << 8;   // R2:R1:R0, R0 = 0
    if (fresh >= old)                                                   // SUBB, SUBB: no borrow
        state += (fresh - old) * k;                                     // Mul16x8; ADD, ADDC, ADDC
    else
        state -= (old - fresh) * k;                                     // negated; Mul16x8; SUBB x 3
    put(t, 0, state >> 16);                                             // LoadHi: R2
    put(t, 1, state >> 8);                                              // LoadMid: R1
}

// ---------------------------------------------------------------------------------------------------
// Task 11 (format 8): the advance's rest, rom_lean's pipeline stage 4 from Ign_Offset to Ign_Slew.
// Inputs, in turn: Handover (the maps' sum, task 4's), Rpm, Load, the bytes of BoostOver's bits (21h),
// SlewHighLoad's (22h) and StartMode's (23h), Advance, SlewCount, BoostCount, InjCount, and the snoop's
// In5 and Coolant. Data: Ign_SpeedOffset's id; Constants' address; the places there of Cal_AdvanceMax,
// then Boost_MaxLoad's, Boost_MaxMap's and Overrun_CutSpeed's ids, then the places of Cal_BoostGuardMap,
// Cal_BoostDelay, Cal_BoostDelayMap, Cal_BoostRelease, Cal_BoostHyst, Cal_BoostReleaseTime,
// Cal_OverrunHyst, Cal_ResumeStepThrottle, Cal_ResumeStepRpm, Cal_SlewResume, Cal_SlewOverrun,
// Cal_SlewHighLoad, Cal_SlewNormal, Cal_SlewLoad and Cal_SlewRpm; the bits StartMode, IdleSwitch,
// NotSynced, FuelCut, BoostOver, BoostCut, ResumeRamp, ResumeByRpm, SlewHighLoad, SlewOverrun,
// SlewResume. Out: Advance, SlewCount, BoostCount, InjCount, and the flags (AdvFlags, below), which the
// 8051 copies into its bits; they are kept here too (pair.cpp's turn reads the cuts).

namespace
{

// The advance's flags, bit by bit as the 8051 copies them (AdvFlags).
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
    uint32_t advance, slew, boost, count;
    bool over, cut, fuel_cut, ramp, by_rpm, high, overrun, resume;
};

__force_inline uint32_t cal(const Task &t, uint32_t place)
{
    return at(t.rom, word(t, 1) + get(t, place));
}

} // namespace

// Ign_Boost: the overboost guard (with Cal_BoostGuardMap the snoop's In5 against Boost_MaxMap, else the
// load byte against Boost_MaxLoad, both by speed): over the limit for its delay of periods (BoostCount,
// counted down by the 8051's tick), the fuel cut till at or below the release level for its time, or
// the throttle closed. Starting: none.
static void UNIT_CODE(boost_guard)(const Task &t, AdvanceState &s, bool starting, bool idle)
{
    if (starting)
    {
        s.cut = s.over = false;                                         // Ign_Boost_Release
        return;
    }
    bool on_map = cal(t, 7) != 0;                                       // Cal_BoostGuardMap
    uint32_t b = on_map ? input(t, 10) : input(t, 2);                   // MOV B,In5 / MOV B,Load
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
    uint32_t level = on_map ? cal(t, 10) : (limit - cal(t, 11)) & 0xFF; // Cal_BoostRelease; or CPL; INC; ADD A,R3
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
    uint32_t rpm = input(t, 1), cutoff = value(t, get(t, 6));           // Overrun_CutSpeed
    if (cutoff >= rpm)                                                  // SUBB A,Rpm; JNC Ign_Overrun_Below
    {
        s.overrun = false;
        if (!s.fuel_cut)
            return false;
        s.by_rpm = true;
        return resume(t, s, cal(t, 15), target);                        // Cal_ResumeStepRpm
    }
    uint32_t hyst = at(t.rom, word(t, 1) + get(t, 13) + (s.by_rpm ? 1 : 0)); // Cal_OverrunHyst (+1)
    if (hyst + ((cutoff - rpm) & 0xFF) > 0xFF)                          // ADD A,R3; JC Ign_Taken: within it
        return false;
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
    bool c = cal(t, 20) < input(t, 2)                                   // Cal_SlewLoad; SUBB A,Load; JC
             || cal(t, 21) >= input(t, 1);                              // Cal_SlewRpm; SUBB A,Rpm; CPL C
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

static void UNIT_CODE(advance_rest)(const Task &t)
{
    AdvanceState s;                                                     // (field by field: no initializer in flash)
    s.advance = input(t, 6);
    s.slew = input(t, 7);
    s.boost = input(t, 8);
    s.count = input(t, 9);
    s.over = bit(t, get(t, 26));
    s.cut = bit(t, get(t, 27));
    s.fuel_cut = bit(t, get(t, 25));
    s.ramp = bit(t, get(t, 28));
    s.by_rpm = bit(t, get(t, 29));
    s.high = bit(t, get(t, 30));
    s.overrun = bit(t, get(t, 31));
    s.resume = bit(t, get(t, 32));
    bool starting = bit(t, get(t, 22)), idle = bit(t, get(t, 23));
    uint32_t target = target_of(t, input(t, 0), get(t, 0), word(t, 1), get(t, 3)); // Ign_Offset: R5, Handover
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
    t.ram[at(t.rom, t.inputs) & 0x7F] = target;                         // Handover, as rom_lean's left it
    put(t, 0, s.advance);
    put(t, 1, s.slew);
    put(t, 2, s.boost);
    put(t, 3, s.count);
    put(t, 4, flags);
}

// ---------------------------------------------------------------------------------------------------
// Task 12 (format 8): the tick's start mode, rom_lean's Main_Tick from Cal_StartRpmEnter on, with
// Start_Variants: at Cal_StartRpmEnter or under, start mode and the cold-start variants by coolant
// (at or under Cal_ColdFuel, Cal_ColdIgn); in it, till over Start_ExitSpeed (by coolant) or the
// valve's ramp done (IsvRamp 0): then over, and the next reference injects (pair.cpp). Inputs: Rpm,
// IsvRamp, the byte of StartMode's bit (23h), the snoop's Coolant. Data: Constants' address; the places
// there of Cal_StartRpmEnter, Cal_ColdFuel and Cal_ColdIgn; Start_ExitSpeed's id; StartMode's bit. Out:
// StartFlags (below), which the 8051 applies; StartMode kept here too.

namespace
{

enum : uint32_t
{
    START_MODE = 1,     // StartMode
    START_COLD_FUEL = 2, // ColdStartFuel ...
    START_COLD_IGN = 4, // ... and ColdStartIgn, made (START_VARIANTS)
    START_VARIANTS = 8, // starting: the variants made
    START_OVER = 0x10,  // start mode over: Inj720Odd cleared, the AFM's filter the AFM (the 8051's)
};

} // namespace

static void UNIT_CODE(start_mode)(const Task &t)
{
    uint32_t c = word(t, 0), rpm = input(t, 0), coolant = input(t, 3), flags = 0;
    bool starting = bit(t, get(t, 6));
    if (at(t.rom, c + get(t, 2)) >= rpm)                                // Cal_StartRpmEnter; SUBB A,Rpm; JNC
    {
        starting = true;
        flags |= START_VARIANTS;
        if (at(t.rom, c + get(t, 3)) >= coolant)                        // Cal_ColdFuel; CJNE A,Coolant; CPL C
            flags |= START_COLD_FUEL;
        if (at(t.rom, c + get(t, 4)) >= coolant)                        // Cal_ColdIgn
            flags |= START_COLD_IGN;
    }
    else if (starting && (input(t, 1) == 0 || value(t, get(t, 5)) < rpm)) // IsvRamp 0; Start_ExitSpeed; JNC
    {
        starting = false;
        flags |= START_OVER;
        pair_start_over();
    }
    if (starting)
        flags |= START_MODE;
    set_bit(t, get(t, 6), starting);
    put(t, 0, flags);
}

bool UNIT_CODE(unit_task)(uint32_t task, uint32_t description, const uint8_t *rom, uint8_t *ram)
{
    uint32_t outputs = description + 1 + at(rom, description); // their count, then their addresses
    Task t = {rom, ram, description + 1, outputs + 1, outputs + 1 + at(rom, outputs)};
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
    else
        return false;
    return true;
}

#endif // FEATURE_MAP_UNIT
