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
static void UNIT_CODE(offset)(const Task &t)
{
    uint32_t sum = sum_map(t, byte_of(t, 4), get(t, 0));                // MOV B,Handover; Sum_Map
    uint32_t negative = sum >> 7;                                       // MOV C,ACC.7
    uint32_t limit = at(t.rom, word(t, 1) + ((get(t, 3) + negative) & 0xFF)); // MOV A,#..; (INC A;) MOVC
    uint32_t kept = negative ? (sum >= limit ? sum : limit)             // the sum less the limit, no borrow
                             : (limit >= sum ? sum : limit);            // the limit less the sum, no borrow
    put(t, 0, kept >> 1 | (kept & 0x80));                               // MOV C,ACC.7; RRC A
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
    else
        return false;
    return true;
}

#endif // FEATURE_MAP_UNIT
