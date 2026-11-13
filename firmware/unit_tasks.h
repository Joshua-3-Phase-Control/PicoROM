#if !defined(UNIT_TASKS_H)
#define UNIT_TASKS_H 1

#include <stdint.h>

// The map unit's tasks (FEATURE_MAP_UNIT, format 3; docs/map_unit.md): rom_lean's slow work, worked
// out for the 8051 bit for bit as rom_lean's routines make it (DME-Sim's experiments/coproc_slow).
// A task's description, in the unit's own memory, is the count of its inputs and their RAM
// addresses, the count of its outputs and theirs, then its data: the maps it looks up (by id), the
// tables and constants it reads in the 8051's 8 KiB, the 8051's bits it tests. ram holds the 8051's
// RAM bytes sent (by address, 00h-7Fh) and takes those the task makes.
#if defined(FEATURE_MAP_UNIT)

enum : uint32_t
{
    TASK_WARMUP,  // rom_lean's Slow_WarmUp: the warm-up enrichment
    TASK_FUEL,    // Slow_Fuel: the slow fuel factor
    TASK_IGN,     // Main_Ign: the advance's slow sum and the MAP pulse's gain
    TASK_CHARGE,  // Slow_Charge: the charge's time
    TASK_ADVANCE, // Ign_Advance: the advance's maps summed (the pipeline's stage 3)
    TASK_OFFSET,  // Ign_Offset to Ign_Halve: the target, the sum with the speed's offset, clamped, halved
    TASK_TIME,    // Fuel_Time: the injection time a turn and the dead time (the pipeline's stage 6)
    TASK_LOAD,    // Load_Tick's running path: the load a tick, filtered
    TASK_COIL,    // the coil's advance (format 6): sent alone, pair.cpp's plans take it from the RAM
    TASK_PULSE,   // the pulse (format 7): Fuel_Maps or Fuel_Crank, Fuel_Time and Fuel_Mode, TH0's load
                  //   written into the 8051's code (pair_pulse)
    TASK_AGAIN,   // the same, the pulse made again before its reference
    TASK_REST,    // the advance's rest (format 8): Ign_Offset, the overboost guard, the overrun cut, the
                  //   slew (the pipeline's stage 4)
    TASK_START,   // the tick's start mode (format 8): Main_Tick's, with Start_Variants; its end to pair.cpp
    TASK_TICK,    // the tick's values the 8051 makes (format 9), sent once a tick: the switches into their
                  //   bits, then pair.cpp's pulse made again
    TASK_SPEED,   // the pipeline's stage 2 (format 9): Speed_Count and Idle_Spark, which pair.cpp's
                  //   pipeline runs (the 8051 never asks for it)
    TASK_TICKWORK, // the tick's work (format 10): the switches, the enrichments, the dynamic spark's
                   //   filter, the AFM's rise, the load and its byte, the warm-up's weight, which
                   //   pair.cpp's tick job runs after task 12 (the 8051 never asks for it)
    TASK_LAMBDA,   // lambda (format 11): Tick_Stop's open loop, Main_Tick's sensor timer, Main_Lambda,
                   //   which pair.cpp's tick runs in parts (unit_task_part)
    TASK_IDLE,     // the idle valve's control (format 11): Main_Idle, then Idle_Duty with Idle_Ramp
    TASK_MAP,      // the MAP pulse (format 12): Main_Map for one read and Accel_Pulse's arithmetic, which
                   //   pair.cpp's MAP job runs at each MAP read the snoop sees (unit_map_pulse; the 8051
                   //   never asks for it)
};

// Format 11's parts of tasks 15, 16 and 17, which pair.cpp's tick runs in rom_lean's order.
enum : uint32_t
{
    TICK_SWITCHES, // Main_Tick's first: the throttle switches (task 15 then makes none), which the open
                   //   loop takes
    LAMBDA_OPEN,  // Tick_Stop's: the loop open at once (Lambda_Load_Open), its integrator x 1.0
    LAMBDA_TIMER, // Main_Tick's: the sensor's period, ready or not
    LAMBDA_RUN,   // Main_Lambda
    IDLE_STATE,   // Main_Idle: the target and the integrator; true if the duty is due (in step)
    IDLE_DUTY,    // Idle_Duty with Idle_Ramp: the valve's high phase into IsvHighL and IsvHighH
};

// Format 9 (DME-Sim's docs/Paired_Pipeline.md): the pipeline is pair.cpp's, and the bytes it keeps the
// unit's own: tasks 7 and 12 take the speed byte so, and task 12 hands the overboost guard's periods
// to the pipeline's level.
static constexpr uint32_t PIPELINE_FORMAT = 9;

// Format 10 (DME-Sim's docs/Paired_Tick.md): the tick's work is pair.cpp's too (task 15, queued at
// task 12's end, which sets the AFM's filter at start mode's end); task 13 sends lambda's integrator
// and the idle target alone.
static constexpr uint32_t TICK_FORMAT = 10;

// Format 11 (DME-Sim's docs/Paired_Lambda_Idle.md): the whole tick is pair.cpp's, at the 8051's ping:
// task 12 runs there (the window asks for it at power-up alone), then the tick's work, lambda (task
// 16) and the idle control (task 17); the 8051 sends no tick's values (task 13 gone) and does no
// serial (the log is pair.cpp's). Task 12 keeps the guard's count in its own copy then.
static constexpr uint32_t LOG_FORMAT = 11;

// Format 12 (DME-Sim's docs/Paired_Map_Pulse.md): the MAP pulse is pair.cpp's, made at each MAP read
// the snoop sees (task 18); the 8051 applies it from a take of its own.
static constexpr uint32_t MAP_FORMAT = 12;

// Format 13 (DME-Sim's docs/Paired_Slow_Work.md): the slow work (tasks 0 to 3) is pair.cpp's, at each
// slow result the snoop sees, and at the 8051's start with start mode (task 12): the 8051 asks for no
// task through the window.
static constexpr uint32_t SLOW_FORMAT = 13;

// Format 14 (DME-Sim's docs/Paired_Transients.md): task 18 carries a rise under the deadband to its
// slot's next read, as the image's Cal_MapCarry says.
static constexpr uint32_t CARRY_FORMAT = 14;

// Format 15 (DME-Sim's docs/Paired_Tip_Outs.md): a fall of the MAP past its own deadband owes a cut, by a
// gain by coolant (task 18), which the pulse task takes off the turn's pulse; its data the cut's two
// bytes.
static constexpr uint32_t DECEL_FORMAT = 15;

// The pulse's flags (PulseFlags, tasks 9 and 10): the 8051 applies the held pulse; the rest say what
// the unit did with its own count and bits (format 8), as rom_lean's routines did.
enum : uint32_t
{
    PULSE_DECAY = 1,    // Fuel_Crank's decay begun: CrankDecay set (InjCount 0)
    PULSE_RAMP_END = 2, // the resume's ramp over: ResumeRamp cleared
    PULSE_SLEW_END = 4, // its slew over (Fuel_Mode_Slew_At): SlewResume cleared, SlewCount 0, if set
    PULSE_HELD = 8,     // over its turns, held (InjHold)
    PULSE_LATE = 0x10,  // made again after its reference: its load not written
};

// Works task out from its description on ram, with own the own values that go with it (format 18; null:
// map_unit_owns_on's); false if there is no such task.
bool unit_task(uint32_t task, uint32_t description, const uint8_t *rom, uint8_t *ram, uint32_t *own = nullptr);

// Format 11: a part of task 15, 16 or 17 (above); IDLE_STATE's answer, else true.
bool unit_task_part(uint32_t task, uint32_t part, uint32_t description, const uint8_t *rom, uint8_t *ram,
                    uint32_t *own = nullptr);

// Format 12: task 18 for one MAP read (slot 0 the first MAP read's, 1 the second's; read its byte) on
// ram: its slot's filter stepped; true if a pulse is due, its loads then in *load: TH0's start for a
// pulse of its own (bits 0-7) and its overflows (8-15), the pulse on lengthened by (16-31).
bool unit_map_pulse(uint32_t description, const uint8_t *rom, uint8_t *ram, uint32_t slot, uint32_t read,
                    uint32_t *load);

// From the map unit (map_unit.cpp): map id's value, its inputs as ram and own (format 18: the own values
// that go with ram, map_unit_owns_on's) hold them.
uint32_t unit_map_value(uint32_t id, const uint8_t *ram, const uint32_t *own);

// Format 18: Idle_Error with the target in rpm (pair_test's, for DME-Sim's twin).
uint32_t unit_fine_error_rpm(uint32_t target, uint32_t p, uint32_t k);

// Format 20: the load task's air of a firing and its filter (pair_test's, for DME-Sim's twin): the own
// value 81h's word after a firing of P cycles at the flow (g/min), filtered by k.
uint32_t unit_filter_air(uint32_t flow, uint32_t p, uint32_t k, uint32_t own);

// Format 21 (pair_test's, for DME-Sim's twins): K (its mantissa, bits 0-15, and exponent, 16-31) from the
// fuel's constants; the time a turn from its parts (unit_tasks.cpp's in[] order), running or starting;
// a fade's share of a count at its period and the tick counter's remainder.
uint32_t unit_fuel_k(uint32_t stoich, uint32_t flow, uint32_t density, uint32_t rated, uint32_t rail);
uint32_t unit_fuel_time(const uint32_t *in, bool starting);
uint32_t unit_fade_share(uint32_t period, uint32_t r);

// Format 22 (pair_test's, for DME-Sim's twin): the MAP pulse's gains' factor, 1/32 cycle a count of the MAP
// for 1 ug of fuel a kPa (its mantissa, bits 0-15, and exponent, 16-31), from K's constants (in[0] to in[4])
// and the MAP sensor's Pa a count (in[5]).
uint32_t unit_map_gain_k(const uint32_t *in);

#endif

#endif
