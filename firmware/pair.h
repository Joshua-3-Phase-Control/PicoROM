#if !defined(PAIR_H)
#define PAIR_H 1

#include <stddef.h>
#include <stdint.h>

// The paired core's P28 side (FEATURE_MAP_UNIT, format 6, pair.cpp; DME-Sim's docs/Paired_Core.md):
// the P28 keeps the crank from INT1's vector, which every tooth fetches, and writes that vector
// before each tooth: a RETI, or an AJMP to the event of the tooth to come. It plans the coil as
// rom_lean's main level did, from the advance the 8051 sends (task 8).
#if defined(FEATURE_MAP_UNIT)

// Its state as the board starts it, with the image it serves (and writes: the live bytes).
void pair_start(uint8_t *rom);

// From the snoop: each read of page 0 (A0-A7), and the microsecond timer then.
void pair_page_read(uint32_t address, uint32_t now);

// From the map unit's interrupt, its words all taken: the coil's plans and the next tooth's vector.
void pair_after();

// Task 8: the advance the 8051 sent (half-teeth, signed: Advance + AdvanceDyn), which the next plans
// take.
void pair_advance(uint32_t advance);

// Tasks 9 and 10 (format 7): TH0's load for the next pulse, its start and its overflows, written
// into the copies of the 8051's loads not in use, then the switches turned to them. False for a pulse
// made again (again) after its reference came: none written.
bool pair_pulse(uint32_t start, uint32_t overflows, bool again);

// Task 12 (format 8): start mode over, as the tick found it: the next reference injects (rom_lean's
// Main_Tick_Over cleared Inj720Odd).
void pair_start_over();

// Format 11: whether a task runs on the tick's copy of the RAM (L2's job, DME-Sim's
// docs/Paired_Levels.md), whose commit gives start mode's end to the phase (pair_start_over), not
// the task itself.
bool pair_on_tick_copy(const uint8_t *ram);

// Format 9 (the pipeline on the unit, DME-Sim's docs/Paired_Pipeline.md): from the bus level's tasks,
// a job for the pipeline's level: task 12's overboost guard's period (BoostCount one less, to 0) and
// task 13's tick values sent (the waiting pulse made again).
void pair_guard_period();
void pair_tick_sent();

// Format 10 (the tick's work on the unit, DME-Sim's docs/Paired_Tick.md): from task 12's end, the
// tick's work (task 15) for the pipeline's level, once a ping has come since the 8051's start.
void pair_tick_work();

// The levels (DME-Sim's docs/Paired_Levels.md). L1, the engine's events (format 9's pipeline):
// SPARE_IRQ_0's handler, at 80h (the bus's interrupts break into it); it does the jobs the bus level
// queued, one a run. L2, the tick (format 11): SPARE_IRQ_1's, at C0h (L1 breaks into it too); the
// whole tick at each ping, then the log's frame.
void pair_control();
void pair_tick();

// Format 12 (DME-Sim's docs/Paired_Map_Pulse.md): from the snoop's interrupt, a slot's read (format 5's
// slot) and its byte: a MAP read's job for L1, which makes the MAP pulse (task 18) and gives it to the
// 8051's MAP take.
void pair_slot_read(uint32_t slot, uint32_t data);

// The levels' counts for the PC: pair_events (L1: jobs, lost, most queued, longest us, over the bound),
// pair_tick (L2: ticks, run again, late, lost, longest us, over the bound) and pair_map (format 12: the
// MAP jobs, the pulses given). False: another name.
bool pair_get_parameter(const char *name, char *value, size_t value_size);

// Format 11 (DME-Sim's docs/Paired_Lambda_Idle.md): the log, each frame of it made at its tick into a
// ring (the 8051 does no serial). The bytes written since *from (a count of all written), at most
// most of them, into bytes; *from moved past them (on past what the ring no longer holds).
uint32_t pair_log_read(uint32_t *from, uint8_t *bytes, uint32_t most);

// DME-Sim's checks of the ported routines: routine's inputs and outputs in pair_case (pair.cpp).
uint32_t pair_test(uint32_t routine);

#endif

#endif
