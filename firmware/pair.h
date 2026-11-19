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
// made again (again) after its reference came: none written. Format 25: fuel, the pulse's fuel (its
// cycles less the dead time), which the port films take as it is injected.
bool pair_pulse(uint32_t start, uint32_t overflows, bool again, uint32_t fuel);

// Format 26 (DME-Sim's docs/Paired_Wall_Film.md, step 2): the films' term for the running pulse, R (cycles,
// the low half) and F (1/8192, the high half), the pulse (the need - R) x F; 0: none. And the pulse task's
// need (cycles) and its air of a firing (0.1 mg), for the MAP pulse's gains.
uint32_t pair_film_term();
void pair_film_need(uint32_t need, uint32_t air);

// Format 27 (step 4): the manifold's take of a firing (0.1 mg; 0: none), L2's, which the pulse task takes off
// the air of a firing for the need.
uint32_t pair_fill_take();

// Format 29 (step 6): the warm-up enrichment (a word, 8000h a whole fuel) blended toward the share of it a hot
// chamber keeps (Fuel_WarmUpKept by coolant, on ram and own) by the chamber's estimate; the tick's (in bank 2).
uint32_t pair_chamber_warm_up(uint32_t warm, const uint8_t *ram, const uint32_t *own);

// Task 12 (format 8): start mode over, as the tick found it: the next reference injects (rom_lean's
// Main_Tick_Over cleared Inj720Odd).
void pair_start_over();

// Format 11: whether a task runs on the tick's copy of the RAM (L2's job, DME-Sim's
// docs/Paired_Levels.md), whose commit gives start mode's end to the phase (pair_start_over), not
// the task itself.
bool pair_on_tick_copy(const uint8_t *ram);

// Format 18: one of the P28's own 16-bit values (map_unit.h's) set by a task working on ram: on a level's
// copy of the RAM that level's copy of it, given back at its commit (a level breaking in sees the unit's
// own, as with the RAM's bytes; map_unit_own_on reads them); else the unit's.
void pair_set_own(uint32_t input, uint32_t value, const uint8_t *ram);

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
// slot), its byte and the channel whose conversion it collects: a MAP read's job for L1, which makes the
// MAP pulse (task 18) and gives it to the 8051's MAP take; format 13 (docs/Paired_Slow_Work.md), a slow
// result's job, which turns slot 5's channel and does the slow work (tasks 0 to 3).
void pair_slot_read(uint32_t slot, uint32_t data, uint32_t channel);

// The levels' counts for the PC: pair_events (L1: jobs, lost, most queued, longest us, over the bound),
// pair_tick (L2: ticks, run again, late, lost, longest us, over the bound), pair_map (format 12: the
// MAP jobs, the pulses given) and pair_slow (format 13: the slow results worked on). False: another
// name.
bool pair_get_parameter(const char *name, char *value, size_t value_size);

// Format 11 (DME-Sim's docs/Paired_Lambda_Idle.md): the log, each frame of it made at its tick into a
// ring (the 8051 does no serial). The bytes written since *from (a count of all written), at most
// most of them, into bytes; *from moved past them (on past what the ring no longer holds).
uint32_t pair_log_read(uint32_t *from, uint8_t *bytes, uint32_t most);

// DME-Sim's checks of the ported routines: routine's inputs and outputs in pair_case (pair.cpp).
uint32_t pair_test(uint32_t routine);

// The store's knock kind (store.h; DME-Sim's docs/Paired_Fuel_Learning.md), in bank 2 with the store,
// its record two parts of 64 bytes (each pair's lower, a byte a point: A and D's, B and C's):
// knock_differs whether part one's bytes differ from kept's (128), knock_copy part one's bytes into out
// (128), knock_restore the maps from the record's bytes, knock_clear the maps to 0 (L2's, as knock_tick
// is), knock_key the key of what the maps mean (their layout and axes in the image; 0 with the work
// off), knock_stopped the engine stopped (the P28 out of step), when the PC may clear them.
bool knock_differs(uint32_t one, const uint8_t *kept);
void knock_copy(uint32_t one, uint8_t *out);
void knock_restore(const uint8_t *in);
void knock_clear();
uint32_t knock_key();
bool knock_stopped();

// Format 30 (DME-Sim's docs/Paired_Fuel_Learning.md, step 2): the store's trims kind, in bank 2 with the store, its
// record two parts (the map's points 0 to 31, then 32 to 63 and the idle factor; a word each, high byte first: the
// factor less 1 in 2^-15, signed, 130 bytes): trims_differs whether part one's bytes differ from kept's, trims_copy
// part one's bytes into out, trims_restore the trims from the record's bytes, trims_clear the trims to 0 (L2's, as
// the trims' tick is), trims_key the key of what they correct (their layout and axes, the steady fuel's maps and
// constants in the image; 0 with the work off). pair_live a kind's live copy in the image's P28 half (its bytes as
// last stored, which thread mode writes after each record and the board's start from what it loaded; null: none).
bool trims_differs(uint32_t one, const uint8_t *kept);
void trims_copy(uint32_t one, uint8_t *out);
void trims_restore(const uint8_t *in);
void trims_clear();
uint32_t trims_key();
volatile uint8_t *pair_live(uint32_t kind);

#endif

#endif
