#if !defined(PAIR_H)
#define PAIR_H 1

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

// Format 9 (the pipeline on the unit, DME-Sim's docs/Paired_Pipeline.md): from the bus level's tasks,
// a job for the pipeline's level: task 12's overboost guard's period (BoostCount one less, to 0) and
// task 13's tick values sent (the waiting pulse made again).
void pair_guard_period();
void pair_tick_sent();

// The pipeline's level: SPARE_IRQ_0's handler, at the lowest priority (the bus's interrupts break into
// it); it does the jobs the bus level queued, in turn.
void pair_control();

// DME-Sim's checks of the ported routines: routine's inputs and outputs in pair_case (pair.cpp).
uint32_t pair_test(uint32_t routine);

#endif

#endif
