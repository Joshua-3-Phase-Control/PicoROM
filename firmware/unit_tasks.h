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
};

// Works task out from its description; false if there is no such task.
bool unit_task(uint32_t task, uint32_t description, const uint8_t *rom, uint8_t *ram);

// From the map unit (map_unit.cpp): map id's value, its inputs as ram holds them.
uint32_t unit_map_value(uint32_t id, const uint8_t *ram);

#endif

#endif
