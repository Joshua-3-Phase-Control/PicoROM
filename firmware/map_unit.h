#if !defined(MAP_UNIT_H)
#define MAP_UNIT_H 1

#include <stddef.h>
#include <stdint.h>

// The map unit (FEATURE_MAP_UNIT): the board looks maps up for the 8051 (map_unit.cpp,
// docs/map_unit.md). Without the feature these do nothing, and the PC reads and writes the image
// as before.
#if defined(FEATURE_MAP_UNIT)

// After the image is loaded from flash, with the address mask the board serves through.
void map_unit_init(uint8_t *rom, uint32_t addr_mask);
void map_unit_set_mask(uint32_t addr_mask);

// The unit's state as the board starts it, without the hardware: map_unit_init calls it, and
// DME-Sim's emulated core runs it to start the unit as the board does.
void map_unit_start(uint8_t *rom, uint32_t addr_mask);

// The PC's writes and reads of the image: the unit's page holds its answers, and the PC sees the
// image as it wrote it there.
void map_unit_write(uint32_t offset, const uint8_t *data, uint32_t size);
void map_unit_read(uint32_t offset, uint8_t *into, uint32_t size);

// Around a commit: the image as the PC wrote it goes to flash.
void map_unit_hide();
void map_unit_show();

bool map_unit_set_parameter(const char *name, const char *value);
bool map_unit_get_parameter(const char *name, char *value, size_t value_size);
extern const char *const map_unit_parameter_names[];

// From the main loop: ends a bus bench that has finished.
void map_unit_update();

// From the snoop's interrupt: an ADC result on the channel, which the bytes the unit makes from it
// take (format 4).
void map_unit_adc(uint32_t channel);

// The 8051's RAM byte at address as the unit has it: sent with a task, or a task's output (pair.cpp
// takes the advance and the charge's time so).
uint32_t map_unit_ram(uint32_t address);

// Format 8: a RAM byte the unit keeps for the 8051 changed (pair.cpp's counts at its marks), and the
// bytes from..to cleared (as the 8051's Power_Up clears its own).
void map_unit_ram_write(uint32_t address, uint32_t value);
void map_unit_ram_clear(uint32_t from, uint32_t to);

// Format 9 (pair.cpp's pipeline): the unit's copy of the 8051's RAM as 32 words (its bytes 00h-7Fh),
// which the pipeline's level takes and gives back with the interrupts held off; the bytes the snoop
// makes put over a copy of it (as a task's snooped inputs are); a task's description by its number
// (0: the image has none).
uint32_t *map_unit_ram_words();
void map_unit_snooped_over(uint8_t *bytes);
uint32_t map_unit_task_at(uint32_t task);

#else

static inline void map_unit_hide() {}
static inline void map_unit_show() {}

#endif

#endif // MAP_UNIT_H
