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

#else

static inline void map_unit_hide() {}
static inline void map_unit_show() {}

#endif

#endif // MAP_UNIT_H
