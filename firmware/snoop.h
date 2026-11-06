#if !defined(SNOOP_H)
#define SNOOP_H 1

#include <stddef.h>
#include <stdint.h>

// The snoop (FEATURE_MAP_UNIT, snoop.cpp, docs/snoop.md): what the board learns of the 8051 from the
// socket alone, each MOVX (an ADC read's channel and result, a write) and each interrupt it takes.
#if defined(FEATURE_MAP_UNIT)

// After map_unit_init (which puts the unit's bank in place), with the image the board serves.
void snoop_init(const uint8_t *rom);

// From the map unit's interrupt, each read of page 0 (A0-A7): its vectors.
void snoop_page_read(uint32_t address);

// The 8051 read the board since the last call (the activity LED).
bool snoop_activity();

bool snoop_set_parameter(const char *name, const char *value);
bool snoop_get_parameter(const char *name, char *value, size_t value_size);

#endif

#endif
