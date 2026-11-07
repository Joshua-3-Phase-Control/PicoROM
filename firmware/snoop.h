#if !defined(SNOOP_H)
#define SNOOP_H 1

#include <stddef.h>
#include <stdint.h>

// The snoop (FEATURE_MAP_UNIT, snoop.cpp, docs/snoop.md): what the board learns of the 8051 from the
// socket alone, each MOVX (an ADC read's channel and result, a write) and each interrupt it takes.
#if defined(FEATURE_MAP_UNIT)

// After map_unit_init (which puts the unit's bank in place), with the image the board serves.
void snoop_init(const uint8_t *rom);

// The snoop's state as the board starts it, without the hardware: snoop_init calls it, and DME-Sim's
// emulated core runs it.
void snoop_start(const uint8_t *rom);

// From the map unit's interrupt, each read of page 0 (A0-A7): its vectors.
void snoop_page_read(uint32_t address);

// An ADC channel's last result (back 0) or the one before it (back 1), 0 before there is one, and how
// many results it has had.
uint32_t snoop_result(uint32_t channel, uint32_t back);
uint32_t snoop_results(uint32_t channel);

// The 8051 read the board since the last call (the activity LED).
bool snoop_activity();

bool snoop_set_parameter(const char *name, const char *value);
bool snoop_get_parameter(const char *name, char *value, size_t value_size);

#endif

#endif
