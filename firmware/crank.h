#if !defined(CRANK_H)
#define CRANK_H 1

#include <stdint.h>

// The crank model (FEATURE_MAP_UNIT, crank.cpp; step 2b of DME-Sim's docs/P28_Pairing.md): from what
// the snoop sees alone, the tooth the engine is at and each firing's length.
#if defined(FEATURE_MAP_UNIT)

// Its state as the board starts it, with the image it serves (format 5 names the 8051's slot reads).
void crank_start(const uint8_t *rom);

// From the snoop: a tooth's routine began (INT1's vector), and a MOVX read whose opcode the byte at
// after follows; now, the microsecond timer then. crank_read gives the read's slot (format 5's table),
// or CRANK_NO_SLOT for a read that is none of the slots'.
static constexpr uint32_t CRANK_NO_SLOT = 0xFF;
void crank_tooth(uint32_t now);
uint32_t crank_read(uint32_t after, uint32_t now);

#endif

#endif
