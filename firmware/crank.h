#if !defined(CRANK_H)
#define CRANK_H 1

#include <stdint.h>

// The crank model (FEATURE_MAP_UNIT, crank.cpp; step 2b of DME-Sim's docs/P28_Pairing.md): from what
// the snoop sees alone, the tooth the engine is at and each firing's length.
#if defined(FEATURE_MAP_UNIT)

// Its state as the board starts it, with the image it serves (format 5 names the 8051's slot reads).
void crank_start(const uint8_t *rom);

// From the snoop: a tooth's routine began (INT1's vector), and a MOVX read whose opcode the byte at
// after follows; now, the microsecond timer then.
void crank_tooth(uint32_t now);
void crank_read(uint32_t after, uint32_t now);

#endif

#endif
