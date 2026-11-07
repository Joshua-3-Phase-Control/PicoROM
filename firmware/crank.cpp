// The crank model (FEATURE_MAP_UNIT; step 2b of DME-Sim's docs/P28_Pairing.md): from what the snoop
// sees alone, the tooth the engine is at and each firing's length, as the 8051's chain and its Timer 1
// capture have them.
//
//   the tooth: each tooth's routine (INT1's vector, snoop_page_read) moves it on one, round the
//   firing's teeth; each of the 8051's slot reads (an ADC read at a crank event) says which tooth it is
//   on, and sets it there when the count was off (a slip): short by the teeth the 8051 took inside
//   another's routine, which the snoop doesn't see begin; over by one the chain set back.
//   the firing's length: from one read of slot 0 to the next, in microseconds, as the 8051 takes it
//   from Timer 1 at slot 0.
//
// Format 5 names the slot reads, in a table the mark's eleventh and twelfth bytes point to: the teeth a
// firing, a count, then for each read the address of the byte after its MOVX's opcode (what the snoop
// sees read last), its tooth in the firing and its slot.

#include "crank.h"

#if defined(FEATURE_MAP_UNIT)

#include "unit_ram.h"

static constexpr uint32_t MARK = 0x2000;
static constexpr uint32_t NONE = ~0u;
static constexpr uint32_t LOG = 16; // the firings' lengths kept

struct Crank
{
    const uint8_t *rom;
    uint32_t table;               // the slot reads' table, or 0 (an image before format 5)
    uint32_t teeth;               // teeth a firing
    uint32_t tooth;               // the tooth the engine is at in its firing (0 to teeth - 1), or NONE
    uint32_t last;                // when slot 0 was read last (us), and whether it has been
    bool started;
    volatile uint32_t firings;    // slot 0's reads
    volatile uint32_t reads;      // slot reads
    volatile uint32_t slips;      // slot reads that found the count off
    volatile uint32_t teeth_seen; // teeth whose routine began
    volatile uint32_t period;     // the last firing's length (us)
    volatile uint32_t periods;    // firings measured
    uint32_t log[LOG];            // the last firings' lengths (us), by periods
    volatile uint32_t missed;     // teeth the count was short at slot reads, summed (taken unseen)
    volatile uint32_t extra;      // teeth it was over (the 8051's chain set back a tooth)
};

static Crank crank UNIT_DATA = {};

static __force_inline uint32_t at(uint32_t address)
{
    return crank.rom[address & 0xFFFF];
}

void UNIT_CODE(crank_start)(const uint8_t *rom)
{
    crank.rom = rom;
    bool marked = at(MARK) == 'M' && at(MARK + 1) == 'A' && at(MARK + 2) == 'P' && at(MARK + 3) == 'U';
    crank.table = marked && at(MARK + 4) >= 5 ? at(MARK + 11) << 8 | at(MARK + 12) : 0;
    crank.teeth = crank.table != 0 ? at(crank.table) : 0;
    crank.tooth = NONE;
    crank.started = false;
}

void UNIT_CODE(crank_tooth)(uint32_t now)
{
    (void)now;
    crank.teeth_seen++;
    if (crank.tooth != NONE && ++crank.tooth == crank.teeth)
        crank.tooth = 0;
}

void UNIT_CODE(crank_read)(uint32_t after, uint32_t now)
{
    if (crank.table == 0)
        return;
    for (uint32_t i = 0, n = at(crank.table + 1); i < n; i++)
    {
        uint32_t entry = crank.table + 2 + 4 * i;
        if ((at(entry) << 8 | at(entry + 1)) != after)
            continue;
        uint32_t tooth = at(entry + 2), slot = at(entry + 3);
        crank.reads++;
        if (crank.tooth != tooth)
        {
            if (crank.tooth != NONE)
            {
                crank.slips++;
                uint32_t short_by = tooth >= crank.tooth ? tooth - crank.tooth : tooth + crank.teeth - crank.tooth;
                if (short_by <= crank.teeth / 2)
                    crank.missed += short_by;
                else
                    crank.extra += crank.teeth - short_by;
            }
            crank.tooth = tooth;
        }
        if (slot == 0)
        {
            if (crank.started)
            {
                crank.period = now - crank.last;
                crank.log[crank.periods++ % LOG] = crank.period;
            }
            crank.last = now;
            crank.started = true;
            crank.firings++;
        }
        return;
    }
}

#endif // FEATURE_MAP_UNIT
