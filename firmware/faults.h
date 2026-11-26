#if !defined(FAULTS_H)
#define FAULTS_H 1

#include <stddef.h>
#include <stdint.h>

// The fault codes (FEATURE_MAP_UNIT, format 31; DME-Sim's docs/Paired_Faults.md): 19 checks at L2 on what the
// P28 has already, each confirmed when it has stood for its time and healed when gone for its time, Porsche's
// four-digit codes (11xx present, 12xx once gone, with a cause), and a memory of 7 records kept as the store's
// third kind. Format 31 records only; the actions come a format each from 32 (the note's section 3): 33 the trims'
// rule (pair.cpp's trim_tick) and lambda's loop opened at 1123's fuel-off limit (task 16 reads fault_loop_open); 35
// the NTCs' substitutes (map_unit_adc gives fault_substitutes); 36 1112's check at the overrun's cut-off and the idle
// contact taken open above it while 1112 stands (task 15's switches read fault_idle_open). All in bank 2
// (faults.cpp).
#if defined(FEATURE_MAP_UNIT)

// The checks the P28 knows, by id: the image's code table has an entry for each, in this order (its code 0
// where the car has none: the 944's MAP, knock line and overboost cut), and the log's bits are theirs.
enum : uint32_t
{
    FAULT_SUPPLY,     // 1111 the supply voltage
    FAULT_IDLE_SW,    // 1112 the idle contact
    FAULT_FULL_SW,    // 1113 the full-load signal (the KLR's)
    FAULT_COOLANT,    // 1114 the coolant sensor (NTC II)
    FAULT_AFM,        // 1121 the air flow sensor
    FAULT_IDLE,       // 1122 the idle speed control
    FAULT_MIXTURE,    // 1123 the mixture (lambda's integrator, the trims, at a limit)
    FAULT_OXYGEN,     // 1124 the oxygen sensor
    FAULT_AIR_TEMP,   // 1125 the intake air sensor (NTC I)
    FAULT_KNOCK,      // 1131 the knock line (the 951's)
    FAULT_MAP,        // 1132 the MAP sensor (the 951's)
    FAULT_BOOST,      // 1137 the overboost cut (the 951's)
    FAULT_DME,        // 1141 the DME: the 8051 restarted running, or stopped by the heartbeat
    FAULT_REFERENCE,  // 1151 the reference sensor
    FAULT_SPEED,      // 1152 the speed sensor
    FAULT_CRANK,      // 1153 the crank count
    FAULT_EVENTS,     // 1161 the P28's events (L1)
    FAULT_TICK,       // 1162 the P28's tick (L2)
    FAULT_STORE,      // 1163 the P28's store
    FAULT_CHECKS,
};

// Format 33 (the actions' second step: lambda and the trims): the format whose faults act on them, the codes under
// which the trims learn nothing (1112, 1113, 1114, 1121, 1125) and those under which they learn only toward rich
// (1123, 1124), a bit by id as the faults present now have them (own value 99h: ids 0 to 15).
static constexpr uint32_t FAULTS_LAMBDA_FORMAT = 33;
static constexpr uint32_t FAULTS_NO_LEARNING = 1u << FAULT_IDLE_SW | 1u << FAULT_FULL_SW | 1u << FAULT_COOLANT
                                               | 1u << FAULT_AFM | 1u << FAULT_AIR_TEMP;
static constexpr uint32_t FAULTS_RICH_LEARNING = 1u << FAULT_MIXTURE | 1u << FAULT_OXYGEN;

// Format 33: lambda's loop held open this drive, its integrator having stood at 0 (the sensor reading rich) for
// 1123's confirm time by an up/down count of its own (1123's fuel-off limit confirmed); closed again at each drive's
// start (the 8051's) and at a clear, so that the check tests it again. L2's: faults_tick sets it, task 16 (lambda,
// unit_tasks.cpp) reads it in the next tick's job. The fuel-on limit keeps the loop.
extern volatile uint8_t fault_loop_open;

// Format 35 (the actions' fourth step): the NTCs' substitutes. A code past the checks' limits (Cal_FaultNtcOpen,
// Cal_FaultNtcShort) never reaches the control: the bus level (map_unit_adc) makes the byte the snoop gives the
// control from the substitute instead, at once; L2 makes the substitutes each tick, as the control's bytes (the
// linearisation's: C = (byte - 58.6) / 1.49). The coolant's: its last plausible byte this power-up, else the intake
// air's first this power-up, else Cal_FaultSubCold; once 1114 is present, Cal_FaultSubRise more a tick while the
// engine runs, to Cal_FaultSubWarm (a base already over it kept). The intake air's: its last plausible byte, else
// Cal_FaultSubCold. The checks read the raw codes.
static constexpr uint32_t FAULTS_SUBSTITUTE_FORMAT = 35;
struct FaultSubstitutes
{
    uint8_t on;              // a format 35 image, the faults on
    uint8_t open, shorted;   // Cal_FaultNtcOpen, Cal_FaultNtcShort (a code at or over, at or under)
    uint8_t air, coolant;    // the substitutes for IN2 (the intake air) and IN3 (the coolant)
    uint8_t air_at, coolant_at; // their bytes in the RAM's copy (AirTemp, Coolant)
};
extern volatile FaultSubstitutes fault_substitutes;

// Format 36 (the actions' fifth step): 1112's check looks above the overrun's cut-off speed (Overrun_CutSpeed by
// coolant, as task 11's cut takes it) at the contact's raw state (IN6 at both switches' level or the idle
// contact's), not the IdleSwitch bit; while 1112 stands and the engine runs above that speed, the contact is taken
// open: L2 sets fault_idle_open each tick, and task 15's switches (tick_switches) clear IdleSwitch by it in the next
// tick's job, so no overrun cut while driving, the main maps off idle and the knock retard applied; below the
// cut-off, at idle, the contact as it reads.
static constexpr uint32_t FAULTS_IDLE_FORMAT = 36;
extern volatile uint8_t fault_idle_open;

// Where the checks look in the tick's copy and what the image gives them (pair.cpp's, as the board starts): each
// bit its byte's place << 8 | its mask, each byte its place, the constants.
struct FaultPlaces
{
    uint32_t trim_most, trim_steady_min;                // Cal_TrimMost, Cal_TrimSteadyTicks
    uint32_t rev_limit, rpm_max, load_max, map_max;     // Cal_RevLimit (rpm); Cal_LamRpmMax, Cal_LamLoadMax and
                                                        //   Cal_LamMapMax (the closed loop's limits)
    uint32_t lam_most, idle_most, idle_least;           // Cal_LamIntMax; the valve's integrator's limits (Main_Idle's)
    uint16_t start, idle, full, closed, mid_band, fuel_cut, boost_cut; // StartMode, IdleSwitch, FullLoadSwitch,
                                                        //   ClosedLoop, O2MidBand, FuelCut, BoostCut
    uint8_t pins, coolant, lam_int, idle_int, in5;      // the pins' byte (O2_LEAN its bit 1), Coolant, LamInt,
                                                        //   IdleIntHi, In5
    uint8_t air;                                        // AirTemp (format 35's)
    uint8_t cut_speed;                                  // Overrun_CutSpeed's id (format 36's; 0: none)
    uint8_t spare;
};

// What the checks count on each tick (pair.cpp's, each its one writer's, taken after the tick's commit).
struct FaultCounts
{
    uint32_t starts, synced;              // the 8051's starts; in step (the bus level's)
    uint32_t teeth, references, turns;    // the vector's fetches, the references, the turns
    uint32_t missing, crank_faults;       // a turn's reference missing; a reference off by more than a tooth
    uint32_t beat_stops;                  // the engine stopped by the heartbeat (the 8051's Pair_Lost)
    uint32_t event_over, event_lost;      // L1's jobs over their bound, into a full queue
    uint32_t tick_lost, tick_over, again; // L2's pings into a full queue, ticks over their bound, ticks run again
    uint32_t knock_faulty;                // the knock line marked faulty (format 24)
    uint32_t trim_factor, trim_steady;    // the trims' factor in use (8000h x 1.0), their steady ticks (format 30)
    uint32_t results;                     // the snoop's results so far, every channel's
    uint8_t adc[8];                       // the snoop's codes: each channel's last result
};

// As the board starts (pair_start, from the table at u + 194 on: the code table's, the limits', the frame's
// list's and the live copy's addresses): the work on with format 31's table; rom the image, places where the
// checks look (null: the work off), counts the counts as they stand.
void faults_start(const uint8_t *rom, uint32_t u, const FaultPlaces *places, const FaultCounts &counts);

// L2 once a tick, after the trims' work, on the tick's copy (bytes) at its ping (now, us) with the counts: the
// checks, their states, the memory, the faults present now (own values 99h and 9Ah, the log's).
void faults_tick(const uint8_t *bytes, uint32_t now, const FaultCounts &counts);

// The records whose frame is due (made this tick), a bit a slot (cleared as they are given), and a record's
// frame: its 18 bytes, to fill from the list at faults_frame_list (the log's addresses: RAM bytes, the P28's own
// from 80h) as the log's frame takes them.
uint32_t faults_frames_due();
uint8_t *faults_frame(uint32_t slot);
uint32_t faults_frame_list();

// The live copy's place in the image (0: none), for pair_live.
uint32_t faults_live();

// The store's third kind (store.h), in bank 2 with the store: its record one part, the memory's 232 bytes
// as they stand (a head, then the seven records); faults_differs whether it has changed since its last
// record by what the minute's rule writes (a fault gone, a drive counted), faults_copy its bytes, faults_restore
// the memory from the record's bytes (each check present or gone as its record says), faults_clear the memory
// empty (L2's, from the store's tick), faults_key the key of what it means (its layout, the code table's codes
// and the frame's list; 0 with the work off).
bool faults_differs(uint32_t part, const uint8_t *kept);
void faults_copy(uint32_t part, uint8_t *out);
void faults_restore(uint8_t *in);
void faults_clear();
uint32_t faults_key();

// The PC's pair_faults: ticks worked, the faults present now and pending (bits by id), confirmations, faults lost
// to a full memory.
bool faults_get_parameter(const char *name, char *value, size_t value_size);

#endif

#endif
