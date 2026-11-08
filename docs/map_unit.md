# The map unit

A build of the fork's firmware (2.0-map, with 2.0-window's comms window) in which the board looks
maps up for the DME's 8051: DME-Sim's map unit prototype (its `experiments/coproc_maps` and
`MapUnit`) on an RP2040. The 8051 keeps a byte a map; the maps, and the lookups, are the board's.
Nothing on the DME changes: the 8051 asks through one page of its 8 KiB. The builds hold the snoop
too (`docs/snoop.md`): what the board learns of the 8051 from the socket alone.

## Builds

| Build               | Board                   | Serves  | Note                         |
|---------------------|-------------------------|---------|------------------------------|
| P28_27C512_70ns_MAP | P28, in the DME socket  | 64 KiB  | Built, not yet run on a P28  |
| POG_27C020_70ns_MAP | POG, 32 pins            | 192 KiB | To try it on a 32-pin board  |

The unit's code and data live in the last 64 KiB SRAM bank, so the POG build serves three. Both
keep the image where the stock builds keep it in flash: a board keeps its image and settings when
its build changes. `firmware/load_uf2.ps1` loads a build on Windows with no driver to install: it
sends the board to its boot mode through its reset interface, as picotool does, and copies the UF2
to the RPI-RP2 drive.

```
powershell -ExecutionPolicy Bypass -File firmware/load_uf2.ps1 -Board dme `
    -Uf2 build/PicoROM-POG_27C020_70ns_MAP-2_0-map.uf2
```

## Using it

1. Write an image whose first 8 KiB is the 8051's and whose second holds the unit's mark, index and
   maps (DME-Sim's `tools/coproc_maps/make_coproc.py` makes rom_lean's), and set `addr_mask` to
   `0x1fff`, as for any 2764 image: the DME ties the socket's A13-A15 high (951digit.pdf), so the
   8051 reads only the first 8 KiB and the rest is the board's own. The unit is on while the mark
   is there and the mask is 0x1fff.
2. The 8051 asks by the addresses it reads in the window, a page of its 8 KiB that the mark names,
   the socket having no write line. The unit fills the page with each answer:

| Read          | Asks                         | The byte read                                  |
|---------------|------------------------------|------------------------------------------------|
| window + id   | for map id                   | the page as written: its first input's address |
| window + v    | v, the map's next input      | the next input's RAM address, or 0             |
| window + any  | after the last: the value    | as rom_lean's Map_Lookup or Map_Lookup_2D      |
| 0000h         | nothing: the 8051's start    | the image's (the page as written again after)  |

After the value the page is as written again, and the unit waits for a pick. The mark, at 2000h:
"MAPU", format 2, the window's page (1 to 1Fh), the number of maps and the index's address, high
byte first. The index holds 3 bytes a map: its address (high, low) and its axes. An answer is
ready a few microseconds after the read that asks; DME-Sim's routine reads each 4 machine cycles
(8 us) or more after.

### Tasks (format 3)

Format 3 adds tasks: work the board does for the 8051, rom_lean's main loop a stage at a time
(DME-Sim's `experiments/coproc_slow` to `coproc_load`; `unit_tasks.cpp`). A task's index entry is
its description's address and 80h + its number. The 8051 picks it and sends its inputs as a map's.
After the last input, each read gives what comes next:

| After                    | The byte read                                        |
|--------------------------|------------------------------------------------------|
| the last input           | the RAM address of the task's first output (0: none) |
| an output's address      | that output's value                                  |
| an output's value        | the next output's address, or 0                      |
| 0                        | (the page is as written again)                       |

A description is the input count and the inputs' RAM addresses, the output count and the outputs'
addresses, then the task's data: the maps it looks up (by id), the tables and constants it reads in
the 8051's 8 KiB, the 8051 bits it tests. The board keeps the 8051's RAM bytes sent, the task reads
them as rom_lean's routine reads its RAM, and puts its outputs there. DME-Sim checks each task
against rom_lean's routine at every input, running this firmware's own unit bank.

| Task | rom_lean's routine | Outputs                             |
|------|--------------------|-------------------------------------|
| 0    | Slow_WarmUp        | WarmUp                              |
| 1    | Slow_Fuel          | FuelFactorL, FuelFactorH, FuelShift |
| 2    | Main_Ign           | IgnSlow, MapGainNow                 |
| 3    | Slow_Charge's sum  | ChargeL, ChargeH                    |
| 4    | Ign_Advance        | Handover: the maps summed           |
| 5    | Ign_Offset's start | Handover: the target                |
| 6    | Fuel_Time          | InjTimeH:L; the dead time           |
| 7    | Load_Tick_Run      | LoadHi, LoadMid: the load, filtered |
| 8    | (format 6)         | none: pair.cpp takes the advance    |
| 9    | Fuel_Maps to       | InjTimeH:L, DeadNow, its flags;     |
|      | Fuel_Mode (fmt 7)  | TH0's load in 8051 code             |
| 10   | the same, again    | the same (none after its reference) |
| 11   | Ign_Offset to      | Advance, its flags (the cuts the    |
|      | Ign_Slew (fmt 8)   | 8051's; counts, flags its own)      |
| 12   | Main_Tick's start  | StartFlags; BoostCount its own      |
|      | mode (fmt 8)       |                                     |

### Inputs the snoop makes (format 4)

Format 4 (DME-Sim's `experiments/coproc_adc`) lets the board make a task's ADC bytes itself, from
the reads its snoop sees (`docs/snoop.md`), so the 8051 doesn't send them. The mark's ninth and tenth
bytes are the address of a table: a count, then for each byte its RAM address, its ADC channel, the
way and a map's id. The ways are rom_lean's: 0 the channel's last result (the battery, the fuel
quality), 1 its last two averaged as Ring_Average (the AFM, the MAP sensor), 2 its last complemented,
through the map (Ntc_Linearise: the temperatures). A description's input with 80h set is one of
those, its RAM address below: the unit names only the others to the 8051 (Co_Task sends one at least,
so a task keeps one sent), and copies it in before the task. The unit makes each byte as its
channel's result comes (map_unit_adc, from the snoop's interrupt), so a task's answer waits for no
map lookup. The board's bytes are its latest: the 8051's AFM and MAP bytes were the tick's, its
temperatures the slow work's.

### The paired core (formats 6 to 8)

Format 6 (DME-Sim's `experiments/rom_pair`, its docs/Paired_Core.md; `pair.cpp`) makes the board
keep the crank for an 8051 that keeps no tooth chain. Every tooth's interrupt fetches INT1's vector
(0013h), and the board writes that vector before each tooth: a RETI for a plain tooth, or an AJMP
to the event of the tooth to come (an ADC read, the KLR strobe, the coil's charge, the spark's arm).
It counts the teeth by the vector's fetches, takes the latch's tooth at each reference by the
teeth's times, times each firing at slot 0's tooth, and plans the coil after each spark with
rom_lean's arithmetic: Spark_Plan, Dwell_Plan, Spark_Load, Period_T64, bit for bit. It writes the
arm's TL0 load into the arm's handler a tooth before it.

The mark's 13th and 14th bytes are the address of its table (Co_Pair):
- the teeth a firing and the firings a turn;
- the crank events: each its tooth in its firing and its handler;
- the coil's handlers;
- the live bytes: the arm's two operands, and the opcodes of NotSynced's and ToothFault's at the
  8051's reference;
- page 0's marks: the ping (the heartbeat's operand after it), the count lost, the spark;
- the coil's numbers (SPARK_B, ARM_OFS_S, ARM_MIN_S, MARGIN, FLOOR_TEETH, RECIP32) and the
  latch's (TEETH_HIGH, and whether fall 1 is the latch's tooth or the one after);
- the RAM bytes it takes from the unit: the charge's time (task 3's outputs) and the advance
  (task 8's input).

Of the 8 KiB the board writes only the window's page and these live bytes, so that the 8051 never
fetches a torn instruction:
- A RETI is one byte.
- An AJMP goes operand first, then opcode.
- The handlers are in one page, so one event follows another by the operand alone.

Each ping moves the heartbeat on, and a ping that stands still for 45 ms stops the engine (the
8051's). `pair_test` runs the ported routines for DME-Sim's checks.

Format 7 (step 3.2) adds the pulse: tasks 9 and 10 work out rom_lean's pipeline stages 5 to 7
(Fuel_Maps or Fuel_Crank, Fuel_Time and Fuel_Mode) bit for bit, the turn's pulse and the pulse made
again before its reference. The task's TH0 load (its start and its overflows) goes into the 8051's
code: the table adds two places the 8051 takes a load from (at a pulse's end, and while TH0 is
stopped), each an AJMP whose operand is a switch, and its two copies of `MOV TH0,#start; MOV
InjHi,#overflows`. The board writes the copies not in use, then turns each switch (one byte), so the
8051 loads a whole pair, the old or the new. A pulse made again after a reference is not written.
The task takes up to about 11 us, so the 8051 reads its first output 34 us after its last input.

Format 8 (step 3.3) adds the advance's rest, start mode and the injection's phase:
- Task 11 works out rom_lean's pipeline stage 4 (Ign_Offset to Ign_Slew: the target, the overboost
  guard, the overrun cut and its resume, the slew), and task 12 the tick's start mode with its
  cold-start variants and the overboost guard's periods, bit for bit.
- The tasks keep their own state in the board's copy of the 8051's RAM, never sent or taken back:
  the counts (SlewCount, BoostCount, InjCount), the advance's target, and the flags only they read.
  Task 11 takes 3 bytes and gives 2. The 8051 tells the board where it moved those counts, by three
  more page-0 marks (each a RET): the engine stopped (the count lost too; InjCount 0, CrankDecay
  cleared), a firing (SlewCount one less) and an injection counted (InjCount one more, to 80h). The
  8051's start clears the copy.
- The board owns the 720-degree phase. At each reference it turns the next one's and writes the
  reference's AJMP's operand (0004h: to the injection's handler or the one that leaves it out) and
  the opcode of the 8051's copy. At slot 0's tooth of the turn's first firing it decides the turn
  (starting, its pulse; a turn the phase leaves out, none; over Cal_RevLimit or in a cut, a cut;
  else its pulse) and writes Main_Turn's SJMP's offset.
- The table adds the reference's two handlers, the SJMP and its three targets, the copy's opcode,
  what the cut reads (Cal_RevLimit, the speed byte, StartMode, FuelCut, BoostCut), then the marks
  and the places they change.

## How it works

- `map_window` (map_unit.pio; pio1's state machine 2, comms_clock's, which only FEATURE_CLOCK builds
  load) pushes A0-A7 of each read of the window's page once the read is over, so an answer never
  changes a byte the 8051 is reading, shifted up a bit; and of each read of page 0, with a 1 below:
  0000h is the 8051's start, the rest go to the snoop (its vectors). It runs from start-up; while
  the unit is off its y holds no page. It takes the pins 26 ns after /OE falls: margin only, the
  DME having /CE grounded and the address out long before /PSEN falls.
- Its interrupt (priority 0, above USB and the timers) runs on core 0 from the unit's bank and calls
  nothing outside it (its files are built with no loop patterns and no jump tables, whose helpers
  are in flash). A pick or a send fills the page with the next answer, the last send looks the map
  up (or works the task out) and fills the page with its value, and the value's read (or a start)
  copies the page as written back. A fill takes about 105 cycles; the interrupt's own times are
  under Measured.
- The PC reads and writes the image as it wrote it: the page is kept for it (the shadow), and a
  commit writes that. While the unit is on, the PC's bytes for the page reach the 8051 once the page
  is as written again (after the next lookup, or the 8051's start).
- The main stack is 1,536 bytes in these builds; the comms window's queue moved to the unit's bank
  to make room. With 640, as in the stock builds, a parameter's answer in the USB callback with an
  interrupt on top ran past the stack into TinyUSB's state, and USB stopped; the deepest since has
  been 716 bytes.

## Parameters

| Name           | Set               | Get                                                       |
|----------------|-------------------|-----------------------------------------------------------|
| map_unit       |                   | off, or the maps and the window                           |
| map_counts     |                   | the lookups made and the starts seen                      |
| map_lookup     | id,first[,second] | the value and its cycles                                  |
| map_bench      | id                | every input: the values' CRC-32, cycles least, mean, most |
| map_task       | id                | a task's cases (core 1): running, then CRC-32 and cycles  |
| map_bus        | n                 | the bus bench: n lookups, then done, wrong, late, lost    |
| map_bus_cycles |                   | its times from each read's end, by kind: mean and most    |
| map_cycles     |                   | the interrupt's own times by kind since start             |
| map_probe      | address           | one read: the pins, map_window's place, reads answered    |
| map_stack      |                   | how much of the main stack has never been used            |

The kinds: a pick or a send before the last; the last send (the lookup); the page as written again
(the value's read, or a start). The bus bench and the probe make reads on the board itself, through
the GPIO input overrides (`bus_bench.cpp`, shared with the snoop's): no pin is driven and the data
buffer is held off, but the board serves no 8051 meanwhile. Core 1 plays the 8051, reading as
coproc_maps' routine does: each read 465 ns long, one every 8 us, 20 us between lookups, and one
lookup in 64 broken off after its pick (and, one time in two, an input) by a read of 0000h. A task
(format 3) it asks as coproc_slow's Co_Task does, its inputs at random: its first output read 12 us
after its last input, each output checked against the task worked out on core 1. map_task takes a
task's cases as DME-Sim's UnitTasks.Cases does: every value of each input byte (a byte of bits 00h
and FFh), then each of those bits alone over the first other input's range; beyond 2^18 of those,
2^18 at random instead: each input in turn from xorshift32's next (from 2545F491h), its low byte,
or with bit 8 set one of the edges 00h, 01h, 40h, 7Fh, 80h, 81h, FEh and FFh by bits 9-11, so that
a value a path turns on (Fuel_Time's Handover 80h) comes often. Times are in cycles of the 270 MHz
clock. An answer is 35 characters at most (a packet's payload).

## Measured

A POG (RP2040 at 270 MHz) with coproc_maps' images, 2026-11-06, by DME-Sim's
`tools/coproc_maps/BoardCheck`:

- Every map at every input (467,712 lookups an image) gives MapUnit.Lookup's value, both cars.
- A lookup takes 127 cycles (0.47 us) on one axis, 147 at most; 238 (0.88 us) on two, 287 (1.06
  us) at most.
- The bus bench, the 951's image, with the snoop running: 200,000 lookups (and 3,125 broken off by a
  start) with USB quiet, then as many with the PC reading the image throughout: none wrong, none
  late, none lost.

| From the end of the 8051's read to the answer | Mean    | Most, USB quiet | Most, USB busy |
|-----------------------------------------------|---------|-----------------|----------------|
| A pick's, or a send's before the last         | 1.01 us | 5.60 us         | 2.21 us        |
| The last send's (the lookup)                  | 1.47 us | 3.07 us         | 2.62 us        |
| The page as written again (value, start)      | 1.23 us | 5.73 us         | 2.93 us        |

The 8051 reads 7.5 us after a pick or a send ends, and 19.5 us or more after the value. The
interrupt itself takes 0.76 us (0.81 at most) for a pick or a send, 1.23 us (1.77) for the lookup
and 0.96 us to put the page back; the rest is the time to reach it (core 0 finishing what it was
doing) and to see the answer (the bench's core 1 polls for it). The most is a few runs' outliers
(the same build has given 2.2 to 5.6 us); before the snoop, 4.2 to 4.4 us. The unit saw one start
more than the benches made in one run: with no DME on its pins the board's inputs float, and a read
of 0000h can appear outside the benches.

The tasks (e7406ca, coproc_slow's 951 image, 2026-11-06): each task's outputs at each of its cases
(map_task) have the CRC-32 of rom_lean's routine's. A task takes 265 to 308 cycles (Slow_WarmUp), 420
to 552 (Slow_Fuel), 628 to 726 (Main_Ign) and 311 to 350 (Slow_Charge). On the bus bench, with the
tasks among the maps: 200,000 requests, USB quiet and busy, none wrong, late or lost; the last send's
answer (a lookup's or a task's) 1.65 us on average, 4.4 us at most, and the interrupt itself 3.66 us at
most (a task's 12 us before the 8051 reads its first output).

The advance's tasks too (coproc_advance's 951 image, 2026-11-06): each of the six tasks' outputs at
its cases (task 4's 2^18 at random) have the CRC-32 of rom_lean's routine's. Task 4 takes 413 to 579
cycles, task 5 312 to 353. The bus bench, all six tasks among the maps: 200,000 requests, none wrong,
late or lost; the answers' times as before. (A first run counted task 5's outputs wrong: its output
is its input's byte, and the bench sent the byte it had worked the output into; it sends the inputs
it chose now.)

The pulse's time too (f9356d6, coproc_time's 951 image): all seven tasks have rom_lean's CRCs (the
random cases now with edge values), task 6 taking 453 to 883 cycles. The bus bench, all seven tasks
among the maps: 200,000 requests, none wrong, late or lost; the last send's answer 5.7 us at most.

The load too (31bb5ba, coproc_load's 951 image): all eight tasks have rom_lean's CRCs, task 7 taking
310 to 393 cycles. The bus bench, all eight tasks among the maps: 200,000 requests, none wrong, late
or lost; the last send's answer 5.8 us at most. A pick's or a send's answer took 6.6 us once with
USB busy (the 8051 reads 7.5 us after), the interrupt itself 0.82 us of it at most.

## Open

- Run it on a P28 in the DME.
- The tasks on a P28 in a DME (run on a POG's benches only).
- A commit with the unit on (the shadow to flash) hasn't been tried on a board.
- The bus bench leaves the ROM's bank unread; in a car core 1 reads it every 7 cycles (with the bus
  priority), slowing the unit's fills and copies of the page a little. map_bench runs with core 1
  serving, so its times are a car's.
- `picorom()` takes one define after the config: the 100ns builds have never had
  FEATURE_STABLE_ADDRESS (upstream's too). The map builds pass theirs as one list.
- The stock builds' 640-byte stack may be short as well: by the compiler's figures, a parameter's
  snprintf in the USB callback is about 540 bytes deep before an interrupt comes on top.
- `build_time` is set when CMake configures, so a rebuild keeps the time before.
