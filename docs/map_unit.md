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

## How it works

- `map_window` (map_unit.pio; pio1's state machine 2, comms_clock's, which only FEATURE_CLOCK builds
  load) pushes A0-A7 of each read of the window's page once the read is over, so an answer never
  changes a byte the 8051 is reading, shifted up a bit; and of each read of page 0, with a 1 below:
  0000h is the 8051's start, the rest go to the snoop (its vectors). It runs from start-up; while
  the unit is off its y holds no page. It takes the pins 26 ns after /OE falls: margin only, the
  DME having /CE grounded and the address out long before /PSEN falls.
- Its interrupt (priority 0, above USB and the timers) runs on core 0 from the unit's bank and calls
  nothing outside it. A pick or a send fills the page with the next answer, the last send looks the
  map up and fills the page with its value, and the value's read (or a start) copies the page as
  written back. A fill takes about 105 cycles; the interrupt's own times are under Measured.
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
lookup in 64 broken off after its pick (and, one time in two, an input) by a read of 0000h. Times
are in cycles of the 270 MHz clock. An answer is 35 characters at most (a packet's payload).

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

## Open

- Run it on a P28 in the DME.
- A commit with the unit on (the shadow to flash) hasn't been tried on a board.
- The bus bench leaves the ROM's bank unread; in a car core 1 reads it every 7 cycles (with the bus
  priority), slowing the unit's fills and copies of the page a little. map_bench runs with core 1
  serving, so its times are a car's.
- `picorom()` takes one define after the config: the 100ns builds have never had
  FEATURE_STABLE_ADDRESS (upstream's too). The map builds pass theirs as one list.
- The stock builds' 640-byte stack may be short as well: by the compiler's figures, a parameter's
  snprintf in the USB callback is about 540 bytes deep before an interrupt comes on top.
- `build_time` is set when CMake configures, so a rebuild keeps the time before.
