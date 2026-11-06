# The map unit

A build of the fork's firmware (2.0-map, with 2.0-window's comms window) in which the board looks
maps up for the DME's 8051: DME-Sim's map unit prototype (its `experiments/coproc_maps` and
`MapUnit`) on an RP2040. The 8051 keeps a byte a map; the maps, and the lookups, are the board's.

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

1. Wire the 8051's A13 (P2.5) to the socket's pin 26, and set `addr_mask` to `0x3fff`. The DME
   ties pin 26 to +5 V (and /CE to ground: 951digit.pdf), and the owner rules out changing it, so
   this build is a measurement: see Open.
2. Write an image whose second 8 KiB holds the unit's maps, index and mark (DME-Sim's
   `tools/coproc_maps/make_coproc.py` makes rom_lean's). The unit is on while the mark is there
   and A13 is served.
3. The 8051 asks by the addresses it reads, the socket having no write line:

| Read       | Asks                    | The byte read                                       |
|------------|-------------------------|-----------------------------------------------------|
| 3C00h + id | for map id              | its first input's RAM address                       |
| 3D00h + v  | v, the map's next input | the next input's RAM address, or 0                  |
| 3E00h      | for the map's value     | as rom_lean's Map_Lookup or Map_Lookup_2D, exactly  |
| 3F00h      | (the mark, not read)    | "MAPU", format 1, the maps, the index's address     |

The index holds 3 bytes a map: its address (high, low) and its axes. An answer is ready a few
microseconds after the read that asks; DME-Sim's routine reads each 4 machine cycles (8 us) or more
after.

## How it works

- `map_window` (map_unit.pio; pio1's state machine 2, comms_clock's, which only FEATURE_CLOCK builds
  load) pushes A0-A9 of each read of 3C00h-3FFFh once the read is over, so an answer never changes
  a byte the 8051 is reading. It takes the pins 26 ns after /OE falls: margin only, the DME having
  /CE grounded and the address out long before /PSEN falls.
- Its interrupt (priority 0, above USB and the timers) runs on core 0 from the unit's bank and calls
  nothing outside it. A pick fills the input page with the next answer; the last send looks the map
  up and fills the value page. A page fills 16 bytes a store, in about 105 cycles.
- The PC reads and writes the image as it wrote it: the three pages keep a copy for it (the shadow),
  and a commit writes that copy to flash.
- The main stack is 1,536 bytes in these builds; the comms window's queue moved to the unit's bank
  to make room. With 640, as in the stock builds, a parameter's answer in the USB callback with an
  interrupt on top ran past the stack into TinyUSB's state, and USB stopped; the deepest since has
  been 716 bytes.

## Parameters

| Name           | Set               | Get                                                       |
|----------------|-------------------|-----------------------------------------------------------|
| map_unit       |                   | off, or the maps and the lookups made                     |
| map_lookup     | id,first[,second] | the value and its cycles                                  |
| map_bench      | id                | every input: the values' CRC-32, cycles least, mean, most |
| map_bus        | n                 | the bus bench: n lookups, then done, wrong, late, lost    |
| map_bus_cycles |                   | its times from each read's end: mean and most             |
| map_cycles     |                   | the interrupt's own times since start (in a car too)      |
| map_probe      | address           | one read: the pins, map_window's place, reads answered    |
| map_stack      |                   | how much of the main stack has never been used            |

The bus bench and the probe make reads on the board itself, through the GPIO input overrides: no pin
is driven and the data buffer is held off, but the board serves no 8051 meanwhile. Core 1 plays the
8051, reading as coproc_maps' routine does: each read 465 ns long, one every 8 us. Times are in
cycles of the 270 MHz clock.

## Measured

A POG (RP2040 at 270 MHz) with coproc_maps' image for the 951 (42 maps), 2026-11-06, by DME-Sim's
`tools/coproc_maps/BoardCheck`:

- Every map at every input (467,712 lookups) gives MapUnit.Lookup's value, the 944's image's too.
- A lookup takes 134 cycles (0.50 us) on one axis, 154 at most; 243 (0.90 us) on two, 293 (1.09 us)
  at most.
- The bus bench, 200,000 lookups (633,000 reads of the window) with USB quiet, then as many with the
  PC reading the image throughout: none wrong, none late, none lost.

| From the end of the 8051's read to the answer | Mean    | Most, USB quiet | Most, USB busy |
|-----------------------------------------------|---------|-----------------|----------------|
| A pick's or a send's (the input page)         | 0.67 us | 2.23 us         | 2.58 us        |
| The lookup's (the value page)                 | 1.46 us | 3.24 us         | 3.29 us        |

The 8051 reads the next answer 7.5 us after a read ends. The interrupt itself takes 0.41 us (0.76
at most) for a pick and 1.20 us (1.73) for a lookup; the rest is reaching it while core 0 finishes
what it was doing.

## Open

- No A13 wire (owner, 2026-11-06: no change to the DME). The window goes inside the 8 KiB, as the
  comms window does, and the maps into the board's own memory, where only the unit reads them.
- Run it on a P28 in the DME.
- A commit with the unit on (the shadow to flash) hasn't been tried on a board.
- The bus bench leaves the ROM's bank unread; in a car core 1 reads it every 7 cycles, slowing the
  lookup's reads of the maps a little. map_bench runs with core 1 serving, so its times are a car's.
- `picorom()` takes one define after the config: the 100ns builds have never had
  FEATURE_STABLE_ADDRESS (upstream's too). The map builds pass theirs as one list.
- The stock builds' 640-byte stack may be short as well: by the compiler's figures, a parameter's
  snprintf in the USB callback is about 540 bytes deep before an interrupt comes on top.
