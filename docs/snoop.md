# The snoop

Part of the map unit's builds (2.0-map; `docs/map_unit.md`): the board learns from the socket alone
what the DME's 8051 reads and when, as DME-Sim's model of it predicts (DME-Sim's
`docs/Socket_View.md`, `P28Snoop`). It finds each MOVX (an ADC read's channel and result, a write's
address and byte) and each interrupt the 8051 takes. The 8051 sends the board nothing for it.

## How it works

- **A MOVX** is the only instruction that leaves /PSEN high for longer than a read's gap: the next
  read comes 2.5 us after the read before ends, where reads otherwise come 1 us apart. The `snoop`
  program (map_unit.pio; pio1's state machine 1, report_data_access's) times /OE high after each
  read; past 1 us it takes all the pins at 1.5 us (A0-A12 the data address, D0-D7 the byte) and
  pushes them with the pins of the read before. That read was of the byte after the MOVX's opcode:
  the image says read (E0h, E2h, E3h) or write (F0h, F2h, F3h). Every read is the ADC's: its OE,
  START and ALE are all NOT /RD (DME-Sim's docs/Hardware.md). A8-A10 is the channel the read starts,
  and its byte is the result of the conversion the read before started, so `snoop_adc` files it
  under that read's channel.
- **An interrupt**: `map_window` pushes every read of page 0 (tagged odd; the window's are even),
  and the map unit's interrupt hands them to the snoop. They are most of its words (about eight a
  tooth), so they take a short way through it; only 0000h's is the unit's as well. A vector's read
  followed by the byte after it is its routine starting; a RETI's or a jump's dropped reads of a
  vector aren't followed by that. Code that runs through an unused vector's address is (rom_lean's
  injection routine runs on through 0023h), so only the vectors set in `snoop_vectors` count.
  `map_window` runs from start-up, the unit on or off.
- The snoop's interrupt (priority 40h: above USB and the timers, below the map unit's) and its data
  live in the unit's bank. It takes report_data_access's place, so the activity LED follows its
  pushes.

## Parameters

| Name             | Set                | Get                                                      |
|------------------|--------------------|----------------------------------------------------------|
| snoop            |                    | MOVX reads and writes found, gaps with no MOVX           |
| snoop_adc        |                    | each ADC channel's last result, hex, channel 0 first     |
| snoop_last       |                    | the last MOVX: read or write, its address and byte       |
| snoop_irq        | k                  | vector 0003h + 8k: routines begun, when the last began   |
| snoop_vectors    | mask               | the vectors watched, bit k for 0003h + 8k (all: 31)      |
| snoop_window     | on,off             | the bench's byte on the data pins, ns after P            |
| snoop_bench      | n,read_at,write_at | n rounds; then done, wrong, lost, irq (rounds)           |
| snoop_bench_data |                    | where the bench's edges landed (cycles), a write, a lag  |

## The bench

Core 1 plays the 8051 through the GPIO input overrides (`bus_bench.cpp`, shared with the map unit's
bench), the data pins' too, a read every 1 us as the 8051 makes them. A round holds a MOVX read at
`read_at` (rom_lean's ADC read; the channel the round's number), a MOVX write at `write_at`, INT1's
routine (its LCALL's reads, then the shared RETI's dropped reads of 0013h) and INT0's (an AJMP at
0009h whose dropped reads are of 000Bh, Timer 0's vector). In a MOVX's gap the byte's bit 0 is the
other way except from P + on to P + off (P: the read before ends), so a sample outside reads wrong.
The snoop must find both MOVX with their addresses and bytes, both routines and nothing else.

A pin's control takes 111 ns to write here, so the bench writes only the pins that change, takes P
where /OE reads back high, asks for each edge its own lag (378 ns) early, and records where it
landed. Its code runs from the unit's bank: from flash, core 1 waited on the XIP cache, which core
0 shares, for microseconds at a time. Its own gaps (a jump's many pins) show as gaps with no MOVX.

## Measured

A POG (RP2040 at 270 MHz), 2026-11-06, by DME-Sim's `tools/coproc_maps/BoardCheck --snoop 20000`:

- 20,000 rounds with the byte where the bus puts it (it landed 1,326 to 1,356 ns and 1,778 to 1,815
  ns after P): every MOVX found with its address and byte, every routine found once, none false.
- Where the byte is taken, the window's edges moved 25 ns at a time (200 rounds each):

| The byte's edge landed, ns after P | Rounds right |
|------------------------------------|--------------|
| arriving by 1,489 to 1,519         | 200          |
| arriving 1,515 to 1,544            | 73           |
| leaving by 1,493                   | 0            |
| leaving 1,519 to 1,567             | 90           |

So the board takes the byte about 1,525 ns after the read before ends: the program's 408 cycles
and the input synchroniser's two. At the bus's tolerances a read's byte is on P0 from 1,360 to 1,800
ns at least (DME-Sim's docs/Socket_View.md), 165 ns and 275 ns either side.

## Open

- **The byte needs the P28's data buffer turned towards the RP2040** for the sample. The DME's ADC
  drives P0; the P28's data GPIOs are outputs throughout (PIO0's data_output) and its buffer faces
  the socket (DIR, GPIO27, a plain output set once). For the sample the data GPIOs must be inputs
  (pindirs from PIO0), DIR turned while the buffer is off, and the buffer on (BUF_OE, which
  or_output_enable drives from PIO1 every cycle). That spans both PIO blocks, and pio1 is full
  (or_output_enable 4, snoop 11, map_window 16, write_tca_bits 1). Without it a P28 finds every
  MOVX, its address and every interrupt, not the byte. The POG's buffer has no DIR: its bench only.
- Run on a P28 in a bench DME, with a scope on /PSEN, /RD and P0.
