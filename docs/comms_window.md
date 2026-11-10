# The comms window

Firmware 2.0's comms session makes a 1 KiB block of the ROM the board's, its registers written into
it. The window is a session for a program that only sends: nothing is written into the ROM, and
each byte costs the program one read.

## Using it

1. Set `comms_mode` to `window`, with no session open. It isn't kept in flash, so a board starts
   with `block`, 2.0's session. A 2.0 board answers `ParameterError`: start no session on it, since
   its CommsStart writes its registers into the ROM the target is running.
2. CommsStart with an address. The window is the 256-byte page that holds it.
3. Each read of the page (/OE and /CE low) sends the read's low byte, once a read, as CommsData:
   24 bytes a packet at most, or fewer once a millisecond. A program sends the byte b by reading
   the page's first address plus b; with the page holding 00h to FFh, the read gives b back.
4. CommsEnd ends the session, and so does USB going (unmounted). With the PC not reading, the bytes
   past a 1 KiB queue are dropped, and `comms_dropped` counts them: nothing waits on the PC.

`comms_modes` (read only) lists the modes a build has, so the PC can choose without trying them:
`block window` here, `block window log` with the map unit. A 2.0 board answers ParameterError:
`block` only.

## Notes

- The address lines `addr_mask` leaves out read 0, so the page is matched within the ROM as served.
- Any read of the page is a byte, an instruction's prefetch too: on an 8051, no instruction may end
  on the byte before the page.
- A CommsData packet leaves the 64-byte USB buffer room for the answer to a request: that answer
  is sent inside TinyUSB's callback, where it can't wait for the buffer to empty.
- `comms_window` (comms.pio) reads /OE and /CE from `BASE_OE_PIN`, as `report_data_access` does,
  and its two counts that depend on the board's pins are set as it loads (pio_programs.cpp). It
  shares pio0's state machine 2 with `comms_detect`, and with it fills pio0's 32 instructions.

## Trying it

Any ROM will do: a window on a page the target reads (a table, say) sends the offsets it reads
there. With `pypicorom`:

```python
import pypicorom, time

pico = pypicorom.open("name")
print(pico.set_parameter("comms_mode", "window"))   # ParameterError: not this build
pico.start_comms(0x1700)                            # the page 1700h-17FFh
end = time.time() + 5
while time.time() < end:
    data = pico.read()
    if data:
        print(bytes(data).hex(" "))
pico.end_comms()
print("dropped", pico.get_parameter("comms_dropped"))
```

Motronic Link (`PicoRomModel` in its Formats project) models the window exactly, for its tests and
its DME simulator.
