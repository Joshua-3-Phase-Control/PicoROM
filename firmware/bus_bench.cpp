// The bus bench (bus_bench.h): core 1 plays the 8051 through the GPIO input overrides.

#include "bus_bench.h"

#if defined(FEATURE_MAP_UNIT)

#include "hardware/gpio.h"
#include "hardware/structs/io_bank0.h"
#include "hardware/sync.h"
#include "pico/multicore.h"

#include "rom.h"
#include "unit_ram.h"

static_assert(N_OE_PINS == 2, "the bus bench reads with /OE and /CE");

static constexpr uint32_t INOVER_LOW = 2u << IO_BANK0_GPIO0_CTRL_INOVER_LSB;
static constexpr uint32_t INOVER_HIGH = 3u << IO_BANK0_GPIO0_CTRL_INOVER_LSB;
static constexpr uint32_t OE = N_ADDR_PINS, CE = N_ADDR_PINS + 1, DATA = N_ADDR_PINS + N_OE_PINS;
static constexpr uint32_t PINS = DATA + N_DATA_PINS; // the address, /OE and /CE, then the data pins

enum : uint32_t
{
    IDLE,
    RUNNING,
    FINISHED // core 1 has finished; the main loop gives the board back
};

static volatile uint32_t state UNIT_DATA = IDLE;
static uint32_t core1_stack[128] UNIT_DATA = {};
static uint32_t ctrl[PINS] UNIT_DATA = {};  // the pins' CTRL with no input override
static uint32_t saved[PINS] UNIT_DATA = {}; // as they were (addr_mask's overrides, the data pins')
static uint32_t shown_address UNIT_DATA = 0, shown_data UNIT_DATA = 0; // what the pins read now

static uint32_t pin(uint32_t i)
{
    return i < OE ? BASE_ADDR_PIN + i : i < DATA ? BASE_OE_PIN + i - OE : BASE_DATA_PIN + i - DATA;
}

static __force_inline uint32_t level(uint32_t bits, uint32_t i)
{
    return (bits >> i) & 1 ? INOVER_HIGH : INOVER_LOW;
}

// Only the pins that change are written: a register write takes several cycles, and the 8051's reads
// come 1 us apart. These run from the unit's bank: from flash, core 1 waits on the XIP cache, which
// core 0 shares, for microseconds at a time.
void UNIT_CODE(bench_address)(uint32_t address)
{
    uint32_t change = (address ^ shown_address) & ((1u << N_ADDR_PINS) - 1);
    shown_address = address;
    for (uint32_t i = 0; change != 0; i++, change >>= 1)
        if (change & 1)
            io_bank0_hw->io[BASE_ADDR_PIN + i].ctrl = ctrl[i] | level(address, i);
}

void UNIT_CODE(bench_oe)(bool low)
{
    io_bank0_hw->io[BASE_OE_PIN].ctrl = ctrl[OE] | (low ? INOVER_LOW : INOVER_HIGH);
}

void UNIT_CODE(bench_data)(uint32_t byte)
{
    uint32_t change = (byte ^ shown_data) & 0xFF;
    shown_data = byte;
    for (uint32_t i = 0; change != 0; i++, change >>= 1)
        if (change & 1)
            io_bank0_hw->io[BASE_DATA_PIN + i].ctrl = ctrl[DATA + i] | level(byte, i);
}

uint32_t UNIT_CODE(bench_read)(uint32_t address, const uint8_t *rom, uint32_t mask, uint32_t *value, uint32_t *end)
{
    bench_address(address);
    uint32_t start = systick_hw->cvr;
    bench_oe(true);
    while (since(start) < READ_CYCLES)
    {
    }
    *value = rom[address & mask];
    bench_oe(false);
    *end = systick_hw->cvr;
    return start;
}

// The pins' controls kept, then /CE low throughout, as the DME grounds it (951digit.pdf), and /OE
// high.
void bench_take()
{
    gpio_set_outover(BUF_OE_PIN, GPIO_OVERRIDE_HIGH); // the socket's data buffer off: nothing driven
    for (uint32_t i = 0; i < PINS; i++)
    {
        saved[i] = io_bank0_hw->io[pin(i)].ctrl;
        ctrl[i] = saved[i] & ~IO_BANK0_GPIO0_CTRL_INOVER_BITS;
    }
    io_bank0_hw->io[BASE_OE_PIN + 1].ctrl = ctrl[CE] | INOVER_LOW;
    bench_oe(false);
    for (uint32_t i = 0; i < N_ADDR_PINS; i++) // address 0 and data 0, all written once
        io_bank0_hw->io[BASE_ADDR_PIN + i].ctrl = ctrl[i] | INOVER_LOW;
    for (uint32_t i = 0; i < N_DATA_PINS; i++)
        io_bank0_hw->io[BASE_DATA_PIN + i].ctrl = ctrl[DATA + i] | INOVER_LOW;
    shown_address = shown_data = 0;
}

// The pins as they were, and the data buffer back under the output enable program.
void bench_give()
{
    for (uint32_t i = 0; i < PINS; i++)
        io_bank0_hw->io[pin(i)].ctrl = saved[i];
    gpio_set_outover(BUF_OE_PIN, GPIO_OVERRIDE_NORMAL);
}

bool bench_launch(void (*run)())
{
    if (state != IDLE)
        return false;
    rom_service_stop();
    bench_take();
    state = RUNNING;
    multicore_launch_core1_with_stack(run, core1_stack, sizeof(core1_stack));
    return true;
}

bool bench_running()
{
    return state != IDLE;
}

void bench_finished()
{
    state = FINISHED;
    while (true)
        __wfe();
}

void bench_update()
{
    if (state != FINISHED)
        return;
    multicore_reset_core1();
    bench_give();
    rom_service_start();
    state = IDLE;
}

#endif // FEATURE_MAP_UNIT
