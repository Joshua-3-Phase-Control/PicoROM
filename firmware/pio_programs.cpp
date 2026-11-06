#include "pio_programs.h"

#include "system.h"
#include "comms.pio.h"
#include "data_bus.pio.h"
#include "map_unit.pio.h"
#include <hardware/pio.h>

PIOProgram prg_map_window;
PIOProgram prg_snoop;
PIOProgram prg_comms_detect;
PIOProgram prg_comms_window;
PIOProgram prg_comms_clock;
PIOProgram prg_write_tca_bits;
PIOProgram prg_data_output;
PIOProgram prg_set_output_enable;
PIOProgram prg_set_pindir_hi;
PIOProgram prg_set_pindir_lo;
PIOProgram prg_report_data_access;

#define add_program(p, s, name, prg)                                                                                   \
    do                                                                                                                 \
    {                                                                                                                  \
        prg.pio_index = PIO_NUM(p);                                                                                    \
        prg.sm = s;                                                                                                    \
        prg.config_func = name##_program_get_default_config;                                                           \
        prg.offset = pio_add_program(p, &name##_program);                                                              \
        if (prg.offset < 0) return false;                                                                              \
    } while (false)

// The address, seen from the OE pins (comms_window's in pins): A0 after them, all of it below them.
static_assert(BASE_ADDR_PIN + N_ADDR_PINS <= BASE_OE_PIN, "the comms window reads the address after the OE pins");
static constexpr uint32_t WINDOW_TO_ADDRESS = ((BASE_ADDR_PIN - BASE_OE_PIN) & 31) - 2;
static_assert(WINDOW_TO_ADDRESS >= 1 && N_ADDR_PINS > 8, "the comms window's counts are 1 to 32");

// comms_window's two counts for this board's pins (comms.pio): written over the program as loaded.
static void set_comms_window_counts()
{
    PIO p = prg_comms_window.pio();
    p->instr_mem[prg_comms_window.offset + comms_window_offset_to_address] = pio_encode_out(pio_null, WINDOW_TO_ADDRESS);
    p->instr_mem[prg_comms_window.offset + comms_window_offset_page_bits] = pio_encode_out(pio_x, N_ADDR_PINS - 8);
}

// map_window's two counts for this board's pins (map_unit.pio), as comms_window's: to A0, then A8 up.
static void set_map_window_counts()
{
    PIO p = prg_map_window.pio();
    p->instr_mem[prg_map_window.offset + map_window_offset_to_address] = pio_encode_out(pio_null, WINDOW_TO_ADDRESS);
    p->instr_mem[prg_map_window.offset + map_window_offset_page_bits] = pio_encode_out(pio_x, N_ADDR_PINS - 8);
}

// The snoop's /OE and its two counts (map_unit.pio): the gap, (n + 1) turns of 9 cycles to 1 us at
// least, and the rest to 1.5 us from the read's end (two cycles for the two sets).
static constexpr uint32_t SNOOP_GAP_TURNS = (SYS_CLK_HZ + 8999999) / 9000000;
static constexpr uint32_t SNOOP_SAMPLE_TURNS = (SYS_CLK_HZ / 2000000 * 3 - 2 - 9 * SNOOP_GAP_TURNS + 4) / 9;
static_assert(SNOOP_GAP_TURNS - 1 < 32 && SNOOP_SAMPLE_TURNS >= 1 && SNOOP_SAMPLE_TURNS - 1 < 32, "set's 5 bits");

static void set_snoop_counts()
{
    PIO p = prg_snoop.pio();
    p->instr_mem[prg_snoop.offset + snoop_offset_read] = pio_encode_wait_gpio(false, BASE_OE_PIN);
    p->instr_mem[prg_snoop.offset + snoop_offset_read_ends] = pio_encode_wait_gpio(true, BASE_OE_PIN);
    p->instr_mem[prg_snoop.offset + snoop_offset_gap_count] = pio_encode_set(pio_x, SNOOP_GAP_TURNS - 1);
    p->instr_mem[prg_snoop.offset + snoop_offset_sample_count] = pio_encode_set(pio_x, SNOOP_SAMPLE_TURNS - 1);
}

void pio_programs_reset()
{
    pio_clear_instruction_memory(pio0);
    pio_clear_instruction_memory(pio1);

    prg_map_window.reset();
    prg_snoop.reset();
    prg_report_data_access.reset();
    prg_comms_detect.reset();
    prg_comms_window.reset();
    prg_comms_clock.reset();
    prg_write_tca_bits.reset();
    prg_data_output.reset();
    prg_set_output_enable.reset();
    prg_set_pindir_hi.reset();
    prg_set_pindir_lo.reset();
}

bool pio_programs_init()
{
    pio_programs_reset();

    add_program(pio0, 0, set_pindir, prg_set_pindir_lo);
    prg_set_pindir_hi = prg_set_pindir_lo;
    prg_set_pindir_hi.sm = 1;
    add_program(pio0, 2, comms_detect, prg_comms_detect);
    add_program(pio0, 2, comms_window, prg_comms_window); // the same sm: comms_mode runs one or the other
    set_comms_window_counts();
    add_program(pio0, 3, data_output, prg_data_output);

    if (N_OE_PINS == 2)
        add_program(pio1, 0, or_output_enable, prg_set_output_enable);
    else
        add_program(pio1, 0, route_output_enable, prg_set_output_enable);

#if defined(FEATURE_MAP_UNIT) // the snoop in its place: it shows the 8051's activity too
    add_program(SNOOP_PIO, SNOOP_SM, snoop, prg_snoop);
    set_snoop_counts();
#else
    add_program(pio1, 1, report_data_access, prg_report_data_access);
#endif

#if defined(FEATURE_CLOCK) // the only builds that use it
    add_program(pio1, 2, comms_clock, prg_comms_clock);
#endif
#if defined(FEATURE_MAP_UNIT)
    add_program(MAP_UNIT_PIO, MAP_UNIT_SM, map_window, prg_map_window);
    set_map_window_counts();
#endif
    add_program(pio1, 3, write_tca_bits, prg_write_tca_bits);

    return true;
}
