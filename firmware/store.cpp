// The store (store.h; DME-Sim's docs/Paired_Fuel_Learning.md, steps 1 and 2): what the P28 learns kept in its
// flash across power cycles. A ring of eight 4 KiB sectors below the image holds records, a 256-byte
// page each: the kind (a byte), the kind's layout (a byte), the data's length (16 bits), a sequence
// number (one more each record), the key (a CRC-32 of what the kind's data mean in the image), a CRC-32
// over those and the data, the data (236 bytes at most), and in the page's last word the commit,
// programmed to 0 once the record's program has ended. A page without its commit or with a bad CRC is
// skipped, so a record torn by the power going leaves the one before it standing.
//
// L2 (store_tick, once a tick; each kind's only writer) looks once a second at a part of each kind, for
// one that has changed since its last record, at most once a minute (the kinds in turn: one look a tick
// at most), then copies the kind into the post's page a part a tick (the knock maps: a pair's 64 bytes
// each), so that no tick gets much more, and posts it.
// Thread mode (store_update, from map_unit_update) programs a posted record and reads it back, then
// commits it, with only the interrupts whose code is in flash held off: USB's, the comms window's, the
// SDK's timer. Core 1 and the levels run from RAM throughout. When the sector being written has
// AHEAD_PAGES pages left, thread mode erases the next one, after copying forward any kind whose newest
// record is there. The board's start (store_start) loads each kind's newest whole record whose key
// matches the image.
//
// Its kinds: 1, the knock maps (format 24's: each pair's lower); 2, the fuel trims (format 30's); 3, the fault
// memory (format 31's, faults.cpp: posted 10 s after its last record at a fault's confirmation, the minute's
// rule aside). From format 30 each kind as last stored stands in a live copy in the image's P28 half (pair_live),
// for the PC: thread mode writes it after each record it has programmed, the board's start from what it loaded
// (neutral, 0, if nothing).
//
// Flash is read through XIP, a word at a time (DME-Sim's model of it takes words only): through the
// cache for the start's scan, past it from then on. The store's own erase and program (the SDK's
// flash_range_erase and flash_range_program but for the cache's flush: store_flash_erase, below) run
// from RAM and are reached through the store's pointers to them, which DME-Sim reads to stand its model
// in for them.

#if defined(FEATURE_MAP_UNIT)

#include <stdio.h>

#include "hardware/flash.h"
#include "hardware/structs/io_qspi.h"
#include "hardware/structs/nvic.h"
#include "hardware/structs/pads_qspi.h"
#include "hardware/structs/scb.h"
#include "hardware/structs/systick.h"
#include "hardware/structs/xip_ctrl.h"
#include "hardware/sync.h"
#include "pico/bootrom.h"

#include "bus_bench.h"
#include "faults.h"
#include "pair.h"
#include "store.h"
#include "str_util.h"
#include "system.h"

static constexpr uint32_t SECTOR = 4096, PAGE = 256, SECTORS = 8;
static constexpr uint32_t SECTOR_PAGES = SECTOR / PAGE, PAGES = SECTORS * SECTOR_PAGES;
static constexpr uint32_t OFFSET = FLASH_SIZE - SECTOR - ROM_FLASH_SIZE - SECTORS * SECTOR; // below the image
static constexpr uint32_t HEADER = 16, COMMIT = PAGE - 4, MOST = COMMIT - HEADER;           // 236 bytes of data
static constexpr uint32_t KINDS = STORE_FAULTS;  // the kinds, 1 to KINDS
static constexpr uint32_t NONE = ~0u;
static constexpr uint32_t AHEAD_PAGES = 4;       // pages left when the next sector is erased: a copy
                                                 // forward of each kind and a record posted meanwhile
static constexpr uint32_t MINUTE_US = 60000000, SECOND_US = 1000000;
static constexpr uint32_t SOON_US = 10000000;     // format 31: a confirmation's record, 10 s after the kind's last
static constexpr uint32_t FLASH_IRQS = 1u << 3 | 1u << 5 | 1u << 7; // TIMER_IRQ_3, USBCTRL_IRQ, PIO0_IRQ_0
static constexpr uint32_t XIP = 0x10000000;      // the flash as the core reads it (XIP_BASE)
static constexpr uint32_t PAST_CACHE = 0x13000000; // the same past the cache (XIP_NOCACHE_NOALLOC_BASE)

static_assert(OFFSET % SECTOR == 0, "the store's sectors are the flash's");

static void BOARD_CODE(store_flash_erase)(uint32_t flash_offs, size_t count);
static void BOARD_CODE(store_flash_program)(uint32_t flash_offs, const uint8_t *data, size_t count);

// Each kind's layout (a record's second byte) and data's length: 1, the knock maps (format 24); 2, the fuel trims
// (format 30: 64 points and the idle factor, a word each); 3, the fault memory (format 31: a head of 8 bytes and
// seven records of 32).
static constexpr uint32_t KNOCK_RECORD = 1, KNOCK_BYTES = 128;
static constexpr uint32_t TRIM_RECORD = 1, TRIM_BYTES = 130;
static constexpr uint32_t FAULT_RECORD = 1, FAULT_BYTES = 232;

// The store's state. Read by DME-Sim (its model takes these offsets) up to its own part.
struct Store
{
    volatile uint8_t posted;                             // 0: L2 posted a record, thread mode's to program
    volatile uint8_t ahead;                              // 1: thread mode's own: the next sector to erase
    uint8_t spare[2];
    volatile uint32_t records;                           // 4: records written
    volatile uint32_t erases;                            // 8: sectors erased
    volatile uint32_t faults;                            // 12: pages read back wrong, left uncommitted
    volatile uint32_t sequence;                          // 16: the next record's
    volatile uint32_t loaded;                            // 20: the kinds loaded at the start (bit k)
    uint32_t offset;                                     // 24: the store's first sector in the flash
    uint32_t sectors;                                    // 28: its sectors, 4 KiB each
    void (*erase)(uint32_t, size_t);                     // 32: store_flash_erase (in RAM; the SDK's but the flush)
    void (*program)(uint32_t, const uint8_t *, size_t);  // 36: store_flash_program
    uint32_t jedec;                                      // 40: the flash's JEDEC ID (the board's start)
    uint32_t program_us, erase_us;                       // 44, 48: the longest program and erase, timed here
    volatile uint8_t asked[KINDS + 1];                   // 52: clears the PC asked for, a count a kind
    // Its own:
    uint8_t done[KINDS + 1];                             // the clears L2 has done
    uint8_t force[KINDS + 1];                            // a kind posted at L2's next look, whatever else
    uint8_t soon[KINDS + 1];                             // ... once 10 s have passed since its last post (format 31)
    uint32_t page;                                       // the next page to write, 0 to PAGES - 1
    uint32_t entered;                                    // the sector found blank to write in (NONE: none)
    uint32_t newest[KINDS + 1];                          // each kind's newest record's page (NONE: none)
    uint32_t key[KINDS + 1];                             // each kind's key in this image (0: the kind off)
    uint32_t posted_at[KINDS + 1];                       // each kind's last post (us)
    uint8_t look[KINDS + 1];                             // each kind's part L2 looks at next
    uint32_t looked_at[KINDS + 1];                       // each kind's last look for a change (us)
    uint32_t copying, copied;                            // the kind L2 copies for its record, its parts so far
    uint32_t kind, length;                               // the record posted: its kind, its data's length
    uint32_t turn;                                       // the kinds looked at in turn: the next after this (0 on)
};

Store store_state STORE_DATA = {0, 0, {0, 0}, 0, 0, 0, 1, 0, OFFSET, SECTORS, store_flash_erase, store_flash_program,
                                0, 0, 0, {}, {}, {}, {}, 0, NONE, {}, {}, {}, {}, {}, 0, 0, 0, 0, 0};

// Where the store reads its sectors: through the cache for the start's scan, which comes before any
// write, then past it, so that no line of them is ever read stale (the store's writes leave the cache
// as it is).
static uint32_t read_base STORE_DATA = PAST_CACHE;

// The post's page (L2's, then thread mode's), thread mode's own (a copy forward), the commit's (FFh but
// its last word, 0), and each kind as last stored: in bank 2 beside what the store keeps (not loaded:
// store_start sets them).
alignas(4) static uint8_t post_page[PAGE] __attribute__((section(".store")));
alignas(4) static uint8_t own_page[PAGE] __attribute__((section(".store")));
alignas(4) static uint8_t commit_page[PAGE] __attribute__((section(".store")));
static uint8_t kept[KINDS + 1][MOST] __attribute__((section(".store")));

// The CRC-32's table, a nibble at a time (in the bank: its code calls nothing outside it).
static uint32_t crc_nibbles[16] STORE_DATA = {
    0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC, 0x76DC4190, 0x6B6B51F4, 0x4DB26158, 0x5005713C,
    0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C, 0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C};

uint32_t STORE_CODE(store_crc)(uint32_t crc, uint32_t byte)
{
    crc ^= byte & 0xFF;
    crc = crc >> 4 ^ crc_nibbles[crc & 15];
    return crc >> 4 ^ crc_nibbles[crc & 15];
}

static __force_inline uint32_t now_us()
{
    return *(volatile uint32_t *)0x40054028;                            // the timer's TIMERAWL
}

// A word of the store's flash, at a byte offset from its first sector.
static __force_inline uint32_t flash_word(uint32_t at)
{
    return *(volatile const uint32_t *)(read_base + store_state.offset + at);
}

static bool STORE_CODE(page_blank)(uint32_t p)
{
    for (uint32_t at = p * PAGE; at < (p + 1) * PAGE; at += 4)
        if (flash_word(at) != NONE)
            return false;
    return true;
}

static bool STORE_CODE(sector_blank)(uint32_t sector)
{
    for (uint32_t p = sector * SECTOR_PAGES; p < (sector + 1) * SECTOR_PAGES; p++)
        if (!page_blank(p))
            return false;
    return true;
}

// The CRC of a record (its header's first 12 bytes, then its data) in the flash at page p.
static uint32_t STORE_CODE(flash_crc)(uint32_t p, uint32_t length)
{
    uint32_t crc = ~0u, base = p * PAGE;
    for (uint32_t i = 0; i < 12; i++)
        crc = store_crc(crc, flash_word(base + (i & ~3u)) >> 8 * (i & 3));
    for (uint32_t i = 0; i < length; i++)
        crc = store_crc(crc, flash_word(base + HEADER + (i & ~3u)) >> 8 * (i & 3));
    return ~crc;
}

// The same of a record in a page in RAM.
static uint32_t STORE_CODE(page_crc)(const uint8_t *page, uint32_t length)
{
    uint32_t crc = ~0u;
    for (uint32_t i = 0; i < 12; i++)
        crc = store_crc(crc, page[i]);
    for (uint32_t i = 0; i < length; i++)
        crc = store_crc(crc, page[HEADER + i]);
    return ~crc;
}

static __force_inline void put_word(uint8_t *at, uint32_t value)
{
    at[0] = value & 0xFF;
    at[1] = value >> 8 & 0xFF;
    at[2] = value >> 16 & 0xFF;
    at[3] = value >> 24;
}

// The kinds: their layout, data's length, key, bytes and restore (1: the knock maps, 2: the fuel trims,
// pair.cpp's).
static uint32_t STORE_CODE(kind_layout)(uint32_t kind)
{
    return kind == STORE_KNOCK ? KNOCK_RECORD : kind == STORE_TRIMS ? TRIM_RECORD : kind == STORE_FAULTS ? FAULT_RECORD : 0;
}

static uint32_t STORE_CODE(kind_bytes)(uint32_t kind)
{
    return kind == STORE_KNOCK ? KNOCK_BYTES : kind == STORE_TRIMS ? TRIM_BYTES : kind == STORE_FAULTS ? FAULT_BYTES : 0;
}

// A kind's record in parts, each a tick's work at most for L2 (the knock maps: a pair's 64 bytes each; the trims:
// 32 points, then 32 and the idle factor; the fault memory one, plain words).
static uint32_t STORE_CODE(kind_parts)(uint32_t kind)
{
    return kind == STORE_KNOCK || kind == STORE_TRIMS ? 2 : 1;
}

// A kind's live copy in the image (format 30): its bytes as last stored.
static void STORE_CODE(live)(uint32_t kind, const uint8_t *bytes)
{
    volatile uint8_t *to = pair_live(kind);
    if (to != nullptr)
        for (uint32_t i = 0; i < kind_bytes(kind); i++)
            to[i] = bytes[i];
}

// ---- The flash's erase and program (bank 2's .board_code: the board's alone; DME-Sim stands in) ----

// The SDK's flash_range_erase and flash_range_program as on the RP2040, but for the cache's flush at
// their end. A flush leaves the XIP cache cold, and the flash code that runs next with the interrupts
// off (the SDK's and TinyUSB's critical sections in the main loop) then held L0 off for up to ~10 us:
// beside the bus bench, store_bench made 84 of 200,000 answers late, and store_flush alone 89 to 145
// in 1,000 flushes, however the flush was waited for. The flush only keeps the cache coherent: the
// store reads its sectors past the cache once its start's scan is done (read_base), and nothing else
// in the flash changes. The ROM's flush's other duties stay: the cache enabled and the chip select's IO
// forcing cleared. Then boot2 (a copy made before the first operation, as the SDK makes its own) takes
// the flash back to XIP, and the QSPI pads are put back as they were.
static constexpr uint32_t BLOCK_ERASE = 0xD8;            // the SDK's FLASH_BLOCK_ERASE_CMD
static constexpr uint32_t BOOT2_WORDS = 64;              // boot2's 256 bytes at the flash's start
static uint32_t boot2_copy[BOOT2_WORDS] __attribute__((section(".store")));
static bool boot2_copied STORE_DATA = false;

static __force_inline void flash_sequence(uint32_t flash_offs, const uint8_t *data, size_t count)
{
    auto connect = (rom_connect_internal_flash_fn)rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
    auto exit_xip = (rom_flash_exit_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
    auto erase = (rom_flash_range_erase_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_RANGE_ERASE);
    auto program = (rom_flash_range_program_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_RANGE_PROGRAM);
    if (!boot2_copied)
    {
        for (uint32_t i = 0; i < BOOT2_WORDS; i++)
            boot2_copy[i] = ((const volatile uint32_t *)XIP)[i];
        __compiler_memory_barrier();
        boot2_copied = true;
    }
    uint32_t pads[count_of(pads_qspi_hw->io)];
    for (uint32_t i = 0; i < count_of(pads_qspi_hw->io); i++)
        pads[i] = pads_qspi_hw->io[i];
    __compiler_memory_barrier();                                // no flash access from here on
    connect();
    exit_xip();
    if (data != nullptr)
        program(flash_offs, data, count);
    else
        erase(flash_offs, count, FLASH_BLOCK_SIZE, BLOCK_ERASE);
    hw_set_bits(&xip_ctrl_hw->ctrl, XIP_CTRL_EN_BITS);          // the cache enabled, not flushed
    hw_write_masked(&io_qspi_hw->io[1].ctrl,                    // the chip select's forcing cleared
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_BITS);
    ((void (*)(void))((uintptr_t)boot2_copy + 1))();            // XIP again
    for (uint32_t i = 0; i < count_of(pads_qspi_hw->io); i++)
        pads_qspi_hw->io[i] = pads[i];
}

static void BOARD_CODE(store_flash_erase)(uint32_t flash_offs, size_t count)
{
    flash_sequence(flash_offs, nullptr, count);
}

static void BOARD_CODE(store_flash_program)(uint32_t flash_offs, const uint8_t *data, size_t count)
{
    flash_sequence(flash_offs, data, count);
}

// ---- Thread mode: the flash's programs and erases ----

// A page programmed at a flash offset (page), or the sector there erased (null), by the store's own
// (from RAM: the core waits in the boot ROM meanwhile), only the interrupts whose code is in flash held
// off. Returns its time (us).
static uint32_t STORE_CODE(flash_op)(uint32_t at, const uint8_t *page)
{
    Store &s = store_state;
    uint32_t held = nvic_hw->iser & FLASH_IRQS;
    nvic_hw->icer = held;
    uint32_t start = now_us();
    if (page != nullptr)
        s.program(at, page, PAGE);
    else
        s.erase(at, SECTOR);
    uint32_t took = now_us() - start;
    nvic_hw->iser = held;
    return took;
}

static void STORE_CODE(program)(uint32_t p, const uint8_t *page)
{
    Store &s = store_state;
    uint32_t took = flash_op(s.offset + p * PAGE, page);
    if (took > s.program_us)
        s.program_us = took;
}

static void STORE_CODE(erase)(uint32_t sector)
{
    Store &s = store_state;
    uint32_t took = flash_op(s.offset + sector * SECTOR, nullptr);
    if (took > s.erase_us)
        s.erase_us = took;
    s.erases = s.erases + 1;
    if (s.entered == sector)
        s.entered = NONE;                                       // found blank again when entered
}

// The next page to write: on from store.page, past pages not blank (a program the power cut); a sector
// is written in only once found blank whole (erased). NONE: the sector to enter is not, so the caller
// erases it first.
static uint32_t STORE_CODE(blank_page)()
{
    Store &s = store_state;
    for (uint32_t tries = 0; tries < PAGES; tries++)
    {
        uint32_t sector = s.page / SECTOR_PAGES;
        if (s.entered != sector)
        {
            if (!sector_blank(sector))
                return NONE;
            s.entered = sector;
        }
        if (page_blank(s.page))
            return s.page;
        s.page = (s.page + 1) % PAGES;
    }
    return NONE;
}

// A record's page programmed at p, read back, then committed and its commit read back; the next page
// to write past it either way. False: a fault (it stays uncommitted, so it is skipped).
static bool STORE_CODE(put)(uint32_t p, const uint8_t *page)
{
    Store &s = store_state;
    s.page = (p + 1) % PAGES;
    program(p, page);
    for (uint32_t at = 0; at < COMMIT; at += 4)
    {
        uint32_t word = page[at] | page[at + 1] << 8 | page[at + 2] << 16 | (uint32_t)page[at + 3] << 24;
        if (flash_word(p * PAGE + at) != word)
        {
            s.faults = s.faults + 1;
            return false;
        }
    }
    program(p, commit_page);
    if (flash_word(p * PAGE + COMMIT) != 0)
    {
        s.faults = s.faults + 1;
        return false;
    }
    s.records = s.records + 1;
    return true;
}

// A record of a kind made in page (its data there already, its length bytes): its header, its CRC,
// FFh after the data and in the commit's word, then put at the next blank page. False: no page (the
// sector to enter not erased) or a fault.
static bool STORE_CODE(write)(uint8_t *page, uint32_t kind, uint32_t length)
{
    Store &s = store_state;
    uint32_t p = blank_page();
    if (p == NONE)
        return false;
    page[0] = kind;
    page[1] = kind_layout(kind);
    page[2] = length & 0xFF;
    page[3] = length >> 8;
    put_word(page + 4, s.sequence);
    put_word(page + 8, s.key[kind]);
    for (uint32_t i = HEADER + length; i < PAGE; i++)
        page[i] = 0xFF;
    put_word(page + 12, page_crc(page, length));
    s.sequence = s.sequence + 1;
    if (!put(p, page))
        return false;
    s.newest[kind] = p;
    return true;
}

// The sector after the one being written erased once it has AHEAD_PAGES pages left, unless blank.
static void STORE_CODE(check_ahead)()
{
    Store &s = store_state;
    uint32_t left = SECTOR_PAGES - s.page % SECTOR_PAGES;
    if (left <= AHEAD_PAGES && !sector_blank((s.page / SECTOR_PAGES + 1) % SECTORS))
        s.ahead = 1;
}

// The sector to erase: the one being entered if it is not blank, else the one after the sector being
// written. Each kind whose newest record is in it copied forward first (one record a call), into the
// sector being written; with no room there, posted again from RAM after the erase.
static void STORE_CODE(prepare)()
{
    Store &s = store_state;
    uint32_t sector = s.page / SECTOR_PAGES;
    bool entering = s.entered != sector;
    if (!entering)
        sector = (sector + 1) % SECTORS;
    for (uint32_t kind = 1; kind <= KINDS; kind++)
    {
        uint32_t p = s.newest[kind];
        if (p == NONE || p / SECTOR_PAGES != sector)
            continue;
        s.newest[kind] = NONE;
        if (!entering)
        {
            for (uint32_t at = 0; at < PAGE; at += 4)
                put_word(own_page + at, flash_word(p * PAGE + at));
            uint32_t length = own_page[2] | own_page[3] << 8;
            if (write(own_page, kind, length))
                return;                                         // one program a call
        }
        s.force[kind] = 1;                                      // L2 posts it again
    }
    erase(sector);
    s.ahead = 0;
}

void STORE_CODE(store_update)()
{
    Store &s = store_state;
    if (s.posted && blank_page() != NONE)
    {
        __asm volatile("" ::: "memory");                        // the post's record read after its flag
        uint32_t kind = s.kind;
        if (write(post_page, kind, s.length))
        {
            for (uint32_t i = 0; i < s.length; i++)
                kept[kind][i] = post_page[HEADER + i];
            live(kind, kept[kind]);                             // format 30: the PC's copy of what is stored
        }
        __asm volatile("" ::: "memory");                        // the page done before L2 refills it
        s.posted = 0;                                           // a fault: posted again at its next change
        check_ahead();
        return;
    }
    if (s.ahead || s.posted)
        prepare();                                              // no blank page: the sector to enter erased first
}

// ---- L2: the kinds posted ----

static bool STORE_CODE(kind_differs)(uint32_t kind, uint32_t part)
{
    if (kind == STORE_TRIMS)
        return trims_differs(part, kept[kind]);
    if (kind == STORE_FAULTS)
        return faults_differs(part, kept[kind]);
    return kind == STORE_KNOCK && knock_differs(part, kept[kind]);
}

static void STORE_CODE(kind_copy)(uint32_t kind, uint32_t part)
{
    if (kind == STORE_KNOCK)
        knock_copy(part, post_page + HEADER);
    else if (kind == STORE_TRIMS)
        trims_copy(part, post_page + HEADER);
    else if (kind == STORE_FAULTS)
        faults_copy(part, post_page + HEADER);
}

// Once a tick: a clear the PC asked for; a kind being copied, a part a tick, then posted; else a part of
// a kind whose minute has passed looked at, once a second for each kind (the kinds in turn, each kind's
// parts in turn: one look a tick at most), and copied from the next tick on if it differs from what is
// kept (a forced kind at once: a clear's neutral record; format 31, a kind asked for soon once 10 s have
// passed since its last post: a fault's confirmation).
void STORE_CODE(store_tick)()
{
    Store &s = store_state;
    if (s.posted)
        return;                                                 // thread mode has the post's page
    uint32_t now = now_us();
    if (s.asked[STORE_KNOCK] != s.done[STORE_KNOCK])            // the PC's clears, each at once, the engine
    {                                                           //   running or not (the owner, 2026-11-19)
        knock_clear();                                          // (a neutral record at once)
        s.force[STORE_KNOCK] = 1;
        s.done[STORE_KNOCK] = s.asked[STORE_KNOCK];
    }
    if (s.asked[STORE_TRIMS] != s.done[STORE_TRIMS])            // (the trims': format 30)
    {
        trims_clear();
        s.force[STORE_TRIMS] = 1;
        s.done[STORE_TRIMS] = s.asked[STORE_TRIMS];
    }
    if (s.asked[STORE_FAULTS] != s.done[STORE_FAULTS])          // (the fault memory's: format 31)
    {
        faults_clear();
        s.force[STORE_FAULTS] = 1;
        s.done[STORE_FAULTS] = s.asked[STORE_FAULTS];
    }
    if (s.copying != 0)
    {
        uint32_t kind = s.copying;
        kind_copy(kind, s.copied);
        s.copied = s.copied + 1;
        if (s.copied < kind_parts(kind))
            return;
        s.copying = 0;
        s.kind = kind;
        s.length = kind_bytes(kind);
        s.posted_at[kind] = now;
        s.force[kind] = s.soon[kind] = 0;
        __asm volatile("" ::: "memory");                        // the record whole before its post
        s.posted = 1;
        return;
    }
    for (uint32_t n = 0; n < KINDS; n++)                        // the kinds in turn, from the one after the last
    {                                                           //   looked at: a part of one a tick at most
        uint32_t next = s.turn + n;                             // (no %: with three kinds, a division in flash)
        uint32_t kind = (next >= KINDS ? next - KINDS : next) + 1;
        bool due = s.force[kind] || (s.soon[kind] && now - s.posted_at[kind] >= SOON_US);
        if (s.key[kind] == 0 || (!due && (now - s.posted_at[kind] < MINUTE_US || now - s.looked_at[kind] < SECOND_US)))
            continue;                                           // (off, its minute or its second not passed)
        s.turn = kind == KINDS ? 0 : kind;
        if (!due)
        {
            s.looked_at[kind] = now;
            uint32_t part = s.look[kind];
            s.look[kind] = part + 1 < kind_parts(kind) ? part + 1 : 0;
            if (!kind_differs(kind, part))
                return;                                         // (the next kind's look at a later tick)
        }
        s.copying = kind;                                       // copied from the next tick on
        s.copied = 0;
        return;                                                 // one at a time
    }
}

// ---- The board's start ----

void STORE_CODE(store_soon)(uint32_t kind)
{
    store_state.soon[kind] = 1;
}

uint32_t STORE_CODE(store_faults)()
{
    return store_state.faults;
}

void STORE_CODE(store_start)()
{
    Store &s = store_state;
    uint32_t now = now_us();
    for (uint32_t i = 0; i < PAGE; i++)                         // (loops: no memset in the bank)
        commit_page[i] = i < COMMIT ? 0xFF : 0;
    s.posted = s.ahead = 0;
    s.loaded = 0;
    s.copying = s.copied = s.turn = 0;
    s.key[STORE_KNOCK] = knock_key();
    s.key[STORE_TRIMS] = trims_key();
    s.key[STORE_FAULTS] = faults_key();
    read_base = s.records == 0 && s.erases == 0 && s.faults == 0 ? XIP : PAST_CACHE; // the cache: nothing written
    uint32_t best[KINDS + 1], top = 0, top_page = NONE;         // (zeroed in the loop: an initialiser of three
    for (uint32_t kind = 0; kind <= KINDS; kind++)              //   words is the SDK's memset, in flash)
    {
        best[kind] = 0;
        s.newest[kind] = NONE;
        s.force[kind] = s.soon[kind] = 0;
        s.look[kind] = 0;
        s.looked_at[kind] = now;
        s.done[kind] = s.asked[kind];
        s.posted_at[kind] = kind == STORE_FAULTS ? now - SOON_US : now; // (format 31: a confirmation's record at once
                                                                         //   from the start, 10 s after its last)
        for (uint32_t i = 0; i < MOST; i++)
            kept[kind][i] = 0;
    }
    for (uint32_t p = 0; p < PAGES; p++)                        // each committed record whose CRC holds
    {
        if (flash_word(p * PAGE + COMMIT) != 0)
            continue;
        uint32_t head = flash_word(p * PAGE), kind = head & 0xFF, length = head >> 16;
        if (kind < 1 || kind > KINDS || length > MOST || flash_crc(p, length) != flash_word(p * PAGE + 12))
            continue;
        uint32_t sequence = flash_word(p * PAGE + 4);
        if (sequence > best[kind])
        {
            best[kind] = sequence;
            s.newest[kind] = p;
        }
        if (sequence >= top)
        {
            top = sequence;
            top_page = p;
        }
    }
    s.sequence = top + 1;
    s.page = top_page == NONE ? 0 : (top_page + 1) % PAGES;
    s.entered = top_page == NONE || s.page % SECTOR_PAGES == 0 ? NONE : s.page / SECTOR_PAGES;
    for (uint32_t kind = 1; kind <= KINDS; kind++)              // each kind's newest, if it means the same
    {
        uint32_t p = s.newest[kind];
        if (p == NONE)
            continue;
        uint32_t head = flash_word(p * PAGE);
        if (s.key[kind] == 0 || flash_word(p * PAGE + 8) != s.key[kind] || (head >> 8 & 0xFF) != kind_layout(kind)
            || head >> 16 != kind_bytes(kind))
        {
            s.newest[kind] = NONE;                              // another image's: left to be erased
            continue;
        }
        for (uint32_t i = 0; i < kind_bytes(kind); i++)
            kept[kind][i] = flash_word(p * PAGE + HEADER + (i & ~3u)) >> 8 * (i & 3) & 0xFF;
        if (kind == STORE_KNOCK)
            knock_restore(kept[kind]);
        else if (kind == STORE_TRIMS)
            trims_restore(kept[kind]);
        else if (kind == STORE_FAULTS)
            faults_restore(kept[kind]);
        s.loaded = s.loaded | 1u << kind;
    }
    for (uint32_t kind = 1; kind <= KINDS; kind++)              // format 30: the live copy, what was loaded
        live(kind, kept[kind]);                                 //   (0, neutral, if nothing)
    read_base = PAST_CACHE;                                     // from here on past the cache
    check_ahead();
}

// The board's alone (flash code, before the 8051 is served): the flash's JEDEC ID (9Fh).
void store_board_start()
{
    uint8_t out[4] = {0x9F, 0, 0, 0}, in[4] = {};
    uint32_t ints = save_and_disable_interrupts();
    flash_do_cmd(out, in, 4);
    restore_interrupts(ints);
    store_state.jedec = (uint32_t)in[1] << 16 | in[2] << 8 | in[3];
}

// ---- store_bench: the flash's erase and program timed (flash code, the board's alone) ----

// The sector below the store's, which nothing else uses: erased (and found blank) and its 16 pages
// programmed (each read back), n times, an operation a pass of the main loop after the store's own work,
// then erased once more, each timed as the store's are (flash_op). The bus bench may run meanwhile
// (BoardCheck --store-bench): with the flash off the bus, whatever runs must run from RAM.
static constexpr uint32_t BENCH = OFFSET - SECTOR;

struct Bench
{
    uint32_t want, cycles, op;                  // cycles asked and done; op 0 the erase, 1 to 16 a page's
    uint32_t erases, erase_sum, erase_most;     // us
    uint32_t programs, program_sum, program_most;
    uint32_t faults;                            // a sector not blank after its erase, a page read back wrong
    bool running, ran;
};

static Bench bench STORE_DATA = {};                                  // in bank 2: the main RAM is full
alignas(4) static uint8_t bench_page[PAGE] __attribute__((section(".store")));

static uint32_t bench_word(uint32_t at)
{
    return *(volatile const uint32_t *)(PAST_CACHE + BENCH + at);   // past the cache, as the store's
}

static void store_flush_update();

void store_bench_update()
{
    store_flush_update();
    Bench &b = bench;
    if (!b.running || store_state.posted || store_state.ahead)
        return;
    if (b.op == 0)
    {
        uint32_t took = flash_op(BENCH, nullptr);
        b.erases++;
        b.erase_sum += took;
        if (took > b.erase_most)
            b.erase_most = took;
        for (uint32_t at = 0; at < SECTOR; at += 4)
            if (bench_word(at) != NONE)
            {
                b.faults++;
                break;
            }
        if (b.cycles == b.want)
            b.running = false;                  // the last erase: the sector left blank
        else
            b.op = 1;
        return;
    }
    uint32_t page = b.op - 1, seed = b.cycles << 8 | page;
    for (uint32_t at = 0; at < PAGE; at += 4)
    {
        seed = seed * 1664525u + 1013904223u;
        put_word(bench_page + at, seed);
    }
    uint32_t took = flash_op(BENCH + page * PAGE, bench_page);
    b.programs++;
    b.program_sum += took;
    if (took > b.program_most)
        b.program_most = took;
    for (uint32_t at = 0; at < PAGE; at += 4)
        if (bench_word(page * PAGE + at) != *(const uint32_t *)(bench_page + at))
        {
            b.faults++;
            break;
        }
    if (++b.op > SECTOR_PAGES)
    {
        b.op = 0;
        b.cycles++;
    }
}

// store_flush=n,way: the XIP cache flushed n times, a flush a pass of the main loop, each timed (cycles):
// way 0 as the SDK's erase and program end (the boot ROM's flash_flush_cache reads FLUSH back, which
// holds core 0's bus until the flush is done), way 1 with STAT polled (the bus free meanwhile). Nothing
// in the flash changes. From RAM (bank 2's .board_code): from flash the fetches would wait for it too.
struct Flushes
{
    uint32_t want, done, way, most, sum;        // cycles
    uint32_t unfinished;                        // way 2: flushes not done by the wait's end
    bool running, ran;
};

static Flushes flushes STORE_DATA = {};

static constexpr uint32_t FLUSH_WAIT = 10 * (SYS_CLK_HZ / 1000000); // way 2: 10 us untouched

static uint32_t BOARD_CODE(flush_once)(uint32_t way)
{
    uint32_t start = systick_hw->cvr;
    xip_ctrl_hw->flush = 1;
    if (way == 0)
        (void)xip_ctrl_hw->flush;
    else if (way == 1)
        while ((xip_ctrl_hw->stat & XIP_STAT_FLUSH_READY_BITS) == 0)
        {
        }
    else
    {
        while (((start - systick_hw->cvr) & 0xFFFFFF) < FLUSH_WAIT) // nothing of the XIP block touched
        {
        }
        if ((xip_ctrl_hw->stat & XIP_STAT_FLUSH_READY_BITS) == 0)
            flushes.unfinished++;                                // not done in the wait
    }
    return (start - systick_hw->cvr) & 0xFFFFFF;            // SysTick counts down, 24 bits
}

static void store_flush_update()
{
    Flushes &f = flushes;
    if (!f.running)
        return;
    uint32_t took = flush_once(f.way);
    f.sum += took;
    if (took > f.most)
        f.most = took;
    if (++f.done == f.want)
        f.running = false;
}

// The interrupts enabled on core 0 whose handler (the vector table's entry) is in flash: an operation
// holds off FLASH_IRQS alone, so any other here would run flash code with the flash off the bus.
static uint32_t flash_handlers()
{
    const uint32_t *table = (const uint32_t *)scb_hw->vtor;
    uint32_t enabled = nvic_hw->iser, found = 0;
    for (uint32_t irq = 0; irq < 32; irq++)
        if ((enabled >> irq & 1) != 0 && table[16 + irq] >= XIP && table[16 + irq] < 0x20000000)
            found |= 1u << irq;
    return found;
}

// ---- The PC's requests (thread mode's: the parameters below; DME-Sim's board link calls these too) ----

void STORE_CODE(store_ask_clear)(uint32_t kinds)
{
    for (uint32_t kind = 1; kind <= KINDS; kind++)
        if (kinds & 1u << kind)
            store_state.asked[kind] = store_state.asked[kind] + 1;
}

uint32_t STORE_CODE(store_clears_pending)()
{
    uint32_t kinds = 0;
    for (uint32_t kind = 1; kind <= KINDS; kind++)
        if (store_state.asked[kind] != store_state.done[kind])
            kinds |= 1u << kind;
    return kinds;
}

// store: records written, sectors erased, faults, the next record's sequence, the kinds loaded at the
// start (bits); store_flash: the flash's JEDEC ID, the longest program and erase (us); store_clear: the
// kinds asked to be cleared and not yet (L2 clears them at its next tick), their words ("knock", "trims",
// "faults"), or "none".
bool store_get_parameter(const char *name, char *value, size_t value_size)
{
    Store &s = store_state;
    if (streq(name, "store"))
    {
        snprintf(value, value_size, "%u %u %u %u %u", (unsigned)s.records, (unsigned)s.erases, (unsigned)s.faults,
                 (unsigned)s.sequence, (unsigned)s.loaded);
        return true;
    }
    if (streq(name, "store_flash"))
    {
        snprintf(value, value_size, "%06x %u %u", (unsigned)s.jedec, (unsigned)s.program_us, (unsigned)s.erase_us);
        return true;
    }
    if (streq(name, "store_clear"))
    {
        uint32_t pending = store_clears_pending();
        static const char *const words[KINDS + 1] = {nullptr, "knock", "trims", "faults"}; // (thread mode's: flash)
        size_t n = 0;
        for (uint32_t kind = 1; kind <= KINDS; kind++)
            if ((pending & 1u << kind) && n < value_size)
                n += snprintf(value + n, value_size - n, n == 0 ? "%s" : " %s", words[kind]);
        if (n == 0)
            strcpyz(value, value_size, "none");
        return true;
    }
    if (streq(name, "store_bench"))                     // cycles, faults, erase most, mean, program most, mean
    {
        const Bench &b = bench;
        if (!b.ran)
            strcpyz(value, value_size, "none");
        else if (b.running)
            snprintf(value, value_size, "running %u of %u", (unsigned)b.cycles, (unsigned)b.want);
        else
            snprintf(value, value_size, "%u %u %u %u %u %u", (unsigned)b.cycles, (unsigned)b.faults,
                     (unsigned)b.erase_most, (unsigned)(b.erase_sum / b.erases), (unsigned)b.program_most,
                     (unsigned)(b.program_sum / b.programs));
        return true;
    }
    if (streq(name, "store_flush"))                     // flushes, the way, cycles most and mean
    {
        const Flushes &f = flushes;
        if (!f.ran)
            strcpyz(value, value_size, "none");
        else if (f.running)
            snprintf(value, value_size, "running %u of %u", (unsigned)f.done, (unsigned)f.want);
        else
            snprintf(value, value_size, "%u %u %u %u %u", (unsigned)f.done, (unsigned)f.way, (unsigned)f.most,
                     (unsigned)(f.sum / f.done), (unsigned)f.unfinished);
        return true;
    }
    if (streq(name, "store_irqs"))                      // enabled, those with flash handlers, SysTick's own
    {
        snprintf(value, value_size, "%08x %08x %u", (unsigned)nvic_hw->iser, (unsigned)flash_handlers(),
                 (unsigned)(systick_hw->csr >> 1 & 1));
        return true;
    }
    return false;
}

// store_clear=knock, trims, faults or all: the knock maps, the trims, the fault memory or every kind cleared at L2's
// next tick, the engine running or not, each with a neutral record written at once. store_bench=n (1 to 1000): the
// bench, n cycles, unless one runs.
bool store_set_parameter(const char *name, const char *value)
{
    if (streq(name, "store_flush"))
    {
        const char *comma = value;
        while (*comma != '\0' && *comma != ',')
            comma++;
        uint32_t n = strtoul(value), way = *comma == ',' ? strtoul(comma + 1) : 0;
        if (flushes.running || n < 1 || n > 100000 || way > 2)
            return false;
        flushes = {};
        flushes.want = n;
        flushes.way = way;
        flushes.running = flushes.ran = true;
        return true;
    }
    if (streq(name, "store_bench"))
    {
        uint32_t n = strtoul(value);
        if (bench.running || n < 1 || n > 1000)
            return false;
        bench = {};
        bench.want = n;
        bench.running = bench.ran = true;
        return true;
    }
    uint32_t kinds = streq(value, "knock")    ? 1u << STORE_KNOCK
                     : streq(value, "trims")  ? 1u << STORE_TRIMS
                     : streq(value, "faults") ? 1u << STORE_FAULTS
                     : streq(value, "all")    ? 1u << STORE_KNOCK | 1u << STORE_TRIMS | 1u << STORE_FAULTS
                                              : 0;
    if (!streq(name, "store_clear") || kinds == 0)
        return false;
    store_ask_clear(kinds);
    return true;
}

#endif // FEATURE_MAP_UNIT
