// The store (store.h; DME-Sim's docs/Paired_Fuel_Learning.md, step 1): what the P28 learns kept in its
// flash across power cycles. A ring of eight 4 KiB sectors below the image holds records, a 256-byte
// page each: the kind (a byte), the kind's layout (a byte), the data's length (16 bits), a sequence
// number (one more each record), the key (a CRC-32 of what the kind's data mean in the image), a CRC-32
// over those and the data, the data (236 bytes at most), and in the page's last word the commit,
// programmed to 0 once the record's program has ended. A page without its commit or with a bad CRC is
// skipped, so a record torn by the power going leaves the one before it standing.
//
// L2 (store_tick, once a tick; each kind's only writer) looks once a second for a part of a kind that
// has changed since its last record, at most once a minute, then copies the kind into the post's page a
// part a tick (the knock maps: a pair's 64 bytes each), so that no tick gets much more, and posts it.
// Thread mode (store_update, from map_unit_update) programs a posted record and reads it back, then
// commits it, with only the interrupts whose code is in flash held off: USB's, the comms window's, the
// SDK's timer. Core 1 and the levels run from RAM throughout. When the sector being written has
// AHEAD_PAGES pages left, thread mode erases the next one, after copying forward any kind whose newest
// record is there. The board's start (store_start) loads each kind's newest whole record whose key
// matches the image.
//
// Flash is read through XIP, a word at a time (DME-Sim's model of it takes words only); the SDK's
// flash_range_erase and flash_range_program, which run from RAM, are reached through the store's
// pointers to them, which DME-Sim reads to stand its model in for them.

#if defined(FEATURE_MAP_UNIT)

#include <stdio.h>

#include "hardware/flash.h"
#include "hardware/structs/nvic.h"
#include "hardware/sync.h"

#include "pair.h"
#include "store.h"
#include "str_util.h"
#include "system.h"

static constexpr uint32_t SECTOR = 4096, PAGE = 256, SECTORS = 8;
static constexpr uint32_t SECTOR_PAGES = SECTOR / PAGE, PAGES = SECTORS * SECTOR_PAGES;
static constexpr uint32_t OFFSET = FLASH_SIZE - SECTOR - ROM_FLASH_SIZE - SECTORS * SECTOR; // below the image
static constexpr uint32_t HEADER = 16, COMMIT = PAGE - 4, MOST = COMMIT - HEADER;           // 236 bytes of data
static constexpr uint32_t KINDS = STORE_KNOCK;   // the kinds, 1 to KINDS
static constexpr uint32_t NONE = ~0u;
static constexpr uint32_t AHEAD_PAGES = 4;       // pages left when the next sector is erased: a copy
                                                 // forward of each kind and a record posted meanwhile
static constexpr uint32_t MINUTE_US = 60000000, SECOND_US = 1000000;
static constexpr uint32_t FLASH_IRQS = 1u << 3 | 1u << 5 | 1u << 7; // TIMER_IRQ_3, USBCTRL_IRQ, PIO0_IRQ_0
static constexpr uint32_t XIP = 0x10000000;      // the flash as the core reads it (XIP_BASE)

static_assert(OFFSET % SECTOR == 0, "the store's sectors are the flash's");

// Each kind's layout (a record's second byte) and data's length: 1, the knock maps (format 24).
static constexpr uint32_t KNOCK_RECORD = 1, KNOCK_BYTES = 128;

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
    void (*erase)(uint32_t, size_t);                     // 32: the SDK's flash_range_erase (in RAM)
    void (*program)(uint32_t, const uint8_t *, size_t);  // 36: flash_range_program
    uint32_t jedec;                                      // 40: the flash's JEDEC ID (the board's start)
    uint32_t program_us, erase_us;                       // 44, 48: the longest program and erase, timed here
    volatile uint8_t asked[KINDS + 1];                   // 52: clears the PC asked for, a count a kind
    // Its own:
    uint8_t done[KINDS + 1];                             // the clears L2 has done
    uint8_t force[KINDS + 1];                            // a kind posted at L2's next look, whatever else
    uint32_t page;                                       // the next page to write, 0 to PAGES - 1
    uint32_t entered;                                    // the sector found blank to write in (NONE: none)
    uint32_t newest[KINDS + 1];                          // each kind's newest record's page (NONE: none)
    uint32_t key[KINDS + 1];                             // each kind's key in this image (0: the kind off)
    uint32_t posted_at[KINDS + 1];                       // each kind's last post (us)
    uint8_t look[KINDS + 1];                             // each kind's part L2 looks at next
    uint32_t looked_at;                                  // L2's last look for a change (us)
    uint32_t copying, copied;                            // the kind L2 copies for its record, its parts so far
    uint32_t kind, length;                               // the record posted: its kind, its data's length
};

Store store_state STORE_DATA = {0, 0, {0, 0}, 0, 0, 0, 1, 0, OFFSET, SECTORS, flash_range_erase, flash_range_program,
                                0, 0, 0, {}, {}, {}, 0, NONE, {}, {}, {}, {}, 0, 0, 0, 0, 0};

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
    return *(volatile const uint32_t *)(XIP + store_state.offset + at);
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

// The kinds: their layout, data's length, key, bytes and restore (1: the knock maps, pair.cpp's).
static uint32_t STORE_CODE(kind_layout)(uint32_t kind)
{
    return kind == STORE_KNOCK ? KNOCK_RECORD : 0;
}

static uint32_t STORE_CODE(kind_bytes)(uint32_t kind)
{
    return kind == STORE_KNOCK ? KNOCK_BYTES : 0;
}

// A kind's record in parts, each a tick's work at most for L2 (the knock maps: a pair's 64 bytes each).
static uint32_t STORE_CODE(kind_parts)(uint32_t kind)
{
    return kind == STORE_KNOCK ? 2 : 1;
}

// ---- Thread mode: the flash's programs and erases ----

// A page programmed (the SDK's, from RAM: the core waits in the boot ROM meanwhile), only the
// interrupts whose code is in flash held off; timed.
static void STORE_CODE(program)(uint32_t p, const uint8_t *page)
{
    Store &s = store_state;
    uint32_t held = nvic_hw->iser & FLASH_IRQS;
    nvic_hw->icer = held;
    uint32_t start = now_us();
    s.program(s.offset + p * PAGE, page, PAGE);
    uint32_t took = now_us() - start;
    nvic_hw->iser = held;
    if (took > s.program_us)
        s.program_us = took;
}

static void STORE_CODE(erase)(uint32_t sector)
{
    Store &s = store_state;
    uint32_t held = nvic_hw->iser & FLASH_IRQS;
    nvic_hw->icer = held;
    uint32_t start = now_us();
    s.erase(s.offset + sector * SECTOR, SECTOR);
    uint32_t took = now_us() - start;
    nvic_hw->iser = held;
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
            for (uint32_t i = 0; i < s.length; i++)
                kept[kind][i] = post_page[HEADER + i];
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
    return kind == STORE_KNOCK && knock_differs(part, kept[kind]);
}

static void STORE_CODE(kind_copy)(uint32_t kind, uint32_t part)
{
    if (kind == STORE_KNOCK)
        knock_copy(part, post_page + HEADER);
}

// Once a tick: a clear the PC asked for; a kind being copied, a part a tick, then posted; else, once a
// second, a part of a kind whose minute has passed looked at (its parts in turn), and copied from the next
// tick on if it differs from what is kept (a forced kind at once: a clear's neutral record).
void STORE_CODE(store_tick)()
{
    Store &s = store_state;
    if (s.posted)
        return;                                                 // thread mode has the post's page
    uint32_t now = now_us();
    if (s.asked[STORE_KNOCK] != s.done[STORE_KNOCK] && knock_stopped())
    {
        knock_clear();                                          // the PC's clear: a neutral record at once
        s.force[STORE_KNOCK] = 1;
        s.done[STORE_KNOCK] = s.asked[STORE_KNOCK];
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
        s.force[kind] = 0;
        __asm volatile("" ::: "memory");                        // the record whole before its post
        s.posted = 1;
        return;
    }
    bool forced = false;
    for (uint32_t kind = 1; kind <= KINDS; kind++)
        forced = forced || s.force[kind];
    if (!forced && now - s.looked_at < SECOND_US)
        return;
    s.looked_at = now;
    for (uint32_t kind = 1; kind <= KINDS; kind++)
    {
        if (s.key[kind] == 0 || (!s.force[kind] && now - s.posted_at[kind] < MINUTE_US))
            continue;
        if (!s.force[kind])
        {
            uint32_t part = s.look[kind];
            s.look[kind] = part + 1 < kind_parts(kind) ? part + 1 : 0;
            if (!kind_differs(kind, part))
                continue;
        }
        s.copying = kind;                                       // copied from the next tick on
        s.copied = 0;
        return;                                                 // one at a time
    }
}

// ---- The board's start ----

void STORE_CODE(store_start)()
{
    Store &s = store_state;
    uint32_t now = now_us();
    for (uint32_t i = 0; i < PAGE; i++)                         // (loops: no memset in the bank)
        commit_page[i] = i < COMMIT ? 0xFF : 0;
    s.posted = s.ahead = 0;
    s.loaded = 0;
    s.looked_at = now;
    s.copying = s.copied = 0;
    s.key[STORE_KNOCK] = knock_key();
    uint32_t best[KINDS + 1] = {}, top = 0, top_page = NONE;
    for (uint32_t kind = 0; kind <= KINDS; kind++)
    {
        s.newest[kind] = NONE;
        s.force[kind] = 0;
        s.look[kind] = 0;
        s.done[kind] = s.asked[kind];
        s.posted_at[kind] = now;
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
        s.loaded = s.loaded | 1u << kind;
    }
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
// kinds asked to be cleared and not yet ("knock": L2 clears the knock maps once the engine has stopped).
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
        strcpyz(value, value_size, store_clears_pending() & 1u << STORE_KNOCK ? "knock" : "none");
        return true;
    }
    return false;
}

// store_clear=knock (or all): the knock maps cleared once the engine has stopped, and a neutral record
// written at once.
bool store_set_parameter(const char *name, const char *value)
{
    if (!streq(name, "store_clear") || !(streq(value, "knock") || streq(value, "all")))
        return false;
    store_ask_clear(1u << STORE_KNOCK);
    return true;
}

#endif // FEATURE_MAP_UNIT
