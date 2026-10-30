#include "io_boot_id.h"
#include <stddef.h>

bool IoBootId_Next(uint32_t *boot_id)
{
    uint32_t page, offset, maximum = 0U, latest_page = 0U;
    uint32_t free_address[2] = {0U, 0U}, address, next;
    if (boot_id == NULL) { return false; }
    *boot_id = 0U;
    for (page = 0U; page < 2U; ++page) {
        for (offset = 0U; offset + IO_BOOT_RECORD_SIZE <= IO_BOOT_PAGE_SIZE; offset += IO_BOOT_RECORD_SIZE) {
            uint32_t value, inverse, commit;
            address = IO_BOOT_BASE + page * IO_BOOT_PAGE_SIZE + offset;
            value = IoBootFlash_Read(address);
            inverse = IoBootFlash_Read(address + 4U);
            commit = IoBootFlash_Read(address + 8U);
            if (value == 0xFFFFFFFFUL && inverse == 0xFFFFFFFFUL && commit == 0xFFFFFFFFUL) {
                if (free_address[page] == 0U) { free_address[page] = address; }
            } else if (value != 0U && inverse == ~value && commit == IO_BOOT_COMMIT && value > maximum) {
                maximum = value; latest_page = page;
            }
        }
    }
    if (maximum == 0xFFFFFFFFUL) { return false; } /* Never reuse an epoch. */
    next = maximum + 1U;
    address = free_address[latest_page];
    if (address == 0U) {
        uint32_t other = 1U - latest_page;
        address = free_address[other];
        if (address == 0U) {
            /* The page containing the last committed epoch is never erased. */
            address = IO_BOOT_BASE + other * IO_BOOT_PAGE_SIZE;
            if (!IoBootFlash_Erase(address)) { return false; }
        }
    }
    /* Commit last; a torn write consumes the slot but cannot activate a boot. */
    if (!IoBootFlash_Write(address, next) || !IoBootFlash_Write(address + 4U, ~next) ||
        !IoBootFlash_Write(address + 8U, IO_BOOT_COMMIT)) { return false; }
    if (IoBootFlash_Read(address) != next || IoBootFlash_Read(address + 4U) != ~next ||
        IoBootFlash_Read(address + 8U) != IO_BOOT_COMMIT) { return false; }
    *boot_id = next;
    return true;
}
