#ifndef IO_BOOT_ID_H
#define IO_BOOT_ID_H
#include <stdbool.h>
#include <stdint.h>
/* F103C8: last two 1 KiB pages are outside the linked application. */
#define IO_BOOT_BASE 0x0800F800UL
#define IO_BOOT_PAGE_SIZE 1024U
#define IO_BOOT_RECORD_SIZE 12U
#define IO_BOOT_COMMIT 0xB0071D42UL
bool IoBootId_Next(uint32_t *boot_id);
/* Flash adapter. Only used at startup, before serial/actuator service. */
uint32_t IoBootFlash_Read(uint32_t address);
bool IoBootFlash_Erase(uint32_t page_address);
bool IoBootFlash_Write(uint32_t address, uint32_t value);
#endif
