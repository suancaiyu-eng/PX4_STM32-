#include "io_boot_id.h"
#include "main.h"

uint32_t IoBootFlash_Read(uint32_t address) { return *(volatile const uint32_t *)address; }
bool IoBootFlash_Erase(uint32_t address)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t error = 0U;
    bool ok;
    if ((address != IO_BOOT_BASE && address != IO_BOOT_BASE + IO_BOOT_PAGE_SIZE) ||
        HAL_FLASH_Unlock() != HAL_OK) { return false; }
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = address;
    erase.NbPages = 1U;
    ok = HAL_FLASHEx_Erase(&erase, &error) == HAL_OK;
    (void)HAL_FLASH_Lock();
    return ok;
}
bool IoBootFlash_Write(uint32_t address, uint32_t value)
{
    bool ok;
    if (address < IO_BOOT_BASE || address >= IO_BOOT_BASE + 2U * IO_BOOT_PAGE_SIZE ||
        (address & 3U) != 0U || HAL_FLASH_Unlock() != HAL_OK) { return false; }
    ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, value) == HAL_OK;
    (void)HAL_FLASH_Lock();
    return ok;
}
