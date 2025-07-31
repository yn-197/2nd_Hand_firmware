#include "flash.h"
#include "stm32f7xx_hal.h"

#define FLASH_USER_SECTOR_START 0x081C0000 // use sector 23 for user data
#define NUM_FLOATS 12


void Flash_WriteFloatArray(float* data) {
    HAL_FLASH_Unlock();

    // 消去設定
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t sectorError = 0;

    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 2.7V〜3.6V
    eraseInit.Sector = FLASH_SECTOR_11;
    eraseInit.NbSectors = 1;

    HAL_FLASHEx_Erase(&eraseInit, &sectorError);

    // 書き込み
    for (uint32_t i = 0; i < NUM_FLOATS; i++) {
        uint32_t address = FLASH_USER_SECTOR_START + i * sizeof(float);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, *(uint32_t*)&data[i]);
    }

    HAL_FLASH_Lock();
}

void Flash_ReadFloatArray(float* dest) {
    for (uint32_t i = 0; i < NUM_FLOATS; i++) {
        uint32_t address = FLASH_USER_SECTOR_START + i * sizeof(float);
        dest[i] = *(float*)address;
    }
}