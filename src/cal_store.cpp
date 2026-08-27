#include <Arduino.h>
#include <string.h>
#include "cal_store.h"

// Device geometry comes from the HAL headers rather than being written out
// here, so a different H7 variant does not silently get the wrong sector size
// or the wrong programming width. The fallbacks only fire if a future HAL
// stops defining them.
#ifndef FLASH_SECTOR_SIZE
#define FLASH_SECTOR_SIZE   0x20000UL          // 128 KB
#endif
#ifndef FLASH_SECTOR_TOTAL
#define FLASH_SECTOR_TOTAL  8U
#endif
#ifndef FLASH_NB_32BITWORD_IN_FLASHWORD
#define FLASH_NB_32BITWORD_IN_FLASHWORD 8U     // 256-bit flash word
#endif

#define FLASHWORD_BYTES     (FLASH_NB_32BITWORD_IN_FLASHWORD * 4U)
#define CFG_SECTOR          (FLASH_SECTOR_TOTAL - 1U)
#define CFG_BASE            (FLASH_BASE + (uint32_t)CFG_SECTOR * FLASH_SECTOR_SIZE)

// 'T89C'. A blank sector reads 0xFF everywhere, so any header that is not this
// exact pair means "nothing stored", not "stored and broken".
#define CFG_MAGIC           0x54383943UL

typedef struct {
    uint32_t magic;
    uint32_t len;
} CfgHeader;

uint32_t cal_store_base() { return CFG_BASE; }

bool cal_store_read(void *blob, uint32_t len) {
    const CfgHeader *h = (const CfgHeader *)CFG_BASE;
    if (h->magic != CFG_MAGIC || h->len != len) return false;
    memcpy(blob, (const void *)(CFG_BASE + sizeof(CfgHeader)), len);
    return true;
}

bool cal_store_write(const void *blob, uint32_t len) {
    if (sizeof(CfgHeader) + len > FLASH_SECTOR_SIZE) return false;

    // Staged one flash word at a time: the H7 programs 256 bits atomically and
    // will not accept a partial word, and the source has to be word-aligned.
    static uint8_t wbuf[FLASHWORD_BYTES] __attribute__((aligned(32)));

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef e = {};
    e.TypeErase    = FLASH_TYPEERASE_SECTORS;
    e.Banks        = FLASH_BANK_1;
    e.Sector       = CFG_SECTOR;
    e.NbSectors    = 1;
    e.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t sectorError = 0;
    if (HAL_FLASHEx_Erase(&e, &sectorError) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    CfgHeader h = { CFG_MAGIC, len };

    // Header and payload are one contiguous byte stream, cut into flash words.
    const uint32_t total = sizeof(h) + len;
    uint32_t addr = CFG_BASE;
    bool ok = true;

    for (uint32_t off = 0; off < total && ok; off += FLASHWORD_BYTES) {
        memset(wbuf, 0xFF, sizeof(wbuf));      // pad the tail with erased bytes
        for (uint32_t i = 0; i < FLASHWORD_BYTES && (off + i) < total; i++) {
            uint32_t src = off + i;
            wbuf[i] = (src < sizeof(h)) ? ((const uint8_t *)&h)[src]
                                        : ((const uint8_t *)blob)[src - sizeof(h)];
        }
        // On H7 the third argument is the ADDRESS of the data, not the data.
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr,
                               (uint64_t)(uint32_t)wbuf) == HAL_OK;
        addr += FLASHWORD_BYTES;
    }

    HAL_FLASH_Lock();

    // If the cache is on, it may still be holding the pre-erase contents of
    // this region. Drop them so the read-back below sees flash, not history.
    SCB_InvalidateDCache_by_Addr((uint32_t *)CFG_BASE,
                                 (int32_t)(total + FLASHWORD_BYTES));

    if (!ok) return false;

    // Read back before claiming success. A write that reports OK and stores
    // something else is the failure mode worth catching here.
    return memcmp((const void *)(CFG_BASE + sizeof(CfgHeader)), blob, len) == 0;
}
