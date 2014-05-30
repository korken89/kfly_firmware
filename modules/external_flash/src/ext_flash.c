/* *
 *
 * Hardware Abstraction Layer for External Flash
 *
 * */

#include "ch.h"
#include "hal.h"
#include "ext_flash.h"
#include "sensor_read.h"
#include "control.h"
#include "rc_input.h"

/* Claim and release macros for external flash memory */
#define ExternalFlash_Claim()    chMtxLock(&ext_flash_lock)
#define ExternalFlash_Release()  chMtxUnlock(&ext_flash_lock)


/* Global variable defines */

/* Private variable defines */
static mutex_t ext_flash_lock;

/* Private function defines */
static void ExternalFlash_WaitForWriteEnd(uint32_t delay_ms);
static void ExternalFlash_WriteEnable(void);

/* Private external functions */


void ExternalFlashInit(void)
{
    uint32_t id;

    chMtxObjectInit(&ext_flash_lock);

    id = ExternalFlash_ReadID();

    if (id != FLASH_M25PE40_ID)
        chSysHalt("External Flash ID error.");
}

void ExternalFlash_EraseBulk(void)
{
    /* Claim external flash */
    ExternalFlash_Claim();

    /* Enable the write access to the External Flash */
    ExternalFlash_WriteEnable();

    /* Select the External Flash: Chip Select low */
    FLASH_CS_LOW();

    /* Send Bulk Erase instruction  */
    spiPolledExchange(EXTERNAL_FLASH_SPI, FLASH_CMD_BE);

    /* Deselect the External Flash: Chip Select high */
    FLASH_CS_HIGH();

    /* Wait the end of Flash writing */
    ExternalFlash_WaitForWriteEnd(100);

    /* Release external flash */
    ExternalFlash_Release();
}

void ExternalFlash_EraseSector(uint32_t sector)
{
    /* Claim external flash */
    ExternalFlash_Claim();

    /* Enable the write access to the External Flash */
    ExternalFlash_WriteEnable();

    /* Select the External Flash: Chip Select low */
    FLASH_CS_LOW();

    /* Send Bulk Erase instruction  */
    spiPolledExchange(EXTERNAL_FLASH_SPI, FLASH_CMD_SE);
    
    /* Send address high nibbles */
    spiPolledExchange(EXTERNAL_FLASH_SPI, (sector & 0xFF0000) >> 16);
    spiPolledExchange(EXTERNAL_FLASH_SPI, (sector & 0xFF00) >> 8);
    spiPolledExchange(EXTERNAL_FLASH_SPI, sector & 0xFF);
    
    /* Deselect the External Flash: Chip Select high */
    FLASH_CS_HIGH();

    /* Wait the end of Flash writing */
    ExternalFlash_WaitForWriteEnd(100);

    /* Release external flash */
    ExternalFlash_Release();
}

/**
 * @brief       Gets the ID of the External Flash.
 *
 * @return      External Flash ID.
 */
uint32_t ExternalFlash_ReadID(void)
{
    uint32_t t1, t2, t3;

    /* Claim external flash */
    ExternalFlash_Claim();

    /* Claim the SPI bus */
    spiAcquireBus(EXTERNAL_FLASH_SPI);

    /* Select the External Flash: Chip Select low */
    FLASH_CS_LOW();

    /* Send "Read ID" instruction */
    spiPolledExchange(EXTERNAL_FLASH_SPI, FLASH_CMD_RDID);

    /* Read the three ID bytes from the External Flash */
    t1 = spiPolledExchange(EXTERNAL_FLASH_SPI, FLASH_DUMMY_BYTE);
    t2 = spiPolledExchange(EXTERNAL_FLASH_SPI, FLASH_DUMMY_BYTE);
    t3 = spiPolledExchange(EXTERNAL_FLASH_SPI, FLASH_DUMMY_BYTE);

    /* Deselect the External Flash: Chip Select high */
    FLASH_CS_HIGH();

    /* Release the SPI bus */
    spiReleaseBus(EXTERNAL_FLASH_SPI);

    /* Release external flash */
    ExternalFlash_Release();

    return ((t1 << 16) | (t2 << 8) | t3);
}

/**
 * @brief           Writes data to a Flash page.
 * 
 * @param buffer    Pointer to the buffer holding the data.
 * @param address   Where in the Flash to save the data.
 * @param count     Number of bytes to write (max 256 bytes).
 */
void ExternalFlash_WritePage(uint8_t *buffer,
                             uint32_t address, 
                             uint16_t count)
{
    /* Claim external flash */
    ExternalFlash_Claim();

    /* Enable the write access to the External Flash */
    ExternalFlash_WriteEnable();

    /* Claim the SPI bus */
    spiAcquireBus(EXTERNAL_FLASH_SPI);

    /* Select the External Flash: Chip Select low */
    FLASH_CS_LOW();

    /* Send "Write to Memory" instruction */
    spiPolledExchange(EXTERNAL_FLASH_SPI, FLASH_CMD_PAGE_PROGRAM);

    /* Send address nibbles for address byte to write to */
    spiPolledExchange(EXTERNAL_FLASH_SPI, (address & 0xFF0000) >> 16);  
    spiPolledExchange(EXTERNAL_FLASH_SPI, (address & 0xFF00) >> 8);
    spiPolledExchange(EXTERNAL_FLASH_SPI, address & 0xFF);

    /* While there is data to be written on the External Flash */
    while (count--)
        spiPolledExchange(EXTERNAL_FLASH_SPI, *(buffer++));

    /* Deselect the External Flash: Chip Select high */
    FLASH_CS_HIGH();

    /* Release the SPI bus */
    spiReleaseBus(EXTERNAL_FLASH_SPI);

    /* Wait the end of Flash writing */
    ExternalFlash_WaitForWriteEnd(1);

    /* Release external flash */
    ExternalFlash_Release();
}

/**
 * @brief           Read a block of data from the External Flash.
 * 
 * @param buffer    Pointer to the buffer saving the data.
 * @param address   Where in the Flash to read the data.
 * @param count     Number of bytes to read.
 */
void ExternalFlash_ReadBuffer(uint8_t *buffer, 
                              uint32_t address, 
                              uint16_t count)
{
    /* Claim external flash */
    ExternalFlash_Claim();

    /* Claim the SPI bus */
    spiAcquireBus(EXTERNAL_FLASH_SPI);

    /* Select the External Flash: Chip Select low */
    FLASH_CS_LOW();

    /* Send "Read from Memory" instruction */
    spiPolledExchange(EXTERNAL_FLASH_SPI, FLASH_CMD_READ);

    /* Send address nibbles for address byte to read from */
    spiPolledExchange(EXTERNAL_FLASH_SPI, (address & 0xFF0000) >> 16);
    spiPolledExchange(EXTERNAL_FLASH_SPI, (address& 0xFF00) >> 8);
    spiPolledExchange(EXTERNAL_FLASH_SPI, address & 0xFF);

    while (count--)
        *(buffer++) = spiPolledExchange(EXTERNAL_FLASH_SPI, FLASH_DUMMY_BYTE);

    /* Deselect the External Flash: Chip Select high */
    FLASH_CS_HIGH();

    /* Release the SPI bus */
    spiReleaseBus(EXTERNAL_FLASH_SPI);

    /* Release external flash */
    ExternalFlash_Release();
}

/**
 * @brief       Polls the status of the Write In Progress (WIP) flag in the External
 *              Flash's status register until write operation has completed.
 */
static void ExternalFlash_WaitForWriteEnd(uint32_t delay_ms)
{
    uint8_t wip;

    /* Loop as long as the memory is busy with a write cycle */
    do
    {
        /* If a delay was specified: wait */
        if (delay_ms != 0)
            chThdSleep(OSAL_MS2ST(delay_ms));

        /* Claim the SPI bus */
        spiAcquireBus(EXTERNAL_FLASH_SPI);

        /* Select the External Flash: Chip Select low */
        FLASH_CS_LOW();

        /* Send "Read Status Register" instruction */
        spiPolledExchange(EXTERNAL_FLASH_SPI, FLASH_CMD_RDSR);

        wip = spiPolledExchange(EXTERNAL_FLASH_SPI, FLASH_DUMMY_BYTE);

        /* Deselect the External Flash: Chip Select high */
        FLASH_CS_HIGH();

        /* Release the SPI bus */
        spiReleaseBus(EXTERNAL_FLASH_SPI); 
    } while (wip & FLASH_WIP_FLAG);
}

/**
 * @brief   Enables the write access to the External Flash.
 */
static void ExternalFlash_WriteEnable(void)
{
    /* Claim the SPI bus */
    spiAcquireBus(EXTERNAL_FLASH_SPI);

    /* Select the External Flash: Chip Select low */
    FLASH_CS_LOW();

    /* Send "Write Enable" instruction */
    spiPolledExchange(EXTERNAL_FLASH_SPI, FLASH_CMD_WREN);

    /* Deselect the External Flash: Chip Select high */
    FLASH_CS_HIGH();

    /* Release the SPI bus */
    spiReleaseBus(EXTERNAL_FLASH_SPI);
}