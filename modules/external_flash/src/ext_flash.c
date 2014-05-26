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

/* Global variable defines */

/* Private variable defines */

/* Area to hold the design of the external flash memory */
static const Flash_Save_Template Flash_Save_Structure[] = {
    {   /* To indicate the end of the save structure */
        .ptr = FLASH_END,
        .count = FLASH_END
    }
};

/* Private function defines */
static uint32_t SaveStructure_NumberOfBytes(const Flash_Save_Template *template);
static void SaveStructure_WriteToMemory(const Flash_Save_Template *template,
                                        uint32_t sector);

/* Private external functions */


void ExternalFlashInit(void)
{
    uint32_t id;

    id = ExternalFlash_ReadID();

    if (id != FLASH_M25PE40_ID)
        chSysHalt("External Flash ID error.");
}

void ExternalFlash_EraseBulk(void)
{
    /* Enable the write access to the External Flash */
    ExternalFlash_WriteEnable();

    /* Select the External Flash: Chip Select low */
    FLASH_CS_LOW();

    /* Send Bulk Erase instruction  */
    spiPolledExchange(EXTERNAL_FLASH_SPI, FLASH_CMD_BE);

    /* Deselect the External Flash: Chip Select high */
    FLASH_CS_HIGH();

    /* Wait the end of Flash writing */
    ExternalFlash_WaitForWriteEnd();
}

void ExternalFlash_EraseSector(uint32_t sector)
{
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
    ExternalFlash_WaitForWriteEnd();
}

/**
 * @brief       Gets the ID of the External Flash.
 *
 * @return      External Flash ID.
 */
uint32_t ExternalFlash_ReadID(void)
{
    uint32_t t1 = 0, t2 = 0, t3 = 0;

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

    return ((t1 << 16) | (t2 << 8) | t3);
}


ErrorStatus ExternalFlash_CheckIntegrity(const Flash_Save_Template *template,
                                         uint32_t sector)
{
    uint32_t i, page_address;
    union
    {
        uint8_t data[4];
        uint32_t value;
    } sync_data;

    i = 0;

    /* Calculate the starting sector address */
    page_address = sector * FLASH_SECTOR_SIZE;

    /* Check so we read from one of the sectors */
    if (sector > FLASH_NUM_SECTORS)
        return ERROR;

    while (template[i].ptr != FLASH_END)
    {
        /* Read the sync word from the flash */
        ExternalFlash_ReadBuffer(sync_data.data, page_address, 4);

        /* If the sync word is not correct the memory is fragmented.
           Return error. */
        if (sync_data.value != FLASH_SYNC_WORD_REV)
            return ERROR;

        /* Increment Flash address and template address */
        page_address += template[i].count + 4; /* The 4 comes from the SYNC */
        i++;
    }

    return SUCCESS;
}


ErrorStatus ExternalFlash_SaveSettings(const Flash_Save_Template *template,
                                       uint32_t sector)
{
    /* Check so we write to one of the sectors */
    if (sector > FLASH_NUM_SECTORS)
        return ERROR;

    /* Erase the selected sector */
    ExternalFlash_EraseSector(sector * FLASH_SECTOR_SIZE);

    /* Start saving data to the external flash memory */
    SaveStructure_WriteToMemory(template, sector);

    return SUCCESS;
}


ErrorStatus ExternalFlash_LoadSettings(const Flash_Save_Template *template, 
                                       uint32_t sector)
{
    uint32_t i, page_address;

    i = 0;

    /* Calculate the starting sector address */
    page_address = sector * FLASH_SECTOR_SIZE + 4;

    /* Check so we read from one of the sectors */
    if (sector > FLASH_NUM_SECTORS)
        return ERROR;

    while (template[i].ptr != FLASH_END)
    {
        /* Read the setting from the flash */
        ExternalFlash_ReadBuffer(template[i].ptr, page_address, template[i].count);

        /* Increment Flash address and template address */
        page_address += template[i].count + 4; /* The 4 comes from the SYNC */
        i++;
    }

    return SUCCESS;
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
    /* Enable the write access to the External Flash */
    ExternalFlash_WriteEnable();

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

    /* Wait the end of Flash writing */
    ExternalFlash_WaitForWriteEnd();
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
}

/**
 * @brief   Enables the write access to the External Flash.
 */
void ExternalFlash_WriteEnable(void)
{
    /* Select the External Flash: Chip Select low */
    FLASH_CS_LOW();

    /* Send "Write Enable" instruction */
    spiPolledExchange(EXTERNAL_FLASH_SPI, FLASH_CMD_WREN);

    /* Deselect the External Flash: Chip Select high */
    FLASH_CS_HIGH();
}

/**
 * @brief       Polls the status of the Write In Progress (WIP) flag in the External
 *              Flash's status register until write operation has completed.
 */
void ExternalFlash_WaitForWriteEnd(void)
{
    /* Select the External Flash: Chip Select low */
    FLASH_CS_LOW();

    /* Send "Read Status Register" instruction */
    spiPolledExchange(EXTERNAL_FLASH_SPI, FLASH_CMD_RDSR);

    /* Loop as long as the memory is busy with a write cycle */
    while (spiPolledExchange(EXTERNAL_FLASH_SPI, FLASH_DUMMY_BYTE) & FLASH_WIP_FLAG);

    /* Deselect the External Flash: Chip Select high */
    FLASH_CS_HIGH();
}


static uint32_t SaveStructure_NumberOfBytes(const Flash_Save_Template *template)
{
    uint32_t i = 0, num_bytes = 0;

    /* While there is an valid pointer add its byte count to the total */
    while (template[i].ptr != 0)
        num_bytes += template[i++].count;

    return num_bytes;
}

/**
 * @brief           Processes a Flash_Save structure to determine the
 *                  correct way of writing it to memory.
 * 
 * @param template  Pointer to the Flash_Save template.
 * @return          Number of bytes to save.
 */
static void SaveStructure_WriteToMemory(const Flash_Save_Template *template, uint32_t sector)
{
    int32_t i, j, num_bytes_written_to_page, page_address;

    i = 0;
    num_bytes_written_to_page = 0;
    page_address = sector * FLASH_SECTOR_SIZE;

    /* Enable the write access to the External Flash */
    ExternalFlash_WriteEnable();

    /* Select the External Flash: Chip Select low */
    FLASH_CS_LOW();

    /* Send "Write to Memory" instruction */
    spiPolledExchange(EXTERNAL_FLASH_SPI, FLASH_CMD_PAGE_PROGRAM);

    /* Send address nibbles for address byte to write to */
    spiPolledExchange(EXTERNAL_FLASH_SPI, (page_address & 0xFF0000) >> 16);
    spiPolledExchange(EXTERNAL_FLASH_SPI, (page_address & 0xFF00) >> 8);
    spiPolledExchange(EXTERNAL_FLASH_SPI, page_address & 0xFF);

    while (template[i].ptr != FLASH_END)
    {
        for (j = -4; j < template[i].count; j++)
        {
            if (++num_bytes_written_to_page >= FLASH_PAGE_SIZE)
            {
                /* Wait for write complete and start a new write cycle */

                /* Deselect the External Flash: Chip Select high */
                FLASH_CS_HIGH();

                /* Wait the end of Flash writing */
                ExternalFlash_WaitForWriteEnd();

                /* Start new write cycle and increment the address */
                page_address += FLASH_PAGE_SIZE;

                /* Enable the write access to the External Flash */
                ExternalFlash_WriteEnable();

                /* Select the External Flash: Chip Select low */
                FLASH_CS_LOW();

                /* Send "Write to Memory" instruction */
                spiPolledExchange(EXTERNAL_FLASH_SPI, FLASH_CMD_PAGE_PROGRAM);

                /* Send address nibbles for address byte to write to */
                spiPolledExchange(EXTERNAL_FLASH_SPI, (page_address & 0xFF0000) >> 16);
                spiPolledExchange(EXTERNAL_FLASH_SPI, (page_address & 0xFF00) >> 8);
                spiPolledExchange(EXTERNAL_FLASH_SPI, page_address & 0xFF);

                /* Set to one for the byte to be written directly after */
                num_bytes_written_to_page = 1;
            }

            /* Send SYNC first, then the data */


            if (j < 0)
                spiPolledExchange(EXTERNAL_FLASH_SPI, (FLASH_SYNC_WORD >> (-j - 1) * 8) & 0xff);
            else
                spiPolledExchange(EXTERNAL_FLASH_SPI, template[i].ptr[j]);
        }

        i++;
    }

    /* Deselect the External Flash: Chip Select high */
    FLASH_CS_HIGH();

    /* Wait the end of Flash writing */
    ExternalFlash_WaitForWriteEnd();
}