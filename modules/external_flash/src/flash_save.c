/* *
 *
 * Abstraction Layer for Save to Flash functionality
 *
 * */

 /* Structure of the external flash:
  *  __________ __________  _  _  _  __________
  * |          |          |         |          |
  * |  Save 1  |  Save 2  | -  -  - |  Save N  |
  * |__________|__________| _  _  _ |__________|
  *                                /            \  
  *    _ _ _ _ _ _ _ _ _ _ _ _ _ _/              \_ _ _ _ _ 
  *  /                                                     \
  * /__________________________ _____________ ______________\
  * |                          |             |              |
  * |  32-bit Unique ID (UID)  |  Data size  |  Saved Data  |
  * |__________________________|_____________|______________|
  *            4 bytes              1 byte      1-250 bytes
  *            
  * Each block can only span one page as maximum limiting the saved data to be
  * maximum 250 bytes.
  * 
  * - The worst case time to complete a save is approximately 50 ms.
  * - The worst case time to complete a read is approximately 35 ms.
  */

#include "ch.h"
#include "hal.h"
#include "flash_save.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

/*
 * External Flash configuration
 */
ExternalFlashData flash_data;
static const ExternalFlashConfig flashcfg = {
    &SPID1,             /* SPI driver connected to the external flash.       */
    GPIOC,              /* GPIO port where the chip select is connected.     */
    GPIOC_FLASH_SEL,    /* GPIO pin where the chip select is connected.      */
    M25PE40_NUM_PAGES,  /* Number of pages available on the flash memory.    */
    FLASH_M25PE40_ID,   /* JEDEC and manufacturer ID of the flash memory.    */
    &flash_data         /* Pointer to mutex and temporary data structure.    */
};

EVENTSOURCE_DECL(save_to_flash_es);

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief               Writes data to a Flash page using DMA.
 * @note                This assumes that the page to be written to has been
 *                      erased prior to the call to this function.
 *
 * @param[in] config    Pointer to External Flash config.
 * @param[in] address   Where in the Flash to save the data.
 * @param[in] uid       The unique ID.
 * @param[in] buffer    Pointer to the buffer holding the data.
 * @param[in] count     Number of bytes to write (max 256 bytes).
 */
static void FlashSave_WritePage(const ExternalFlashConfig *config,
                                uint32_t address, 
                                uint32_t uid,
                                uint8_t *buffer,
                                uint16_t count)
{
    /* Error check. */
    if (count > FLASH_PAGE_SIZE)
        osalSysHalt("Page write size too big");

#if SPI_USE_MUTUAL_EXCLUSION
    /* Claim the SPI bus */
    spiAcquireBus(config->spip);
#endif /* SPI_USE_MUTUAL_EXCLUSION */

    /* Enable the write access to the External Flash */
    ExternalFlash_WriteEnable(config);

    /* Select the External Flash: Chip Select low */
    ExternalFlash_Select(config);

    /* Load the command and address data */
    config->data->flash_tmp[0] = (uint8_t)FLASH_CMD_PAGE_PROGRAM;
    config->data->flash_tmp[1] = (uint8_t)((address & 0xFF0000) >> 16);
    config->data->flash_tmp[2] = (uint8_t)((address & 0xFF00) >> 8);
    config->data->flash_tmp[3] = (uint8_t)(address & 0xFF);

    /* Send "Write to Memory" instruction and send address nibbles
       from address to read from */
    spiSend(config->spip, 4, config->data->flash_tmp);

    /* Send the header consisting of UID + size (5 bytes) */
    spiPolledExchange(config->spip, (uint8_t)(uid & 0x000000FF));
    spiPolledExchange(config->spip, (uint8_t)((uid & 0x0000FF00) >> 8));
    spiPolledExchange(config->spip, (uint8_t)((uid & 0x00FF0000) >> 16));
    spiPolledExchange(config->spip, (uint8_t)((uid & 0xFF000000) >> 24));
    spiPolledExchange(config->spip, (uint8_t)count);

    /* Send the data to memory using DMA */
    spiSend(config->spip, count, buffer);

    /* Deselect the External Flash: Chip Select high */
    ExternalFlash_Unselect(config);

#if SPI_USE_MUTUAL_EXCLUSION
    /* Release the SPI bus */
    spiReleaseBus(config->spip);
#endif /* SPI_USE_MUTUAL_EXCLUSION */

    /* Wait the end of Flash writing */
    ExternalFlash_WaitForWriteEnd(config, 1);
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes the External Flash.
 */
void FlashSaveInit(void)
{
    /* Initialize Save to Flash event source */
    osalEventObjectInit(&save_to_flash_es);

    /* Initialize external flash */
    if (ExternalFlashInit(&flashcfg) != MSG_OK)
        osalSysHalt("External Flash ID error.");
}

/**
 * @brief       Seek the flash memory for the requested UID and reports back
 *              the page number and size of the saved data.
 *  
 * @param[in]  uid          UID to search for.
 * @param[out] page_number  Pointer to saving variable for page number.
 * @param[out] size         Pointer to saving variable for data size.
 * @return      Returns true if there was a match.
 */
bool FlashSave_Seek(uint32_t uid, int16_t *page_number, uint8_t *size)
{
    uint16_t current_page = 0;

    /* Union to format raw data from the flash memory */
    union {
        struct {
            uint32_t id;
            uint8_t size;
        } formated_seek;
        uint8_t raw_data[5];
    } seek_union;

    /* Search each page for the correct UID or until the end of allocated
       memory is reached. */
    do {
        ExternalFlash_ReadBufferPolling(&flashcfg,
                                        (current_page * FLASH_PAGE_SIZE),
                                        seek_union.raw_data,
                                        5);
        
        current_page++;
    } while((seek_union.formated_seek.id != uid) &&
            (seek_union.formated_seek.id != FLASHSAVE_UNALLOCATED) &&
            (current_page <= flashcfg.num_pages));

    /* Return the page number */
    if (page_number != NULL)
    {
        /* If the flash is full return -1 */
        if (current_page >= flashcfg.num_pages)
            *page_number = -1;
        else
            *page_number = current_page - 1;
    }

    /* If there was a match return the size */
    if (seek_union.formated_seek.id == uid)
    {
        if (size != NULL)
            *size = seek_union.formated_seek.size;

        return true;
    }
    else
        return false;
}

/**
 * @brief       Writes data to a location with the correct UID, or saved at a 
 *              new location if the UID was not already written. 
 * @note        Max write size is 250 bytes.
 * 
 * @param[in] uid       UID to write at.
 * @param[in] overwrite True to overwrite old data.
 * @param[in] data      Pointer to the data to write.
 * @param[in] count     Number of bytes to write.
 * @return      Returns the status of the operation.
 */
FlashSave_Status FlashSave_Write(uint32_t uid,
                                 bool overwrite,
                                 uint8_t *data,
                                 uint16_t count)
{
    bool result;
    int16_t page_number;
    uint8_t size;

    /* Claim external flash */
    ExternalFlash_Claim(&flashcfg);

    /*  Check if the data is within correct size */
    if (count > 250)
        return FLASHSAVE_OVERSIZE;

    result = FlashSave_Seek(uid, &page_number, &size);

    /* Check if the external flash is full */
    if (page_number == -1)
        return FLASHSAVE_FLASH_FULL;

    if (result == false)
    {
        /* No previous data, save at the end of the flash memory.
           Just in case erase the page. */
        ExternalFlash_ErasePage(&flashcfg, page_number * FLASH_PAGE_SIZE);
        FlashSave_WritePage(&flashcfg,
                            page_number * FLASH_PAGE_SIZE,
                            uid,
                            data,
                            count);
    }
    else
    {
        /* Previous data available */
        if (overwrite == true)
        {
            /* Overwrite old data */
            ExternalFlash_ErasePage(&flashcfg, page_number * FLASH_PAGE_SIZE);
            FlashSave_WritePage(&flashcfg,
                                page_number * FLASH_PAGE_SIZE,
                                uid,
                                data,
                                count);

            ExternalFlash_Release(&flashcfg);

            return FLASHSAVE_NO_OVERWRITE;
        }
    }

    /* Release external flash */
    ExternalFlash_Release(&flashcfg);

    return FLASHSAVE_OK;
}

/**
 * @brief       Reads data from a location with the correct UID. 
 * 
 * @param[in] uid               UID to read from.
 * @param[in] data              Pointer to the save location of the data.
 * @param[in] requested_size    Number of bytes to write.
 * @return      Returns the status of the operation.
 */
FlashSave_Status FlashSave_Read(uint32_t uid,
                                uint8_t *data,
                                uint8_t requested_size)
{
    int16_t page_number;
    uint8_t size;

    /* Claim external flash */
    ExternalFlash_Claim(&flashcfg);

    /* Look for the specified UID in the external memory */
    bool result = FlashSave_Seek(uid, &page_number, &size);

    if (result == true)
    {
        /* If there was a match check the size */
        if (size == requested_size)
        {
            /* The requested data is available, read it */
            ExternalFlash_ReadBuffer(&flashcfg,
                                     page_number * FLASH_PAGE_SIZE +
                                     FLASHSAVE_HEADER_OFFSET,
                                     data,
                                     size);

            /* Release external flash */
            ExternalFlash_Release(&flashcfg);

            return FLASHSAVE_OK;
        }
        else
        {
            /* Release external flash */
            ExternalFlash_Release(&flashcfg);

            return FLASHSAVE_SIZE_MISSMATCH;
        }
    }
    else
    {
        /* Release external flash */
        ExternalFlash_Release(&flashcfg);

        return FLASHSAVE_NO_MATCH;
    }
}

/**
 * @brief       Broadcasts the save to flash event, signaling all threads using
 *              the save to flash functionality to save the current data.
 */
void vBroadcastFlashSaveEvent(void)
{
    //vFlashSaveEraseSettings();
    
    osalSysLock();

    if (chEvtIsListeningI(&save_to_flash_es))
        chEvtBroadcastFlagsI(&save_to_flash_es, FLASHSAVE_SAVE_EVENTMASK);

    /* osalOsRescheduleS() must be called after a chEvtBroadcastFlagsI() */
    osalOsRescheduleS();

    osalSysUnlock();
}

/**
 * @brief       Returns the pointer to the Flash Save event source.
 * 
 * @return      Pointer to the Flash Save event source.
 */
event_source_t *ptrGetFlashSaveEventSource(void)
{
    return &save_to_flash_es;
}
