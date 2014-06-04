/* *
 *
 * Abstraction Layer for Save to Flash functionality
 *
 * */

 /* Structure of the external flash:
  *  _________ _________
  * |         |         |
  * |  Save1  |  Save2  | ......
  * |_________|_________|
  *
  * Each save block:
  *  ____________________ ____________ ____________
  * |                    |            |            |
  * |  32-bit Unique ID  |  Data size | Saved Data |
  * |____________________|____________|____________|
  * 
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

static ExternalFlashData flash_data;
static const ExternalFlashConfig flashcfg = {
    &SPID1,             /* SPI driver connected to the external flash.       */
    GPIOC,              /* GPIO port where the chip select is connected.     */
    GPIOC_FLASH_SEL,    /* GPIO pin where the chip select is connected.      */
    M25PE40_NUM_PAGES,  /* Number of pages available on the flash memory.    */
    FLASH_M25PE40_ID,   /* JEDEC and manufacturer ID of the flash memory.    */
    &flash_data         /* Pointer to mutex and temporary data structure.    */
};

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes the External Flash.
 */
void FlashSaveInit(void)
{
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
        if (current_page <= flashcfg.num_pages)
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
 * @Return      Returns the status of the operation.
 */
FlashSave_Status FlashSave_Write(uint32_t uid,
                                 bool overwrite,
                                 uint8_t *data,
                                 uint16_t count)
{
    bool result;
    int16_t page_number;
    uint8_t size;

    /*  Check if the data is within correct size */
    if (count > 250)
        return FLASHSAVE_OVERSIZE;

    result = FlashSave_Seek(uid, &page_number, &size);

    /* Check if the external flash is full */
    if (page_number == -1)
        return FLASHSAVE_FLASH_FULL;

    if (result == FLASHSAVE_NO_MATCH)
    {
        /* No previous data, save at the end of the flash memory.
           Just in case erase the page. */
        ExternalFlash_ErasePage(&flashcfg, page_number * FLASH_PAGE_SIZE);
        ExternalFlash_WritePage(&flashcfg,
                                page_number * FLASH_PAGE_SIZE,
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
            ExternalFlash_WritePage(&flashcfg,
                                    page_number * FLASH_PAGE_SIZE,
                                    data,
                                    count);
        }
        else
            return FLASHSAVE_NO_OVERWRITE;
    }

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

    /* Look for the specified UID in the external memory */
    bool result = FlashSave_Seek(uid, &page_number, &size);

    if (result == true)
    {
        if (size == requested_size)
        {
            /* The requested data is available, read it */
            ExternalFlash_ReadBuffer(&flashcfg,
                                     page_number * FLASH_PAGE_SIZE,
                                     data,
                                     size);

            return FLASHSAVE_OK;
        }
        else
            return FLASHSAVE_SIZE_MISSMATCH;
    }
    else
        return FLASHSAVE_NO_MATCH;
}
