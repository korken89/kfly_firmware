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
                                        (current_page++ * FLASH_PAGE_SIZE),
                                        seek_union.raw_data,
                                        5);
    } while((seek_union.formated_seek.id != uid) &&
            (seek_union.formated_seek.id != FLASHSAVE_UNALLOCATED) &&
            (current_page <= flashcfg.num_pages));

    /* Return the page number */
    if (page_number != NULL)
        *page_number = current_page - 1;

    /* If there was a match return the size */
    if (seek_union.formated_seek.id == uid)
    {
        if (size != NULL)
            *size = seek_union.formated_seek.size;

        return FLASHSAVE_MATCH;
    }
    else
        return FLASHSAVE_NO_MATCH;
}

void FlashSave_Write(uint32_t uid,
                     bool overwrite,
                     uint8_t *data,
                     uint8_t count)
{
    (void)data;
    (void)count;

    int16_t page_number;
    uint8_t size;
    bool result = FlashSave_Seek(uid, &page_number, &size);

    if (result == FLASHSAVE_NO_MATCH)
    {
        /* No previous data, save at the end of the flash memory */

    }
    else
    {
        /* Previous data available */
        if (overwrite == true)
        {
            /* Overwrite old data */

        }
    }
}

bool FlashSave_Read(uint32_t uid, uint8_t *data, int16_t requested_size)
{
    (void)data;
    int16_t page_number;
    uint8_t size;

    /* Look for the specified UID in the external memory */
    bool result = FlashSave_Seek(uid, &page_number, &size);

    if (result == FLASHSAVE_MATCH)
    {
        if (size == requested_size)
        {
            /* The requested data is available, read it */

            return FLASHSAVE_MATCH;
        }
        else
            return FLASHSAVE_NO_MATCH;
    }
    else
        return FLASHSAVE_NO_MATCH;
}
