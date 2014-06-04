#ifndef __FLASH_SAVE_H
#define __FLASH_SAVE_H

#include "ext_flash.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#define FLASHSAVE_UNALLOCATED       0xFFFFFFFF
#define FLASHSAVE_HEADER_OFFSET     5

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/
/**
 * @brief   Possible Flash Save statuses.
 */
typedef enum
{
    /**
     * @brief   If everything completed successfully.
     */
    FLASHSAVE_OK = 0,
    /**
     * @brief   The requested write or read is over the maximum limit.
     */
    FLASHSAVE_OVERSIZE = 1,
    /**
     * @brief   There was no match.
     */
    FLASHSAVE_NO_MATCH = 2,
    /**
     * @brief   If there was already data with an UID and overwrite was
     *          not allowed.
     */
    FLASHSAVE_NO_OVERWRITE = 3,
    /**
     * @brief   Status if the requested size and actual size does not match.
     */
    FLASHSAVE_SIZE_MISSMATCH = 4,
    /**
     * @brief   If the external flash is out of memory.
     */
    FLASHSAVE_FLASH_FULL = 5,
} FlashSave_Status;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/
/**
 * @brief           Converts a string to a 32-bit ID.
 * 
 * @param str[id]   Input string.
 * @return          Returns 32-bit ID.
 */
static inline uint32_t FlashSave_STR2ID(char str[4])
{
    return (((uint32_t)(str[0]) << 0)  |
            ((uint32_t)(str[1]) << 8)  |
            ((uint32_t)(str[2]) << 16) |
            ((uint32_t)(str[3]) << 24));
}

/**
 * @brief           Converts four bytes to a 32-bit ID.
 * 
 * @param[in] b1    First byte
 * @param[in] b2    Second byte
 * @param[in] b3    Third byte
 * @param[in] b4    Fourth byte
 * @return          Returns 32-bit ID.
 */
static inline uint32_t FlashSave_BYTES2ID(uint8_t b1,
                                          uint8_t b2,
                                          uint8_t b3,
                                          uint8_t b4)
{
    return (((uint32_t)(b1) << 0)  |
            ((uint32_t)(b2) << 8)  |
            ((uint32_t)(b3) << 16) |
            ((uint32_t)(b4) << 24));
}

/**
 * @brief       Swaps the oder of the bytes in a word.
 * 
 * @param[in] swap  The word to be reverse the order of bytes in.
 * @return      Returns the reordered word.
 */
static inline uint32_t FlashSave_Swap32(uint32_t swap)
{
    return ((swap >> 24) & 0xFF)     |  /* Move byte 3 to byte 0 */
           ((swap << 8)  & 0xFF0000) |  /* Move byte 1 to byte 2 */
           ((swap >> 8)  & 0xFF00)   |  /* Move byte 2 to byte 1 */
           ((swap << 24) & 0xFF000000); /* Move byte 0 to byte 3 */
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void FlashSaveInit(void);
bool FlashSave_Seek(uint32_t uid, int16_t *page_number, uint8_t *size);
FlashSave_Status FlashSave_Write(uint32_t uid,
                                 bool overwrite, 
                                 uint8_t *data, 
                                 uint16_t count);
FlashSave_Status FlashSave_Read(uint32_t uid,
                                uint8_t *data,
                                uint8_t requested_size);

#endif /* __FLASH_SAVE_H */
