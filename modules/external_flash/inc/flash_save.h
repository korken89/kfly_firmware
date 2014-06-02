#ifndef __FLASH_SAVE_H
#define __FLASH_SAVE_H

#include "ext_flash.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#define FLASHSAVE_NO_MATCH      false
#define FLASHSAVE_MATCH         true
#define FLASHSAVE_UNALLOCATED   0xFFFFFFFF

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

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
    return (((uint32_t)(str[0]) << 0) |
            ((uint32_t)(str[1]) << 8) |
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
    return (((uint32_t)(b1) << 0) |
            ((uint32_t)(b2) << 8) |
            ((uint32_t)(b3) << 16) |
            ((uint32_t)(b4) << 24));
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void FlashSaveInit(void);
bool FlashSave_Seek(uint32_t uid, int16_t *page_number, uint8_t *size);
void FlashSave_Write(uint32_t uid,
                     bool overwrite, 
                     uint8_t *data, 
                     uint8_t count);
bool FlashSave_Read(uint32_t uid, uint8_t *data, int16_t requested_size);

#endif /* __FLASH_SAVE_H */
