#ifndef __EXT_FLASH_H
#define __EXT_FLASH_H

/*===========================================================================*/
/* Driver global definitions.                                                */
/*===========================================================================*/
#define FLASH_CMD_WRSR          0x01        /* Write Status Register
                                               instruction */
#define FLASH_CMD_PAGE_PROGRAM  0x02        /* Write to Memory instruction */
#define FLASH_CMD_READ          0x03        /* Read from Memory instruction */
#define FLASH_CMD_RDSR          0x05        /* Read Status Register
                                               instruction */
#define FLASH_CMD_WREN          0x06        /* Write enable instruction */
#define FLASH_CMD_PAGE_WRITE    0x0A        /* Writes a page with automatic 
                                               erase */

#define FLASH_CMD_BE            0xC7        /* Bulk Erase instruction */
#define FLASH_CMD_SE            0xD8        /* Sector Erase instruction */
#define FLASH_CMD_PE            0xDB        /* Page erase */
#define FLASH_CMD_RDID          0x9F        /* Read identification */

#define FLASH_WIP_FLAG          0x01        /* Write In Progress (WIP) flag */

#define FLASH_M25P128_ID        0x202018
#define FLASH_M25P64_ID         0x202017
#define FLASH_M25P40_ID         0x202013
#define FLASH_M25PE40_ID        0x208013

#define M25PE40_NUM_SECTORS     8

#define M25PE40_NUM_PAGES       2048

#define FLASH_PAGE_SIZE         0x00000100
#define FLASH_SECTOR_SIZE       0x00010000

#define FLASH_DUMMY_BYTE        0xFF

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/
typedef struct
{
    uint8_t flash_tmp[4];
    mutex_t flash_mutex;
} ExternalFlashData;

typedef struct
{
    SPIDriver *spip;
    ioportid_t cs_port;
    uint16_t cs_pad;
    uint32_t num_pages;
    uint32_t jedec_id;
    ExternalFlashData *data;
} ExternalFlashConfig;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/* Claim and release macros for external flash memory */
#define ExternalFlash_Claim(config)   chMtxLock(&(config)->data->flash_mutex)
#define ExternalFlash_Release(config) chMtxUnlock(&(config)->data->flash_mutex)

/* Flash CS select macros */
#define ExternalFlash_Select(config)    palClearPad((config)->cs_port, \
                                                    (config)->cs_pad)
#define ExternalFlash_Unselect(config)  palSetPad((config)->cs_port, \
                                                  (config)->cs_pad)
#define FLASH_STR_TO_ID(str)    (((uint32_t)((str)[0]) << 0) | \
                                ((uint32_t)((str)[1]) << 8) | \
                                ((uint32_t)((str)[2]) << 16) | \
                                ((uint32_t)((str)[3]) << 24))
#define FLASH_BYTES_TO_ID(a, b, c, d)   (((uint32_t)(a) << 0) | \
                                        ((uint32_t)(b) << 8) | \
                                        ((uint32_t)(c) << 16) | \
                                        ((uint32_t)(d) << 24))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
bool ExternalFlashInit(const ExternalFlashConfig *config);
void ExternalFlash_EraseBulk(const ExternalFlashConfig *config);
void ExternalFlash_EraseSector(const ExternalFlashConfig *config,
                               uint32_t address);
void ExternalFlash_ErasePage(const ExternalFlashConfig *config,
                             uint32_t address);
uint32_t ExternalFlash_ReadID(const ExternalFlashConfig *config);
void ExternalFlash_WritePagePolling(const ExternalFlashConfig *config,
                                    uint8_t *buffer,
                                    uint32_t address, 
                                    uint16_t count);
void ExternalFlash_WritePage(const ExternalFlashConfig *config,
                             uint8_t *buffer,
                             uint32_t address, 
                             uint16_t count);
void ExternalFlash_ReadBufferPolling(const ExternalFlashConfig *config,
                                     uint8_t *buffer,
                                     uint32_t address, 
                                     uint16_t count);
void ExternalFlash_ReadBuffer(const ExternalFlashConfig *config,
                              uint8_t *buffer,
                              uint32_t address, 
                              uint16_t count);

#endif
