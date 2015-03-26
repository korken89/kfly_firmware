#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

/**
 * @brief Base address for the DFU in memory: Table 21, AN2606 for STM32.
 */
#define DFU_BASE_ADDRESS                0x1fff0000
/**
 * @brief Address to the DFU's starting value of the MSP.
 */
#define DFU_MSP_ADDRESS                 DFU_BASE_ADDRESS
/**
 * @brief Address to the DFU's reset vector.
 */
#define DFU_RESET_ADDRESS               (DFU_BASE_ADDRESS + 4)

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

void vBootloaderStartupCheck(void);
void vBootloaderResetAndStartDFU(void);

#endif
