/* *
 *
 *
 * */

#include "ch.h"
#include "hal.h"
#include "bootloader.h"


/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*
 * @brief   Symbol for the end of the RAM.
 */
extern uint32_t __heap_end__;
/*
 * @brief   Bootloader's magic value location.
 */
#define BOOTLOADER_MAGIC_POSITION           (((uint32_t)&(__heap_end__)) - 4)
/*
 * @brief   Bootloader's magic value for entering.
 */
#define BOOTLOADER_MAGIC_VALUE              0xdeadbeef

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*
 * @brief   Reads the bootloader's magic value.
 *
 * @return  The value stored in the magic location.
 */
static inline uint32_t u32BootloaderGetMagicValue(void)
{
    return *((uint32_t *)BOOTLOADER_MAGIC_POSITION);
}

/*
 * @brief           Sets the bootloader's magic value.
 *
 * @param[in] val   The value to be stored in the magic location.
 */
static inline void vBootloaderSetMagicValue(uint32_t val)
{
    *((uint32_t *)BOOTLOADER_MAGIC_POSITION) = val;
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/*
 * @brief   Checks if the DFU bootloader is to be invoked.
 *
 * @note    Shall be called before clocks are initialized in __early_init()
 */
void vBootloaderStartupCheck(void)
{
    /* Check the magic location in memory if it is set. */
    if (u32BootloaderGetMagicValue() == BOOTLOADER_MAGIC_VALUE) {

        /* Reset the magic value so a boot loop is avoided. */
        vBootloaderSetMagicValue(0);

        /* Set MSP as the current stack pointer and force privileged mode. */
        __set_CONTROL(0);

        /* Load MSP value and start the DFU. */
        __set_MSP((*(uint32_t *)DFU_MSP_ADDRESS));
        ((void (*)(void)) (*(uint32_t *)DFU_RESET_ADDRESS))();

        /* Loop to catch errors. */
        while(1);
    }
}

/*
 * @brief   Executes the sequence for entering the DFU bootloader.
 */
void vBootloaderResetAndStartDFU(void)
{
    vBootloaderSetMagicValue(BOOTLOADER_MAGIC_VALUE);

    NVIC_SystemReset();
}
