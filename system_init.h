#ifndef __SYSTEM_INIT_H
#define __SYSTEM_INIT_H

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

/**
 * @brief Key for the system shutdown.
 */
#define SYSTEM_SHUTDOWN_KEY				0xdeadbeef
/**
 * @brief Base address for the DFU in memory: Table 21, AN2606 for STM32.
 */
#define DFU_BASE_ADDRESS        0x1FFF0000
/**
 * @brief Address to the DFU's starting value of the MSP.
 */
#define DFU_MSP_ADDRESS         DFU_BASE_ADDRESS
/**
 * @brief Address to the DFU's reset vector.
 */
#define DFU_RESET_ADDRESS       (DFU_BASE_ADDRESS + 4)

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief Structure for critical task subscription.
 */
typedef struct _system_critical_subscription
{
    /**
     * @brief Pointer to the next critical task structure.
     */
    struct _system_critical_subscription *next;
    /**
     * @brief Pointer to the critical task.
     */
    thread_t *thread;
} system_critical_subscription_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

void vSystemInit(void);
void vSystemDeinit(void);
bool bSystemShutdownRequested(void);
void vSystemRequestShutdown(uint32_t key);
void vSystemCriticalTaskSubscribe(system_critical_subscription_t *sub);
void vSystemStartDFUBootloader(void);

#endif
