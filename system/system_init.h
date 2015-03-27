#ifndef __SYSTEM_INIT_H
#define __SYSTEM_INIT_H

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

/**
 * @brief Key for the system shutdown.
 */
#define SYSTEM_SHUTDOWN_KEY				0xdeadbeef

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

/**
 * @brief Enum for system state.
 */
typedef enum
{
    /**
     * @brief System is not yet initialized.
     */
    SYSTEM_UNINITIALIZED,
    /**
     * @brief System is running initialization.
     */
    SYSTEM_INITIALIZING,
    /**
     * @brief System is running.
     */
    SYSTEM_RUNNING,
    /**
     * @brief System has started the termination sequence.
     */
    SYSTEM_TERMINATING,
    /**
     * @brief System has stopped.
     */
    SYSTEM_STOPPED
} system_state_t;

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

#endif
