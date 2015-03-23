/* *
 *
 *
 * */

#include "ch.h"
#include "hal.h"
#include "system_init.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

/** @brief Bool holding the state of the shutdown request. */
static volatile bool shutdown_requested;

/** @brief Holder for the base pointer for the critical tasks linked list. */
static system_critical_subscription_t *base_sub = NULL;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

static void vSystemTerminateCriticalTasks(void)
{

}

static void vSystemDisableSystick(void)
{
#if OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING

    /* Free running counter mode, disable timer.*/
    stStopAlarm();

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

#if OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC

    /* Ticked mode, disable systick.*/
    SysTick->CTRL = 0;

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC */
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void vSystemInit(void)
{
    /* Initialize the shutdown switch */
    shutdown_requested = false;
}

void vSystemDeinit(void)
{
    /* Starting the shutdown sequence.*/

    /* Terminate critical tasks */
    vSystemTerminateCriticalTasks();

    /* Stop drivers */

    /* Stop SysTick */
    chSysDisable();
    vSystemDisableSystick();
    chSysEnable();
}

bool bSystemShutdownRequested(void)
{
    return shutdown_requested;
}

void vSystemRequestShutdown(uint32_t key)
{
    if (key == SYSTEM_SHUTDOWN_KEY)
    {
        shutdown_requested = true;
    }
}

void vSystemCriticalTaskSubscribe(system_critical_subscription_t *sub)
{
    /* Check if the base pointer in null on not */
    if (base_sub == NULL)
    {
        base_sub = sub;
    }
    else
    {
        /* Base pointer is not null, search the list to find the end */
        system_critical_subscription_t *tmp_sub = base_sub;

        while (tmp_sub->next != NULL)
            tmp_sub = tmp_sub->next;

        /* Save the subscription at the end of the list */
        tmp_sub->next = sub;
    }
}

