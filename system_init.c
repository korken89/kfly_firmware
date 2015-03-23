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
static volatile bool shutdown_requested = false;

/** @brief Holder for the base pointer for the critical tasks linked list. */
static system_critical_subscription_t *base_sub = NULL;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*
 * @brief   Private function for signaling critical tasks for shutdown and
 *          waiting for them to finish execution.
 */
static void vSystemTerminateCriticalTasks(void)
{
    /* Check if the base pointer in null on not */
    if (base_sub != NULL)
    {
        /* Base pointer is not null, search the list to find the end */
        system_critical_subscription_t *tmp_sub = base_sub;

        /* Set all critical thread to terminate */
        while (tmp_sub->next != NULL)
        {
            chThdTerminate(tmp_sub->thread);
            tmp_sub = tmp_sub->next;
        }

        /* All threads signaled, wait for termination */
        tmp_sub = base_sub;
        while (tmp_sub->next != NULL)
        {
            chThdWait(tmp_sub->thread);
            tmp_sub = tmp_sub->next;
        }

        /* All threads terminated, clear the list */
        base_sub = NULL;
    }
}

/*
 * @brief   Private function for disabling the SysTick or free running counter
 *          depending on which mode is selected.
 */
static void vSystemDisableSystick(void)
{
#if OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING

    /* Free running counter mode, disable timer. */
    stStopAlarm();

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

#if OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC

    /* Ticked mode, disable systick. */
    SysTick->CTRL = 0;

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC */
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/*
 * @brief   Used to initialize everything in the system except the kernel
 *          itself.
 *          - Starts all HW drivers, they should all be placed here.
 *          - Starts all Modules,  they should all be placed here.
 */
void vSystemInit(void)
{
    /*
     * Start drivers and modules (ADD NEW HERE)
     */

}

/*
 * @brief   Used to terminate all running tasks and disable the OS. All
 *          deinit of drivers and modules should be placed here.
 */
void vSystemDeinit(void)
{
    /* Starting the shutdown sequence. */

    /*
     * Terminate critical tasks
     */
    vSystemTerminateCriticalTasks();

    /*
     * Stop drivers (ADD NEW HERE)
     */



     /*
      * Stop SysTick
      */
    chSysDisable();
    vSystemDisableSystick();
    chSysEnable();
}

/*
 * @brief   Returns the status if a shutdown has been requested.
 *
 * @return  The current status.
 * @retval  true if a shutdown has been requested.
 * @retval  false if there is no pending shutdown.
 */
bool bSystemShutdownRequested(void)
{
    return shutdown_requested;
}

/*
 * @brief           Requests a shutdown of the OS.
 *
 * @param[in] key   Key to safe-guard the function.
 */
void vSystemRequestShutdown(uint32_t key)
{
    /* Check shutdown key */
    if (key == SYSTEM_SHUTDOWN_KEY)
    {
        /* Signal for shutdown */
        shutdown_requested = true;
    }
}

/*
 * @brief           Subscribes a critical task to the list of functions that
 *                  need safe shutdown.
 *
 * @param[in] sub   The linked list structure for the task.
 */
void vSystemCriticalTaskSubscribe(system_critical_subscription_t *sub)
{
    /* Check if the base pointer in null on not */
    if (base_sub == NULL)
    {
        /* First subscription */
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
        sub->next = NULL;
    }
}

