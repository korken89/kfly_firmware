/* *
 *
 * 
 *
 * */

#include "ch.h"
#include "hal.h"
#include "vicon.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/
EVENTSOURCE_DECL(new_vicon_data_es);

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes the New Vicon Data events.
 */
void InitializeViconSupport()
{
	/* Initialize New Vicon Data event source */
	osalEventObjectInit(&new_vicon_data_es);
}

/**
 * @brief   Broadcast the New Vicon Data events.
 */
void vBroadcastNewViconDataAvailable()
{

}

/**
 * @brief       Returns the pointer to the New Vicon Data event source.
 *
 * @return      Pointer to the New Vicon Data event source.
 */
event_source_t *ptrGetViconDataEventSource(void)
{
	return &new_vicon_data_es;
}
