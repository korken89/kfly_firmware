#ifndef __COMPUTER_CONTROL_H
#define __COMPUTER_CONTROL_H

#include "control.h"
#include "control_definitions.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#define COMPUTER_CONTROL_MESSAGE_SIZE   sizeof(computer_control_reference_t)

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Data transfer structure for the computer control commands.
 */
typedef struct PACKED_VAR
{
    /**
     * @brief Flight mode to differentiate the following data.
     */
    flightmode_t mode;

    union
    {
        /**
         * @brief Direct motor control data.
         */
        uint16_t direct_control[8];

        /**
         * @brief Indirect motor control data.
         */
        struct
        {
            vector3f_t torque;
            float throttle;
        } indirect_control;

        /**
         * @brief Rate control data.
         */
        struct
        {
            vector3f_t rate;
            float throttle;
        } rate;

        /**
         * @brief Attitude control data.
         */
        struct
        {
            quaternion_t attitude;
            float throttle;
        } attitude;
    };
} computer_control_reference_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

void ComputerControlInit(void);
bool ComputerControlLinkActive(void);
bool ComputerControlEnabled(void);
flightmode_t ComputerControlGetFlightmode(void);
void ComputerControlGetAttitudeReference(quaternion_t *attitude_ref,
                                         float *throttle_ref);
void ComputerControlGetRateReference(vector3f_t *rate_ref,
                                     float *throttle_ref);
void ComputerControlGetIndirectReference(vector3f_t *torque_ref,
                                         float *throttle_ref);
void ComputerControlGetDirectReference(float output[8]);
void vParseComputerControlPacket(const uint8_t *payload, const uint8_t size);

#endif
