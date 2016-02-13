#ifndef __COMPUTER_CONTROL_H
#define __COMPUTER_CONTROL_H

#include "control.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#define COMPUTER_CONTROL_MESSAGE_SIZE   sizeof(computer_control_reference_t)

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/
typedef struct PACKED_VAR
{
    /* @brief Flight mode to differentiate the following data. */
    flightmode_t mode;

    union
    {
        /* @brief Direct motor control data. */
        uint16_t direct_control[8];

        /* @brief Indirect motor control data. */
        struct
        {
            float roll, pitch, yaw;
            float throttle;
        } indirect_control;

        /* @brief Rate control data. */
        struct
        {
            float roll, pitch, yaw;
            float throttle;
        } rate;

        /* @brief Attitude control data. */
        struct
        {
            float w, x, y, z;
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
float fGetComputerControlThrottle(void);
float fGetComputerControlRoll(void);
float fGetComputerControlPitch(void);
float fGetComputerControlYaw(void);
void vParseComputerControlPacket(const uint8_t *payload, const uint8_t size);

#endif
