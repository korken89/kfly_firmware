#ifndef __SYSTEM_INFORMATION_H
#define __SYSTEM_INFORMATION_H

#include "kfly_defs.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

/** @brief  Size of unique ID string. */
#define UNIQUE_ID_SIZE 12

/** @brief  Size of version string. */
#define VEHICLE_NAME_SIZE 48

/** @brief  Size of user ID string. */
#define VEHICLE_TYPE_SIZE 48

/** @brief  Size of user ID string. */
#define KFLY_VERSION_SIZE 96

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Struct holding the system information.
 */
typedef struct PACKED_VAR
{
  /*========================*/
  /* Identification         */
  /*========================*/

  /**
   * @brief   User selectable name of the vehicle.
   */
  char vehicle_name[VEHICLE_NAME_SIZE];

  /**
   * @brief   Type of vehicle (quadrotor, Neo, Firefly, Pelican, ...)
   */
  char vehicle_type[VEHICLE_TYPE_SIZE];

  /**
   * @brief   Unique ID, read from the MCU.
   */
  uint8_t unique_id[UNIQUE_ID_SIZE];

  /**
   * @brief   KFly and Git compile string.
   */
  char kfly_version[KFLY_VERSION_SIZE];

  /*========================*/
  /* General info           */
  /*========================*/

  /**
   * @brief   Flight time in seconds.
   */
  float flight_time;

  /**
   * @brief   Up time in seconds.
   */
  float up_time;

  /**
   * @brief   CPU usage in [0, 1]
   */
  float cpu_usage;

  /*========================*/
  /* Hardware info          */
  /*========================*/

  /**
   * @brief   Battery voltage in V.
   */
  float battery_voltage;

  /**
   * @brief   Flag for if the motors are armed.
   */
  bool8_t motors_armed;

  /**
   * @brief   Flag for if the vehicle is flying.
   */
  bool8_t in_air;

  /**
   * @brief   Flag for if the serial computer control is enabled.
   */
  bool8_t serial_interface_enabled;
} system_information_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

void SystemInformationInit(void);
void GetSystemInformation(system_information_t *dest);
void SetSystemNameType(const char name[VEHICLE_NAME_SIZE],
                       const char type[VEHICLE_TYPE_SIZE]);

#endif
