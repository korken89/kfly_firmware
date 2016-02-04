#ifndef __VERSION_INFORMATION_H
#define __VERSION_INFORMATION_H

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

/** @brief  Size of unique ID string. */
#define UNIQUE_ID_SIZE          12

/** @brief  Size of version string. */
#define VERSION_MAX_SIZE        70

/** @brief  Size of user ID string. */
#define USER_ID_MAX_SIZE        100


/** @brief  Location for the unique hardware ID. */
#define UNIQUE_ID_BASE          0x1fff7a10

/** @brief  Base address for the firmware in flash. */
#define FIRMWARE_BASE           0x08000000

/** @brief  Base address for the bootloader in flash. */
#define BOOTLOADER_BASE         0x08000000

/** @brief  Offset from the FIRMWARE_BASE to the software version string. */
#define SW_VERSION_OFFSET       0x200


/** @brief  Date definition. */
#ifndef DATE
    #define DATE                "no timestamp"
#endif

/** @brief  Version definition. */
#ifndef GIT_VERSION
    #define GIT_VERSION         "no version"
#endif

#define FUNCTION_NAME(name)     #name
#define STR(macro)              FUNCTION_NAME(macro)
#define GIT_DATE_NAME           STR(DATE)
#define GIT_VERSION_NAME        STR(GIT_VERSION)

/** @brief  Definition of the version string. */
#define KFLY_VERSION            GIT_VERSION_NAME ", Build date: " \
                                GIT_DATE_NAME "\0"

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
const uint8_t *ptrGetUniqueID(void);
const uint8_t *ptrGetBootloaderVersion(void);
const uint8_t *ptrGetFirmwareVersion(void);
uint8_t *ptrGetUserIDString(void);

#endif
