#ifndef __VERSION_INFORMATION_H
#define __VERSION_INFORMATION_H

/* Defines */
#define UNIQUE_ID_SIZE          12
#define VERSION_MAX_SIZE        70
#define USER_ID_MAX_SIZE        100

#define UNIQUE_ID_BASE          0x1fff7a10
#define FIRMWARE_BASE           0x08000000
#define BOOTLOADER_BASE         0x08000000
#define SW_VERSION_OFFSET       0x1c0

#ifndef DATE
    #define DATE            	"no timestamp"
#endif

#ifndef GIT_VERSION
    #define GIT_VERSION         "no version"
#endif

#define FUNCTION_NAME(name) #name
#define STR(macro) FUNCTION_NAME(macro)
#define GIT_DATE_NAME STR(DATE)
#define GIT_VERSION_NAME STR(GIT_VERSION)

#define KFLY_VERSION    GIT_VERSION_NAME ", Build date: " GIT_DATE_NAME "\0"

/* Typedefs */

/* Macros */

/* Global functions */
uint8_t *ptrGetUniqueID(void);
uint8_t *ptrGetBootloaderVersion(void);
uint8_t *ptrGetFirmwareVersion(void);
uint8_t *ptrGetUserIDString(void);

#endif
