/* *
 *
 * 
 *
 * */

#include "ch.h"
#include "hal.h"
#include "version_information.h"

/* Global variable defines */
__attribute__ ((used, section("vectors"))) const char build_version[] = KFLY_VERSION;

/* Private variable defines */
static uint8_t UserIDString[USER_ID_MAX_SIZE + 1] = "Test ID string!";

/* Private function defines */

/* Private external functions */


void VersionInformationInit(void)
{

}

uint8_t *ptrGetUniqueID(void)
{
	return (uint8_t *)(UNIQUE_ID_BASE);
}

uint8_t *ptrGetBootloaderVersion(void)
{
	return (uint8_t *)(BOOTLOADER_BASE + SW_VERSION_OFFSET);
}

uint8_t *ptrGetFirmwareVersion(void)
{
	return (uint8_t *)(FIRMWARE_BASE + SW_VERSION_OFFSET);
}

uint8_t *ptrGetUserIDString(void)
{
	return UserIDString;
}
