/* *
 *
 * Abstraction Layer for HMC5983 Magnetometer
 *
 * */

#include "ch.h"
#include "hal.h"
#include "hmc5983.h"

/* Global variable defines */

/* Private function defines */

/* Private external functions */

msg_t HMC5983Init(const HMC5983_Configuration *cfg)
{
	/* Setup the sensor */
	static uint8_t txbuf[2];
	msg_t status = RDY_OK;

	/* Setup the sensor */
	/* Set averaging, update rate and bias */
	txbuf[0] = HMC5983_RA_CONFIG_A; 	/* Power management register 1 */
	txbuf[1] = (cfg->temperature_sensor | cfg->sample_averaging | cfg->output_rate | cfg->measurement_mode);
	i2cAcquireBus(cfg->i2cp);
	status = i2cMasterTransmitTimeout(cfg->i2cp, cfg->address_7bit, txbuf, 2, NULL, 0, MS2ST(20));
	i2cReleaseBus(cfg->i2cp);

	/* Error check */
	if (status != RDY_OK)
		return status;

	/* Set the gain */
	txbuf[0] = HMC5983_RA_CONFIG_B; 	/* Power management register 1 */
	txbuf[1] = cfg->gain_configuration;
	i2cAcquireBus(cfg->i2cp);
	status = i2cMasterTransmitTimeout(cfg->i2cp, cfg->address_7bit, txbuf, 2, NULL, 0, MS2ST(20));
	i2cReleaseBus(cfg->i2cp);

	/* Error check */
	if (status != RDY_OK)
		return status;

	/* Set the mode */
	txbuf[0] = HMC5983_RA_MODE; 	/* Power management register 1 */
	txbuf[1] = (cfg->operating_mode | cfg->i2c_speed | cfg->low_power_mode);
	i2cAcquireBus(cfg->i2cp);
	status = i2cMasterTransmitTimeout(cfg->i2cp, cfg->address_7bit, txbuf, 2, NULL, 0, MS2ST(20));
	i2cReleaseBus(cfg->i2cp);

	return status;
}

msg_t HMC5983GetID(const HMC5983_Configuration *cfg, uint8_t id[3])
{
	static uint8_t txbuf[1] = {HMC5983_RA_ID_A};
	msg_t status = RDY_OK;

	/* Get ID */
	i2cAcquireBus(cfg->i2cp);
	status = i2cMasterTransmitTimeout(cfg->i2cp, cfg->address_7bit, txbuf, 1, id, 3, MS2ST(20));
	i2cReleaseBus(cfg->i2cp);

	return status;
}
