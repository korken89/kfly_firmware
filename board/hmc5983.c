/* *
 *
 * Abstraction Layer for HMC5983 Magnetometer
 *
 * */

#include "ch.h"
#include "hal.h"
#include "board.h"
#include "hmc5983.h"

/* Global variable defines */

/* Private function defines */

/* Private external functions */

/**
 * @brief Initializes the HMC5983 sensor
 * 
 * @param[in] cfg Pointer to configuration structure
 * @return RDY_OK if the initialization was successful
 */
msg_t HMC5983Init(const HMC5983_Configuration *cfg)
{
	static uint8_t txbuf[2];
	msg_t status = RDY_OK;

	/* Error: Pointers not defined */
	if ((cfg->data_holder == NULL) || (cfg->i2cp == NULL))
		return RDY_RESET;


	/* Initialize the data event source and mutex */
	chMtxInit(&cfg->data_holder->read_lock);
	chEvtInit(&cfg->data_holder->es);


	/* Initialize the sensor */
	/* Set averaging, update rate and bias */
	txbuf[0] = HMC5983_RA_CONFIG_A; 	/* Power management register 1 */
	txbuf[1] = (cfg->temperature_sensor | 
				cfg->sample_averaging 	|
				cfg->output_rate 		| 
				cfg->measurement_mode);
	i2cAcquireBus(cfg->i2cp);
	status = i2cMasterTransmitTimeout(	cfg->i2cp,
										cfg->address_7bit, 
										txbuf, 
										2, 
										NULL, 
										0, 
										MS2ST(20));
	i2cReleaseBus(cfg->i2cp);

	/* Error check */
	if (status != RDY_OK)
		return status;

	/* Set the gain */
	txbuf[0] = HMC5983_RA_CONFIG_B; 	/* Power management register 1 */
	txbuf[1] = cfg->gain_configuration;
	i2cAcquireBus(cfg->i2cp);
	status = i2cMasterTransmitTimeout(	cfg->i2cp,
										cfg->address_7bit, 
										txbuf, 
										2, 
										NULL, 
										0, 
										MS2ST(20));
	i2cReleaseBus(cfg->i2cp);

	/* Error check */
	if (status != RDY_OK)
		return status;

	/* Set the mode */
	txbuf[0] = HMC5983_RA_MODE; 	/* Power management register 1 */
	txbuf[1] = (cfg->operating_mode | cfg->i2c_speed | cfg->low_power_mode);
	i2cAcquireBus(cfg->i2cp);
	status = i2cMasterTransmitTimeout(	cfg->i2cp,
										cfg->address_7bit, 
										txbuf, 
										2, 
										NULL, 
										0, 
										MS2ST(20));
	i2cReleaseBus(cfg->i2cp);

	return status;
}

/**
 * @brief Reads the ID of the HMC5983 sensor
 * 
 * @param[in] cfg Pointer to configuration structure
 * @param[out] id Pointer to where the three byte id will be saved
 * 
 * @return [description]
 */
msg_t HMC5983GetID(const HMC5983_Configuration *cfg, uint8_t id[3])
{
	static uint8_t txbuf[1] = {HMC5983_RA_ID_A};
	msg_t status = RDY_OK;

	/* Get ID */
	i2cAcquireBus(cfg->i2cp);
	status = i2cMasterTransmitTimeout(	cfg->i2cp,
										cfg->address_7bit, 
										txbuf, 
										1, 
										id, 
										3, 
										MS2ST(20));
	i2cReleaseBus(cfg->i2cp);

	return status;
}

/**
 * @brief Reads the data registers of the HMC5983
 *
 * @param[in] cfg Pointer to configuration structure
 * @param[out] data Pointer to where the data will be saved
 * 
 * @return [description]
 */
msg_t HMC5983ReadData(const HMC5983_Configuration *cfg, uint8_t data[6])
{
	static uint8_t txbuf[1] = {HMC5983_RA_DATAX_H};
	msg_t status = RDY_OK;

	/* Get ID */
	i2cAcquireBus(cfg->i2cp);
	status = i2cMasterTransmitTimeout(	cfg->i2cp, 
										cfg->address_7bit, 
										txbuf, 
										1, 
										data, 
										6, 
										MS2ST(20));
	i2cReleaseBus(cfg->i2cp);

	return status;
}