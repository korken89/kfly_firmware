/* *
 *
 * Abstraction Layer for RC Outputs
 *
 * */

#include "ch.h"
#include "hal.h"
#include "board.h"
#include "rc_output.h"

/* Global variable defines */

/* Private function defines */

/* Private external functions */

/**
 * @brief Initializes RC outputs
 * 
 * @param[in] rcoutputcfg Pointer to configuration structure
 * @return RDY_OK if the initialization was successful
 */
msg_t RCOutputInit(const RCOutput_Configuration *cfg)
{
	msg_t status = RDY_OK;

	pwmStart(cfg->pwmp_lowbank, cfg->pwmcfg);
	pwmStart(cfg->pwmp_highbank, cfg->pwmcfg);

	/* Initialize with lowest output width */
	pwmEnableChannel(cfg->pwmp_lowbank, 0, 1000);
	pwmEnableChannel(cfg->pwmp_lowbank, 1, 1000);
	pwmEnableChannel(cfg->pwmp_lowbank, 2, 1000);
	pwmEnableChannel(cfg->pwmp_lowbank, 3, 1000);

	pwmEnableChannel(cfg->pwmp_highbank, 0, 1000);
	pwmEnableChannel(cfg->pwmp_highbank, 1, 1000);
	pwmEnableChannel(cfg->pwmp_highbank, 2, 1000);
	pwmEnableChannel(cfg->pwmp_highbank, 3, 1000);

	return status;
}


/**
 * @brief Set output channel width in microseconds
 * 
 * @param[in] cfg Pointer to configuration structure
 * @param[in] sel Channel selector
 * @param[in] width_us New width in microseconds
 * @return RDY_OK if the change was successful
 */
msg_t RCOutputSetChannelWidthUs(const RCOutput_Configuration *cfg,
							   RCOutput_Channel_Selector sel, 
							   pwmcnt_t width_us)
{
	static const uint32_t pwmchannellut[8] = {3, 2, 1, 0, 3, 2, 1, 0};

	if (sel > RCOUTPUT_CHANNEL_8)
		return !RDY_OK;

	if (sel <= RCOUTPUT_CHANNEL_4)
		pwmEnableChannel(cfg->pwmp_lowbank,
						 pwmchannellut[sel],
						 width_us);
	else
		pwmEnableChannel(cfg->pwmp_highbank, 
						 pwmchannellut[sel], 
						 width_us);

	return RDY_OK;
}

/**
 * @brief Set relative output channel width
 * @details Set the width from 1 to 2 milliseconds using 0% to 100%.
 * 
 * @param[in] cfg Pointer to configuration structure
 * @param[in] sel Channel selector
 * @param[in] width New width in 0.0 to 1.0
 * @return RDY_OK if the change was successful
 */
msg_t RCOutputSetChannelWidthRelativePositive(const RCOutput_Configuration *cfg,
							 		 		  RCOutput_Channel_Selector sel, 
							 		 		  float width)
{
	if (sel > RCOUTPUT_CHANNEL_8)
		return !RDY_OK;

	/* Bound the width from 0 % to 100 % */
	if (width < 0.0f)
		width = 0.0f;
	else if (width > 1.0f)
		width = 1.0f;

	/* Convert to us and send */
	return RCOutputSetChannelWidthUs(cfg,
							  		 sel, 
							  		 (pwmcnt_t)(width * 1000.0f + 1000.0f));
}

/**
 * @brief Set relative output channel width
 * @details Set the width from 1 to 2 milliseconds using -100% to 100%.
 * 
 * @param[in] cfg Pointer to configuration structure
 * @param[in] sel Channel selector
 * @param[in] width New width in 0.0 to 1.0
 * @return RDY_OK if the change was successful
 */
msg_t RCOutputSetChannelWidthRelative(const RCOutput_Configuration *cfg,
							 		  RCOutput_Channel_Selector sel, 
							 		  float width)
{
	if (sel > RCOUTPUT_CHANNEL_8)
		return !RDY_OK;

	/* Bound the width from -100% to 100% */
	if (width < -1.0f)
		width = -1.0f;
	else if (width > 1.0f)
		width = 1.0f;

	/* Convert to us and send */
	return RCOutputSetChannelWidthUs(cfg,
							  		 sel, 
							  		 (pwmcnt_t)(width * 500.0f + 1500.0f));
}

/**
 * @brief Change the output rate of one output bank
 * @details [long description]
 * 
 * @param[in] cfg Pointer to configuration structure
 * @param[in] sel Bank selector
 * @param[in] rate Rate 
 * @return RDY_OK if the change was successful
 */
msg_t RCOutputSetChannelPeriod(const RCOutput_Configuration *cfg,
							   RCOutput_Bank_Selector sel,
							   RCOutput_Rate_Selector rate)
{
	if ((sel > RCOUTPUT_BANK_5_8) || (rate > RCUTPUT_50HZ))
		return !RDY_OK;

	if (sel == RCOUTPUT_BANK_1_4)
		pwmChangePeriod(cfg->pwmp_lowbank, (pwmcnt_t)rate);
	else
		pwmChangePeriod(cfg->pwmp_highbank, (pwmcnt_t)rate);

	return RDY_OK;
}