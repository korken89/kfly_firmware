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


msg_t RCOutputInit(const RCOutput_Configuration *rcoutputcfg)
{
	msg_t status = RDY_OK;

	pwmStart(rcoutputcfg->pwmp_lowbank, rcoutputcfg->pwmcfg);
	pwmStart(rcoutputcfg->pwmp_highbank, rcoutputcfg->pwmcfg);

	/* Initialize with lowest output width */
	pwmEnableChannel(rcoutputcfg->pwmp_lowbank, 0, 1000);
	pwmEnableChannel(rcoutputcfg->pwmp_lowbank, 1, 1000);
	pwmEnableChannel(rcoutputcfg->pwmp_lowbank, 2, 1000);
	pwmEnableChannel(rcoutputcfg->pwmp_lowbank, 3, 1000);

	pwmEnableChannel(rcoutputcfg->pwmp_highbank, 0, 1000);
	pwmEnableChannel(rcoutputcfg->pwmp_highbank, 1, 1000);
	pwmEnableChannel(rcoutputcfg->pwmp_highbank, 2, 1000);
	pwmEnableChannel(rcoutputcfg->pwmp_highbank, 3, 1000);

	return status;
}



msg_t RCOutputSetChannelWidthUs(const RCOutput_Configuration *rcoutputcfg,
							   RCOutput_Channel_Sector sel, 
							   pwmcnt_t width_us)
{
	static const uint32_t pwmchannellut[8] = {3, 2, 1, 0, 3, 2, 1, 0};

	if (sel > RCOUTPUT_CHANNEL_8)
		return !RDY_OK;

	if (sel <= RCOUTPUT_CHANNEL_4)
		pwmEnableChannel(rcoutputcfg->pwmp_lowbank,
						 pwmchannellut[sel],
						 width_us);
	else
		pwmEnableChannel(rcoutputcfg->pwmp_highbank, 
						 pwmchannellut[sel], 
						 width_us);

	return RDY_OK;
}

msg_t RCOutputSetChannelWidthRelative(const RCOutput_Configuration *rcoutputcfg,
							 		 RCOutput_Channel_Sector sel, 
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
	RCOutputSetChannelWidthUs(rcoutputcfg,
							  sel, 
							  (pwmcnt_t)(width * 1000.0f + 1000.0f));

	return RDY_OK;
}

msg_t RCOutputSetChannelPeriod(const RCOutput_Configuration *rcoutputcfg,
							   RCOutput_Bank_Sector sel,
							   RCOutput_Rate_Sector rate)
{
	if ((sel > RCOUTPUT_BANK_5_8) || (rate > RCUTPUT_50HZ))
		return !RDY_OK;

	if (sel == RCOUTPUT_BANK_1_4)
		pwmChangePeriod(rcoutputcfg->pwmp_lowbank, (pwmcnt_t)rate);
	else
		pwmChangePeriod(rcoutputcfg->pwmp_highbank, (pwmcnt_t)rate);

	return RDY_OK;
}