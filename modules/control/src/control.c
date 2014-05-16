/* *
 *
 * General control structure from position to motors:
 *                 __________        __________        ___________        __________        __________        __________
 *                |          |      |          |      |           |      |          |      |          |      |          |
 *            +-> | Position | -+-> | Velocity | ---> | Targeting | -+-> | Attitude | -+-> |   Rate   | -+-> |  Motors  |
 *            |   |__________|  |   |__________|      |___________|  |   |__________|  |   |__________|  |   |__________|
 *            |                 |                                    |                 |                 |
 *             /                 /                                    /                 /                 / <-- Switch
 * Reference -+-----------------+------------------------------------+-----------------+-----------------+
 *
 * Aim:
 * To be able to connect a reference anywhere in the
 * chain in order to choose what type of control to use.
 * This is done by the generic control structure with a
 * setting of the current control mode.
 *
 * The "Targeting" module does conversion from velocity
 * commands to attitude depending on where the system
 * shall be pointing.
 *
 * Every block is an vector PI controller (except Targeting
 * and Motors) with 3 inputs, 3 outputs and 3 references.
 *
 * */

/* Includes */
#include "control.h"

/* Private Defines */

/* Private Typedefs */

/* Private variable defines */
static Control_Reference_Type Control_Reference;
Control_Data_Type Control_Data;
Control_Limits_Type Control_Limits;
Output_Mixer_Type Output_Mixer;

/* Global variable defines */

/* Private function defines */
static vector3f_t vPositionControl(vector3f_t, Control_Limits_Type *, float);
static vector3f_t vVelocityControl(vector3f_t, Control_Limits_Type *, float);
static vector3f_t vAttitudeControl(vector3f_t, Control_Limits_Type *, float);
static void vRateControl(vector3f_t, Control_Limits_Type *, float);
static void vUpdateOutputs(float, float, float, float);

void vInitControl(void)
{
	/* Initialize the controllers here */
	Control_Data.position_controller[0].P_gain = 2.0f;
	Control_Data.position_controller[0].I_gain = 1.0f;
	Control_Data.position_controller[0].I_limit = 0.2f;

	Control_Data.position_controller[1].P_gain = 2.0f;
	Control_Data.position_controller[1].I_gain = 1.0f;
	Control_Data.position_controller[1].I_limit = 0.2f;

	Control_Data.position_controller[2].P_gain = 2.0f;
	Control_Data.position_controller[2].I_gain = 1.0f;
	Control_Data.position_controller[2].I_limit = 0.2f;


	Control_Data.velocity_controller[0].P_gain = 2.0f;
	Control_Data.velocity_controller[0].I_gain = 1.0f;
	Control_Data.velocity_controller[0].I_limit = 0.2f;

	Control_Data.velocity_controller[1].P_gain = 2.0f;
	Control_Data.velocity_controller[1].I_gain = 1.0f;
	Control_Data.velocity_controller[1].I_limit = 0.2f;

	Control_Data.velocity_controller[2].P_gain = 2.0f;
	Control_Data.velocity_controller[2].I_gain = 1.0f;
	Control_Data.velocity_controller[2].I_limit = 0.2f;


	Control_Data.attitude_controller[0].P_gain = 2.0f;
	Control_Data.attitude_controller[0].I_gain = 1.0f;
	Control_Data.attitude_controller[0].I_limit = 0.2f;

	Control_Data.attitude_controller[1].P_gain = 2.0f;
	Control_Data.attitude_controller[1].I_gain = 1.0f;
	Control_Data.attitude_controller[1].I_limit = 0.2f;

	Control_Data.attitude_controller[2].P_gain = 2.0f;
	Control_Data.attitude_controller[2].I_gain = 1.0f;
	Control_Data.attitude_controller[2].I_limit = 0.2f;


	Control_Data.rate_controller[0].P_gain = 2.0f;
	Control_Data.rate_controller[0].I_gain = 1.0f;
	Control_Data.rate_controller[0].I_limit = 0.2f;

	Control_Data.rate_controller[1].P_gain = 2.0f;
	Control_Data.rate_controller[1].I_gain = 1.0f;
	Control_Data.rate_controller[1].I_limit = 0.2f;

	Control_Data.rate_controller[2].P_gain = 2.0f;
	Control_Data.rate_controller[2].I_gain = 1.0f;
	Control_Data.rate_controller[2].I_limit = 0.2f;


	Control_Limits.max_rate.pitch = 200.0f;
	Control_Limits.max_rate.roll = 200.0f;
	Control_Limits.max_rate.yaw = 90.0f;

	Control_Limits.max_rate_attitude.pitch = 200.0f;
	Control_Limits.max_rate_attitude.roll = 200.0f;
	Control_Limits.max_rate_attitude.yaw = 90.0f;

	Control_Limits.max_angle.pitch = 50.0f;
	Control_Limits.max_angle.roll = 50.0f;

	Control_Limits.max_velocity.horizontal = 10.0f;
	Control_Limits.max_velocity.vertical = 2.0f;
}

void vUpdateControlAction(Control_Reference_Type *reference, Control_Limits_Type *limits, float dt)
{
	vector3f_t u;

	switch (reference->mode)
	{
		case FLIGHTMODE_POSITION_HOLD:

			break;

		case FLIGHTMODE_POSITION:
			u = vPositionControl(reference->reference, limits, dt);
			u = vVelocityControl(u, limits, dt);
			u = vAttitudeControl(u, limits, dt);
			vRateControl(u, limits, dt);
			break;

		case FLIGHTMODE_VELOCITY:
			u = vVelocityControl(reference->reference, limits, dt);
			u = vAttitudeControl(u, limits, dt);
			vRateControl(u, limits, dt);
			break;

		case FLIGHTMODE_ATTITUDE:
			u = vAttitudeControl(reference->reference, limits, dt);
			vRateControl(u, limits, dt);
			break;

		case FLIGHTMODE_RATE:
			vRateControl(reference->reference, limits, dt);
			break;

		case FLIGHTMODE_DISARMED:
		default:
			vUpdateOutputs(0.0f, 0.0f, 0.0f, 0.0f);
			break;
	}

}

Control_Data_Type *ptrGetControlData(void)
{
	return &Control_Data;
}

Control_Limits_Type *ptrGetControlLimits(void)
{
	return &Control_Limits;
}

Output_Mixer_Type *ptrGetOutputMixer(void)
{
	return &Output_Mixer;
}


/* *
 *
 * Private functions
 *
 * */

static vector3f_t vPositionControl(vector3f_t reference, Control_Limits_Type *limits, float dt)
{
	(void)limits;
	float error_x, error_y, error_z;
	vector3f_t u;

	error_x = reference.x - 0.0f; /* Add where the position can be read */
	error_y = reference.y - 0.0f; /* Add where the position can be read */
	error_z = reference.z - 0.0f; /* Add where the position can be read */

	u.x = fPIUpdate(&Control_Data.position_controller[0], error_x, dt);
	u.y = fPIUpdate(&Control_Data.position_controller[1], error_y, dt);
	u.z = fPIUpdate(&Control_Data.position_controller[2], error_z, dt);

	/* Send control signal to the next step in the cascade */
	return u;

}

static vector3f_t vVelocityControl(vector3f_t reference, Control_Limits_Type *limits, float dt)
{
	(void)limits;
	float error_x, error_y, error_z;
	vector3f_t u;


	/* *
	 *
	 * TODO: Add the saturation of velocities.
	 *
	 * */

	error_x = reference.x - 0.0f; /* Add where the velocity can be read */
	error_y = reference.y - 0.0f; /* Add where the velocity can be read */
	error_z = reference.z - 0.0f; /* Add where the velocity can be read */

	u.x = fPIUpdate(&Control_Data.velocity_controller[0], error_x, dt);
	u.y = fPIUpdate(&Control_Data.velocity_controller[1], error_y, dt);
	u.z = fPIUpdate(&Control_Data.velocity_controller[2], error_z, dt);

	/* *
	 *
	 * TODO: Add the conversion block for targeting.
	 *
	 * */

	/* Send control signal to the next step in the cascade */
	return u;
}

static vector3f_t vAttitudeControl(vector3f_t reference, Control_Limits_Type *limits, float dt)
{
	(void)limits;
	(void)reference;
	
	float error_x, error_y, error_z;
	vector3f_t u;


	/* *
	 *
	 * TODO: Add the saturation of angles.
	 *
	 * */

	/* *
	 *
	 * TODO: Calculate quaternion error.
	 *
	 * */

	u.x = fPIUpdate(&Control_Data.attitude_controller[0], error_x, dt);
	u.y = fPIUpdate(&Control_Data.attitude_controller[1], error_y, dt);
	u.z = fPIUpdate(&Control_Data.attitude_controller[2], error_z, dt);

	/* Send control signal to the next step in the cascade */
	return u;
}

static void vRateControl(vector3f_t reference, Control_Limits_Type *limits, float dt)
{
	(void)limits;
	float error_x, error_y, error_z;
	vector3f_t u;


	/* *
	 *
	 * TODO: Add the saturation of rotational rates.
	 *
	 * */

	error_x = reference.x - 0.0f; /* Add where the rotational rate can be read */
	error_y = reference.y - 0.0f; /* Add where the rotational rate can be read */
	error_z = reference.z - 0.0f; /* Add where the rotational rate can be read */

	u.x = fPIUpdate(&Control_Data.rate_controller[0], error_x, dt);
	u.y = fPIUpdate(&Control_Data.rate_controller[1], error_y, dt);
	u.z = fPIUpdate(&Control_Data.rate_controller[2], error_z, dt);

	/* TODO: Send control signal to the engines */
	vUpdateOutputs(Control_Reference.throttle, u.x, u.y, u.z);
}

static void vUpdateOutputs(float u_throttle, float u_pitch, float u_roll, float u_yaw)
{
	float sum;
	int i;

	for (i = 0; i < 8; i++)
	{
		sum = u_throttle * Output_Mixer.weights[i][0];
		sum += u_pitch * Output_Mixer.weights[i][1];
		sum += u_roll * Output_Mixer.weights[i][2];
		sum += u_yaw * Output_Mixer.weights[i][3];

		//vSetRCOutput(i, sum);
		(void)sum;
	}
}
