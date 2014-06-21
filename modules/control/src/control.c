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

#include "ch.h"
#include "hal.h"
#include "quaternion.h"
#include "estimation.h"
#include "control.h"
#include "rc_input.h"
#include "rc_output.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/
CCM_MEMORY static Control_Reference control_reference;
CCM_MEMORY static Control_Data control_data;
CCM_MEMORY static Control_Limits control_limits;
CCM_MEMORY static Output_Mixer output_mixer;

/* RC Output Configuration */
static const PWMConfig pwmcfg = {
    RCOUTPUT_1MHZ_CLOCK_FREQUENCY,      /* 1 MHz PWM clock frequency    */
    RCOUTPUT_400HZ,                     /* Initial PWM period: 400 Hz   */
    NULL,                               /* No callback */
    {
        {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* Active high, no callback     */
        {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* Active high, no callback     */
        {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* Active high, no callback     */
        {PWM_OUTPUT_ACTIVE_HIGH, NULL}  /* Active high, no callback     */
    },
    0,
    0
};
static const RCOutput_Configuration rcoutputcfg = {
    &PWMD4,
    &PWMD8,
    &pwmcfg
};

CCM_MEMORY static THD_WORKING_AREA(waThreadControl, 256);

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/
static THD_FUNCTION(ThreadControl, arg)
{
    (void)arg;

    chRegSetThreadName("Control");

    /* Event registration for new estimation */
    event_listener_t el;

    /* Register to new estimation */
    chEvtRegisterMask(ptrGetEstimationEventSource(),
                      &el,
                      ESTIMATION_NEW_ESTIMATION_EVENTMASK);

    while (1)
    {
        /* Wait for new estimation */ 
        chEvtWaitOne(ESTIMATION_NEW_ESTIMATION_EVENTMASK);

    }

    return MSG_OK;
}

static void vRCInputsToControlAction(void)
{
}

/*static void vPositionControl(vector3f_t *position_m,
                             float dt)
{
}

static void vVelocityControl(vector3f_t *velocity_m,
                             float dt)
{
}*/

static void vAttitudeControl(quaternion_t *attitude_m,
                             float dt)
{
    (void)attitude_m;
    (void)dt;

    //vector3f_t u, error;

    /* *
     *
     * TODO: Calculate quaternion error.
     *
     * */

    // u.x = fPIUpdate(&control_data.attitude_controller[0], error.x, dt);
    // u.y = fPIUpdate(&control_data.attitude_controller[1], error.y, dt);
    // u.z = fPIUpdate(&control_data.attitude_controller[2], error.z, dt);

    /* Send bounded control signal to the next step in the cascade */
    //ref->rate_reference.x = bound( limits->max_rate.pitch,
    //                              -limits->max_rate.pitch,
    //                               u.x);
    //ref->rate_reference.y = bound( limits->max_rate.roll,
    //                              -limits->max_rate.roll,
    //                               u.y);
    //ref->rate_reference.z = bound( limits->max_rate.yaw,
    //                              -limits->max_rate.yaw,
    //                               u.z);
}

static void vRateControl(vector3f_t *omega_m,
                         float dt)
{
    vector3f_t u, error;

    /* Calculate the errors */
    error.x = control_reference.rate_reference.x - omega_m->x;
    error.y = control_reference.rate_reference.y - omega_m->y;
    error.z = control_reference.rate_reference.z - omega_m->z;

    /* Update the PI controllers */
    u.x = fPIUpdate(&control_data.rate_controller[0], error.x, dt);
    u.y = fPIUpdate(&control_data.rate_controller[1], error.y, dt);
    u.z = fPIUpdate(&control_data.rate_controller[2], error.z, dt);

    /* Send control signal to the next stage */
    control_reference.actuator_desired.pitch = u.x;
    control_reference.actuator_desired.roll = u.y;
    control_reference.actuator_desired.yaw = u.z;
    
}

static void vUpdateOutputs(void)
{
    float sum;
    int i;

    for (i = 0; i < 8; i++)
    {
        sum =  control_reference.actuator_desired.throttle * output_mixer.weights[i][0];
        sum += control_reference.actuator_desired.pitch    * output_mixer.weights[i][1];
        sum += control_reference.actuator_desired.roll     * output_mixer.weights[i][2];
        sum += control_reference.actuator_desired.yaw      * output_mixer.weights[i][3];

        RCOutputSetChannelWidthRelativePositive(&rcoutputcfg, i, sum);
    }
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/
void ControlInit(void)
{
    float *p;
    int i;

    /*
     * Initialize the RC Outputs
     */
    if (RCOutputInit(&rcoutputcfg) != MSG_OK)
        osalSysHalt("RC output init failed"); /* Initialization failed */

    /* Initialize all references to 0 and disarm controllers */
    p = (float *)&control_reference;

    for (i = 0; i < ((CONTROL_REFERENCE_SIZE - 2) / 4); i++)
        p[i] = 0.0f;

    control_reference.mode = FLIGHTMODE_DISARMED;
    control_reference.target = TARGET_GOAL;

    /* Initialize the controllers to 0 */
    p = (float *)&control_data;

    for (i = 0; i < (CONTROL_DATA_SIZE / 4); i++)
        p[i] = 0.0f;

    /* Initialize the limits to 0 */
    p = (float *)&control_limits;

    for (i = 0; i < (CONTROL_LIMITS_SIZE / 4); i++)
        p[i] = 0.0f;

    /* Initialize the mixer to 0 */
    p = (float *)&output_mixer;

    for (i = 0; i < (OUTPUT_MIXER_SIZE / 4); i++)
        p[i] = 0.0f;

    /* Initialize control thread */
    chThdCreateStatic(waThreadControl,
                      sizeof(waThreadControl),
                      HIGHPRIO - 1,
                      ThreadControl,
                      NULL);
}

void vUpdateControlAction(quaternion_t *q_m, vector3f_t *omega_m, float dt)
{
    vRCInputsToControlAction();

    switch (control_reference.mode)
    {
        //case FLIGHTMODE_POSITION_HOLD:
        //    break;

        //case FLIGHTMODE_POSITION:
        //    vPositionControl(ref, limits, controllers, dt);

        //case FLIGHTMODE_VELOCITY:
        //    vVelocityControl(ref, limits, controllers, dt);

        case FLIGHTMODE_ATTITUDE:
            vAttitudeControl(q_m,
                             dt);

        case FLIGHTMODE_RATE:
            vRateControl(omega_m,
                         dt);

        case FLIGHTMODE_DIRECT_CONTROL:
            vUpdateOutputs();
            break;

        case FLIGHTMODE_DISARMED:
        default:

            /* Disable all outputs */
            control_reference.actuator_desired.pitch = 0.0f;
            control_reference.actuator_desired.roll = 0.0f;
            control_reference.actuator_desired.yaw = 0.0f;
            control_reference.actuator_desired.throttle = 0.0f;

            /* Update outputs with zero control signal */
            vUpdateOutputs();
            break;
    }

}

Control_Data *ptrGetControlData(void)
{
    return &control_data;
}

Control_Limits *ptrGetControlLimits(void)
{
    return &control_limits;
}

Output_Mixer *ptrGetOutputMixer(void)
{
    return &output_mixer;
}
