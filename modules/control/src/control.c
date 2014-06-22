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
static Control_Data control_data;
static Control_Limits control_limits;
static Output_Mixer output_mixer;

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

    /* Event registration for new estimation */
    event_listener_t el;

    /* Estimation states */
    Attitude_Estimation_States *states = ptrGetAttitudeEstimationStates();

    /* Set thread name */
    chRegSetThreadName("Control");

    /* Register to new estimation */
    chEvtRegisterMask(ptrGetEstimationEventSource(),
                      &el,
                      ESTIMATION_NEW_ESTIMATION_EVENTMASK);

    while (1)
    {
        /* Wait for new estimation */ 
        chEvtWaitOne(ESTIMATION_NEW_ESTIMATION_EVENTMASK);

        /* Run control */
        vUpdateControlAction(&states->q, &states->w, ESTIMATION_DT);
    }

    return MSG_OK;
}

static void vRCInputsToControlAction(void)
{
}

/*
static void vPositionControl(vector3f_t *position_m,
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
    vector3f_t u;
    quaternion_t qerr;

    /* Calculate the quaternion error */
    qerr = qmult(control_reference.attitude_reference, qconj(*attitude_m));

    /* Check if the error represents the shortest route */
    if (qerr.q0 < 0.0f)
        qerr = qconj(qerr);

    /* Update controllers */
    u.x = fPIUpdate(&control_data.attitude_controller[0], qerr.q1, dt);
    u.y = fPIUpdate(&control_data.attitude_controller[1], qerr.q2, dt);
    u.z = fPIUpdate(&control_data.attitude_controller[2], qerr.q3, dt);

    /* Send bounded control signal to the next step in the cascade */
    control_reference.rate_reference.x = bound( control_limits.max_rate.pitch,
                                               -control_limits.max_rate.pitch,
                                                u.x);
    control_reference.rate_reference.y = bound( control_limits.max_rate.roll,
                                               -control_limits.max_rate.roll,
                                                u.y);
    control_reference.rate_reference.z = bound( control_limits.max_rate.yaw,
                                               -control_limits.max_rate.yaw,
                                                u.z);
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
        sum =  control_reference.actuator_desired.throttle *
               output_mixer.weights[i][0];
        sum += control_reference.actuator_desired.pitch *
               output_mixer.weights[i][1];
        sum += control_reference.actuator_desired.roll *
               output_mixer.weights[i][2];
        sum += control_reference.actuator_desired.yaw *
               output_mixer.weights[i][3];

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
        case FLIGHTMODE_POSITION_HOLD:
        //    break;

        case FLIGHTMODE_POSITION:
        //    vPositionControl(pos_m, dt);

        case FLIGHTMODE_VELOCITY:
        //    vVelocityControl(vel_m, dt);

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
