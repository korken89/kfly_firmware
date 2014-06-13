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
#include "ch.h"
#include "hal.h"
#include "control.h"
#include "rc_output.h"

/* Private Defines */

/* Private Typedefs */

/* Private variable defines */
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

/* Global variable defines */

/* Private function defines */
static void vPositionControl(Control_Reference *ref,
                             Control_Limits *limits,
                             float dt);
static void vVelocityControl(Control_Reference *ref,
                             Control_Limits *limits,
                             float dt);
static void vAttitudeControl(Control_Reference *ref,
                             Control_Limits *limits,
                             float dt);
static void vRateControl(Control_Reference *ref,
                         Control_Limits *limits,
                         float dt);
static void vUpdateOutputs(float u_throttle,
                           float u_pitch,
                           float u_roll,
                           float u_yaw);

void ControlInit(void)
{
    float *p;
    int i;

    /*
     *
     * Initialize the RC Outputs
     *
     */
    if (RCOutputInit(&rcoutputcfg) != MSG_OK)
        osalSysHalt("RC output init failed"); /* Initialization failed */

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
}

void vUpdateControlAction(Control_Reference *ref,
                          Control_Limits *limits,
                          float dt)
{
    switch (ref->mode)
    {
        case FLIGHTMODE_POSITION_HOLD:

            break;

        case FLIGHTMODE_POSITION:
            vPositionControl(ref, limits, dt);
            vVelocityControl(ref, limits, dt);
            vAttitudeControl(ref, limits, dt);
            vRateControl(ref, limits, dt);
            break;

        case FLIGHTMODE_VELOCITY:
            vVelocityControl(ref, limits, dt);
            vAttitudeControl(ref, limits, dt);
            vRateControl(ref, limits, dt);
            break;

        case FLIGHTMODE_ATTITUDE:
            vAttitudeControl(ref, limits, dt);
            vRateControl(ref, limits, dt);
            break;

        case FLIGHTMODE_RATE:
            vRateControl(ref, limits, dt);
            break;

        case FLIGHTMODE_DISARMED:
        default:
            vUpdateOutputs(0.0f, 0.0f, 0.0f, 0.0f);
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


/* *
 *
 * Private functions
 *
 * */

static void vPositionControl(Control_Reference *ref,
                             Control_Limits *limits,
                             float dt)
{
    (void)limits;
    (void)ref;
    (void)dt;

    //vector3f_t u, error;

    //error.x = ref->position_reference.x - 0.0f; /* Add where the position can be read */
    //error.y = ref->position_reference.y - 0.0f; /* Add where the position can be read */
    //error.z = ref->position_reference.z - 0.0f; /* Add where the position can be read */

    //u.x = fPIUpdate(&control_data.position_controller[0], error.x, dt);
    //u.y = fPIUpdate(&control_data.position_controller[1], error.y, dt);
    //u.z = fPIUpdate(&control_data.position_controller[2], error.z, dt);

    /* Send control signal to the next step in the cascade */
    //return u;

}

static void vVelocityControl(Control_Reference *ref,
                             Control_Limits *limits,
                             float dt)
{
    (void)limits;
    (void)ref;
    (void)dt;

    //vector3f_t u, error;

    /* *
     *
     * TODO: Add the saturation of velocities.
     *
     * */

    //error.x = ref->velocity_reference.x - 0.0f; /* Add where the velocity can be read */
    //error.y = ref->velocity_reference.y - 0.0f; /* Add where the velocity can be read */
    //error.z = ref->velocity_reference.z - 0.0f; /* Add where the velocity can be read */

    //u.x = fPIUpdate(&control_data.velocity_controller[0], error.x, dt);
    //u.y = fPIUpdate(&control_data.velocity_controller[1], error.y, dt);
    //u.z = fPIUpdate(&control_data.velocity_controller[2], error.z, dt);

    /* *
     *
     * TODO: Add the conversion block for targeting.
     *
     * */

    /* Send control signal to the next step in the cascade */
    //return u;
}

static void vAttitudeControl(Control_Reference *ref,
                             Control_Limits *limits,
                             float dt)
{
    (void)limits;
    (void)ref;
    (void)dt;

    //vector3f_t u, error;

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

    // u.x = fPIUpdate(&control_data.attitude_controller[0], error.x, dt);
    // u.y = fPIUpdate(&control_data.attitude_controller[1], error.y, dt);
    // u.z = fPIUpdate(&control_data.attitude_controller[2], error.z, dt);

    /* Send control signal to the next step in the cascade */
    
}

static void vRateControl(Control_Reference *ref,
                         Control_Limits *limits,
                         float dt)
{
    (void)limits;
    (void)ref;
    (void)dt;

    //vector3f_t u, error;


    /* *
     *
     * TODO: Add the saturation of rotational rates.
     *
     * */

    //error.x = ref->rate_reference.x - 0.0f; /* Add where the rotational rate can be read */
    //error.y = ref->rate_reference.y - 0.0f; /* Add where the rotational rate can be read */
    //error.z = ref->rate_reference.z - 0.0f; /* Add where the rotational rate can be read */

    //u.x = fPIUpdate(&control_data.rate_controller[0], error.x, dt);
    //u.y = fPIUpdate(&control_data.rate_controller[1], error.y, dt);
    //u.z = fPIUpdate(&control_data.rate_controller[2], error.z, dt);

    /* Send control signal to the engines */
    //vUpdateOutputs(control_reference.throttle, u.x, u.y, u.z);
}

static void vUpdateOutputs(float u_throttle,
                           float u_pitch,
                           float u_roll,
                           float u_yaw)
{
    float sum;
    int i;

    for (i = 0; i < 8; i++)
    {
        sum =  u_throttle * output_mixer.weights[i][0];
        sum += u_pitch    * output_mixer.weights[i][1];
        sum += u_roll     * output_mixer.weights[i][2];
        sum += u_yaw      * output_mixer.weights[i][3];

        RCOutputSetChannelWidthRelativePositive(&rcoutputcfg, i, sum);
    }
}
