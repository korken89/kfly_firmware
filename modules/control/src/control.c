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
#include "flash_save.h"
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
static Control_Limits control_limits;
static Output_Mixer output_mixer;
static Control_Parameters flash_save_control_parameters;
    
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
CCM_MEMORY static THD_WORKING_AREA(waThreadControlFlashSave, 64);

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief           Thread for the entire control structure.
 * 
 * @param[in] arg   Unused.
 * 
 * @return          Unused.
 */
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

/**
 * @brief           Thread for the flash save operation.
 * 
 * @param[in] arg   Unused.
 * 
 * @return          Unused.
 */
static THD_FUNCTION(ThreadControlFlashSave, arg)
{
    (void)arg;

    /* Event registration for new estimation */
    event_listener_t el;

    /* Set thread name */
    chRegSetThreadName("Control Save");

    /* Register to new estimation */
    chEvtRegisterMask(ptrGetFlashSaveEventSource(),
                      &el,
                      FLASHSAVE_SAVE_EVENTMASK);

    while (1)
    {
        /* Wait for new estimation */ 
        chEvtWaitOne(FLASHSAVE_SAVE_EVENTMASK);

        /* Get the current control parameters */
        GetControlParameters(&flash_save_control_parameters);

        /* Save Control Parameters */
        FlashSave_Write(FlashSave_STR2ID("CON1"),
                        true,
                        (uint8_t *)&flash_save_control_parameters,
                        CONTROL_PARAMETERS_SIZE);

        /* Save Control Limits */
        FlashSave_Write(FlashSave_STR2ID("CON2"),
                        true,
                        (uint8_t *)&control_limits,
                        CONTROL_LIMITS_SIZE);

        /* Save Output Mixer */
        FlashSave_Write(FlashSave_STR2ID("CON3"),
                        true,
                        (uint8_t *)&output_mixer,
                        OUTPUT_MIXER_SIZE);
    }

    return MSG_OK;
}

/**
 * @brief   Read all control parameters from flash.
 */
static void vReadControlParametersFromFlash(void)
{
    FlashSave_Status status;

    /* Read Control Parameters */
    status = FlashSave_Read(FlashSave_STR2ID("CON1"),
                            (uint8_t *)&flash_save_control_parameters,
                            CONTROL_PARAMETERS_SIZE);

    /* Save the read control parameters */
    if (status == FLASHSAVE_OK)
        SetControlParameters(&flash_save_control_parameters);

    /* Read Control Limits */
    FlashSave_Read(FlashSave_STR2ID("CON2"),
                   (uint8_t *)&control_limits,
                   CONTROL_LIMITS_SIZE);

    /* Read Output Mixer */
    FlashSave_Read(FlashSave_STR2ID("CON3"),
                   (uint8_t *)&output_mixer,
                   OUTPUT_MIXER_SIZE);
}

/**
 * @brief           Converts RC inputs to control action depending on
 *                  the current flight mode.
 */
static void vRCInputsToControlAction(void)
{
}

/**
 * @brief           Implements the position controller.
 * 
 * @param[in] position_m    Position measurement.
 * @param[in] dt            Controller sampling time.
 */
static void vPositionControl(vector3f_t *position_m, float dt)
{
    (void)position_m;
    (void)dt;
}

/**
 * @brief           Implements the velocity controller.
 * 
 * @param[in] velocity_m    Velocity measurement.
 * @param[in] dt            Controller sampling time.
 */
static void vVelocityControl(vector3f_t *velocity_m, float dt)
{
    (void)velocity_m;
    (void)dt;
}

/**
 * @brief           Implements the attitude controller.
 * 
 * @param[in] attitude_m    Attitude measurement.
 * @param[in] dt            Controller sampling time.
 */
static void vAttitudeControl(quaternion_t *attitude_m, float dt)
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

/**
 * @brief           Implements the rate controller.
 * 
 * @param[in] omega_m   Rate measurement.
 * @param[in] dt        Controller sampling time.
 */
static void vRateControl(vector3f_t *omega_m, float dt)
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

/**
 * @brief   Calculates the control signals based on the output weighting
 *          matrix and the desired torque around each axis plus throttle.
 */
static void vUpdateOutputs(void)
{
    float sum;
    int i;

    /* Calculate the control signal for each PWM output */
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

        control_reference.pwm_out[i] = sum;
    }
}

/**
 * @brief   Takes the calculated control signals and sends the to the RC
 *          output subsystem.
 */
static void vSendPWMCommands(void)
{
    int i;

    for (i = 0; i < 8; i++)
        RCOutputSetChannelWidthRelativePositive(&rcoutputcfg,
                                                i,
                                                control_reference.pwm_out[i]);
}

/**
 * @brief   Forces all RC outputs to zero.
 */
static void vDisableAllOutputs(void)
{
    int i;

    for (i = 0; i < 8; i++)
        control_reference.pwm_out[i] = 0.0f;

    vSendPWMCommands();
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief           Initializes the entire control structure.
 */
void ControlInit(void)
{
    float *p;
    int i;

    /* Initialize the RC Outputs */
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

    /* Initialize the mixer's weights to 0 */
    p = (float *)&output_mixer;

    for (i = 0; i < (OUTPUT_MIXER_SIZE / 4); i++)
        p[i] = 0.0f;

    /* Read data from flash (if available) */
    vReadControlParametersFromFlash();

    /* Initialize control thread */
    chThdCreateStatic(waThreadControl,
                      sizeof(waThreadControl),
                      HIGHPRIO - 1,
                      ThreadControl,
                      NULL);

    /* Initialize control flash save thread */
    chThdCreateStatic(waThreadControlFlashSave,
                      sizeof(waThreadControlFlashSave),
                      NORMALPRIO,
                      ThreadControlFlashSave,
                      NULL);
}

/**
 * @brief       Updates all the controllers depending om current flight mode.
 * 
 * @param[in] q_m       Attitude measurement.
 * @param[in] omega_m   Rate measurement.
 * @param[in] dt        Controller sampling rate.
 */
void vUpdateControlAction(quaternion_t *q_m, vector3f_t *omega_m, float dt)
{
    vRCInputsToControlAction();

    switch (control_reference.mode)
    {
        //case FLIGHTMODE_POSITION_HOLD:
        //    break;

        case FLIGHTMODE_POSITION:
            vPositionControl(NULL, dt);

        case FLIGHTMODE_VELOCITY:
            vVelocityControl(NULL, dt);

        case FLIGHTMODE_ATTITUDE:
            vAttitudeControl(q_m,
                             dt);

        case FLIGHTMODE_RATE:
            vRateControl(omega_m,
                         dt);

        case FLIGHTMODE_DIRECT_CONTROL:
            vUpdateOutputs();

        case FLIGTMODE_DIRECT_PWM:
            vSendPWMCommands();
            break;

        case FLIGHTMODE_DISARMED:
        default:
            /* Disable all outputs */
            vDisableAllOutputs();
            break;
    }
}

/**
 * @brief       Return the pointer to the control reference structure.
 * 
 * @return      Pointer to the control reference structure.
 */
Control_Reference *ptrGetControlReferences(void)
{
    return &control_reference;
}

/**
 * @brief       Return the pointer to the control data structure.
 * 
 * @return      Pointer to the control data structure.
 */
Control_Data *ptrGetControlData(void)
{
    return &control_data;
}

/**
 * @brief       Return the pointer to the control limits structure.
 * 
 * @return      Pointer to the control limits structure.
 */
Control_Limits *ptrGetControlLimits(void)
{
    return &control_limits;
}

/**
 * @brief       Return the pointer to the output mixer structure.
 * 
 * @return      Pointer to the output mixer structure.
 */
Output_Mixer *ptrGetOutputMixer(void)
{
    return &output_mixer;
}

/**
 * @brief       Copies current PI control parameters to an external structure.
 * @param[out] param    Save location.
 */
void GetControlParameters(Control_Parameters *param)
{
    int i, j;
    float *f_pi, *f_par;

    /* Cast parameters and PI controller to arrays of respective kind */
    PI_Data *pi = (PI_Data *)&control_data;
    PI_Parameters *par = (PI_Parameters *)param;

    for (i = 0; i < CONTROL_NUMBER_OF_CONTROLLERS; i++)
    {
        /* Cast each of the PI and parameters to float arrays
           for easy copying */
        f_pi = (float *)&pi[i];
        f_par = (float *)&par[i];

        /* Copy PI parameters to the external location */
        for (j = 0; j < 3; j++)
            f_par[j] = f_pi[j];
    }
}

/**
 * @brief       Saves PI control parameters from an external structure to the
 *              current PI control parameters.
 * @param[out] param    Copy location.
 */
void SetControlParameters(Control_Parameters *param)
{
    int i, j;
    float *f_pi, *f_par;

    /* Cast parameters and PI controller to arrays of respective kind */
    PI_Data *pi = (PI_Data *)&control_data;
    PI_Parameters *par = (PI_Parameters *)param;

    for (i = 0; i < CONTROL_NUMBER_OF_CONTROLLERS; i++)
    {
        /* Cast each of the PI and parameters to float arrays
           for easy copying */
        f_pi = (float *)&pi[i];
        f_par = (float *)&par[i];

        /* Save PI parameters from the external location */
        for (j = 0; j < 3; j++)
            f_pi[j] = f_par[j];
    }
}
