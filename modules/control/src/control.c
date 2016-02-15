/* *
 *
 * General control structure from position to motors:
 *                 __________        __________        __________
 *                |          |      |          |      |          |
 *            +-> | Attitude | -+-> |   Rate   | -+-> |  Motors  |
 *            |   |__________|  |   |__________|  |   |__________|
 *            |                 |                 |
 *             /                 /                 / <-- Switch
 * Reference -+-----------------+-----------------+
 *
 * Aim:
 * To be able to connect a reference anywhere in the
 * chain in order to choose what type of control to use.
 * This is done by the generic control structure with a
 * setting of the current control mode.
 *
 * Every block is an vector PI controller (except Motors) with 3 inputs,
 * 3 outputs and 3 references.
 *
 * */

#include "ch.h"
#include "hal.h"
#include "quaternion.h"
#include "flash_save.h"
#include "estimation.h"
#include "arming.h"
#include "control.h"
#include "rc_output.h"
#include "rate_loop.h"
#include "attitude_loop.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

static void vZeroControlIntegrals(void);

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/
static control_reference_t control_reference;
static control_data_t control_data;
static control_limits_t control_limits;
static output_mixer_t output_mixer;
static control_parameters_t flash_save_control_parameters;

THD_WORKING_AREA(waThreadControl, 256);
THD_WORKING_AREA(waThreadControlFlashSave, 256);

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/


/**
 * @brief           Thread for the entire control structure.
 *
 * @param[in] arg   Unused.
 * @return          Unused.
 */
static THD_FUNCTION(ThreadControl, arg)
{
    (void)arg;

    /* Event registration for new estimation */
    event_listener_t el;

    /* Estimation states */
    attitude_states_t *states = ptrGetAttitudeEstimationStates();

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
}

/**
 * @brief           Thread for the flash save operation.
 *
 * @param[in] arg   Unused.
 * @return          Unused.
 */
static THD_FUNCTION(ThreadControlFlashSave, arg)
{
    (void)arg;

    /* Event registration for flash save event */
    event_listener_t el;

    /* Set thread name */
    chRegSetThreadName("Control FlashSave");

    /* Register to flash save event */
    chEvtRegisterMask(ptrGetFlashSaveEventSource(),
                      &el,
                      FLASHSAVE_SAVE_EVENTMASK);

    while (1)
    {
        /* Wait for flash save event */
        chEvtWaitOne(FLASHSAVE_SAVE_EVENTMASK);

        /* Save Control Parameters */
        FlashSave_Write(FlashSave_STR2ID("CONA"),
                        true,
                        (uint8_t *)&arm_settings,
                        CONTROL_ARM_SIZE);

        /* Get the current control parameters */
        GetControlParameters(&flash_save_control_parameters);

        /* Save Control Parameters */
        FlashSave_Write(FlashSave_STR2ID("CONP"),
                        true,
                        (uint8_t *)&flash_save_control_parameters,
                        CONTROL_PARAMETERS_SIZE);

        /* Save Control Limits */
        FlashSave_Write(FlashSave_STR2ID("CONL"),
                        true,
                        (uint8_t *)&control_limits,
                        CONTROL_LIMITS_SIZE);

        /* Save Output Mixer */
        FlashSave_Write(FlashSave_STR2ID("CONM"),
                        true,
                        (uint8_t *)&output_mixer,
                        OUTPUT_MIXER_SIZE);
    }
}

/**
 * @brief   Read all control parameters from flash.
 */
static void vReadControlParametersFromFlash(void)
{
    FlashSave_Status status;

    /* Read Arming Parameters */
    FlashSave_Read(FlashSave_STR2ID("CONA"),
                   (uint8_t *)&arm_settings,
                   CONTROL_ARM_SIZE);

    /* Read Control Parameters */
    status = FlashSave_Read(FlashSave_STR2ID("CONP"),
                            (uint8_t *)&flash_save_control_parameters,
                            CONTROL_PARAMETERS_SIZE);

    /* Save the read control parameters */
    if (status == FLASHSAVE_OK)
        SetControlParameters(&flash_save_control_parameters);

    /* Read Control Limits */
    FlashSave_Read(FlashSave_STR2ID("CONL"),
                   (uint8_t *)&control_limits,
                   CONTROL_LIMITS_SIZE);

    /* Read Output Mixer */
    FlashSave_Read(FlashSave_STR2ID("CONM"),
                   (uint8_t *)&output_mixer,
                   OUTPUT_MIXER_SIZE);
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
        /* Add the throttle weight */
        sum =  control_reference.actuator_desired.throttle *
               output_mixer.weights[i][0];

        /* Add the pitch weight */
        sum += control_reference.actuator_desired.pitch *
               output_mixer.weights[i][1];

        /* Add the roll weight */
        sum += control_reference.actuator_desired.roll *
               output_mixer.weights[i][2];

        /* Add the yaw weight */
        sum += control_reference.actuator_desired.yaw *
               output_mixer.weights[i][3];

        /* Add the channel offset */
        sum += output_mixer.offset[i];

        control_reference.pwm_out[i] = bound(1.0f, 0.0f, sum);
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
        RCOutputSetChannelWidthRelativePositive(i,
                                                control_reference.pwm_out[i]);
}

/**
 * @brief   Forces all RC outputs to zero.
 */
static void vDisableAllOutputs(void)
{
    int i;

    for (i = 0; i < 8; i++)
        control_reference.pwm_out[i] = output_mixer.offset[i];

    vSendPWMCommands();
}

/**
 * @brief   Zeros all control integrals.
 */
static void vZeroControlIntegrals(void)
{
    int i;

    /* Cast the controller data into an array of PI controllers */
    pi_data_t *pi = (pi_data_t *)&control_data;

    /* Zero each controllers Integral state. */
    for (i = 0; i < CONTROL_NUMBER_OF_CONTROLLERS; i++)
        pi[i].I_state = 0.0f;
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
    uint32_t i;


    /* Initialize all references to 0 and disarm controllers */
    p = (float *)&control_reference;

    /* TODO: Fix this hack... */
    for (i = 0; i < ((CONTROL_REFERENCE_SIZE - 2) / 4); i++)
        p[i] = 0.0f;

    control_reference.mode = FLIGHTMODE_DISARMED;

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

    vDisableAllOutputs();


    /* Initialize arming */
    ArmingInit(&arm_settings);

    /* Initialize control thread */
    chThdCreateStatic(waThreadControl,
                      sizeof(waThreadControl),
                      HIGHPRIO - 2,
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
        case FLIGHTMODE_ATTITUDE:
            vAttitudeControl(q_m,
                             false,
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

            /* Zero all the controllers' integrators */
            vZeroControlIntegrals();
            break;
    }
}

/**
 * @brief       Return the pointer to the control reference structure.
 *
 * @return      Pointer to the control reference structure.
 */
control_reference_t *ptrGetControlReferences(void)
{
    return &control_reference;
}

/**
 * @brief       Return the pointer to the control data structure.
 *
 * @return      Pointer to the control data structure.
 */
control_data_t *ptrGetControlData(void)
{
    return &control_data;
}

/**
 * @brief       Return the pointer to the control limits structure.
 *
 * @return      Pointer to the control limits structure.
 */
control_limits_t *ptrGetControlLimits(void)
{
    return &control_limits;
}

/**
 * @brief       Return the pointer to the output mixer structure.
 *
 * @return      Pointer to the output mixer structure.
 */
output_mixer_t *ptrGetOutputMixer(void)
{
    return &output_mixer;
}

/**
 * @brief       Copies current PI control parameters to an external structure.
 * @param[out] param    Save location.
 */
void GetControlParameters(control_parameters_t *param)
{
    int i, j;
    float *f_pi, *f_par;

    /* Cast parameters and PI controller to arrays of respective kind */
    pi_data_t *pi = (pi_data_t *)&control_data;
    pi_parameters_t *par = (pi_parameters_t *)param;

    for (i = 0; i < CONTROL_NUMBER_OF_CONTROLLERS; i++)
    {
        /* Cast each of the PI and parameters to float arrays
           for easy copying */
        f_pi = (float *)&pi[i];
        f_par = (float *)&par[i];

        /* Copy PI parameters to the external location */
        for (j = 0; j < PI_NUM_PARAMETERS; j++)
            f_par[j] = f_pi[j];
    }
}

/**
 * @brief       Saves PI control parameters from an external structure to the
 *              current PI control parameters.
 * @param[out] param    Copy location.
 */
void SetControlParameters(control_parameters_t *param)
{
    int i, j;
    float *f_pi, *f_par;

    /* Cast parameters and PI controller to arrays of respective kind */
    pi_data_t *pi = (pi_data_t *)&control_data;
    pi_parameters_t *par = (pi_parameters_t *)param;

    for (i = 0; i < CONTROL_NUMBER_OF_CONTROLLERS; i++)
    {
        /* Cast each of the PI and parameters to float arrays
           for easy copying */
        f_pi = (float *)&pi[i];
        f_par = (float *)&par[i];

        /* Save PI parameters from the external location */
        for (j = 0; j < PI_NUM_PARAMETERS; j++)
            f_pi[j] = f_par[j];
    }
}
