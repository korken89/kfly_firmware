/* *
 *
 * General control structure from attitude to motors:
 *
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
 * 3 outputs and 3 references when in manual control and the attitude is based
 * on the Quaternion Attitude Controller by Emil Fresk when running via
 * computer control.
 *
 * */

#include "ch.h"
#include "hal.h"
#include "control.h"
#include "flash_save.h"
#include "estimation.h"
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
control_reference_t control_reference;
control_data_t control_data;
control_limits_t control_limits;
output_mixer_t output_mixer;
control_parameters_t flash_save_control_parameters;
control_signals_t control_signals_save;
control_reference_save_t control_reference_save;

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

    /* Event registration for new estimation. */
    event_listener_t el;

    /* Estimation states. */
    attitude_states_t *states = ptrGetAttitudeEstimationStates();

    /* Set thread name. */
    chRegSetThreadName("Control");

    /* Register to new estimation. */
    chEvtRegisterMask(ptrGetEstimationEventSource(),
                      &el,
                      ESTIMATION_NEW_ESTIMATION_EVENTMASK);

    while (1)
    {
        /* Wait for new estimation. */
        chEvtWaitOne(ESTIMATION_NEW_ESTIMATION_EVENTMASK);

        /* Run control. */
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

    /* Event registration for flash save event. */
    event_listener_t el;

    /* Set thread name. */
    chRegSetThreadName("Control FlashSave");

    /* Register to flash save event. */
    chEvtRegisterMask(ptrGetFlashSaveEventSource(),
                      &el,
                      FLASHSAVE_SAVE_EVENTMASK);

    while (1)
    {
        /* Wait for flash save event. */
        chEvtWaitOne(FLASHSAVE_SAVE_EVENTMASK);

        /* Save Control Parameters. */
        FlashSave_Write(FlashSave_STR2ID("CONA"),
                        true,
                        (uint8_t *)ptrGetControlArmSettings(),
                        CONTROL_ARM_SIZE);

        /* Get the current control parameters. */
        GetControlParameters(&flash_save_control_parameters);

        /* Save Control Parameters. */
        FlashSave_Write(FlashSave_STR2ID("CONP"),
                        true,
                        (uint8_t *)&flash_save_control_parameters,
                        CONTROL_PARAMETERS_SIZE);

        /* Save Control Limits. */
        FlashSave_Write(FlashSave_STR2ID("CONL"),
                        true,
                        (uint8_t *)&control_limits,
                        CONTROL_LIMITS_SIZE);

        /* Save Output Mixer. */
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

    /* Read Arming Parameters. */
    FlashSave_Read(FlashSave_STR2ID("CONA"),
                   (uint8_t *)ptrGetControlArmSettings(),
                   CONTROL_ARM_SIZE);

    /* Read Control Parameters. */
    status = FlashSave_Read(FlashSave_STR2ID("CONP"),
                            (uint8_t *)&flash_save_control_parameters,
                            CONTROL_PARAMETERS_SIZE);

    /* Save the read control parameters. */
    if (status == FLASHSAVE_OK)
        SetControlParameters(&flash_save_control_parameters);

    /* Read Control Limits. */
    FlashSave_Read(FlashSave_STR2ID("CONL"),
                   (uint8_t *)&control_limits,
                   CONTROL_LIMITS_SIZE);

    /* Read Output Mixer. */
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

    /* Calculate the control signal for each PWM output. */
    for (i = 0; i < 8; i++)
    {
        /* Add the throttle weight. */
        sum =  control_reference.actuator_desired.throttle *
               output_mixer.weights[i][0];

        /* Add the roll (around x) weight. */
        sum += control_reference.actuator_desired.torque.x *
               output_mixer.weights[i][1];

        /* Add the pitch (around y) weight. */
        sum += control_reference.actuator_desired.torque.y *
               output_mixer.weights[i][2];

        /* Add the yaw (around z) weight. */
        sum += control_reference.actuator_desired.torque.z *
               output_mixer.weights[i][3];

        /* Add the channel offset. */
        sum += output_mixer.offset[i];

        /* Save the control command. */
        control_reference.output[i] = sum;
    }
}

/**
 * @brief   Takes the calculated control signals and sends the to the RC
 *          output subsystem.
 */
static void vSendPWMCommands(void)
{
    int i;

    /* The setting function bounds the control signal internally. */
    for (i = 0; i < 8; i++)
        RCOutputSetChannelWidthRelativePositive(i,
                                                control_reference.output[i]);
}

/**
 * @brief   Forces all outputs to their default value.
 */
static void vSetOutputsDefault(void)
{
    int i;

    for (i = 0; i < 8; i++)
        control_reference.output[i] = output_mixer.offset[i];

    vSendPWMCommands();
}

/**
 * @brief   Zeros all control integrals.
 */
static void vZeroControlIntegrals(void)
{
    int i;

    /* Cast the controller data into an array of PI controllers. */
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


    /* Initialize all references to 0 and disarm controllers. */
    p = (float *)&control_reference;

    /* TODO: Fix this hack... */
    for (i = 0; i < ((CONTROL_REFERENCE_SIZE - 2) / 4); i++)
        p[i] = 0.0f;

    control_reference.mode = FLIGHTMODE_DISARMED;

    /* Initialize the controllers to 0. */
    p = (float *)&control_data;

    for (i = 0; i < (CONTROL_DATA_SIZE / 4); i++)
        p[i] = 0.0f;

    /* Initialize the limits to 0. */
    p = (float *)&control_limits;

    for (i = 0; i < (CONTROL_LIMITS_SIZE / 4); i++)
        p[i] = 0.0f;

    /* Initialize the mixer's weights to 0. */
    p = (float *)&output_mixer;

    for (i = 0; i < (OUTPUT_MIXER_SIZE / 4); i++)
        p[i] = 0.0f;

    /* Set outputs to default value. */
    vSetOutputsDefault();

    /* Initializing computer control. */
    ComputerControlInit();

    /* Initialize arming. */
    ArmingInit();

    /* Read data from flash (if available). */
    vReadControlParametersFromFlash();

    /* Initialize control thread. */
    chThdCreateStatic(waThreadControl,
                      sizeof(waThreadControl),
                      HIGHPRIO - 2,
                      ThreadControl,
                      NULL);

    /* Initialize control flash save thread. */
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
void vUpdateControlAction(const quaternion_t *attitude_m,
                          const vector3f_t *rate_m,
                          const float dt)
{
    /* Check if manual control or computer control. */
    if (GetReferenceSource() == REFERENCE_SOURCE_MANUAL)
    {
        if (bIsSystemArmed() == true)
        {
            /* TODO: Make this settable via software and switches. */
            control_reference.mode = FLIGHTMODE_RATE;

            RCInputsToControlAction(&control_reference,
                                    &control_limits.max_rate,
                                    (vector3f_t *)&control_limits.max_angle);
        }
        else
        {
            /* When disarmed, set the throttle to the default. */
            control_reference.mode = FLIGHTMODE_INDIRECT;

            control_reference.actuator_desired.torque.x = 0.0f;
            control_reference.actuator_desired.torque.y = 0.0f;
            control_reference.actuator_desired.torque.z = 0.0f;
            control_reference.actuator_desired.throttle = fGetDisarmedThrottle();
        }

    }
    else
    {
        control_reference.mode = GetComputerFlightMode();

        if (control_reference.mode == FLIGHTMODE_ATTITUDE)
        {
            GetComputerAttitudeReference(&control_reference.attitude_reference,
                                         &control_reference.actuator_desired.throttle);
        }
        else if (control_reference.mode == FLIGHTMODE_RATE)
        {
            GetComputerRateReference(&control_reference.rate_reference,
                                     &control_reference.actuator_desired.throttle);
        }
        else if (control_reference.mode == FLIGHTMODE_INDIRECT)
        {
            GetComputerIndirectReference(&control_reference.actuator_desired.torque,
                                         &control_reference.actuator_desired.throttle);
        }
        else if (control_reference.mode == FLIGHTMODE_DIRECT)
        {
            GetComputerDirectReference(control_reference.output);
        }
        else
        {
            /* Fallback - disarm. */
            control_reference.mode = FLIGHTMODE_DISARMED;
        }
    }


    /* Apply the correct controller. */
    switch (control_reference.mode)
    {
        case FLIGHTMODE_ATTITUDE_EULER:
        case FLIGHTMODE_ATTITUDE:
            if (control_reference.mode == FLIGHTMODE_ATTITUDE_EULER)
            {
                vAttitudeControlEuler(&control_reference.attitude_reference_euler,
                                      attitude_m,
                                      &control_reference.rate_reference,
                                      control_data.attitude_controller,
                                      &control_limits.max_rate_attitude,
                                      dt);
            }
            else
            {
                vAttitudeControl(&control_reference.attitude_reference,
                                 attitude_m,
                                 &control_reference.rate_reference,
                                 control_data.attitude_controller,
                                 &control_limits.max_rate_attitude,
                                 dt);
            }

        case FLIGHTMODE_RATE:
            vRateControl(&control_reference.rate_reference,
                         rate_m,
                         &control_reference.actuator_desired.torque,
                         control_data.rate_controller,
                         dt);

        case FLIGHTMODE_INDIRECT:
            vUpdateOutputs();

        case FLIGHTMODE_DIRECT:
            vSendPWMCommands();
            break;

        case FLIGHTMODE_DISARMED:
        default:
            /* Disable all outputs. */
            vSetOutputsDefault();

            /* Zero all the controllers' integrators. */
            vZeroControlIntegrals();
            break;
    }

    osalSysLock();

    /* Save control signals for transfer under lock to prevent data race. */
    control_signals_save.throttle = control_reference.actuator_desired.throttle;
    control_signals_save.torque = control_reference.actuator_desired.torque;

    control_reference_save.throttle = control_reference.actuator_desired.throttle;
    control_reference_save.attitude = control_reference.attitude_reference;
    control_reference_save.rate = control_reference.rate_reference;

    osalSysUnlock();
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

    /* Cast parameters and PI controller to arrays of respective kind. */
    pi_data_t *pi = (pi_data_t *)&control_data;
    pi_parameters_t *par = (pi_parameters_t *)param;

    for (i = 0; i < CONTROL_NUMBER_OF_CONTROLLERS; i++)
    {
        /* Cast each of the PI and parameters to float arrays
           for easy copying. */
        f_pi = (float *)&pi[i];
        f_par = (float *)&par[i];

        /* Copy PI parameters to the external location. */
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

    /* Cast parameters and PI controller to arrays of respective kind. */
    pi_data_t *pi = (pi_data_t *)&control_data;
    pi_parameters_t *par = (pi_parameters_t *)param;

    for (i = 0; i < CONTROL_NUMBER_OF_CONTROLLERS; i++)
    {
        /* Cast each of the PI and parameters to float arrays
           for easy copying. */
        f_pi = (float *)&pi[i];
        f_par = (float *)&par[i];

        /* Save PI parameters from the external location. */
        for (j = 0; j < PI_NUM_PARAMETERS; j++)
            f_pi[j] = f_par[j];
    }
}

/**
 * @brief           Copies current control singals to data structure.
 * @param[out] sig  Save location.
 */
void GetControlSignals(control_signals_t *sig)
{
    sig->throttle = control_signals_save.throttle;
    sig->torque = control_signals_save.torque;
}

/**
 * @brief           Copies current control references to data structure.
 * @param[out] ref  Save location.
 */
void GetControlReference(control_reference_save_t *ref)
{
    ref->throttle = control_reference_save.throttle;
    ref->attitude = control_reference_save.attitude;
    ref->rate = control_reference_save.rate;
}

