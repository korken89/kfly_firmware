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

#include "string.h"
#include "ch.h"
#include "hal.h"
#include "control.h"
#include "flash_save.h"
#include "estimation.h"
#include "rc_output.h"
#include "rate_loop.h"
#include "attitude_loop.h"
#include "sensor_read.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

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
control_filters_t control_filters;

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
        vUpdateControlAction(&states->q, &states->w, SENSOR_ACCGYRO_DT);
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
        RCOutputSetChannelWidth(i, control_reference.output[i]);

    RCOutputSync();
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
 * @brief   Set D-term filters to a safe value
 */
static void DTermFilterDefaults(void)
{
  for (int i = 0; i < 3; i++)
  {
    control_filters.settings.dterm_cutoff[i] = 50.0f;
    control_filters.settings.dterm_filter_mode[i] = BIQUAD_MODE_BIQUAD;
  }
}



/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief           Initializes the entire control structure.
 */
void ControlInit(void)
{
    /* Initialize all references to 0 and disarm controllers. */
    memset((uint8_t *)&control_reference, 0, CONTROL_REFERENCE_SIZE);
    control_reference.mode = FLIGHTMODE_DISARMED;

    /* Initialize the controllers to 0. */
    memset((uint8_t *)&control_data, 0, CONTROL_DATA_SIZE);

    /* Initialize the limits to 0. */
    memset((uint8_t *)&control_limits, 0, CONTROL_LIMITS_SIZE);

    /* Initialize the mixer's weights to 0. */
    memset((uint8_t *)&output_mixer, 0, OUTPUT_MIXER_SIZE);

    /* Initializing computer control. */
    ComputerControlInit();

    /* Initialize arming. */
    ArmingInit();

    /* Dterm filters defaults */
    DTermFilterDefaults();

    /* Read data from flash (if available). */
    vReadControlParametersFromFlash();

    /* Initialize control filter */
    ControlFiltersInit();

    /* Set outputs to default value. */
    vSetOutputsDefault();

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
 * @brief   Calculate filter coefficients
 */
void ControlFiltersInit(void)
{
  for (int i = 0; i < 3; i++)
  {
    BiquadInitStateDF2T(&control_filters.dterm_biquads[i].state);

    const float cutoff = control_filters.settings.dterm_cutoff[i];
    const biquad_mode_t mode = control_filters.settings.dterm_filter_mode[i];

    // Sanity check
    if (cutoff > (SENSOR_ACCGYRO_HZ / 2) || cutoff < 1.0f)
      control_filters.settings.dterm_cutoff[i] = 50.0f;

    if (mode != BIQUAD_MODE_PT1 && mode != BIQUAD_MODE_BIQUAD)
      control_filters.settings.dterm_filter_mode[i] = BIQUAD_MODE_BIQUAD;


    // Init filters
    if (control_filters.settings.dterm_filter_mode[i] == BIQUAD_MODE_BIQUAD)
    {
      BiquadUpdateCoeffs(&control_filters.dterm_biquads[i].coeffs,
                         SENSOR_ACCGYRO_HZ,
                         control_filters.settings.dterm_cutoff[i],
                         ACCGYRO_BUTTERWORTH_Q,
                         BIQUAD_TYPE_LPF);
    }
    else
    {
      BiquadPT1LPFUpdateCoeffs(&control_filters.dterm_biquads[i].coeffs,
                               SENSOR_ACCGYRO_HZ,
                               control_filters.settings.dterm_cutoff[i]);
    }
  }
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

    if (bMotorOverrideActive())
    {
        control_reference.mode = FLIGHTMODE_DIRECT;
        vGetMotorOverrideValues(control_reference.output);
    }
    else if (bIsSystemArmed() == false)
    {
        control_reference.mode = FLIGHTMODE_DISARMED;
    }
    else if ((ComputerControlLinkActive() == true) &&
        (RCInputGetSwitchState(RCINPUT_ROLE_ENABLE_SERIAL_CONTROL) ==
            RCINPUT_SWITCH_POSITION_TOP))
    {
        control_reference.mode = ComputerControlGetFlightmode();

        if (control_reference.mode == FLIGHTMODE_ATTITUDE)
        {
            ComputerControlGetAttitudeReference(
                &control_reference.attitude_reference,
                &control_reference.actuator_desired.throttle);
        }
        else if (control_reference.mode == FLIGHTMODE_ATTITUDE_EULER)
        {
            ComputerControlGetAttitudeEulerReference(
                &control_reference.attitude_reference_euler,
                &control_reference.actuator_desired.throttle);
        }
        else if (control_reference.mode == FLIGHTMODE_RATE)
        {
            ComputerControlGetRateReference(
                &control_reference.rate_reference,
                &control_reference.actuator_desired.throttle);
        }
        else if (control_reference.mode == FLIGHTMODE_INDIRECT)
        {
            ComputerControlGetIndirectReference(
                &control_reference.actuator_desired.torque,
                &control_reference.actuator_desired.throttle);
        }
        else if (control_reference.mode == FLIGHTMODE_DIRECT)
        {
            ComputerControlGetDirectReference(control_reference.output);
        }
        else
        {
            /* Fallback - disarm. */
            control_reference.mode = FLIGHTMODE_DISARMED;
        }
    }
    else
    {
        /* TODO: Make this settable via software and switches. */
        control_reference.mode = FLIGHTMODE_RATE;

        RCInputsToControlAction(&control_reference,
                                &control_limits);
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
                                      &control_limits.max_rate.max_rate,
                                      (float *)&control_limits.max_angle,
                                      dt);
            }
            else
            {
                vAttitudeControl(&control_reference.attitude_reference,
                                 attitude_m,
                                 &control_reference.rate_reference,
                                 control_data.attitude_controller,
                                 &control_limits.max_rate.max_rate,
                                 dt);
            }

        case FLIGHTMODE_RATE:
            vRateControl(&control_reference.rate_reference,
                         rate_m,
                         &control_reference.actuator_desired.torque,
                         control_data.rate_controller,
                         control_filters.dterm_biquads,
                         dt);

        case FLIGHTMODE_INDIRECT:
            vUpdateOutputs();

        case FLIGHTMODE_DIRECT:
            vSendPWMCommands();

            /* Set the arming LED. */
            palSetPad(GPIOC, GPIOC_LED_ERR);

            break;

        case FLIGHTMODE_DISARMED:
        default:
            /* Disable all outputs. */
            vSetOutputsDefault();

            /* Clear the arming LED. */
            palClearPad(GPIOC, GPIOC_LED_ERR);

            /* Zero all the controllers' integrators. */
            vZeroControlIntegrals();
            break;
    }
}

/**
 * @brief   Zeros all control integrals.
 */
void vZeroControlIntegrals(void)
{
    int i;

    /* Zero each controllers Integral state. */
    for (i = 0; i < 3; i++)
    {
        control_data.attitude_controller[i].I_state = 0.0f;
        control_data.rate_controller[i].I_state = 0.0f;
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
 * @brief       Return the pointer to the control filters structure.
 *
 * @return      Pointer to the control filters structure.
 */
control_filter_settings_t *ptrGetControlFilters(void)
{
    return &control_filters.settings;
}

/**
 * @brief       Copies current PI control parameters to an external structure.
 * @param[out] param    Save location.
 */
void GetControlParameters(control_parameters_t *param)
{
  // Rate controller parameters
  for (int i = 0; i < 3; i++)
  {
    param->rate_parameters[i].P = control_data.rate_controller[i].gains.P;
    param->rate_parameters[i].I = control_data.rate_controller[i].gains.I;
    param->rate_parameters[i].D = control_data.rate_controller[i].gains.D;
    param->dterm_cutoff[i]      = control_filters.settings.dterm_cutoff[i];
    param->dterm_filter_mode[i] = control_filters.settings.dterm_filter_mode[i];
  }

  // Attitude controller parameters
  for (int i = 0; i < 3; i++)
  {
    param->attitude_parameters[i].P = control_data.attitude_controller[i].gains.P;
    param->attitude_parameters[i].I = control_data.attitude_controller[i].gains.I;
    param->attitude_parameters[i].D = control_data.attitude_controller[i].gains.D;
  }
}

/**
 * @brief       Saves PI control parameters from an external structure to the
 *              current PI control parameters.
 * @param[out] param    Copy location.
 */
void SetControlParameters(const control_parameters_t *param)
{
  for (int i = 0; i < 3; i++)
  {
    control_data.rate_controller[i].gains.P       = param->rate_parameters[i].P;
    control_data.rate_controller[i].gains.I       = param->rate_parameters[i].I;
    control_data.rate_controller[i].gains.D       = param->rate_parameters[i].D;
    control_filters.settings.dterm_cutoff[i]      = param->dterm_cutoff[i];
    control_filters.settings.dterm_filter_mode[i] = param->dterm_filter_mode[i];
  }

  // Attitude controller parameters
  for (int i = 0; i < 3; i++)
  {
    control_data.attitude_controller[i].gains.P = param->attitude_parameters[i].P;
    control_data.attitude_controller[i].gains.I = param->attitude_parameters[i].I;
    control_data.attitude_controller[i].gains.D = param->attitude_parameters[i].D;
  }
}

/**
 * @brief           Copies current control signals to data structure.
 * @param[out] sig  Save location.
 */
void GetControlSignals(control_signals_t *sig)
{
  osalSysLock();

  sig->torque = control_reference.actuator_desired.torque;
  sig->throttle = control_reference.actuator_desired.throttle;

  for (int i = 0; i < 8; i++)
    sig->motor_command[i] = control_reference.output[i];

  osalSysUnlock();
}

/**
 * @brief           Copies current control references to data structure.
 * @param[out] ref  Save location.
 */
void GetControlReference(control_reference_save_t *ref)
{
  osalSysLock();

  ref->attitude = control_reference.attitude_reference;
  ref->rate = control_reference.rate_reference;
  ref->throttle = control_reference.actuator_desired.throttle;

  osalSysUnlock();
}
