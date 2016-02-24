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

    /* Read data from flash (if available). */
    vReadControlParametersFromFlash();

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
            /* When disarmed, set the throttle disarmed. */
            control_reference.mode = FLIGHTMODE_DISARMED;
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

    void RecordData(void);
    RecordData();

}

/**
 * @brief   Zeros all control integrals.
 */
void vZeroControlIntegrals(void)
{
    int i;

    /* Cast the controller data into an array of PI controllers. */
    pi_data_t *pi = (pi_data_t *)&control_data;

    /* Zero each controllers Integral state. */
    for (i = 0; i < CONTROL_NUMBER_OF_CONTROLLERS; i++)
        pi[i].I_state = 0.0f;
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


/*
 *
 * Temporary test code
 *
 */
#include "sensor_read.h"

typedef struct PACKED_VAR
{
    int16_t acc_z;
    int16_t gyro[3];
    int16_t torque[3];
    int16_t throttle;
    uint16_t cnt;
} experiment_data_t;

volatile experiment_data_t exp_data_1[4600];
volatile experiment_data_t exp_data_2[3600] CCM_MEMORY;

void RecordData(void)
{
    const int skip = 2; // Must be larger than 0.

    static int cnt = 0;
    static uint16_t save_cnt = 0;
    static vector3f_t gyro_mean, torque_mean;
    static float throttle_mean, acc_mean;

    uint16_t save_cnt_tmp;

    if (cnt == 0)
    {
        gyro_mean = ptrGetAttitudeEstimationStates()->w;
        acc_mean = ptrGetAccelerometerData()[2];
        torque_mean = control_reference.actuator_desired.torque;
        throttle_mean = control_reference.actuator_desired.throttle;

        cnt++;
    }
    else if (cnt == skip)
    {
        gyro_mean = vector_add(gyro_mean, ptrGetAttitudeEstimationStates()->w);
        acc_mean += ptrGetAccelerometerData()[2];
        torque_mean = vector_add(torque_mean, control_reference.actuator_desired.torque);
        throttle_mean += control_reference.actuator_desired.throttle;

        cnt = 0;

        gyro_mean = vector_scale(gyro_mean, 1.0f / (float)(skip + 1));
        torque_mean = vector_scale(torque_mean, 1.0f / (float)(skip + 1));
        throttle_mean = throttle_mean / ((float)(skip + 1));

        /* Save data. */
        if (save_cnt < 4600)
        {
            save_cnt_tmp = save_cnt/(skip + 1);

            /* Use first save location. */
            exp_data_1[save_cnt_tmp].cnt = save_cnt / (skip + 1);
            exp_data_1[save_cnt_tmp].acc_z = (int16_t)(acc_mean);
            exp_data_1[save_cnt_tmp].throttle = (int16_t)(throttle_mean * 10000.0f);

            exp_data_1[save_cnt_tmp].gyro[0] = (int16_t)(gyro_mean.x * 10000.0f);
            exp_data_1[save_cnt_tmp].gyro[1] = (int16_t)(gyro_mean.y * 10000.0f);
            exp_data_1[save_cnt_tmp].gyro[2] = (int16_t)(gyro_mean.z * 10000.0f);

            exp_data_1[save_cnt_tmp].torque[0] = (int16_t)(torque_mean.x * 10000.0f);
            exp_data_1[save_cnt_tmp].torque[1] = (int16_t)(torque_mean.y * 10000.0f);
            exp_data_1[save_cnt_tmp].torque[2] = (int16_t)(torque_mean.z * 10000.0f);
        }
        else if (save_cnt < 4600 + 3600 && save_cnt >= 4600)
        {
            save_cnt_tmp = (save_cnt - 4600)/(skip + 1);

            /* Use second save location. */
            exp_data_2[save_cnt_tmp].cnt = save_cnt / (skip + 1);
            exp_data_2[save_cnt_tmp].acc_z = (int16_t)(acc_mean);
            exp_data_2[save_cnt_tmp].throttle = (int16_t)(throttle_mean * 10000.0f);

            exp_data_2[save_cnt_tmp].gyro[0] = (int16_t)(gyro_mean.x * 10000.0f);
            exp_data_2[save_cnt_tmp].gyro[1] = (int16_t)(gyro_mean.y * 10000.0f);
            exp_data_2[save_cnt_tmp].gyro[2] = (int16_t)(gyro_mean.z * 10000.0f);

            exp_data_2[save_cnt_tmp].torque[0] = (int16_t)(torque_mean.x * 10000.0f);
            exp_data_2[save_cnt_tmp].torque[1] = (int16_t)(torque_mean.y * 10000.0f);
            exp_data_2[save_cnt_tmp].torque[2] = (int16_t)(torque_mean.z * 10000.0f);
        }
        else
        {
            /* Transmit data. */
            chSysHalt("Data captured");
        }

    }
    else
    {
        gyro_mean = vector_add(gyro_mean, ptrGetAttitudeEstimationStates()->w);
        acc_mean += ptrGetAccelerometerData()[2];
        torque_mean = vector_add(torque_mean, control_reference.actuator_desired.torque);
        throttle_mean += control_reference.actuator_desired.throttle;

        cnt++;
    }

    save_cnt++;
}
