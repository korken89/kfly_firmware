#ifndef _EICU_H_
#define _EICU_H_

#if HAL_USE_EICU || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  EICU_UNINIT = 0,              /* Not initialized.                           */
  EICU_STOP = 1,                /* Stopped.                                   */
  EICU_READY = 2,               /* Ready.                                     */
  EICU_WAITING = 3,             /* Waiting for first edge.                    */
  EICU_ACTIVE = 4,              /* Active cycle phase.                        */
  EICU_IDLE = 5                 /* Idle cycle phase.                          */
} eicustate_t;

/** 
 * @brief EICU channel selection definition
 */
typedef enum {
  EICU_CHANNEL_1 = 0,
  EICU_CHANNEL_2 = 1,
  EICU_CHANNEL_3 = 2,
  EICU_CHANNEL_4 = 3
} eicuchannel_t;

/**
 * @brief   Type of a structure representing an EICU driver.
 */
typedef struct EICUDriver EICUDriver;

/**
 * @brief EICU notification callback type.
 *
 * @param[in] eicup   Pointer to a EICUDriver object
 * @param[in] channel   EICU channel that fired the interrupt
 */
typedef void (*eicucallback_t)(EICUDriver *eicup, eicuchannel_t channel);

#include "eicu_lld.h"

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Enables the extended input capture.
 *
 * @param[in] eicup     Pointer to the @p EICUDriver object
 *
 * @iclass
 */
#define eicuEnableI(eicup) eicu_lld_enable(eicup)

/**
 * @brief   Disables the extended input capture.
 *
 * @param[in] eicup     Pointer to the @p EICUDriver object
 *
 * @iclass
 */
#define eicuDisableI(eicup) eicu_lld_disable(eicup)

/**
 * @brief   Returns the width of the latest pulse.
 * @details The pulse width is defined as number of ticks between the start
 *          edge and the stop edge.
 * @note    This function is meant to be invoked from the width capture
 *          callback only.
 *
 * @param[in] eicup     Pointer to the @p EICUDriver object
 * @param[in] channel   The timer channel that fired the interrupt.
 * @return              The number of ticks.
 *
 * @special
 */
#define eicuGetWidth(eicup, channel) eicu_lld_get_width((eicup), (channel))

/**
 * @brief   Returns the width of the latest cycle.
 * @details The cycle width is defined as number of ticks between a start
 *          edge and the next start edge.
 * @note    This function is meant to be invoked from the width capture
 *          callback only.
 *
 * @param[in] eicup     Pointer to the @p EICUDriver object
 * @return              The number of ticks.
 *
 * @special
 */
#define eicuGetPeriod(eicup) eicu_lld_get_period(eicup)
/** @} */

/**
 * @name    Low Level driver helper macros
 * @{
 */
/**
 * @brief   Common ISR code, EICU PWM width event.
 *
 * @param[in] eicup     Pointer to the EICUDriver object
 * @param[in] channel   The timer channel that fired the interrupt.
 *
 * @notapi
 */
#define _eicu_isr_invoke_pwm_width_cb(eicup, channel) {                        \
  if ((eicup)->state != EICU_WAITING) {                                        \
    (eicup)->state = EICU_IDLE;                                                \
    (eicup)->config->iccfgp[channel]->width_cb((eicup), (channel));            \
  }                                                                            \
}

/**
 * @brief   Common ISR code, EICU PWM period event.
 *
 * @param[in] icup      Pointer to the EICUDriver object
 * @param[in] channel   The timer channel that fired the interrupt.
 *
 * @notapi
 */
#define _eicu_isr_invoke_pwm_period_cb(eicup, channel) {                       \
  eicustate_t previous_state = (eicup)->state;                                 \
  (eicup)->state = EICU_ACTIVE;                                                \
  if (previous_state != EICU_WAITING)                                          \
    (eicup)->config->period_cb((eicup), (channel));                            \
}

/**
 * @brief   Common ISR code, EICU Pulse width event.
 * @details This macro needs special care since it needs to invert the
 *          correct polarity bit to detect pulses.
 * @note    This macro assumes that the polarity is not changed by some
 *          external user. It must only be changed using the HAL.
 * 
 * @param[in] icup      Pointer to the EICUDriver object
 * @param[in] channel   The timer channel that fired the interrupt.
 *
 * @notapi
 */
#define _eicu_isr_invoke_pulse_width_cb(eicup, channel) {                      \
  if ((eicup)->state == EICU_ACTIVE) {                                         \
    (eicup)->state = EICU_READY;                                               \
    eicu_lld_invert_polarity((eicup), (channel));                              \
    (eicup)->config->iccfgp[(channel)]->width_cb((eicup), (channel));          \
  } else {                                                                     \
    (eicup)->state = EICU_ACTIVE;                                              \
    (eicup)->last_count[(channel)] = eicu_lld_get_compare((eicup), (channel)); \
    eicu_lld_invert_polarity((eicup), (channel));                              \
  }                                                                            \
}

/**
 * @brief   Common ISR code, EICU Edge detect event.
 *
 * @param[in] icup      Pointer to the EICUDriver object
 * @param[in] channel   The timer channel that fired the interrupt.
 *
 * @notapi
 */
#define _eicu_isr_invoke_edge_detect_cb(eicup, channel) {                      \
  (eicup)->state = EICU_READY;                                                 \
  (eicup)->config->iccfgp[(channel)]->width_cb((eicup), (channel));            \
}

/**
 * @brief   Common ISR code, EICU timer overflow event.
 *
 * @param[in] icup      Pointer to the EICUDriver object
 *
 * @notapi
 */
#define _eicu_isr_invoke_overflow_cb(icup) {                                   \
  (eicup)->config->overflow_cb(eicup, 0);                                      \
}
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void eicuInit(void);
  void eicuObjectInit(EICUDriver *eicup);
  void eicuStart(EICUDriver *eicup, const EICUConfig *config);
  void eicuStop(EICUDriver *eicup);
  void eicuEnable(EICUDriver *eicup);
  void eicuDisable(EICUDriver *eicup);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_EICU */

#endif /* _EICU_H_ */

/** @} */
