/* *
 *
 * Hardware Abstraction Layer for Extended Input Capture Unit
 *
 * */

#include "ch.h"
#include "hal.h"
#include "extended_icu.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/
/**
 * @brief   EICUD1 driver identifier.
 * @note    The driver EICUD1 allocates the complex timer TIM1 when enabled.
 */
#if STM32_EICU_USE_TIM1 && !defined(__DOXYGEN__)
EICUDriver EICUD1;
#endif

/**
 * @brief   EICUD2 driver identifier.
 * @note    The driver EICUD2 allocates the timer TIM2 when enabled.
 */
#if STM32_EICU_USE_TIM2 && !defined(__DOXYGEN__)
EICUDriver EICUD2;
#endif

/**
 * @brief   EICUD3 driver identifier.
 * @note    The driver EICUD3 allocates the timer TIM3 when enabled.
 */
#if STM32_EICU_USE_TIM3 && !defined(__DOXYGEN__)
EICUDriver EICUD3;
#endif

/**
 * @brief   EICUD4 driver identifier.
 * @note    The driver EICUD4 allocates the timer TIM4 when enabled.
 */
#if STM32_EICU_USE_TIM4 && !defined(__DOXYGEN__)
EICUDriver EICUD4;
#endif

/**
 * @brief   EICUD5 driver identifier.
 * @note    The driver EICUD5 allocates the timer TIM5 when enabled.
 */
#if STM32_EICU_USE_TIM5 && !defined(__DOXYGEN__)
EICUDriver EICUD5;
#endif

/**
 * @brief   EICUD8 driver identifier.
 * @note    The driver EICUD8 allocates the timer TIM8 when enabled.
 */
#if STM32_EICU_USE_TIM8 && !defined(__DOXYGEN__)
EICUDriver EICUD8;
#endif

/**
 * @brief   EICUD9 driver identifier.
 * @note    The driver EICUD9 allocates the timer TIM9 when enabled.
 */
#if STM32_EICU_USE_TIM9 && !defined(__DOXYGEN__)
EICUDriver EICUD9;
#endif

/**
 * @brief   EICUD12 driver identifier.
 * @note    The driver EICUD12 allocates the timer TIM12 when enabled.
 */
#if STM32_EICU_USE_TIM12 && !defined(__DOXYGEN__)
EICUDriver EICUD12;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
/**
 * @brief   Shared IRQ handler.
 *
 * @param[in] icup      pointer to the @p ICUDriver object
 */
static void eicu_lld_serve_interrupt(EICUDriver *eicup)
{
	uint16_t sr;
	sr = eicup->tim->SR;

	/* Clear interrupts */
	eicup->tim->SR = ~sr;

	if (eicup->config->pwmcfg->channel == EICU_PWM_CHANNEL_1)
	{
		if ((sr & STM32_TIM_SR_CC1IF) != 0)
			_eicu_isr_invoke_period_cb(eicup, EICU_CHANNEL_1);
		if ((sr & STM32_TIM_SR_CC2IF) != 0)
			_eicu_isr_invoke_width_cb(eicup, EICU_CHANNEL_1);
	}
	else
	{
		if ((sr & STM32_TIM_SR_CC1IF) != 0)
			_eicu_isr_invoke_width_cb(eicup, EICU_CHANNEL_2);
		if ((sr & STM32_TIM_SR_CC2IF) != 0)
			_eicu_isr_invoke_period_cb(eicup, EICU_CHANNEL_2);
	}

	if ((sr & STM32_TIM_SR_UIF) != 0)
		_eicu_isr_invoke_overflow_cb(eicup, EICU_CHANNEL_1);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if STM32_EICU_USE_TIM1
#if STM32_ICU_USE_TIM1
#error "The ICU1 driver is active, it must be disabled to use this driver."
#endif
#if !defined(STM32_TIM1_UP_HANDLER)
#error "STM32_TIM1_UP_HANDLER not defined"
#endif
/**
 * @brief   TIM1 compare interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_TIM1_UP_HANDLER) {

  CH_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD1);

  CH_IRQ_EPILOGUE();
}

#if !defined(STM32_TIM1_CC_HANDLER)
#error "STM32_TIM1_CC_HANDLER not defined"
#endif
/**
 * @brief   TIM1 compare interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_TIM1_CC_HANDLER) {

  CH_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD1);

  CH_IRQ_EPILOGUE();
}
#endif /* STM32_EICU_USE_TIM1 */

#if STM32_EICU_USE_TIM2
#if STM32_ICU_USE_TIM2
#error "The ICU2 driver is active, it must be disabled to use this driver."
#endif
#if !defined(STM32_TIM2_HANDLER)
#error "STM32_TIM2_HANDLER not defined"
#endif
/**
 * @brief   TIM2 interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_TIM2_HANDLER) {

  CH_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD2);

  CH_IRQ_EPILOGUE();
}
#endif /* STM32_EICU_USE_TIM2 */

#if STM32_EICU_USE_TIM3
#if STM32_ICU_USE_TIM3
#error "The ICU3 driver is active, it must be disabled to use this driver."
#endif
#if !defined(STM32_TIM3_HANDLER)
#error "STM32_TIM3_HANDLER not defined"
#endif
/**
 * @brief   TIM3 interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_TIM3_HANDLER) {

  CH_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD3);

  CH_IRQ_EPILOGUE();
}
#endif /* STM32_EICU_USE_TIM3 */

#if STM32_EICU_USE_TIM4
#if STM32_ICU_USE_TIM4
#error "The ICU4 driver is active, it must be disabled to use this driver."
#endif
#if !defined(STM32_TIM4_HANDLER)
#error "STM32_TIM4_HANDLER not defined"
#endif
/**
 * @brief   TIM4 interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_TIM4_HANDLER) {

  CH_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD4);

  CH_IRQ_EPILOGUE();
}
#endif /* STM32_EICU_USE_TIM4 */

#if STM32_EICU_USE_TIM5
#if STM32_ICU_USE_TIM5
#error "The ICU5 driver is active, it must be disabled to use this driver."
#endif
#if !defined(STM32_TIM5_HANDLER)
#error "STM32_TIM5_HANDLER not defined"
#endif
/**
 * @brief   TIM5 interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_TIM5_HANDLER) {

  CH_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD5);

  CH_IRQ_EPILOGUE();
}
#endif /* STM32_EICU_USE_TIM5 */

#if STM32_EICU_USE_TIM8
#if STM32_ICU_USE_TIM8
#error "The ICU8 driver is active, it must be disabled to use this driver."
#endif
#if !defined(STM32_TIM8_UP_HANDLER)
#error "STM32_TIM8_UP_HANDLER not defined"
#endif
/**
 * @brief   TIM8 compare interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_TIM8_UP_HANDLER) {

  CH_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD8);

  CH_IRQ_EPILOGUE();
}

#if !defined(STM32_TIM8_CC_HANDLER)
#error "STM32_TIM8_CC_HANDLER not defined"
#endif
/**
 * @brief   TIM8 compare interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_TIM8_CC_HANDLER) {

  CH_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD8);

  CH_IRQ_EPILOGUE();
}
#endif /* STM32_EICU_USE_TIM8 */

#if STM32_EICU_USE_TIM9
#if STM32_ICU_USE_TIM9
#error "The ICU9 driver is active, it must be disabled to use this driver."
#endif
#if !defined(STM32_TIM9_HANDLER)
#error "STM32_TIM9_HANDLER not defined"
#endif
/**
 * @brief   TIM9 interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_TIM9_HANDLER) {

  CH_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD9);

  CH_IRQ_EPILOGUE();
}
#endif /* STM32_EICU_USE_TIM9 */

#if STM32_EICU_USE_TIM12
#if STM32_ICU_USE_TIM12
#error "The ICU12 driver is active, it must be disabled to use this driver."
#endif
#if !defined(STM32_TIM12_HANDLER)
#error "STM32_TIM12_HANDLER not defined"
#endif
/**
 * @brief   TIM12 interrupt handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_TIM12_HANDLER) {

  CH_IRQ_PROLOGUE();

  eicu_lld_serve_interrupt(&EICUD12);

  CH_IRQ_EPILOGUE();
}
#endif /* STM32_EICU_USE_TIM9 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/