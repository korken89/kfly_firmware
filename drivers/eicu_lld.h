#ifndef __EICU_LLD_H
#define __EICU_LLD_H

#include "stm32_tim.h"

#if HAL_USE_EICU || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/* Input capture Polarity */
#define EICU_ICPolarity_Rising				((uint16_t)0x0000)
#define EICU_ICPolarity_Falling				((uint16_t)0x0002)
#define EICU_ICPolarity_BothEdge			((uint16_t)0x000A)

/* Input capture selection */
#define EICU_ICSelection_DirectTI			((uint16_t)0x0001)
#define EICU_ICSelection_IndirectTI 		((uint16_t)0x0002)

/* Input capture prescaler */
#define EICU_ICPSC_DIV1						((uint16_t)0x0000)
#define EICU_ICPSC_DIV2						((uint16_t)0x0004)
#define EICU_ICPSC_DIV4						((uint16_t)0x0008)
#define EICU_ICPSC_DIV8						((uint16_t)0x000C)

/* Interrupts */
#define EICU_IT_Update						((uint16_t)0x0001)
#define EICU_IT_CC1							((uint16_t)0x0002)
#define EICU_IT_CC2							((uint16_t)0x0004)
#define EICU_IT_CC3							((uint16_t)0x0008)
#define EICU_IT_CC4							((uint16_t)0x0010)

/* Input Trigger Sources */
#define EICU_TS_TI1FP1						((uint16_t)0x0050)
#define EICU_TS_TI2FP2						((uint16_t)0x0060)

#define EICU_SlaveMode_Reset				((uint16_t)0x0004)


/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   EICUD1 driver enable switch.
 * @details If set to @p TRUE the support for EICUD1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_EICU_USE_TIM1) || defined(__DOXYGEN__)
#define STM32_EICU_USE_TIM1                  FALSE
#endif

/**
 * @brief   EICUD2 driver enable switch.
 * @details If set to @p TRUE the support for EICUD2 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_EICU_USE_TIM2) || defined(__DOXYGEN__)
#define STM32_EICU_USE_TIM2                  FALSE
#endif

/**
 * @brief   EICUD3 driver enable switch.
 * @details If set to @p TRUE the support for EICUD3 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_EICU_USE_TIM3) || defined(__DOXYGEN__)
#define STM32_EICU_USE_TIM3                  FALSE
#endif

/**
 * @brief   EICUD4 driver enable switch.
 * @details If set to @p TRUE the support for EICUD4 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_EICU_USE_TIM4) || defined(__DOXYGEN__)
#define STM32_EICU_USE_TIM4                  FALSE
#endif

/**
 * @brief   EICUD5 driver enable switch.
 * @details If set to @p TRUE the support for EICUD5 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_EICU_USE_TIM5) || defined(__DOXYGEN__)
#define STM32_EICU_USE_TIM5                  FALSE
#endif

/**
 * @brief   EICUD8 driver enable switch.
 * @details If set to @p TRUE the support for EICUD8 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_EICU_USE_TIM8) || defined(__DOXYGEN__)
#define STM32_EICU_USE_TIM8                  FALSE
#endif

/**
 * @brief   EICUD9 driver enable switch.
 * @details If set to @p TRUE the support for EICUD9 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_EICU_USE_TIM9) || defined(__DOXYGEN__)
#define STM32_EICU_USE_TIM9                  FALSE
#endif

/**
 * @brief   EICUD12 driver enable switch.
 * @details If set to @p TRUE the support for EICUD12 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_EICU_USE_TIM12) || defined(__DOXYGEN__)
#define STM32_EICU_USE_TIM12                 FALSE
#endif

/**
 * @brief   EICUD1 interrupt priority level setting.
 */
#if !defined(STM32_EICU_TIM1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EICU_TIM1_IRQ_PRIORITY         7
#endif

/**
 * @brief   EICUD2 interrupt priority level setting.
 */
#if !defined(STM32_EICU_TIM2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EICU_TIM2_IRQ_PRIORITY         7
#endif

/**
 * @brief   EICUD3 interrupt priority level setting.
 */
#if !defined(STM32_EICU_TIM3_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EICU_TIM3_IRQ_PRIORITY         7
#endif

/**
 * @brief   EICUD4 interrupt priority level setting.
 */
#if !defined(STM32_EICU_TIM4_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EICU_TIM4_IRQ_PRIORITY         7
#endif

/**
 * @brief   EICUD5 interrupt priority level setting.
 */
#if !defined(STM32_EICU_TIM5_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EICU_TIM5_IRQ_PRIORITY         7
#endif

/**
 * @brief   EICUD8 interrupt priority level setting.
 */
#if !defined(STM32_EICU_TIM8_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EICU_TIM8_IRQ_PRIORITY         7
#endif

/**
 * @brief   EICUD9 interrupt priority level setting.
 */
#if !defined(STM32_EICU_TIM9_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EICU_TIM9_IRQ_PRIORITY         7
#endif

/**
 * @brief   EICUD12 interrupt priority level setting.
 */
#if !defined(STM32_EICU_TIM12_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_EICU_TIM12_IRQ_PRIORITY        7
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if STM32_EICU_USE_TIM1 && !STM32_HAS_TIM1
#error "TIM1 not present in the selected device"
#endif

#if STM32_EICU_USE_TIM2 && !STM32_HAS_TIM2
#error "TIM2 not present in the selected device"
#endif

#if STM32_EICU_USE_TIM3 && !STM32_HAS_TIM3
#error "TIM3 not present in the selected device"
#endif

#if STM32_EICU_USE_TIM4 && !STM32_HAS_TIM4
#error "TIM4 not present in the selected device"
#endif

#if STM32_EICU_USE_TIM5 && !STM32_HAS_TIM5
#error "TIM5 not present in the selected device"
#endif

#if STM32_EICU_USE_TIM8 && !STM32_HAS_TIM8
#error "TIM8 not present in the selected device"
#endif

#if STM32_EICU_USE_TIM9 && !STM32_HAS_TIM9
#error "TIM9 not present in the selected device"
#endif

#if STM32_EICU_USE_TIM12 && !STM32_HAS_TIM12
#error "TIM12 not present in the selected device"
#endif

#if !STM32_EICU_USE_TIM1 && !STM32_EICU_USE_TIM2 &&                           \
    !STM32_EICU_USE_TIM3 && !STM32_EICU_USE_TIM4 &&                           \
    !STM32_EICU_USE_TIM5 && !STM32_EICU_USE_TIM8 &&                           \
    !STM32_EICU_USE_TIM9 && !STM32_EICU_USE_TIM9
#error "EICU driver activated but no TIM peripheral assigned"
#endif

#if STM32_EICU_USE_TIM1 &&                                                   \
    !CORTEX_IS_VALID_KERNEL_PRIORITY(STM32_EICU_TIM1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM1"
#endif

#if STM32_EICU_USE_TIM2 &&                                                   \
    !CORTEX_IS_VALID_KERNEL_PRIORITY(STM32_EICU_TIM2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM2"
#endif

#if STM32_EICU_USE_TIM3 &&                                                   \
    !CORTEX_IS_VALID_KERNEL_PRIORITY(STM32_EICU_TIM3_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM3"
#endif

#if STM32_EICU_USE_TIM4 &&                                                   \
    !CORTEX_IS_VALID_KERNEL_PRIORITY(STM32_EICU_TIM4_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM4"
#endif

#if STM32_EICU_USE_TIM5 &&                                                   \
    !CORTEX_IS_VALID_KERNEL_PRIORITY(STM32_EICU_TIM5_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM5"
#endif

#if STM32_EICU_USE_TIM8 &&                                                   \
    !CORTEX_IS_VALID_KERNEL_PRIORITY(STM32_EICU_TIM8_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM8"
#endif

#if STM32_EICU_USE_TIM9 &&                                                   \
    !CORTEX_IS_VALID_KERNEL_PRIORITY(STM32_EICU_TIM9_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM9"
#endif

#if STM32_EICU_USE_TIM12 &&                                                   \
    !CORTEX_IS_VALID_KERNEL_PRIORITY(STM32_EICU_TIM12_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to TIM12"
#endif

#if STM32_EICU_USE_TIM1 && STM32_ICU_USE_TIM1
#error "The ICU1 driver is active, it must be disabled to use this driver."
#endif

#if STM32_EICU_USE_TIM2 && STM32_ICU_USE_TIM2
#error "The ICU2 driver is active, it must be disabled to use this driver."
#endif

#if STM32_EICU_USE_TIM3 && STM32_ICU_USE_TIM3
#error "The ICU3 driver is active, it must be disabled to use this driver."
#endif

#if STM32_EICU_USE_TIM4 && STM32_ICU_USE_TIM4
#error "The ICU4 driver is active, it must be disabled to use this driver."
#endif

#if STM32_EICU_USE_TIM5 && STM32_ICU_USE_TIM5
#error "The ICU5 driver is active, it must be disabled to use this driver."
#endif

#if STM32_EICU_USE_TIM8 && TM32_ICU_USE_TIM8
#error "The ICU8 driver is active, it must be disabled to use this driver."
#endif

#if STM32_EICU_USE_TIM9 && STM32_ICU_USE_TIM9
#error "The ICU9 driver is active, it must be disabled to use this driver."
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/
/**
 * @brief   EICU frequency type.
 */
typedef uint32_t eicufreq_t;

/**
 * @brief   EICU period type.
 */
typedef uint32_t eicuper_t;

/**
 * @brief   EICU counter type.
 */
typedef uint16_t eicucnt_t;

/** 
 * @brief	EICU Time Base Settings structure definition
 */
typedef struct
{
	eicufreq_t frequency; /* Specifies the Timer clock in Hz. 				  */

	eicuper_t period;	 /* Specifies the period value of the timer. This
							parameter can be between 0x0000 and 0xFFFF.  	 */
} EICU_TimeBase_Settings; 

/** 
 * @brief	EICU Input Capture Settings structure definition  
 */
typedef struct
{
	uint16_t EICU_ICPolarity;	/* Specifies the active edge of the input
								   signal. 									  */

	uint16_t EICU_ICSelection;  /* Specifies the input selection. 			  */

	uint16_t EICU_ICPrescaler;  /* Specifies the Input Capture Prescaler. 	  */

	uint16_t EICU_ICFilter;     /* Specifies the input capture filter. This
								   parameter can be a number between
								   0x0 and 0xF. 							  */

	eicucallback_t width_cb;	/* Capture event callback. Used for PWM width,
								   Pulse width and normal capture event.	  */

	eicucallback_t period_cb;	/* Period capture event callback. 			  */

	eicucallback_t overflow_cb;	/* Timer overflow event callback. 			  */
} EICU_IC_Settings;

typedef struct 
{
	eicupwmchannel_t channel;			/* Timer input channel to be used for
										   PWM input 						  */

	uint16_t EICU_InputTriggerSource;	/* Select the input trigger for PWM
										   measurement mode. This parameter
										   can be EICU_TS_TI1FP1 or 
										   EICU_TS_TI2FP2. 					  */
} EICU_PWM_Settings;

/** 
 * @brief	EICU Input Capture Config structure definition  
 */
typedef struct
{
	eicuinput_t input_type;				/* Select which input type the driver 
										   will be configured for			  */

	EICU_TimeBase_Settings tbcfg;		/* TimeBase settings. 				  */

	EICU_IC_Settings *iccfgp[4];		/* Pointer to each Input Capture channel
										   configuration. A NULL parameter
										   indicates the channel as unused. 
										   Note: In PWM mode, only Channel 1 OR
										   Channel 2 may be used. 			  */

	EICU_PWM_Settings *pwmcfg;			/* Pointer to the PWM input
										   configuration. A NULL parameter
										   indicates the feature as unused.   */
} EICUConfig;

/** 
 * @brief	EICU Input Capture Driver structure definition  
 */
struct EICUDriver
{
	stm32_tim_t *tim;					/* Timer peripheral for
										   Input Capture. 					  */

	eicustate_t state;					/* Driver state. 					  */

	uint32_t clock;						/* Timer base clock. 				  */

	const EICUConfig *config;			/* Pointer to configuration for the
										   driver. 							  */

	volatile uint32_t *wccrp[4];		/* CCR registers for width capture.   */

	volatile uint32_t *pccrp;			/* CCR register for period capture.
										   Only one is needed since only one
										   PWM input per timer is allowed. 	  */
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Returns the width of the latest pulse.
 * @details The pulse width is defined as number of ticks between the start
 *          edge and the stop edge.
 *
 * @param[in] eicup     Pointer to the EICUDriver object.
 * @param[in] channel   The timer channel that fired the interrupt.
 * @return              The number of ticks.
 *
 * @notapi
 */
#define eicu_lld_get_width(eicup, channel) (*((eicup)->wccrp[channel]) + 1)

/**
 * @brief   Returns the width of the latest cycle.
 * @details The cycle width is defined as number of ticks between a start
 *          edge and the next start edge.
 *
 * @param[in] eicup     Pointer to the EICUDriver object.
 * @param[in] channel   The timer channel that fired the interrupt.
 * @return              The number of ticks.
 *
 * @notapi
 */
#define eicu_lld_get_period(eicup) (*((eicup)->pccrp) + 1)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
#if STM32_EICU_USE_TIM1 && !defined(__DOXYGEN__)
extern EICUDriver EICUD1;
#endif

#if STM32_EICU_USE_TIM2 && !defined(__DOXYGEN__)
extern EICUDriver EICUD2;
#endif

#if STM32_EICU_USE_TIM3 && !defined(__DOXYGEN__)
extern EICUDriver EICUD3;
#endif

#if STM32_EICU_USE_TIM4 && !defined(__DOXYGEN__)
extern EICUDriver EICUD4;
#endif

#if STM32_EICU_USE_TIM5 && !defined(__DOXYGEN__)
extern EICUDriver EICUD5;
#endif

#if STM32_EICU_USE_TIM8 && !defined(__DOXYGEN__)
extern EICUDriver EICUD8;
#endif

#if STM32_EICU_USE_TIM9 && !defined(__DOXYGEN__)
extern EICUDriver EICUD9;
#endif

#if STM32_EICU_USE_TIM12 && !defined(__DOXYGEN__)
extern EICUDriver EICUD12;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void eicu_lld_init(void);
  void eicu_lld_start(EICUDriver *eicup);
  void eicu_lld_stop(EICUDriver *eicup);
  void eicu_lld_enable(EICUDriver *eicup);
  void eicu_lld_disable(EICUDriver *eicup);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_EICU */

#endif /* __EICU_LLD_H */
