#ifndef __EICU_LLD_H
#define __EICU_LLD_H

#include "stm32_tim.h"


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
  EICU_UNINIT = 0,						/* Not initialized.					  */
  EICU_STOP = 1,						/* Stopped.							  */
  EICU_READY = 2,						/* Ready.							  */
  EICU_WAITING = 3,						/* Waiting for first edge.			  */
  EICU_ACTIVE = 4,						/* Active cycle phase.				  */
  EICU_IDLE = 5							/* Idle cycle phase.				  */
} eicustate_t;

/**
 * @brief   Input type selector.
 */
typedef enum {
  EICU_INPUT_EDGE = 0,					/* Triggers callback on input edge */
  EICU_INPUT_PULSE = 1,					/* Triggers callback on detected
  										   pulse */
  EICU_INPUT_PWM = 2					/* Triggers callback on detected PWM
										   period and width */
} eicuinput_t;

/** 
 * @brief	EICU channel selection definition
 */
typedef enum {
	EICU_CHANNEL_1 = 0,
	EICU_CHANNEL_2 = 1,
	EICU_CHANNEL_3 = 2,
	EICU_CHANNEL_4 = 3
} eicuchannel_t;

/** 
 * @brief	EICU PWM channel selection definition
 */
typedef enum {
	EICU_PWM_CHANNEL_1 = 0,
	EICU_PWM_CHANNEL_2 = 1
} eicupwmchannel_t;


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

typedef struct EICUDriver EICUDriver;

/**
 * @brief	EICU notification callback type.
 *
 * @param[in] eicup 	Pointer to a EICUDriver object
 * @param[in] channel 	EICU channel that fired the interrupt
 */
typedef void (*eicucallback_t)(EICUDriver *eicup, eicuchannel_t channel);

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
} EICU_Config;

/** 
 * @brief	EICU Input Capture Driver structure definition  
 */
struct EICUDriver
{
	stm32_tim_t *tim;					/* Timer peripheral for
										   Input Capture. 					  */

	eicustate_t state;					/* Driver state. 					  */

	uint32_t clock;						/* Timer base clock. 				  */

	EICU_Config *config;				/* Pointer to configuration for the
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
 * @brief   Common ISR code, EICU width event.
 *
 * @param[in] icup      Pointer to the EICUDriver object
 *
 * @notapi
 */
#define _eicu_isr_invoke_width_cb(eicup, n) {                                \
  if ((eicup)->state != EICU_WAITING) {                                      \
    (eicup)->state = EICU_IDLE;                                              \
    (eicup)->config->iccfgp[n]->width_cb(eicup, n);                          \
  }                                                                          \
}

/**
 * @brief   Common ISR code, EICU period event.
 *
 * @param[in] icup      Pointer to the EICUDriver object
 *
 * @notapi
 */
#define _eicu_isr_invoke_period_cb(eicup, n) {                               \
  eicustate_t previous_state = (eicup)->state;                               \
  (eicup)->state = EICU_ACTIVE;                                              \
  if (previous_state != EICU_WAITING)                                        \
    (eicup)->config->iccfgp[n]->period_cb(eicup, n);                         \
}

/**
 * @brief   Common ISR code, EICU timer overflow event.
 *
 * @param[in] icup      Pointer to the EICUDriver object
 *
 * @notapi
 */
#define _eicu_isr_invoke_overflow_cb(icup, n) {                              \
  (eicup)->config->iccfgp[n]->overflow_cb(eicup, n);                         \
}

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
  void eicu_lld_start(EICUDriver *icup);
  void eicu_lld_stop(EICUDriver *icup);
  void eicu_lld_enable(EICUDriver *icup);
  void eicu_lld_disable(EICUDriver *icup);
#ifdef __cplusplus
}
#endif

#endif /* __EICU_LLD_H */
