#ifndef __EXTENDED_ICU_H
#define __EXTENDED_ICU_H

#include "stm32_tim.h"

/* Defines */
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

/* Global variable defines */


/* Typedefs */
/** 
 * @brief	EICU channel selection definition
 */
typedef enum {
	EICU_Channel_1 = 0,
	EICU_Channel_2 = 1,
	EICU_Channel_3 = 2,
	EICU_Channel_4 = 3,
} EICU_Channel;

/**
 * @brief	EICU notification callback type.
 *
 * @param[in] eicup 	Pointer to a EICUDriver object
 * @param[in] channel 	EICU channel that fired the interrupt
 */
typedef void (*eicucallback_t)(ICUDriver *eicup, EICU_Channel channel);

/** 
 * @brief	EICU Time Base Settings structure definition
 */
typedef struct
{
	uint16_t EICU_Prescaler; /* Specifies the prescaler value used to divide
								the timer clock.  This parameter can be between 
								0x0000 and 0xFFFF */

	uint32_t EICU_Period;	 /* Specifies the period value of the timer. This
								parameter can be between 0x0000 and 0xFFFF */
} EICU_TimeBase_Settings; 

/** 
 * @brief	EICU Input Capture Settings structure definition  
 */
typedef struct
{
	uint16_t EICU_ICPolarity;	/* Specifies the active edge of the input
								   signal. */

	uint16_t EICU_ICSelection;  /* Specifies the input selection. */

	uint16_t EICU_ICPrescaler;  /* Specifies the Input Capture Prescaler. */

	uint16_t EICU_ICFilter;     /* Specifies the input capture filter. This
								   parameter can be a number between
								   0x0 and 0xF */

	eicucallback_t capture_cb;	/* Capture event callback */

	eicucallback_t period_cb;	/* Period capture event callback */

	eicucallback_t overflow_cb;	/* Timer overflow event callback */
} EICU_IC_Settings;

/** 
 * @brief	EICU Input Capture Config structure definition  
 */
typedef struct
{
	bool_t pwm_measurement;				/* PWM Measurement can be
										   TRUE or FALSE */

	uint16_t EICU_InputTriggerSource;	/* Select the input trigger for PWM
										   measurement mode. This parameter can
										   be EICU_TS_TI1FP1 or 
										   EICU_TS_TI2FP2 */

	EICU_TimeBase_Settings tbcfg;		/* TimeBase settings */

	EICU_IC_Settings *iccfgp[4];		/* Pointer to each Input Capture channel
										   configuration. A NULL parameter
										   indicates the channel as unused. 
										   Note: In PWM mode, only Channel 1 OR
										   Channel 2 may be used. */
} EICU_Config;

/** 
 * @brief	EICU Input Capture Driver structure definition  
 */
typedef struct
{
	stm32_tim_t *tim;					/* Timer peripheral for
										   Input Capture. */

	uint32_t clock;						/* Timer base clock. */

	EICU_Config *cfg;					/* Pointer to configuration for the
										   driver. */

	volatile uint32_t *wccrp[4];		/* CCR registers for width capture. */

	volatile uint32_t *pccrp;			/* CCR register for period capture.
										   Only one is needed since only one
										   PWM input per timer is allowed. */
} EICUDriver;


/* Global Variable Defines */
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

/* Global function defines */

#endif
