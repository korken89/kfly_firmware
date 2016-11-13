#ifndef BOARD_H
#define BOARD_H

/*
 * Board identifier.
 */
#define BOARD_KFLY
#define BOARD_NAME                  "KFly"
#define BOARD_MAJOR_VERSION         4
#define BOARD_MINOR_VERSION         0

/*
 * Board oscillators-related settings.
 */
#if !defined(STM32_HSECLK)
#define STM32_HSECLK                25000000U
#endif

#define STM32_HSE_BYPASS
//#define BOARD_OTG_NOVBUSSENS

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   330U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F746xx

/*
 * IO pins assignments.
 */
#define GPIOA_PIN0                  0U
#define GPIOA_PIN1                  1U
#define GPIOA_PIN2                  2U
#define GPIOA_PIN3                  3U
#define GPIOA_PIN4                  4U
#define GPIOA_PIN5                  5U
#define GPIOA_PIN6                  6U
#define GPIOA_PIN7                  7U
#define GPIOA_PIN8                  8U
#define GPIOA_OTG_FS_VBUS           9U
#define GPIOA_PIN10                 10U
#define GPIOA_OTG_FS_DM             11U
#define GPIOA_OTG_FS_DP             12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_PIN15                 15U

#define GPIOB_PIN0                  0U
#define GPIOB_PIN1                  1U
#define GPIOB_PIN2                  2U
#define GPIOB_PIN3                  3U
#define GPIOB_PIN4                  4U
#define GPIOB_PIN5                  5U
#define GPIOB_PIN6                  6U
#define GPIOB_PIN7                  7U
#define GPIOB_PIN8                  8U
#define GPIOB_PIN9                  9U
#define GPIOB_PIN10                 10U
#define GPIOB_PIN11                 11U
#define GPIOB_PIN12                 12U
#define GPIOB_PIN13                 13U
#define GPIOB_PIN14                 14U
#define GPIOB_PIN15                 15U

#define GPIOC_LED1                  0U
#define GPIOC_LED2                  1U
#define GPIOC_LED3                  2U
#define GPIOC_PIN3                  3U
#define GPIOC_PIN4                  4U
#define GPIOC_PIN5                  5U
#define GPIOC_PIN6                  6U
#define GPIOC_PIN7                  7U
#define GPIOC_SD_D0                 8U
#define GPIOC_SD_D1                 9U
#define GPIOC_SD_D2                 10U
#define GPIOC_SD_D3                 11U
#define GPIOC_SD_CLK                12U
#define GPIOC_PIN13                 13U
#define GPIOC_PIN14                 14U
#define GPIOC_PIN15                 15U

#define GPIOD_PIN0                  0U
#define GPIOD_PIN1                  1U
#define GPIOD_SD_CMD                2U
#define GPIOD_PIN3                  3U
#define GPIOD_PIN4                  4U
#define GPIOD_PIN5                  5U
#define GPIOD_PIN6                  6U
#define GPIOD_PIN7                  7U
#define GPIOD_PIN8                  8U
#define GPIOD_PIN9                  9U
#define GPIOD_PIN10                 10U
#define GPIOD_PIN11                 11U
#define GPIOD_PIN12                 12U
#define GPIOD_PIN13                 13U
#define GPIOD_PIN14                 14U
#define GPIOD_PIN15                 15U

#define GPIOE_PIN0                  0U
#define GPIOE_PIN1                  1U
#define GPIOE_PIN2                  2U
#define GPIOE_PIN3                  3U
#define GPIOE_PIN4                  4U
#define GPIOE_PIN5                  5U
#define GPIOE_PIN6                  6U
#define GPIOE_PIN7                  7U
#define GPIOE_PIN8                  8U
#define GPIOE_PIN9                  9U
#define GPIOE_PIN10                 10U
#define GPIOE_PIN11                 11U
#define GPIOE_PIN12                 12U
#define GPIOE_PIN13                 13U
#define GPIOE_PIN14                 14U
#define GPIOE_PIN15                 15U

#define GPIOF_PIN0                  0U
#define GPIOF_PIN1                  1U
#define GPIOF_PIN2                  2U
#define GPIOF_PIN3                  3U
#define GPIOF_PIN4                  4U
#define GPIOF_PIN5                  5U
#define GPIOF_PIN6                  6U
#define GPIOF_PIN7                  7U
#define GPIOF_PIN8                  8U
#define GPIOF_PIN9                  9U
#define GPIOF_PIN10                 10U
#define GPIOF_PIN11                 11U
#define GPIOF_PIN12                 12U
#define GPIOF_PIN13                 13U
#define GPIOF_PIN14                 14U
#define GPIOF_PIN15                 15U

#define GPIOG_PIN0                  0U
#define GPIOG_PIN1                  1U
#define GPIOG_PIN2                  2U
#define GPIOG_PIN3                  3U
#define GPIOG_PIN4                  4U
#define GPIOG_PIN5                  5U
#define GPIOG_PIN6                  6U
#define GPIOG_PIN7                  7U
#define GPIOG_PIN8                  8U
#define GPIOG_PIN9                  9U
#define GPIOG_PIN10                 10U
#define GPIOG_PIN11                 11U
#define GPIOG_PIN12                 12U
#define GPIOG_SD_DETECT             13U
#define GPIOG_PIN14                 14U
#define GPIOG_PIN15                 15U

#define GPIOH_OSC_IN                0U
#define GPIOH_OSC_OUT               1U
#define GPIOH_PIN2                  2U
#define GPIOH_PIN3                  3U
#define GPIOH_PIN4                  4U
#define GPIOH_PIN5                  5U
#define GPIOH_PIN6                  6U
#define GPIOH_PIN7                  7U
#define GPIOH_PIN8                  8U
#define GPIOH_PIN9                  9U
#define GPIOH_PIN10                 10U
#define GPIOH_PIN11                 11U
#define GPIOH_PIN12                 12U
#define GPIOH_PIN13                 13U
#define GPIOH_PIN14                 14U
#define GPIOH_PIN15                 15U

#define GPIOI_PIN0                  0U
#define GPIOI_PIN1                  1U
#define GPIOI_PIN2                  2U
#define GPIOI_PIN3                  3U
#define GPIOI_PIN4                  4U
#define GPIOI_PIN5                  5U
#define GPIOI_PIN6                  6U
#define GPIOI_PIN7                  7U
#define GPIOI_PIN8                  8U
#define GPIOI_PIN9                  9U
#define GPIOI_PIN10                 10U
#define GPIOI_PIN11                 11U
#define GPIOI_PIN12                 12U
#define GPIOI_PIN13                 13U
#define GPIOI_PIN14                 14U
#define GPIOI_PIN15                 15U

#define GPIOJ_PIN0                  0U
#define GPIOJ_PIN1                  1U
#define GPIOJ_PIN2                  2U
#define GPIOJ_PIN3                  3U
#define GPIOJ_PIN4                  4U
#define GPIOJ_PIN5                  5U
#define GPIOJ_PIN6                  6U
#define GPIOJ_PIN7                  7U
#define GPIOJ_PIN8                  8U
#define GPIOJ_PIN9                  9U
#define GPIOJ_PIN10                 10U
#define GPIOJ_PIN11                 11U
#define GPIOJ_PIN12                 12U
#define GPIOJ_PIN13                 13U
#define GPIOJ_PIN14                 14U
#define GPIOJ_PIN15                 15U

#define GPIOK_PIN0                  0U
#define GPIOK_PIN1                  1U
#define GPIOK_PIN2                  2U
#define GPIOK_PIN3                  3U
#define GPIOK_PIN4                  4U
#define GPIOK_PIN5                  5U
#define GPIOK_PIN6                  6U
#define GPIOK_PIN7                  7U
#define GPIOK_PIN8                  8U
#define GPIOK_PIN9                  9U
#define GPIOK_PIN10                 10U
#define GPIOK_PIN11                 11U
#define GPIOK_PIN12                 12U
#define GPIOK_PIN13                 13U
#define GPIOK_PIN14                 14U
#define GPIOK_PIN15                 15U

/*
 * IO lines assignments.
 */
#define LINE_OTG_FS_VBUS            PAL_LINE(GPIOA, 9U)
#define LINE_OTG_FS_DM              PAL_LINE(GPIOA, 11U)
#define LINE_OTG_FS_DP              PAL_LINE(GPIOA, 12U)

#define LINE_SWDIO                  PAL_LINE(GPIOA, 13U)
#define LINE_SWCLK                  PAL_LINE(GPIOA, 14U)

#define LINE_LED1                   PAL_LINE(GPIOC, 0U)
#define LINE_LED2                   PAL_LINE(GPIOC, 1U)
#define LINE_LED3                   PAL_LINE(GPIOC, 2U)

#define LINE_SD_D0                  PAL_LINE(GPIOC, 8U)
#define LINE_SD_D1                  PAL_LINE(GPIOC, 9U)
#define LINE_SD_D2                  PAL_LINE(GPIOC, 10U)
#define LINE_SD_D3                  PAL_LINE(GPIOC, 11U)
#define LINE_SD_CLK                 PAL_LINE(GPIOC, 12U)
#define LINE_SD_CMD                 PAL_LINE(GPIOD, 2U)
#define LINE_SD_DETECT              PAL_LINE(GPIOG, 13U)

#define LINE_OSC_IN                 PAL_LINE(GPIOH, 0U)
#define LINE_OSC_OUT                PAL_LINE(GPIOH, 1U)



/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/*
 * GPIOA setup:
 *
 * PA11 - OTG_FS_DM                 (alternate 10).
 * PA12 - OTG_FS_DP                 (alternate 10).
 * PA13 - SWDIO                     (alternate 0).
 * PA14 - SWCLK                     (alternate 0).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_PIN0) |             \
                                     PIN_MODE_INPUT(GPIOA_PIN1) |             \
                                     PIN_MODE_INPUT(GPIOA_PIN2) |             \
                                     PIN_MODE_INPUT(GPIOA_PIN3) |             \
                                     PIN_MODE_INPUT(GPIOA_PIN4) |             \
                                     PIN_MODE_INPUT(GPIOA_PIN5) |             \
                                     PIN_MODE_INPUT(GPIOA_PIN6) |             \
                                     PIN_MODE_INPUT(GPIOA_PIN7) |             \
                                     PIN_MODE_INPUT(GPIOA_PIN8) |             \
                                     PIN_MODE_INPUT(GPIOA_OTG_FS_VBUS) |      \
                                     PIN_MODE_INPUT(GPIOA_PIN10) |            \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |        \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |        \
                                     PIN_MODE_INPUT(GPIOA_PIN15))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_PIN0) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN1) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN6) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN7) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN8) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_VBUS) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN10) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DM) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DP) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN15))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_HIGH(GPIOA_PIN0) |            \
                                     PIN_OSPEED_HIGH(GPIOA_PIN1) |            \
                                     PIN_OSPEED_HIGH(GPIOA_PIN2) |            \
                                     PIN_OSPEED_HIGH(GPIOA_PIN3) |            \
                                     PIN_OSPEED_HIGH(GPIOA_PIN4) |            \
                                     PIN_OSPEED_HIGH(GPIOA_PIN5) |            \
                                     PIN_OSPEED_HIGH(GPIOA_PIN6) |            \
                                     PIN_OSPEED_HIGH(GPIOA_PIN7) |            \
                                     PIN_OSPEED_HIGH(GPIOA_PIN8) |            \
                                     PIN_OSPEED_HIGH(GPIOA_OTG_FS_VBUS) |     \
                                     PIN_OSPEED_HIGH(GPIOA_PIN10) |           \
                                     PIN_OSPEED_HIGH(GPIOA_OTG_FS_DM) |       \
                                     PIN_OSPEED_HIGH(GPIOA_OTG_FS_DP) |       \
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO) |           \
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK) |           \
                                     PIN_OSPEED_HIGH(GPIOA_PIN15))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOA_PIN0) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN1) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN2) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN3) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN4) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN5) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN6) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN7) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN8) |         \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_VBUS) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN10) |        \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DM) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DP) |    \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWDIO) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN15))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_PIN0) |               \
                                     PIN_ODR_HIGH(GPIOA_PIN1) |               \
                                     PIN_ODR_HIGH(GPIOA_PIN2) |               \
                                     PIN_ODR_HIGH(GPIOA_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOA_PIN4) |               \
                                     PIN_ODR_HIGH(GPIOA_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOA_PIN6) |               \
                                     PIN_ODR_HIGH(GPIOA_PIN7) |               \
                                     PIN_ODR_HIGH(GPIOA_PIN8) |               \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_VBUS) |        \
                                     PIN_ODR_HIGH(GPIOA_PIN10) |              \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DM) |          \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DP) |          \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) |              \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) |              \
                                     PIN_ODR_HIGH(GPIOA_PIN15))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_PIN0, 0U) |            \
                                     PIN_AFIO_AF(GPIOA_PIN1, 0U) |            \
                                     PIN_AFIO_AF(GPIOA_PIN2, 0U) |            \
                                     PIN_AFIO_AF(GPIOA_PIN3, 0U) |            \
                                     PIN_AFIO_AF(GPIOA_PIN4, 0U) |            \
                                     PIN_AFIO_AF(GPIOA_PIN5, 0U) |            \
                                     PIN_AFIO_AF(GPIOA_PIN6, 0U) |            \
                                     PIN_AFIO_AF(GPIOA_PIN7, 0U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_PIN8, 0U) |            \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_VBUS, 0U) |     \
                                     PIN_AFIO_AF(GPIOA_PIN10, 0U) |           \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10U) |      \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10U) |      \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0U) |           \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U) |           \
                                     PIN_AFIO_AF(GPIOA_PIN15, 0U))

/*
 * GPIOB setup:
 *
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(GPIOB_PIN0) |             \
                                     PIN_MODE_INPUT(GPIOB_PIN1) |             \
                                     PIN_MODE_INPUT(GPIOB_PIN2) |             \
                                     PIN_MODE_INPUT(GPIOB_PIN3) |             \
                                     PIN_MODE_INPUT(GPIOB_PIN4) |             \
                                     PIN_MODE_INPUT(GPIOB_PIN5) |             \
                                     PIN_MODE_INPUT(GPIOB_PIN6) |             \
                                     PIN_MODE_INPUT(GPIOB_PIN7) |             \
                                     PIN_MODE_INPUT(GPIOB_PIN8) |             \
                                     PIN_MODE_INPUT(GPIOB_PIN9) |             \
                                     PIN_MODE_INPUT(GPIOB_PIN10) |            \
                                     PIN_MODE_INPUT(GPIOB_PIN11) |            \
                                     PIN_MODE_INPUT(GPIOB_PIN12) |            \
                                     PIN_MODE_INPUT(GPIOB_PIN13) |            \
                                     PIN_MODE_INPUT(GPIOB_PIN14) |            \
                                     PIN_MODE_INPUT(GPIOB_PIN15))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_PIN0) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN1) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN6) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN7) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN8) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN9) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN10) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN11) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN12) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN13) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN14) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN15))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_PIN0) |            \
                                     PIN_OSPEED_HIGH(GPIOB_PIN1) |            \
                                     PIN_OSPEED_HIGH(GPIOB_PIN2) |            \
                                     PIN_OSPEED_HIGH(GPIOB_PIN3) |            \
                                     PIN_OSPEED_HIGH(GPIOB_PIN4) |            \
                                     PIN_OSPEED_HIGH(GPIOB_PIN5) |            \
                                     PIN_OSPEED_HIGH(GPIOB_PIN6) |            \
                                     PIN_OSPEED_HIGH(GPIOB_PIN7) |            \
                                     PIN_OSPEED_HIGH(GPIOB_PIN8) |            \
                                     PIN_OSPEED_HIGH(GPIOB_PIN9) |            \
                                     PIN_OSPEED_HIGH(GPIOB_PIN10) |           \
                                     PIN_OSPEED_HIGH(GPIOB_PIN11) |           \
                                     PIN_OSPEED_HIGH(GPIOB_PIN12) |           \
                                     PIN_OSPEED_HIGH(GPIOB_PIN13) |           \
                                     PIN_OSPEED_HIGH(GPIOB_PIN14) |           \
                                     PIN_OSPEED_HIGH(GPIOB_PIN15))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOB_PIN0) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN1) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN2) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN3) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN4) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN5) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN6) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN7) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN8) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN9) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN10) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN11) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN12) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN13) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN14) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOB_PIN15))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_PIN0) |               \
                                     PIN_ODR_HIGH(GPIOB_PIN1) |               \
                                     PIN_ODR_HIGH(GPIOB_PIN2) |               \
                                     PIN_ODR_HIGH(GPIOB_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOB_PIN4) |               \
                                     PIN_ODR_HIGH(GPIOB_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOB_PIN6) |               \
                                     PIN_ODR_HIGH(GPIOB_PIN7) |               \
                                     PIN_ODR_HIGH(GPIOB_PIN8) |               \
                                     PIN_ODR_HIGH(GPIOB_PIN9) |               \
                                     PIN_ODR_HIGH(GPIOB_PIN10) |              \
                                     PIN_ODR_HIGH(GPIOB_PIN11) |              \
                                     PIN_ODR_HIGH(GPIOB_PIN12) |              \
                                     PIN_ODR_HIGH(GPIOB_PIN13) |              \
                                     PIN_ODR_HIGH(GPIOB_PIN14) |              \
                                     PIN_ODR_HIGH(GPIOB_PIN15))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_PIN0, 0U) |            \
                                     PIN_AFIO_AF(GPIOB_PIN1, 0U) |            \
                                     PIN_AFIO_AF(GPIOB_PIN2, 0U) |            \
                                     PIN_AFIO_AF(GPIOB_PIN3, 0U) |            \
                                     PIN_AFIO_AF(GPIOB_PIN4, 0U) |            \
                                     PIN_AFIO_AF(GPIOB_PIN5, 0U) |            \
                                     PIN_AFIO_AF(GPIOB_PIN6, 0U) |            \
                                     PIN_AFIO_AF(GPIOB_PIN7, 0U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_PIN8, 0U) |            \
                                     PIN_AFIO_AF(GPIOB_PIN9, 0U) |            \
                                     PIN_AFIO_AF(GPIOB_PIN10, 0U) |           \
                                     PIN_AFIO_AF(GPIOB_PIN11, 0U) |           \
                                     PIN_AFIO_AF(GPIOB_PIN12, 0U) |           \
                                     PIN_AFIO_AF(GPIOB_PIN13, 0U) |           \
                                     PIN_AFIO_AF(GPIOB_PIN14, 0U) |           \
                                     PIN_AFIO_AF(GPIOB_PIN15, 0U))

/*
 * GPIOC setup:
 *
 * PC0  - LED1
 * PC1  - LED2
 * PC2  - LED3
 *
 * PC8  - SD_D0                     (alternate 12).
 * PC9  - SD_D1                     (alternate 12).
 * PC10 - SD_D2                     (alternate 12).
 * PC11 - SD_D3                     (alternate 12).
 * PC12 - SD_CLK                    (alternate 12).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_OUTPUT(GPIOC_LED1) |            \
                                     PIN_MODE_OUTPUT(GPIOC_LED2) |            \
                                     PIN_MODE_OUTPUT(GPIOC_LED3) |            \
                                     PIN_MODE_INPUT(GPIOC_PIN3) |             \
                                     PIN_MODE_INPUT(GPIOC_PIN4) |             \
                                     PIN_MODE_INPUT(GPIOC_PIN5) |             \
                                     PIN_MODE_INPUT(GPIOC_PIN6) |             \
                                     PIN_MODE_INPUT(GPIOC_PIN7) |             \
                                     PIN_MODE_ALTERNATE(GPIOC_SD_D0) |        \
                                     PIN_MODE_ALTERNATE(GPIOC_SD_D1) |        \
                                     PIN_MODE_ALTERNATE(GPIOC_SD_D2) |        \
                                     PIN_MODE_ALTERNATE(GPIOC_SD_D3) |        \
                                     PIN_MODE_ALTERNATE(GPIOC_SD_CLK) |       \
                                     PIN_MODE_INPUT(GPIOC_PIN13) |            \
                                     PIN_MODE_INPUT(GPIOC_PIN14) |            \
                                     PIN_MODE_INPUT(GPIOC_PIN15))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_LED1) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_LED2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_LED3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN6) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN7) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_D0) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_D1) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_D2) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_D3) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_CLK) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN13) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN14) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN15))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOC_LED1) |         \
                                     PIN_OSPEED_VERYLOW(GPIOC_LED2) |         \
                                     PIN_OSPEED_VERYLOW(GPIOC_LED3) |         \
                                     PIN_OSPEED_HIGH(GPIOC_PIN3) |            \
                                     PIN_OSPEED_HIGH(GPIOC_PIN4) |            \
                                     PIN_OSPEED_HIGH(GPIOC_PIN5) |            \
                                     PIN_OSPEED_HIGH(GPIOC_PIN6) |            \
                                     PIN_OSPEED_HIGH(GPIOC_PIN7) |            \
                                     PIN_OSPEED_HIGH(GPIOC_SD_D0) |           \
                                     PIN_OSPEED_HIGH(GPIOC_SD_D1) |           \
                                     PIN_OSPEED_HIGH(GPIOC_SD_D2) |           \
                                     PIN_OSPEED_HIGH(GPIOC_SD_D3) |           \
                                     PIN_OSPEED_HIGH(GPIOC_SD_CLK) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN13) |           \
                                     PIN_OSPEED_HIGH(GPIOC_PIN14) |           \
                                     PIN_OSPEED_HIGH(GPIOC_PIN15))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOC_LED1) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOC_LED2) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOC_LED3) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN3) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN4) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN5) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN6) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN7) |         \
                                     PIN_PUPDR_FLOATING(GPIOC_SD_D0) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_SD_D1) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_SD_D2) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_SD_D3) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_SD_CLK) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN13) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN14) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN15))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_LED1) |               \
                                     PIN_ODR_HIGH(GPIOC_LED2) |               \
                                     PIN_ODR_HIGH(GPIOC_LED3) |               \
                                     PIN_ODR_HIGH(GPIOC_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOC_PIN4) |               \
                                     PIN_ODR_HIGH(GPIOC_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOC_PIN6) |               \
                                     PIN_ODR_HIGH(GPIOC_PIN7) |               \
                                     PIN_ODR_HIGH(GPIOC_SD_D0) |              \
                                     PIN_ODR_HIGH(GPIOC_SD_D1) |              \
                                     PIN_ODR_HIGH(GPIOC_SD_D2) |              \
                                     PIN_ODR_HIGH(GPIOC_SD_D3) |              \
                                     PIN_ODR_HIGH(GPIOC_SD_CLK) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN13) |              \
                                     PIN_ODR_HIGH(GPIOC_PIN14) |              \
                                     PIN_ODR_HIGH(GPIOC_PIN15))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_LED1, 0U) |            \
                                     PIN_AFIO_AF(GPIOC_LED2, 0U) |            \
                                     PIN_AFIO_AF(GPIOC_LED3, 0U) |            \
                                     PIN_AFIO_AF(GPIOC_PIN3, 0U) |            \
                                     PIN_AFIO_AF(GPIOC_PIN4, 0U) |            \
                                     PIN_AFIO_AF(GPIOC_PIN5, 0U) |            \
                                     PIN_AFIO_AF(GPIOC_PIN6, 0U) |            \
                                     PIN_AFIO_AF(GPIOC_PIN7, 0U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_SD_D0, 12U) |          \
                                     PIN_AFIO_AF(GPIOC_SD_D1, 12U) |          \
                                     PIN_AFIO_AF(GPIOC_SD_D2, 12U) |          \
                                     PIN_AFIO_AF(GPIOC_SD_D3, 12U) |          \
                                     PIN_AFIO_AF(GPIOC_SD_CLK, 12U) |         \
                                     PIN_AFIO_AF(GPIOC_PIN13, 0U) |           \
                                     PIN_AFIO_AF(GPIOC_PIN14, 0U) |           \
                                     PIN_AFIO_AF(GPIOC_PIN15, 0U))


/*
 * GPIOD setup:
 *
 * PD2  - SD_CMD                    (alternate 12).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(GPIOD_PIN0) |             \
                                     PIN_MODE_INPUT(GPIOD_PIN1) |             \
                                     PIN_MODE_ALTERNATE(GPIOD_SD_CMD) |       \
                                     PIN_MODE_INPUT(GPIOD_PIN3) |             \
                                     PIN_MODE_INPUT(GPIOD_PIN4) |             \
                                     PIN_MODE_INPUT(GPIOD_PIN5) |             \
                                     PIN_MODE_INPUT(GPIOD_PIN6) |             \
                                     PIN_MODE_INPUT(GPIOD_PIN7) |             \
                                     PIN_MODE_INPUT(GPIOD_PIN8) |             \
                                     PIN_MODE_INPUT(GPIOD_PIN9) |             \
                                     PIN_MODE_INPUT(GPIOD_PIN10) |            \
                                     PIN_MODE_INPUT(GPIOD_PIN11) |            \
                                     PIN_MODE_INPUT(GPIOD_PIN12) |            \
                                     PIN_MODE_INPUT(GPIOD_PIN13) |            \
                                     PIN_MODE_INPUT(GPIOD_PIN14) |            \
                                     PIN_MODE_INPUT(GPIOD_PIN15))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_PIN0) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN1) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SD_CMD) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN6) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN8) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN9) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN10) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN11) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN12) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN13) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN14) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIOD_PIN0) |            \
                                     PIN_OSPEED_HIGH(GPIOD_PIN1) |            \
                                     PIN_OSPEED_HIGH(GPIOD_SD_CMD) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN3) |            \
                                     PIN_OSPEED_HIGH(GPIOD_PIN4) |            \
                                     PIN_OSPEED_HIGH(GPIOD_PIN5) |            \
                                     PIN_OSPEED_HIGH(GPIOD_PIN6) |            \
                                     PIN_OSPEED_HIGH(GPIOD_PIN7) |            \
                                     PIN_OSPEED_HIGH(GPIOD_PIN8) |            \
                                     PIN_OSPEED_HIGH(GPIOD_PIN9) |            \
                                     PIN_OSPEED_HIGH(GPIOD_PIN10) |           \
                                     PIN_OSPEED_HIGH(GPIOD_PIN11) |           \
                                     PIN_OSPEED_HIGH(GPIOD_PIN12) |           \
                                     PIN_OSPEED_HIGH(GPIOD_PIN13) |           \
                                     PIN_OSPEED_HIGH(GPIOD_PIN14) |           \
                                     PIN_OSPEED_HIGH(GPIOD_PIN15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOD_PIN0) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN1) |         \
                                     PIN_PUPDR_FLOATING(GPIOD_SD_CMD) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN3) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN4) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN5) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN6) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN7) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN8) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN9) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN10) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN11) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN12) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN13) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN14) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN15))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_PIN0) |               \
                                     PIN_ODR_HIGH(GPIOD_PIN1) |               \
                                     PIN_ODR_HIGH(GPIOD_SD_CMD) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOD_PIN4) |               \
                                     PIN_ODR_HIGH(GPIOD_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOD_PIN6) |               \
                                     PIN_ODR_HIGH(GPIOD_PIN7) |               \
                                     PIN_ODR_HIGH(GPIOD_PIN8) |               \
                                     PIN_ODR_HIGH(GPIOD_PIN9) |               \
                                     PIN_ODR_HIGH(GPIOD_PIN10) |              \
                                     PIN_ODR_HIGH(GPIOD_PIN11) |              \
                                     PIN_ODR_HIGH(GPIOD_PIN12) |              \
                                     PIN_ODR_HIGH(GPIOD_PIN13) |              \
                                     PIN_ODR_HIGH(GPIOD_PIN14) |              \
                                     PIN_ODR_HIGH(GPIOD_PIN15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_PIN0, 0U) |            \
                                     PIN_AFIO_AF(GPIOD_PIN1, 0U) |            \
                                     PIN_AFIO_AF(GPIOD_SD_CMD, 12U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN3, 0U) |            \
                                     PIN_AFIO_AF(GPIOD_PIN4, 0U) |            \
                                     PIN_AFIO_AF(GPIOD_PIN5, 0U) |            \
                                     PIN_AFIO_AF(GPIOD_PIN6, 0U) |            \
                                     PIN_AFIO_AF(GPIOD_PIN7, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8, 0U) |            \
                                     PIN_AFIO_AF(GPIOD_PIN9, 0U) |            \
                                     PIN_AFIO_AF(GPIOD_PIN10, 0U) |           \
                                     PIN_AFIO_AF(GPIOD_PIN11, 0U) |           \
                                     PIN_AFIO_AF(GPIOD_PIN12, 0U) |           \
                                     PIN_AFIO_AF(GPIOD_PIN13, 0U) |           \
                                     PIN_AFIO_AF(GPIOD_PIN14, 0U) |           \
                                     PIN_AFIO_AF(GPIOD_PIN15, 0U))


/*
 * GPIOE setup:
 *
 */
#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(GPIOE_PIN0) |             \
                                     PIN_MODE_INPUT(GPIOE_PIN1) |             \
                                     PIN_MODE_INPUT(GPIOE_PIN2) |             \
                                     PIN_MODE_INPUT(GPIOE_PIN3) |             \
                                     PIN_MODE_INPUT(GPIOE_PIN4) |             \
                                     PIN_MODE_INPUT(GPIOE_PIN5) |             \
                                     PIN_MODE_INPUT(GPIOE_PIN6) |             \
                                     PIN_MODE_INPUT(GPIOE_PIN7) |             \
                                     PIN_MODE_INPUT(GPIOE_PIN8) |             \
                                     PIN_MODE_INPUT(GPIOE_PIN9) |             \
                                     PIN_MODE_INPUT(GPIOE_PIN10) |            \
                                     PIN_MODE_INPUT(GPIOE_PIN11) |            \
                                     PIN_MODE_INPUT(GPIOE_PIN12) |            \
                                     PIN_MODE_INPUT(GPIOE_PIN13) |            \
                                     PIN_MODE_INPUT(GPIOE_PIN14) |            \
                                     PIN_MODE_INPUT(GPIOE_PIN15))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_PIN0) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN1) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN6) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN7) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN8) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN9) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN10) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN11) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN12) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN13) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN14) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN15))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_HIGH(GPIOE_PIN0) |            \
                                     PIN_OSPEED_HIGH(GPIOE_PIN1) |            \
                                     PIN_OSPEED_HIGH(GPIOE_PIN2) |            \
                                     PIN_OSPEED_HIGH(GPIOE_PIN3) |            \
                                     PIN_OSPEED_HIGH(GPIOE_PIN4) |            \
                                     PIN_OSPEED_HIGH(GPIOE_PIN5) |            \
                                     PIN_OSPEED_HIGH(GPIOE_PIN6) |            \
                                     PIN_OSPEED_HIGH(GPIOE_PIN7) |            \
                                     PIN_OSPEED_HIGH(GPIOE_PIN8) |            \
                                     PIN_OSPEED_HIGH(GPIOE_PIN9) |            \
                                     PIN_OSPEED_HIGH(GPIOE_PIN10) |           \
                                     PIN_OSPEED_HIGH(GPIOE_PIN11) |           \
                                     PIN_OSPEED_HIGH(GPIOE_PIN12) |           \
                                     PIN_OSPEED_HIGH(GPIOE_PIN13) |           \
                                     PIN_OSPEED_HIGH(GPIOE_PIN14) |           \
                                     PIN_OSPEED_HIGH(GPIOE_PIN15))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOE_PIN0) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN1) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN2) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN3) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN4) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN5) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN6) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN7) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN8) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN9) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN10) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN11) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN12) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN13) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN14) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN15))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_PIN0) |               \
                                     PIN_ODR_HIGH(GPIOE_PIN1) |               \
                                     PIN_ODR_HIGH(GPIOE_PIN2) |               \
                                     PIN_ODR_HIGH(GPIOE_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOE_PIN4) |               \
                                     PIN_ODR_HIGH(GPIOE_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOE_PIN6) |               \
                                     PIN_ODR_HIGH(GPIOE_PIN7) |               \
                                     PIN_ODR_HIGH(GPIOE_PIN8) |               \
                                     PIN_ODR_HIGH(GPIOE_PIN9) |               \
                                     PIN_ODR_HIGH(GPIOE_PIN10) |              \
                                     PIN_ODR_HIGH(GPIOE_PIN11) |              \
                                     PIN_ODR_HIGH(GPIOE_PIN12) |              \
                                     PIN_ODR_HIGH(GPIOE_PIN13) |              \
                                     PIN_ODR_HIGH(GPIOE_PIN14) |              \
                                     PIN_ODR_HIGH(GPIOE_PIN15))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_PIN0, 0U) |            \
                                     PIN_AFIO_AF(GPIOE_PIN1, 0U) |            \
                                     PIN_AFIO_AF(GPIOE_PIN2, 0U) |            \
                                     PIN_AFIO_AF(GPIOE_PIN3, 0U) |            \
                                     PIN_AFIO_AF(GPIOE_PIN4, 0U) |            \
                                     PIN_AFIO_AF(GPIOE_PIN5, 0U) |            \
                                     PIN_AFIO_AF(GPIOE_PIN6, 0U) |            \
                                     PIN_AFIO_AF(GPIOE_PIN7, 0U))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_PIN8, 0U) |            \
                                     PIN_AFIO_AF(GPIOE_PIN9, 0U) |            \
                                     PIN_AFIO_AF(GPIOE_PIN10, 0U) |           \
                                     PIN_AFIO_AF(GPIOE_PIN11, 0U) |           \
                                     PIN_AFIO_AF(GPIOE_PIN12, 0U) |           \
                                     PIN_AFIO_AF(GPIOE_PIN13, 0U) |           \
                                     PIN_AFIO_AF(GPIOE_PIN14, 0U) |           \
                                     PIN_AFIO_AF(GPIOE_PIN15, 0U))


/*
 * GPIOF setup:
 *
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_PIN0) |             \
                                     PIN_MODE_INPUT(GPIOF_PIN1) |             \
                                     PIN_MODE_INPUT(GPIOF_PIN2) |             \
                                     PIN_MODE_INPUT(GPIOF_PIN3) |             \
                                     PIN_MODE_INPUT(GPIOF_PIN4) |             \
                                     PIN_MODE_INPUT(GPIOF_PIN5) |             \
                                     PIN_MODE_INPUT(GPIOF_PIN6) |             \
                                     PIN_MODE_INPUT(GPIOF_PIN7) |             \
                                     PIN_MODE_INPUT(GPIOF_PIN8) |             \
                                     PIN_MODE_INPUT(GPIOF_PIN9) |             \
                                     PIN_MODE_INPUT(GPIOF_PIN10) |            \
                                     PIN_MODE_INPUT(GPIOF_PIN11) |            \
                                     PIN_MODE_INPUT(GPIOF_PIN12) |            \
                                     PIN_MODE_INPUT(GPIOF_PIN13) |            \
                                     PIN_MODE_INPUT(GPIOF_PIN14) |            \
                                     PIN_MODE_INPUT(GPIOF_PIN15))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_PIN0) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN1) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN6) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN7) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN8) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN9) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN10) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN11) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN12) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN13) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN14) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN15))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_HIGH(GPIOF_PIN0) |            \
                                     PIN_OSPEED_HIGH(GPIOF_PIN1) |            \
                                     PIN_OSPEED_HIGH(GPIOF_PIN2) |            \
                                     PIN_OSPEED_HIGH(GPIOF_PIN3) |            \
                                     PIN_OSPEED_HIGH(GPIOF_PIN4) |            \
                                     PIN_OSPEED_HIGH(GPIOF_PIN5) |            \
                                     PIN_OSPEED_HIGH(GPIOF_PIN6) |            \
                                     PIN_OSPEED_HIGH(GPIOF_PIN7) |            \
                                     PIN_OSPEED_HIGH(GPIOF_PIN8) |            \
                                     PIN_OSPEED_HIGH(GPIOF_PIN9) |            \
                                     PIN_OSPEED_HIGH(GPIOF_PIN10) |           \
                                     PIN_OSPEED_HIGH(GPIOF_PIN11) |           \
                                     PIN_OSPEED_HIGH(GPIOF_PIN12) |           \
                                     PIN_OSPEED_HIGH(GPIOF_PIN13) |           \
                                     PIN_OSPEED_HIGH(GPIOF_PIN14) |           \
                                     PIN_OSPEED_HIGH(GPIOF_PIN15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOF_PIN0) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOF_PIN1) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOF_PIN2) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOF_PIN3) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOF_PIN4) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOF_PIN5) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOF_PIN6) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOF_PIN7) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOF_PIN8) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOF_PIN9) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOF_PIN10) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOF_PIN11) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOF_PIN12) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOF_PIN13) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOF_PIN14) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOF_PIN15))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_PIN0) |               \
                                     PIN_ODR_HIGH(GPIOF_PIN1) |               \
                                     PIN_ODR_HIGH(GPIOF_PIN2) |               \
                                     PIN_ODR_HIGH(GPIOF_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOF_PIN4) |               \
                                     PIN_ODR_HIGH(GPIOF_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOF_PIN6) |               \
                                     PIN_ODR_HIGH(GPIOF_PIN7) |               \
                                     PIN_ODR_HIGH(GPIOF_PIN8) |               \
                                     PIN_ODR_HIGH(GPIOF_PIN9) |               \
                                     PIN_ODR_HIGH(GPIOF_PIN10) |              \
                                     PIN_ODR_HIGH(GPIOF_PIN11) |              \
                                     PIN_ODR_HIGH(GPIOF_PIN12) |              \
                                     PIN_ODR_HIGH(GPIOF_PIN13) |              \
                                     PIN_ODR_HIGH(GPIOF_PIN14) |              \
                                     PIN_ODR_HIGH(GPIOF_PIN15))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_PIN0, 0U) |            \
                                     PIN_AFIO_AF(GPIOF_PIN1, 0U) |            \
                                     PIN_AFIO_AF(GPIOF_PIN2, 0U) |            \
                                     PIN_AFIO_AF(GPIOF_PIN3, 0U) |            \
                                     PIN_AFIO_AF(GPIOF_PIN4, 0U) |            \
                                     PIN_AFIO_AF(GPIOF_PIN5, 0U) |            \
                                     PIN_AFIO_AF(GPIOF_PIN6, 0U) |            \
                                     PIN_AFIO_AF(GPIOF_PIN7, 0U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8, 0U) |            \
                                     PIN_AFIO_AF(GPIOF_PIN9, 0U) |            \
                                     PIN_AFIO_AF(GPIOF_PIN10, 0U) |           \
                                     PIN_AFIO_AF(GPIOF_PIN11, 0U) |           \
                                     PIN_AFIO_AF(GPIOF_PIN12, 0U) |           \
                                     PIN_AFIO_AF(GPIOF_PIN13, 0U) |           \
                                     PIN_AFIO_AF(GPIOF_PIN14, 0U) |           \
                                     PIN_AFIO_AF(GPIOF_PIN15, 0U))


/*
 * GPIOG setup:
 *
 */
#define VAL_GPIOG_MODER             (PIN_MODE_INPUT(GPIOG_PIN0) |             \
                                     PIN_MODE_INPUT(GPIOG_PIN1) |             \
                                     PIN_MODE_INPUT(GPIOG_PIN2) |             \
                                     PIN_MODE_INPUT(GPIOG_PIN3) |             \
                                     PIN_MODE_INPUT(GPIOG_PIN4) |             \
                                     PIN_MODE_INPUT(GPIOG_PIN5) |             \
                                     PIN_MODE_INPUT(GPIOG_PIN6) |             \
                                     PIN_MODE_INPUT(GPIOG_PIN7) |             \
                                     PIN_MODE_INPUT(GPIOG_PIN8) |             \
                                     PIN_MODE_INPUT(GPIOG_PIN9) |             \
                                     PIN_MODE_INPUT(GPIOG_PIN10) |            \
                                     PIN_MODE_INPUT(GPIOG_PIN11) |            \
                                     PIN_MODE_INPUT(GPIOG_PIN12) |            \
                                     PIN_MODE_INPUT(GPIOG_SD_DETECT) |        \
                                     PIN_MODE_INPUT(GPIOG_PIN14) |            \
                                     PIN_MODE_INPUT(GPIOG_PIN15))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_PIN0) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN1) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN6) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN7) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN8) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN9) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN10) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN11) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN12) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOG_SD_DETECT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN14) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN15))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_HIGH(GPIOG_PIN0) |            \
                                     PIN_OSPEED_HIGH(GPIOG_PIN1) |            \
                                     PIN_OSPEED_HIGH(GPIOG_PIN2) |            \
                                     PIN_OSPEED_HIGH(GPIOG_PIN3) |            \
                                     PIN_OSPEED_HIGH(GPIOG_PIN4) |            \
                                     PIN_OSPEED_HIGH(GPIOG_PIN5) |            \
                                     PIN_OSPEED_HIGH(GPIOG_PIN6) |            \
                                     PIN_OSPEED_HIGH(GPIOG_PIN7) |            \
                                     PIN_OSPEED_HIGH(GPIOG_PIN8) |            \
                                     PIN_OSPEED_HIGH(GPIOG_PIN9) |            \
                                     PIN_OSPEED_HIGH(GPIOG_PIN10) |           \
                                     PIN_OSPEED_HIGH(GPIOG_PIN11) |           \
                                     PIN_OSPEED_HIGH(GPIOG_PIN12) |           \
                                     PIN_OSPEED_HIGH(GPIOG_SD_DETECT) |       \
                                     PIN_OSPEED_HIGH(GPIOG_PIN14) |           \
                                     PIN_OSPEED_HIGH(GPIOG_PIN15))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOG_PIN0) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOG_PIN1) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOG_PIN2) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOG_PIN3) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOG_PIN4) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOG_PIN5) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOG_PIN6) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOG_PIN7) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOG_PIN8) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOG_PIN9) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOG_PIN10) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOG_PIN11) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOG_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_SD_DETECT) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOG_PIN14) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOG_PIN15))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_PIN0) |               \
                                     PIN_ODR_HIGH(GPIOG_PIN1) |               \
                                     PIN_ODR_HIGH(GPIOG_PIN2) |               \
                                     PIN_ODR_HIGH(GPIOG_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOG_PIN4) |               \
                                     PIN_ODR_HIGH(GPIOG_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOG_PIN6) |               \
                                     PIN_ODR_HIGH(GPIOG_PIN7) |               \
                                     PIN_ODR_HIGH(GPIOG_PIN8) |               \
                                     PIN_ODR_HIGH(GPIOG_PIN9) |               \
                                     PIN_ODR_HIGH(GPIOG_PIN10) |              \
                                     PIN_ODR_HIGH(GPIOG_PIN11) |              \
                                     PIN_ODR_HIGH(GPIOG_PIN12) |              \
                                     PIN_ODR_HIGH(GPIOG_SD_DETECT) |          \
                                     PIN_ODR_HIGH(GPIOG_PIN14) |              \
                                     PIN_ODR_HIGH(GPIOG_PIN15))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_PIN0, 0U) |            \
                                     PIN_AFIO_AF(GPIOG_PIN1, 0U) |            \
                                     PIN_AFIO_AF(GPIOG_PIN2, 0U) |            \
                                     PIN_AFIO_AF(GPIOG_PIN3, 0U) |            \
                                     PIN_AFIO_AF(GPIOG_PIN4, 0U) |            \
                                     PIN_AFIO_AF(GPIOG_PIN5, 0U) |            \
                                     PIN_AFIO_AF(GPIOG_PIN6, 0U) |            \
                                     PIN_AFIO_AF(GPIOG_PIN7, 0U))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_PIN8, 0U) |            \
                                     PIN_AFIO_AF(GPIOG_PIN9, 0U) |            \
                                     PIN_AFIO_AF(GPIOG_PIN10, 0U) |           \
                                     PIN_AFIO_AF(GPIOG_PIN11, 0U) |           \
                                     PIN_AFIO_AF(GPIOG_PIN12, 0U) |           \
                                     PIN_AFIO_AF(GPIOG_SD_DETECT, 0U) |       \
                                     PIN_AFIO_AF(GPIOG_PIN14, 0U) |           \
                                     PIN_AFIO_AF(GPIOG_PIN15, 0U))


/*
 * GPIOH setup:
 *
 * PH0  - OSC_IN                    (input floating).
 * PH1  - OSC_OUT                   (input floating).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |           \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN2) |             \
                                     PIN_MODE_INPUT(GPIOH_PIN3) |             \
                                     PIN_MODE_INPUT(GPIOH_PIN4) |             \
                                     PIN_MODE_INPUT(GPIOH_PIN5) |             \
                                     PIN_MODE_INPUT(GPIOH_PIN6) |             \
                                     PIN_MODE_INPUT(GPIOH_PIN7) |             \
                                     PIN_MODE_INPUT(GPIOH_PIN8) |             \
                                     PIN_MODE_INPUT(GPIOH_PIN9) |             \
                                     PIN_MODE_INPUT(GPIOH_PIN10) |            \
                                     PIN_MODE_INPUT(GPIOH_PIN11) |            \
                                     PIN_MODE_INPUT(GPIOH_PIN12) |            \
                                     PIN_MODE_INPUT(GPIOH_PIN13) |            \
                                     PIN_MODE_INPUT(GPIOH_PIN14) |            \
                                     PIN_MODE_INPUT(GPIOH_PIN15))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN6) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN7) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN8) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN9) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN10) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN11) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN12) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN13) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN14) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN15))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_HIGH(GPIOH_OSC_IN) |          \
                                     PIN_OSPEED_HIGH(GPIOH_OSC_OUT) |         \
                                     PIN_OSPEED_HIGH(GPIOH_PIN2) |            \
                                     PIN_OSPEED_HIGH(GPIOH_PIN3) |            \
                                     PIN_OSPEED_HIGH(GPIOH_PIN4) |            \
                                     PIN_OSPEED_HIGH(GPIOH_PIN5) |            \
                                     PIN_OSPEED_HIGH(GPIOH_PIN6) |            \
                                     PIN_OSPEED_HIGH(GPIOH_PIN7) |            \
                                     PIN_OSPEED_HIGH(GPIOH_PIN8) |            \
                                     PIN_OSPEED_HIGH(GPIOH_PIN9) |            \
                                     PIN_OSPEED_HIGH(GPIOH_PIN10) |           \
                                     PIN_OSPEED_HIGH(GPIOH_PIN11) |           \
                                     PIN_OSPEED_HIGH(GPIOH_PIN12) |           \
                                     PIN_OSPEED_HIGH(GPIOH_PIN13) |           \
                                     PIN_OSPEED_HIGH(GPIOH_PIN14) |           \
                                     PIN_OSPEED_HIGH(GPIOH_PIN15))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN2) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN3) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN4) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN5) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN6) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN7) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN8) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN9) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN10) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN11) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN12) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN13) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN14) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN15))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) |             \
                                     PIN_ODR_HIGH(GPIOH_OSC_OUT) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN2) |               \
                                     PIN_ODR_HIGH(GPIOH_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOH_PIN4) |               \
                                     PIN_ODR_HIGH(GPIOH_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOH_PIN6) |               \
                                     PIN_ODR_HIGH(GPIOH_PIN7) |               \
                                     PIN_ODR_HIGH(GPIOH_PIN8) |               \
                                     PIN_ODR_HIGH(GPIOH_PIN9) |               \
                                     PIN_ODR_HIGH(GPIOH_PIN10) |              \
                                     PIN_ODR_HIGH(GPIOH_PIN11) |              \
                                     PIN_ODR_HIGH(GPIOH_PIN12) |              \
                                     PIN_ODR_HIGH(GPIOH_PIN13) |              \
                                     PIN_ODR_HIGH(GPIOH_PIN14) |              \
                                     PIN_ODR_HIGH(GPIOH_PIN15))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN2, 0U) |            \
                                     PIN_AFIO_AF(GPIOH_PIN3, 0U) |            \
                                     PIN_AFIO_AF(GPIOH_PIN4, 0U) |            \
                                     PIN_AFIO_AF(GPIOH_PIN5, 0U) |            \
                                     PIN_AFIO_AF(GPIOH_PIN6, 0U) |            \
                                     PIN_AFIO_AF(GPIOH_PIN7, 0U))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_PIN8, 0U) |            \
                                     PIN_AFIO_AF(GPIOH_PIN9, 0U) |            \
                                     PIN_AFIO_AF(GPIOH_PIN10, 0U) |           \
                                     PIN_AFIO_AF(GPIOH_PIN11, 0U) |           \
                                     PIN_AFIO_AF(GPIOH_PIN12, 0U) |           \
                                     PIN_AFIO_AF(GPIOH_PIN13, 0U) |           \
                                     PIN_AFIO_AF(GPIOH_PIN14, 0U) |           \
                                     PIN_AFIO_AF(GPIOH_PIN15, 0U))


/*
 * GPIOI setup:
 *
 */
#define VAL_GPIOI_MODER             (PIN_MODE_INPUT(GPIOI_PIN0) |             \
                                     PIN_MODE_INPUT(GPIOI_PIN1) |             \
                                     PIN_MODE_INPUT(GPIOI_PIN2) |             \
                                     PIN_MODE_INPUT(GPIOI_PIN3) |             \
                                     PIN_MODE_INPUT(GPIOI_PIN4) |             \
                                     PIN_MODE_INPUT(GPIOI_PIN5) |             \
                                     PIN_MODE_INPUT(GPIOI_PIN6) |             \
                                     PIN_MODE_INPUT(GPIOI_PIN7) |             \
                                     PIN_MODE_INPUT(GPIOI_PIN8) |             \
                                     PIN_MODE_INPUT(GPIOI_PIN9) |             \
                                     PIN_MODE_INPUT(GPIOI_PIN10) |            \
                                     PIN_MODE_INPUT(GPIOI_PIN11) |            \
                                     PIN_MODE_INPUT(GPIOI_PIN12) |            \
                                     PIN_MODE_INPUT(GPIOI_PIN13) |            \
                                     PIN_MODE_INPUT(GPIOI_PIN14) |            \
                                     PIN_MODE_INPUT(GPIOI_PIN15))
#define VAL_GPIOI_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOI_PIN0) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN1) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN6) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN7) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN8) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN9) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN10) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN11) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN12) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN13) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN14) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN15))
#define VAL_GPIOI_OSPEEDR           (PIN_OSPEED_HIGH(GPIOI_PIN0) |            \
                                     PIN_OSPEED_HIGH(GPIOI_PIN1) |            \
                                     PIN_OSPEED_HIGH(GPIOI_PIN2) |            \
                                     PIN_OSPEED_HIGH(GPIOI_PIN3) |            \
                                     PIN_OSPEED_HIGH(GPIOI_PIN4) |            \
                                     PIN_OSPEED_HIGH(GPIOI_PIN5) |            \
                                     PIN_OSPEED_HIGH(GPIOI_PIN6) |            \
                                     PIN_OSPEED_HIGH(GPIOI_PIN7) |            \
                                     PIN_OSPEED_HIGH(GPIOI_PIN8) |            \
                                     PIN_OSPEED_HIGH(GPIOI_PIN9) |            \
                                     PIN_OSPEED_HIGH(GPIOI_PIN10) |           \
                                     PIN_OSPEED_HIGH(GPIOI_PIN11) |           \
                                     PIN_OSPEED_HIGH(GPIOI_PIN12) |           \
                                     PIN_OSPEED_HIGH(GPIOI_PIN13) |           \
                                     PIN_OSPEED_HIGH(GPIOI_PIN14) |           \
                                     PIN_OSPEED_HIGH(GPIOI_PIN15))
#define VAL_GPIOI_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOI_PIN0) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOI_PIN1) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOI_PIN2) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOI_PIN3) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOI_PIN4) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOI_PIN5) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOI_PIN6) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOI_PIN7) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOI_PIN8) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOI_PIN9) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOI_PIN10) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOI_PIN11) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOI_PIN12) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOI_PIN13) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOI_PIN14) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOI_PIN15))
#define VAL_GPIOI_ODR               (PIN_ODR_HIGH(GPIOI_PIN0) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN1) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN2) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN4) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN6) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN7) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN8) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN9) |               \
                                     PIN_ODR_HIGH(GPIOI_PIN10) |              \
                                     PIN_ODR_HIGH(GPIOI_PIN11) |              \
                                     PIN_ODR_HIGH(GPIOI_PIN12) |              \
                                     PIN_ODR_HIGH(GPIOI_PIN13) |              \
                                     PIN_ODR_HIGH(GPIOI_PIN14) |              \
                                     PIN_ODR_HIGH(GPIOI_PIN15))
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_PIN0, 0U) |            \
                                     PIN_AFIO_AF(GPIOI_PIN1, 0U) |            \
                                     PIN_AFIO_AF(GPIOI_PIN2, 0U) |            \
                                     PIN_AFIO_AF(GPIOI_PIN3, 0U) |            \
                                     PIN_AFIO_AF(GPIOI_PIN4, 0U) |            \
                                     PIN_AFIO_AF(GPIOI_PIN5, 0U) |            \
                                     PIN_AFIO_AF(GPIOI_PIN6, 0U) |            \
                                     PIN_AFIO_AF(GPIOI_PIN7, 0U))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(GPIOI_PIN8, 0U) |            \
                                     PIN_AFIO_AF(GPIOI_PIN9, 0U) |            \
                                     PIN_AFIO_AF(GPIOI_PIN10, 0U) |           \
                                     PIN_AFIO_AF(GPIOI_PIN11, 0U) |           \
                                     PIN_AFIO_AF(GPIOI_PIN12, 0U) |           \
                                     PIN_AFIO_AF(GPIOI_PIN13, 0U) |           \
                                     PIN_AFIO_AF(GPIOI_PIN14, 0U) |           \
                                     PIN_AFIO_AF(GPIOI_PIN15, 0U))


/*
 * GPIOJ setup:
 *
 */
#define VAL_GPIOJ_MODER             (PIN_MODE_INPUT(GPIOJ_PIN0) |             \
                                     PIN_MODE_INPUT(GPIOJ_PIN1) |             \
                                     PIN_MODE_INPUT(GPIOJ_PIN2) |             \
                                     PIN_MODE_INPUT(GPIOJ_PIN3) |             \
                                     PIN_MODE_INPUT(GPIOJ_PIN4) |             \
                                     PIN_MODE_INPUT(GPIOJ_PIN5) |             \
                                     PIN_MODE_INPUT(GPIOJ_PIN6) |             \
                                     PIN_MODE_INPUT(GPIOJ_PIN7) |             \
                                     PIN_MODE_INPUT(GPIOJ_PIN8) |             \
                                     PIN_MODE_INPUT(GPIOJ_PIN9) |             \
                                     PIN_MODE_INPUT(GPIOJ_PIN10) |            \
                                     PIN_MODE_INPUT(GPIOJ_PIN11) |            \
                                     PIN_MODE_INPUT(GPIOJ_PIN12) |            \
                                     PIN_MODE_INPUT(GPIOJ_PIN13) |            \
                                     PIN_MODE_INPUT(GPIOJ_PIN14) |            \
                                     PIN_MODE_INPUT(GPIOJ_PIN15))
#define VAL_GPIOJ_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOJ_PIN0) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN1) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN6) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN7) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN8) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN9) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN10) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN11) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN12) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN13) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN14) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN15))
#define VAL_GPIOJ_OSPEEDR           (PIN_OSPEED_HIGH(GPIOJ_PIN0) |            \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN1) |            \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN2) |            \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN3) |            \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN4) |            \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN5) |            \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN6) |            \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN7) |            \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN8) |            \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN9) |            \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN10) |           \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN11) |           \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN12) |           \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN13) |           \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN14) |           \
                                     PIN_OSPEED_HIGH(GPIOJ_PIN15))
#define VAL_GPIOJ_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOJ_PIN0) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOJ_PIN1) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOJ_PIN2) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOJ_PIN3) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOJ_PIN4) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOJ_PIN5) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOJ_PIN6) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOJ_PIN7) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOJ_PIN8) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOJ_PIN9) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOJ_PIN10) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOJ_PIN11) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOJ_PIN12) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOJ_PIN13) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOJ_PIN14) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOJ_PIN15))
#define VAL_GPIOJ_ODR               (PIN_ODR_HIGH(GPIOJ_PIN0) |               \
                                     PIN_ODR_HIGH(GPIOJ_PIN1) |               \
                                     PIN_ODR_HIGH(GPIOJ_PIN2) |               \
                                     PIN_ODR_HIGH(GPIOJ_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOJ_PIN4) |               \
                                     PIN_ODR_HIGH(GPIOJ_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOJ_PIN6) |               \
                                     PIN_ODR_HIGH(GPIOJ_PIN7) |               \
                                     PIN_ODR_HIGH(GPIOJ_PIN8) |               \
                                     PIN_ODR_HIGH(GPIOJ_PIN9) |               \
                                     PIN_ODR_HIGH(GPIOJ_PIN10) |              \
                                     PIN_ODR_HIGH(GPIOJ_PIN11) |              \
                                     PIN_ODR_HIGH(GPIOJ_PIN12) |              \
                                     PIN_ODR_HIGH(GPIOJ_PIN13) |              \
                                     PIN_ODR_HIGH(GPIOJ_PIN14) |              \
                                     PIN_ODR_HIGH(GPIOJ_PIN15))
#define VAL_GPIOJ_AFRL              (PIN_AFIO_AF(GPIOJ_PIN0, 0U) |            \
                                     PIN_AFIO_AF(GPIOJ_PIN1, 0U) |            \
                                     PIN_AFIO_AF(GPIOJ_PIN2, 0U) |            \
                                     PIN_AFIO_AF(GPIOJ_PIN3, 0U) |            \
                                     PIN_AFIO_AF(GPIOJ_PIN4, 0U) |            \
                                     PIN_AFIO_AF(GPIOJ_PIN5, 0U) |            \
                                     PIN_AFIO_AF(GPIOJ_PIN6, 0U) |            \
                                     PIN_AFIO_AF(GPIOJ_PIN7, 0U))
#define VAL_GPIOJ_AFRH              (PIN_AFIO_AF(GPIOJ_PIN8, 0U) |            \
                                     PIN_AFIO_AF(GPIOJ_PIN9, 0U) |            \
                                     PIN_AFIO_AF(GPIOJ_PIN10, 0U) |           \
                                     PIN_AFIO_AF(GPIOJ_PIN11, 0U) |           \
                                     PIN_AFIO_AF(GPIOJ_PIN12, 0U) |           \
                                     PIN_AFIO_AF(GPIOJ_PIN13, 0U) |           \
                                     PIN_AFIO_AF(GPIOJ_PIN14, 0U) |           \
                                     PIN_AFIO_AF(GPIOJ_PIN15, 0U))


/*
 * GPIOK setup:
 *
 */
#define VAL_GPIOK_MODER             (PIN_MODE_INPUT(GPIOK_PIN0) |             \
                                     PIN_MODE_INPUT(GPIOK_PIN1) |             \
                                     PIN_MODE_INPUT(GPIOK_PIN2) |             \
                                     PIN_MODE_INPUT(GPIOK_PIN3) |             \
                                     PIN_MODE_INPUT(GPIOK_PIN4) |             \
                                     PIN_MODE_INPUT(GPIOK_PIN5) |             \
                                     PIN_MODE_INPUT(GPIOK_PIN6) |             \
                                     PIN_MODE_INPUT(GPIOK_PIN7) |             \
                                     PIN_MODE_INPUT(GPIOK_PIN8) |             \
                                     PIN_MODE_INPUT(GPIOK_PIN9) |             \
                                     PIN_MODE_INPUT(GPIOK_PIN10) |            \
                                     PIN_MODE_INPUT(GPIOK_PIN11) |            \
                                     PIN_MODE_INPUT(GPIOK_PIN12) |            \
                                     PIN_MODE_INPUT(GPIOK_PIN13) |            \
                                     PIN_MODE_INPUT(GPIOK_PIN14) |            \
                                     PIN_MODE_INPUT(GPIOK_PIN15))
#define VAL_GPIOK_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOK_PIN0) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN1) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN6) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN7) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN8) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN9) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN10) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN11) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN12) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN13) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN14) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN15))
#define VAL_GPIOK_OSPEEDR           (PIN_OSPEED_HIGH(GPIOK_PIN0) |            \
                                     PIN_OSPEED_HIGH(GPIOK_PIN1) |            \
                                     PIN_OSPEED_HIGH(GPIOK_PIN2) |            \
                                     PIN_OSPEED_HIGH(GPIOK_PIN3) |            \
                                     PIN_OSPEED_HIGH(GPIOK_PIN4) |            \
                                     PIN_OSPEED_HIGH(GPIOK_PIN5) |            \
                                     PIN_OSPEED_HIGH(GPIOK_PIN6) |            \
                                     PIN_OSPEED_HIGH(GPIOK_PIN7) |            \
                                     PIN_OSPEED_HIGH(GPIOK_PIN8) |            \
                                     PIN_OSPEED_HIGH(GPIOK_PIN9) |            \
                                     PIN_OSPEED_HIGH(GPIOK_PIN10) |           \
                                     PIN_OSPEED_HIGH(GPIOK_PIN11) |           \
                                     PIN_OSPEED_HIGH(GPIOK_PIN12) |           \
                                     PIN_OSPEED_HIGH(GPIOK_PIN13) |           \
                                     PIN_OSPEED_HIGH(GPIOK_PIN14) |           \
                                     PIN_OSPEED_HIGH(GPIOK_PIN15))
#define VAL_GPIOK_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOK_PIN0) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOK_PIN1) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOK_PIN2) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOK_PIN3) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOK_PIN4) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOK_PIN5) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOK_PIN6) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOK_PIN7) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOK_PIN8) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOK_PIN9) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOK_PIN10) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOK_PIN11) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOK_PIN12) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOK_PIN13) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOK_PIN14) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOK_PIN15))
#define VAL_GPIOK_ODR               (PIN_ODR_HIGH(GPIOK_PIN0) |               \
                                     PIN_ODR_HIGH(GPIOK_PIN1) |               \
                                     PIN_ODR_HIGH(GPIOK_PIN2) |               \
                                     PIN_ODR_HIGH(GPIOK_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOK_PIN4) |               \
                                     PIN_ODR_HIGH(GPIOK_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOK_PIN6) |               \
                                     PIN_ODR_HIGH(GPIOK_PIN7) |               \
                                     PIN_ODR_HIGH(GPIOK_PIN8) |               \
                                     PIN_ODR_HIGH(GPIOK_PIN9) |               \
                                     PIN_ODR_HIGH(GPIOK_PIN10) |              \
                                     PIN_ODR_HIGH(GPIOK_PIN11) |              \
                                     PIN_ODR_HIGH(GPIOK_PIN12) |              \
                                     PIN_ODR_HIGH(GPIOK_PIN13) |              \
                                     PIN_ODR_HIGH(GPIOK_PIN14) |              \
                                     PIN_ODR_HIGH(GPIOK_PIN15))
#define VAL_GPIOK_AFRL              (PIN_AFIO_AF(GPIOK_PIN0, 0U) |            \
                                     PIN_AFIO_AF(GPIOK_PIN1, 0U) |            \
                                     PIN_AFIO_AF(GPIOK_PIN2, 0U) |            \
                                     PIN_AFIO_AF(GPIOK_PIN3, 0U) |            \
                                     PIN_AFIO_AF(GPIOK_PIN4, 0U) |            \
                                     PIN_AFIO_AF(GPIOK_PIN5, 0U) |            \
                                     PIN_AFIO_AF(GPIOK_PIN6, 0U) |            \
                                     PIN_AFIO_AF(GPIOK_PIN7, 0U))
#define VAL_GPIOK_AFRH              (PIN_AFIO_AF(GPIOK_PIN8, 0U) |            \
                                     PIN_AFIO_AF(GPIOK_PIN9, 0U) |            \
                                     PIN_AFIO_AF(GPIOK_PIN10, 0U) |           \
                                     PIN_AFIO_AF(GPIOK_PIN11, 0U) |           \
                                     PIN_AFIO_AF(GPIOK_PIN12, 0U) |           \
                                     PIN_AFIO_AF(GPIOK_PIN13, 0U) |           \
                                     PIN_AFIO_AF(GPIOK_PIN14, 0U) |           \
                                     PIN_AFIO_AF(GPIOK_PIN15, 0U))



#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
