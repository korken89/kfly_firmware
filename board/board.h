/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for KFly r3.1 board. By Emil Fresk.
 */

/*
 * Board identifier.
 */
#define BOARD_KFLY
#define BOARD_NAME                  "KFly"
#define BOARD_MAJOR_VERSION         3
#define BOARD_MINOR_VERSION         1


/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                12000000
#endif

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   330

/*
 * MCU type as defined in the ST header.
 */
//#define STM32F40_41xxx
#define STM32F405xx

/*
 * IO pins assignments.
 */
#define GPIOA_AUX3_TX               0
#define GPIOA_AUX3_RX               1
#define GPIOA_CONTROL_IN1           2
#define GPIOA_CONTROL_IN6           3
#define GPIOA_RF_SEL                4
#define GPIOA_RF_SCLK               5
#define GPIOA_RF_MISO               6
#define GPIOA_RF_MOSI               7
#define GPIOA_PIN8                  8
#define GPIOA_VBUS_FS               9
#define GPIOA_PIN10                 10
#define GPIOA_OTG_FS_DM             11
#define GPIOA_OTG_FS_DP             12
#define GPIOA_SWDIO                 13
#define GPIOA_SWCLK                 14
#define GPIOA_PIN15                 15

#define GPIOB_CONTROL_IN5           0
#define GPIOB_CONTROL_IN4           1
#define GPIOB_PIN2                  2
#define GPIOB_PIN3                  3
#define GPIOB_PIN4                  4
#define GPIOB_PIN5                  5
#define GPIOB_ENGINE_PWM4           6
#define GPIOB_ENGINE_PWM3           7
#define GPIOB_ENGINE_PWM2           8
#define GPIOB_ENGINE_PWM1           9
#define GPIOB_SENSORS_SCL           10
#define GPIOB_SENSORS_SDA           11
#define GPIOB_CAN_RX                12
#define GPIOB_CAN_TX                13
#define GPIOB_CONTROL_IN2           14
#define GPIOB_CONTROL_IN3           15

#define GPIOC_LED_ERR               0
#define GPIOC_LED_USR               1
#define GPIOC_PIN2                  2
#define GPIOC_PIN3                  3
#define GPIOC_FLASH_SEL             4
#define GPIOC_PIN5                  5
#define GPIOC_ENGINE_PWM8           6
#define GPIOC_ENGINE_PWM7           7
#define GPIOC_ENGINE_PWM6           8
#define GPIOC_ENGINE_PWM5           9
#define GPIOC_AUX1_TX               10
#define GPIOC_AUX1_RX               11
#define GPIOC_AUX2_TX               12
#define GPIOC_IRQ_MAG               13
#define GPIOC_IRQ_MPU               14
#define GPIOC_IRQ_RF                15

#define GPIOD_PIN0                  0
#define GPIOD_PIN1                  1
#define GPIOD_AUX2_RX               2
#define GPIOD_PIN3                  3
#define GPIOD_PIN4                  4
#define GPIOD_PIN5                  5
#define GPIOD_PIN6                  6
#define GPIOD_PIN7                  7
#define GPIOD_PIN8                  8
#define GPIOD_PIN9                  9
#define GPIOD_PIN10                 10
#define GPIOD_PIN11                 11
#define GPIOD_PIN12                 12
#define GPIOD_PIN13                 13
#define GPIOD_PIN14                 14
#define GPIOD_PIN15                 15

#define GPIOE_PIN0                  0
#define GPIOE_PIN1                  1
#define GPIOE_PIN2                  2
#define GPIOE_PIN3                  3
#define GPIOE_PIN4                  4
#define GPIOE_PIN5                  5
#define GPIOE_PIN6                  6
#define GPIOE_PIN7                  7
#define GPIOE_PIN8                  8
#define GPIOE_PIN9                  9
#define GPIOE_PIN10                 10
#define GPIOE_PIN11                 11
#define GPIOE_PIN12                 12
#define GPIOE_PIN13                 13
#define GPIOE_PIN14                 14
#define GPIOE_PIN15                 15

#define GPIOF_PIN0                  0
#define GPIOF_PIN1                  1
#define GPIOF_PIN2                  2
#define GPIOF_PIN3                  3
#define GPIOF_PIN4                  4
#define GPIOF_PIN5                  5
#define GPIOF_PIN6                  6
#define GPIOF_PIN7                  7
#define GPIOF_PIN8                  8
#define GPIOF_PIN9                  9
#define GPIOF_PIN10                 10
#define GPIOF_PIN11                 11
#define GPIOF_PIN12                 12
#define GPIOF_PIN13                 13
#define GPIOF_PIN14                 14
#define GPIOF_PIN15                 15

#define GPIOG_PIN0                  0
#define GPIOG_PIN1                  1
#define GPIOG_PIN2                  2
#define GPIOG_PIN3                  3
#define GPIOG_PIN4                  4
#define GPIOG_PIN5                  5
#define GPIOG_PIN6                  6
#define GPIOG_PIN7                  7
#define GPIOG_PIN8                  8
#define GPIOG_PIN9                  9
#define GPIOG_PIN10                 10
#define GPIOG_PIN11                 11
#define GPIOG_PIN12                 12
#define GPIOG_PIN13                 13
#define GPIOG_PIN14                 14
#define GPIOG_PIN15                 15

#define GPIOH_OSC_IN                0
#define GPIOH_OSC_OUT               1
#define GPIOH_PIN2                  2
#define GPIOH_PIN3                  3
#define GPIOH_PIN4                  4
#define GPIOH_PIN5                  5
#define GPIOH_PIN6                  6
#define GPIOH_PIN7                  7
#define GPIOH_PIN8                  8
#define GPIOH_PIN9                  9
#define GPIOH_PIN10                 10
#define GPIOH_PIN11                 11
#define GPIOH_PIN12                 12
#define GPIOH_PIN13                 13
#define GPIOH_PIN14                 14
#define GPIOH_PIN15                 15

#define GPIOI_PIN0                  0
#define GPIOI_PIN1                  1
#define GPIOI_PIN2                  2
#define GPIOI_PIN3                  3
#define GPIOI_PIN4                  4
#define GPIOI_PIN5                  5
#define GPIOI_PIN6                  6
#define GPIOI_PIN7                  7
#define GPIOI_PIN8                  8
#define GPIOI_PIN9                  9
#define GPIOI_PIN10                 10
#define GPIOI_PIN11                 11
#define GPIOI_PIN12                 12
#define GPIOI_PIN13                 13
#define GPIOI_PIN14                 14
#define GPIOI_PIN15                 15

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2))
#define PIN_AFIO_AF(n, v)           ((v##U) << ((n % 8) * 4))

/*
 * GPIOA setup:
 *
 * PA0  - AUX3_TX                   (input pullup).
 * PA1  - AUX3_RX                   (input pullup).
 * PA2  - CONTROL_IN1               (input pullup).
 * PA3  - CONTROL_IN6               (input pullup).
 * PA4  - RF_SEL                    (input pullup).
 * PA5  - RF_SCLK                   (input pullup).
 * PA6  - RF_MISO                   (input pullup).
 * PA7  - RF_MOSI                   (input pullup).
 * PA8  - PIN8                      (input pullup).
 * PA9  - VBUS_FS                   (input pullup).
 * PA10 - PIN10                     (input pullup).
 * PA11 - OTG_FS_DM                 (input pullup).
 * PA12 - OTG_FS_DP                 (input pullup).
 * PA13 - SWDIO                     (input pullup).
 * PA14 - SWCLK                     (input pullup).
 * PA15 - PIN15                     (input pullup).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_ALTERNATE(GPIOA_AUX3_TX) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_AUX3_RX) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_CONTROL_IN1) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_CONTROL_IN6) |  \
                                     PIN_MODE_OUTPUT(GPIOA_RF_SEL) |          \
                                     PIN_MODE_ALTERNATE(GPIOA_RF_SCLK) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_RF_MISO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_RF_MOSI) |      \
                                     PIN_MODE_INPUT(GPIOA_PIN8) |             \
                                     PIN_MODE_INPUT(GPIOA_VBUS_FS) |          \
                                     PIN_MODE_INPUT(GPIOA_PIN10) |            \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |        \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |        \
                                     PIN_MODE_INPUT(GPIOA_PIN15))

#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_AUX3_TX) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_AUX3_RX) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_CONTROL_IN1) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_CONTROL_IN6) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_RF_SEL) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_RF_SCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_RF_MISO) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_RF_MOSI) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN8) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_VBUS_FS) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN10) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DM) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DP) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN15))

#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_25M(GPIOA_AUX3_TX) |          \
                                     PIN_OSPEED_25M(GPIOA_AUX3_RX) |          \
                                     PIN_OSPEED_100M(GPIOA_CONTROL_IN1) |     \
                                     PIN_OSPEED_100M(GPIOA_CONTROL_IN6) |     \
                                     PIN_OSPEED_2M(GPIOA_RF_SEL) |            \
                                     PIN_OSPEED_50M(GPIOA_RF_SCLK) |          \
                                     PIN_OSPEED_50M(GPIOA_RF_MISO) |          \
                                     PIN_OSPEED_50M(GPIOA_RF_MOSI) |          \
                                     PIN_OSPEED_100M(GPIOA_PIN8) |            \
                                     PIN_OSPEED_100M(GPIOA_VBUS_FS) |         \
                                     PIN_OSPEED_100M(GPIOA_PIN10) |           \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_DM) |       \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_DP) |       \
                                     PIN_OSPEED_100M(GPIOA_SWDIO) |           \
                                     PIN_OSPEED_100M(GPIOA_SWCLK) |           \
                                     PIN_OSPEED_100M(GPIOA_PIN15))

#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_AUX3_TX) |      \
                                     PIN_PUPDR_PULLUP(GPIOA_AUX3_RX) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOA_CONTROL_IN1) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOA_CONTROL_IN6) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_RF_SEL) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_RF_SCLK) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_RF_MISO) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_RF_MOSI) |      \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN8) |           \
                                     PIN_PUPDR_PULLDOWN(GPIOA_VBUS_FS) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN10) |        \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DM) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DP) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_SWDIO) |        \
                                     PIN_PUPDR_FLOATING(GPIOA_SWCLK) |        \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN15))

#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_AUX3_TX) |            \
                                     PIN_ODR_HIGH(GPIOA_AUX3_RX) |            \
                                     PIN_ODR_HIGH(GPIOA_CONTROL_IN1) |        \
                                     PIN_ODR_HIGH(GPIOA_CONTROL_IN6) |        \
                                     PIN_ODR_HIGH(GPIOA_RF_SEL) |             \
                                     PIN_ODR_HIGH(GPIOA_RF_SCLK) |            \
                                     PIN_ODR_HIGH(GPIOA_RF_MISO) |            \
                                     PIN_ODR_HIGH(GPIOA_RF_MOSI) |            \
                                     PIN_ODR_HIGH(GPIOA_PIN8) |               \
                                     PIN_ODR_HIGH(GPIOA_VBUS_FS) |            \
                                     PIN_ODR_HIGH(GPIOA_PIN10) |              \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DM) |          \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DP) |          \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) |              \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) |              \
                                     PIN_ODR_HIGH(GPIOA_PIN15))

#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_AUX3_TX, 8) |          \
                                     PIN_AFIO_AF(GPIOA_AUX3_RX, 8) |          \
                                     PIN_AFIO_AF(GPIOA_CONTROL_IN1, 3) |      \
                                     PIN_AFIO_AF(GPIOA_CONTROL_IN6, 3) |      \
                                     PIN_AFIO_AF(GPIOA_RF_SEL, 0) |           \
                                     PIN_AFIO_AF(GPIOA_RF_SCLK, 5) |          \
                                     PIN_AFIO_AF(GPIOA_RF_MISO, 5) |          \
                                     PIN_AFIO_AF(GPIOA_RF_MOSI, 5))

#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_PIN8, 0) |             \
                                     PIN_AFIO_AF(GPIOA_VBUS_FS, 0) |          \
                                     PIN_AFIO_AF(GPIOA_PIN10, 0) |            \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10) |       \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10) |       \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0) |            \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0) |            \
                                     PIN_AFIO_AF(GPIOA_PIN15, 0))

/*
 * GPIOB setup:
 *
 * PB0  - CONTROL_IN5               (input pullup).
 * PB1  - CONTROL_IN4               (input pullup).
 * PB2  - PIN2                      (input pullup).
 * PB3  - PIN3                      (input pullup).
 * PB4  - PIN4                      (input pullup).
 * PB5  - PIN5                      (input pullup).
 * PB6  - ENGINE_PWM4               (input pullup).
 * PB7  - ENGINE_PWM3               (input pullup).
 * PB8  - ENGINE_PWM2               (input pullup).
 * PB9  - ENGINE_PWM1               (input pullup).
 * PB10 - SENSORS_SCL               (input pullup).
 * PB11 - SENSORS_SDA               (input pullup).
 * PB12 - CAN_RX                    (input pullup).
 * PB13 - CAN_TX                    (input pullup).
 * PB14 - CONTROL_IN2               (input pullup).
 * PB15 - CONTROL_IN3               (input pullup).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_ALTERNATE(GPIOB_CONTROL_IN5) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_CONTROL_IN4) |  \
                                     PIN_MODE_INPUT(GPIOB_PIN2) |             \
                                     PIN_MODE_INPUT(GPIOB_PIN3) |             \
                                     PIN_MODE_INPUT(GPIOB_PIN4) |             \
                                     PIN_MODE_INPUT(GPIOB_PIN5) |             \
                                     PIN_MODE_ALTERNATE(GPIOB_ENGINE_PWM4) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_ENGINE_PWM3) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_ENGINE_PWM2) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_ENGINE_PWM1) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_SENSORS_SCL) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_SENSORS_SDA) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_CAN_RX) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_CAN_TX) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_CONTROL_IN2) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_CONTROL_IN3))

#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_CONTROL_IN5) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_CONTROL_IN4) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN4) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ENGINE_PWM4) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ENGINE_PWM3) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ENGINE_PWM2) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ENGINE_PWM1) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_SENSORS_SCL) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_SENSORS_SDA) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAN_RX) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAN_TX) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_CONTROL_IN2) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_CONTROL_IN3))

#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_100M(GPIOB_CONTROL_IN5) |     \
                                     PIN_OSPEED_100M(GPIOB_CONTROL_IN4) |     \
                                     PIN_OSPEED_100M(GPIOB_PIN2) |            \
                                     PIN_OSPEED_100M(GPIOB_PIN3) |            \
                                     PIN_OSPEED_100M(GPIOB_PIN4) |            \
                                     PIN_OSPEED_100M(GPIOB_PIN5) |            \
                                     PIN_OSPEED_2M(GPIOB_ENGINE_PWM4) |       \
                                     PIN_OSPEED_2M(GPIOB_ENGINE_PWM3) |       \
                                     PIN_OSPEED_2M(GPIOB_ENGINE_PWM2) |       \
                                     PIN_OSPEED_2M(GPIOB_ENGINE_PWM1) |       \
                                     PIN_OSPEED_2M(GPIOB_SENSORS_SCL) |       \
                                     PIN_OSPEED_2M(GPIOB_SENSORS_SDA) |       \
                                     PIN_OSPEED_25M(GPIOB_CAN_RX) |           \
                                     PIN_OSPEED_25M(GPIOB_CAN_TX) |           \
                                     PIN_OSPEED_100M(GPIOB_CONTROL_IN2) |     \
                                     PIN_OSPEED_100M(GPIOB_CONTROL_IN3))

#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOB_CONTROL_IN5) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOB_CONTROL_IN4) |  \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN2) |           \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN3) |           \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN4) |           \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN5) |           \
                                     PIN_PUPDR_PULLUP(GPIOB_ENGINE_PWM4) |    \
                                     PIN_PUPDR_PULLUP(GPIOB_ENGINE_PWM3) |    \
                                     PIN_PUPDR_PULLUP(GPIOB_ENGINE_PWM2) |    \
                                     PIN_PUPDR_PULLUP(GPIOB_ENGINE_PWM1) |    \
                                     PIN_PUPDR_PULLUP(GPIOB_SENSORS_SCL) |    \
                                     PIN_PUPDR_PULLUP(GPIOB_SENSORS_SDA) |    \
                                     PIN_PUPDR_PULLUP(GPIOB_CAN_RX) |         \
                                     PIN_PUPDR_PULLUP(GPIOB_CAN_TX) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOB_CONTROL_IN2) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOB_CONTROL_IN3))

#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_CONTROL_IN5) |        \
                                     PIN_ODR_HIGH(GPIOB_CONTROL_IN4) |        \
                                     PIN_ODR_HIGH(GPIOB_PIN2) |               \
                                     PIN_ODR_HIGH(GPIOB_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOB_PIN4) |               \
                                     PIN_ODR_HIGH(GPIOB_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOB_ENGINE_PWM4) |        \
                                     PIN_ODR_HIGH(GPIOB_ENGINE_PWM3) |        \
                                     PIN_ODR_HIGH(GPIOB_ENGINE_PWM2) |        \
                                     PIN_ODR_HIGH(GPIOB_ENGINE_PWM1) |        \
                                     PIN_ODR_HIGH(GPIOB_SENSORS_SCL) |        \
                                     PIN_ODR_HIGH(GPIOB_SENSORS_SDA) |        \
                                     PIN_ODR_HIGH(GPIOB_CAN_RX) |             \
                                     PIN_ODR_HIGH(GPIOB_CAN_TX) |             \
                                     PIN_ODR_HIGH(GPIOB_CONTROL_IN2) |        \
                                     PIN_ODR_HIGH(GPIOB_CONTROL_IN3))

#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_CONTROL_IN5, 2) |      \
                                     PIN_AFIO_AF(GPIOB_CONTROL_IN4, 2) |      \
                                     PIN_AFIO_AF(GPIOB_PIN2, 0) |             \
                                     PIN_AFIO_AF(GPIOB_PIN3, 0) |             \
                                     PIN_AFIO_AF(GPIOB_PIN4, 0) |             \
                                     PIN_AFIO_AF(GPIOB_PIN5, 0) |             \
                                     PIN_AFIO_AF(GPIOB_ENGINE_PWM4, 2) |      \
                                     PIN_AFIO_AF(GPIOB_ENGINE_PWM3, 2))

#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_ENGINE_PWM2, 2) |      \
                                     PIN_AFIO_AF(GPIOB_ENGINE_PWM1, 2) |      \
                                     PIN_AFIO_AF(GPIOB_SENSORS_SCL, 4) |      \
                                     PIN_AFIO_AF(GPIOB_SENSORS_SDA, 4) |      \
                                     PIN_AFIO_AF(GPIOB_CAN_RX, 9) |           \
                                     PIN_AFIO_AF(GPIOB_CAN_TX, 9) |           \
                                     PIN_AFIO_AF(GPIOB_CONTROL_IN2, 9) |      \
                                     PIN_AFIO_AF(GPIOB_CONTROL_IN3, 9))

/*
 * GPIOC setup:
 *
 * PC0  - LED_ERR                   (input pullup).
 * PC1  - LED_USR                   (input pullup).
 * PC2  - PIN2                      (input pullup).
 * PC3  - PIN3                      (input pullup).
 * PC4  - FLASH_SEL                 (input pullup).
 * PC5  - PIN5                      (input pullup).
 * PC6  - ENGINE_PWM8               (input pullup).
 * PC7  - ENGINE_PWM7               (input pullup).
 * PC8  - ENGINE_PWM6               (input pullup).
 * PC9  - ENGINE_PWM5               (input pullup).
 * PC10 - AUX1_TX                   (input pullup).
 * PC11 - AUX1_RX                   (input pullup).
 * PC12 - AUX2_TX                   (input pullup).
 * PC13 - IRQ_MAG                   (input pullup).
 * PC14 - IRQ_MPU                   (input pullup).
 * PC15 - IRQ_RF                    (input pullup).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_OUTPUT(GPIOC_LED_ERR) |         \
                                     PIN_MODE_OUTPUT(GPIOC_LED_USR) |         \
                                     PIN_MODE_INPUT(GPIOC_PIN2) |             \
                                     PIN_MODE_INPUT(GPIOC_PIN3) |             \
                                     PIN_MODE_OUTPUT(GPIOC_FLASH_SEL) |       \
                                     PIN_MODE_INPUT(GPIOC_PIN5) |             \
                                     PIN_MODE_ALTERNATE(GPIOC_ENGINE_PWM8) |  \
                                     PIN_MODE_ALTERNATE(GPIOC_ENGINE_PWM7) |  \
                                     PIN_MODE_ALTERNATE(GPIOC_ENGINE_PWM6) |  \
                                     PIN_MODE_ALTERNATE(GPIOC_ENGINE_PWM5) |  \
                                     PIN_MODE_ALTERNATE(GPIOC_AUX1_TX) |      \
                                     PIN_MODE_ALTERNATE(GPIOC_AUX1_RX) |      \
                                     PIN_MODE_ALTERNATE(GPIOC_AUX2_TX) |      \
                                     PIN_MODE_INPUT(GPIOC_IRQ_MAG) |          \
                                     PIN_MODE_INPUT(GPIOC_IRQ_MPU) |          \
                                     PIN_MODE_INPUT(GPIOC_IRQ_RF))

#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_LED_ERR) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_LED_USR) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_FLASH_SEL) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN5) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ENGINE_PWM8) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ENGINE_PWM7) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ENGINE_PWM6) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ENGINE_PWM5) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_AUX1_TX) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_AUX1_RX) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_AUX2_TX) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_IRQ_MAG) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_IRQ_MPU) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_IRQ_RF))

#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_2M(GPIOC_LED_ERR) |           \
                                     PIN_OSPEED_2M(GPIOC_LED_USR) |           \
                                     PIN_OSPEED_100M(GPIOC_PIN2) |            \
                                     PIN_OSPEED_100M(GPIOC_PIN3) |            \
                                     PIN_OSPEED_2M(GPIOC_FLASH_SEL) |         \
                                     PIN_OSPEED_100M(GPIOC_PIN5) |            \
                                     PIN_OSPEED_2M(GPIOC_ENGINE_PWM8) |       \
                                     PIN_OSPEED_2M(GPIOC_ENGINE_PWM7) |       \
                                     PIN_OSPEED_2M(GPIOC_ENGINE_PWM6) |       \
                                     PIN_OSPEED_2M(GPIOC_ENGINE_PWM5) |       \
                                     PIN_OSPEED_25M(GPIOC_AUX1_TX) |          \
                                     PIN_OSPEED_25M(GPIOC_AUX1_RX) |          \
                                     PIN_OSPEED_25M(GPIOC_AUX2_TX) |          \
                                     PIN_OSPEED_100M(GPIOC_IRQ_MAG) |         \
                                     PIN_OSPEED_100M(GPIOC_IRQ_MPU) |         \
                                     PIN_OSPEED_100M(GPIOC_IRQ_RF))

#define VAL_GPIOC_PUPDR             (PIN_PUPDR_PULLUP(GPIOC_LED_ERR) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_LED_USR) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN2) |           \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN3) |           \
                                     PIN_PUPDR_PULLUP(GPIOC_FLASH_SEL) |      \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN5) |           \
                                     PIN_PUPDR_PULLUP(GPIOC_ENGINE_PWM8) |    \
                                     PIN_PUPDR_PULLUP(GPIOC_ENGINE_PWM7) |    \
                                     PIN_PUPDR_PULLUP(GPIOC_ENGINE_PWM6) |    \
                                     PIN_PUPDR_PULLUP(GPIOC_ENGINE_PWM5) |    \
                                     PIN_PUPDR_PULLUP(GPIOC_AUX1_TX) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_AUX1_RX) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_AUX2_TX) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_IRQ_MAG) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_IRQ_MPU) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_IRQ_RF))

#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_LED_ERR) |            \
                                     PIN_ODR_HIGH(GPIOC_LED_USR) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN2) |               \
                                     PIN_ODR_HIGH(GPIOC_PIN3) |               \
                                     PIN_ODR_HIGH(GPIOC_FLASH_SEL) |          \
                                     PIN_ODR_HIGH(GPIOC_PIN5) |               \
                                     PIN_ODR_HIGH(GPIOC_ENGINE_PWM8) |        \
                                     PIN_ODR_HIGH(GPIOC_ENGINE_PWM7) |        \
                                     PIN_ODR_HIGH(GPIOC_ENGINE_PWM6) |        \
                                     PIN_ODR_HIGH(GPIOC_ENGINE_PWM5) |        \
                                     PIN_ODR_HIGH(GPIOC_AUX1_TX) |            \
                                     PIN_ODR_HIGH(GPIOC_AUX1_RX) |            \
                                     PIN_ODR_HIGH(GPIOC_AUX2_TX) |            \
                                     PIN_ODR_HIGH(GPIOC_IRQ_MAG) |            \
                                     PIN_ODR_HIGH(GPIOC_IRQ_MPU) |            \
                                     PIN_ODR_HIGH(GPIOC_IRQ_RF))

#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_LED_ERR, 0) |          \
                                     PIN_AFIO_AF(GPIOC_LED_USR, 0) |          \
                                     PIN_AFIO_AF(GPIOC_PIN2, 0) |             \
                                     PIN_AFIO_AF(GPIOC_PIN3, 0) |             \
                                     PIN_AFIO_AF(GPIOC_FLASH_SEL, 0) |        \
                                     PIN_AFIO_AF(GPIOC_PIN5, 0) |             \
                                     PIN_AFIO_AF(GPIOC_ENGINE_PWM8, 3) |      \
                                     PIN_AFIO_AF(GPIOC_ENGINE_PWM7, 3))

#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_ENGINE_PWM6, 3) |      \
                                     PIN_AFIO_AF(GPIOC_ENGINE_PWM5, 3) |      \
                                     PIN_AFIO_AF(GPIOC_AUX1_TX, 7) |          \
                                     PIN_AFIO_AF(GPIOC_AUX1_RX, 7) |          \
                                     PIN_AFIO_AF(GPIOC_AUX2_TX, 8) |          \
                                     PIN_AFIO_AF(GPIOC_IRQ_MAG, 0) |          \
                                     PIN_AFIO_AF(GPIOC_IRQ_MPU, 0) |          \
                                     PIN_AFIO_AF(GPIOC_IRQ_RF, 0))

/*
 * GPIOD setup:
 *
 * PD0  - PIN0                      (input pullup).
 * PD1  - PIN1                      (input pullup).
 * PD2  - AUX2_RX                   (input pullup).
 * PD3  - PIN3                      (input pullup).
 * PD4  - PIN4                      (input pullup).
 * PD5  - PIN5                      (input pullup).
 * PD6  - PIN6                      (input pullup).
 * PD7  - PIN7                      (input pullup).
 * PD8  - PIN8                      (input pullup).
 * PD9  - PIN9                      (input pullup).
 * PD10 - PIN10                     (input pullup).
 * PD11 - PIN11                     (input pullup).
 * PD12 - PIN12                     (input pullup).
 * PD13 - PIN13                     (input pullup).
 * PD14 - PIN14                     (input pullup).
 * PD15 - PIN15                     (input pullup).
 */
 #define VAL_GPIOD_MODER            (PIN_MODE_INPUT(GPIOD_PIN0) |             \
                                     PIN_MODE_INPUT(GPIOD_PIN1) |             \
                                     PIN_MODE_ALTERNATE(GPIOD_AUX2_RX) |      \
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
                                     PIN_OTYPE_PUSHPULL(GPIOD_AUX2_RX) |      \
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

#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_100M(GPIOD_PIN0) |            \
                                     PIN_OSPEED_100M(GPIOD_PIN1) |            \
                                     PIN_OSPEED_25M(GPIOD_AUX2_RX) |          \
                                     PIN_OSPEED_100M(GPIOD_PIN3) |            \
                                     PIN_OSPEED_100M(GPIOD_PIN4) |            \
                                     PIN_OSPEED_100M(GPIOD_PIN5) |            \
                                     PIN_OSPEED_100M(GPIOD_PIN6) |            \
                                     PIN_OSPEED_100M(GPIOD_PIN7) |            \
                                     PIN_OSPEED_100M(GPIOD_PIN8) |            \
                                     PIN_OSPEED_100M(GPIOD_PIN9) |            \
                                     PIN_OSPEED_100M(GPIOD_PIN10) |           \
                                     PIN_OSPEED_100M(GPIOD_PIN11) |           \
                                     PIN_OSPEED_100M(GPIOD_PIN12) |           \
                                     PIN_OSPEED_100M(GPIOD_PIN13) |           \
                                     PIN_OSPEED_100M(GPIOD_PIN14) |           \
                                     PIN_OSPEED_100M(GPIOD_PIN15))

#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_PIN0) |         \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_AUX2_RX) |        \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN3) |         \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN4) |         \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN5) |         \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN6) |         \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN7) |         \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN8) |         \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN9) |         \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN10) |        \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN11) |        \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN12) |        \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN13) |        \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN14) |        \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN15))

#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_PIN0) |               \
                                     PIN_ODR_HIGH(GPIOD_PIN1) |               \
                                     PIN_ODR_HIGH(GPIOD_AUX2_RX) |            \
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

#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_PIN0, 0) |             \
                                     PIN_AFIO_AF(GPIOD_PIN1, 0) |             \
                                     PIN_AFIO_AF(GPIOD_AUX2_RX, 8) |          \
                                     PIN_AFIO_AF(GPIOD_PIN3, 0) |             \
                                     PIN_AFIO_AF(GPIOD_PIN4, 0) |             \
                                     PIN_AFIO_AF(GPIOD_PIN5, 0) |             \
                                     PIN_AFIO_AF(GPIOD_PIN6, 0) |             \
                                     PIN_AFIO_AF(GPIOD_PIN7, 0))

#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8, 0) |             \
                                     PIN_AFIO_AF(GPIOD_PIN9, 0) |             \
                                     PIN_AFIO_AF(GPIOD_PIN10, 0) |            \
                                     PIN_AFIO_AF(GPIOD_PIN11, 0) |            \
                                     PIN_AFIO_AF(GPIOD_PIN12, 0) |            \
                                     PIN_AFIO_AF(GPIOD_PIN13, 0) |            \
                                     PIN_AFIO_AF(GPIOD_PIN14, 0) |            \
                                     PIN_AFIO_AF(GPIOD_PIN15, 0))

/*
 * GPIOE setup:
 *
 * PE0  - PIN0                      (input pullup).
 * PE1  - PIN1                      (input pullup).
 * PE2  - PIN2                      (input pullup).
 * PE3  - PIN3                      (input pullup).
 * PE4  - PIN4                      (input pullup).
 * PE5  - PIN5                      (input pullup).
 * PE6  - PIN6                      (input pullup).
 * PE7  - PIN7                      (input pullup).
 * PE8  - PIN8                      (input pullup).
 * PE9  - PIN9                      (input pullup).
 * PE10 - PIN10                     (input pullup).
 * PE11 - PIN11                     (input pullup).
 * PE12 - PIN12                     (input pullup).
 * PE13 - PIN13                     (input pullup).
 * PE14 - PIN14                     (input pullup).
 * PE15 - PIN15                     (input pullup).
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
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_100M(GPIOE_PIN0) |            \
                                     PIN_OSPEED_100M(GPIOE_PIN1) |            \
                                     PIN_OSPEED_100M(GPIOE_PIN2) |            \
                                     PIN_OSPEED_100M(GPIOE_PIN3) |            \
                                     PIN_OSPEED_100M(GPIOE_PIN4) |            \
                                     PIN_OSPEED_100M(GPIOE_PIN5) |            \
                                     PIN_OSPEED_100M(GPIOE_PIN6) |            \
                                     PIN_OSPEED_100M(GPIOE_PIN7) |            \
                                     PIN_OSPEED_100M(GPIOE_PIN8) |            \
                                     PIN_OSPEED_100M(GPIOE_PIN9) |            \
                                     PIN_OSPEED_100M(GPIOE_PIN10) |           \
                                     PIN_OSPEED_100M(GPIOE_PIN11) |           \
                                     PIN_OSPEED_100M(GPIOE_PIN12) |           \
                                     PIN_OSPEED_100M(GPIOE_PIN13) |           \
                                     PIN_OSPEED_100M(GPIOE_PIN14) |           \
                                     PIN_OSPEED_100M(GPIOE_PIN15))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(GPIOE_PIN0) |         \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN1) |         \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN2) |         \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN3) |         \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN4) |         \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN5) |         \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN6) |         \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN7) |         \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN8) |         \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN9) |         \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN10) |        \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN11) |        \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN12) |        \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN13) |        \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN14) |        \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN15))
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
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_PIN0, 0) |             \
                                     PIN_AFIO_AF(GPIOE_PIN1, 0) |             \
                                     PIN_AFIO_AF(GPIOE_PIN2, 0) |             \
                                     PIN_AFIO_AF(GPIOE_PIN3, 0) |             \
                                     PIN_AFIO_AF(GPIOE_PIN4, 0) |             \
                                     PIN_AFIO_AF(GPIOE_PIN5, 0) |             \
                                     PIN_AFIO_AF(GPIOE_PIN6, 0) |             \
                                     PIN_AFIO_AF(GPIOE_PIN7, 0))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_PIN8, 0) |             \
                                     PIN_AFIO_AF(GPIOE_PIN9, 0) |             \
                                     PIN_AFIO_AF(GPIOE_PIN10, 0) |            \
                                     PIN_AFIO_AF(GPIOE_PIN11, 0) |            \
                                     PIN_AFIO_AF(GPIOE_PIN12, 0) |            \
                                     PIN_AFIO_AF(GPIOE_PIN13, 0) |            \
                                     PIN_AFIO_AF(GPIOE_PIN14, 0) |            \
                                     PIN_AFIO_AF(GPIOE_PIN15, 0))

/*
 * GPIOF setup:
 *
 * PF0  - PIN0                      (input pullup).
 * PF1  - PIN1                      (input pullup).
 * PF2  - PIN2                      (input pullup).
 * PF3  - PIN3                      (input pullup).
 * PF4  - PIN4                      (input pullup).
 * PF5  - PIN5                      (input pullup).
 * PF6  - PIN6                      (input pullup).
 * PF7  - PIN7                      (input pullup).
 * PF8  - PIN8                      (input pullup).
 * PF9  - PIN9                      (input pullup).
 * PF10 - PIN10                     (input pullup).
 * PF11 - PIN11                     (input pullup).
 * PF12 - PIN12                     (input pullup).
 * PF13 - PIN13                     (input pullup).
 * PF14 - PIN14                     (input pullup).
 * PF15 - PIN15                     (input pullup).
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
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_100M(GPIOF_PIN0) |            \
                                     PIN_OSPEED_100M(GPIOF_PIN1) |            \
                                     PIN_OSPEED_100M(GPIOF_PIN2) |            \
                                     PIN_OSPEED_100M(GPIOF_PIN3) |            \
                                     PIN_OSPEED_100M(GPIOF_PIN4) |            \
                                     PIN_OSPEED_100M(GPIOF_PIN5) |            \
                                     PIN_OSPEED_100M(GPIOF_PIN6) |            \
                                     PIN_OSPEED_100M(GPIOF_PIN7) |            \
                                     PIN_OSPEED_100M(GPIOF_PIN8) |            \
                                     PIN_OSPEED_100M(GPIOF_PIN9) |            \
                                     PIN_OSPEED_100M(GPIOF_PIN10) |           \
                                     PIN_OSPEED_100M(GPIOF_PIN11) |           \
                                     PIN_OSPEED_100M(GPIOF_PIN12) |           \
                                     PIN_OSPEED_100M(GPIOF_PIN13) |           \
                                     PIN_OSPEED_100M(GPIOF_PIN14) |           \
                                     PIN_OSPEED_100M(GPIOF_PIN15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_PIN0) |         \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN1) |         \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN2) |         \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN3) |         \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN4) |         \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN5) |         \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN6) |         \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN7) |         \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN8) |         \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN9) |         \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN10) |        \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN11) |        \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN12) |        \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN13) |        \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN14) |        \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN15))
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
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_PIN0, 0) |             \
                                     PIN_AFIO_AF(GPIOF_PIN1, 0) |             \
                                     PIN_AFIO_AF(GPIOF_PIN2, 0) |             \
                                     PIN_AFIO_AF(GPIOF_PIN3, 0) |             \
                                     PIN_AFIO_AF(GPIOF_PIN4, 0) |             \
                                     PIN_AFIO_AF(GPIOF_PIN5, 0) |             \
                                     PIN_AFIO_AF(GPIOF_PIN6, 0) |             \
                                     PIN_AFIO_AF(GPIOF_PIN7, 0))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8, 0) |             \
                                     PIN_AFIO_AF(GPIOF_PIN9, 0) |             \
                                     PIN_AFIO_AF(GPIOF_PIN10, 0) |            \
                                     PIN_AFIO_AF(GPIOF_PIN11, 0) |            \
                                     PIN_AFIO_AF(GPIOF_PIN12, 0) |            \
                                     PIN_AFIO_AF(GPIOF_PIN13, 0) |            \
                                     PIN_AFIO_AF(GPIOF_PIN14, 0) |            \
                                     PIN_AFIO_AF(GPIOF_PIN15, 0))

/*
 * GPIOG setup:
 *
 * PG0  - PIN0                      (input pullup).
 * PG1  - PIN1                      (input pullup).
 * PG2  - PIN2                      (input pullup).
 * PG3  - PIN3                      (input pullup).
 * PG4  - PIN4                      (input pullup).
 * PG5  - PIN5                      (input pullup).
 * PG6  - PIN6                      (input pullup).
 * PG7  - PIN7                      (input pullup).
 * PG8  - PIN8                      (input pullup).
 * PG9  - PIN9                      (input pullup).
 * PG10 - PIN10                     (input pullup).
 * PG11 - PIN11                     (input pullup).
 * PG12 - PIN12                     (input pullup).
 * PG13 - PIN13                     (input pullup).
 * PG14 - PIN14                     (input pullup).
 * PG15 - PIN15                     (input pullup).
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
                                     PIN_MODE_INPUT(GPIOG_PIN13) |            \
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
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN13) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN14) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN15))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_100M(GPIOG_PIN0) |            \
                                     PIN_OSPEED_100M(GPIOG_PIN1) |            \
                                     PIN_OSPEED_100M(GPIOG_PIN2) |            \
                                     PIN_OSPEED_100M(GPIOG_PIN3) |            \
                                     PIN_OSPEED_100M(GPIOG_PIN4) |            \
                                     PIN_OSPEED_100M(GPIOG_PIN5) |            \
                                     PIN_OSPEED_100M(GPIOG_PIN6) |            \
                                     PIN_OSPEED_100M(GPIOG_PIN7) |            \
                                     PIN_OSPEED_100M(GPIOG_PIN8) |            \
                                     PIN_OSPEED_100M(GPIOG_PIN9) |            \
                                     PIN_OSPEED_100M(GPIOG_PIN10) |           \
                                     PIN_OSPEED_100M(GPIOG_PIN11) |           \
                                     PIN_OSPEED_100M(GPIOG_PIN12) |           \
                                     PIN_OSPEED_100M(GPIOG_PIN13) |           \
                                     PIN_OSPEED_100M(GPIOG_PIN14) |           \
                                     PIN_OSPEED_100M(GPIOG_PIN15))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_FLOATING(GPIOG_PIN0) |         \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN1) |         \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN2) |         \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN3) |         \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN4) |         \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN5) |         \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN6) |         \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN7) |         \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN8) |         \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN9) |         \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN10) |        \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN11) |        \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN12) |        \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN13) |        \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN14) |        \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN15))
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
                                     PIN_ODR_HIGH(GPIOG_PIN13) |              \
                                     PIN_ODR_HIGH(GPIOG_PIN14) |              \
                                     PIN_ODR_HIGH(GPIOG_PIN15))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_PIN0, 0) |             \
                                     PIN_AFIO_AF(GPIOG_PIN1, 0) |             \
                                     PIN_AFIO_AF(GPIOG_PIN2, 0) |             \
                                     PIN_AFIO_AF(GPIOG_PIN3, 0) |             \
                                     PIN_AFIO_AF(GPIOG_PIN4, 0) |             \
                                     PIN_AFIO_AF(GPIOG_PIN5, 0) |             \
                                     PIN_AFIO_AF(GPIOG_PIN6, 0) |             \
                                     PIN_AFIO_AF(GPIOG_PIN7, 0))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_PIN8, 0) |             \
                                     PIN_AFIO_AF(GPIOG_PIN9, 0) |             \
                                     PIN_AFIO_AF(GPIOG_PIN10, 0) |            \
                                     PIN_AFIO_AF(GPIOG_PIN11, 0) |            \
                                     PIN_AFIO_AF(GPIOG_PIN12, 0) |            \
                                     PIN_AFIO_AF(GPIOG_PIN13, 0) |            \
                                     PIN_AFIO_AF(GPIOG_PIN14, 0) |            \
                                     PIN_AFIO_AF(GPIOG_PIN15, 0))

/*
 * GPIOH setup:
 *
 * PH0  - OSC_IN                    (input pullup).
 * PH1  - OSC_OUT                   (input pullup).
 * PH2  - PIN2                      (input pullup).
 * PH3  - PIN3                      (input pullup).
 * PH4  - PIN4                      (input pullup).
 * PH5  - PIN5                      (input pullup).
 * PH6  - PIN6                      (input pullup).
 * PH7  - PIN7                      (input pullup).
 * PH8  - PIN8                      (input pullup).
 * PH9  - PIN9                      (input pullup).
 * PH10 - PIN10                     (input pullup).
 * PH11 - PIN11                     (input pullup).
 * PH12 - PIN12                     (input pullup).
 * PH13 - PIN13                     (input pullup).
 * PH14 - PIN14                     (input pullup).
 * PH15 - PIN15                     (input pullup).
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
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_100M(GPIOH_OSC_IN) |          \
                                     PIN_OSPEED_100M(GPIOH_OSC_OUT) |         \
                                     PIN_OSPEED_100M(GPIOH_PIN2) |            \
                                     PIN_OSPEED_100M(GPIOH_PIN3) |            \
                                     PIN_OSPEED_100M(GPIOH_PIN4) |            \
                                     PIN_OSPEED_100M(GPIOH_PIN5) |            \
                                     PIN_OSPEED_100M(GPIOH_PIN6) |            \
                                     PIN_OSPEED_100M(GPIOH_PIN7) |            \
                                     PIN_OSPEED_100M(GPIOH_PIN8) |            \
                                     PIN_OSPEED_100M(GPIOH_PIN9) |            \
                                     PIN_OSPEED_100M(GPIOH_PIN10) |           \
                                     PIN_OSPEED_100M(GPIOH_PIN11) |           \
                                     PIN_OSPEED_100M(GPIOH_PIN12) |           \
                                     PIN_OSPEED_100M(GPIOH_PIN13) |           \
                                     PIN_OSPEED_100M(GPIOH_PIN14) |           \
                                     PIN_OSPEED_100M(GPIOH_PIN15))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN2) |         \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN3) |         \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN4) |         \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN5) |         \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN6) |         \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN7) |         \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN8) |         \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN9) |         \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN10) |        \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN11) |        \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN12) |        \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN13) |        \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN14) |        \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN15))
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
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0) |           \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN2, 0) |             \
                                     PIN_AFIO_AF(GPIOH_PIN3, 0) |             \
                                     PIN_AFIO_AF(GPIOH_PIN4, 0) |             \
                                     PIN_AFIO_AF(GPIOH_PIN5, 0) |             \
                                     PIN_AFIO_AF(GPIOH_PIN6, 0) |             \
                                     PIN_AFIO_AF(GPIOH_PIN7, 0))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_PIN8, 0) |             \
                                     PIN_AFIO_AF(GPIOH_PIN9, 0) |             \
                                     PIN_AFIO_AF(GPIOH_PIN10, 0) |            \
                                     PIN_AFIO_AF(GPIOH_PIN11, 0) |            \
                                     PIN_AFIO_AF(GPIOH_PIN12, 0) |            \
                                     PIN_AFIO_AF(GPIOH_PIN13, 0) |            \
                                     PIN_AFIO_AF(GPIOH_PIN14, 0) |            \
                                     PIN_AFIO_AF(GPIOH_PIN15, 0))

/*
 * GPIOI setup:
 *
 * PI0  - PIN0                      (input pullup).
 * PI1  - PIN1                      (input pullup).
 * PI2  - PIN2                      (input pullup).
 * PI3  - PIN3                      (input pullup).
 * PI4  - PIN4                      (input pullup).
 * PI5  - PIN5                      (input pullup).
 * PI6  - PIN6                      (input pullup).
 * PI7  - PIN7                      (input pullup).
 * PI8  - PIN8                      (input pullup).
 * PI9  - PIN9                      (input pullup).
 * PI10 - PIN10                     (input pullup).
 * PI11 - PIN11                     (input pullup).
 * PI12 - PIN12                     (input pullup).
 * PI13 - PIN13                     (input pullup).
 * PI14 - PIN14                     (input pullup).
 * PI15 - PIN15                     (input pullup).
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
#define VAL_GPIOI_OSPEEDR           (PIN_OSPEED_100M(GPIOI_PIN0) |            \
                                     PIN_OSPEED_100M(GPIOI_PIN1) |            \
                                     PIN_OSPEED_100M(GPIOI_PIN2) |            \
                                     PIN_OSPEED_100M(GPIOI_PIN3) |            \
                                     PIN_OSPEED_100M(GPIOI_PIN4) |            \
                                     PIN_OSPEED_100M(GPIOI_PIN5) |            \
                                     PIN_OSPEED_100M(GPIOI_PIN6) |            \
                                     PIN_OSPEED_100M(GPIOI_PIN7) |            \
                                     PIN_OSPEED_100M(GPIOI_PIN8) |            \
                                     PIN_OSPEED_100M(GPIOI_PIN9) |            \
                                     PIN_OSPEED_100M(GPIOI_PIN10) |           \
                                     PIN_OSPEED_100M(GPIOI_PIN11) |           \
                                     PIN_OSPEED_100M(GPIOI_PIN12) |           \
                                     PIN_OSPEED_100M(GPIOI_PIN13) |           \
                                     PIN_OSPEED_100M(GPIOI_PIN14) |           \
                                     PIN_OSPEED_100M(GPIOI_PIN15))
#define VAL_GPIOI_PUPDR             (PIN_PUPDR_FLOATING(GPIOI_PIN0) |         \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN1) |         \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN2) |         \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN3) |         \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN4) |         \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN5) |         \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN6) |         \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN7) |         \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN8) |         \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN9) |         \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN10) |        \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN11) |        \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN12) |        \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN13) |        \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN14) |        \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN15))
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
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_PIN0, 0) |             \
                                     PIN_AFIO_AF(GPIOI_PIN1, 0) |             \
                                     PIN_AFIO_AF(GPIOI_PIN2, 0) |             \
                                     PIN_AFIO_AF(GPIOI_PIN3, 0) |             \
                                     PIN_AFIO_AF(GPIOI_PIN4, 0) |             \
                                     PIN_AFIO_AF(GPIOI_PIN5, 0) |             \
                                     PIN_AFIO_AF(GPIOI_PIN6, 0) |             \
                                     PIN_AFIO_AF(GPIOI_PIN7, 0))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(GPIOI_PIN8, 0) |             \
                                     PIN_AFIO_AF(GPIOI_PIN9, 0) |             \
                                     PIN_AFIO_AF(GPIOI_PIN10, 0) |            \
                                     PIN_AFIO_AF(GPIOI_PIN11, 0) |            \
                                     PIN_AFIO_AF(GPIOI_PIN12, 0) |            \
                                     PIN_AFIO_AF(GPIOI_PIN13, 0) |            \
                                     PIN_AFIO_AF(GPIOI_PIN14, 0) |            \
                                     PIN_AFIO_AF(GPIOI_PIN15, 0))


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
