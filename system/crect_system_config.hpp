//          Copyright Emil Fresk 2018.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once

/*
 * Place MCU definition file here.
 *
 * Needs to define:
 * - __NVIC_PRIO_BITS     Number of priority bits
 * - __STACK_END          Address for the end of the stack
 * - __MCU_NUM_VECTORS    Number of peripheral ISR vectors (not including ARM's)
 * - __F_CPU              Clock speed of the MCU (needed for time conversions)
 * - __CRECT_ASYNC_QUEUE_SIZE  The async queue size.
 */
#include "stm32f7xx.h"

/**
 * @brief Stack definition.
 */
extern uint32_t __stack;

#define __STACK_END &__stack
#define __MCU_NUM_VECTORS (MDIOS_IRQn + 1)
#define __F_CPU 216000000
#define __CRECT_ASYNC_QUEUE_SIZE 20

/****************************************************************************
 * End MCU definitions.
 ****************************************************************************/
