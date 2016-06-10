#ifndef __KFLY_DEFS_H
#define __KFLY_DEFS_H

/*===========================================================================*/
/* Global includes.                                                          */
/*===========================================================================*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/*===========================================================================*/
/* Global definitions.                                                       */
/*===========================================================================*/

/**
 * @brief   Setting for max supported number of inputs.
 */
#define RCINPUT_MAX_NUMBER_OF_INPUTS    12

/*===========================================================================*/
/* Global data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Definition of a 1-bit bool struct.
 */
typedef struct PACKED_VAR {
    bool value : 1;
} bool1_t;

/**
 * @brief   Definition of a 8-bit bool struct.
 */
typedef struct PACKED_VAR {
    bool value : 1;
    uint32_t __reserved : 7;
} bool8_t;

/**
 * @brief   Definition of a 16-bit bool struct.
 */
typedef struct PACKED_VAR {
    bool value : 1;
    uint32_t __reserved : 15;
} bool16_t;

/**
 * @brief   Definition of a 32-bit bool struct.
 */
typedef struct PACKED_VAR {
    bool value : 1;
    uint32_t __reserved : 31;
} bool32_t;

/*===========================================================================*/
/* Global macros.                                                            */
/*===========================================================================*/

/* Simplifcation for defining unused parameters. */
#define UNUSED(expr)    do { (void)(expr); } while (0)

/*===========================================================================*/
/* Global inline functions.                                                  */
/*===========================================================================*/


#endif
