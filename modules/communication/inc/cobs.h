#ifndef __COBS_H
#define __COBS_H

#include "circularbuffer.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/


/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef enum
{
    COBS_FrameDelimiter = 0x00, /* Unused (framing character placeholder)    */

    COBS_DiffZero = 0x01,       /* Range 0x01 - 0xD1:                        */
    COBS_DiffZeroMax = 0xD1,    /* n-1 explicit characters plus a zero       */
    COBS_Diff = 0xD2,           /* 209 explicit characters, no added zero    */

    COBS_RunZero = 0xD3,        /* Range 0xD3 - 0xDF:                        */
    COBS_RunZeroMax = 0xDF,     /* 3-15 zeros (Zero Run Encoding)            */

    COBS_Diff2Zero = 0xE0,      /* Range 0xE0 - 0xFF:                        */
    COBS_Diff2ZeroMax = 0xFF,   /* 0-31 characters plus 2 implicit zeros (Zero
                                   Pair Encoding)                            */
} cobs_stuffingcode_t;

/**
 * @brief   Possible states of the COBS state machine.
 */
typedef enum
{
    /**
     * @brief   Parser not initialized yet.
     */
    COBS_STATE_UNINITIALIZED = 0,
    /**
     * @brief   Waiting for the code byte.
     */
    COBS_STATE_AWAITING_CODE,
    /**
     * @brief   Receiving data.
     */
    COBS_STATE_AWAITING_DATA
} cobs_state_t;

/**
 * @brief   The structure to keep track of the states and data through the
 *          COBS state machine.
 */
typedef struct _cobs_parser
{
    /**
     * @brief   Pointer to the buffer storing the data.
     */
    uint8_t *buffer;
    /**
     * @brief   The size of the buffer.
     */
    size_t buffer_size;
    /**
     * @brief   The current location in the buffer.
     */
    size_t buffer_count;
    /**
     * @brief   Buffer overrun counter.
     */
    uint64_t buffer_overrun;
    /**
     * @brief   The number of receive errors.
     */
    uint64_t rx_error;
    /**
     * @brief   The number of correctly received packets.
     */
    uint64_t rx_success;
    /**
     * @brief   Current state of the state machine.
     */
    cobs_state_t state;
    /**
     * @brief    Pointer to the parser to parse the data after a
     *           successful transfer.
     */
    void (*parser)(struct _cobs_parser *);
} cobs_parser_t;

/**
 * @brief   Structure to keep track of the code byte and index during encoding.
 */
typedef struct
{
    /**
     * @brief The current value of the COBS code byte.
     */
    uint8_t code;
    /**
     * @brief Index of the current code byte in the circular buffer.
     */
    size_t code_index;
} cobs_encoder_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

bool COBSEncode(const uint8_t *data,
                const size_t size,
                circular_buffer_t *cb,
                cobs_encoder_t *enc);
bool COBSEncode_MultiChunk(const uint8_t *ptr_list[],
                           const size_t length_list[],
                           const size_t size,
                           circular_buffer_t *cb,
                           cobs_encoder_t *enc);
void COBSInitParser(cobs_parser_t *p,
                    uint8_t *buffer,
                    const size_t buffer_size,
                    void (*parser)(cobs_parser_t *));
void COBSResetParser(cobs_parser_t *p);
void COBSDecode(const uint8_t data, cobs_parser_t *p);

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

inline size_t COBSGetMaxEncodedSize(const size_t size)
{
    /* The expansion of COBS is 1 byte per 208 bytes, plus the frame delimiter
     * added between packets.
     */
    return size + size / 208 + 2;
}

inline bool COBSisDiff2Zero(const uint8_t code)
{
    return (code >= COBS_Diff2Zero);
}

inline bool COBSisRunZero(const uint8_t code)
{
    return ((code >= COBS_RunZero) && (code <= COBS_RunZeroMax));
}

#endif
