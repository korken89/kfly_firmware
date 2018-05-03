#ifndef __COBS_H
#define __COBS_H

#include "circularbuffer.h"
#include "communication_defs.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/


/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

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
typedef struct _cobs_decoder
{
    /**
     * @brief   Holds the generic decoding information.
     */
    communication_decoder_t generic_decoder;
    /**
     * @brief   Current state of the state machine.
     */
    cobs_state_t state;
    /**
     * @brief   Number of zeros to add to the output.
     */
    size_t num_zeros;
    /**
     * @brief   Number of data bytes to add to the output.
     */
    size_t num_data;
} cobs_decoder_t;

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
void COBSInitDecoder(uint8_t *buffer,
                     const size_t buffer_size,
                     void (*parser)(communication_decoder_t *),
                     cobs_decoder_t *p);
void COBSResetDecoder(cobs_decoder_t *p);
void COBSDecode(const uint8_t data, cobs_decoder_t *p);

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

static inline size_t COBSGetMaxEncodedSize(const size_t size)
{
    /* The expansion of COBS is 1 byte per 208 bytes, plus the frame delimiter
     * added between packets.
     */
    return size + size / 208 + 2;
}

#endif
