#ifndef __COMMUNICATION_DEFS_H
#define __COMMUNICATION_DEFS_H

#include <stdint.h>
#include <stddef.h>

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   The generic structure to keep track of the data through the
 *          decoding flow.
 */
typedef struct _communication_decoder
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
     * @brief    Pointer to the parser to parse the data after a
     *           successful transfer.
     */
    void (*parser)(struct _communication_decoder *);
} communication_decoder_t;

/**
 * @brief   The generic parser definition.
 */
typedef void (*generic_parser_t)(communication_decoder_t *);

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

static inline void GenericDecoderInit(uint8_t *buffer,
                                      const size_t buffer_size,
                                      generic_parser_t parser,
                                      communication_decoder_t *dec)
{
    dec->buffer = buffer;
    dec->buffer_size = buffer_size;
    dec->buffer_count = 0;
    dec->buffer_overrun = 0;
    dec->rx_error = 0;
    dec->rx_success = 0;
    dec->parser = parser;
}

static inline void GenericDecoderReset(communication_decoder_t *dec)
{
    dec->buffer_count = 0;
    dec->buffer_overrun = 0;
    dec->rx_error = 0;
    dec->rx_success = 0;
}

#endif
