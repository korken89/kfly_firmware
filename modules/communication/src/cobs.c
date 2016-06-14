/* *
 *
 * Consistent Overhead Byte Stuffing protocol implementation.
 *
 * */

#include "ch.h"
#include "hal.h"
#include "crc.h"
#include "circularbuffer.h"
#include "cobs.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/


/*===============================================================*/
/* Expansion of circular buffers required by the serial protocol */
/*===============================================================*/

/**
 * @brief               Finished a COBS packet in the buffer.
 *
 * @param[in/out] cb        Pointer to the circular buffer.
 * @param[in/out] count     Pointer to tracking variable for the buffer.
 */
static inline void CircularBuffer_COBSFinishPacket(circular_buffer_t *cb,
                                                   int32_t *count)
{
}

/**
 * @brief               Writes a chunk of data to the circular buffer and
 *                      encodes it with the COBS protocol on the fly.
 *
 * @param[out]    cb    Pointer to the circular buffer.
 * @param[in]     data  Pointer to the data to be written.
 * @param[in]     size  Size of the data.
 * @param[in/out] count Pointer to tracking variable for the buffer.
 */
static inline void CircularBuffer_COBSWriteChunkNoInc(circular_buffer_t *cb,
                                                      const uint8_t *data,
                                                      const size_t size,
                                                      int32_t *count)
{
}

/*================================*/
/* Generate message functionality */
/*================================*/

/**
 * @brief               Encodes a chunk of data with the COBS protocol.
 *
 * @param[in]     data  Pointer to the data to be encoded.
 * @param[in]     size  Size of the data.
 * @param[out]    cb    Pointer to the circular buffer where the data will be
 *                      stored.
 * @param[in/out] enc   Pointer to the COBS encoder structure.
 *
 * @return  Returns HAL_FAILED if the packet doesn't fit in the buffer.
 */
bool COBSEncode(const uint8_t *data,
                const size_t size,
                circular_buffer_t *cb,
                cobs_encoder_t *enc)
{
    int32_t count = 0;

    /* Check if the worst case will fit. */
    if (CircularBuffer_SpaceLeft(cb) < COBSGetMaxEncodedSize(size))
        return HAL_FAILED;

    /* Encode and send the data. */
    CircularBuffer_COBSWriteChunkNoInc(cb, data, size, &count);

    /* Finish the packet. */
    CircularBuffer_COBSFinishPacket(cb, &count);

    /* Increment the buffer pointer */
    return CircularBuffer_Increment(cb, count);
}

/**
 * @brief               Encodes multiple chunks of data from a list of pointers
 *                      with the full COBS protocol.
 *
 * @param[in]  ptr_list     Pointer to the list of pointers, pointing to data
 *                          chunks.
 * @param[in]  length_list  List of lengths for each chunk.
 * @param[in]  size         Size of the lists.
 * @param[out] cb           Pointer to the circular buffer where the data will
 *                          be stored.
 * @param[in/out] enc       Pointer to the COBS encoder structure.
 *
 * @return  Returns HAL_FAILED if the packet doesn't fit in the buffer.
 */
bool COBSEncode_MultiChunk(const uint8_t *ptr_list[],
                           const size_t length_list[],
                           const size_t list_size,
                           circular_buffer_t *cb,
                           cobs_encoder_t *enc)
{
    int32_t count = 0;
    uint32_t i, total_size = 0;

    for (i = 0; i < list_size; i++)
        total_size += length_list[i];

    /* Check if the worst case will fit. */
    if (CircularBuffer_SpaceLeft(cb) < COBSGetMaxEncodedSize(total_size))
        return HAL_FAILED;

    /* Encode and send the data. */
    for (i = 0; i < list_size; i++)
        CircularBuffer_COBSWriteChunkNoInc(cb, ptr_list[i],
                                           length_list[i], &count);

    /* Add stop byte. */
    CircularBuffer_COBSFinishPacket(cb, &count);

    /* Increment the buffer pointer */
    return CircularBuffer_Increment(cb, count);
}

/*============================*/
/* COBS parsing functionality */
/*============================*/

/**
 * @brief                       Initializes the cobs_parser_t structure.
 *
 * @param[in/out] p             Pointer to cobs_parser_t structure.
 * @param[in]     buffer        Buffer used for intermediate data.
 * @param[in]     buffer_size   Buffer size.
 * @param[in]     parser        Pointer to the parser of the passed data.
 */
void COBSInitParser(cobs_parser_t *p,
                    uint8_t *buffer,
                    const size_t buffer_size,
                    void (*parser)(cobs_parser_t *))
{
    p->buffer = buffer;
    p->buffer_size = buffer_size;
    p->buffer_count = 0;
    p->buffer_overrun = 0;
    p->rx_error = 0;
    p->rx_success = 0;
    p->state = COBS_STATE_AWAITING_CODE;
    p->parser = parser;
}

/**
 * @brief                       Resets the cobs_parser_t structure.
 *
 * @param[in/out] p             Pointer to cobs_parser_t structure.
 */
void COBSResetParser(cobs_parser_t *p)
{
    p->buffer_count = 0;
    p->buffer_overrun = 0;
    p->rx_error = 0;
    p->rx_success = 0;
    p->state = COBS_STATE_AWAITING_CODE;
}

/**
 * @brief              The entry point of serial data to the state machine.
 *                     When a finished COBS packet is detected the parser
 *                     function will be called to handle the next level of
 *                     parsing.
 *
 * @param[in] data     Input data to be parsed.
 * @param[in] p        Pointer to cobs_parser_t structure.
 */
void COBSDecode(const uint8_t data, cobs_parser_t *p)
{
    /* Parse the data based on the current state. */
    switch (p->state)
    {
    case COBS_STATE_AWAITING_DATA:


        break;

    case COBS_STATE_AWAITING_CODE:


        break;


    default:

        /* Should never come here. */
        p->state = COBS_STATE_AWAITING_CODE;
        p->rx_error++;

    }
}
