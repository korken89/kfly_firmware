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

/**
 * @brief Convert from single-zero code to corresponding double-zero code.
 */
#define CONVERTZP                   (COBS_Diff2Zero - COBS_DiffZero)

/**
 * @brief Highest single-zero code with a corresponding double-zero code.
 */
#define MAXCONVERTIBLE              (COBS_Diff2ZeroMax - CONVERTZP)

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

static inline void FinishBlock(int32_t *count,
                               circular_buffer_t *cb,
                               cobs_encoder_t *enc)
{
    size_t index;

    /* Save final code and prepare for next block. */
    cb->buffer[enc->code_index] = enc->code;
    enc->code = COBS_DiffZero;

    /* Add dummy byte for the next code, or the frame delimiter if there are
     * no more data to be encoded. */
    index = (cb->head + *count) & cb->mask;

    enc->code_index = index;
    cb->buffer[index] = COBS_FrameDelimiter;
    *count += 1;
}

/**
 * @brief       Prepares the buffer and encoder for a new packet.
 *
 * @param[in/out] count     Pointer to tracking variable for the buffer.
 * @param[in/out] cb        Pointer to the circular buffer.
 * @param[in/out] enc       Pointer to the encoder data structure.
 */
static inline void StartPacket(int32_t *count,
                               circular_buffer_t *cb,
                               cobs_encoder_t *enc)
{
    size_t index = cb->head;

    /* Initialize the encoding buffer. */
    cb->buffer[index] = COBS_FrameDelimiter;
    enc->code = COBS_DiffZero;
    enc->code_index = index;

    *count += 1;
}
inline bool isDiff2Zero(const uint8_t code)
{
    return (code >= COBS_Diff2Zero);
}

inline bool isRunZero(const uint8_t code)
{
    return ((code >= COBS_RunZero) && (code <= COBS_RunZeroMax));
}

static inline bool DecodedSizeFitsBuffer(const size_t added_size,
                                         cobs_decoder_t *dec)
{
    /* Check if the data fits in the buffer, if not reset the decoder. */
    if (dec->buffer_count + added_size < dec->buffer_size)
    {
        return true;
    }
    else
    {
        dec->buffer_count = 0;
        dec->buffer_overrun++;
        dec->state = COBS_STATE_AWAITING_CODE;

        return false;
    }
}

/*===============================================================*/
/* Expansion of circular buffers required by the serial protocol */
/*===============================================================*/


/**
 * @brief               Writes a chunk of data to the circular buffer and
 *                      encodes it with the COBS protocol on the fly.
 *
 * @param[out]    cb    Pointer to the circular buffer.
 * @param[in]     data  Pointer to the data to be written.
 * @param[in]     size  Size of the data.
 * @param[in/out] count Pointer to tracking variable for the buffer.
 */
static inline void CircularBuffer_COBSWriteChunkNoInc(const uint8_t *data,
                                                      const size_t size,
                                                      int32_t *count,
                                                      circular_buffer_t *cb,
                                                      cobs_encoder_t *enc)
{
    size_t i;
    uint8_t ch;

    for (i = 0; i < size; i++)
    {
        ch = data[i];

        if (ch == 0)
        {
            /* If the data byte is zero apply the COBS encodng. */

            if (isRunZero(enc->code) && enc->code < COBS_RunZeroMax)
            {
                /* If already in ZRE mode, continue filling. */
                enc->code++;
            }
            else if (enc->code == COBS_Diff2Zero)
            {
                /* 2 zeros and no data, switch to ZRE. */
                enc->code = COBS_RunZero;
            }
            else if (enc->code <= MAXCONVERTIBLE)
            {
                /* In 1 zero and data is small enough to switch to ZPE. */
                enc->code += CONVERTZP;
            }
            else
            {
                /* Worst case scenario: Too much data for ZPE or too many
                 * zeros for ZRE. Finish the block and start over. */
                FinishBlock(count, cb, enc);
            }
        }
        else
        {
            /* If the data byte is non-zero, check the code and add the byte. */

            /* Note that the code is one step ahead here and needs to be
             * converted back one step. */
            if (isDiff2Zero(enc->code))
            {
                enc->code -= CONVERTZP;
                FinishBlock(count, cb, enc);
            }
            else if (enc->code == COBS_RunZero)
            {
                enc->code = COBS_Diff2Zero;
                FinishBlock(count, cb, enc);
            }
            else if (isRunZero(enc->code))
            {
                enc->code -= 1;
                FinishBlock(count, cb, enc);
            }

            /* Put the data in the buffer. */
            cb->buffer[(cb->head + *count) & cb->mask] = ch;
            *count += 1;

            if (++enc->code == COBS_Diff)
            {
                /* We will hit the max limit of codes, finish and start new. */
                FinishBlock(count, cb, enc);
            }
        }
    }
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

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
    else
    {
        /* Prepare the buffer for the packet. */
        StartPacket(&count, cb, enc);

        /* Encode and send the data. */
        CircularBuffer_COBSWriteChunkNoInc(data, size, &count, cb, enc);

        /* Finish the packet. */
        FinishBlock(&count, cb, enc);

        /* Increment the buffer pointer */
        return CircularBuffer_Increment(cb, count);
    }
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
    else
    {
        /* Prepare the buffer for the packet. */
        StartPacket(&count, cb, enc);

        /* Encode and send the data. */
        for (i = 0; i < list_size; i++)
            CircularBuffer_COBSWriteChunkNoInc(ptr_list[i], length_list[i],
                                               &count, cb, enc);

        /* Add stop byte. */
        FinishBlock(&count, cb, enc);

        /* Increment the buffer pointer */
        return CircularBuffer_Increment(cb, count);
    }
}

/*============================*/
/* COBS parsing functionality */
/*============================*/

/**
 * @brief                       Initializes the cobs_decoder_t structure.
 *
 * @param[in]     buffer        Buffer used for intermediate data.
 * @param[in]     buffer_size   Buffer size.
 * @param[in]     parser        Pointer to the parser of the passed data.
 * @param[in/out] p             Pointer to cobs_decoder_t structure.
 */
void COBSInitDecoder(uint8_t *buffer,
                     const size_t buffer_size,
                     void (*parser)(cobs_decoder_t *),
                     cobs_decoder_t *dec)
{
    dec->buffer = buffer;
    dec->buffer_size = buffer_size;
    dec->buffer_count = 0;
    dec->buffer_overrun = 0;
    dec->rx_error = 0;
    dec->rx_success = 0;
    dec->state = COBS_STATE_AWAITING_CODE;
    dec->parser = parser;
}

/**
 * @brief               Resets the cobs_decoder_t structure.
 *
 * @param[in/out] p     Pointer to cobs_decoder_t structure.
 */
void COBSResetDecoder(cobs_decoder_t *dec)
{
    dec->buffer_count = 0;
    dec->buffer_overrun = 0;
    dec->rx_error = 0;
    dec->rx_success = 0;
    dec->state = COBS_STATE_AWAITING_CODE;
}

/**
 * @brief              The entry point of serial data to the state machine.
 *                     When a finished COBS packet is detected the parser
 *                     function will be called to handle the next level of
 *                     parsing.
 *
 * @param[in] data     Input data to be parsed.
 * @param[in] dec      Pointer to cobs_decoder_t structure.
 */
void COBSDecode(const uint8_t in, cobs_decoder_t *dec)
{
    /* Check if the data fits the buffer. */
    DecodedSizeFitsBuffer(1, dec);

    /* Parse the data based on the current state. */
    if ((dec->state == COBS_STATE_AWAITING_CODE) && (in != 0))
    {
        /* Change state. */
        dec->state = COBS_STATE_AWAITING_DATA;

        if (in == COBS_Diff)
        {
            /* No zeros, all data. */
            dec->num_zeros = 0;
            dec->num_data = in - 1;
        }
        else if (isRunZero(in))
        {
            /* Extract the number of zeros from ZRE (and no data). */
            dec->num_zeros = in & 0x0f;
            dec->num_data = 0;
        }
        else if (isDiff2Zero(in))
        {
            /* Extract the number of data bytes and add 2 zeros (ZPE). */
            dec->num_zeros = 2;
            dec->num_data = in & 0x1f;
        }
        else
        {
            /* Extract the number of data bytes and add 1 zero (DiffZero). */
            dec->num_zeros = 1;
            dec->num_data = in - 1;
        }

        /* Check if there is no data, only zeros. */
        if (dec->num_data == 0)
        {
            if (!DecodedSizeFitsBuffer(dec->num_zeros, dec))
                return;

            /* Add all zeros. */
            while(dec->num_zeros-- > 0)
                dec->buffer[dec->buffer_count++] = 0;

            dec->state = COBS_STATE_AWAITING_CODE;
        }
    }
    else if ((dec->state == COBS_STATE_AWAITING_DATA) && (in != 0))
    {
        /* Decrease the data counter. */
        dec->num_data--;

        /* While there are data bytes left, add it to the output. */
        dec->buffer[dec->buffer_count++] = in;

        if (dec->num_data == 0)
        {
            if (!DecodedSizeFitsBuffer(dec->num_zeros, dec))
                return;

            /* Add all zeros. */
            while(dec->num_zeros-- > 0)
                dec->buffer[dec->buffer_count++] = 0;

            dec->state = COBS_STATE_AWAITING_CODE;
        }
    }
    else
    {
        /* Packet separator detected! */

        if (dec->state == COBS_STATE_AWAITING_CODE)
        {
            /* If there are data bytes in the buffer, remove the extra zero. */
            if (dec->buffer_count > 0)
            {
                dec->buffer_count--;

                /* RX successful! Add statistics and call the parser. */
                dec->rx_success++;

                if (dec->parser != NULL)
                    dec->parser(dec);
            }
            else
            {
                /* Packet without data detected, just a consecutive frame
                 * delimiter. */
                dec->rx_error++;
            }
        }
        else
        {
            /* Packet separator detected in data section: Error!
             * Increase error counter. */
            dec->rx_error++;
        }

        /* Reset state and buffer. */
        dec->state = COBS_STATE_AWAITING_CODE;
        dec->buffer_count = 0;
    }
}

