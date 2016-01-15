/* *
 *
 * Serial Line Internet Protocol support
 *
 * */

#include "ch.h"
#include "hal.h"
#include "crc.h"
#include "circularbuffer.h"
#include "slip.h"

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
 * @brief               Writes a byte to the circular buffer, if its vaule is
 *                      reserved - escape it.
 *
 * @param[in/out] cb    Pointer to the circular buffer.
 * @param[in/out] data  Byte being written.
 * @param[in/out] count Pointer to tracking variable for the buffer.
 */
static void CircularBuffer_SLIPWriteNoInc(circular_buffer_t *cb,
                                          uint8_t data,
                                          int32_t *count)
{
    /* Check if we have an error from previous write */
    if (*count >= 0)
    {
        /* Check if we have 2 bytes free, in case of data = reserved */
        if ((CircularBuffer_SpaceLeft(cb) - *count) >= 2)
        {
            cb->buffer[(cb->head + *count) % cb->size] = data;
            *count += 1;
        }
        else
            *count = -1;
    }
}

/**
 * @brief               Writes an SLIP_END to the buffer without encodeing it.
 *
 * @param[in/out] cb    Pointer to the circular buffer.
 * @param[in/out] count Pointer to tracking variable for the buffer.
 */
static void CircularBuffer_SLIPWriteENDNoInc(circular_buffer_t *cb,
                                             int32_t *count)
{
    /* Check if we have an error from previous write */
    if (*count >= 0)
    {
        /* Check if we have 2 bytes free, in case of data = reserved */
        if ((CircularBuffer_SpaceLeft(cb) - *count) >= 1)
        {
            cb->buffer[(cb->head + *count) % cb->size] = SLIP_END;
            *count += 1;
        }
        else
            *count = -1;
    }
}

/**
 * @brief               Writes a chunk of data to the circular buffer and
 *                      encodes it with the SLIP protocol on the fly.
 *
 * @param[out]    cb    Pointer to the circular buffer.
 * @param[in]     data  Pointer to the data to be written.
 * @param[in]     size  Size of the data.
 * @param[in/out] count Pointer to tracking variable for the buffer.
 */
static void CircularBuffer_SLIPWriteChunkNoInc(circular_buffer_t *cb,
                                               uint8_t *data,
                                               const uint32_t size,
                                               int32_t *count)
{
    uint32_t i, p_sp;
    const uint32_t space = CircularBuffer_SpaceLeft(cb);

    /* Check if we have an error from previous write */
    if (*count >= 0)
    {
        /* Check if we have 2 times the data packet size free, then we can
         * encode the entire buffer directly without worries. */
        if ((space - *count + 1) / 2 >= size)
        {
            for (i = 0; i < size; i++)
            {
                if ((data[i] != SLIP_END) && (data[i] != SLIP_ESC))
                {
                    cb->buffer[(cb->head + *count) % cb->size] = data[i];
                    *count += 1;
                }
                else if (data[i] == SLIP_END)
                {
                    cb->buffer[(cb->head + *count) % cb->size] = SLIP_ESC;
                    *count += 1;
                    cb->buffer[(cb->head + *count) % cb->size] = SLIP_ESC_END;
                    *count += 1;
                }
                else
                {
                    cb->buffer[(cb->head + *count) % cb->size] = SLIP_ESC;
                    *count += 1;
                    cb->buffer[(cb->head + *count) % cb->size] = SLIP_ESC_ESC;
                    *count += 1;
                }
            }
        }

        /* Worst case buffer will not fit, start writing and hope there are not
         * many special cases as the "no special case" is guaranteed to fit. */
        else
        {
            /* Keep the pointer count. */
            p_sp = 0;
            i = 0;

            /* Loop while there is space left and there is data left. */
            while((p_sp < size) && (i < space))
            {
                if ((data[p_sp] != SLIP_END) && (data[p_sp] != SLIP_ESC))
                {
                    cb->buffer[(cb->head + *count) % cb->size] = data[p_sp];
                    *count += 1;
                }
                else if (data[p_sp] == SLIP_END)
                {
                    cb->buffer[(cb->head + *count) % cb->size] = SLIP_ESC;
                    *count += 1;
                    cb->buffer[(cb->head + *count) % cb->size] = SLIP_ESC_END;
                    *count += 1;
                    i++;
                }
                else
                {
                    cb->buffer[(cb->head + *count) % cb->size] = SLIP_ESC;
                    *count += 1;
                    cb->buffer[(cb->head + *count) % cb->size] = SLIP_ESC_ESC;
                    *count += 1;
                    i++;
                }

                /* Increment the buffer index. */
                p_sp++;
            }

            /* Check if the operation failed. */
            if (p_sp < size)
            {
                *count = -1;
            }
        }
    }
}

/*================================*/
/* Generate message functionality */
/*================================*/

/**
 * @brief               Encodes a chunk of data with the full SLIP protocol.
 *
 * @param[in]     data  Pointer to the data to be encoded.
 * @param[in]     size  Size of the data.
 * @param[out]    cb    Pointer to the circular buffer where the data will be
 *                      stored.
 */
bool GenerateSLIP(uint8_t *data,
                  const uint32_t size,
                  circular_buffer_t *cb)
{
    int32_t count = 0;

    /* Check if the "best case" will fit. */
    if (CircularBuffer_SpaceLeft(cb) < (size + 2))
        return HAL_FAILED;

    /* Add start byte. */
    CircularBuffer_SLIPWriteENDNoInc(cb, &count);

    /* Encode and send the data. */
    CircularBuffer_SLIPWriteChunkNoInc(cb, data, size, &count);

    /* Add stop byte. */
    CircularBuffer_SLIPWriteENDNoInc(cb, &count);

    /* Increment the buffer pointer */
    return CircularBuffer_Increment(cb, count);
}

/**
 * @brief               Encodes multiple chunks of data with the full SLIP
 *                      protocol. Assumes a header, body and tail from of the
 *                      data.
 *
 * @param[in]   head    Pointer to the data to be encoded.
 * @param[in]   h_size  Size of the data.
 * @param[in]   body    Pointer to the data to be encoded.
 * @param[in]   b_size  Size of the data.
 * @param[in]   tail    Pointer to the data to be encoded.
 * @param[in]   t_size  Size of the data.
 * @param[out]  cb      Pointer to the circular buffer where the data will be
 *                      stored.
 */
bool GenerateSLIP_HBT(uint8_t *head,
                      const uint32_t h_size,
                      uint8_t *body,
                      const uint32_t b_size,
                      uint8_t *tail,
                      const uint32_t t_size,
                      circular_buffer_t *cb)
{
    int32_t count = 0;

    /* Check if the "best case" will fit. */
    if (CircularBuffer_SpaceLeft(cb) < (h_size + b_size + t_size + 2))
        return HAL_FAILED;

    /* Add start byte. */
    CircularBuffer_SLIPWriteENDNoInc(cb, &count);

    /* Encode and send the data. */
    if (head != NULL)
        CircularBuffer_SLIPWriteChunkNoInc(cb, head, h_size, &count);

    if (body != NULL)
        CircularBuffer_SLIPWriteChunkNoInc(cb, body, b_size, &count);

    if (tail != NULL)
        CircularBuffer_SLIPWriteChunkNoInc(cb, tail, t_size, &count);

    /* Add stop byte. */
    CircularBuffer_SLIPWriteENDNoInc(cb, &count);

    /* Increment the buffer pointer */
    return CircularBuffer_Increment(cb, count);
}

/*============================*/
/* SLIP parsing functionality */
/*============================*/


