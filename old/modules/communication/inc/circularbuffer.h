#ifndef __CIRCULARBUFFER_H
#define __CIRCULARBUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Circular buffer holder definition.
 */
typedef struct
{
    /**
     * @brief   Write lock mutex.
     */
    mutex_t write_lock;
    /**
     * @brief   Position of the head of the buffer.
     */
    size_t head;
    /**
     * @brief   Position of the tail of the buffer.
     */
    size_t tail;
    /**
     * @brief   Size of the circular buffer.
     */
    size_t size;
    /**
     * @brief   Mask for wrapping the buffer on overflow.
     */
    size_t mask;
    /**
     * @brief   Pointer to the data holding region.
     */
    uint8_t *buffer;
} circular_buffer_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief               Claims a circular buffer using the mutex.
 *
 * @param[in] Cbuff     Pointer to the circular buffer.
 */
static inline void CircularBuffer_Claim(circular_buffer_t *Cbuff)
{
    chMtxLock(&Cbuff->write_lock);
}

/**
 * @brief               Releases a circular buffer using the mutex.
 *
 * @param[in] Cbuff     Pointer to the circular buffer.
 */
static inline void CircularBuffer_Release(circular_buffer_t *Cbuff)
{
    chMtxUnlock(&Cbuff->write_lock);
}

/**
 * @brief               Calculates the space left in a circular buffer.
 *
 * @param[in] Cbuff     Pointer to the circular buffer.
 */
static inline size_t CircularBuffer_SpaceLeft(circular_buffer_t *Cbuff)
{
    return (Cbuff->tail + Cbuff->size - Cbuff->head - 1) & Cbuff->mask;
}

/**
 * @brief               Increases the tail pointer by count.
 *
 * @param[in/out] Cbuff Pointer to the circular buffer.
 * @param[in] count     Number to increase with.
 */
static inline void CircularBuffer_IncrementTail(circular_buffer_t *Cbuff,
                                                size_t count)
{
    Cbuff->tail = ((Cbuff->tail + count) & Cbuff->mask);
}

/**
 * @brief               Writes a byte to a circular buffer.
 *
 * @param[in] Cbuff     Pointer to the circular buffer.
 */
static inline void CircularBuffer_WriteSingle(circular_buffer_t *Cbuff,
                                              uint8_t data)
{
    Cbuff->buffer[Cbuff->head] = data;
    Cbuff->head = ((Cbuff->head + 1) & Cbuff->mask);
}

/**
 * @brief               Reads a byte from a circular buffer.
 *
 * @param[in/out] Cbuff Pointer to the circular buffer.
 */
static inline uint8_t CircularBuffer_ReadSingle(circular_buffer_t *Cbuff)
{
    uint8_t data;

    data = Cbuff->buffer[Cbuff->tail];
    Cbuff->tail = ((Cbuff->tail + 1) & Cbuff->mask);

    return data;
}

/**
 * @brief               Increment the circular buffer pointer to math the
 *                      number of bytes written.
 *
 * @param[in/out] Cbuff Pointer to the circular buffer.
 * @param[in] count     Number of bytes to increment the pointer.
 */
static inline bool CircularBuffer_Increment(circular_buffer_t *Cbuff,
                                            const int32_t count)
{
    if (count == -1) /* Error! */
        return HAL_FAILED;

    else
    {
        Cbuff->head = ((Cbuff->head + count) & Cbuff->mask);
        return HAL_SUCCESS;
    }
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

void CircularBuffer_Init(circular_buffer_t *Cbuff,
                         uint8_t *buffer,
                         const size_t buffer_size);
void CircularBuffer_InitMutex(circular_buffer_t *Cbuff);
void CircularBuffer_WriteChunk(circular_buffer_t *Cbuff,
                               uint8_t *data,
                               const size_t count);
void CircularBuffer_ReadChunk(circular_buffer_t *Cbuff,
                              uint8_t *data,
                              const size_t count);
uint8_t *CircularBuffer_GetReadPointer(circular_buffer_t *Cbuff,
                                       size_t *size);

#endif
