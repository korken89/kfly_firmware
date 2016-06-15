#ifndef __CIRCULARBUFFER_H
#define __CIRCULARBUFFER_H

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
    mutex_t write_lock;     /* Write lock mutex */
    /**
     * @brief   Position of the head of the buffer.
     */
    size_t head;          /* Newest element */
    /**
     * @brief   Position of the tail of the buffer.
     */
    size_t tail;          /* Oldest element */
    /**
     * @brief   Size of the circular buffer.
     */
    size_t size;          /* Size of buffer */
    /**
     * @brief   Pointer to the data holding region.
     */
    uint8_t *buffer;        /* Pointer to memory area */
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
static inline uint32_t CircularBuffer_SpaceLeft(circular_buffer_t *Cbuff)
{
    return (Cbuff->tail + Cbuff->size - Cbuff->head - 1) % Cbuff->size;
}

/**
 * @brief               Increases the tail pointer by count.
 *
 * @param[in/out] Cbuff Pointer to the circular buffer.
 * @param[in] count     Number to increase with.
 */
static inline void CircularBuffer_IncrementTail(circular_buffer_t *Cbuff,
                                                uint32_t count)
{
    Cbuff->tail = ((Cbuff->tail + count) % Cbuff->size);
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

void CircularBuffer_Init(circular_buffer_t *Cbuff,
                         uint8_t *buffer,
                         uint32_t buffer_size);
void CircularBuffer_InitMutex(circular_buffer_t *Cbuff);
void CircularBuffer_WriteSingle(circular_buffer_t *Cbuff, uint8_t data);
void CircularBuffer_WriteChunk(circular_buffer_t *Cbuff,
                               uint8_t *data,
                               const uint32_t count);
uint8_t CircularBuffer_ReadSingle(circular_buffer_t *Cbuff);
void CircularBuffer_ReadChunk(circular_buffer_t *Cbuff,
                              uint8_t *data,
                              uint32_t count);
bool CircularBuffer_Increment(circular_buffer_t *Cbuff, int32_t count);
uint8_t *CircularBuffer_GetReadPointer(circular_buffer_t *Cbuff,
                                       uint32_t *size);

#endif
