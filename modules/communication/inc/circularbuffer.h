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
    uint32_t head;          /* Newest element */
	/**
	 * @brief   Position of the tail of the buffer.
	 */
    uint32_t tail;          /* Oldest element */
	/**
	 * @brief   Size of the circular buffer.
	 */
    uint32_t size;          /* Size of buffer */
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

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

void CircularBuffer_Init(circular_buffer_t *Cbuff,
						 uint8_t *buffer, 
						 uint32_t buffer_size);
void CircularBuffer_InitMutex(circular_buffer_t *Cbuff);
void CircularBuffer_Claim(circular_buffer_t *Cbuff);
void CircularBuffer_Release(circular_buffer_t *Cbuff);
uint32_t CircularBuffer_SpaceLeft(circular_buffer_t *Cbuff);
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
void CircularBuffer_IncrementTail(circular_buffer_t *Cbuff, int32_t count);
#endif
