#ifndef __CIRCULARBUFFER_H
#define __CIRCULARBUFFER_H

/* Defines */

/* Typedefs */
typedef struct
{
    mutex_t write_lock;     /* Write lock mutex */
    uint32_t head;          /* Newest element */
    uint32_t tail;          /* Oldest element */
    uint32_t size;          /* Size of buffer */
    uint8_t *buffer;        /* Pointer to memory area */
} Circular_Buffer_Type;

/* Global variable defines */

/* Global function defines */

void CircularBuffer_Init(Circular_Buffer_Type *Cbuff,
						 uint8_t *buffer, 
						 uint32_t buffer_size);
void CircularBuffer_InitMutex(Circular_Buffer_Type *Cbuff);
void CircularBuffer_Claim(Circular_Buffer_Type *Cbuff);
void CircularBuffer_Release(Circular_Buffer_Type *Cbuff);
uint32_t CircularBuffer_SpaceLeft(Circular_Buffer_Type *Cbuff);
void CircularBuffer_WriteSingle(Circular_Buffer_Type *Cbuff, uint8_t data);
void CircularBuffer_WriteChunk(Circular_Buffer_Type *Cbuff, 
							   uint8_t *data, 
							   const uint32_t count);
void CircularBuffer_WriteSYNCNoIncrement(Circular_Buffer_Type *Cbuff, 
										 int32_t *count, 
										 uint8_t *crc8, 
										 uint16_t *crc16);
void CircularBuffer_WriteNoIncrement(Circular_Buffer_Type *Cbuff,
                                     uint8_t data, 
                                     int32_t *count, 
                                     uint8_t *crc8, 
                                     uint16_t *crc16);
bool CircularBuffer_Increment(Circular_Buffer_Type *Cbuff, uint32_t count);
uint8_t CircularBuffer_ReadSingle(Circular_Buffer_Type *Cbuff);
void CircularBuffer_ReadChunk(Circular_Buffer_Type *Cbuff, 
							  uint8_t *data, 
							  uint32_t count);

#endif
