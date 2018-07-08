/*
 *******************************************************************************
 *  [curebuffer.h]
 *  This module is for FIFO buffer.
 *
 *  This program is under the terms of the GPLv3.
 *  https://www.gnu.org/licenses/gpl-3.0.html
 *
 *  Copyright(c) 2017 Keshikan (www.keshikan.net)
 *******************************************************************************
 */

#ifndef CUREBUFFER_H_
#define CUREBUFFER_H_

#include <stdint.h>

typedef enum{
	BUFFER_FAILURE,BUFFER_SUCCESS
}BUFFER_STATUS;

typedef struct{
	uint16_t idx_front;
	uint16_t idx_rear;
	uint16_t length;
	uint8_t  *buffer;
}RingBufferU8;

typedef struct{
	uint16_t idx_front;
	uint16_t idx_rear;
	uint16_t length;
	int16_t *buffer;
}RingBuffer16;

typedef struct{
	uint16_t idx_front;
	uint16_t idx_rear;
	uint16_t length;
	uint32_t *buffer;
}RingBuffer32;


/////////////////////////////
//methods for uint8_t FIFO.
/////////////////////////////
extern BUFFER_STATUS cureRingBufferU8Init(RingBufferU8 *rbuf, uint16_t buflen);
extern BUFFER_STATUS cureRingBufferU8Free(RingBufferU8 *rbuf);
extern BUFFER_STATUS cureRingBufferU8Enqueue(RingBufferU8 *rbuf, uint8_t *inputc);
extern BUFFER_STATUS cureRingBufferU8Dequeue(RingBufferU8 *rbuf, uint8_t *ret);

/////////////////////////////
//methods for int16_t FIFO.
/////////////////////////////
extern BUFFER_STATUS cureRingBuffer16Init(RingBuffer16 *rbuf, uint16_t buflen);
extern BUFFER_STATUS cureRingBuffer16Free(RingBuffer16 *rbuf);
extern BUFFER_STATUS cureRingBuffer16Enqueue(RingBuffer16 *rbuf, int16_t *inputc);
extern BUFFER_STATUS cureRingBuffer16EnqueueIgnoreErr(RingBuffer16 *rbuf, int16_t *inputc);
extern BUFFER_STATUS cureRingBuffer16Dequeue(RingBuffer16 *rbuf, int16_t *ret);
extern BUFFER_STATUS cureRingBuffer16GetElement(RingBuffer16 *rbuf, int16_t *ret, uint16_t delaynum, uint16_t length);

/////////////////////////////
//methods for uint32_t FIFO.
/////////////////////////////
extern BUFFER_STATUS cureRingBufferU32Init(RingBuffer32 *rbuf, uint16_t buflen);
extern BUFFER_STATUS cureRingBufferU32Free(RingBuffer32 *rbuf);
extern BUFFER_STATUS cureRingBufferU32Enqueue(RingBuffer32 *rbuf, uint32_t *inputc);
extern BUFFER_STATUS cureRingBufferU32EnqueueIgnoreErr(RingBuffer32 *rbuf, uint32_t *inputc);
extern BUFFER_STATUS cureRingBufferU32Dequeue(RingBuffer32 *rbuf, uint32_t *ret);
extern BUFFER_STATUS cureRingBufferU32GetElement(RingBuffer32 *rbuf, uint32_t *ret, uint16_t delaynum, uint16_t delay_buffer_length);


//debug
extern uint16_t _cureRingBufferU8GetUsedSize(RingBufferU8 *rbuf);

#endif /* CUREBUFFER_H_ */
