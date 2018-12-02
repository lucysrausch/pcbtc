/*
 *******************************************************************************
 *  [curebuffer.c]
 *  This module is for FIFO buffer.
 *
 *  This program is under the terms of the GPLv3.
 *  https://www.gnu.org/licenses/gpl-3.0.html
 *
 *  Copyright(c) 2017 Keshikan (www.keshikan.net)
 *******************************************************************************
 */

#include "curebuffer.h"
#include <stdint.h>
#include <stdlib.h>


/////////////////////////////
//methods for uint8_t FIFO.
/////////////////////////////

BUFFER_STATUS cureRingBufferU8Init(RingBufferU8 *rbuf, uint16_t buflen)
{

	uint32_t i;

	cureRingBufferU8Free(rbuf);

		rbuf->buffer = (uint8_t *)malloc( buflen * sizeof(uint8_t) );
		if(NULL == rbuf->buffer){
			return BUFFER_FAILURE;
		}
		for(i=0; i<buflen; i++){
			rbuf->buffer[i] = 0;
		}

	rbuf->length = buflen;

	return BUFFER_SUCCESS;
}

BUFFER_STATUS cureRingBufferU8Free(RingBufferU8 *rbuf)
{
	if(NULL != rbuf->buffer){
		free(rbuf->buffer);
	}

	rbuf->idx_front = rbuf->idx_rear = 0;
	rbuf->length = 0;

	return BUFFER_SUCCESS;
}

BUFFER_STATUS cureRingBufferU8Enqueue(RingBufferU8 *rbuf, uint8_t *inputc)
{
	if( ((rbuf->idx_front +1)&(rbuf->length -1)) == rbuf->idx_rear ){//buffer overrun error occurs.
		return BUFFER_FAILURE;
	}else{

		rbuf->buffer[rbuf->idx_front]=  *inputc;
		rbuf->idx_front++;
		rbuf->idx_front &= (rbuf->length -1);
		return BUFFER_SUCCESS;
	}
}

BUFFER_STATUS cureRingBufferU8Dequeue(RingBufferU8 *rbuf, uint8_t *ret)
{
	if(rbuf->idx_front == rbuf->idx_rear){//if buffer underrun error occurs.
		return BUFFER_FAILURE;
	}else{

		*ret = (rbuf->buffer[rbuf->idx_rear]);
		rbuf->idx_rear++;
		rbuf->idx_rear &= (rbuf->length -1);
		return BUFFER_SUCCESS;
	}
}

//debug
uint16_t _cureRingBufferU8GetUsedSize(RingBufferU8 *rbuf)
{
	if(rbuf->idx_front >= rbuf->idx_rear){
		return rbuf->idx_front - rbuf->idx_rear;
	}else{
		return rbuf->idx_front + rbuf->length - rbuf->idx_rear;
	}

}

/////////////////////////////
//methods for int16_t FIFO.
/////////////////////////////

BUFFER_STATUS cureRingBuffer16Init(RingBuffer16 *rbuf, uint16_t buflen)
{

	uint32_t i;

	cureRingBuffer16Free(rbuf);

		rbuf->buffer = (int16_t *)malloc( buflen * sizeof(int16_t) );
		if(NULL == rbuf->buffer){
			return BUFFER_FAILURE;
		}
		for(i=0; i<buflen; i++){
			rbuf->buffer[i] = 0;
		}
	rbuf->length = buflen;
	return BUFFER_SUCCESS;
}

BUFFER_STATUS cureRingBuffer16Free(RingBuffer16 *rbuf)
{
	if(NULL != rbuf->buffer){
		free(rbuf->buffer);
	}

	rbuf->idx_front = rbuf->idx_rear = 0;
	rbuf->length = 0;

	return BUFFER_SUCCESS;
}

BUFFER_STATUS cureRingBuffer16Enqueue(RingBuffer16 *rbuf, int16_t *inputc)
{
	if( ((rbuf->idx_front +1)&(rbuf->length -1)) == rbuf->idx_rear ){//buffer overrun error occurs.
		return BUFFER_FAILURE;
	}else{
		rbuf->buffer[rbuf->idx_front]=  *inputc;
		rbuf->idx_front++;
		rbuf->idx_front &= (rbuf->length -1);
		return BUFFER_SUCCESS;
	}
}

BUFFER_STATUS cureRingBuffer16EnqueueIgnoreErr(RingBuffer16 *rbuf, int16_t *inputc)
{

	rbuf->buffer[rbuf->idx_front]= *inputc;
	rbuf->idx_front++;
	rbuf->idx_front &= (rbuf->length -1);
	return BUFFER_SUCCESS;

}


BUFFER_STATUS cureRingBuffer16Dequeue(RingBuffer16 *rbuf, int16_t *ret)
{
	if(rbuf->idx_front == rbuf->idx_rear){//if buffer underrun error occurs.
		return BUFFER_FAILURE;
	}else{
		*ret = (rbuf->buffer[rbuf->idx_rear]);
		rbuf->idx_rear++;
		rbuf->idx_rear &= (rbuf->length -1);
		return BUFFER_SUCCESS;
	}
}


BUFFER_STATUS cureRingBuffer16GetElement(RingBuffer16 *rbuf, int16_t *ret, uint16_t delaynum, uint16_t delay_buffer_length)
{

	if(rbuf->idx_front >= delaynum){
		rbuf->idx_rear = rbuf->idx_front - delaynum;
	}else{
		rbuf->idx_rear = delay_buffer_length - (delaynum - rbuf->idx_front);
	}
		*ret = (rbuf->buffer[rbuf->idx_rear]);
		return BUFFER_SUCCESS;

}

/////////////////////////////
//methods for uint32_t FIFO.
/////////////////////////////

BUFFER_STATUS cureRingBufferU32Init(RingBuffer32 *rbuf, uint16_t buflen)
{

	uint32_t i;

	cureRingBufferU32Free(rbuf);

		rbuf->buffer = (uint32_t *)malloc( buflen * sizeof(uint32_t) );
		if(NULL == rbuf->buffer){
			return BUFFER_FAILURE;
		}
		for(i=0; i<buflen; i++){
			rbuf->buffer[i] = 0;
		}
	rbuf->length = buflen;
	return BUFFER_SUCCESS;
}

BUFFER_STATUS cureRingBufferU32Free(RingBuffer32 *rbuf)
{
	if(NULL != rbuf->buffer){
		free(rbuf->buffer);
	}

	rbuf->idx_front = rbuf->idx_rear = 0;
	rbuf->length = 0;

	return BUFFER_SUCCESS;
}

BUFFER_STATUS cureRingBufferU32Enqueue(RingBuffer32 *rbuf, uint32_t *inputc)
{
	if( ((rbuf->idx_front +1)&(rbuf->length -1)) == rbuf->idx_rear ){//buffer overrun error occurs.
		return BUFFER_FAILURE;
	}else{
		rbuf->buffer[rbuf->idx_front]=  *inputc;
		rbuf->idx_front++;
		rbuf->idx_front &= (rbuf->length -1);
		return BUFFER_SUCCESS;
	}
}

BUFFER_STATUS cureRingBufferU32EnqueueIgnoreErr(RingBuffer32 *rbuf, uint32_t *inputc)
{

	rbuf->buffer[rbuf->idx_front]= *inputc;
	rbuf->idx_front++;
	rbuf->idx_front &= (rbuf->length -1);
	return BUFFER_SUCCESS;

}


BUFFER_STATUS cureRingBufferU32Dequeue(RingBuffer32 *rbuf, uint32_t *ret)
{
	if(rbuf->idx_front == rbuf->idx_rear){//if buffer underrun error occurs.
		return BUFFER_FAILURE;
	}else{
		*ret = (rbuf->buffer[rbuf->idx_rear]);
		rbuf->idx_rear++;
		rbuf->idx_rear &= (rbuf->length -1);
		return BUFFER_SUCCESS;
	}
}


BUFFER_STATUS cureRingBufferU32GetElement(RingBuffer32 *rbuf, uint32_t *ret, uint16_t delaynum, uint16_t delay_buffer_length)
{
	uint16_t buf;


	if(rbuf->idx_front >= delaynum){
		buf = rbuf->idx_front - delaynum;
	}else{
		buf = delay_buffer_length - (delaynum - rbuf->idx_front);
	}
		*ret = (rbuf->buffer[buf]);
		return BUFFER_SUCCESS;

}


//BUFFER_STATUS cureRingBufferU32GetElement(RingBuffer32 *rbuf, uint32_t *ret, uint16_t delaynum, uint16_t delay_buffer_length)
//{
//
//
//	if(rbuf->idx_front >= delaynum){
//		rbuf->idx_rear = rbuf->idx_front - delaynum;
//	}else{
//		rbuf->idx_rear = delay_buffer_length - (delaynum - rbuf->idx_front);
//	}
//		*ret = (rbuf->buffer[rbuf->idx_rear]);
//		return BUFFER_SUCCESS;
//
//}
