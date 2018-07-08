/*
 *******************************************************************************
 *  [curemisc.h]
 *
 *  This program is under the terms of the GPLv3.
 *  https://www.gnu.org/licenses/gpl-3.0.html
 *
 *  Copyright(c) 2017 Keshikan (www.keshikan.net)
 *******************************************************************************
 */

#ifndef _CUREMISC_H_
#define _CUREMISC_H_


#include <stdbool.h>
#include <stdint.h>


#ifndef BOOL
	#define BOOL bool
	#define TRUE true
	#define FALSE false
#endif


//LO(HI)WORD: return lower(higher) 16bit of 32bit
//LO(HI)BYTE: return lower(higher) 4bit of 8bit
#ifndef LOWORD
	#define LOWORD(n) ( (uint16_t)(n) )
#endif

#ifndef HIWORD
	#define HIWORD(n) ( (uint16_t)(((uint32_t)(n) >> 16) & 0xFFFF) )
#endif

#ifndef LOBYTE
	#define LOBYTE(n) ( ((uint8_t)(n)) & 0x0F )
#endif

#ifndef HIBYTE
	#define HIBYTE(n) ( (uint8_t)(((n) >> 4) & 0x0F) )
#endif

typedef enum{
	FUNC_ERROR,FUNC_SUCCESS
}FUNC_STATUS;

#endif
