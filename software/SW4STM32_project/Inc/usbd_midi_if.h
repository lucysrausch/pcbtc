/**
  ******************************************************************************
  * @file           : usbd_midi_if.h
  * @brief          : Header for usbd_midi_if file.
  ******************************************************************************
*/

#ifndef __USBD_MIDI_IF_H
#define __USBD_MIDI_IF_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "usbd_midi.h"
#include "usbd_desc.h"
#include "curemisc.h"

#include "curebuffer.h"

#define MIDI_BUFFER_LENGTH (1024)
#define MIDI_SENDDATA_MAX (64)

////public typedef////
typedef enum{
	START_ANALYSIS,    // Initial Status, including exception.
	WAIT_DATA1,        // Waiting data byte(1st byte)
	WAIT_DATA2,        // Waiting data byte(2nd byte)
	WAIT_SYSTEM_DATA,  // Waiting data byte(system exclusive)
	END_ANALYSIS       // Analysis is ended.
}AnalysisStatus;

typedef enum{
	MSG_NOTHING,    // Exception(can't resolved, missing data, etc.)
	MSG_SYSEX,      // System Exclusive message
	MSG_ONE_BYTE,
	MSG_TWO_BYTE,
	MSG_THREE_BYTE,
}EventType;

typedef struct{
	uint8_t length;
	uint8_t midi_byte[MIDI_SENDDATA_MAX]; //data_byte[0]=MSB, [1]=LSB, [2]=OTHER...(e.g. sysEx, Control Change...)
}MIDIEvent;

typedef struct{
	AnalysisStatus stat;
	EventType type;
	bool is_system_common;
	uint8_t data_idx;
}MidiAnalysisStatus;

extern USBD_MIDI_ItfTypeDef  USBD_Interface_fops_FS;

//for cure series
extern FUNC_STATUS midiInit();//call before use functions in this files.
extern FUNC_STATUS midiGetFromUsbRx(uint8_t ch, uint8_t* dat);
extern FUNC_STATUS midiGetFromJackRx(uint8_t cable_num);
extern FUNC_STATUS midiSetFromJackRx(uint8_t cable_num, uint8_t* dat);
extern bool isUsbRxBufEmpty(uint8_t ch);
extern bool isJackRxBufEmpty(uint8_t ch);
extern bool isRxBufEmpty();

//USB function
extern void sendMidiMessage(uint8_t *msg, uint16_t size);
extern uint8_t USBD_MIDI_SendData (USBD_HandleTypeDef *pdev, uint8_t *pBuf, uint16_t length);


// Call in main loop
extern void midiProcess(void);
extern void USBD_MIDI_SendPacket(void);



#ifdef __cplusplus
}
#endif
  
#endif /* __USBD_MIDI_IF_H */
