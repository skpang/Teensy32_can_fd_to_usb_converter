
#ifndef FRONTEND_H_
#define FRONTEND_H_


#include <Arduino.h>
#include <string.h>

#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" { 
#endif

#define BELL 7
#define CR 13
#define LR 10

#define RX_STEP_TYPE 0
#define RX_STEP_ID_EXT 1
#define RX_STEP_ID_STD 6
#define RX_STEP_DLC 9
#define RX_STEP_DATA 10
//#define RX_STEP_TIMESTAMP 26
#define RX_STEP_TIMESTAMP 138
#define RX_STEP_CR 30
#define RX_STEP_FINISHED 0xff

#define STATE_CONFIG 0
#define STATE_OPEN 1
#define STATE_LISTEN 2

#define VERSION_HARDWARE_MAJOR 1
#define VERSION_HARDWARE_MINOR 0
#define VERSION_FIRMWARE_MAJOR 1
#define VERSION_FIRMWARE_MINOR 0

void out_usb(void);
void sendByteHex(unsigned char value);
unsigned char transmitStd(char *line);
void parseLine(char * line);
unsigned char canmsg2ascii_getNextChar(uint32_t id, unsigned char dlc,unsigned char * step);



#ifdef  __cplusplus
}
#endif

#endif /* FRONTEND_H_ */
