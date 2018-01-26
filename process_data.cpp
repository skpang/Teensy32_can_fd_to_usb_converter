#include "process_data.h"
#include "drv_canfdspi_defines.h"
#include "drv_canfdspi_api.h"


unsigned char timestamping = 0;
CAN_BITTIME_SETUP bt;
extern CAN_TX_MSGOBJ txObj;
extern CAN_RX_MSGOBJ rxObj;
extern uint8_t txd[MAX_DATA_BYTES];
extern uint8_t rxd[MAX_DATA_BYTES];
extern uint8_t prompt;
extern volatile uint8_t state;

#define DRV_CANFDSPI_INDEX_0         0

unsigned char parseHex(char * line, unsigned char len, unsigned long * value) 
{
    *value = 0;
    while (len--) {
        if (*line == 0) return 0;
        *value <<= 4;
        if ((*line >= '0') && (*line <= '9')) {
           *value += *line - '0';
        } else if ((*line >= 'A') && (*line <= 'F')) {
           *value += *line - 'A' + 10;
        } else if ((*line >= 'a') && (*line <= 'f')) {
           *value += *line - 'a' + 10;
        } else return 0;
        line++;
    }
    return 1;
}


void sendByteHex(unsigned char value) 
{

    unsigned char ch = value >> 4;
    if (ch > 9) ch = ch - 10 + 'A';
    else ch = ch + '0';
  
    Serial.print(ch,BYTE);
   
    ch = value & 0xF;
    if (ch > 9) ch = ch - 10 + 'A';
    else ch = ch + '0';
    
    Serial.print(ch,BYTE);
}


unsigned char transmitStd(char *line) 
{

    uint32_t temp;
    unsigned char idlen;
    unsigned char i;
    unsigned char length;

    switch(line[0])
    {
      case 'D':
      case 'd':
        txObj.bF.ctrl.BRS = 1;
        txObj.bF.ctrl.FDF = 1;
        break;
      case 'B':
      case 'b':
        txObj.bF.ctrl.BRS = 0;
        txObj.bF.ctrl.FDF = 1;
        break;
      default:
        txObj.bF.ctrl.BRS = 0;
        txObj.bF.ctrl.FDF = 0;
        break;

    }
   
    if (line[0] < 'Z') {
      txObj.bF.ctrl.IDE = 1;
        idlen = 8;
    } else {
      txObj.bF.ctrl.IDE = 0;
        idlen = 3;
    }

    if (!parseHex(&line[1], idlen, &temp)) return 0;


    if (line[0] < 'Z') {
        txObj.bF.id.SID = (uint32_t)(0xffff0000 & temp) >> 18;
        txObj.bF.id.EID = 0x0000ffff & temp;
     } else {
       txObj.bF.id.SID = temp;
     }


    if (!parseHex(&line[1 + idlen], 1, &temp)) return 0;
    txObj.bF.ctrl.DLC = temp;
    length = temp;

    if(txObj.bF.ctrl.FDF == 0)
    {
      // Classic CAN
      if (length > 8) length = 8;
        for (i = 0; i < length; i++) {
            if (!parseHex(&line[idlen + 2 + i*2], 2, &temp)) return 0;
            txd[i] = temp;
        }
    }else
    {
      // CAN FD
      length = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC) txObj.bF.ctrl.DLC);
      if (length > 64) length = 64;
            for (i = 0; i < length; i++) {
                if (!parseHex(&line[idlen + 2 + i*2], 2, &temp)) return 0;
                txd[i] = temp;
            }

    }
    APP_TransmitMessageQueue();
    return 1;

}


void parseLine(char * line) 
{

    unsigned char result = BELL;
   
    switch (line[0]) {
        case 'S': // Setup with standard CAN bitrates
            if (state == STATE_CONFIG)
            {
                switch (line[1]) {
               //     case '0': ;  result = CR; break;
               //     case '1': ;  result = CR; break;
               //     case '2': ;  result = CR; break;
               //     case '3': ;  result = CR; break;
                      case '4':  APP_CANFDSPI_Init(CAN_125K_500K); result = CR; break;
                      case '5':  APP_CANFDSPI_Init(CAN_250K_500K); result = CR; break;
                      case '6':  APP_CANFDSPI_Init(CAN_500K_1M); result = CR; break;
              //      case '7':  ; result = CR; break;
                      case '8':  APP_CANFDSPI_Init(CAN_1000K_4M); result = CR; break;
                      case '9':  APP_CANFDSPI_Init(CAN_500K_1M); result = CR; break;
                      case 'A':  APP_CANFDSPI_Init(CAN_500K_2M); result = CR; break;
                      case 'B':  APP_CANFDSPI_Init(CAN_500K_3M); result = CR; break;
                      case 'C':  APP_CANFDSPI_Init(CAN_500K_4M); result = CR; break;
                      case 'D':  APP_CANFDSPI_Init(CAN_500K_5M); result = CR; break;
                      case 'E':  APP_CANFDSPI_Init(CAN_500K_8M); result = CR; break;
                      case 'F':  APP_CANFDSPI_Init(CAN_500K_10M); result = CR; break;
                      case 'G':  APP_CANFDSPI_Init(CAN_250K_500K); result = CR; break;
                      case 'H':  APP_CANFDSPI_Init(CAN_250K_1M); result = CR; break;
                      case 'I':  APP_CANFDSPI_Init(CAN_250K_2M); result = CR; break;
                      case 'J':  APP_CANFDSPI_Init(CAN_250K_3M); result = CR; break;
                      case 'K':  APP_CANFDSPI_Init(CAN_250K_4M); result = CR; break;
                      case 'L':  APP_CANFDSPI_Init(CAN_1000K_4M); result = CR; break;
                      case 'M':  APP_CANFDSPI_Init(CAN_1000K_8M); result = CR; break;
                      case 'N':  APP_CANFDSPI_Init(CAN_125K_500K); result = CR; break;

                }

            }
            break;
        case 's': 
            if (state == STATE_CONFIG)
            {
                unsigned long cnf1, cnf2, cnf3;
                if (parseHex(&line[1], 2, &cnf1) && parseHex(&line[3], 2, &cnf2) && parseHex(&line[5], 2, &cnf3)) {
                    result = CR;
                }
            }
            break;
        case 'G': 
            {
                unsigned long address;
                if (parseHex(&line[1], 2, &address)) {
                  //  
                  //  sendByteHex(value);
                    result = CR;
                }
            }
            break;
        case 'W': 
            {
                unsigned long address, data;
                if (parseHex(&line[1], 2, &address) && parseHex(&line[3], 2, &data)) {
                  
                    result = CR;
                }

            }
            break;
        case 'V': // Get hardware version
            {

                Serial.print('V');
                sendByteHex(VERSION_HARDWARE_MAJOR);
                sendByteHex(VERSION_HARDWARE_MINOR);
                result = CR;
            }
            break;
        case 'v': // Get firmware version
            {

                Serial.print('v');;
                sendByteHex(VERSION_FIRMWARE_MAJOR);
                sendByteHex(VERSION_FIRMWARE_MINOR);
                result = CR;
            }
            break;
        case 'N': // Get serial number
            {
             
                result = CR;
            }
            break;
        case 'O': // Open CAN channel
            
            if (state == STATE_CONFIG)
            {
           
               DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, CAN_NORMAL_MODE);
                   
                state = STATE_OPEN;
                result = CR;
            }
            break;
        case 'l': // Loop-back mode
            if (state == STATE_CONFIG)
            {
              
                state = STATE_OPEN;
                result = CR;
            }
            break;
        case 'L': // Open CAN channel in listen-only mode
            if (state == STATE_CONFIG)
            {
             

                state = STATE_LISTEN;
                result = CR;
            }
            break;
        case 'C': // Close CAN channel
            if (state != STATE_CONFIG)
            {
            
              DRV_CANFDSPI_OperationModeSelect(0, CAN_CONFIGURATION_MODE);
              state = STATE_CONFIG;
              result = CR;
            }
            break;
        case 'r': // Transmit standard RTR (11 bit) frame
        case 'R': // Transmit extended RTR (29 bit) frame
        case 't': // Transmit standard (11 bit) frame
        case 'T': // Transmit extended (29 bit) frame
        case 'd': // Transmit FD standard (11 bit) frame with BRS
        case 'D': // Transmit FD extended (29 bit) frame with BRS
        case 'B': // Transmit FD standard (11 bit) frame no BRS
        case 'b': // Transmit FD extended (29 bit) frame no BRS
      
            if (state == STATE_OPEN)
            {
                if (transmitStd(line)) {
                Serial.print('z');
                result = CR;
                }

            }
            break;
        case 'F': // Read status flags
            {
                unsigned char status = 0;
                sendByteHex(status);
                result = CR;
                
            }
            break;
         case 'Z': // Set time stamping
            {
                unsigned long stamping;
                if (parseHex(&line[1], 1, &stamping)) {
                    timestamping = (stamping != 0);
                    result = CR;
                }
            }
            break;
         case 'm': // Set accpetance filter mask
            if (state == STATE_CONFIG)
            {
                unsigned long am0, am1, am2, am3;
          
                    result = CR;
              
            }
            break;
         case 'M': // Set accpetance filter code
            if (state == STATE_CONFIG)
            {
                unsigned long ac0, ac1, ac2, ac3;
                result = CR;
              
            }
            break;

    }
   
   Serial.print(result,BYTE); 
}

unsigned char canmsg2ascii_getNextChar(uint32_t id, unsigned char dlc, unsigned char * step) {

    char ch = BELL;
    char newstep = *step;

    if (*step == RX_STEP_TYPE) {

        // type 1st char
        if ((rxObj.bF.ctrl.IDE == 1) && (rxObj.bF.ctrl.FDF == 1) && (rxObj.bF.ctrl.BRS == 1))
        {
           newstep = RX_STEP_ID_EXT;
           ch = 'D';
        } else if ((rxObj.bF.ctrl.IDE == 0) && (rxObj.bF.ctrl.FDF == 1)&& (rxObj.bF.ctrl.BRS == 1))
        {
            newstep = RX_STEP_ID_STD;
            ch = 'd';
        } else if ((rxObj.bF.ctrl.IDE == 1) && (rxObj.bF.ctrl.FDF == 0))
        {
            newstep = RX_STEP_ID_EXT;
            ch = 'T';
        }else if ((rxObj.bF.ctrl.IDE == 0) && (rxObj.bF.ctrl.FDF == 0))
        {
            newstep = RX_STEP_ID_STD;
            ch = 't';
        }else if ((rxObj.bF.ctrl.IDE == 1) && (rxObj.bF.ctrl.FDF == 1)&& (rxObj.bF.ctrl.BRS == 0))
        {
            newstep = RX_STEP_ID_EXT;
            ch = 'B';
        }else if ((rxObj.bF.ctrl.IDE == 0) && (rxObj.bF.ctrl.FDF == 1)&& (rxObj.bF.ctrl.BRS == 0))
        {
            newstep = RX_STEP_ID_STD;
            ch = 'b';
        }


    } else if (*step < RX_STEP_DLC) {

        unsigned char i = *step - 1;
        unsigned char * id_bp = (unsigned char*)&id; // rxObj.bF.id.SID;
        ch = id_bp[3 - (i / 2)];
        if ((i % 2) == 0) ch = ch >> 4;

        ch = ch & 0xF;
        if (ch > 9) ch = ch - 10 + 'A';
        else ch = ch + '0';

        newstep++;

    } else if (*step < RX_STEP_DATA) {

        ch = rxObj.bF.ctrl.DLC;
        ch = ch & 0xF;
        if (ch > 9) ch = ch - 10 + 'A';
        else ch = ch + '0';

        if (dlc ==0) newstep = RX_STEP_TIMESTAMP;
        else newstep++;

    } else if (*step < RX_STEP_TIMESTAMP) {

        unsigned char i = *step - RX_STEP_DATA;

        ch = rxd[i/2];
        if ((i % 2) == 0) ch = ch >> 4;

        ch = ch & 0xF;
        if (ch > 9) ch = ch - 10 + 'A';
        else ch = ch + '0';

        newstep++;
        if (newstep - RX_STEP_DATA ==dlc*2) newstep = RX_STEP_TIMESTAMP;

    } else if (timestamping && (*step < RX_STEP_CR)) {

        unsigned char i = *step - RX_STEP_TIMESTAMP;

        if ((i % 2) == 0) ch = ch >> 4;

        ch = ch & 0xF;
        if (ch > 9) ch = ch - 10 + 'A';
        else ch = ch + '0';

        newstep++;

    } else {

        ch = CR;
        newstep = RX_STEP_FINISHED;
    }

    *step = newstep;
    return ch;
}

void out_usb(void)
{

  uint8_t dlc,i,rxstep;
  uint8_t out_buff[200];
  uint8_t outdata = 0;
  uint32_t id = 0;

  i =0;
  rxstep = 0;

  dlc = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC)rxObj.bF.ctrl.DLC);

  if(rxObj.bF.ctrl.IDE == 1)
  {
      id = (((uint32_t)rxObj.bF.id.SID) <<18) |rxObj.bF.id.EID;
  }else
  {
      id = rxObj.bF.id.SID;
  }

  while( rxstep != RX_STEP_FINISHED)
  {
    outdata = canmsg2ascii_getNextChar(id, dlc, &rxstep);
    out_buff[i++] =outdata;
  }
  
  out_buff[i] = 0;  //Add null

  Serial.print((char *)out_buff);
}



