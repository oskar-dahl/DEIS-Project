// By Oskar Dahl & Fredrik Johansson

#include <Thread.h>
#include <ThreadController.h>
#include <BasicLinearAlgebra.h>

/*

"""
PROTOCOL
  ----------------
  0 - Start byte
    0x40 (@) = START
  1 - Sender/Receiver
    0x01 = ARDUINO
    0x02 = Pi/PC
  2 - Command/Info
    0x00 = START COMM
    0x01 = ACK
    0x02 = NACK
    0x03 = SET SPEED // Set speed in next byte (3 - Speed)
    0x04 = ENCODER // Indicate encoder transmit. Used on Arduino side, otherwise 0
    0x05 = CHANGE LINE
    0x06 = FOLLOW LINE
  3 - Speed
    0x00 - 0xFF = SPEED //Only set on Pi side, otherwise 0
  4 - Encoder
    0x00 - 0xFF = Dr1
    0x00 - 0xFF = Dr2
    0x00 - 0xFF = Dr3
    0x00 - 0xFF = Dr4
  
    0x00 - 0xFF = Dl1
    0x00 - 0xFF = Dl2
    0x00 - 0xFF = Dl3
    0x00 - 0xFF = Dl4
  
  5 - END
    0x23 (#) = END BYTE

    //===================
    0 - START BYTE
    1 - Sender/Receiver
    2 - COMMAND/INFO
    3 - Speed //Used on Pi side, otherwise 0x00
    4 - POS //Used on Arduino side, otherwise 0x00
    5 - END BYTE
"""

*/

void clean_buffer(){
  for(uint16_t i = 0; i < BUFFER_MAX_LEN; i++){
    buffer[i] = 0;
  }
  buffer_index = 0;
}// void clean_buffer()

uint8_t * createProtPOS(){

  uint8_t *dDrBytes = rightshifty32((uint8_t)dDr);
  uint8_t *dDlBytes = rightshifty32((uint8_t)dDl);

  static uint8_t prot[]  = {(uint8_t)'@', 0x01, 0x07, 0x00, dDrBytes[0], dDrBytes[1], dDrBytes[2], dDlBytes[0], dDlBytes[1], dDlBytes[2], (uint8_t)'#'};

  return prot;
}

uint8_t * createProtACK(){

  static uint8_t prot[]  = {(uint8_t)'@', 0x01, 0x01, (uint8_t)'#'};

  return prot;
}



void serial_comm(){

  while(Serial.available() > 0){

    if(buffer_index < 100){
      buffer[buffer_index++] = Serial.read() - 48;
    }else{
      clean_buffer();
    }
  }

  // Receiver
  if(buffer_index > 0){

    if(SERIAL_STATES == 0){
       if(buffer_index >= 4){
        //Start comm 
        // 0x40:0x2:0x0:0x0:0x23:
        //if(buffer[0] ==  (uint8_t)'@' &&  buffer[1] == 2 && buffer[2] == 0 && buffer[3] ==  0 && buffer[4] == (uint8_t)'#'){
          SERIAL_STATES = 1;
          // send ACK 
          //uint8_t * sendMsg = createProtACK();
          uint8_t prot[]  = {(uint8_t)'@', 0x01, 0x01, (uint8_t)'#'};
          Serial.write( (uint8_t*) prot, 4 ); 
        
          clean_buffer();
        //}
      }
     }
  }

  // Send pos when comm is live
  if(SERIAL_SEND_ENC == 1 && SERIAL_STATES > 0){

    uint8_t * sendMsg = createProtPOS();
    Serial.write( (uint8_t*) sendMsg, sizeof(sendMsg) ); 

    SERIAL_SEND_ENC = 0;
  }
}//void serial_comm()

void setup_serial_thread(){
  
  // Configure serial_thread
  serial_thread.onRun(serial_comm);
  serial_thread.setInterval(250);

  controll_thread.add(&serial_thread); 
}// void setup_serial_thread()


 
