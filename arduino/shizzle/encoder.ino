#include <Thread.h>
#include <ThreadController.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;


/*
Author: Oskar Dahl
FuncName: odometry
*/
/*
void odometry(){

    float dDr = Dr - DrP;
    float dDl = Dl - DlP;

    float dD = (dDr + dDl) / 2;
    float dA = (dDr - dDl) / WHEEL_BASE;

    float dX = dD * cos(XkP(3)  + dA/2);
    float dY = dD * sin(XkP(3)  + dA/2);

    Xk(0) = XkP(0) + dX;
    Xk(1) = XkP(1) + dY;
    Xk(2) = fmod((XkP(3) + dA), 2*M_PI);
}
*/


/*
Author: Oskar Dahl
FuncName: encoderLoop
*/
void encoderLoop(){

  // Clear encoders
  if (digitalRead(button_pin) == LOW)
  {
    encoder.clearEnc(BOTH);  // Reset the counters.
  }

  // Stor encoder ticks
  float lTicks = encoder.getTicks(LEFT);    // read the left motor encoder
  float rTicks = encoder.getTicks(RIGHT);   // read the right motor encoder

  //distance = (wheel circumference)*counts/(counts per complete revolution)
  Dr = wheelCircumference * (rTicks) / countsPerRev;
  Dl = wheelCircumference * (lTicks) / countsPerRev;
  
  if(FIRST_TIME == 1){

    DrP = Dr;
    DlP = Dl;
    
    FIRST_TIME = 0;
  }

  dDr = Dr - DrP;
  dDl = Dl - DlP;

  DrP = Dr;
  DlP = Dl;

  //Send pos to pi every 1 seconds (change SEND_ENC)
  if(millis() - timer_send_enc > SEND_ENC){
    SERIAL_SEND_ENC = 1;
    timer_send_enc = millis();
  }
  
  //delay(100);
}


/*
Author: Oskar Dahl
FuncName: setup_encoderThread
*/
void setup_encoder_thread(){
  
  encoderThread.onRun(encoderLoop);
  encoderThread.setInterval(250);

  controll_thread.add(&encoderThread); 
}// void setup_serial_thread()
