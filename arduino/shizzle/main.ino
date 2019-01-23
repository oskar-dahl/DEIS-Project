#include <RedBot.h>
// By Oskar Dahl & Fredrik Johansson

#include <RedBotSoftwareSerial.h>
#include <RedBot.h>
#include <Thread.h>
#include <ThreadController.h>
#include <BasicLinearAlgebra.h>

RedBotMotors motors;
uint16_t spd = 0;

// Serial
unsigned long timer_send_enc = millis(); //Send pos to pi every 10 second
unsigned long SEND_ENC = 125; //

//STATES
uint8_t SERIAL_STATES = 0;
uint8_t START_COMM = 0;
uint8_t SERIAL_SEND_ENC = 0;

// Buttons
uint8_t button_pin = 12;
uint8_t FIRST_TIME = 1;

// Robot 
float WHEEL_BASE = 158;
float wheelCircumference = 60 * M_PI;
float countsPerRev = 192;   // 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev

// Encoder
RedBotEncoder encoder = RedBotEncoder(A2, 10);  // initializes encoder on pins A2 and 10
BLA::Matrix<4, 1> Xk;
BLA::Matrix<4, 1> XkP;

float Dr = 0;
float Dl = 0;
float DrP = 0;
float DlP = 0;

float dDr = 0;
float dDl = 0;

// BUFFER
uint8_t BUFFER_MAX_LEN = 100;
uint8_t buffer[100];
uint16_t buffer_index = 0;
String SERIAL_STATE = "IDLE";

// Threads
Thread serial_thread = Thread();
Thread encoderThread = Thread();

// ThreadController that will controll all threads
ThreadController controll_thread = ThreadController();

void setup(void)
{
  Serial.begin(9600);
  setup_serial_thread(); // Start serial threads
  setup_encoder_thread();
}


void loop(void)
{

  controll_thread.run();
  //motors.rightMotor(-(spd));
  //motors.leftMotor(-(spd));

  

 

  
}
