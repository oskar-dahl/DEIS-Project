/*
 *  ROS serial communication
 */
#define ROS
#define JERRY //Remove define to run 'TOM'
//#define SERIALPRINT

#ifdef ROS
#include <ros.h>
#include <ros/time.h>
#include <robot_msgs/sensorData.h>
#include <robot_msgs/ctrlData.h>
#endif
#include <RedBot.h>

#define LINETHRESHOLD 900

RedBotMotors motors;
RedBotEncoder encoder = RedBotEncoder(A2, 10);
RedBotSensor sensorLeft = RedBotSensor(A3);   //-- Initialize a left sensor object on A3
RedBotSensor sensorCenter = RedBotSensor(A6); //-- Initialize a center sensor object on A6
RedBotSensor sensorRight = RedBotSensor(A7);  //-- Initialize a right sensor object on A7

/***** VARIABLES *****/
float targetSpeed = 100;
float error = 0;
int sensors[] = {0, 0, 0};


/***** FLAGS *****/
#define NONE 0
#define OK 1
#define FAIL 0
#define ON 1
#define OFF 0

/***** STATE *****/
#define INIT 0
#define LINEFOLLOWER 1
#define CHANGELINELEFT 2
#define CHANGELINERIGHT 3
#define DRIVETOLINELEFT 4
#define DRIVETOLINERIGHT 5

/***** ROS *****/
int speedLeft = NONE;
int speedRight = NONE;
int state = INIT;

/***** PID variables *****/
float P = 0;
float I = 0;
float D = 0;
float PIDval = 0;
float prevError = 0;
float currentPID = 0;

/***** Ultra sonic sensor *****/
long duration = 0;
int distance = 0;
int lock = 0;
#define ULTRASONICSENSORTHRESHOLD 10

// Pin connections for the HC-SR04 module
#define trigPin A0
#define echoPin A1

#ifdef ROS
ros::NodeHandle nh;
robot_msgs::sensorData sensorMsg;
#endif

float ultraSonicSensor(){

  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delay(2); //2 ms
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delay(10); //10 ms
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034/2; //0.034 cm/microsecond
  
  // Prints the distance on the Serial Monitor
  //Serial.print("");
  //Serial.println(distance);

  return distance;
}

// Standard PID controller.
float calculatePID(float Kp, float Ki, float Kd, float err) {
    P = err;
    I = I + err;
    D = err - prevError;
    PIDval = (Kp*P) + (Ki*I) + (Kd*D);
    prevError = err;
    return PIDval;
}

void followLine(int speedLeft, int speedRight, float PIDcorr) {

    if (speedLeft == 0){
        motors.leftBrake();
    } else {
        motors.leftMotor(-speedLeft - PIDcorr);
    }
    
    if (speedRight == 0){
        motors.rightBrake();
    } else {
        motors.rightMotor(speedRight - PIDcorr);
    }
}

#ifdef ROS
void ctrl(const robot_msgs::ctrlData &ctrlMsg){
  speedLeft = ctrlMsg.SpeedLeft;
  speedRight = ctrlMsg.SpeedRight;
  state = ctrlMsg.State;
}
#endif

void stateMachine()
{
  //Break the robot if collision is detected
  if (ultraSonicSensor() < ULTRASONICSENSORTHRESHOLD){
    lock = ON;
  }else{
    lock = OFF;  
  }
  if (lock == OFF){
    switch(state){
      case INIT:
        motors.brake();
        break;
      case LINEFOLLOWER:
        // Read IR sensors
        sensors[1] = (sensorCenter.read() > LINETHRESHOLD);
        sensors[2] = (sensorRight.read() > LINETHRESHOLD);
        sensors[0] = (sensorLeft.read() > LINETHRESHOLD);

        if(sensors[0] == 1 && sensors[1] == 0 && sensors[2] == 0) {
          error = -2;
        } else if(sensors[0] == 1 && sensors[1] == 1 && sensors[2] == 0) {
          error = -1;   
        } else if(sensors[0] == 0 && sensors[1] == 1 && sensors[2] == 0) {
            error = 0;
        } else if(sensors[0] == 0 && sensors[1] == 1 && sensors[2] == 1){
         error = 1;
        } else if(sensors[0] == 0 && sensors[1] == 0 && sensors[2] == 1){
          error = 2;
        }

        currentPID = calculatePID(25, 0.00, 0, error);
        //currentPID = calculatePID(25, 0.00, 20, error);
        followLine(speedLeft, speedRight, currentPID);

        break;
      case CHANGELINELEFT:
        motors.pivot(70, 700); // Turn left
        state = DRIVETOLINELEFT;

        break;
      case CHANGELINERIGHT:
        motors.pivot(-70, 700); // Turn right
        state = DRIVETOLINERIGHT;

        break;
      case DRIVETOLINELEFT:

        // Read IR sensors
        sensors[1] = (sensorCenter.read() > LINETHRESHOLD);
        sensors[2] = (sensorRight.read() > LINETHRESHOLD);
        sensors[0] = (sensorLeft.read() > LINETHRESHOLD);
  
        if(sensors[0] == 1 || sensors[1] == 1 || sensors[2] == 1){
          motors.drive(70, 500); // Turn left
          motors.pivot(-70, 700); // Turn right
          state = LINEFOLLOWER;
        }else{
          motors.drive(80);
        }
        break;
      case DRIVETOLINERIGHT:

        // Read IR sensors
        sensors[1] = (sensorCenter.read() > LINETHRESHOLD);
        sensors[2] = (sensorRight.read() > LINETHRESHOLD);
        sensors[0] = (sensorLeft.read() > LINETHRESHOLD);
  
        if(sensors[0] == 1 || sensors[1] == 1 || sensors[2] == 1){
          motors.drive(70, 500);
          motors.pivot(70, 700); // Turn left
          state = LINEFOLLOWER;
        }else{
          motors.drive(80);
        }
        break;
    }
  }else{
    motors.brake();
  }
}

#ifdef ROS
//ros::Publisher chatter("g5_sensor_channel", &sensorMsg);	             	//-- Publisher
#ifdef JERRY
ros::Subscriber<robot_msgs::ctrlData> sub("g5_arduino_ctrl_channel_jerry", &ctrl );   //-- Subscriber jerry
#else
ros::Subscriber<robot_msgs::ctrlData> sub("g5_arduino_ctrl_channel_tom", &ctrl );   //-- Subscriber tom
#endif
#endif

// Init.
void setup() {
  
    #ifdef ROS
    nh.initNode();
    //nh.advertise(chatter);	//-- sensor_channel
    nh.subscribe(sub);		//-- ctrl_arduino_channel
    #else
    Serial.begin(9600);
    #endif

    //Ultra sonic sensor
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

}

// Main loop
void loop() {

    // Run state machine.
    stateMachine();
   
    #ifdef ROS
    // Retrieve current sensor values
    //sensorMsg.IRLeft = sensorLeft.read();
    //sensorMsg.IRCenter = sensorCenter.read();
    //sensorMsg.IRRight = sensorRight.read();
    //sensorMsg.EncLeft = (int)encoder.getTicks(LEFT);
    //sensorMsg.EncRight = (int)encoder.getTicks(RIGHT);
    #else
    int irLeft = sensorLeft.read();
    int irCenter = sensorCenter.read();
    int irRight = sensorRight.read();
    int encLeft = (int)encoder.getTicks(LEFT);
    int encRight = (int)encoder.getTicks(RIGHT);

    #ifdef SERIALPRINT
    Serial.println("== SENSOR VALUES ==");
    Serial.println(irLeft, DEC);
    Serial.println(irCenter, DEC);
    Serial.println(irRight, DEC);
    Serial.println(encLeft, DEC);
    Serial.println(encRight, DEC);
    #endif
    #endif
    
    #ifdef ROS
    // Publish message
    //chatter.publish(&sensorMsg);
    nh.spinOnce();
    #endif
}
