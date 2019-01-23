/*
 *  ROS serial communication
 */

#include <RedBot.h>
#include <ros.h>
#include <ros/time.h>
#include <robot_msgs/sensorData.h>
#include <robot_msgs/ctrlData.h>

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

//PID variables
float P = 0;
float I = 0;
float D = 0;
float PIDval = 0;
float prevError = 0;
float currentPID = 0;

ros::NodeHandle nh;

robot_msgs::sensorData sensorMsg;

float calculatePID(float Kp, float Ki, float Kd, float err) {
    P = err;
    I = I + err;
    D = err - prevError;
    PIDval = (Kp*P) + (Ki*I) + (Kd*D);
    prevError = err;
    return PIDval;
}

void followLine(int speed_left, int speed_right, float PIDcorr) {
    
    if (speed_left == 0){
        motors.leftBrake();
    } else {
        motors.leftMotor(-speed_left - PIDcorr);
    }
    
    if (speed_right == 0){
        motors.rightBrake();
    } else {
        motors.rightMotor(-speed_right + PIDcorr);
    }
}

void ctrl(const robot_msgs::ctrlData &ctrlMsg){
    int speed_left = ctrlMsg.SpeedLeft;
    int speed_right = ctrlMsg.SpeedRight;
    int state = ctrlMsg.State;
    
    if(state == 1){
        sensors[1] = (center.read() > LINETHRESHOLD);
        sensors[2] = (right.read() > LINETHRESHOLD);
        sensors[0] = (left.read() > LINETHRESHOLD);

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

        currentPID = calculatePID(25, 0.01, 20, error);
        followLine(speed_left, speed_right, currentPID);
    } else{
         if (speed_left == 0){
            motors.leftBrake();
        } else {
            motors.leftMotor(-speed_left);
        }
    
        if (speed_right == 0){
            motors.rightBrake();
        } else {
            motors.rightMotor(-speed_right);
        }  
    }
}

ros::Publisher chatter("g5_sensor_channel", &sensorMsg);	             		//-- Publisher
ros::Subscriber<robot_msgs::ctrlData> sub("g5_arduino_ctrl_channel", &ctrl );   //-- Subscriber

void setup() {
    nh.initNode();
    nh.advertise(chatter);	//-- sensor_channel
    nh.subscribe(sub);		//-- ctrl_arduino_channel
}

void loop() {  
    //-- Retrieve current sensor values
    sensorMsg.IRLeft = sensorLeft.read();
    sensorMsg.IRCenter = sensorCenter.read();
    sensorMsg.IRRight = sensorRight.read();
    sensorMsg.EncLeft = (int)encoder.getTicks(LEFT);
    sensorMsg.EncRight = (int)encoder.getTicks(RIGHT);

    //-- Publish message
    chatter.publish(&sensorMsg);
    nh.spinOnce();
    //delay(1); //-- Delay in ms
}
