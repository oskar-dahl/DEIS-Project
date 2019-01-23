#include <RedBot.h>

RedBotMotors motors;
RedBotEncoder encoder = RedBotEncoder(A2, 10);
RedBotSensor left = RedBotSensor(A3);   // initialize a left sensor object on A3
RedBotSensor center = RedBotSensor(A6); // initialize a center sensor object on A6
RedBotSensor right = RedBotSensor(A7);  // initialize a right sensor object on A7

#define LINETHRESHOLD 900

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

float calculatePID(float Kp, float Ki, float Kd, float err) {
    P = err;
    I = I + err;
    D = err - prevError;
    PIDval = (Kp*P) + (Ki*I) + (Kd*D);
    prevError = err;
    return PIDval;
}

void PIDcontrol(float PIDcorr) {
    motors.rightMotor(-targetSpeed + PIDcorr);
    motors.leftMotor(-targetSpeed - PIDcorr);
}

void setup() {
    Serial.begin(9600);
}

void loop() {

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

    currentPID = calculatePID(25, 0.01, 20, error);
    PIDcontrol(currentPID);
}
