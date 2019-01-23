#include <RedBot.h>
#include <RedBotSoftwareSerial.h>

long duration;
int distance;

// Pin connections for the HC-SR04 module
#define trigPin A0
#define echoPin A1

void setup() {

  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

}

void loop() {

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
  Serial.print("");
  Serial.println(distance);

}
