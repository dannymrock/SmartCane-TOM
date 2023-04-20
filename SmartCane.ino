#include "SonarEZ0pw.h"
#include "DYPlayerArduino.h"

SonarEZ0pw Sonar(7); // pin D7
float cm_dis = 0.00;
float Inch_dis = 0.00;

// Initialise the player, it defaults to using Serial.
DY::Player player;

// Define constants for easier customization
unsigned long previousMillis = 0; 
long interval = 2000;
const float LOWER_BOUND = 12.7;
const float DETECTION_LIMIT = 250.0;
const float repeatTimer = 7.0;
const float SPEED_LIMIT = -7.0;
const String TOO_CLOSE_MSG = "The object is too close to know how close it is";
const String NO_OBJECT_DETECTED_MSG = "No object detected within range, nearest object is ";
bool beep = false;
float IntervalRatio = 3.5; //(5000ms-250ms)/(50cm - 12.7cm) where 5000 max beep interval, 250ms min beep interval, DETECTION_LIMIT - min detection 
int speachDelay = 0;
float interceptoY = 345.536;

void setup() {
  Serial.begin(9600);
  player.begin();
  player.setVolume(30); // 50% Volume
}

// Function to get the distance in centimeters
float getDistance() {
  return Sonar.Distance(cm);
}

// Function to print a message to the serial monitor
void printToSerialMonitor(String s) {
  Serial.println(s);
  
}

// Function to check if the object is close (within the detection limit)
bool isClose(float distance) {
  return distance <= DETECTION_LIMIT;
}

void loop() {

  unsigned long currentMillis = millis();

  static float previous_distance = 0;
  static bool object_detected = false;

  float distance = getDistance();
  float dx = distance - previous_distance;

 if(currentMillis - previousMillis >= interval && object_detected) {
    delay(speachDelay);
    speachDelay = 0;
    player.playSpecified(3);
    previousMillis = currentMillis;
    delay(50);
    beep = true;
  }



  if(distance <= DETECTION_LIMIT){
    //objeto detectado
    String strdx = "x: " + String(distance) + " dx: " + String(dx) + "Interval: " + String(interval);
    printToSerialMonitor(strdx);
    if(!object_detected){
      player.playSpecified(1);
      previousMillis = millis() - 1;
      speachDelay = 3000;
    }
    if(dx < SPEED_LIMIT && object_detected){
      
      //player.playSpecified(2);
      //speachDelay = 0;
      
    }
    if(object_detected && beep){

      interval = (IntervalRatio * distance) + interceptoY;

    }
    object_detected = true;

  } else {
    object_detected = false;

  }

  previous_distance = distance;
  delay(repeatTimer);
  beep = false;

}