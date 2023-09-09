#include <Arduino.h>
#include <DFRobotDFPlayerMini.h>
#include "SonarEZ0pw.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "HMC5883L.h"

//debug mode
//###################################################################################################################################
#define DEBUG
#define DEBUGCOMPASS
//###################################################################################################################################


//variable declarations
// pins:
const int buttonUSPin = 2;
const int buttonCOMPin = 3;
const int digialUSPin = 6;

// initialize Sonar
SonarEZ0pw Sonar(digialUSPin);

// button variables
bool isUSOn;
bool isCOMOn;
unsigned long lastUSOn = 0;
unsigned long lastCOMOn = 0;

//Ultrasonic variables
unsigned long lastSensedUSTime = 0;
float distanceBuffer[5];
int numMeasurment = 0;
//float min = 999;
//float max = 0;
int hasDetected = 0;
unsigned long playMultipleStartTime = 0;

//mp3 variables
unsigned long lastPlayingTime = 0;
unsigned int playTime = 0;
DFRobotDFPlayerMini myDFPlayer;
int playMultipleMP3ID = 0;


//compass variables
HMC5883L Compass;
int16_t mx, my, mz;
float declination = -4.4; //medellin inclination

//###################################################################################################################################

//methods:
/*
void readRawMagnetometerData(int16_t *mx, int16_t *my, int16_t *mz) {
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x03); // Set register pointer to X MSB
  Wire.endTransmission();
  Wire.requestFrom(HMC5883L_ADDRESS, 6);
  
  if (Wire.available() == 6) {
    *mx = Wire.read() << 8 | Wire.read();
    *mz = Wire.read() << 8 | Wire.read(); // Note that the order is X, Z, Y
    *my = Wire.read() << 8 | Wire.read();
  }
}*/

bool isMP3Finished(){ //returns true if enough time has passed for the current playing mp3 to finsh playing
  unsigned long currentMillis = millis();
  if(currentMillis - lastPlayingTime >= playTime){
    return true;
  }
  else return false;
}

void playMP3(int audio){ //takes id of the mp3 to play, plays mp3, updates isPlaying to true and playtime to song duration NOT FINISHED
  if(isMP3Finished()){
  switch (audio)
  {
  case 1:
    lastPlayingTime = millis(); // 5 metros
    playTime = 2000;//change 1500 to real time
    myDFPlayer.play(audio);
    break;
  case 2:
    lastPlayingTime = millis(); //obstaculo detectado
    playTime = 2500;//change 1500 to real time
    myDFPlayer.play(audio);
    break;
  case 3:
    lastPlayingTime = millis(); // 6 metros
    playTime = 2000;//change 1500 to real time
    myDFPlayer.play(audio);
    break;
  case 4:
    lastPlayingTime = millis();  //4 metros
    playTime = 2000;//change 1500 to real time
    myDFPlayer.play(audio);
    break;
  case 5:
    lastPlayingTime = millis();  //
    playTime = 2200;// 3 metros
    myDFPlayer.play(audio);
    break;
  case 6:
    lastPlayingTime = millis(); // 2 metros
    playTime = 3000;//change 1500 to real time
    myDFPlayer.play(audio);
    break;
  case 7:
    lastPlayingTime = millis(); //deteccion activada
    playTime = 3000;//change 1500 to real time
    myDFPlayer.play(audio);
    break;
  case 8:
    lastPlayingTime = millis(); //deteccion desactivada
    playTime = 3000;//change 1500 to real time
    myDFPlayer.play(audio);
    break;
    case 9:
    lastPlayingTime = millis(); //norte
    playTime = 4000;//change 1500 to real time
    myDFPlayer.play(audio);
    break;
  case 21:
    lastPlayingTime = millis(); //
    playTime = 4000;//change 1500 to real time
    myDFPlayer.play(audio);
    break;
  case 19:
    lastPlayingTime = millis();
    playTime = 4000;//change 1500 to real time
    myDFPlayer.play(audio);
    break;
  case 17:
    lastPlayingTime = millis();
    playTime = 5000;//change 1500 to real time
    myDFPlayer.play(audio);
    break;
  case 15:
    lastPlayingTime = millis();
    playTime = 5000;//change 1500 to real time
    myDFPlayer.play(audio);
    break;
  case 13:
    lastPlayingTime = millis();
    playTime = 5000;//change 1500 to real time
    myDFPlayer.play(audio);
    break;
  case 11:
    lastPlayingTime = millis();
    playTime = 5000;//change 1500 to real time
    myDFPlayer.play(audio);
    break;
  case 23:
    lastPlayingTime = millis();
    playTime = 4000;//change 1500 to real time
    myDFPlayer.play(audio);
    break;
  case 24:
    lastPlayingTime = millis();
    playTime = 500;//change 1500 to real time
    myDFPlayer.play(2);
    break;
  
  default:
  #ifdef DEBUG
    Serial.println("error: song not found");
  #endif
    break;
  }
  //set a pause of 1s for each track
  playTime += 1000;
  } 
  #ifdef DEBUG
    Serial.println("error: song already playing");
  #endif
}

void updateMP3(){
  if(isMP3Finished()){
   lastPlayingTime = millis();
    playTime = 0;
  }
}

float readDistance() {
  unsigned long sensingTime = millis();
  if (sensingTime - lastSensedUSTime > 50) {
    lastSensedUSTime = sensingTime;
    return Sonar.Distance(cm);
  } else {
    return -1;
  }
}

int analyzeDistance(float distanceCM) {
  
if (distanceCM < 180) {
hasDetected++;
return 24;  //play "obstáculo detectado"
}
else if (distanceCM < 250) {
hasDetected++;
return 6; // play "2 metros"
}
else if (distanceCM < 350) {
hasDetected++;
return 5; // play "3 metros"
}
else if (distanceCM < 450) {
hasDetected++;
return 4; // play "4 metros"
}
else if (distanceCM < 550) {
hasDetected++;
return 1; // play "5 metros"
}
else if (distanceCM < 600) {
hasDetected++;
return 3; // play "6 metros"
}
else if (distanceCM < 630) {
hasDetected = 0;
return -1; // do not play any track
}
else return -1;
}

int analizeCompass(){// measures compas and outputs the correct audio id to play
  
  
  Compass.getHeading(&mx, &my, &mz);
  // There is likely a spurious read in the i2c buffer, I reading again to discard previous reading and apply a delay.
  //TODO: fix this code to improve performance
  delay(5);
  Compass.getHeading(&mx, &my, &mz);
  float theta = atan2(my, mx);
  theta = theta * (180 / M_PI);
  theta -= declination;
  // 186 degrees is the factor of correction because of the positioning of the sensor in the case
  // I measured the difference using a digital compass in a mobile phone as a reference device
  theta -= 186;

  if (theta < 0) theta += 360;

  #ifdef DEBUGCOMPASS
  Serial.print("compass reading: ");
  Serial.println(theta);
  #endif

  if ((theta >= 0 && theta < 22.5) || (theta >= 337.5 && theta < 360)) {
#ifdef DEBUGCOMPASS
    Serial.println(" heading North");
#endif
    return 9; // North
} else if (theta >= 22.5 && theta < 67.5) {
#ifdef DEBUGCOMPASS 
    Serial.println(" heading North East");
#endif
    return 13; // North East
} else if (theta >= 67.5 && theta < 112.5) {
#ifdef DEBUGCOMPASS
    Serial.println(" heading East");
#endif
    return 21; // East
} else if (theta >= 112.5 && theta < 157.5) {
#ifdef DEBUGCOMPASS
    Serial.println(" heading South East");
#endif
    return 17; // South East
} else if (theta >= 157.5 && theta < 202.5) {
#ifdef DEBUGCOMPASS
    Serial.println(" heading South");
#endif
    return 23; // South
} else if (theta >= 202.5 && theta < 247.5) {
#ifdef DEBUGCOMPASS
    Serial.println(" heading South West");
#endif
    return 15; // South West
} else if (theta >= 247.5 && theta < 292.5) {
#ifdef DEBUGCOMPASS
    Serial.println(" heading West");
#endif
    return 19; // West
} else if (theta >= 292.5 && theta < 337.5) {
#ifdef DEBUGCOMPASS
    Serial.println(" heading North West");
#endif
    return 11; // North West
} else {
#ifdef DEBUGCOMPASS
    Serial.println("error ");
#endif
    return -1; // North

}

}

void buttonUSISR(){ //Ultra Sonic Sensor Interuption, true/false flag swich
  unsigned long interruptTime = millis();
  #ifdef DEBUG
  Serial.println("In changing button method");
  delay(500);
  #endif
  if(interruptTime - lastUSOn > 1000){
    Serial.println("Changing button");
    if(isUSOn){
      isUSOn = false;
      playMP3(8);
      hasDetected = 0;
    } else {
      isUSOn = true;
      playMP3(7);
    }
    
  }
  lastUSOn = interruptTime;
}

float averageFloat(float arr[5]) {
    float total = 0;
    for (int i = 0; i < 5; i++) {
        total += arr[i];
    }
    total /= 5;
    return total;
}

float minFloat(float arr[5]){
  float min = arr[0];
  for(int i = 1; i < 5; i++){
    if(arr[i] < min){
      min = arr[i];
    }
  }
  return min;
}

void buttonCOMISR(){ //Compass Interuption, true flag
  unsigned long interruptTime = millis();
  #ifdef DEBUG
  Serial.println("inside com button ISR");
  #endif
  if(interruptTime - lastCOMOn > 1000){
    isCOMOn = true;
  }
  lastCOMOn = interruptTime;
}

void playMultipleMP3ToDetect(int ID){
  if(playMultipleStartTime == 0){
    playMultipleMP3ID = ID;
    playMP3(2);
    playMultipleStartTime = millis();
  } else {
    unsigned long currentMultipleTime = millis();
    if(currentMultipleTime - playMultipleStartTime > 2500){
      playMP3(playMultipleMP3ID);
      playMultipleStartTime = 0;
    }
  }
}


//setup and loop methods
//###################################################################################################################################

void setup(){
  Serial.begin(9600);
  Wire.begin();
  Compass.initialize();
  Wire.setClock(100000L);
  #ifdef DEBUG
  Serial.println("compass initialized");
  #endif
  pinMode(buttonUSPin, INPUT_PULLUP);
  pinMode(buttonCOMPin, INPUT_PULLUP);
  pinMode(digialUSPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(buttonUSPin), buttonUSISR ,FALLING);
  attachInterrupt(digitalPinToInterrupt(buttonCOMPin), buttonCOMISR , FALLING);

  myDFPlayer.begin(Serial);
  myDFPlayer.volume(30);

}

void loop(){


  #ifdef DEBUG
 // Serial.print("Ultra Sonic Button: ");
 // Serial.println(isUSOn);
  #endif
  updateMP3();
  if(isUSOn)
  {
    if(numMeasurment < 5){

      float measurment = readDistance();
      if(measurment != -1){
      distanceBuffer[numMeasurment] = measurment;
      //if(measurment > max) max = measurment;
      //if(measurment < min) min = measurment;
      numMeasurment++;
      }
    } else if (numMeasurment == 5){
      float avrg = averageFloat(distanceBuffer);
      int mp3 = analyzeDistance(avrg);
      Serial.println(avrg);
      if(hasDetected == 1){
        playMultipleMP3ToDetect(mp3);
        hasDetected++;
      } else playMP3(mp3);
      numMeasurment = 0;
      //max = 0;
      //min = 999;
    }
  }

  if(isCOMOn){
    int audio = analizeCompass();
    playMP3(audio);
    isCOMOn = false;
    #ifdef DEBUG
    Serial.print("audio playing due to compass:  ");
    Serial.println(audio);

    #endif
  }
  
}