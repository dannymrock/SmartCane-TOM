#include <Arduino.h>
#include <DFRobotDFPlayerMini.h>
#include "SonarEZ0pw.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "HMC5883L.h"

//debug mode
//###################################################################################################################################
//#define DEBUG
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

//mp3 variables
unsigned long lastPlayingTime = 0;
unsigned int playTime = 0;
DFRobotDFPlayerMini myDFPlayer;


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
    lastPlayingTime = millis();
    playTime = 3000;//change 1500 to real time
    myDFPlayer.play(1);
    break;
  case 2:
    lastPlayingTime = millis();
    playTime = 3000;//change 1500 to real time
    myDFPlayer.play(2);
    break;
  case 3:
    lastPlayingTime = millis();
    playTime = 3000;//change 1500 to real time
    myDFPlayer.play(3);
    break;
  case 4:
    lastPlayingTime = millis();
    playTime = 3000;//change 1500 to real time
    myDFPlayer.play(4);
    break;
  case 5:
    lastPlayingTime = millis();
    playTime = 3000;//change 1500 to real time
    myDFPlayer.play(5);
    break;
  case 6:
    lastPlayingTime = millis();
    playTime = 3000;//change 1500 to real time
    myDFPlayer.play(6);
    break;
  case 7:
    lastPlayingTime = millis();
    playTime = 3000;//change 1500 to real time
    myDFPlayer.play(7);
    break;
  case 8:
    lastPlayingTime = millis();
    playTime = 3000;//change 1500 to real time
    myDFPlayer.play(8);
    break;

  
  default:
    Serial.println("error: song not found");
    break;
  }
  } Serial.println("error: song already playing");
}

void updateMP3(){
  if(isMP3Finished()){
   lastPlayingTime = millis();
    playTime = 0;
  }
}

/*int analizeDistance(){ // takes in distance, returns the number of the audio that must be played
  
  #ifdef DEBUG
  Serial.println("began analing: ");
  #endif 
  unsigned long sensingTime = millis();
  #ifdef DEBUG
  Serial.print("dt from analysed: ");
  Serial.println(sensingTime - lastSencedUSTime);
  Serial.print("distance: ");
  Serial.println(Sonar.Distance(cm));
  #endif 
  if(sensingTime - lastSencedUSTime > 500){
    lastSencedUSTime = sensingTime;
    #ifdef DEBUG
    Serial.print("reading...: ");
    #endif 
    distanceBuffer[i] = Sonar.Distance(cm);
    distanceCM;
    Serial.print("cm: ");
    Serial.println(distanceCM);
    if(distanceCM < 200) return 8;
    else if(distanceCM < 300) return 7; 
    else if (distanceCM < 400) return 6;
    else if (distanceCM < 500) return 5;
    else if (distanceCM < 600) return 4;
    else return 3;
  } else {
    return -1;
  }
}

*/

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
  if (distanceCM < 200) return 8;
  else if (distanceCM < 300) return 7;
  else if (distanceCM < 400) return 6;
  else if (distanceCM < 500) return 5;
  else if (distanceCM < 600) return 4;
  else return 3;
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

  if (theta < 0) theta += 360;

  

 //GPT4 code:
  /*readRawMagnetometerData(&mx, &my, &mz);
  float theta = atan2(my, mx);
  theta = theta * (180 / M_PI);
  theta -= declination;

  if (theta < 0) theta += 360;
  //end of GPT4 code
*/
  #ifdef DEBUGCOMPASS
  Serial.print("compass reading: ");
  Serial.println(theta);
  #endif

  if (theta >= 0 && theta < 22.5) {
#ifdef DEBUGCOMPASS
    Serial.println(" heading North");
#endif
    return 9; // North
} else if (theta >= 22.5 && theta < 67.5) {
#ifdef DEBUGCOMPASS 
    Serial.println(" heading North East");
#endif
    return 10; // North East
} else if (theta >= 67.5 && theta < 112.5) {
#ifdef DEBUGCOMPASS
    Serial.println(" heading East");
#endif
    return 11; // East
} else if (theta >= 112.5 && theta < 157.5) {
#ifdef DEBUGCOMPASS
    Serial.println(" heading South East");
#endif
    return 12; // South East
} else if (theta >= 157.5 && theta < 202.5) {
#ifdef DEBUGCOMPASS
    Serial.println(" heading South");
#endif
    return 13; // South
} else if (theta >= 202.5 && theta < 247.5) {
#ifdef DEBUGCOMPASS
    Serial.println(" heading South West");
#endif
    return 14; // South West
} else if (theta >= 247.5 && theta < 292.5) {
#ifdef DEBUGCOMPASS
    Serial.println(" heading West");
#endif
    return 15; // West
} else if (theta >= 292.5 && theta < 337.5) {
#ifdef DEBUGCOMPASS
    Serial.println(" heading North West");
#endif
    return 16; // North West
} else {
#ifdef DEBUGCOMPASS
    Serial.println(" heading North");
#endif
    return 9; // North

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
    } else {
      isUSOn = true;
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
      if(readDistance() != -1){
      distanceBuffer[numMeasurment] = readDistance();
      numMeasurment++;
      }
    } else if (numMeasurment == 5){
      float avrg = averageFloat(distanceBuffer);
      int mp3 = analyzeDistance(avrg);
      Serial.println(avrg);
      playMP3(mp3);
      numMeasurment = 0;
    }
    
    
    /*
    int trackID = analizeDistance();
    #ifdef DEBUG
    Serial.print("analized as follows: ");
    Serial.println(trackID);
    #endif
    if(trackID !=  -1){
      #ifdef DEBUG
      Serial.print("id to be played: ");
      Serial.println(trackID);
      #endif
      playMP3(trackID);

    }
  }
  */
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