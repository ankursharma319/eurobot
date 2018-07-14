#include <Wire.h>

#define MD25ADDRESS         0x58
#define SPEED1              0x00                        
#define SPEED2              0x01
#define ENCODERONE          0x02                              // Byte to read motor encoder 1
#define ENCODERTWO          0x06                              // Byte to read motor encoder 2
#define ACCELERATION        0xE                               // Byte to define motor acceleration
#define CMD                 0x10                              // Byte to reset encoder values
#define MODE_SELECTOR       0xF                               // Byte to change between control MODES

int MD25_Mode = 2;
int front_speed = 180;
int reverse_speed = 100;
int rotational_speed = 60;



int frontSensorPin = 5;
int rearSensorPin = 4;
int directionModePin = A1;
int pullSwitchPin = 8;
int redLedPin = 1;
int greenLedPin = A2;
int blueLedPin = A3;
int piezoPin = A0;


unsigned long time_start;
int pullSwitchState = 0;
int directionMode = 0;

void setup(){
  Serial.begin(9600);
  Wire.begin();                                               // Begin I2C bus
  delay(100);                                                 // Wait for everything to power up
  setMD25Mode(MD25_Mode);
  encodeReset();
  pinMode(pullSwitchPin, INPUT);
  pinMode(directionModePin, INPUT);
  pinMode(frontSensorPin, INPUT);
  pinMode(rearSensorPin, INPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);  
  playStartTone();
  setLedColor(255, 0, 0);
}

void loop(){
  pullSwitchState = digitalRead(pullSwitchPin);
  directionMode = digitalRead(directionModePin);
  delay(100);
  if(directionMode==0) {
      setLedColor(0, 0, 255);
  } else {
      setLedColor(255, 255, 0);
  }
  if(pullSwitchState == 1) {
    time_start = millis();
    setLedColor(0, 0, 0);
//    playStartTone();
    
    //YELLOW SIDE MODE
    if(directionMode==1) {

      rev(400, -1);//360
      delay(1000);
      auto_correction(400, encoder1(), encoder2(), 1, -1, -1);
      delay(1000);
      CloseBoth();                         //Collect module 1
      delay(1000); 
      rev(90, -1);
      delay(1000);
      leftRot(45, -1);//32
      delay(1000);
      auto_correction(45, encoder1(), encoder2(), 3, -1, -1);
      delay(1000);
      rev(1200, -1);
      delay(1000);
      auto_correction(1120, encoder1(), encoder2(), 2, -1, -1);      
      delay(1000);
      leftRot(132, -1);
      delay(1000);
      auto_correction(150, encoder1(), encoder2(), 3, -1, -1);
      delay(1000);
      straight(235, -1);
      delay(1000);
      correct_at_milestone(4, 250, 250, 0);

      delay(1000);

      encodeReset();                                    //Score Module 2
      setMD25Mode(2);
      do{
        changeAccelerationRegister(5);
        writeSpeed1(220);
      }while(abs(encoder1()) < 270);
      halt();

      delay(1000);

      straight(100, -1);

      delay(100);     
      
    
      rev(170, -1);
      delay(1000);
      rightRot(265, -1);
      delay(1000);
      OpenServo1();                                     //Drop Module 1
      OpenServo2();
      delay(500);
      straight(102, -1);
      delay(1000);
      leftRot(265, -1);
      //senseDistance1();
      //senseDistance2();
      //senseDistance3();
      //delay(1000);
      delay(1000);
      straight(80, -1);
      delay(1000);
      correct_at_milestone(4, 250, 250, 0);
      delay(1000);
      straight(450, 255);                              //score module 1
      delay(1000);
      rev(450, -1);
  
      straight(300, 255);                                   //score module 1
      delay(1000);
  
      rev(800, -1);
      delay(1000);
      CloseBoth();
      delay(1000);
      leftRot(115, -1);
      delay(1000);
      rev(750, -1);
      delay(1000);
      rightRot(58, -1);
      delay(1000);
      rev(1000, -1);
      delay(1000);
      OpenServo1();                                     
      OpenServo2();
      delay(500);
      straight(100, -1);
      delay(1000);
      rightRot(264, -1);
      delay(100);
      straight(200, -1);
    
    delay(5000);
  }
  // BLUE SIDE MODE 
  else {

    rev(360, -1);
    delay(1000);
    auto_correction(360, encoder1(), encoder2(), 1, -1, -1);
    
    CloseBoth();                                      //Collect module 1
    delay(1000);
    rev(90, -1);
    delay(1000);
    rightRot(49, -1);
    delay(500);
    auto_correction(49, encoder1(), encoder2(), 2, -1, -1);
    delay(500);
    rev(1200, -1);
    delay(1000);
    auto_correction(1200, encoder1(), encoder2(), 3, -1, -1);
    delay(1000);
    rightRot(127, -1);
    delay(1000);
    auto_correction(127, encoder1(), encoder2(), 2, -1, -1);
    delay(1000);
    straight(125, -1);
    delay(1000);
    correct_at_milestone(4, 250, 250, 0);
    delay(1000);
  
    encodeReset();                                    //Score Module 2
    setMD25Mode(2);
    do{
      changeAccelerationRegister(5);
      writeSpeed1(220);
    }while(abs(encoder1()) < 270);
    halt();
    
    delay(1000);
    straight(100, -1);
    delay(1000);
    rev(188, -1); //185
    delay(1000);
    rightRot(268, -1);
    delay(1000);
    OpenServo1();                                     //Drop Module 1
    OpenServo2();
    delay(500);
    straight(100, -1);
    delay(1000);
    leftRot(264, -1);
    delay(1000);
    straight(42, -1);
    delay(1000);
    correct_at_milestone(4, 250, 250, 0);
    delay(1000);
    straight(300, 255);                                   //score module 1
    delay(1000);

    rev(800, -1);
    delay(1000);
    CloseBoth();
    delay(1000);
    rightRot(115, -1);
    delay(1000);
    rev(750, -1);
    delay(1000);
    leftRot(58, -1);
    delay(1000);
    rev(1000, -1);
    delay(1000);
    OpenServo1();                                     
    OpenServo2();
    delay(5000);
    delay(1000);

    //////SCORING THE MODULE AT THE BACK
    /*
    rev(395, -1);
    delay(1000);
    leftRot(180, -1);
    delay(1000);
    rev(950, -1);
    delay(1000);
    CloseBoth();                         //collect module 3
    delay(1000);
    straight(660, -1);
    delay(1000);
    leftRot(50, -1);
    delay(1000);
    OpenServo1();                                     
    OpenServo2();
    delay(1000);
    straight(75, -1);
    delay(1000);
    leftRot(245, -1);
    delay(1000);
    straight(40, -1);
    correct_at_milestone(4, 250, 250, 0);
    delay(1000);
    straight(500, -1);          //score module 3
    delay(100);
    rev(400, -1);
    delay(1000);
    */
  }
  delay(200);
  }
}

void avoidCollision(bool useFrontSensorReading) {
  if(useFrontSensorReading){
  if(xsenseDistance1()<170) {
    halt();
    delay(1000);
    avoidCollision(true);
  }
  } else {
  if(xsenseDistance2()<170) {
    halt();
    delay(1000);
    avoidCollision(false);
  }
  }
}

bool isTimeUp() {
  if(millis()-time_start >= 90000) {
    Serial.print("Time up");
    halt();
    OpenServo3();
    delay(10000000000);
    
    return true;
  } else {
    return false;
  }
}

void correct_at_milestone(int milestone, int expected_distance_1, int expected_distance_2, int expected_distance_3) {
  delay(200);
  int dis1 = senseDistance1();
  int dis2 = senseDistance2();
  int dis3 = senseDistance3();

  if(milestone==1) {
    if(dis1>50 && dis2>50) {
      //No Problem
    } else if(dis1<50) {
      rightRot(10, -1);
    } else if(dis2<50) {
      leftRot(10, -1);
    }
  } else if(milestone==2) {
      if(dis3>expected_distance_3) {
        leftRot(8, -1);
        //correct_at_milestone(2, 0, 0, expected_distance_3);
      }
  } else if(milestone==3) {
    if(dis3>expected_distance_3) {
      rev((dis3-expected_distance_3)/1.4, -1);
    }
  } else if(milestone==4) {
    if((dis1>expected_distance_1 and dis2>expected_distance_2) or (dis1<expected_distance_1 and dis2<expected_distance_2)) {
      //straight(0, -1);
      //Serial.println("all good continue...");
    } else if(dis1<expected_distance_1) {
      rightRot(0.75 , 160);
    } else if(dis2<expected_distance_2){
      leftRot(0.75, 100);
    }
  } else if(milestone==5) {
    if(dis1>300 && dis2>300) {
      
    } else if(dis1<300) {
      rightRot(2 , -1);
    } else if(dis2<300){
      leftRot(2, -1);
    }
  }
}

//MOVE STRAIGHT
void straight(float distance, int optional_speed){
  encodeReset();
  Serial.print(senseDistance1());
  Serial.print(senseDistance2());
  setMD25Mode(2);
  if (optional_speed<0) {
    optional_speed=front_speed;
  }
  unsigned long temp_time = millis();
  while(abs(encoder1()) < distance and !isTimeUp()){
    avoidCollision(true);
    changeAccelerationRegister(1);
    writeSpeed1(optional_speed);
  }
  halt();
}

//MOVE BACK
void rev(float distance, int optional_speed){
  setMD25Mode(2);
  encodeReset();
  if (optional_speed<0) {
    optional_speed=reverse_speed;
  }
  while(abs(encoder1()) < distance and !isTimeUp()){
    avoidCollision(false);
    changeAccelerationRegister(1);
    writeSpeed1(optional_speed);
  }
  halt();
}

//RIGHT ROTATE
void rightRot( float distance, int optional_speed) {
  encodeReset();
  setMD25Mode(2);
  if (optional_speed<0) {
    optional_speed=128+rotational_speed;
  }
  while(abs(encoder1())<distance and !isTimeUp()) {
    changeAccelerationRegister(1);
    writeSpeed2(optional_speed);
  }
  halt();
}

//LEFT ROTATE
void leftRot(float distance, int optional_speed) {
  encodeReset();
  setMD25Mode(2);
  if (optional_speed<0) {
    optional_speed=128-rotational_speed;
  }
  while(abs(encoder1())<distance and !isTimeUp()){
    changeAccelerationRegister(1);
    writeSpeed2(optional_speed);
  }
  halt();
}

void auto_correction(float original_distance, float en1, float en2, int m_case, int correctionSpeed, float correctionPercentage) {
  if(m_case==2||m_case==3) {
    if(correctionPercentage<0) {
      correctionPercentage=0.005;
    }
  } else {
    if(correctionPercentage<0) {
      correctionPercentage=0.01;
    }
  }
  //STRAIGHT CASE
  if(m_case==0) {
    if(abs(en1)>original_distance && abs(en2)>original_distance) {
      rev(correctionPercentage*(abs(en1)-original_distance), correctionSpeed);
    } else if (abs(en1)<original_distance && abs(en2)<original_distance){
      straight(correctionPercentage*(abs(en1)-original_distance), correctionSpeed);
    }
  }
  //REVERSE CASE
  else if(m_case==1) {
    if(abs(en1)>original_distance && abs(en2)>original_distance) {
      straight(correctionPercentage*(abs(en1)-original_distance), correctionSpeed);
    } else if (abs(en1)<original_distance && abs(en2)<original_distance){
      rev(correctionPercentage*(abs(en1)-original_distance), correctionSpeed);
    }
  }
  //RIGHT TURN CASE
  else if(m_case==2) {
    if(abs(en1)>original_distance && abs(en2)>original_distance) {
      leftRot(correctionPercentage*(abs(en1)-original_distance), correctionSpeed);
    } else if (abs(en1)<original_distance && abs(en2)<original_distance) {
      rightRot(correctionPercentage*(abs(en1)-original_distance), correctionSpeed);
    }
  }
  //LEFT TURN CASE
  else if(m_case==3) {
    if(abs(en1)>original_distance && abs(en2)>original_distance) {
      rightRot(correctionPercentage*(abs(en1)-original_distance), correctionSpeed);
    } else if (abs(en1)<original_distance && abs(en2)<original_distance) {
      leftRot(correctionPercentage*(abs(en1)-original_distance), correctionSpeed);
    }
  }
}

void halt(){                                           //Function to stop motors by sending a 128 to the speeds.
  writeSpeed1(128);
  writeSpeed2(128);
  delay(500);
}

void setMD25Mode(int mode) {
  Wire.beginTransmission(MD25ADDRESS);                        // Set MD25 operation MODE
  Wire.write(MODE_SELECTOR);
  Wire.write(mode);                                           
  Wire.endTransmission();
}

void changeAccelerationRegister(int acc_register) {
    Wire.beginTransmission(MD25ADDRESS);                      // Sets the acceleration to register
    Wire.write(ACCELERATION);
    Wire.write(acc_register);
    Wire.endTransmission();
}

void writeSpeed1(int m_speed) {
    Wire.beginTransmission(MD25ADDRESS);                      // Sets a combined motor speed value
    Wire.write(SPEED1);
    Wire.write(m_speed);
    Wire.endTransmission();
}

void writeSpeed2(int m_speed) {
    Wire.beginTransmission(MD25ADDRESS);                      // Sets a combined motor speed value
    Wire.write(SPEED2);
    Wire.write(m_speed);
    Wire.endTransmission();  
}

//////////////// ALTERNATIVE /////////////////////////////////////////
////////////// NEED TO DELETE THIS SECTION AT SOME POINT//////////////
//MOVE STRAIGHT 2
void straight2(float distance, int optional_speed){
  encodeReset();
  setMD25Mode(0);
  if (optional_speed<0) {
    optional_speed=front_speed;
  }
  do{
    changeAccelerationRegister(1);
    writeSpeed1(optional_speed);
    writeSpeed2(optional_speed);
  }while(abs(encoder1()) < distance);
  halt();
}

//MOVE BACK 2
void rev2(float distance, int optional_speed){
  encodeReset();
  setMD25Mode(0);
  if (optional_speed<0) {
    optional_speed=reverse_speed;
  }
  do{
    changeAccelerationRegister(1);
    writeSpeed1(optional_speed);
    writeSpeed2(optional_speed);
  } while(abs(encoder1()) < distance);
  halt();
}

/////////////////////////////////////////////////////////////////////

void encodeReset(){ // This function resets the encoder values to 0
 Wire.beginTransmission(MD25ADDRESS);
 Wire.write(CMD);
 Wire.write(0x20); // Putting the value 0x20 to reset encoders
 Wire.endTransmission();
}

long encoder1(){ // Function to read and display value of encoder 1 as a long
  Wire.beginTransmission(MD25ADDRESS); // Send byte to get a reading from encoder 1
  Wire.write(ENCODERONE);
  Wire.endTransmission();
  
  Wire.requestFrom(MD25ADDRESS, 4); // Request 4 bytes from MD25
  while(Wire.available() < 4); // Wait for 4 bytes to arrive
  long poss1 = Wire.read(); // First byte for encoder 1, HH.
  poss1 <<= 8;
  poss1 += Wire.read(); // Second byte for encoder 1, HL
  poss1 <<= 8;
  poss1 += Wire.read(); // Third byte for encoder 1, LH
  poss1 <<= 8;
  poss1 +=Wire.read(); // Fourth byte for encoder 1, LL
  delay(50); // Wait for everything to make sure everything is sent
  
  return(poss1);
}

long encoder2(){ // Function to read and display velue of encoder 2 as a long
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(ENCODERTWO);
  Wire.endTransmission();
  
  Wire.requestFrom(MD25ADDRESS, 4); // Request 4 bytes from MD25
  while(Wire.available() < 4); // Wait for 4 bytes to become available
  long poss2 = Wire.read();
  poss2 <<= 8;
  poss2 += Wire.read();
  poss2 <<= 8;
  poss2 += Wire.read();
  poss2 <<= 8;
  poss2 +=Wire.read();
  
  return(poss2);
}

////////////////////////////////////////////////////////////////////////////////

const int servoPin1 = 12;
const int servoPin2 = 13;
const int servoPin3 = 11;

const int loopTime = 17;
const int servoSpeed = 100;
const int referenceSpeed = 1508;

void CloseServo1() {
  pinMode(servoPin1, OUTPUT);
  for(int i=0; i<loopTime; i++) {
    digitalWrite(servoPin1, HIGH);
    delayMicroseconds(referenceSpeed+servoSpeed);
    digitalWrite(servoPin1, LOW);
    delay(20);
  }
}

void CloseServo2(bool alt) {
  if(alt) {
    pinMode(servoPin2, OUTPUT);
    for(int i=0; i<7; i++) {
      digitalWrite(servoPin2, HIGH);
      delayMicroseconds(referenceSpeed-servoSpeed);
      digitalWrite(servoPin2, LOW);
      delay(20);
    } 
  } else {
  pinMode(servoPin2, OUTPUT);
  for(int i=0; i<loopTime+10; i++) {
    digitalWrite(servoPin2, HIGH);
    delayMicroseconds(referenceSpeed-servoSpeed);
    digitalWrite(servoPin2, LOW);
    delay(20);
  }
  }
}

void OpenServo2() {
  pinMode(servoPin2, OUTPUT);
  for(int i=0; i<loopTime+20; i++) {
    digitalWrite(servoPin2, HIGH);
    delayMicroseconds(referenceSpeed+servoSpeed);
    digitalWrite(servoPin2, LOW);
    delay(20);
  }
}

void OpenServo1() {
  pinMode(servoPin1, OUTPUT);
  for(int i=0; i<loopTime+20; i++) {
    digitalWrite(servoPin1, HIGH);
    delayMicroseconds(referenceSpeed-servoSpeed);
    digitalWrite(servoPin1, LOW);
    delay(20);
  }
}

void OpenServo3() {
  pinMode(servoPin3, OUTPUT);
  for(int i=0; i<250; i++) {
    digitalWrite(servoPin3, HIGH);
    delayMicroseconds(2500);
    digitalWrite(servoPin3, LOW);
    delay(20);
  }
}

void CloseServo3(bool alt) {
  if(alt) {
    pinMode(servoPin3, OUTPUT);
    for(int i=0; i<7; i++) {
      digitalWrite(servoPin3, HIGH);
      delayMicroseconds(referenceSpeed-servoSpeed);
      digitalWrite(servoPin3, LOW);
      delay(20);
    } 
  } else {
  pinMode(servoPin3, OUTPUT);
  for(int i=0; i<loopTime+10; i++) {
    digitalWrite(servoPin3, HIGH);
    delayMicroseconds(referenceSpeed-servoSpeed);
    digitalWrite(servoPin3, LOW);
    delay(20);
  }
  }
}

void CloseBoth() {
  pinMode(servoPin1, OUTPUT);
  for(int i=0; i<25; i++) {
    digitalWrite(servoPin1, HIGH);
    digitalWrite(servoPin2, HIGH);
    delayMicroseconds(referenceSpeed-servoSpeed);
    digitalWrite(servoPin2, LOW);
    delay(2*servoSpeed);
    digitalWrite(servoPin1, LOW);
    
    delay(20);
  }  
}

////////////////////////////////////////////////////////////////////////////////
/*
* Ultrasonic Sensor HC-SR04 and Arduino Tutorial
*
* Crated by Dejan Nedelkovski,
* www.HowToMechatronics.com
*
*/

// defines pins numbers
// RIGHT FRONT ULTRASONIC SENSOR
const int trigPin1 = 6;
const int echoPin1 = 7;
//LEFT FRONT ULTRASONIC SENSOR
const int trigPin2 = 9;
const int echoPin2 = 10;
//BACK ULTRASONIC SENSOR
const int trigPin3 = 2;
const int echoPin3 = 3;

// defines variables
long duration1, duration2, duration3;
int distance1, distance2, distance3;

int senseDistance1() {
  
  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin1, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication

  delay(50);
  
  // Clears the trigPin
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(5);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration1 = pulseIn(echoPin1, HIGH);
  
  // Calculating the distance
  distance1= duration1*0.034/2;
  
  // Prints the distance on the Serial Monitor
  Serial.println("First Distance in cm: ");
  Serial.println(distance1);

  return distance1*10;
}

int senseDistance2() {
  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin2, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication

  delay(200);
  
  // Clears the trigPin
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(5);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration2 = pulseIn(echoPin2, HIGH);
  
  // Calculating the distance
  distance2= duration2*0.034/2;
  
  // Prints the distance on the Serial Monitor
  Serial.println("Second Distance in cm: ");
  Serial.println(distance2);

  return distance2*10;
}

int senseDistance3() {
  pinMode(trigPin3, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin3, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication

  delay(200);
  
//  Clears the trigPin
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(5);
  
//  Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  
//  Reads the echoPin, returns the sound wave travel time in microseconds
  duration3 = pulseIn(echoPin3, HIGH);
  
  // Calculating the distance
  distance3= duration3*0.034/2;
  
//  Prints the distance on the Serial Monitor
  Serial.print("Third Distance in cm: ");
  Serial.println(distance3);

  return distance3*10;
}


//////////////////////////////////////////////////////////////////////////////////////////
double pulse1, inches1, cm1;

double xsenseDistance1() {
  //Used to read in the pulse that is being sent by the MaxSonar device. //Pulse Width representation with a scale factor of 147 uS per Inch.
  pulse1 = pulseIn(frontSensorPin, HIGH);
  //147uS per inch
  inches1 = pulse1/147;
  cm1 = inches1 * 2.54;
  Serial.print("Front sensor distance: ");
  Serial.print(cm1);
  Serial.print(" cm");
  Serial.println();
  delay(50);
  return cm1*10;
}

double pulse2, inches2, cm2;
double xsenseDistance2() {
  //Used to read in the pulse that is being sent by the MaxSonar device. //Pulse Width representation with a scale factor of 147 uS per Inch.
  pulse2 = pulseIn(rearSensorPin, HIGH);
  //147uS per inch
  inches2 = pulse2/147;
  cm2 = inches2 * 2.54;
  Serial.print("Rear sensor distance: ");
  Serial.print(cm2);
  Serial.print(" cm");
  Serial.println();
  delay(50);
  return cm2*10;
}

//double pulse3, inches3, cm3;
//double xsenseDistance3() {
//  //Used to read in the pulse that is being sent by the MaxSonar device. //Pulse Width representation with a scale factor of 147 uS per Inch.
//  pulse3 = pulseIn(rearSensorPin, HIGH);
//  //147uS per inch
//  inches3 = pulse3/147;
//  cm3 = inches3 * 2.54;
//  Serial.print("Rear sensor distance: ");
//  Serial.print(cm3);
//  Serial.print(" cm");
//  Serial.println();
//  delay(50);
//  return cm3*10;
//}

#define COMMON_ANODE
 
void setLedColor(int red, int green, int blue)
{
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  analogWrite(redLedPin, red);
  analogWrite(greenLedPin, green);
  analogWrite(blueLedPin, blue);  
}

/*
  Melody

 Plays a melody

 circuit:
 * 8-ohm speaker on digital pin 8

 created 21 Jan 2010
 modified 30 Aug 2011
 by Tom Igoe

This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/Tone

 */
#include "pitches.h"

// notes in the melody:
int melody1[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations1[] = {
  4, 8, 8, 4
};

void playStartTone() {
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 4; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations1[thisNote];
    tone(piezoPin, melody1[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(piezoPin);
  }
}

// notes in the melody:
int melody2[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations2[] = {
  4, 8, 8, 4
};

void playStopTone() {
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 4; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations2[thisNote];
    tone(piezoPin, melody2[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(piezoPin);
  }
}



