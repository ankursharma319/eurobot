#include <Wire.h>
#include <Servo.h>

#define MD25ADDRESS         0x58
#define SPEED1              0x00                        
#define SPEED2              0x01
#define ENCODERONE          0x02                              // Byte to read motor encoder 1
#define ENCODERTWO          0x06                              // Byte to read motor encoder 2
#define ACCELERATION        0xE                               // Byte to define motor acceleration
#define CMD                 0x10                              // Byte to reset encoder values
#define MODE_SELECTOR       0xF                               // Byte to change between control MODES

int MD25_Mode = 2;
int front_speed = 175;
int reverse_speed = 106;
int rotational_speed = 60;

int helperServoPin = 7;
int frontSensorPin = 2;
int rearSensorPin = 3;
int frontMotorPin = 5;
int rearMotorsPin = 6;
int redLedPin = 10;
int greenLedPin = 9;
int blueLedPin = 8;
int piezoPin = 11;
int directionModePin = 12;
int pullSwitchPin = 13;

unsigned long time_start;
int pullSwitchState = 0;
int directionMode = 0;
int angleH = 1; // servo position in degrees

Servo helperServo;

void setup(){
  Serial.begin(9600);
  Wire.begin();                                               // Begin I2C bus
  delay(100);                                                 // Wait for everything to power up
  setMD25Mode(MD25_Mode);
  
  encodeReset();
  pinMode(frontMotorPin, OUTPUT);
  pinMode(rearMotorsPin, OUTPUT);
  pinMode(pullSwitchPin, INPUT);
  pinMode(directionModePin, INPUT);
  pinMode(frontSensorPin, INPUT);
  pinMode(rearSensorPin, INPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);  
  helperServo.attach(helperServoPin);
  playStartTone();
  setLedColor(255, 0, 0);
  setHelperServo();
}

void loop(){
  pullSwitchState = digitalRead(pullSwitchPin);
  directionMode = digitalRead(directionModePin);
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
      delay(20000);
      straight(800, 150);
      delay(1000);
      auto_correction(750, encoder1(), encoder2(), 0, -1, -1);
      delay(1000);
      leftRot(43, -1);
      delay(1000);
      auto_correction(43, encoder1(), encoder2(), 3, -1, -1);
      delay(1000);
      straight(250, -1);
      delay(100);
      auto_correction(250, encoder1(), encoder2(), 0, -1, -1);
      delay(1000);
      leftRot(81, -1);
      delay(100);
      auto_correction(79, encoder1(), encoder2(), 3, -1, -1);
 
      delay(100);

      frontMotorStart();
      delay(3000);
      straight(700, 170);
      delay(1000);
      frontMotorStop();
      delay(1000);

      rightRot(45, -1);
      delay(500);
      rev(100, -1);
      delay(500);
      
      rearMotorsStart();
      delay(4000);
      openHelperServo();
      delay(1000);
      rearMotorsStop();
      delay(100);

      closeHelperServo();
      delay(100);

      //trip 2
      straight(200, -1);
      delay(100);
      frontMotorStart();
      delay(3000);
      straight(200, -1);
      delay(1000);
      frontMotorStop();
      delay(1000);

      rev(200, -1);
      delay(100);
      rightRot(2, -1);
      delay(100);
      rev(200, -1);
      delay(100);

      rearMotorsStart();
      delay(4000);
      openHelperServo();
      delay(1000);
      rearMotorsStop();
      delay(100);

      delay(10000000);
    }
    // BLUE SIDE MODE 
    else {
      delay(12000);
      straight(800, 150);
      delay(1000);
      auto_correction(750, encoder1(), encoder2(), 0, -1, -1);
      delay(1000);
      rightRot(50, -1);
      delay(1000);
      auto_correction(50, encoder1(), encoder2(), 2, -1, -1);
      delay(1000);
      straight(300, -1);
      delay(100);
      auto_correction(300, encoder1(), encoder2(), 0, -1, -1);
      delay(1000);
      rightRot(100, -1);
      delay(100);
      auto_correction(85, encoder1(), encoder2(), 2, -1, -1);
 
      delay(100);

      frontMotorStart();
      delay(3000);
      straight(650, 200);
      delay(1000);
      frontMotorStop();
      delay(1000);

      leftRot(55, -1);
      delay(500);
      rev(100, -1);
      delay(500);
      
      rearMotorsStart();
      delay(4000);
      openHelperServo();
      delay(1000);
      rearMotorsStop();
      delay(100);

      closeHelperServo();
      delay(100);

      //trip 2
      straight(900, -1);
      delay(100);
      frontMotorStart();
      delay(3000);
      straight(200, 200);
      delay(1000);
      frontMotorStop();
      delay(1000);

      rev(200, 80);
      delay(100);
      rightRot(2, -1);
      delay(100);
      rev(1200, -1);
      delay(100);

      rearMotorsStart();
      delay(4000);
      openHelperServo();
      delay(1000);
      rearMotorsStop();
      delay(100);
    }
  }
  delay(200);
}

void avoidCollision(bool useFrontSensorReading) {
  if(useFrontSensorReading){
    if(senseDistance1()<170) {
      halt();
      delay(1000);
      avoidCollision(true);
    }
  } else {
    if(senseDistance2()<170) {
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
    frontMotorStop();
    rearMotorsStop();
    setLedColor(200, 0, 0);
    playStopTone();
    delay(100000000);
    return true;
  } else {
    return false;
  }
}

void frontMotorStart() {
  digitalWrite(frontMotorPin, HIGH);
}

void rearMotorsStart() {
  digitalWrite(rearMotorsPin, HIGH);
}

void rearMotorTest() {
	for(int i=0; i<100; i++) {
		digitalWrite(rearMotorsPin, HIGH);
		delayMicroseconds(100);
		digitalWrite(rearMotorsPin, LOW);
		delayMicroseconds(100);
	}
}

void rearMotorsStop() {
  digitalWrite(rearMotorsPin, LOW);
}

void frontMotorStop() {
  digitalWrite(frontMotorPin, LOW);
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
  Serial.print(senseDistance1());
  Serial.print(senseDistance2());
  if (optional_speed<0) {
    optional_speed=reverse_speed;
  }
  unsigned long temp_time = millis();
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
  while(abs(encoder1())<distance and !isTimeUp()){
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

// ULTRASONIC CODE

double pulse1, inches1, cm1;
double senseDistance1() {
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
double senseDistance2() {
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

/*
Adafruit Arduino - Lesson 3. RGB LED
*/
 
 
//uncomment this line if using a Common Anode LED
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


// HELPER SERVO CODE

void openHelperServo()
{
  // scan from 0 to 180 degrees
  for(angleH = 0; angleH < 80; angleH++)
  {
    helperServo.write(angleH);
    delay(15);
  }

  delay(500);
}

void setHelperServo()
{
  // scan from 0 to 180 degrees
  for(angleH = 1; angleH < 2; angleH++)
  {
    helperServo.write(angleH);
    delay(15);
  }

  delay(500);
}


void closeHelperServo()
{
  // now scan back from 180 to 0 degrees
  for(angleH = 79; angleH > 1; angleH--)
  {
    helperServo.write(angleH);
    delay(15);
  }
  
  delay(500);
}
