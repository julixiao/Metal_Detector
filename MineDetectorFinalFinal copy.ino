#include <Servo.h>
#include <Adafruit_VC0706.h>
#include "HX711.h"
#define calibration_factor 1000 //This value is obtained using the SparkFun_HX711_Calibration sketch

// load cell
#define DOUT  3 //declaring load cell pins 
#define CLK  2
HX711 scale(DOUT, CLK);

// camera
Adafruit_VC0706 cam = Adafruit_VC0706(&Serial3);

// ultrasonics
const int trigPin1 = 9; //declaring ultrasonic sensor pins
const int echoPin1 = 10;
const int trigPin2 = 5;
const int echoPin2 = 4;

byte count = 0;

Servo crane; //crane servo
Servo servo1; //inner CONTINUOUS servo
Servo servo2; //outer CONTINUOUS servo

//int data[15] = {0, 1, 250, 150, 1, 30, 40, 0, 0, 0, 0, 0, 0, 0, 0};
int data[15];
int canLidPos[5][2] = {{0,0}, {0,0}, {0,0}, {0,0}, {0,0}};

float lastWeight;

long duration, duration1, duration2;
long dfront, dright;
//int turnCount = 0;
int turnCount = 0;
int dperfect = 10;
//int dperfect = 30;
int usDelay = 700;
int d180 = 200;
int instruction;
int lidCountInt = 0;
int turning = 0;

unsigned long lastTime;

boolean running;

const int servoPin = 7; //declaring crane servo pin

const byte npulse = 3;  //cdeclaring values needed for detection(); two for each for both detection coils
const bool debug = true;
const byte pin_pulse = A0; //pulse pins
const byte pin_cap  = A1; //capacitor pins
const byte pin_pulse2 = A4;
const byte pin_cap2 = A3;
const int nmeas = 256; //measurements to take
long int sumsum = 0; //running sum of 64 sums
long int skip = 0; //number of skipped sums
long int diff = 0;      //difference between sum and avgsum
long int flash_period = 0; //period (in ms)
long int sumsum2 = 0;
long int flash_period2 = 0;
long int diff2 = 0;
long int skip2 = 0;
byte lidCountByte = 0; //counter for number of metal readings detected

float weight;

//2D array for 7-segment display
byte seven[10][7] = { { 0, 0, 0, 0, 0, 0, 1 }, // = 0
  { 1 , 0, 0, 1, 1, 1, 1 }, // = 0
  { 0, 0, 1, 0, 0, 1, 0 }, // = 2
  { 0, 0, 0, 0, 1, 1, 0 }, // = 3
  { 1, 0, 0, 1, 1, 0, 0 }, // = 4
  { 0, 1, 0, 0, 1, 0, 0 }, // = 5
  { 0, 1, 0, 0, 0, 0, 0 }, // = 6
  { 0, 0, 0, 1, 1, 1, 1 }, // = 7
  { 0, 0, 0, 0, 0, 0, 0 }, // = 8
  { 0, 0, 0, 1, 1, 0, 0 } // = 9
};

const int a = 30; //declaring 7-segemnet display pin
const int b = 31;
const int c = 28;
const int d = 26;
const int e = 22;
const int f = 27;
const int g = 24;
const int dp = 29;
const int pow3 = 23;
const int pow8 = 25;

void setup() {
  Serial.begin(9600);
  Serial1.begin(230400);
  Serial3.begin(230400);

  Serial.println("I'M IN SETUP STILL");

  Serial.println(turnCount);

  //servo1.attach(9);
  servo1.attach(11);//right wheel, turns cw
  servo2.attach(13);//left wheel, turns ccw

  Serial.println("servo pins are attached");

  //trigPin is output
  pinMode(trigPin1, OUTPUT);
  //echoPin is input
  pinMode(echoPin1, INPUT);

  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  running = true;
  Serial.print(running);

  Serial.println(turnCount);

  delay(200);

  if (debug) Serial.begin(9600);
  pinMode(pin_pulse, OUTPUT);
  digitalWrite(pin_pulse, LOW);
  pinMode(pin_pulse2, OUTPUT);
  digitalWrite(pin_pulse2, LOW);

  pinMode(pin_cap, INPUT);
  pinMode(pin_cap2, INPUT);

  crane.attach(servoPin);

  pinMode(pow3, OUTPUT);
  pinMode(pow8, OUTPUT);
  digitalWrite(pow3, HIGH);
  digitalWrite(pow8, HIGH);

  pinMode(a, OUTPUT);
  pinMode(b, OUTPUT);
  pinMode(c, OUTPUT);
  pinMode(d, OUTPUT);
  pinMode(e, OUTPUT);
  pinMode(f, OUTPUT);
  pinMode(g, OUTPUT);
  pinMode(dp, OUTPUT);
  digitalWrite(29, 1);

  scale.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale.tare(); //Assuming there is no weight on the scale at start up, reset the scale to 0

  displayNum(0);
  crane.write(90);

  digitalWrite (trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  dfront = duration1 * 0.034 / 2;
  Serial.print("Dfront: ");
  Serial.println(dfront);

  digitalWrite (trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  dright = duration2 * 0.0343 / 2;
  Serial.print("dperfect: ");
  Serial.println(dperfect);
  Serial.print("dright: ");
  Serial.println(dright);

  data[0] = turning;
  data[1] = turnCount;
  data[2] = dright;
  data[3] = dfront;
  data[4] = lidCountInt;

}

boolean detection () { //method commanding the two metal detectors and returning boolean value; true if metal is detected, false if no metal

  int minval = 1023;
  int maxval = 0;
  int minval2 = 1023;
  int maxval2 = 0;

  //perform measurement
  long unsigned int sum = 0;
  long unsigned int sum2 = 0;
  for (int imeas = 0; imeas < nmeas + 2; imeas++) {
    //reset the capacitor
    pinMode(pin_cap, OUTPUT);
    digitalWrite(pin_cap, LOW);
    pinMode(pin_cap2, OUTPUT);
    digitalWrite(pin_cap2, LOW);

    delayMicroseconds(20);
    pinMode(pin_cap, INPUT);
    pinMode(pin_cap2, INPUT);
    //apply pulses
    for (int ipulse = 0; ipulse < npulse; ipulse++) {
      digitalWrite(pin_pulse, HIGH); //takes 3.5 microseconds
      digitalWrite(pin_pulse2, HIGH);
      delayMicroseconds(3);
      digitalWrite(pin_pulse, LOW); //takes 3.5 microseconds
      digitalWrite(pin_pulse2, LOW);
      delayMicroseconds(3);

    }
    //read the charge on the capacitor
    int val = analogRead(pin_cap); //takes 13x8=104 microseconds
    int val2 = analogRead(pin_cap2);
    minval = min(val, minval);
    minval2 = min(val2, minval2);
    maxval = max(val, maxval);
    maxval2 = max(val2, maxval2);
    sum += val;
    sum2 += val2;

    delayMicroseconds(1000);

  }

  //subtract minimum and maximum value to remove spikes
  sum -= minval; sum -= maxval;
  sum2 -= minval2; sum2 -= maxval2;

  //process
  if (sumsum == 0) sumsum = sum << 6; //set sumsum to expected value
  long int avgsum = (sumsum + 32) >> 6;
  diff = sum - avgsum;
  if (abs(diff)<avgsum >> 10) {   //adjust for small changes
    sumsum = sumsum + sum - avgsum;
    skip = 0;
  } else {
    skip++;
  }
  if (skip > 64) {  // break off in case of prolonged skipping
    sumsum = sum << 6;
    skip = 0;
  }
  if (sumsum2 == 0) sumsum2 = sum2 << 6; //set sumsum to expected value
  long int avgsum2 = (sumsum2 + 32) >> 6;
  diff2 = sum2 - avgsum2;
  if (abs(diff2)<avgsum2 >> 10) {   //adjust for small changes
    sumsum2 = sumsum2 + sum2 - avgsum2;
    skip2 = 0;
  } else {
    skip2++;
  }
  if (skip2 > 64) {  // break off in case of prolonged skipping
    sumsum2 = sum2 << 6;
    skip2 = 0;
  }

  if (diff == 0) flash_period = 1000000;
  else flash_period = avgsum / (2 * abs(diff));
  if (diff2 == 0) flash_period2 = 1000000;
  else flash_period2 = avgsum2 / (2 * abs(diff2));

  if (abs(diff) > 230 && abs(diff) < 1500 && flash_period < 25 && flash_period > 0) {
    return true;
  }
  else if (abs(diff2) > 450 && abs(diff2) < 1500 && flash_period2 < 25 && flash_period2 > 1) {
    return true;
  }
  else {
    return false;
  }

  if (debug) {
    Serial.print(nmeas);
    Serial.print(" ");
    Serial.print(minval);
    Serial.print(" ");
    Serial.print(maxval);
    Serial.print(" ");
    Serial.print(sum);
    Serial.print(" ");
    Serial.print(avgsum);
    Serial.print(" ");
    Serial.print(diff);
    Serial.print(" ");
    Serial.print(flash_period);
    Serial.println();

    Serial.print(nmeas);
    Serial.print(" ");
    Serial.print(minval2);
    Serial.print(" ");
    Serial.print(maxval2);
    Serial.print(" ");
    Serial.print(sum2);
    Serial.print(" ");
    Serial.print(avgsum2);
    Serial.print(" ");
    Serial.print(diff2);
    Serial.print(" ");
    Serial.print(flash_period2);
    Serial.println();
    Serial.println();
  }
}
void displayNum(int num) //method that accepts num (nummber you want displayed) and displays number on 7-segment display
{

  digitalWrite(a, seven[num][0]); // a
  digitalWrite(b, seven[num][1]); // b
  digitalWrite(c, seven[num][2]); // c
  digitalWrite(d, seven[num][3]); // d
  digitalWrite(e, seven[num][4]); // e
  digitalWrite(f, seven[num][5]); // f
  digitalWrite(g, seven[num][6]); //g

}

void pickup() //method commanding movmement of rod magnet from down to detachment of can lid, then returning to initial position (80 degrees)
{
  for (int angle = 5; angle < 170; angle += 5) // command to move from 0 degrees to 180 degrees
  {
    crane.write(angle);                 //command to rotate the servo to the specified angle
    delay(25);
  }

  delay(1000);
  crane.write(80);
}

void lower() { //method commanding rod to lower to lowest angle in order to retrieve can lid
  for (int angle = 80; angle > 5; angle -= 5) {
    crane.write(angle);
    delay(15);
  }
}

void backup() { //method commanding cart to back-up from current position
  int duration1, duration2;
  int backupInitial, dfront, dright;

  //trigPin is output
  pinMode(trigPin1, OUTPUT);
  //echoPin is input
  pinMode(echoPin1, INPUT);

  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  servo1.attach(11);//right wheel, turns cw
  servo2.attach(13);//left wheel, turns ccw

  straighten();

  servo1.detach();
  servo2.detach();

  digitalWrite (trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  dfront = duration1 * 0.034 / 2;
  Serial.print("Dfront: ");
  Serial.println(dfront);

  backupInitial = dfront;

  Serial.print("backupInitial: ");
  Serial.println(backupInitial);

  servo1.attach(11);//right wheel, turns cw
  servo2.attach(13);//left wheel, turns ccw

  while ((dfront - backupInitial) <= 27) {
    servo1.detach();
    servo2.detach();

    digitalWrite (trigPin1, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin1, LOW);
    duration1 = pulseIn(echoPin1, HIGH);
    dfront = duration1 * 0.034 / 2;
    Serial.print("Dfront: ");
    Serial.println(dfront);

    servo1.attach(11);//right wheel, turns cw
    servo2.attach(13);

    servo1.write(120);
    servo2.write(60);

    delay(100);
  }

  servo1.write(90);
  servo2.write(90);


}

void straighten() {
  Serial.print("ENTERING");

  int dFrontMin, dFrontInitial, servo2Speed, servo1Speed, straighteningDelay, equalCounter, knockbackDelay;

  straighteningDelay = 130;
  knockbackDelay = 130;

  servo1Speed = 70;
  servo2Speed = 90;

  servo1.detach();
  servo2.detach();

  delay(100);

  digitalWrite (trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  dfront = duration1 * 0.034 / 2;
  Serial.print("Dfront: ");
  Serial.println(dfront);

  dFrontInitial = dfront;

  servo1.attach(11);//right wheel, turns cw
  servo2.attach(13);//left wheel, turns ccw

  servo1.write(servo1Speed);
  servo2.write(servo2Speed);

  delay(straighteningDelay);

  servo1.detach();
  servo2.detach();

  delay(100);

  digitalWrite (trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  dfront = duration1 * 0.034 / 2;
  Serial.print("Dfront: ");
  Serial.println(dfront);

  servo1.attach(11);//right wheel, turns cw
  servo2.attach(13);//left wheel, turns ccw

  if (dfront < dFrontInitial) {
    dFrontMin = dfront;
  }
  else if (dfront > dFrontInitial) {
    servo1Speed = 90;
    servo2Speed = 100;

    servo1.write(servo1Speed);
    servo2.write(servo2Speed);

    delay(straighteningDelay);

    servo1.detach();
    servo2.detach();

    delay(100);

    digitalWrite (trigPin1, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin1, LOW);
    duration1 = pulseIn(echoPin1, HIGH);
    //pulseIn(echoPin1, LOW);
    dfront = duration1 * 0.034 / 2;
    Serial.print("Dfront: ");
    Serial.println(dfront);

    dFrontMin = dfront;

    servo1.attach(11);//right wheel, turns cw
    servo2.attach(13);//left wheel, turns ccw
  }

  servo1.write(servo1Speed);
  servo2.write(servo2Speed);

  delay(straighteningDelay);

  servo1.detach();
  servo2.detach();

  delay(100);

  digitalWrite (trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  //pulseIn(echoPin1, LOW);
  dfront = duration1 * 0.034 / 2;
  Serial.print("Dfront: ");
  Serial.println(dfront);

  servo1.attach(11);//right wheel, turns cw
  servo2.attach(13);//left wheel, turns ccw

  equalCounter = 0;

  while (dfront <= dFrontMin) {
    Serial.println("In backing up while loop");
    if (dfront == dFrontMin) {
      equalCounter++;
    }
    else {
      equalCounter = 0;
    }

    dFrontMin = dfront;

    servo1.write(servo1Speed);
    servo2.write(servo2Speed);

    delay(straighteningDelay);

    servo1.detach();
    servo2.detach();

    delay(100);

    digitalWrite (trigPin1, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin1, LOW);
    duration1 = pulseIn(echoPin1, HIGH);
    //pulseIn(echoPin1, LOW);
    dfront = duration1 * 0.034 / 2;
    Serial.print("Dfront: ");
    Serial.println(dfront);

    servo1.attach(11);//right wheel, turns cw
    servo2.attach(13);//left wheel, turns ccw
  }

  Serial.print("equalCounter: ");
  Serial.print(equalCounter);

  if (servo1Speed == 70) {
    servo1.write(90);
    servo2.write(100);
  }
  else {
    servo1.write(70);
    servo2.write(90);
  }
  delay(knockbackDelay);

  for (int i = 0; i <= (equalCounter - 1) / 2; i++) {
    if (servo1Speed == 70) {
      servo1.write(90);
      servo2.write(100);
    }
    else {
      servo1.write(70);
      servo2.write(90);
    }
    delay(knockbackDelay);
  }

  Serial.print("DONE");

  servo1.write(90);  
  servo2.write(90);

}

double dturning180 (Servo servo1, Servo servo2, int s1speed, int s2speed) {

  servo1.detach();
  servo2.detach();

  delay(300);

  digitalWrite (trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  dfront = duration1 * 0.034 / 2;
  Serial.print("Dfront: ");
  Serial.println(dfront);
  delay(usDelay);

  servo1.attach(11);//right wheel, turns cw
  servo2.attach(13);

  return dfront;

}

double rightTurn180(int dfront, int turncount) {

  int dperfect = 98;
  servo1.write(90);
  servo2.write(180);
  delay(2200);

  Serial.println(" right turn 180 Enters turning loop here" + turnCount);
  double dright = dturning180(servo1, servo2, 90, 180);

  Serial.println("Stops turning here");
  while (dright > dperfect + 2 || dright < dperfect - 2)
  {
    Serial.println("right turn 180");

    servo1.write(90);
    servo2.write(180);
    delay(100);

    dright = dturning180(servo1, servo2, 90, 180);

    Serial.print("dPerfect: ");
    Serial.println(dperfect);
  }
  Serial.println("Expected distance: " + ((turncount - 4) * 20 + 10));
  return (turncount) * 20 + 10;
}

double leftTurn180(int dfront, int turncount) {

  int dperfect = 98;
  servo1.write(0);
  servo2.write(90);
  delay(3000);
  
  Serial.println("Enters turning loop here");
  double dright = dturning180(servo1, servo2, 0, 90);

  Serial.println("Stops turning here");
  while (dright > dperfect + 2 || dright < dperfect - 2)
  {
    Serial.println("left turn 180");

    servo1.write(0);
    servo2.write(90);
    delay(100);

    dright = dturning180(servo1, servo2, 80, 90);

    Serial.print("dPerfect: ");
    Serial.println(dperfect);
  }
  return 300 - ((turncount + 1) * 20 + 10);

}

double oppositeLeftTurn180(int dfront, int turncount) {

  int dperfect = 98;
  servo1.write(180);
  servo2.write(90);
  //delay(2000);
  delay(2300);


  Serial.println("Enters turning loop here");
  double dright = dturning180(servo1, servo2, 0, 90);

  Serial.println("Stops turning here");
  while (dright > dperfect + 4 || dright < dperfect - 4)
  {
    Serial.println("left turn 180");

    servo1.write(180);
    servo2.write(90);
    delay(100);

    dright = dturning180(servo1, servo2, 80, 90);

    Serial.print("dPerfect: ");
    Serial.println(dperfect);
  }
  return 300 - ((turncount + 1) * 20 + 10);
}

int leftTurn90(int dright, int turncount, int dperfect) {
  servo1.write(0);
  servo2.write(90);
  delay(600);

  while (dright > dperfect + 2  || dright < dperfect - 2)
  {
    Serial.println("Here still turning");
    Serial.print("dperfect: ");
    Serial.println(dperfect);

    servo1.write(0);
    servo2.write(90);
    delay(100);

    servo1.detach();
    servo2.detach();

    digitalWrite (trigPin2, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin2, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin2, LOW);
    duration2 = pulseIn(echoPin2, HIGH);
    dright = duration2 * 0.034 / 2;
    Serial.print("dright: ");
    Serial.println(dright);

    servo1.attach(11);//right wheel, turns cw
    servo2.attach(13);

  }

  int out = 30;
  return out;
}

void stopOperation() {
  servo1.write(90);
  servo2.write(90);
  delay(2000);
}

void movement() {

  Serial.println("I'm in the movement method");

  Serial.println();
  Serial.print("turnCount: ");
  Serial.println(turnCount);
  Serial.print("dperfect: ");
  Serial.println(dperfect);

  if (turnCount >= 15 && turnCount <= 19 && (millis() - lastTime) % 1600 < 120 && dperfect > 10) {
    lastTime = millis();
    dperfect -= 5;
  }

  if (dright == dperfect) {
    servo1.write(30);
    servo2.write(150);
  }
  else if (dright > dperfect - 20 && dright < dperfect + 20 && dright > dperfect)
  {
    servo1.write(85);
    servo2.write(180);
    Serial.println("Smaller");

  }
  else if (dright > dperfect - 20 && dright < dperfect + 20 && dright < dperfect)
  {
    servo1.write(0);
    servo2.write(95);
    Serial.println("Bigger");

  }

  delay(80);

  if (dfront <= 20) {
    Serial.println("turning");
    turning = 1;
    turnCount++;
    Serial.println(turnCount);

    servo1.write(90);
    servo2.write(90);

    if (turnCount % 2 == 0 && turnCount < 14) {
      dperfect = rightTurn180(dfront, turnCount);
    }
    else if (turnCount % 2 != 0 && turnCount != 13 && turnCount < 14) {
      dperfect = leftTurn180(dfront, turnCount);
    }
    else if (turnCount == 13) {
      dperfect = oppositeLeftTurn180(dfront, turnCount);
    }
    else if (turnCount == 14) {
      dperfect = leftTurn90(dright, turnCount, 35);
    }
    else if (turnCount >= 15 && turnCount <= 19) {
      Serial.println("LEFT TURN 90!");
      dperfect = leftTurn90(dright, turnCount, 35);
      lastTime = millis();
      Serial.println("lastTime defined!!!!");
    }
    else if (turnCount == 20) {
      stopOperation();
      running = false;
    }
    turning = 0;
    Serial1.read();
  }

}

void bluetooth() {
  Serial.println("in bluetooth");
  if (Serial1.available()) {
    servo1.write(90);
    servo2.write(90);

    instruction = Serial1.read();
    Serial.print("instruction: ");
    Serial.println(instruction);

    if (instruction == 49) {

      cam.begin();
      delay(500);
      cam.setImageSize(VC0706_160x120);
      cam.takePicture();

      delay(1000);
      uint16_t jpglen = cam.frameLength();
      Serial.print("jpglen after delay: ");
      Serial.println(jpglen);

      int32_t time = millis();
      while (jpglen > 0) {
        uint8_t *buffer;
        uint8_t bytesToRead = min(64, jpglen);
        buffer = cam.readPicture(bytesToRead);
        for (int i = 0; i < bytesToRead; i++) {
          Serial1.write(buffer[i]);
        }

        jpglen -= bytesToRead;
      }

      time = millis() - time;
      Serial.println("done!");
      Serial.print(time); Serial.println(" ms elapsed");
    }
    else if (instruction == 48) {
      data[0] = turning;
      data[1] = turnCount;
      data[2] = dright;
      data[3] = dfront;
      data[4] = lidCountInt;

      for (int i = 0; i < 15; i++) {
        Serial1.write(data[i]);
      }
    }
  }
}

void sideToSide() {
  servo1.write(90);
  servo2.write(90);

  //first to the left
  servo1.write(70);
  servo2.write(90);
  delay (800);

  servo1.write(110);
  servo2.write(90);
  delay (800);

  //then to the right
  servo1.write(90);
  servo2.write(110);
  delay (800);

  servo1.write(90);
  servo2.write(70);
  delay (800);

  servo1.write(90);
  servo2.write(90);
}

void loop() {

  Serial.println("Getting into the normal loop");
  Serial.println(running);

  boolean isMetal, moveCrane = true;
  float weight;

  while (running) {
    ("Getting into WHILE loop");
    bluetooth();
    lastWeight = scale.get_units();

    servo1.detach();
    servo2.detach();

    digitalWrite (trigPin2, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin2, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin2, LOW);
    duration2 = pulseIn(echoPin2, HIGH);

    dright = duration2 * 0.034 / 2;

    Serial.print("dright: ");
    Serial.println(dright);

    delay(100);

    digitalWrite (trigPin1, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin1, LOW);
    duration1 = pulseIn(echoPin1, HIGH);
    //pulseIn(echoPin1, LOW);
    dfront = duration1 * 0.034 / 2;
    Serial.print("Dfront: ");
    Serial.println(dfront);

    servo1.attach(11);//right wheel, turns cw
    servo2.attach(13);

    servo1.write(90);
    servo2.write(90);
    
    movement();
    
        for (int x = 0; x < 5; x++)//Adjust VALUE HERE
        {
          isMetal = detection();
          // Serial.println(isMetal);
          if (isMetal) {
            count++;
          }
          else
            count = 0;
        }
        if (count > 3) {
          moveCrane = true;
          count = 0;
        }
        else {
          moveCrane = false;
          count = 0;
        }

        if (moveCrane) {
          servo1.write(90);
          servo2.write(90);
          lower();
          backup();
          sideToSide();
          pickup();
        }

        delay(1000);

        weight = scale.get_units();
        if (abs(lastWeight - weight) > 0.3) {

          int xPos, yPos;

          if (turnCount % 2 == 0 && turnCount < 15) { //on an odd column
            xPos = dright + 10 + 10;// + 10 to centre the image of the cart
            yPos = 300 - dfront - 20;// - 200 to centre the image of the cart
          } else if (turnCount % 2 != 0 && turnCount < 15) { // on an even column
            xPos = 300 - dright - 10;
            yPos = dfront + 20;
          } else if (turnCount == 0 || turnCount == 15 || turnCount == 19) { //last left 90 turn or the car has stopped
            xPos = dfront + 20;
            yPos = dright + 10 + 10;
          } else if (turnCount == 16 || turnCount == 18) {
            xPos = dright + 10 + 10;
            yPos = 300 - dfront - 20;
          } else if (turnCount == 17) {
            xPos = 300 - dfront - 20;
            yPos = 300 - dright - 10;
          }

          data[5 + 2 * lidCountInt] = xPos;
          data[5 + 2 * lidCountInt + 1] = yPos;

          lidCountByte++;
          lidCountInt++;

          displayNum(lidCountByte);
        }
        Serial.println("Here2");
  }
}

