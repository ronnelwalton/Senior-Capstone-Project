/* -------------------------------------------------------------------------
 * Created by Ronnel Walton
 * 
 * This program runs the modified EELE 101 car's Arduino UNO.
 * This program was created for the sole purpose of 
 * Capstone Project.
 * This program is written in a way that will showcase the end result
 * of being part of EELE department's degree program.
 * 
 * This program will add the following features
 * in addition to the original EELE 101 car:
 * 
 * 1. Speed/PWM control
 * 2. Serial Communication between Arduino UNO and ESP32
 * 3. Distance Measurement
 * 4. Distance following capabilities
 * 5. Enhanced QTI Line follower algorithm
 * 
 * The Pin diagram for the connection is included in a different document.
 * Please make sure these connections are followed correctly to
 * ensure proper operation of the car.
 * 
 * Additional information regarding the capstone can be found in
 * the final report.
 * Team ACT 65:
 * Members
 * - Brent Bergman (EE)
 * - Nikolette Sabiers (EE)
 * - Dee Vang (EE)
 * - Ronnel Walton (CpE)
 * -------------------------------------------------------------------------
 */


// Imports
#include <Servo.h>

// Pins
#define trigPin 10
#define echoPin 11
#define rMotorPin 12
#define lMotorPin 13
#define farLeftSensorPin A3
#define leftSensorPin A1
#define rightSensorPin A0
#define farRightSensorPin A2
#define buzzerPin 4

// Servos
Servo leftServo;
Servo rightServo;


// Constants
const bool calibrate = false;
const int backwards = 1; // Servo are places backwards = 1, normal -1
const int upperDistance = 50;
const int pulseInterval = 20;
const int trigDist = 30;
const int upperPWM = 2250; const int lowerPWM = 750; // Datasheet
const int midPWM = (upperPWM + lowerPWM) / 2;
const int maxSpeed = 200; // Based on RPM graph from book (PWM)
const int minSpeed = 0;
const float inSpeed = 0.3475;
const float hInSpeed = 0.2925; 
const float outSpeed = 1.0;
const float minDistance = 12;
const float targetDistance = 15;
const bool fourSensors = true;
const int resolution = 5;

long mistake = 0;
long statCount = 0;
bool lineFlagG = false;
bool following = false;

// Globals
int curIndex = 0;
bool initialData = false;
float distanceData[resolution];
float prevDist = 0;

// Functions Declarations
float distanceMeasurement(long *distanceTimer, float *distanceReturn, bool doubleCheck);
int cmpFunc(const void* elem1, const void* elem2);
float medianFilter(long *distanceTimer);
float optimizedDistance (long *distanceTimer);
float averageDistance();
void serialCom(bool serialEN, float distance, long *sendTimer, float *prevSerial, int *tries);
int QTIsensor(byte sensor);
void sensorRead(bool *sensorL, bool *sensorR, bool *sensorFL, bool *sensorFR);
float speedControl(float *speedMulti, float distance, float minDistance, float targetDistance);
void joinLine(float curDistance, float trigDist, bool *lineFlag);
void forward(float speedMulti);
void halt();
void hardRight(float speedMulti);
void hardLeft(float speedMulti);
void gradualRight(float speedMulti);
void gradualLeft(float speedMulti);
void steerR(int midPWM, int maxSpeed);
void steerL(int midPWM, int maxSpeed);
void drive(long *pulseTimer, bool *needToChangeDirection, float distance, float minDistance, float targetDistance, float *speedMulti);
void initialBeep(int pin);
void statTest(float distance);


// Main Functions ------------

// Setup/Initializations
void setup() {
  Serial.begin(9600);
  initialBeep(buzzerPin);
  while(!Serial.availableForWrite()){
    delay(1); 
  }
  Serial.print("0S");
  
  leftServo.attach(lMotorPin);
  rightServo.attach(rMotorPin);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  if (fourSensors){
    pinMode(rightSensorPin, INPUT);
    pinMode(leftSensorPin, INPUT);
    pinMode(farRightSensorPin, INPUT);
    pinMode(farLeftSensorPin, INPUT);
  } else {
    pinMode(rightSensorPin, INPUT);
    pinMode(leftSensorPin, INPUT);
  }
  
  halt(midPWM);
  delay(1000);
  return;
}

// Main Loop
void loop() {
  // Variables
  const bool serialEN = true;
  float distance = 0;
  float speedMulti = 0.4; // Default
  float prevSerial = 0;
  int tries = 0;
  bool needToChangeDirection;
  bool lineFlag = false;


  // Timers
  long sendTimer = millis();
  long pulseTimer = millis();
  long distanceTimer = millis();

  while(true) {
    lineFlagG = lineFlag;
    if (calibrate) {
      halt(midPWM);
    } else {
      distance = optimizedDistance(&distanceTimer);
      if (!lineFlag){
        joinLine(&distanceTimer, &distance, trigDist, &lineFlag);
      } 
      else {
        drive(&pulseTimer, &needToChangeDirection, distance, minDistance, targetDistance, &speedMulti);  
        // See if measurement was sent through before sending another in an interval
        serialCom(serialEN, distance, &sendTimer, &prevSerial, &tries); 
      }
    }
  }
  return;
}


// End Main Functions --------------------------------------------------


// ---------------------------------------------------------------------
// Function Definitions 
// ---------------------------------------------------------------------

void statTest(float distance){
  if (lineFlagG && !following && ((distance > 12) && (distance < 18))) {
    following = true;
  }
  else if (lineFlagG && following) {
    if (((distance > 18) || (distance < 12))) {
      mistake++;
    }
    statCount++;
    Serial.println("Distance Count: " + (String)statCount + " Out of Bound: " + (String)mistake);
  }
  return;
}

float optimizedDistance (long *distanceTimer) {
  float median = medianFilter (distanceTimer);
  float movingAvg = averageDistance();
  float medianDif = abs(median - prevDist);
  float movingAvgDif = abs(movingAvg - prevDist);
  if (medianDif < movingAvgDif) {
    //statTest(median);
    return median;
  } else {
    //statTest(movingAvg);
    return movingAvg;
  }
}

float averageDistance() {
  float sum = 0;
  for (int i = 0; i < resolution; i++) {
    sum += distanceData[i];
  }
  return (sum/resolution);
}

// Median Filter
float medianFilter(long *distanceTimer) {
  if (!initialData) {
    initialData = true;
    for (int i = 0; i < resolution; i++) {
      distanceMeasurement(distanceTimer, &distanceData[i], true);
    }
    qsort(distanceData, resolution, sizeof(distanceData[0]), cmpFunc);
    prevDist = distanceData[(int)(resolution/2)];
    return distanceData[(int)(resolution/2)];
  }
  if (resolution == curIndex) {
    curIndex = 0;
  }
  distanceMeasurement(distanceTimer, &distanceData[curIndex], true);
  qsort(distanceData, resolution, sizeof(distanceData[0]), cmpFunc);
  curIndex++;
  return distanceData[(int)(resolution/2)];
}

int cmpFunc(const void* elem1, const void* elem2)
{
    if(*(const float*)elem1 < *(const float*)elem2)
        return -1;
    return *(const float*)elem1 > *(const float*)elem2;
}

// Distance Measurement of HC-SR04
float distanceMeasurement(long *distanceTimer, float *distanceReturn, bool doubleCheck) {
  const int timeoutEcho = (2 * ( upperDistance * 2 / 0.0343)) + 60;
  if (((distanceTimer + 20) < millis()) || !doubleCheck) {
    *distanceTimer = millis();
    long duration;
    float distance;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH, timeoutEcho);
    // Calculating the distance
    distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
    // Serial.println(distance);
    if (doubleCheck && ((distance > (*distanceReturn + 3)) || (distance < (*distanceReturn - 3)))) {
      distanceMeasurement(distanceTimer, distanceReturn, false);
    }
    if ((distance > 2) && (distance <= upperDistance)) {
      *distanceReturn = distance;
    } else {
      *distanceReturn = upperDistance;
    }
  }
  return *distanceReturn;
}

// Serial Communication between Arduino UNO and ESP32
void serialCom(bool serialEN, float distance, long *sendTimer, float *prevSerial, int *tries) {
  const int delaySend = 50;
  if (true) {
    if ((Serial.availableForWrite()) && ((*sendTimer + delaySend) < millis())) {
      *sendTimer = millis();
      String msg = (String)distance;
      if (distance == upperDistance) {
        msg = "-1";
      } else if (((*prevSerial - 3) < distance) || ((*prevSerial + 3) > distance) || (tries > 5)) {
        Serial.print(msg + "S");
        *prevSerial = distance;
        *tries = 0;
      } else {
        *tries++;
      }
    }
  }
  return;
}

// QTI Sensor ----
int QTIsensor(byte sensor) {
  digitalWrite(sensor, HIGH);
  return analogRead(sensor);
}
// Read all 4 QTI Sensors
void sensorRead(bool *sensorL, bool *sensorR, bool *sensorFL, bool *sensorFR) {
  const int threshold = 500;
  if (fourSensors){
    int readFL = QTIsensor(farLeftSensorPin);
    int readL = QTIsensor(leftSensorPin);
    int readR = QTIsensor(rightSensorPin);
    int readFR = QTIsensor(farRightSensorPin);
    *sensorL = readL > threshold;
    *sensorR = readR > threshold;
    *sensorFL = readFL > threshold;
    *sensorFR = readFR > threshold;
  } else {
    int readL = QTIsensor(leftSensorPin);
    int readR = QTIsensor(rightSensorPin);
    *sensorL = readL > threshold;
    *sensorR = readR > threshold;
  }
  return;
}
// End QTI Sensor ----


// Servo Control  ------

// Speed/RPM Control for Servos
float speedControl(float *speedMulti, float distance, float minDistance, float targetDistance) {
  // Boundaries
  const float maxSpeed = 0.28;
  const float slowSpeed = 0.17;
  if (*speedMulti < 0){
    *speedMulti = 0;
  }
  if (*speedMulti > maxSpeed) {
    *speedMulti = maxSpeed;
  }

  // Algorithm for speed control
  if ((distance <= (minDistance + 1))) {
    *speedMulti = 0;
  } else if (distance <= (targetDistance)) {
    *speedMulti = slowSpeed;
  }
  else if (distance > (targetDistance)) {
    *speedMulti = maxSpeed;
  } 
  return *speedMulti;
}

// Joining track algorithm
void joinLine(long *distanceTimer, float *curDistance, float trigDist, bool *lineFlag) {
  float mergeS = 0.8;// Merge Speed
  if ((*curDistance < trigDist) && (*curDistance > 0)) {
    *lineFlag = true;
    long startTime= millis();
    long joinTimer = 2000;
    while ((startTime + joinTimer) >= millis()) {
      *curDistance = optimizedDistance(distanceTimer); 
      if ((*curDistance < trigDist) && (*curDistance > 0)) {
        startTime = millis();
      }
    }
    Serial.print("1S");
    merge(mergeS);
    
  }
  return;
}

void merge(float speedMulti){
  bool sensorL, sensorR, sensorFR, sensorFL;
  sensorRead(&sensorL, &sensorR, &sensorFL, &sensorFR);
  float mergeSpeedMulti = backwards * speedMulti;
  while((!sensorR && !sensorL)||(sensorR && sensorL)) {
    sensorRead(&sensorL, &sensorR, &sensorFL, &sensorFR);
    forward(mergeSpeedMulti);
  }
  if (sensorR){
    while(sensorR && !sensorL) {
      sensorRead(&sensorL, &sensorR, &sensorFL, &sensorFR);
      forward(mergeSpeedMulti);
    }
  } else if (sensorL) {
    while(sensorL && !sensorR){
      sensorRead(&sensorL, &sensorR, &sensorFL, &sensorFR);
      forward(mergeSpeedMulti);
    }
  }
  long forwardTime = millis();
  long forwardTimeThreshold = 1000;
  while((forwardTime + forwardTimeThreshold) > millis()){
    sensorRead(&sensorL, &sensorR, &sensorFL, &sensorFR);
    if (sensorR && !sensorL) {
      if (backwards == 1) {
        steerL(midPWM, maxSpeed);
      } else {
        steerR(midPWM, maxSpeed);
      }
    } else if (sensorL && !sensorR) {
      if (backwards == 1) {
        steerR(midPWM, maxSpeed);
      } else {
        steerL(midPWM, maxSpeed);
      }
    } else{
      forward(mergeSpeedMulti);
    }
  }
}

// Line Follower Algorithm
void drive(long *pulseTimer, bool *needToChangeDirection, float distance, float minDistance, float targetDistance, float *speedMulti) {
  bool sensorL, sensorR, sensorFR, sensorFL;
  float driveSpeedMulti = speedControl(speedMulti, distance, minDistance, targetDistance) * backwards;
  
  sensorRead(&sensorL, &sensorR, &sensorFL, &sensorFR);
  if ((sensorL && sensorR) || (!sensorL && sensorR)) {
    *needToChangeDirection = true;
  } else if (!sensorL && !sensorR) {
    *needToChangeDirection = true;
  }
  if (((*pulseTimer + pulseInterval) < millis()) || (*needToChangeDirection)) {
    *pulseTimer = millis();
    *needToChangeDirection = false;
    if (fourSensors) {
      if (sensorFR) {
        hardRight(driveSpeedMulti);
      } else if (sensorFR && sensorR) {
        gradualRight(driveSpeedMulti);
      } else if (sensorFL) {
        hardLeft(driveSpeedMulti);
      } else if (sensorFL && sensorL) {
        gradualLeft(driveSpeedMulti);
      } else if (!sensorFL && !sensorL && !sensorFR && !sensorR) {
        halt(midPWM);
        delay(100);
        tone(buzzerPin, 6000, 10);
      }
      else{
        forward(driveSpeedMulti);
      }
    } else {
      if (!sensorR && sensorL) {
        hardRight(driveSpeedMulti);
      } else if (!sensorL && sensorR) {
        hardLeft(driveSpeedMulti);
      } else if (!sensorL && !sensorR) {
        halt(midPWM);
        delay(100);
        tone(buzzerPin, 6000, 10);
      } else {
        forward(driveSpeedMulti);
      }
    }
  }
}

// Stop
void halt(int midPWM) {
  leftServo.writeMicroseconds(midPWM);
  rightServo.writeMicroseconds(midPWM);
  return;
}
// Forward
void forward(float speedMulti) {
  leftServo.writeMicroseconds(midPWM + (maxSpeed*outSpeed*speedMulti));
  rightServo.writeMicroseconds(midPWM - (maxSpeed*outSpeed*speedMulti));
  return;
}
// Steers for merging
// Left
void steerL(int midPWM, int maxSpeed) {
  rightServo.writeMicroseconds((int)(midPWM + (maxSpeed * backwards)));
  leftServo.writeMicroseconds((int)(midPWM + (maxSpeed * backwards)));
  return;
}
// Right
void steerR(int midPWM, int maxSpeed) {
  rightServo.writeMicroseconds((int)(midPWM - (maxSpeed * backwards)));
  leftServo.writeMicroseconds((int)(midPWM - (maxSpeed * backwards)));
  return;
}

// Steers while on track
// Left
void gradualLeft(float speedMulti) {
  leftServo.writeMicroseconds(midPWM + (int)(maxSpeed*inSpeed*speedMulti));
  rightServo.writeMicroseconds(midPWM - (int)(maxSpeed*outSpeed*speedMulti));
  return;
}
// Right
void gradualRight(float speedMulti) {
  leftServo.writeMicroseconds(midPWM + (maxSpeed*outSpeed*speedMulti));
  rightServo.writeMicroseconds(midPWM - (maxSpeed*inSpeed*speedMulti));
  return;
}
// Harder Left (For soft adjustments)
void hardLeft(float speedMulti) {
  leftServo.writeMicroseconds(midPWM + (maxSpeed*hInSpeed*speedMulti));
  rightServo.writeMicroseconds(midPWM - (maxSpeed*outSpeed*speedMulti));
  return;
}
// Harder Right (For soft adjustments)
void hardRight(float speedMulti) {
  leftServo.writeMicroseconds(midPWM + (maxSpeed*outSpeed*speedMulti));
  rightServo.writeMicroseconds(midPWM - (maxSpeed*hInSpeed*speedMulti));
  return;
}
// End Servo Control ---

// Buzzer
void initialBeep(int pin) {
  tone(pin, 500, 500); delay(500);
  tone(pin, 1500, 500); delay(500);
  tone(pin, 1000, 500); delay(500);
  tone(pin, 2000, 500); delay(500);
  return;
}
