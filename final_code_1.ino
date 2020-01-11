// Defining the required libraries.
#include "ESP8266WiFi.h"

// Defining the required variables for connecting to the beacon
const char* ssid = "Infidel attachments";
const char* password = "seekwhatyoucrave";
volatile int rssiArr[500];

// Defining the pins for ultrasonic sensors
const int trigPin = 13;
const int echoPinCenter = 15;
const int echoPinLeft = 12;
const int echoPinRight = 14;
int frontDistance = 0;
int leftDistance = 0;
int rightDistance = 0;

// Defining the motor pins and pwm gain.
const int left_positive = 0;
const int left_negetive = 16;
const int right_positive = 2;
const int right_negetive = 5;
const int pwm2 = 4;
int speed2 = 690;
int obstacleCount = 0;
float rssiAverage = 0;
float previousRssi = -100;
int iterationCount = 0;
int maxLengthCount = 0;
// Defining the functions
float rssiCalculate();
bool detectObstacles();

void setup()
{
  Serial.begin(115200);
  //  Defining the pins as inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPinCenter, INPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(echoPinRight, INPUT);
  pinMode(left_positive, OUTPUT);
  pinMode(left_negetive, OUTPUT);
  pinMode(right_positive, OUTPUT);
  pinMode(right_negetive, OUTPUT);
  pinMode(pwm2, OUTPUT);

  //Connecting the robot to the beacon.
  Serial.printf("Connecting to %s ", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    //    Serial.print(".");
  }
  //  for (int i = 0; i < 10; i++) {/
  rssiAverage = rssiCalculate();
  //  }/

  //  Serial.println(" connected");

}


void loop() {
  if (iterationCount == 0) {
    float aveRssi = rssiCalculate();
    Serial.println(aveRssi);
    Serial.println(previousRssi);
    if (aveRssi < -56 and maxLengthCount == 2) {
      moveLeft();
      delay(750);
      maxLengthCount = 0;
    }
    else if (aveRssi < -56) {
      maxLengthCount++;
    }
    else {
      maxLengthCount = 0;
    }
    if (aveRssi > -43) {
      exit(0);
    }
    if (aveRssi > previousRssi) {
      previousRssi = aveRssi;
    }
    else if (aveRssi < previousRssi - 5) {
      moveLeft();
      delay(750);
    }
    else{
      moveForward();
      delay(500);
    }
  }
  else {
    bool obstacle = detectObstacles();
    if (obstacle) {

      if (obstacleCount == 0) {

        moveLeft();
        delay(500);

      }


      else if (obstacleCount == 1) {

        moveRight();
        delay(500);

      }

      else if(obstacleCount >1) {

        moveRight();
        delay(500);

      }
      obstacleCount++;
    }

    else {

      obstacleCount = 0;
      moveForward();
      delay(500);

    }
  }
  stopMove();
  delay(500);
  iterationCount = (iterationCount + 1) % 5;
}

float rssiCalculate() {
  //  Slide the moving average window to the left and then add the current RSSI to the final window.
  float rssiAve = 0;
  for (int j = 0; j < 500; j++) {

    for (int i = 0; i < 500; i++) {
      rssiArr[i] = rssiArr[(i + 1) % 500];
    }
    rssiArr[499] = WiFi.RSSI();


    // Calculate the average RSSI strength

  }
  for (int i = 0; i < 500; i++) {
    rssiAve = rssiAve + rssiArr[i];
    delay(3);
  }
  rssiAve = rssiAve / 500;

  return rssiAve;
}

bool detectObstacles() {
  // Detect the front obstacles.

  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  frontDistance = (pulseIn(echoPinCenter, HIGH) / 2) / 74;
  delay(10);

  // Detect the left obstacles.
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  leftDistance = (pulseIn(echoPinLeft, HIGH) / 2) / 74;
  delay(10);

  // Detect the right obstacles.
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  rightDistance = (pulseIn(echoPinRight, HIGH) / 2) / 74;

  delay(10);

  if ((frontDistance < 5) or (leftDistance < 5) or (rightDistance < 5)) {

    return true;

  }
  return false;
}

void moveForward() {

  digitalWrite(left_positive, HIGH);
  digitalWrite(left_negetive, LOW);
  digitalWrite(right_positive, LOW);
  digitalWrite(right_negetive, HIGH);
  analogWrite(pwm2, speed2);

}

void moveLeft() {

  digitalWrite(left_positive, HIGH);
  digitalWrite(left_negetive, LOW);
  digitalWrite(right_positive, HIGH);
  digitalWrite(right_negetive, LOW);
  analogWrite(pwm2, speed2);

}


void moveRight() {
  digitalWrite(left_positive, LOW);
  digitalWrite(left_negetive, HIGH);
  digitalWrite(right_positive, LOW);
  digitalWrite(right_negetive, HIGH);
  analogWrite(pwm2, speed2);

}

void stopMove() {

  digitalWrite(left_positive, LOW);
  digitalWrite(left_negetive, LOW);
  digitalWrite(right_positive, LOW);
  digitalWrite(right_negetive, LOW);

}
