#include <math.h>
#define noPaverageseInterrupts 1
#define esp32pin A5



bool whiteLine = true;

const int motorAenable = 11;
const int motorAcontrolA = 10;
const int motorAcontrolB = 9;

const int motorBcontrolA = 8;
const int motorBcontrolB = 7;
const int motorBenable = 6;

const int LeftInfraRed = 2;
const int RightInfraRed = 3;


int baseSpeed = 100;
int turnSpeed = 70;

int RightInfraRedValue = 1;
int LeftInfraRedValue = 1;


void motorAForward(int speedA) {
  analogWrite(motorAenable, speedA);
  digitalWrite(motorAcontrolA, LOW);
  digitalWrite(motorAcontrolB, HIGH);
}

void motorBForward(int speedB) {
  analogWrite(motorBenable, speedB);
  digitalWrite(motorBcontrolA, HIGH);
  digitalWrite(motorBcontrolB, LOW);
}

void forward(int carSpeed) {
  motorAForward(carSpeed);
  motorBForward(carSpeed);
}

void motorABackward(int speedA) {
  analogWrite(motorAenable, speedA);
  digitalWrite(motorAcontrolA, HIGH);
  digitalWrite(motorAcontrolB, LOW);
}

void motorBBackward(int speedB) {
  analogWrite(motorBenable, speedB);
  digitalWrite(motorBcontrolA, LOW);
  digitalWrite(motorBcontrolB, HIGH);
}
void backward(int speed) {
  motorABackward(speed);
  motorBBackward(speed);
}



void motorAStop() {
  analogWrite(motorAenable, 0);
}

void motorBStop() {
  analogWrite(motorBenable, 0);
}

void stop() {
  motorAStop();
  motorBStop();
}

void turnLeft45() {
  motorABackward(180);
  motorBForward(180);
  delay(200);  // Adjust the delay to achieve a 45-degree turn
  stop();
  delay(1000);
}

void turnLeft90() {
  motorBForward(180);
  motorABackward(180);
  delay(300);  // Adjust the delay to achieve a 90-degree turn
  stop();
  delay(1000);
}

void turnRight45() {
  motorBBackward(180);
  motorAForward(180);
  delay(200);  // Adjust the delay to achieve a 45-degree turn
  stop();
  delay(1000);
}

void turnRight90() {
  motorAForward(180);
  motorBBackward(180);
  delay(400);  // Adjust the delay to achieve a 90-degree turn
  stop();
  delay(1000);
}

void goStraightLong() {
  forward(baseSpeed);
  delay(2500);  // Adjust the delay to move a short distance
}

void goStraightShort() {
  if (whiteLine) {
    do {
      forward(baseSpeed);
      InfraRedManager();
    } while (RightInfraRedValue == HIGH && LeftInfraRedValue == HIGH);

  } else {
    do {
      forward(baseSpeed);
      InfraRedManager();
    } while (RightInfraRedValue == LOW && LeftInfraRedValue == LOW);
  }
  stop();
  delay(500);
}


void turn180() {
  motorABackward(baseSpeed);
  motorBForward(baseSpeed);
  delay(1000);

  if (whiteLine) {
    do {
      motorABackward(baseSpeed);
      motorBForward(baseSpeed);
      InfraRedManager();

    } while (RightInfraRedValue == HIGH && LeftInfraRedValue == HIGH);

  } else {
    do {
      motorABackward(baseSpeed);
      motorBForward(baseSpeed);
      InfraRedManager();

    } while (RightInfraRedValue == LOW && LeftInfraRedValue == LOW);
  }


  stop();
  delay(10);
}



void InfraRedManager() {
  RightInfraRedValue = digitalRead(RightInfraRed);
  LeftInfraRedValue = digitalRead(LeftInfraRed);
}


void setup() {
  pinMode(motorAcontrolA, OUTPUT);
  pinMode(motorAcontrolB, OUTPUT);
  pinMode(motorBcontrolA, OUTPUT);
  pinMode(motorBcontrolB, OUTPUT);
  pinMode(motorAenable, OUTPUT);
  pinMode(motorBenable, OUTPUT);

  pinMode(LeftInfraRed, INPUT);
  pinMode(RightInfraRed, INPUT);

  analogWrite(motorAenable, 255);  //inital speed of Motors
  analogWrite(motorBenable, 255);

  pinMode(esp32pin, INPUT);  // Enable pull-up resistor

  Serial.begin(9600);
}


void checkDestination() {
  int ESP32PIN = analogRead(esp32pin);  // Read Pin 12
  Serial.print("Pin A5 State: ");
  Serial.println(ESP32PIN);  // Print the state for debugging

  if (ESP32PIN >= 1020) {
    turn180();
    Serial.println("Turning 180");
    delay(10);
  }
}


void loop() {
  InfraRedManager();
  checkDestination();
  if (whiteLine) {
    if (LeftInfraRedValue == HIGH && RightInfraRedValue == HIGH) {
      forward(baseSpeed);                                                 // Black line detected, go straight
    } else if (LeftInfraRedValue == LOW && RightInfraRedValue == HIGH) {  // Left sensor off the line, right sensor on the line, turn left
      motorABackward(150);                                                //reduce speed of left motor
      motorBForward(baseSpeed + turnSpeed);                               //increase speed of right motor
    } else if (LeftInfraRedValue == HIGH && RightInfraRedValue == LOW) {  //// Right sensor off the line, left sensor on the line, turn right
      motorAForward(baseSpeed + turnSpeed);                               //increase speed of left motor
      motorBBackward(150);                                                //reduce speed of right motor
    } else {                                                              //error meaning both sensors are not finding a black line, meaning car is off tracks
      backward(baseSpeed);
    }



    //////For Black Line:



  } else {
    if (LeftInfraRedValue == LOW && RightInfraRedValue == LOW) {
      forward(baseSpeed);                                                 // Black line detected, go straight
    } else if (LeftInfraRedValue == HIGH && RightInfraRedValue == LOW) {  // Left sensor off the line, right sensor on the line, turn left
      motorABackward(150);                                                //reduce speed of left motor
      motorBForward(baseSpeed + turnSpeed);                               //increase speed of right motor
    } else if (LeftInfraRedValue == LOW && RightInfraRedValue == HIGH) {  //// Right sensor off the line, left sensor on the line, turn right
      motorAForward(baseSpeed + turnSpeed);                               //increase speed of left motor
      motorBBackward(150);                                                //reduce speed of right motor
    } else {                                                              //error meaning both sensors are not finding a black line, meaning car is off tracks
      backward(baseSpeed);
    }
  }
}
