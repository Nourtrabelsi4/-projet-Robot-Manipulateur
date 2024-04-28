/* Include the HCPCA9685 library */
#include "HCPCA9685.h"
#include <SoftwareSerial.h>
#include <AccelStepper.h>
#define  I2CAdd 0x40
HCPCA9685 HCPCA9685(I2CAdd);

SoftwareSerial Bluetooth(A8, 38); // Arduino(RX, TX) - HC-05 Bluetooth (TX, RX)

// Define the stepper motors and the pins the will use
AccelStepper LeftBackWheel(1, 2, 3);   // (Type:driver, STEP, DIR) - Stepper1
AccelStepper LeftFrontWheel(1, 4, 5);  // Stepper2
AccelStepper RightBackWheel(1, 6, 7);  // Stepper3
AccelStepper Righ=tFrontWheel(1, 8, 9); // Stepper4

#define led 14

int wheelSpeed = 1500;

int lbw[50], lfw[50], rbw[50], rfw[50]; // arrays for storing positions/steps

int servo1Pos, servo2Pos, servo3Pos, servo4Pos, servo5Pos, servo6Pos; // current position
int servo1PPos, servo2PPos, servo3PPos, servo4PPos, servo5PPos, servo6PPos; // previous position
int servo01SP[50], servo02SP[50], servo03SP[50], servo04SP[50], servo05SP[50], servo06SP[50]; // for storing positions/steps
int speedDelay = 20;
int index = 0;
int dataIn;
int m = 0;

void setup() {
HCPCA9685.Init(SERVO_MODE);
HCPCA9685.Sleep(false);

  // Set initial seed values for the steppers
  LeftFrontWheel.setMaxSpeed(400);
  LeftBackWheel.setMaxSpeed(400);
  RightFrontWheel.setMaxSpeed(400);
  RightBackWheel.setMaxSpeed(400);
  pinMode(led, OUTPUT);
 
  Bluetooth.begin(9600); // Default baud rate of the Bluetooth module 
    Serial.begin(9600);
    
  // Move robot arm to initial position
    servo1PPos = 180;
    Serial.print( servo1PPos);
    HCPCA9685.Servo(0,   servo1PPos);  
    servo2PPos = 340;
    HCPCA9685.Servo(1,   servo2PPos);  
    servo3PPos = 0;
    HCPCA9685.Servo(2,   servo3PPos);  
    servo4PPos = 420;
    HCPCA9685.Servo(3,   servo4PPos);  
    servo5PPos = 220;
    HCPCA9685.Servo(4,   servo5PPos);  
    servo6PPos = 250;
    HCPCA9685.Servo(5,   servo6PPos);  
    }

void loop() {
  // Check for incoming data
  if (Bluetooth.available() > 0) {
    dataIn = Bluetooth.read();  // Read the data
      Serial.print( dataIn);
    if (dataIn == 0) {
      m = 0;
    }
    if (dataIn == 1) {
      m = 1;
    }
    if (dataIn == 2) {
      m = 2;
    }
    if (dataIn == 3) {
      m = 3;
    }
    if (dataIn == 4) {
      m = 4;
    }
    if (dataIn == 5) {
      m = 5;
    }
    if (dataIn == 6) {
      m = 6;
    }
    if (dataIn == 7) {
      m = 7;
    }
    if (dataIn == 8) {
      m = 8;
    }
    if (dataIn == 9) {
      m = 9;
    }
    if (dataIn == 10) {
      m = 10;
    }
    if (dataIn == 11) {
      m = 11;
    }
    if (dataIn == 12) {
      m = 12;
    }
    if (dataIn == 14) {
      m = 14;
    }
    if (dataIn == 16) {
      m = 16;
    }
    if (dataIn == 17) {
      m = 17;
    }
    if (dataIn == 18) {
      m = 18;
    }
    if (dataIn == 19) {
      m = 19;
    }
    if (dataIn == 20) {
      m = 20;
    }
    if (dataIn == 21) {
      m = 21;
    }
    if (dataIn == 22) {
      m = 22;
    }
    if (dataIn == 23) {
      m = 23;
    }
    if (dataIn == 24) {
      m = 24;
    }
    if (dataIn == 25) {
      m = 25;
    }
    if (dataIn == 26) {
      m = 26;
    }
    if (dataIn == 27) {
      m = 27;
    }

    // Move the Mecanum wheels platform
    if (m == 4) {
      moveSidewaysLeft();
    }
    if (m == 5) {
      moveSidewaysRight();
    }
    if (m == 2) {
      moveForward();
    }
    if (m == 7) {
      moveBackward();
    }
    if (m == 3) {
      moveRightForward();
    }
    if (m == 1) {
      moveLeftForward();
    }
    if (m == 8) {
      moveRightBackward();
    }
    if (m == 6) {
      moveLeftBackward();
    }
    if (m == 9) {
      rotateLeft();
    }
    if (m == 10) {
      rotateRight();
    }

    if (m == 0) {
      stopMoving();
    }

    // Mecanum wheels speed
    if (dataIn > 30 & dataIn < 100) {
      wheelSpeed = dataIn * 20;
    }

    // Move robot arm
    // Move servo 1 in positive direction
    while (m == 16) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
     servo1PPos--;
    HCPCA9685.Servo(0,   servo1PPos);
      delay(speedDelay);
    }
    // Move servo 1 in negative direction
    while (m == 17) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
     servo1PPos++;
    HCPCA9685.Servo(0,   servo1PPos);
      delay(speedDelay);
    }
    // Move servo 2
    while (m == 19) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo2PPos++;
    HCPCA9685.Servo(1,   servo2PPos);
      delay(speedDelay);
    }
    while (m == 18) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo1PPos--;
    HCPCA9685.Servo(1,   servo1PPos);
      delay(speedDelay);
    }
    // Move servo 3
    while (m == 21) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
    servo3PPos++;
    HCPCA9685.Servo(2,   servo3PPos);
      delay(speedDelay);
    }
    while (m == 20) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
   servo3PPos--;
    HCPCA9685.Servo(2,   servo3PPos);
      delay(speedDelay);
    }
    // Move servo 4
    while (m == 23) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo4PPos++;
    HCPCA9685.Servo(3,   servo4PPos);
      delay(speedDelay);
    }
    while (m == 22) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo4PPos--;
    HCPCA9685.Servo(3,   servo4PPos);
      delay(speedDelay);
    }
    // Move servo 5
    while (m == 25) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo5PPos++;
    HCPCA9685.Servo(4,   servo5PPos);
      delay(speedDelay);
    }
    while (m == 24) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo5PPos--;
    HCPCA9685.Servo(4,   servo5PPos);
      delay(speedDelay);
    }
    // Move servo 6
    while (m == 27) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo6PPos++;
    HCPCA9685.Servo(5,   servo6PPos);
      delay(speedDelay);
    }
    while (m == 26) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo6PPos--;
    HCPCA9685.Servo(5,   servo6PPos);
      delay(speedDelay);
    }

    // If arm speed slider is changed
    if (dataIn > 101 & dataIn < 250) {
      speedDelay = dataIn / 10; // Change servo speed (delay time)
    }

    // If button "SAVE" is pressed
    if (m == 12) {
      //if it's initial save, set the steppers position to 0
      if (index == 0) {
        LeftBackWheel.setCurrentPosition(0);
        LeftFrontWheel.setCurrentPosition(0);
        RightBackWheel.setCurrentPosition(0);
        RightFrontWheel.setCurrentPosition(0);
      }
      lbw[index] = LeftBackWheel.currentPosition();  // save position into the array
      lfw[index] = LeftFrontWheel.currentPosition();
      rbw[index] = RightBackWheel.currentPosition();
      rfw[index] = RightFrontWheel.currentPosition();

      servo01SP[index] = servo1PPos;  // save position into the array
      servo02SP[index] = servo2PPos;
      servo03SP[index] = servo3PPos;
      servo04SP[index] = servo4PPos;
      servo05SP[index] = servo5PPos;
      servo06SP[index] = servo6PPos;
      index++;                        // Increase the array index
      m = 0;
    }

    // If button "RUN" is pressed
    if (m == 14) {
      runSteps();

      // If button "RESET" is pressed
      if (dataIn != 14) {
        stopMoving();
        memset(lbw, 0, sizeof(lbw)); // Clear the array data to 0
        memset(lfw, 0, sizeof(lfw));
        memset(rbw, 0, sizeof(rbw));
        memset(rfw, 0, sizeof(rfw));
        memset(servo01SP, 0, sizeof(servo01SP)); // Clear the array data to 0
        memset(servo02SP, 0, sizeof(servo02SP));
        memset(servo03SP, 0, sizeof(servo03SP));
        memset(servo04SP, 0, sizeof(servo04SP));
        memset(servo05SP, 0, sizeof(servo05SP));
        memset(servo06SP, 0, sizeof(servo06SP));
        index = 0;  // Index to 0
      }
    }
  }
  LeftFrontWheel.runSpeed();
  LeftBackWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightBackWheel.runSpeed();

  // Monitor the battery voltage
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023.00) * 3; // Convert the reading values from 5v to suitable 12V i
  //Serial.println(voltage);
  // If voltage is below 11V turn on the LED
  if (voltage < 11) {
    digitalWrite(led, HIGH);
  }
  else {
    digitalWrite(led, LOW);
  }
}
void moveForward() {
  LeftFrontWheel.setSpeed(wheelSpeed);
  LeftBackWheel.setSpeed(wheelSpeed);
  RightFrontWheel.setSpeed(wheelSpeed);
  RightBackWheel.setSpeed(wheelSpeed);
}
void moveBackward() {
  LeftFrontWheel.setSpeed(-wheelSpeed);
  LeftBackWheel.setSpeed(-wheelSpeed);
  RightFrontWheel.setSpeed(-wheelSpeed);
  RightBackWheel.setSpeed(-wheelSpeed);
}
void moveSidewaysRight() {
  LeftFrontWheel.setSpeed(wheelSpeed);
  LeftBackWheel.setSpeed(-wheelSpeed);
  RightFrontWheel.setSpeed(-wheelSpeed);
  RightBackWheel.setSpeed(wheelSpeed);
}
void moveSidewaysLeft() {
  LeftFrontWheel.setSpeed(-wheelSpeed);
  LeftBackWheel.setSpeed(wheelSpeed);
  RightFrontWheel.setSpeed(wheelSpeed);
  RightBackWheel.setSpeed(-wheelSpeed);
}
void rotateLeft() {
  LeftFrontWheel.setSpeed(-wheelSpeed);
  LeftBackWheel.setSpeed(-wheelSpeed);
  RightFrontWheel.setSpeed(wheelSpeed);
  RightBackWheel.setSpeed(wheelSpeed);
}
void rotateRight() {
  LeftFrontWheel.setSpeed(wheelSpeed);
  LeftBackWheel.setSpeed(wheelSpeed);
  RightFrontWheel.setSpeed(-wheelSpeed);
  RightBackWheel.setSpeed(-wheelSpeed);
}
void moveRightForward() {
  LeftFrontWheel.setSpeed(wheelSpeed);
  LeftBackWheel.setSpeed(0);
  RightFrontWheel.setSpeed(0);
  RightBackWheel.setSpeed(wheelSpeed);
}
void moveRightBackward() {
  LeftFrontWheel.setSpeed(0);
  LeftBackWheel.setSpeed(-wheelSpeed);
  RightFrontWheel.setSpeed(-wheelSpeed);
  RightBackWheel.setSpeed(0);
}
void moveLeftForward() {
  LeftFrontWheel.setSpeed(0);
  LeftBackWheel.setSpeed(wheelSpeed);
  RightFrontWheel.setSpeed(wheelSpeed);
  RightBackWheel.setSpeed(0);
}
void moveLeftBackward() {
  LeftFrontWheel.setSpeed(-wheelSpeed);
  LeftBackWheel.setSpeed(0);
  RightFrontWheel.setSpeed(0);
  RightBackWheel.setSpeed(-wheelSpeed);
}
void stopMoving() {
  LeftFrontWheel.setSpeed(0);
  LeftBackWheel.setSpeed(0);
  RightFrontWheel.setSpeed(0);
  RightBackWheel.setSpeed(0);
}

// Automatic mode custom function - run the saved steps
void runSteps() {
  while (dataIn != 13) {   // Run the steps over and over again until "RESET" button is pressed
    for (int i = 0; i <= index - 2; i++) {  // Run through all steps(index)
      if (Bluetooth.available() > 0) {      // Check for incomding data
        dataIn = Bluetooth.read();
        if ( dataIn == 15) {           // If button "PAUSE" is pressed
          while (dataIn != 14) {         // Wait until "RUN" is pressed again
            if (Bluetooth.available() > 0) {
              dataIn = Bluetooth.read();
              if ( dataIn == 13) {
                break;
              }
            }
          }
        }
        // If speed slider is changed
        if (dataIn > 100 & dataIn < 150) {
          speedDelay = dataIn / 10; // Change servo speed (delay time)
        }
        // Mecanum wheels speed
        if (dataIn > 30 & dataIn < 100) {
          wheelSpeed = dataIn * 10;
          dataIn = 14;
        }
      }
      LeftFrontWheel.moveTo(lfw[i]);
      LeftFrontWheel.setSpeed(wheelSpeed);
      LeftBackWheel.moveTo(lbw[i]);
      LeftBackWheel.setSpeed(wheelSpeed);
      RightFrontWheel.moveTo(rfw[i]);
      RightFrontWheel.setSpeed(wheelSpeed);
      RightBackWheel.moveTo(rbw[i]);
      RightBackWheel.setSpeed(wheelSpeed);

      while (LeftBackWheel.currentPosition() != lbw[i] & LeftFrontWheel.currentPosition() != lfw[i] & RightFrontWheel.currentPosition() != rfw[i] & RightBackWheel.currentPosition() != rbw[i]) {
        LeftFrontWheel.runSpeedToPosition();
        LeftBackWheel.runSpeedToPosition();
        RightFrontWheel.runSpeedToPosition();
        RightBackWheel.runSpeedToPosition();
      }
      // Servo 1
      if (servo01SP[i] == servo01SP[i + 1]) {
      }
      if (servo01SP[i] > servo01SP[i + 1]) {
        for ( int j = servo01SP[i]; j >= servo01SP[i + 1]; j--) {
            HCPCA9685.Servo(0,   j);
          delay(speedDelay);
        }
      }
      if (servo01SP[i] < servo01SP[i + 1]) {
        for ( int j = servo01SP[i]; j <= servo01SP[i + 1]; j++) {
    HCPCA9685.Servo(0,   j);
          delay(speedDelay);
        }
      }

      // Servo 2
      if (servo02SP[i] == servo02SP[i + 1]) {
      }
      if (servo02SP[i] > servo02SP[i + 1]) {
        for ( int j = servo02SP[i]; j >= servo02SP[i + 1]; j--) {
    HCPCA9685.Servo(1,   j);
          delay(speedDelay);
        }
      }
      if (servo02SP[i] < servo02SP[i + 1]) {
        for ( int j = servo02SP[i]; j <= servo02SP[i + 1]; j++) {
    HCPCA9685.Servo(1,   j);
          delay(speedDelay);
        }
      }

      // Servo 3
      if (servo03SP[i] == servo03SP[i + 1]) {
      }
      if (servo03SP[i] > servo03SP[i + 1]) {
        for ( int j = servo03SP[i]; j >= servo03SP[i + 1]; j--) {
    HCPCA9685.Servo(2,   j);
          delay(speedDelay);
        }
      }
      if (servo03SP[i] < servo03SP[i + 1]) {
        for ( int j = servo03SP[i]; j <= servo03SP[i + 1]; j++) {
    HCPCA9685.Servo(2,   j);
          delay(speedDelay);
        }
      }

      // Servo 4
      if (servo04SP[i] == servo04SP[i + 1]) {
      }
      if (servo04SP[i] > servo04SP[i + 1]) {
        for ( int j = servo04SP[i]; j >= servo04SP[i + 1]; j--) {
    HCPCA9685.Servo(3,   j);
          delay(speedDelay);
        }
      }
      if (servo04SP[i] < servo04SP[i + 1]) {
        for ( int j = servo04SP[i]; j <= servo04SP[i + 1]; j++) {
    HCPCA9685.Servo(3,   j);
          delay(speedDelay);
        }
      }

      // Servo 5
      if (servo05SP[i] == servo05SP[i + 1]) {
      }
      if (servo05SP[i] > servo05SP[i + 1]) {
        for ( int j = servo05SP[i]; j >= servo05SP[i + 1]; j--) {
    HCPCA9685.Servo(4,   j);
          delay(speedDelay);
        }
      }
      if (servo05SP[i] < servo05SP[i + 1]) {
        for ( int j = servo05SP[i]; j <= servo05SP[i + 1]; j++) {
    HCPCA9685.Servo(4,   j);
          delay(speedDelay);
        }
      }

      // Servo .6
      if (servo06SP[i] == servo06SP[i + 1]) {
      }
      if (servo06SP[i] > servo06SP[i + 1]) {
        for ( int j = servo06SP[i]; j >= servo06SP[i + 1]; j--) {
    HCPCA9685.Servo(5,   j);
          delay(speedDelay);
        }
      }
      if (servo06SP[i] < servo06SP[i + 1]) {
        for ( int j = servo06SP[i]; j <= servo06SP[i + 1]; j++) {
    HCPCA9685.Servo(5,   j);
          delay(speedDelay);
        }
      }
    }
  }
  }
