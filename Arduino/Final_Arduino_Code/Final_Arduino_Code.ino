#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>

// Define the pins for the stepper motor drivers
#define STEP_PIN_1 10
#define DIR_PIN_1 11
#define STEP_PIN_2 8
#define DIR_PIN_2 9
#define STEP_PIN_3 12
#define DIR_PIN_3 13

#define limitSwitchRightPin 6
#define limitSwitchLeftPin 5

#define SERVO_PIN 4

// Create instances of AccelStepper for the motors
AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN_3, DIR_PIN_3);

// Create an instance of Servo
Servo myServo;

// Variables to store direction and speed for both motors
int direction1 = 0;   // Default direction for motor 1 (1 = forward, -1 = reverse, 0 = stop)
int speed1 = 1000;    // Default speed for motor 1 (steps per second)

int direction2 = 0;   // Default direction for motor 2 (1 = forward, -1 = reverse, 0 = stop)
int speed2 = 1000;    // Default speed for motor 2 (steps per second)

// Variables to track the last sent state
int lastDirection1 = 0;
int lastSpeed1 = 0;
int lastDirection2 = 0;
int lastSpeed2 = 0;

void setup() {
  // Start serial communication
  Serial.begin(9600);

  // Set maximum speed and acceleration for both motors
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);
  
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);

  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(500);

  Serial.println("Ready");

  pinMode(limitSwitchRightPin, INPUT_PULLUP);
  pinMode(limitSwitchLeftPin, INPUT_PULLUP);

  // Attach the servo to the pin
  myServo.attach(SERVO_PIN);

  // Move the servo to 90 degrees
  myServo.write(90);
}


// Function to send feedback to the Python script
void sendFeedback(String message) {
  Serial.println(message);
}

// Function to send all current values
void sendValues() {
  Serial.print("Motor 1 - Direction: ");
  Serial.print(direction1 == 1 ? "Forward" : (direction1 == -1 ? "Reverse" : "Stopped"));
  Serial.print(", Speed: ");
  Serial.println(speed1);
  
  Serial.print("Motor 2 - Direction: ");
  Serial.print(direction2 == 1 ? "Forward" : (direction2 == -1 ? "Reverse" : "Stopped"));
  Serial.print(", Speed: ");
  Serial.println(speed2);
}

// Function to check for state changes and send updates
void checkStateChange() {
  if (lastDirection1 != direction1 || lastSpeed1 != speed1) {
    sendFeedback("Motor 1 state changed: Direction = " + String(direction1) + ", Speed = " + String(speed1));
    lastDirection1 = direction1;
    lastSpeed1 = speed1;
  }

  if (lastDirection2 != direction2 || lastSpeed2 != speed2) {
    sendFeedback("Motor 2 state changed: Direction = " + String(direction2) + ", Speed = " + String(speed2));
    lastDirection2 = direction2;
    lastSpeed2 = speed2;
  }
}

// Function to move steppers according to received steps
void moveSteppers(int horizontal_steps, int vertical_steps) {
  // Check the state of the limit switches
  bool LS_RIGHT = digitalRead(limitSwitchRightPin) == LOW; // LOW if pressed
  bool LS_LEFT = digitalRead(limitSwitchLeftPin) == LOW;  // LOW if pressed
  
  // Adjust vertical_steps based on limit switches
  if (vertical_steps > 0 && LS_RIGHT) {
    vertical_steps = 0; // Stop moving in positive direction if LS_RIGHT is pressed
  } else if (vertical_steps < 0 && LS_LEFT) {
    vertical_steps = 0; // Stop moving in negative direction if LS_LEFT is pressed
  }

  // Move the steppers
  stepper1.move(horizontal_steps * -1);
  stepper2.move(horizontal_steps);
  stepper3.move(vertical_steps);

  // Run the steppers until they reach their target
  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();

    // Check limit switches during motion
    LS_RIGHT = digitalRead(limitSwitchRightPin) == LOW;
    LS_LEFT = digitalRead(limitSwitchLeftPin) == LOW;

    if ((stepper3.distanceToGo() > 0 && LS_RIGHT) || (stepper3.distanceToGo() < 0 && LS_LEFT)) {
      stepper3.stop(); // Stop the motor if the limit switch is triggered
      stepper3.setCurrentPosition(0);
    } else {
      stepper3.run();
    }
  }

  // Send serial command to capture image
  Serial.println("Steppers reached location");
}
void loop() {
  // if (digitalRead(LS_RIGHT) == LOW) {
  //   Serial.println("Right limit switch pressed");
  // }
  // if (digitalRead(LS_LEFT) == LOW) {
  //   Serial.println("Left limit switch pressed");
  // }

  // Check if data is available on serial
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    
    // Parse the input for direction and speed for motor 1
    if (input.startsWith("DIR1:")) {
      if (input.equals("DIR1:1")) {
        direction1 = 1;
        sendFeedback("Motor 1 Direction set to Forward");
      } 
      else if (input.equals("DIR1:-1")) {
        direction1 = -1;
        sendFeedback("Motor 1 Direction set to Reverse");
      }
      else if (input.equals("DIR1:0")) {
        direction1 = 0;  // Stop motor 1
        sendFeedback("Motor 1 Stopped");
      }
    } 
    
    else if (input.startsWith("SPD1:")) {
      speed1 = input.substring(5).toInt();
      stepper1.setMaxSpeed(speed1);
      stepper2.setMaxSpeed(speed1);
      sendFeedback("Motor 1 Speed set to " + String(speed1));
    }

    // Parse the input for direction and speed for motor 2
    if (input.startsWith("DIR2:")) {
      if (input.equals("DIR2:1")) {
        direction2 = 1;
        sendFeedback("Motor 2 Direction set to Forward");
      } 
      else if (input.equals("DIR2:-1")) {
        direction2 = -1;
        sendFeedback("Motor 2 Direction set to Reverse");
      }
      else if (input.equals("DIR2:0")) {
        direction2 = 0;  // Stop motor 2
        sendFeedback("Motor 2 Stopped");
      }
    } 
    
    else if (input.startsWith("SPD2:")) {
      speed2 = input.substring(5).toInt();
      stepper3.setMaxSpeed(speed2);
      sendFeedback("Motor 2 Speed set to " + String(speed2));
    }

    // Handle the "values" command to return current states
    else if (input.equals("values")) {
      sendValues();
    }

    // Handle the steps command
    else if (input.startsWith("H") && input.indexOf("V") != -1) {
      int hIndex = input.indexOf("H");
      int vIndex = input.indexOf("V");
      int horizontal_steps = input.substring(hIndex + 1, vIndex).toInt();
      int vertical_steps = input.substring(vIndex + 1).toInt();
      moveSteppers(horizontal_steps, vertical_steps);
      sendFeedback("Steppers moved: Horizontal = " + String(horizontal_steps) + ", Vertical = " + String(vertical_steps));
    }

    else if (input.equals("CAMERA:POS")){
      if (digitalRead(limitSwitchRightPin) == LOW){
        sendFeedback("Camera on position");
      }
      else{
        sendFeedback("Camera not on position");
      }
    }
  }
  

  // Check if LS_RIGHT or LS_LEFT is triggered
  if (digitalRead(limitSwitchRightPin) == LOW || digitalRead(limitSwitchLeftPin) == LOW) {
    // Stop all steppers
    stepper1.setSpeed(0);
    stepper2.setSpeed(0);
    stepper3.setSpeed(0);
  } else {
    // Set the speed for both motors
    if (direction1 != 0) {
      stepper1.setSpeed(direction1 * speed1);
      stepper2.setSpeed(direction1 * speed1);
    } else {
      stepper1.setSpeed(0);  // Stop motor 1
      stepper2.setSpeed(0);  // Stop motor 1
    }

    if (direction2 != 0) {
      stepper3.setSpeed(direction2 * speed2);
    } else {
      stepper3.setSpeed(0);  // Stop motor 2
    }
  }

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);

  // Check for state changes to provide periodic feedback
  checkStateChange();
}