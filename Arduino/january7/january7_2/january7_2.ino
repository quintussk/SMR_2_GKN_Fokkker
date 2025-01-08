#include <AccelStepper.h>

// Define the pins for the first stepper motor driver
#define STEP_PIN_1 3
#define DIR_PIN_1 4

// Define the pins for the second stepper motor driver
#define STEP_PIN_2 5
#define DIR_PIN_2 6

// Create instances of AccelStepper for both motors
AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);

// Variables to store direction and speed for both motors
int direction1 = 1;   // Default direction for motor 1 (1 = forward, -1 = reverse)
int speed1 = 1000;     // Default speed for motor 1 (steps per second)

int direction2 = 1;   // Default direction for motor 2 (1 = forward, -1 = reverse)
int speed2 = 1000;     // Default speed for motor 2 (steps per second)

void setup() {
  // Start serial communication
  Serial.begin(9600);
  // Set maximum speed and acceleration for both motors
  stepper1.setMaxSpeed(5000);   // Maximum speed (steps per second) for motor 1
  stepper1.setAcceleration(500); // Acceleration (steps per second squared) for motor 1
  
  stepper2.setMaxSpeed(5000);   // Maximum speed (steps per second) for motor 2
  stepper2.setAcceleration(500); // Acceleration (steps per second squared) for motor 2

  Serial.println("Stepper Control Ready!");
}

void loop() {
  // Check if data is available on serial
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    
    // Parse the input for direction and speed for motor 1
    if (input.startsWith("DIR1:")) {
      if (input.equals("DIR1:1")) {
        direction1 = 1;  // Forward direction for motor 1
      } 
      else if (input.equals("DIR1:-1")) {
        direction1 = -1; // Reverse direction for motor 1
      }
    } 
    
    else if (input.startsWith("SPD1:")) {
      speed1 = input.substring(5).toInt(); // Get the speed value for motor 1 (e.g., SPD1:500)
      stepper1.setMaxSpeed(speed1); // Set the new speed for motor 1
    }

    // Parse the input for direction and speed for motor 2
    if (input.startsWith("DIR2:")) {
      if (input.equals("DIR2:1")) {
        direction2 = 1;  // Forward direction for motor 2
      } 
      else if (input.equals("DIR2:-1")) {
        direction2 = -1; // Reverse direction for motor 2
      }
    } 
    
    else if (input.startsWith("SPD2:")) {
      speed2 = input.substring(5).toInt(); // Get the speed value for motor 2 (e.g., SPD2:500)
      stepper2.setMaxSpeed(speed2); // Set the new speed for motor 2
    }

    // Print the current direction and speed for both motors
    Serial.print("Motor 1 - Direction: ");
    Serial.println(direction1 == 1 ? "Forward" : "Reverse");
    Serial.print("Motor 1 - Speed: ");
    Serial.println(speed1);

    Serial.print("Motor 2 - Direction: ");
    Serial.println(direction2 == 1 ? "Forward" : "Reverse");
    Serial.print("Motor 2 - Speed: ");
    Serial.println(speed2);
  }

  // Set the speed for both motors (positive for forward, negative for reverse)
  stepper1.setSpeed(direction1 * speed1);
  stepper2.setSpeed(direction2 * speed2);

  // Move the stepper motors in the desired direction and speed
  stepper1.runSpeed();  // Step the motor 1 to the target position
  stepper2.runSpeed();  // Step the motor 2 to the target position
  
}
