#include <AccelStepper.h>

// Define the pins for the stepper motor driver
#define STEP_PIN 3
#define DIR_PIN 4

// Create an instance of AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Variables to store direction and speed
int direction = 1;   // Default direction (1 = forward, -1 = reverse)
int speed = 1000;     // Default speed (steps per second)

void setup() {
  // Start serial communication
  Serial.begin(9600);
  // Set maximum speed and acceleration for smooth movement
  stepper.setMaxSpeed(5000);   // Maximum speed (steps per second)
  stepper.setAcceleration(500); // Acceleration (steps per second squared)

  Serial.println("Stepper Control Ready!");
}

void loop() {
  // Check if data is available on serial
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    
    // Parse the input for direction and speed
    if (input.startsWith("DIR:")) {
      if (input.equals("DIR:1")) {
        direction = 1;  // Forward direction
        // No need to change stepper pins; just set the direction variable
      } 
      else if (input.equals("DIR:-1")) {
        direction = -1; // Reverse direction
        // Again, no need to change stepper pins; just set the direction variable
      }
    } 
    
    else if (input.startsWith("SPD:")) {
      speed = input.substring(4).toInt(); // Get the speed value from input (e.g., SPD:500)
      stepper.setMaxSpeed(speed); // Set the new speed
    }
    
    // Print the current direction and speed
    Serial.print("Direction: ");
    Serial.println(direction == 1 ? "Forward" : "Reverse");
    Serial.print("Speed: ");
    Serial.println(speed);
  }

  // Set the speed (positive for forward, negative for reverse)
  stepper.setSpeed(direction * speed);
  
  // Move the stepper motor in the desired direction and speed
  stepper.runSpeed();  // Step the motor to the target position
}
