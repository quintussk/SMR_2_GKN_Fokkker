// Define stepper motor pins
#define STEP_PIN_1 3
#define DIR_PIN_1 4

#define STEP_PIN_2 5
#define DIR_PIN_2 6

// Number of steps for each motor
#define STEPS 400

void setup() {
  // Set motor pins as OUTPUT
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);

  // Set the direction of both motors
  digitalWrite(DIR_PIN_1, HIGH);  // Direction for motor 1
  digitalWrite(DIR_PIN_2, LOW);   // Direction for motor 2
}

void loop() {
  // Drive both motors simultaneously for 500 steps
  for (int i = 0; i < STEPS; i++) {
    // Step motor 1
    digitalWrite(STEP_PIN_1, HIGH);  // Generate step pulse
    digitalWrite(STEP_PIN_2, HIGH);  // Generate step pulse
    delayMicroseconds(200);          // Adjust the speed by changing delay
    digitalWrite(STEP_PIN_1, LOW);
    digitalWrite(STEP_PIN_2, LOW);
    delayMicroseconds(200);  // Adjust the speed by changing delay
  }

  // Add a delay before repeating the loop (optional)
  delay(1000);
}
