#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PID_v1.h>
#include <SPI.h>
#include <Wire.h>

// Define OLED display dimensions and I2C address
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C

// Define motor control pins
#define MOTOR_DIR1 18
#define MOTOR_DIR2 19
#define MOTOR_SPEED 5

// Create SSD1306 display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// PID control variables
double setpoint = 320;  // Middle of the frame (desired position)
double input = 320;       // Average X from Python script (received from serial)
double output = 0;      // PID output for motor speed

// PID gains
double Kp = 2.0, Ki = 0, Kd = 0.1;
PID motorPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Function to update OLED display
void updateDisplay(float avgX) {
  // Clear the display buffer
  display.clearDisplay();

  // Display the received average X-coordinate
  display.setTextSize(1); // Small text for better fit
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0); // Start at the top-left corner
  display.println(F("Average X Value:"));
  display.setTextSize(2); // Larger text for emphasis
  display.setCursor(0, 20); // Centered vertically
  display.print(avgX, 2); // Print with 2 decimal places

  // Display the PID output
  display.setTextSize(1);
  display.setCursor(0, 50);
  display.print(F("Motor Output: "));
  display.println(output, 1);

  // Render the buffer to the display
  display.display();
}

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);

  // Initialize the OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  // Display a welcome message
  display.clearDisplay();
  display.setTextSize(1); // Small text size
  display.setTextColor(SSD1306_WHITE); // White text color
  display.setCursor(0, 0); // Top-left corner
  display.println(F("Initializing..."));
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the display for real-time updates
  display.clearDisplay();

  // Motor control pin setup
  pinMode(MOTOR_DIR1, OUTPUT);
  pinMode(MOTOR_DIR2, OUTPUT);
  pinMode(MOTOR_SPEED, OUTPUT);

  // Initialize PID
  motorPID.SetMode(AUTOMATIC);
  motorPID.SetOutputLimits(0, 255);  // PWM range for motor speed
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    input = Serial.parseFloat(); // Parse the incoming float value
  }

  // Debugging output to Serial Monitor

  // Compute PID output
  motorPID.Compute();
  int revOutput = 255 - output;

  // Determine motor direction and set speed
  if (input > setpoint) {
    digitalWrite(MOTOR_DIR1, HIGH);
    digitalWrite(MOTOR_DIR2, LOW);
    analogWrite(MOTOR_SPEED, (int)revOutput);  // Set speed based on PID output
  } else {
    digitalWrite(MOTOR_DIR1, LOW);
    digitalWrite(MOTOR_DIR2, HIGH);
    analogWrite(MOTOR_SPEED, (int)(output));  // Set speed based on inverted PID output
  }

  // Update the OLED display with the received value and PID output
  updateDisplay(input);

  delay(10); // Add delay for stability
}
