#include <PID_v1.h>

#define MOTOR_1_ENCODER_A 23
#define MOTOR_1_ENCODER_B 22
#define MOTOR_2_ENCODER_A 19
#define MOTOR_2_ENCODER_B 18
#define MOTOR_3_ENCODER_A 5
#define MOTOR_3_ENCODER_B 17

#define MOTOR_1_DIR_FORWARD 27
#define MOTOR_1_DIR_REVERSE 14
#define MOTOR_2_DIR_FORWARD 12
#define MOTOR_2_DIR_REVERSE 13
#define MOTOR_3_DIR_FORWARD 17
#define MOTOR_3_DIR_REVERSE 16
#define MOTOR_4_DIR_FORWARD 4
#define MOTOR_4_DIR_REVERSE 0

#define MOTOR_1_PWM 32
#define MOTOR_2_PWM 33
#define MOTOR_3_PWM 25
#define MOTOR_4_PWM 26

int prvEncoder[4] = {0, 0, 0, 0};  
int speed[4] = {0, 0, 0, 0};  
float filteredSpeed[4] = {0, 0, 0, 0};  

double Setpoint[4] = {5, 5, 5, 5};  
double Input[4] = {0, 0, 0, 0};  
double Output[4] = {0, 0, 0, 0};  
double Kp = 10, Ki = 5, Kd = 0.05;

PID myPID0(&Input[0], &Output[0], &Setpoint[0], Kp, Ki, Kd, DIRECT);
PID myPID1(&Input[1], &Output[1], &Setpoint[1], Kp, Ki, Kd, DIRECT);
PID myPID2(&Input[2], &Output[2], &Setpoint[2], Kp, Ki, Kd, DIRECT);
PID myPID3(&Input[3], &Output[3], &Setpoint[3], Kp, Ki, Kd, DIRECT);

volatile int encoder_value[4] = {0, 0, 0, 0};

const float alpha = 0.1; 

// Function for handling encoder interrupts
void encoder_isr0() {
  int A = digitalRead(MOTOR_1_ENCODER_A);
  int B = digitalRead(MOTOR_1_ENCODER_B);
  if ((A == HIGH) != (B == LOW)) {
    encoder_value[0]--;
  } else {
    encoder_value[0]++;
  }
}

void encoder_isr1() {
  int A = digitalRead(MOTOR_2_ENCODER_A);
  int B = digitalRead(MOTOR_2_ENCODER_B);
  if ((A == HIGH) != (B == LOW)) {
    encoder_value[1]--;
  } else {
    encoder_value[1]++;
  }
}

void encoder_isr2() {
  int A = digitalRead(MOTOR_3_ENCODER_A);
  int B = digitalRead(MOTOR_3_ENCODER_B);
  if ((A == HIGH) != (B == LOW)) {
    encoder_value[2]--;
  } else {
    encoder_value[2]++;
  }
}

void setup() {
  Serial.begin(115200); 
  Serial.setTimeout(1);

  Serial.println("Enter PID values in the format: P I D");

  // Initialize pins for motors and encoders
  pinMode(MOTOR_1_DIR_FORWARD, OUTPUT);
  pinMode(MOTOR_1_DIR_REVERSE, OUTPUT);
  pinMode(MOTOR_1_PWM, OUTPUT);

  pinMode(MOTOR_2_DIR_FORWARD, OUTPUT);
  pinMode(MOTOR_2_DIR_REVERSE, OUTPUT);
  pinMode(MOTOR_2_PWM, OUTPUT);

  pinMode(MOTOR_3_DIR_FORWARD, OUTPUT);
  pinMode(MOTOR_3_DIR_REVERSE, OUTPUT);
  pinMode(MOTOR_3_PWM, OUTPUT);

  pinMode(MOTOR_4_DIR_FORWARD, OUTPUT);
  pinMode(MOTOR_4_DIR_REVERSE, OUTPUT);
  pinMode(MOTOR_4_PWM, OUTPUT);

  pinMode(MOTOR_1_ENCODER_A, INPUT_PULLUP);
  pinMode(MOTOR_1_ENCODER_B, INPUT_PULLUP);
  pinMode(MOTOR_2_ENCODER_A, INPUT_PULLUP);
  pinMode(MOTOR_2_ENCODER_B, INPUT_PULLUP);
  pinMode(MOTOR_3_ENCODER_A, INPUT_PULLUP);
  pinMode(MOTOR_3_ENCODER_B, INPUT_PULLUP);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(MOTOR_1_ENCODER_A), encoder_isr0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_2_ENCODER_A), encoder_isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_3_ENCODER_A), encoder_isr2, CHANGE);

  // Initialize PID controllers
  myPID0.SetMode(AUTOMATIC);
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  myPID3.SetMode(AUTOMATIC);
}

void loop() {
  unsigned long currentTime = millis();

  for (int i = 0; i < 4; i++) {
    speed[i] = encoder_value[i] - prvEncoder[i];  
    prvEncoder[i] = encoder_value[i]; 

    // Apply the low-pass filter
    filteredSpeed[i] = alpha * speed[i] + (1 - alpha) * filteredSpeed[i];

    // Update PID input
    Input[i] = filteredSpeed[i];

    // Compute PID output
    switch (i) {
      case 0:
        myPID0.Compute();
        run(0, Output[0]);
        break;
      case 1:
        myPID1.Compute();
        run(1, Output[1]);
        break;
      case 2:
        myPID2.Compute();
        run(2, Output[2]);
        break;
      case 3:
        myPID3.Compute();
        run(3, Output[3]);
        break;
    }

    // Print debugging info
    Serial.print((Setpoint[i]));
    Serial.print(" ");
    Serial.print((Output[i]));
    Serial.print(" ");
    Serial.println((filteredSpeed[i]));
  }

  handleSerialInput(); // Check for PID tuning input
}

//receive pyserial data
void receive() {
  // Check if data is available to read from Python
  if (Serial.available() > 0) {
    char receivedChar = Serial.read();  // Read the received data
  }
}

// Run motor based on direction and output
void run(int motor, double output) {
  int directionPinA, directionPinB, pwmPin;
  
  switch (motor) {
    case 0:
      directionPinA = MOTOR_1_DIR_FORWARD; directionPinB = MOTOR_1_DIR_REVERSE; pwmPin = MOTOR_1_PWM;
      break;
    case 1:
      directionPinA = MOTOR_2_DIR_FORWARD; directionPinB = MOTOR_2_DIR_REVERSE; pwmPin = MOTOR_2_PWM;
      break;
    case 2:
      directionPinA = MOTOR_3_DIR_FORWARD; directionPinB = MOTOR_3_DIR_REVERSE; pwmPin = MOTOR_3_PWM;
      break;
    case 3:
      directionPinA = MOTOR_4_DIR_FORWARD; directionPinB = MOTOR_4_DIR_REVERSE; pwmPin = MOTOR_4_PWM;
      break;
  }

  if (output > 0) {
    digitalWrite(directionPinA, HIGH);
    digitalWrite(directionPinB, LOW);
  } else {
    digitalWrite(directionPinA, LOW);
    digitalWrite(directionPinB, HIGH);
  }

  analogWrite(pwmPin, abs(output)); 
}

// Function to handle Serial input for PID tuning
void handleSerialInput() {
  if (Serial.available()) {
    String inputString = Serial.readStringUntil('\n'); // Read a line of input
    inputString.trim(); // Remove leading/trailing whitespace

    int firstSpace = inputString.indexOf(' ');
    int secondSpace = inputString.indexOf(' ', firstSpace + 1);

    if (firstSpace > 0 && secondSpace > firstSpace) {
      String kpStr = inputString.substring(0, firstSpace);
      String kiStr = inputString.substring(firstSpace + 1, secondSpace);
      String kdStr = inputString.substring(secondSpace + 1);

      double newKp = kpStr.toDouble();
      double newKi = kiStr.toDouble();
      double newKd = kdStr.toDouble();

      if (newKp >= 0 && newKi >= 0 && newKd >= 0) {
        Kp = newKp;
        Ki = newKi;
        Kd = newKd;

        myPID0.SetTunings(Kp, Ki, Kd);
        myPID1.SetTunings(Kp, Ki, Kd);
        myPID2.SetTunings(Kp, Ki, Kd);
        myPID3.SetTunings(Kp, Ki, Kd);

      } else {
        Serial.println("Invalid PID values. Please enter positive numbers.");
      }
    } else {
      Serial.println("Invalid format. Use: P I D");
    }
  }
}
