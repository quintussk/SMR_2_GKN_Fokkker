#include <PID_v1.h>

#define ENCODER_A 23
#define ENCODER_B 22
// #define ENCODER_A1 1
// #define ENCODER_B1 3
#define ENCODER_A2 19
#define ENCODER_B2 18
#define ENCODER_A3 5
#define ENCODER_B3 17

#define inA 27
#define inB 14
#define inA1 12
#define inB1 13
#define inA2 17
#define inB2 16
#define inA3 4
#define inB3 0


#define pwm0 32
#define pwm1 33
#define pwm2 25
#define pwm3 26

int prvEncoder = 0;  
int speed = 0;  
float filteredSpeed = 0;  

double Setpoint, Input, Output;
double Kp = 5, Ki = 4.2, Kd = 0.02;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

volatile int encoder_value = 0;

const float alpha = 0.1; 

// Function for handling encoder interrupts
void encoder_isr() {
  int A = digitalRead(ENCODER_A);
  int B = digitalRead(ENCODER_B);
  if ((A == HIGH) != (B == LOW)) {
    encoder_value--;  // If encoder A leads B, rotate counter-clockwise
  } else {
    encoder_value++;  // If encoder B leads A, rotate clockwise
  }
}

unsigned long lastPrintTime = 0; 

void setup() {
  Input = encoder_value;
  Setpoint = 100;  
  myPID.SetMode(AUTOMATIC);

  pinMode(inA, OUTPUT);
  pinMode(inB, OUTPUT);
  pinMode(pwm1, OUTPUT);

  Serial.begin(115200); 
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_isr, CHANGE);
}

void loop() {

  unsigned long currentTime = millis();
  
  if (currentTime - lastPrintTime >= 100) {
    lastPrintTime = currentTime;

    speed = encoder_value - prvEncoder;  
    prvEncoder = encoder_value; 

    // Apply the low-pass filter
    filteredSpeed = alpha * speed + (1 - alpha) * filteredSpeed;

    // Print raw and filtered speeds for debugging
    Serial.print(Setpoint);
    Serial.print(" ");
    Serial.print(Output);
    Serial.print(" ");
    Serial.println(filteredSpeed);
  }

  run(0);
}

void run(bool direction) {
  Input = filteredSpeed;  
  if (direction == 0){
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    myPID.Compute();  
    analogWrite(pwm0, Output); 
  }
  else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
    myPID.Compute();  
    analogWrite(pwm0, Output); 
  }
}
