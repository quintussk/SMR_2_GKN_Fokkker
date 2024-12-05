#include <PID_v1.h>

#define ENCODER_A 32 
#define ENCODER_B 33 
#define in1 18
#define in2 19
#define pwm1 21

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

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
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
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    myPID.Compute();  
    analogWrite(pwm1, Output); 
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    myPID.Compute();  
    analogWrite(pwm1, Output); 
  }
}
