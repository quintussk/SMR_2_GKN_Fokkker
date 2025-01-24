#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>

// Pinconfiguratie
#define STEP_PIN_1 10
#define DIR_PIN_1 11
#define STEP_PIN_2 8
#define DIR_PIN_2 9
#define STEP_PIN_3 12
#define DIR_PIN_3 13
#define RELAY_PIN 30

#define LIMIT_SWITCH_RIGHT 6
#define LIMIT_SWITCH_LEFT 5

#define SERVO_PIN 2

// Stappenmotoren
AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN_3, DIR_PIN_3);

// Servo
Servo myServo;

// Snelheden
int speed1 = 1000; // Default snelheid Mold motor (horizontal)
int speed2 = 1000; // Default snelheid Camera motor (vertical)

void setup() {
  // Seriële communicatie starten
  Serial.begin(9600);
  Serial.println("Arduino is ready");

  // Limietschakelaars configureren
  pinMode(LIMIT_SWITCH_RIGHT, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_LEFT, INPUT_PULLUP);

  // Relais instellen
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

  // Servo initialiseren
  myServo.attach(SERVO_PIN);
  myServo.write(90);  // Startpositie (90 graden)

  // Stappenmotoren configureren
  stepper1.setMaxSpeed(speed1);
  stepper1.setAcceleration(500);

  stepper2.setMaxSpeed(speed1);
  stepper2.setAcceleration(500);

  stepper3.setMaxSpeed(speed2);
  stepper3.setAcceleration(500);
}

// Beweeg stappenmotoren
void moveSteppers(int horizontalSteps, int verticalSteps) {
  // Controleer limietschakelaars
  bool limitRight = digitalRead(LIMIT_SWITCH_RIGHT) == LOW;
  bool limitLeft = digitalRead(LIMIT_SWITCH_LEFT) == LOW;

  // Pas verticale stappen aan op basis van limietschakelaars
  if (verticalSteps > 0 && limitRight) {
    verticalSteps = 0; // Beweging naar rechts stoppen
  } else if (verticalSteps < 0 && limitLeft) {
    verticalSteps = 0; // Beweging naar links stoppen
  }

  // Stel doelen in voor de stappenmotoren
  stepper1.move(-horizontalSteps);
  stepper2.move(horizontalSteps);
  stepper3.move(verticalSteps);

  // Beweeg motoren totdat ze hun doelen bereiken
  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();

    // Controleer limietschakelaars tijdens beweging
    limitRight = digitalRead(LIMIT_SWITCH_RIGHT) == LOW;
    limitLeft = digitalRead(LIMIT_SWITCH_LEFT) == LOW;

    if ((stepper3.distanceToGo() > 0 && limitRight)) {
      stepper3.stop();
      stepper3.setCurrentPosition(0); // Positie resetten
      break; // Stop verticale beweging
    } else {
      stepper3.run();
    }
  }

  // Feedback geven wanneer locatie is bereikt
  Serial.println("Steppers reached location");
}

void loop() {
  // Controleer of er gegevens beschikbaar zijn via de seriële poort
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Verwerk stappenbevel
    if (input.startsWith("H") && input.indexOf("V") != -1) {
      int hIndex = input.indexOf("H");
      int vIndex = input.indexOf("V");
      int horizontalSteps = input.substring(hIndex + 1, vIndex).toInt();
      int verticalSteps = input.substring(vIndex + 1).toInt();

      Serial.println("Moving steppers...");
      moveSteppers(horizontalSteps, verticalSteps);
    }
    // Relais aan/uit
    else if (input.equals("RELAY:ON")) {
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("Relay is ON");
    } else if (input.equals("RELAY:OFF")) {
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("Relay is OFF");
    }
    // Servo-aansturing
    else if (input.startsWith("SERVO:")) {
      int angle = input.substring(6).toInt();
      if (angle >= 0 && angle <= 180) {
        myServo.write(angle);
        Serial.println("Servo moved to " + String(angle) + " degrees");
      } else {
        Serial.println("Invalid servo angle. Please specify an angle between 0 and 180.");
      }
    }
    // Snelheid instellen voor Mold motor (horizontal)
    else if (input.startsWith("SPD1:")) {
      speed1 = input.substring(5).toInt();
      stepper1.setMaxSpeed(speed1);
      stepper2.setMaxSpeed(speed1);
      Serial.println("Mold motor speed set to " + String(speed1));
    }
    // Snelheid instellen voor Camera motor (vertical)
    else if (input.startsWith("SPD2:")) {
      speed2 = input.substring(5).toInt();
      stepper3.setMaxSpeed(speed2);
      Serial.println("Camera motor speed set to " + String(speed2));
    }
    // Limietschakelaarstatus controleren
    else if (input.equals("LIMIT")) {
      Serial.println(String("Right switch: ") + (digitalRead(LIMIT_SWITCH_RIGHT) == LOW ? "Pressed" : "Not pressed"));
      Serial.println(String("Left switch: ") + (digitalRead(LIMIT_SWITCH_LEFT) == LOW ? "Pressed" : "Not pressed"));
    }
    else if (input.equals("CAMERA:POS")){
      if (digitalRead(LIMIT_SWITCH_RIGHT) == LOW){
        Serial.println("Camera on position");
      }
      else{
        Serial.println("Camera not on position");
      }
    }
  }
}