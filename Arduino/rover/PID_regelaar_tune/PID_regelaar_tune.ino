#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <stdio.h>
#include <PID_v1.h>

#define PULSES_PER_REVOLUTION 360  // Number of pulses per revolution of the encoder

#define LED_PIN 15
// Pin definitions
#define MOTOR_1_PWM 32
#define MOTOR_1_DIR_FORWARD 27
#define MOTOR_1_DIR_REVERSE 14
#define MOTOR_1_ENCODER_A 23
#define MOTOR_1_ENCODER_B 22

#define MOTOR_2_PWM 33
#define MOTOR_2_DIR_FORWARD 12
#define MOTOR_2_DIR_REVERSE 13
#define MOTOR_2_ENCODER_A 36
#define MOTOR_2_ENCODER_B 39

#define MOTOR_3_PWM 25
#define MOTOR_3_DIR_FORWARD 17
#define MOTOR_3_DIR_REVERSE 16
#define MOTOR_3_ENCODER_A 21
#define MOTOR_3_ENCODER_B 19

#define MOTOR_4_PWM 26
#define MOTOR_4_DIR_FORWARD 4
#define MOTOR_4_DIR_REVERSE 0
#define MOTOR_4_ENCODER_A 18
#define MOTOR_4_ENCODER_B 5

// Node State
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

rclc_support_t support;
rcl_node_t node;
rcl_subscription_t cmd_vel_sub;
rcl_publisher_t encoder_pub;
rcl_publisher_t pwm_pub; // New publisher for PWM values
rcl_timer_t encoder_timer;
rcl_timer_t pwm_timer; // Timer for publishing PWM values
rclc_executor_t executor;

geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__String encoder_msg;
std_msgs__msg__String pwm_msg; // New message for PWM values

double Kp = 1.8, Ki = 0.5, Kd = 0.05;
double kF = 0.7;  // Feed-forward constante (afstemmen op je systeem)

// PID variables
double Setpoint[4] = {0}, Input[4] = {0}, Output[4] = {0};
PID pidMotor1(&Input[0], &Output[0], &Setpoint[0], Kp, Ki, Kd, DIRECT);
PID pidMotor2(&Input[1], &Output[1], &Setpoint[1], Kp, Ki, Kd, DIRECT);
PID pidMotor3(&Input[2], &Output[2], &Setpoint[2], Kp, Ki, Kd, DIRECT);
PID pidMotor4(&Input[3], &Output[3], &Setpoint[3], Kp, Ki, Kd, DIRECT);

// Encoder variables
volatile int encoder_count[4] = {0};
int last_encoder_count[4] = {0};
double motor_speed[4] = {0};  // Speed of each motor in RPM
double filtered_speed[4] = {0};

// Timer variables
unsigned long last_speed_calc_time = 0;
const unsigned long speed_calc_interval = 100;  // Speed calculation interval in milliseconds

// Encoder interrupt handlers
void IRAM_ATTR encoder_motor1() {
    int A = digitalRead(MOTOR_1_ENCODER_A);
    int B = digitalRead(MOTOR_1_ENCODER_B);
    if ((A == HIGH) != (B == HIGH)) {  // Gebruik relatieve fasen
        encoder_count[0]--;  // Draai tegen de klok in
    } else {
        encoder_count[0]++;  // Draai met de klok mee
    }
}

void IRAM_ATTR encoder_motor2() {
    int A = digitalRead(MOTOR_2_ENCODER_A);
    int B = digitalRead(MOTOR_2_ENCODER_B);
    if ((A == HIGH) != (B == HIGH)) {
        encoder_count[1]--;
    } else {
        encoder_count[1]++;
    }
}

void IRAM_ATTR encoder_motor3() {
    int A = digitalRead(MOTOR_3_ENCODER_A);
    int B = digitalRead(MOTOR_3_ENCODER_B);
    if ((A == HIGH) != (B == HIGH)) {
        encoder_count[2]--;
    } else {
        encoder_count[2]++;
    }
}

void IRAM_ATTR encoder_motor4() {
    int A = digitalRead(MOTOR_4_ENCODER_A);
    int B = digitalRead(MOTOR_4_ENCODER_B);
    if ((A == HIGH) != (B == HIGH)) {
        encoder_count[3]--;
    } else {
        encoder_count[3]++;
    }
}

// Function to compute motor speeds based on linear and angular velocities
void compute_motor_speeds(double linear_x, float linear_y, float angular_z) {
    Setpoint[0] = -linear_x + linear_y + angular_z;  // Front-left motor
    Setpoint[1] = linear_x - linear_y - angular_z;   // Front-right motor
    Setpoint[2] = -linear_x - linear_y + angular_z;  // Rear-left motor
    Setpoint[3] = linear_x + linear_y - angular_z;   // Rear-right motor
}

// Calculate the motor speeds in RPM
void calculate_motor_speeds() {
    unsigned long current_time = millis();
    if (current_time - last_speed_calc_time >= speed_calc_interval) {
        double time_diff = (current_time - last_speed_calc_time) / 1000.0;  // Convert ms to seconds
        last_speed_calc_time = current_time;

        for (int i = 0; i < 4; i++) {
            int delta_count = encoder_count[i] - last_encoder_count[i];  // Preserve sign
            last_encoder_count[i] = encoder_count[i];

            // Speed calculation in RPM
            double raw_speed = (delta_count / (double)PULSES_PER_REVOLUTION) * (60.0 / time_diff);

            // Apply low-pass filter
            double alpha = 0.3;  // Adjust alpha based on your system's response
            filtered_speed[i] = alpha * raw_speed + (1 - alpha) * filtered_speed[i];

            // Update motor speed
            motor_speed[i] = filtered_speed[i];
            // motor_speed[i] = raw_speed;
        }
    }
}

void encoder_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)timer;
    (void)last_call_time;

    char encoder_data[128];
    snprintf(encoder_data, sizeof(encoder_data),
             "Motor1: %d, Motor2: %d, Motor3: %d, Motor4: %d",
             encoder_count[0], encoder_count[1], encoder_count[2], encoder_count[3]);

    encoder_msg.data.data = encoder_data;
    encoder_msg.data.size = strlen(encoder_data);
    encoder_msg.data.capacity = sizeof(encoder_data);

    rcl_publish(&encoder_pub, &encoder_msg, NULL);
}

// Publish PWM values periodically
void pwm_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)timer;
    (void)last_call_time;

    char pwm_data[128];
    snprintf(pwm_data, sizeof(pwm_data),
             "PWM1: %.2f, PWM2: %.2f, PWM3: %.2f, PWM4: %.2f",
             Output[0], Output[1], Output[2], Output[3]);

    pwm_msg.data.data = pwm_data;
    pwm_msg.data.size = strlen(pwm_data);
    pwm_msg.data.capacity = sizeof(pwm_data);

    rcl_publish(&pwm_pub, &pwm_msg, NULL);
}

// Callback function for /cmd_vel topic
void cmd_vel_callback(const void *msg_in) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
    compute_motor_speeds(msg->linear.x, msg->linear.y, msg->angular.z);
}

// Update PID and control motors
void update_pid_and_control() {
    calculate_motor_speeds();  // Bereken motor snelheden op basis van encoder data

    for (int i = 0; i < 4; i++) {
        Input[i] = motor_speed[i];  // Gebruik gemeten snelheid als PID-input
    }
    // Bereken de PID-uitvoer
    pidMotor1.Compute();
    pidMotor2.Compute();
    pidMotor3.Compute();
    pidMotor4.Compute();
    for (int i = 0; i < 4; i++) {
        // Voeg feed-forward toe
        double feedForward = kF * Setpoint[i];  // Feed-forward term gebaseerd op Setpoint
        double motorOutput = Output[i] + feedForward;  // Combineer PID en feed-forward
        
        if (i == 0) control_motor(MOTOR_1_PWM, MOTOR_1_DIR_FORWARD, MOTOR_1_DIR_REVERSE, motorOutput,Setpoint[i]);
        if (i == 1) control_motor(MOTOR_2_PWM, MOTOR_2_DIR_FORWARD, MOTOR_2_DIR_REVERSE, motorOutput,Setpoint[i]);
        if (i == 2) control_motor(MOTOR_3_PWM, MOTOR_3_DIR_FORWARD, MOTOR_3_DIR_REVERSE, motorOutput,Setpoint[i]);
        if (i == 3) control_motor(MOTOR_4_PWM, MOTOR_4_DIR_FORWARD, MOTOR_4_DIR_REVERSE, motorOutput,Setpoint[i]);
            
    }

    // Log data naar de Serial Plotter
    for (int i = 0; i < 4; i++) {
        Serial.print(Setpoint[i]); Serial.print(" ");  // Gewenste snelheid
        Serial.print(Output[i]); Serial.print(" ");    // PID-uitvoer
        // Serial.print(feedForward); Serial.print(" ");  // Feed-forward term
        Serial.print(motor_speed[i]); Serial.print(" ");  // Gemeten snelheid
    }
    Serial.println();
}

// Initialize ROS entities
bool create_entities() {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_motor_driver", "", &support);

    rclc_subscription_init_default(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
    rclc_publisher_init_default(&encoder_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/encoder_values");
    rclc_publisher_init_default(&pwm_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/pwm_values");

    rclc_timer_init_default(&encoder_timer, &support, RCL_MS_TO_NS(100), encoder_timer_callback);
    rclc_timer_init_default(&pwm_timer, &support, RCL_MS_TO_NS(100), pwm_timer_callback);

    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &twist_msg, cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &encoder_timer);
    rclc_executor_add_timer(&executor, &pwm_timer);

    return true;
}

void destroy_entities() {
    rcl_subscription_fini(&cmd_vel_sub, &node);
    rcl_publisher_fini(&encoder_pub, &node);
    rcl_publisher_fini(&pwm_pub, &node);
    rcl_timer_fini(&encoder_timer);
    rcl_timer_fini(&pwm_timer);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// Function to handle serial input and update PID values
void handleSerialInput() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n'); // Read input until newline
        input.trim();  // Remove any extra whitespace

        // Check for "kp=", "ki=", "kd=", or "sp="
        if (input.startsWith("kp=")) {
            double newKp = input.substring(3).toFloat();  // Extract the value
            if (newKp > 0) {
                Kp = newKp;
                pidMotor1.SetTunings(Kp, Ki, Kd);
                pidMotor2.SetTunings(Kp, Ki, Kd);
                pidMotor3.SetTunings(Kp, Ki, Kd);
                pidMotor4.SetTunings(Kp, Ki, Kd);
                Serial.print("Updated Kp to: ");
                Serial.println(Kp);
            } else {
                Serial.println("Invalid value for Kp. Must be positive.");
            }
        } else if (input.startsWith("ki=")) {
            double newKi = input.substring(3).toFloat();  // Extract the value
            if (newKi >= 0) {
                Ki = newKi;
                pidMotor1.SetTunings(Kp, Ki, Kd);
                pidMotor2.SetTunings(Kp, Ki, Kd);
                pidMotor3.SetTunings(Kp, Ki, Kd);
                pidMotor4.SetTunings(Kp, Ki, Kd);
                Serial.print("Updated Ki to: ");
                Serial.println(Ki);
            } else {
                Serial.println("Invalid value for Ki. Must be non-negative.");
            }
        } else if (input.startsWith("kd=")) {
            double newKd = input.substring(3).toFloat();  // Extract the value
            if (newKd >= 0) {
                Kd = newKd;
                pidMotor1.SetTunings(Kp, Ki, Kd);
                pidMotor2.SetTunings(Kp, Ki, Kd);
                pidMotor3.SetTunings(Kp, Ki, Kd);
                pidMotor4.SetTunings(Kp, Ki, Kd);
                Serial.print("Updated Kd to: ");
                Serial.println(Kd);
            } else {
                Serial.println("Invalid value for Kd. Must be non-negative.");
            }
          } else if (input.startsWith("kf=")) {
            double newKf = input.substring(3).toFloat();  // Extract the value
            if (newKf >= 0) {
                kF = newKf;
            } else {
                Serial.println("Invalid value for Kd. Must be non-negative.");
            }
        } else if (input.startsWith("sp=")) {
            double newSpeed = input.substring(3).toFloat();  // Extract the value
            compute_motor_speeds(newSpeed, 0, 0);  // Update motor Setpoints
            Serial.print("Updated speed to: ");
            Serial.println(newSpeed);
        } else {
            Serial.println("Invalid command. Use kp=, ki=, kd=, or sp= followed by a value.");
        }
    }
}

// Motor control function
void control_motor(int pwm_pin, int dir_fwd, int dir_rev, double pid_output, double setpoint) {
    pid_output = constrain(pid_output, -255, 255);
    if (setpoint == 0){
      digitalWrite(dir_fwd, LOW);
      digitalWrite(dir_rev, LOW);
      analogWrite(pwm_pin, 0);

    } if (pid_output > 0) {
        digitalWrite(dir_fwd, HIGH);
        digitalWrite(dir_rev, LOW);
        analogWrite(pwm_pin, pid_output);
    } else if (pid_output < 0) {
        digitalWrite(dir_fwd, LOW);
        digitalWrite(dir_rev, HIGH);
        analogWrite(pwm_pin, abs(pid_output));
    } else {
        digitalWrite(dir_fwd, LOW);
        digitalWrite(dir_rev, LOW);
        analogWrite(pwm_pin, 0);
    }
}

// Setup
void setup() {
    Serial.begin(115200);
    set_microros_transports();

    pinMode(LED_PIN, OUTPUT);

    // Initialize motor control pins
    pinMode(MOTOR_1_DIR_FORWARD, OUTPUT);
    pinMode(MOTOR_1_DIR_REVERSE, OUTPUT);
    pinMode(MOTOR_2_DIR_FORWARD, OUTPUT);
    pinMode(MOTOR_2_DIR_REVERSE, OUTPUT);
    pinMode(MOTOR_3_DIR_FORWARD, OUTPUT);
    pinMode(MOTOR_3_DIR_REVERSE, OUTPUT);
    pinMode(MOTOR_4_DIR_FORWARD, OUTPUT);
    pinMode(MOTOR_4_DIR_REVERSE, OUTPUT);

    // Initialize encoder pins
    pinMode(MOTOR_1_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_1_ENCODER_B, INPUT_PULLUP);
    pinMode(MOTOR_2_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_2_ENCODER_B, INPUT_PULLUP);
    pinMode(MOTOR_3_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_3_ENCODER_B, INPUT_PULLUP);
    pinMode(MOTOR_4_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_4_ENCODER_B, INPUT_PULLUP);

    // Attach interrupts for encoders
    attachInterrupt(digitalPinToInterrupt(MOTOR_1_ENCODER_A), encoder_motor1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_2_ENCODER_A), encoder_motor2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_3_ENCODER_A), encoder_motor3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_4_ENCODER_A), encoder_motor4, CHANGE);

    // Initialize PID controllers
    pidMotor1.SetMode(AUTOMATIC);
    pidMotor2.SetMode(AUTOMATIC);
    pidMotor3.SetMode(AUTOMATIC);
    pidMotor4.SetMode(AUTOMATIC);

    pidMotor1.SetOutputLimits(-255, 255);
    pidMotor2.SetOutputLimits(-255, 255);
    pidMotor3.SetOutputLimits(-255, 255);
    pidMotor4.SetOutputLimits(-255, 255);

    // compute_motor_speeds(90, 0, 0);
    state = WAITING_AGENT;
}

// Main loop function
void loop() {

    // Uncomment to tune PID
    // handleSerialInput();
    // update_pid_and_control();
    // delay(10);

    // Uncomment for ROS
    switch (state) {
        case WAITING_AGENT:
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                state = AGENT_AVAILABLE;
            }
            break;
        case AGENT_AVAILABLE:
            if (create_entities()) {
                state = AGENT_CONNECTED;
            }
            break;
        case AGENT_CONNECTED:
            if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                destroy_entities();
                state = AGENT_DISCONNECTED;
            } else {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
                update_pid_and_control();
            }
            break;
        case AGENT_DISCONNECTED:
            destroy_entities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
    digitalWrite(LED_PIN, state == AGENT_CONNECTED ? HIGH : LOW);
}