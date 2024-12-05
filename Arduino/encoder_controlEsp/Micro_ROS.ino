#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <stdio.h>
#include <PID_v1.h>

// Pin definitions
#define LED_PIN 15

#define MOTOR_1_PWM 32
#define MOTOR_1_DIR_FORWARD 27
#define MOTOR_1_DIR_REVERSE 14
#define MOTOR_1_ENCODER_A 23

#define MOTOR_2_PWM 33
#define MOTOR_2_DIR_FORWARD 12
#define MOTOR_2_DIR_REVERSE 13
#define MOTOR_2_ENCODER_A 1

#define MOTOR_3_PWM 25
#define MOTOR_3_DIR_FORWARD 17
#define MOTOR_3_DIR_REVERSE 16
#define MOTOR_3_ENCODER_A 19

#define MOTOR_4_PWM 26
#define MOTOR_4_DIR_FORWARD 4
#define MOTOR_4_DIR_REVERSE 0
#define MOTOR_4_ENCODER_A 5

// Node State
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// ROS Objects
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t cmd_vel_sub;
rcl_publisher_t debug_pub; // Debugging publisher
rcl_timer_t timer;
rclc_executor_t executor;

// Twist message to handle velocity commands
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__String debug_msg;

// Motor control variables
double Setpoint[4], Input[4], Output[4];
PID pidMotor1(&Input[0], &Output[0], &Setpoint[0], 5.0, 4.2, 0.02, DIRECT);
PID pidMotor2(&Input[1], &Output[1], &Setpoint[1], 5.0, 4.2, 0.02, DIRECT);
PID pidMotor3(&Input[2], &Output[2], &Setpoint[2], 5.0, 4.2, 0.02, DIRECT);
PID pidMotor4(&Input[3], &Output[3], &Setpoint[3], 5.0, 4.2, 0.02, DIRECT);

volatile int encoder_value[4] = {0};
volatile int prvEncoder[4] = {0};

// Encoder interrupt handlers
void encoder_isr_motor1() { encoder_value[0]++; }
void encoder_isr_motor2() { encoder_value[1]++; }
void encoder_isr_motor3() { encoder_value[2]++; }
void encoder_isr_motor4() { encoder_value[3]++; }

// Function to compute motor speeds for omnidirectional movement
void compute_motor_speeds(float linear_x, float linear_y, float angular_z) {
    // Adjust for standard mecanum/omni-wheel configuration
    Setpoint[0] = linear_x + linear_y + angular_z;  // Motor 1 (front-left)
    Setpoint[1] = linear_x - linear_y - angular_z;  // Motor 2 (front-right)
    Setpoint[2] = linear_x - linear_y + angular_z;  // Motor 3 (rear-right)
    Setpoint[3] = linear_x + linear_y - angular_z;  // Motor 4 (rear-left)
}

// Function to control a motor
void control_motor(int pwm_pin, int dir_fwd, int dir_rev, double output) {
    if (output > 0) {
        digitalWrite(dir_fwd, HIGH);
        digitalWrite(dir_rev, LOW);
    } else {
        digitalWrite(dir_fwd, LOW);
        digitalWrite(dir_rev, HIGH);
    }
    analogWrite(pwm_pin, abs(output));
}

// Callback function for /cmd_vel topic
void cmd_vel_callback(const void *msg_in) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;

    compute_motor_speeds(msg->linear.x, msg->linear.y, msg->angular.z);

    // Publish debugging information
    char debug_info[128];
    snprintf(debug_info, sizeof(debug_info), "Linear.x: %.2f, Linear.y: %.2f, Angular.z: %.2f",
             msg->linear.x, msg->linear.y, msg->angular.z);
    debug_msg.data.data = debug_info;
    debug_msg.data.size = strlen(debug_info);
    debug_msg.data.capacity = sizeof(debug_info);
    rcl_publish(&debug_pub, &debug_msg, NULL);
}

// Timer callback for periodic tasks
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)timer;
    (void)last_call_time;
    run_motors(); // Run motors in the periodic task
}

// Run motors based on PID output
void run_motors() {
    for (int i = 0; i < 4; i++) {
        //Input[i] = encoder_value[i];
        Input[i] = encoder_value[i] - prvEncoder[i];  
        prvEncoder[i] = encoder_value[i];
        if (i == 0) pidMotor1.Compute();
        if (i == 1) pidMotor2.Compute();
        if (i == 2) pidMotor3.Compute();
        if (i == 3) pidMotor4.Compute();
    }

    control_motor(MOTOR_1_PWM, MOTOR_1_DIR_FORWARD, MOTOR_1_DIR_REVERSE, Output[0]);
    control_motor(MOTOR_2_PWM, MOTOR_2_DIR_FORWARD, MOTOR_2_DIR_REVERSE, Output[1]);
    control_motor(MOTOR_3_PWM, MOTOR_3_DIR_FORWARD, MOTOR_3_DIR_REVERSE, Output[2]);
    control_motor(MOTOR_4_PWM, MOTOR_4_DIR_FORWARD, MOTOR_4_DIR_REVERSE, Output[3]);
}

// Initialize ROS entities
bool create_entities() {
    const char *node_name = "esp32_motor_driver";
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, node_name, "", &support);

    // Initialize subscription to /cmd_vel
    rclc_subscription_init_default(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");

    // Initialize debugging publisher
    rclc_publisher_init_default(&debug_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/debug_info");

    // Initialize timer
    const unsigned int timer_timeout = 50; // 50ms
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback);

    // Initialize executor
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &twist_msg, cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer);

    return true;
}

// Cleanup ROS entities
void destroy_entities() {
    rcl_subscription_fini(&cmd_vel_sub, &node);
    rcl_publisher_fini(&debug_pub, &node);
    rcl_timer_fini(&timer);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// Setup function
void setup() {
    Serial.begin(115200);
    set_microros_transports();

    pinMode(MOTOR_1_DIR_FORWARD, OUTPUT);
    pinMode(MOTOR_1_DIR_REVERSE, OUTPUT);
    pinMode(MOTOR_2_DIR_FORWARD, OUTPUT);
    pinMode(MOTOR_2_DIR_REVERSE, OUTPUT);
    pinMode(MOTOR_3_DIR_FORWARD, OUTPUT);
    pinMode(MOTOR_3_DIR_REVERSE, OUTPUT);
    pinMode(MOTOR_4_DIR_FORWARD, OUTPUT);
    pinMode(MOTOR_4_DIR_REVERSE, OUTPUT);

    pinMode(LED_PIN, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(MOTOR_1_ENCODER_A), encoder_isr_motor1, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_2_ENCODER_A), encoder_isr_motor2, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_3_ENCODER_A), encoder_isr_motor3, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_4_ENCODER_A), encoder_isr_motor4, RISING);

    pidMotor1.SetMode(AUTOMATIC);
    pidMotor2.SetMode(AUTOMATIC);
    pidMotor3.SetMode(AUTOMATIC);
    pidMotor4.SetMode(AUTOMATIC);

    state = WAITING_AGENT;
}

// Main loop function
void loop() {
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
