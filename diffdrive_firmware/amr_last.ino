/* ================================================
   ESP32 Micro-ROS Differential Drive Robot
   Final Version - All Issues Fixed
   
   Fix history:
   v1 - Base code
   v2 - Added joint states publisher
   v3 - Fixed stop() PWM channel bug
        Fixed IRAM_ATTR on ISRs
        Fixed getRpm() double-call
        Fixed joint name allocation
        Fixed frame IDs
        Replaced magic numbers with const
   v4 - Fixed RPM deadband to prevent
        position drift when robot is still
        (noise from low-pass filter was
        accumulating into wheel angles)
================================================ */

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/int32.h>

#include <rosidl_runtime_c/string_functions.h>

#include <odometry.h>

/* ================================
   PIN CONFIGURATION
================================ */

// LEFT MOTOR
const int8_t L_FORW        = 26;
const int8_t L_BACK        = 27;
const int8_t L_enablePin   = 25;
const int8_t L_encoderPin1 = 18;
const int8_t L_encoderPin2 = 21;

// RIGHT MOTOR
const int8_t R_FORW        = 33;
const int8_t R_BACK        = 32;
const int8_t R_enablePin   = 5;
const int8_t R_encoderPin1 = 23;
const int8_t R_encoderPin2 = 15;

/* ================================
   ROBOT PARAMETERS
================================ */

const float wheels_y_distance_  = 0.4f;   // meters (track width)
//const float wheel_radius        = 0.05f;  // meters
const float wheel_radius = 0.035f;
const float wheel_circumference_ = 2.0f * M_PI * wheel_radius;

// Encoder ticks per wheel revolution
const int tickPerRevolution_LW = 420;
const int tickPerRevolution_RW = 420;

/* ================================
   PID PARAMETERS
================================ */

const float kp_l = 10.0f;
const float ki_l = 0.02f;
const float kd_l = 0.1f;

const float kp_r = 10.0f;
const float ki_r = 0.02f;
const float kd_r = 0.1f;

/* ================================
   PWM CONFIGURATION
================================ */

const int PWM_FREQ       = 30000;
const int pwmChannelL    = 0;
const int pwmChannelR    = 1;
const int PWM_RESOLUTION = 8;
const int threshold      = 0;

/* ================================
   SAMPLING PERIOD
================================ */

const unsigned int SAMPLING_MS = 20;
const float        SAMPLING_S  = SAMPLING_MS / 1000.0f;

/* ================================
   RPM DEADBAND
   RPM values below this are treated
   as zero to prevent noise from
   accumulating into wheel positions
   when the robot is stationary.
   Tune this value if needed.
================================ */

const float RPM_DEADBAND = 0.5f;

/* ================================
   MICRO-ROS OBJECTS
================================ */

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

rcl_publisher_t odom_publisher;
rcl_publisher_t joint_state_publisher;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t  support;
rcl_node_t      node;
rcl_timer_t     ControlTimer;

nav_msgs__msg__Odometry      odom_msg;
sensor_msgs__msg__JointState joint_state_msg;

// Joint state data arrays (2 wheels)
double js_positions[2]  = {0.0, 0.0};
double js_velocities[2] = {0.0, 0.0};
double js_efforts[2]    = {0.0, 0.0};

// Cumulative wheel angles (rad)
float left_wheel_angle  = 0.0f;
float right_wheel_angle = 0.0f;

// Time
unsigned long long time_offset      = 0;
unsigned long      prev_odom_update = 0;

Odometry odometry;

/* ================================
   MOTOR CONTROLLER CLASS
================================ */

class MotorController {

public:

  int8_t Forward;
  int8_t Backward;
  int8_t Enable;
  int8_t EncoderPinA;
  int8_t EncoderPinB;

  volatile long encoderCount = 0;

  long          previousPosition = 0;
  unsigned long previousTime     = 0;
  unsigned long previousPidTime  = 0;

  float rpmFilt = 0.0f;
  float rpmPrev = 0.0f;

  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;

  float previousError = 0.0f;
  float integral      = 0.0f;

  int tick;

  /* ----- Constructor ----- */
  MotorController(
    int8_t ForwardPin,
    int8_t BackwardPin,
    int8_t EnablePin,
    int8_t EncoderA,
    int8_t EncoderB,
    int    tickPerRevolution)
  {
    Forward  = ForwardPin;
    Backward = BackwardPin;
    Enable   = EnablePin;

    EncoderPinA = EncoderA;
    EncoderPinB = EncoderB;

    tick = tickPerRevolution;

    pinMode(Forward,     OUTPUT);
    pinMode(Backward,    OUTPUT);
    pinMode(Enable,      OUTPUT);
    pinMode(EncoderPinA, INPUT_PULLUP);
    pinMode(EncoderPinB, INPUT_PULLUP);
  }

  /* ----- PID Init ----- */
  void initPID(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
  }

  /* ================================
     RPM CALCULATION
     Returns filtered RPM.
     Call once per cycle and reuse
     the result — do not call twice.
  ================================= */

  float getRpm() {

    long          currentPosition = encoderCount;
    unsigned long currentTime     = millis();

    float dt = (currentTime - previousTime) / 1000.0f;

    if (dt <= 0.0f)
      return rpmFilt;

    float velocity = (float)(currentPosition - previousPosition) / dt;
    float rpm      = (velocity / (float)tick) * 60.0f;

    // Second-order low-pass filter
    rpmFilt = 0.854f * rpmFilt + 0.0728f * rpm + 0.0728f * rpmPrev;
    rpmPrev = rpm;

    previousPosition = currentPosition;
    previousTime     = currentTime;

    return rpmFilt;
  }

  /* ================================
     RESET RPM filter and integrators
     Call when robot stops to prevent
     stale state on next move.
  ================================= */

  void resetState() {
    rpmFilt       = 0.0f;
    rpmPrev       = 0.0f;
    integral      = 0.0f;
    previousError = 0.0f;
  }

  /* ================================
     PID CONTROLLER
  ================================= */

  float pid(float setpoint, float feedback) {

    unsigned long currentTime = millis();

    float dt = (currentTime - previousPidTime) / 1000.0f;

    if (dt <= 0.0f)
      return 0.0f;

    float error      = setpoint - feedback;
    integral        += error * dt;
    float derivative = (error - previousError) / dt;

    float output = kp * error + ki * integral + kd * derivative;

    previousError   = error;
    previousPidTime = currentTime;

    return output;
  }

  /* ================================
     MOTOR DRIVE
     pwmChannel passed in — no
     dependency on global channel vars
  ================================= */

  void moveBase(float signal, int thr, int pwmChannel) {

    if (signal > 0.0f) {
      digitalWrite(Forward,  HIGH);
      digitalWrite(Backward, LOW);
    } else {
      digitalWrite(Forward,  LOW);
      digitalWrite(Backward, HIGH);
    }

    int pwm = thr + (int)fabs(signal);
    pwm = constrain(pwm, 0, 255);

    ledcWrite(pwmChannel, pwm);
  }

  /* ================================
     STOP
     Only writes its own PWM channel.
     FIX: original hardcoded both
     global channels incorrectly.
  ================================= */

  void stop(int pwmChannel) {
    digitalWrite(Forward,  LOW);
    digitalWrite(Backward, LOW);
    ledcWrite(pwmChannel, 0);
  }
};

/* ================================
   MOTOR OBJECTS
================================ */

MotorController leftWheel(
  L_FORW, L_BACK, L_enablePin,
  L_encoderPin1, L_encoderPin2,
  tickPerRevolution_LW);

MotorController rightWheel(
  R_FORW, R_BACK, R_enablePin,
  R_encoderPin1, R_encoderPin2,
  tickPerRevolution_RW);

/* ================================
   MACROS
================================ */

#define LED_PIN 2

// Hard error — blink LED forever
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if (temp_rc != RCL_RET_OK) { error_loop(); } \
  }

// Soft error — silently continue
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    (void)temp_rc; \
  }

/* ================================
   ERROR LOOP
================================ */

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

/* ================================
   ENCODER ISRs
   IRAM_ATTR required on ESP32 —
   ensures ISR runs from IRAM not
   flash (flash may be busy during
   WiFi/transport activity)
================================ */

void IRAM_ATTR updateEncoderL() {
  if (digitalRead(leftWheel.EncoderPinB) >
      digitalRead(leftWheel.EncoderPinA))
    leftWheel.encoderCount--;
  else
    leftWheel.encoderCount++;
}

void IRAM_ATTR updateEncoderR() {
  if (digitalRead(rightWheel.EncoderPinA) >
      digitalRead(rightWheel.EncoderPinB))
    rightWheel.encoderCount--;
  else
    rightWheel.encoderCount++;
}

/* ================================
   CMD_VEL CALLBACK
================================ */

void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *twist =
    (const geometry_msgs__msg__Twist *)msgin;
  msg = *twist;
}

/* ================================
   TIME HELPERS
================================ */

void syncTime() {
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  time_offset = ros_time_ms - (unsigned long long)now;
}

struct timespec getTime() {
  struct timespec    tp  = {0};
  unsigned long long now = (unsigned long long)millis() + time_offset;
  tp.tv_sec  = (time_t)(now / 1000ULL);
  tp.tv_nsec = (long)((now % 1000ULL) * 1000000ULL);
  return tp;
}

/* ================================
   ANGLE WRAP HELPER
   Keeps angle within [-pi, pi]
   to prevent float overflow on
   long continuous runs
================================ */

float wrapAngle(float a) {
  while (a >  M_PI) a -= 2.0f * M_PI;
  while (a < -M_PI) a += 2.0f * M_PI;
  return a;
}

/* ================================
   PUBLISH ODOM + JOINT STATES
   Receives RPM values already read
   this cycle — avoids a second
   getRpm() call which would return
   stale/wrong filtered values
================================ */

void publishData(float currentRpmL, float currentRpmR) {

  struct timespec ts = getTime();

  /* ---------- Odometry ---------- */
  odom_msg = odometry.getData();
  odom_msg.header.stamp.sec     = (int32_t)ts.tv_sec;
  odom_msg.header.stamp.nanosec = (uint32_t)ts.tv_nsec;

  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));

  /* ---------- Joint States ---------- */

  // FIX: Apply deadband before integrating.
  // The low-pass RPM filter never reaches exactly 0.0,
  // so sub-threshold noise would slowly accumulate into
  // wheel angle positions even when robot is stationary.
  float rpmL_clean = (fabs(currentRpmL) > RPM_DEADBAND) ? currentRpmL : 0.0f;
  float rpmR_clean = (fabs(currentRpmR) > RPM_DEADBAND) ? currentRpmR : 0.0f;

  // Angular velocity in rad/s
  float omega_L = 2.0f * M_PI * (rpmL_clean / 60.0f);
  float omega_R = 2.0f * M_PI * (rpmR_clean / 60.0f);

  // Integrate wheel angle (rad) — zero when robot is still
  left_wheel_angle  = wrapAngle(left_wheel_angle  + omega_L * SAMPLING_S);
  right_wheel_angle = wrapAngle(right_wheel_angle + omega_R * SAMPLING_S);

  js_positions[0]  = (double)left_wheel_angle;
  js_positions[1]  = (double)right_wheel_angle;
  js_velocities[0] = (double)omega_L;
  js_velocities[1] = (double)omega_R;
  js_efforts[0]    = 0.0;
  js_efforts[1]    = 0.0;

  joint_state_msg.header.stamp.sec     = (int32_t)ts.tv_sec;
  joint_state_msg.header.stamp.nanosec = (uint32_t)ts.tv_nsec;

  joint_state_msg.position.data  = js_positions;
  joint_state_msg.position.size  = 2;
  joint_state_msg.velocity.data  = js_velocities;
  joint_state_msg.velocity.size  = 2;
  joint_state_msg.effort.data    = js_efforts;
  joint_state_msg.effort.size    = 2;

  RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));
}

/* ================================
   MOTOR CONTROL TIMER CALLBACK
   Fires every SAMPLING_MS (20 ms)
================================ */

void MotorControll_callback(
  rcl_timer_t *timer,
  int64_t      last_call_time)
{
  (void)timer;
  (void)last_call_time;

  float linearVelocity  = msg.linear.x;
  float angularVelocity = msg.angular.z;

  /* ----- Differential Drive Kinematics ----- */
  float vL = linearVelocity - (angularVelocity * wheels_y_distance_ / 2.0f);
  float vR = linearVelocity + (angularVelocity * wheels_y_distance_ / 2.0f);

  /* ----- Convert m/s to RPM ----- */
  float targetRpmL = (vL / wheel_circumference_) * 60.0f;
  float targetRpmR = (vR / wheel_circumference_) * 60.0f;

  /* ----- Read RPM once, reuse result everywhere ----- */
  float currentRpmL = leftWheel.getRpm();
  float currentRpmR = rightWheel.getRpm();

  /* ----- PID ----- */
  float controlL = leftWheel.pid(targetRpmL,  currentRpmL);
  float controlR = rightWheel.pid(targetRpmR, currentRpmR);

  /* ----- Motor Output ----- */
  bool stopped = (fabs(vL) < 0.01f && fabs(vR) < 0.01f);

  if (stopped) {
    // Each motor only writes its own PWM channel
    leftWheel.stop(pwmChannelL);
    rightWheel.stop(pwmChannelR);

    // Reset filter state so there's no stale RPM
    // on the next movement command
    leftWheel.resetState();
    rightWheel.resetState();

  } else {
    leftWheel.moveBase(controlL,  threshold, pwmChannelL);
    rightWheel.moveBase(controlR, threshold, pwmChannelR);
  }

  /* ----- Odometry Update ----- */
  float avg_rps_linear  = ((currentRpmL + currentRpmR) / 2.0f) / 60.0f;
  float linear_x        = avg_rps_linear * wheel_circumference_;

  float avg_rps_angular = ((-currentRpmL + currentRpmR) / 2.0f) / 60.0f;
  float angular_z       = (avg_rps_angular * wheel_circumference_) /
                           wheels_y_distance_;

  unsigned long now = millis();
  float         dt  = (now - prev_odom_update) / 1000.0f;
  prev_odom_update  = now;

  odometry.update(dt, linear_x, 0.0f, angular_z);

  /* ----- Publish ----- */
  publishData(currentRpmL, currentRpmR);
}

/* ================================
   JOINT NAME HELPER
   Properly allocates a
   rosidl_runtime_c__String entry.
   A plain char* cast is NOT valid
   in micro-ROS string sequences.
================================ */

void assignJointName(
  rosidl_runtime_c__String &str,
  const char *name)
{
  size_t len   = strlen(name);
  str.data     = (char *)malloc(len + 1);
  str.size     = len;
  str.capacity = len + 1;
  memcpy(str.data, name, len + 1);
}

/* ================================
   SETUP
================================ */

void setup() {

  pinMode(LED_PIN, OUTPUT);

  /* ----- PID ----- */
  leftWheel.initPID(kp_l, ki_l, kd_l);
  rightWheel.initPID(kp_r, ki_r, kd_r);

  /* ----- Encoder Interrupts ----- */
  attachInterrupt(
    digitalPinToInterrupt(leftWheel.EncoderPinB),
    updateEncoderL, RISING);

  attachInterrupt(
    digitalPinToInterrupt(rightWheel.EncoderPinA),
    updateEncoderR, RISING);

  /* ----- PWM ----- */
  ledcSetup(pwmChannelL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(leftWheel.Enable,  pwmChannelL);

  ledcSetup(pwmChannelR, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(rightWheel.Enable, pwmChannelR);

  /* ----- Micro-ROS Transport ----- */
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(
    &node,
    "micro_ros_esp32_node",
    "",
    &support));

  /* ----- Subscriber: cmd_vel ----- */
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  /* ----- Publisher: odom ----- */
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom/unfiltered"));

  /* ----- Publisher: joint_states ----- */
  RCCHECK(rclc_publisher_init_default(
    &joint_state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_states"));

  /* ----- Joint State Message: name array ----- */
  rosidl_runtime_c__String__Sequence__init(&joint_state_msg.name, 2);
  assignJointName(joint_state_msg.name.data[0], "base_left_wheel_joint");
  assignJointName(joint_state_msg.name.data[1], "base_right_wheel_joint");
  joint_state_msg.name.size = 2;

  /* ----- Frame IDs (set once at startup) ----- */
  static char odom_frame_id[] = "odom";
  static char base_frame_id[] = "base_footprint";
  static char js_frame_id[]   = "";

  odom_msg.header.frame_id.data     = odom_frame_id;
  odom_msg.header.frame_id.size     = strlen(odom_frame_id);
  odom_msg.header.frame_id.capacity = strlen(odom_frame_id) + 1;

  odom_msg.child_frame_id.data     = base_frame_id;
  odom_msg.child_frame_id.size     = strlen(base_frame_id);
  odom_msg.child_frame_id.capacity = strlen(base_frame_id) + 1;

  joint_state_msg.header.frame_id.data     = js_frame_id;
  joint_state_msg.header.frame_id.size     = 0;
  joint_state_msg.header.frame_id.capacity = 1;

  /* ----- Control Timer: 20 ms ----- */
  RCCHECK(rclc_timer_init_default(
    &ControlTimer,
    &support,
    RCL_MS_TO_NS(SAMPLING_MS),
    MotorControll_callback));

  /* ----- Executor: 1 subscriber + 1 timer = 2 handles ----- */
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &msg,
    &subscription_callback,
    ON_NEW_DATA));

  RCCHECK(rclc_executor_add_timer(&executor, &ControlTimer));

  /* ----- Sync ROS Time ----- */
  syncTime();
}

/* ================================
   LOOP
================================ */

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(1);
}
