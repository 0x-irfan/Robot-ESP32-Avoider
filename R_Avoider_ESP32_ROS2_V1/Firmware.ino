#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#if !defined(ESP32)
#error "Sketch ini hanya untuk ESP32"
#endif

const char*    WIFI_SSID  = "ESP32_avoider";
const char*    WIFI_PASS  = "12345678";
const char*    AGENT_IP   = "10.12.196.137";
const uint16_t AGENT_PORT = 8888;

#define SERVO_PIN 13
#define TRIG_PIN  33
#define ECHO_PIN  34
#define PWMA 18
#define AIN1 23
#define AIN2 19
#define PWMB 14
#define BIN1 26
#define BIN2 25
#define STBY 27
const bool INVERT_LEFT  = false;
const bool INVERT_RIGHT = true;
#define CH_A 4
#define CH_B 5

const unsigned long CMD_TIMEOUT_MS    = 500;   // failsafe: tanpa perintah > ini -> motor berhenti
const unsigned long SERVO_SETTLE_MS   = 300;   // waktu tunggu servo sampai dianggap diam
const unsigned long SENSOR_PERIOD_MS  = 50;    // baca sonar + publish telemetry (20 Hz)
const unsigned long INFO_PERIOD_MS    = 1000;
const unsigned long PING_PERIOD_MS    = 500;

Servo servo;
int   servoAngle   = 90;
unsigned long servoMoveMs = 0;
float distanceCm   = -1;
float motorL = 0, motorR = 0;
unsigned long lastCmdMs = 0;

#define TEL_LEN 11

rcl_publisher_t    tel_pub, info_pub;
rcl_subscription_t cmd_sub, servo_sub;
std_msgs__msg__Float32MultiArray tel_msg;
std_msgs__msg__String            info_msg;
geometry_msgs__msg__Twist        cmd_msg;
std_msgs__msg__Int32             servo_msg;
rclc_executor_t    executor;
rclc_support_t     support;
rcl_allocator_t    allocator;
rcl_node_t         node;

float telData[TEL_LEN];
char  infoBuf[220];
float rttMs = 0;
bool  failsafe = true;

enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState agentState = WAITING_AGENT;
unsigned long tAgent = 0, tPing = 0, tSensor = 0, tInfo = 0;

void pwmSetup() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PWMA, 1000, 8);
  ledcAttach(PWMB, 1000, 8);
#else
  ledcSetup(CH_A, 1000, 8);
  ledcAttachPin(PWMA, CH_A);
  ledcSetup(CH_B, 1000, 8);
  ledcAttachPin(PWMB, CH_B);
#endif
}

void pwmWrite(int pin, int ch, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}

void setMotor(int in1, int in2, int pwmPin, int ch, float v, bool invert) {
  if (invert) v = -v;
  int duty = (int)(fabsf(v) * 255.0f + 0.5f);
  duty = constrain(duty, 0, 255);
  if (duty == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  } else if (v > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  pwmWrite(pwmPin, ch, duty);
}

void stopMotors() {
  motorL = motorR = 0;
  setMotor(AIN1, AIN2, PWMA, CH_A, 0, INVERT_LEFT);
  setMotor(BIN1, BIN2, PWMB, CH_B, 0, INVERT_RIGHT);
}

void applyDrive(float lin, float ang) {
  motorL = constrain(lin - ang, -1.0f, 1.0f);
  motorR = constrain(lin + ang, -1.0f, 1.0f);
  setMotor(AIN1, AIN2, PWMA, CH_A, motorL, INVERT_LEFT);
  setMotor(BIN1, BIN2, PWMB, CH_B, motorR, INVERT_RIGHT);
}

float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 25000UL);
  if (dur == 0) return -1;
  return dur * 0.0343f / 2.0f;
}

void cmdCallback(const void* msgin) {
  const geometry_msgs__msg__Twist* t = (const geometry_msgs__msg__Twist*)msgin;
  float lin = constrain((float)t->linear.x,  -1.0f, 1.0f);
  float ang = constrain((float)t->angular.z, -1.0f, 1.0f);
  applyDrive(lin, ang);
  lastCmdMs = millis();
}

void servoCallback(const void* msgin) {
  const std_msgs__msg__Int32* m = (const std_msgs__msg__Int32*)msgin;
  int a = constrain((int)m->data, 0, 180);
  if (a != servoAngle) {
    servoAngle  = a;
    servo.write(a);
    servoMoveMs = millis();
  }
}

bool createEntities() {
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "esp32_avoider", "", &support) != RCL_RET_OK) return false;

  if (rclc_publisher_init_best_effort(&tel_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/avoider/telemetry") != RCL_RET_OK) return false;

  if (rclc_publisher_init_best_effort(&info_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/avoider/info") != RCL_RET_OK) return false;

  if (rclc_subscription_init_best_effort(&cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/avoider/cmd_vel") != RCL_RET_OK) return false;

  if (rclc_subscription_init_best_effort(&servo_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/avoider/servo_cmd") != RCL_RET_OK) return false;

  executor = rclc_executor_get_zero_initialized_executor();
  if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg,
                                     &cmdCallback, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg,
                                     &servoCallback, ON_NEW_DATA) != RCL_RET_OK) return false;
  return true;
}

void destroyEntities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  (void) rcl_publisher_fini(&tel_pub, &node);
  (void) rcl_publisher_fini(&info_pub, &node);
  (void) rcl_subscription_fini(&cmd_sub, &node);
  (void) rcl_subscription_fini(&servo_sub, &node);
  (void) rclc_executor_fini(&executor);
  (void) rcl_node_fini(&node);
  (void) rclc_support_fini(&support);
}

void publishTelemetry() {
  bool ready = (millis() - servoMoveMs) >= SERVO_SETTLE_MS;
  telData[0]  = (float)servoAngle;
  telData[1]  = ready ? 1.0f : 0.0f;
  telData[2]  = distanceCm;
  telData[3]  = rttMs;
  telData[4]  = (float)WiFi.RSSI();
  telData[5]  = (float)WiFi.channel();
  telData[6]  = millis() / 1000.0f;
  telData[7]  = ESP.getFreeHeap() / 1024.0f;
  telData[8]  = motorL;
  telData[9]  = motorR;
  telData[10] = failsafe ? 1.0f : 0.0f;
  (void) rcl_publish(&tel_pub, &tel_msg, NULL);
}

void publishInfo() {
  snprintf(infoBuf, sizeof(infoBuf), "%s r%d|%s|%s|%s|%s|%d|%s",
           ESP.getChipModel(), (int)ESP.getChipRevision(),
           WiFi.macAddress().c_str(),
           WiFi.localIP().toString().c_str(),
           WiFi.SSID().c_str(),
           WiFi.BSSIDstr().c_str(),
           (int)ESP.getCpuFreqMHz(),
           ESP.getSdkVersion());
  info_msg.data.size = strlen(infoBuf);
  (void) rcl_publish(&info_pub, &info_msg, NULL);
}

void microRosTask() {
  switch (agentState) {
    case WAITING_AGENT:
      if (millis() - tAgent >= 500) {
        tAgent = millis();
        agentState = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
      }
      break;

    case AGENT_AVAILABLE:
      if (createEntities()) {
        agentState = AGENT_CONNECTED;
        tPing = tSensor = tInfo = millis();
        Serial.println("[ROS] Terhubung ke micro-ROS agent");
      } else {
        destroyEntities();
        agentState = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      if (millis() - tPing >= PING_PERIOD_MS) {     
        tPing = millis();
        unsigned long t0 = micros();
        if (RMW_RET_OK == rmw_uros_ping_agent(100, 3)) rttMs = (micros() - t0) / 1000.0f;
        else agentState = AGENT_DISCONNECTED;
      }
      if (agentState == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));   

        if (millis() - tSensor >= SENSOR_PERIOD_MS) {
          tSensor = millis();
          distanceCm = readDistance();
          publishTelemetry();
        }
        if (millis() - tInfo >= INFO_PERIOD_MS) {
          tInfo = millis();
          publishInfo();
        }
      }
      break;

    case AGENT_DISCONNECTED:
      Serial.println("[ROS] Koneksi agent terputus, mencoba lagi...");
      stopMotors();
      destroyEntities();
      agentState = WAITING_AGENT;
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(STBY, HIGH);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2400);
  servo.write(servoAngle);
  servoMoveMs = millis();

  pwmSetup();
  stopMotors();

  memset(&tel_msg, 0, sizeof(tel_msg));
  tel_msg.data.data     = telData;
  tel_msg.data.size     = TEL_LEN;
  tel_msg.data.capacity = TEL_LEN;

  memset(&info_msg, 0, sizeof(info_msg));
  info_msg.data.data     = infoBuf;
  info_msg.data.size     = 0;
  info_msg.data.capacity = sizeof(infoBuf);

  Serial.printf("\n=== ROBOT AVOIDER (micro-ROS) ===\nWiFi  : %s\nAgent : %s:%u\n",
                WIFI_SSID, AGENT_IP, AGENT_PORT);

  set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASS, (char*)AGENT_IP, AGENT_PORT);
  WiFi.setSleep(false);              

  Serial.print("IP ESP32: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  microRosTask();
  failsafe = (lastCmdMs == 0) || (millis() - lastCmdMs > CMD_TIMEOUT_MS);
  if (failsafe && (motorL != 0 || motorR != 0)) stopMotors();
}
