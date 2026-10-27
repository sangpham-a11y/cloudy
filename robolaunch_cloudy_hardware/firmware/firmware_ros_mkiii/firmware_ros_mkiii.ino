// =============================================================================
//  Cloudy MKIII - firmware ESP32 cho giai doan ROS 2 (khong dung tay phat RC)
//  Viet lai tu firmware_flysky.ino cua robolaunch/cloudy.
//
//  Thay doi so voi ban goc:
//   [1] Bo chan Enable rieng tung motor (ban goc trung chan DIR va chan LED).
//       Bat driver bang jumper EN/GND tren shield; IO12 cung keo LOW (du phong).
//       Driver: TMC2209 che do standalone. Jumper tren CNC shield:
//         - EN/GND (canh nut RESET)
//         - M0 + M1 duoi khe Y va khe Z -> vi buoc 1/16 (khong cam M2)
//       Vref TMC2209: I_RMS ~ 0.71 x Vref -> dat ~1.2-1.4 V cho motor 17HS8401B.
//   [2] Tu ket noi lai micro-ROS: ESP32 cho agent tren Pi, mat ket noi thi
//       dung motor va cho ket noi lai. Khong can bam RESET nua.
//       (ban 2: ping 1 s/lan, 3 lan lien tiep that bai moi coi la mat ket noi)
//   [3] Mat lenh toc do qua 500 ms -> motor tu dung.
//   [4] Them ti so truyen banh rang (GEAR_RATIO) va dao chieu tung motor.
//   [5] Den xi-nhan / khan cap nhap nhay that (ban goc chi chop mot lan).
//       Them lenh 0 = tat den.
//   [6] Bo toan bo phan tay phat RC (IBusBM).
//
//  Thu vien: micro_ros_arduino v2.0.5-humble, FastAccelStepper 0.28.3,
//            Adafruit NeoPixel 1.11.0. Board: esp32 by Espressif 2.0.2.
// =============================================================================
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int8.h>
#include "FastAccelStepper.h"
#include <Adafruit_NeoPixel.h>
#include <driver/gpio.h>

// ------------------------- CAU HINH - CHINH O DAY -----------------------------
// Ti so truyen = so rang banh rang lon 0601_008 / so rang pinion 0601_007.
// ROS 2 gui toc do BANH XE; motor phai quay nhanh hon GEAR_RATIO lan.
// De 1.0 cho toi khi dem duoc so rang that.
// Chi dat ti so truyen O MOT NOI: hoac o day, hoac trong diffbot_system.cpp.
#define GEAR_RATIO        1.0f

// Dat true neu banh tuong ung quay nguoc khi ra lenh tien.
#define INVERT_LEFT       false
#define INVERT_RIGHT      false

// Vi buoc TMC2209 standalone theo jumper duoi moi khe (M0 = MS1, M1 = MS2; M2 khong dung):
//   KHONG cam jumper   -> 1/8  -> 1600.0f
//   chi M0             -> 1/32 -> 6400.0f
//   chi M1             -> 1/64 -> 12800.0f
//   cam M0 + M1        -> 1/16 -> 3200.0f   (dang dung)
// (Neu dung DRV8825: M0 + M1 = 1/8 -> 1600.0f, chi M2 = 1/16 -> 3200.0f.)
#define STEPS_PER_REV     3200.0f
#define MAX_MOTOR_RAD_S   60.0f                // gioi han toc do truc motor (~570 vong/phut; ban cu 23)
#define ACCELERATION      12000                // buoc/s^2 (ban goc 16000)
#define CMD_TIMEOUT_MS    500                  // mat lenh qua muc nay -> dung
#define LED_BRIGHTNESS    128                  // 0..255; 128 ~ 50 %

// Che do test dien (tach loi DIR / driver / nguon khoi loi FastAccelStepper):
//   1 -> KHONG chay micro-ROS. ESP32 tu phat xung STEP bang digitalWrite, lap lai:
//        TIEN nhanh 10 s -> dung 3 s -> LUI nhanh 10 s -> dung 3 s -> LUI cham 10 s -> dung 3 s
//        Serial 115200 in ra pha hien tai va muc chan DIR. Do chan DIR/EN bang dong ho.
//   0 -> firmware ROS 2 binh thuong. NHO DAT LAI 0 SAU KHI TEST.
#define MOTOR_DIR_TEST    0
#define TEST_FAST_HZ      ((uint32_t)(1.5f * STEPS_PER_REV / (2.0f * PI)))   // 1.5 rad/s
#define TEST_SLOW_HZ      ((uint32_t)(0.4f * STEPS_PER_REV / (2.0f * PI)))   // 0.4 rad/s
#define TEST_RUN_MS       10000
#define TEST_STOP_MS      3000

// ------------------------- CHAN (Wemos D1 R32 + CNC shield) -----------------
// Motor trai: khe Z cua shield. Motor phai: khe Y. Khe X bo trong.
#define STEP_LEFT   17
#define DIR_LEFT    14
#define STEP_RIGHT  25
#define DIR_RIGHT   27
#define LED_PIN     26      // X.STEP tren shield -> DIN day LED 1
#define EN_PIN      12      // D8 = chan EN chung cua CNC shield (LOW = bat driver)
                            // Da cam jumper EN/GND; keo LOW them o day vo hai, de nguyen.
#define STATUS_LED  2       // den tren board: sang = da ket noi Pi
#define NUM_PIXELS  32
// -----------------------------------------------------------------------------

const float MOTOR_RAD_TO_STEP = STEPS_PER_REV / (2.0f * PI);   // so buoc cho moi rad truc motor

FastAccelStepperEngine engine;
FastAccelStepper *leftStepper  = NULL;
FastAccelStepper *rightStepper = NULL;
Adafruit_NeoPixel pixels(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_subscription_t l_sub, r_sub, led_sub;
std_msgs__msg__Float32 l_msg, r_msg;
std_msgs__msg__Int8 led_msg;

volatile unsigned long last_cmd_ms = 0;
int led_mode = 0;                 // 0 tat, 1 den pha, 2 khan cap, 3 xi-nhan trai, 4 xi-nhan phai
unsigned long led_last_toggle = 0;
bool led_blink_on = false;

enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { return false; } }
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis(); } \
  if (uxr_millis() - init > MS) { X; init = uxr_millis(); } \
} while (0)

// ------------------------- MOTOR ---------------------------------------------
void setMotor(FastAccelStepper *s, float wheel_rad_s) {
  if (s == NULL) return;
  // Toc do banh xe (ROS 2 gui) -> toc do truc motor -> so buoc/giay
  float motor_rad_s = constrain(wheel_rad_s * GEAR_RATIO, -MAX_MOTOR_RAD_S, MAX_MOTOR_RAD_S);
  uint32_t hz = (uint32_t)(fabs(motor_rad_s) * MOTOR_RAD_TO_STEP);
  if (hz == 0) { s->stopMove(); return; }
  s->setSpeedInHz(hz);
  if (motor_rad_s > 0) s->runForward(); else s->runBackward();
}

void stopMotors() {
  if (leftStepper)  leftStepper->stopMove();
  if (rightStepper) rightStepper->stopMove();
}

void left_cb(const void *msgin) {
  const std_msgs__msg__Float32 *m = (const std_msgs__msg__Float32 *)msgin;
  last_cmd_ms = millis();
  setMotor(leftStepper, m->data);
}

void right_cb(const void *msgin) {
  const std_msgs__msg__Float32 *m = (const std_msgs__msg__Float32 *)msgin;
  last_cmd_ms = millis();
  setMotor(rightStepper, m->data);
}

// ------------------------- LED -----------------------------------------------
// Thu tu chuoi: day 1 truoc-phai (0-7), day 2 truoc-trai (8-15),
//               day 3 sau-trai (16-23), day 4 sau-phai (24-31).
const uint32_t WHITE = Adafruit_NeoPixel::Color(200, 200, 200);
const uint32_t REAR  = Adafruit_NeoPixel::Color(200, 0, 100);
const uint32_t AMBER = Adafruit_NeoPixel::Color(255, 69, 0);

void fillRange(int from, int to, uint32_t c) { for (int i = from; i <= to; i++) pixels.setPixelColor(i, c); }

void renderLeds() {
  pixels.clear();
  switch (led_mode) {
    case 1: fillRange(0, 15, WHITE); fillRange(16, 31, REAR); break;
    case 2: if (led_blink_on) fillRange(0, 31, AMBER); break;
    case 3: if (led_blink_on) fillRange(8, 23, AMBER); break;               // ben trai
    case 4: if (led_blink_on) { fillRange(0, 7, AMBER); fillRange(24, 31, AMBER); } break;  // ben phai
    default: break;                                                          // 0: tat
  }
  pixels.show();
}

void led_cb(const void *msgin) {
  const std_msgs__msg__Int8 *m = (const std_msgs__msg__Int8 *)msgin;
  led_mode = m->data;
  led_blink_on = true;
  led_last_toggle = millis();
  renderLeds();
}

void updateBlink() {
  if (led_mode >= 2 && millis() - led_last_toggle >= 500) {
    led_blink_on = !led_blink_on;
    led_last_toggle = millis();
    renderLeds();
  }
}

// ------------------------- micro-ROS ------------------------------------------
bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  RCCHECK(rclc_subscription_init_default(&l_sub, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "left_motor_speed"));
  RCCHECK(rclc_subscription_init_default(&r_sub, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "right_motor_speed"));
  RCCHECK(rclc_subscription_init_default(&led_sub, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "neopixel_led"));
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &l_sub, &l_msg, &left_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &r_sub, &r_msg, &right_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &led_sub, &led_msg, &led_cb, ON_NEW_DATA));
  return true;
}

void destroy_entities() {
  rmw_context_t *ctx = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0);
  rcl_subscription_fini(&l_sub, &node);
  rcl_subscription_fini(&r_sub, &node);
  rcl_subscription_fini(&led_sub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// Kiem tra agent con song khong. Mot lan ping that bai le te (do cong serial dang ban)
// khong duoc lam ngat ket noi; chi khi that bai 3 lan lien tiep moi chuyen sang mat ket noi.
int ping_fail = 0;
void checkAgent() {
  if (RMW_RET_OK == rmw_uros_ping_agent(200, 3)) {
    ping_fail = 0;
  } else if (++ping_fail >= 3) {
    ping_fail = 0;
    state = AGENT_DISCONNECTED;
  }
}

// ------------------------- TEST DIEN (MOTOR_DIR_TEST = 1) ---------------------
// Chan DIR dong xuat toi da (~40 mA) de giu muc 0V cung hon khi GND shield bi dao dong
// (loi: LUI = DIR LOW bi doc nham thanh HIGH -> motor rung tai cho, khoa cung).
void strongDirPins() {
  gpio_set_drive_capability((gpio_num_t)DIR_LEFT,  GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)DIR_RIGHT, GPIO_DRIVE_CAP_3);
}

// Muc DIR "tien" giong FastAccelStepper: setDirectionPin(pin, !INVERT) -> HIGH la tien.
void testRun(const char *label, bool forward, uint32_t hz, uint32_t ms) {
  digitalWrite(DIR_LEFT,  forward != INVERT_LEFT  ? HIGH : LOW);
  digitalWrite(DIR_RIGHT, forward != INVERT_RIGHT ? HIGH : LOW);
  delayMicroseconds(100);                      // cho DIR on dinh truoc xung STEP dau tien
  Serial.printf("[%8lu ms] %-10s DIR_L(IO14)=%d DIR_R(IO27)=%d EN(IO12)=%d %lu Hz\n",
                millis(), label, digitalRead(DIR_LEFT), digitalRead(DIR_RIGHT),
                digitalRead(EN_PIN), hz);
  uint32_t period = 1000000UL / hz;
  uint32_t start = millis();
  while (millis() - start < ms) {
    digitalWrite(STEP_LEFT, HIGH);
    digitalWrite(STEP_RIGHT, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP_LEFT, LOW);
    digitalWrite(STEP_RIGHT, LOW);
    delayMicroseconds(period - 5);
  }
}

void testStop() {
  Serial.printf("[%8lu ms] DUNG\n", millis());
  delay(TEST_STOP_MS);
}

void motorDirTest() {                          // khong bao gio tro ve
  Serial.begin(115200);
  pinMode(STEP_LEFT, OUTPUT);  digitalWrite(STEP_LEFT, LOW);
  pinMode(STEP_RIGHT, OUTPUT); digitalWrite(STEP_RIGHT, LOW);
  pinMode(DIR_LEFT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);
  strongDirPins();
  delay(2000);
  Serial.println("MOTOR_DIR_TEST bat dau (micro-ROS tat)");
  while (true) {
    testRun("TIEN nhanh", true,  TEST_FAST_HZ, TEST_RUN_MS); testStop();
    testRun("LUI nhanh",  false, TEST_FAST_HZ, TEST_RUN_MS); testStop();
    testRun("LUI cham",   false, TEST_SLOW_HZ, TEST_RUN_MS); testStop();
  }
}

// ------------------------- SETUP / LOOP ---------------------------------------
void setup() {
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  // Bat driver bang phan mem (thay cho jumper EN/GND)
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  pixels.begin();
  pixels.setBrightness(LED_BRIGHTNESS);
  pixels.clear();
  pixels.show();

#if MOTOR_DIR_TEST
  motorDirTest();                              // chay test, bo qua FastAccelStepper va micro-ROS
#endif

  engine.init();
  leftStepper  = engine.stepperConnectToPin(STEP_LEFT);
  rightStepper = engine.stepperConnectToPin(STEP_RIGHT);
  if (leftStepper) {
    leftStepper->setDirectionPin(DIR_LEFT, !INVERT_LEFT);
    leftStepper->setAcceleration(ACCELERATION);
  }
  if (rightStepper) {
    rightStepper->setDirectionPin(DIR_RIGHT, !INVERT_RIGHT);
    rightStepper->setAcceleration(ACCELERATION);
  }
  strongDirPins();                             // sau setDirectionPin (thu vien da pinMode OUTPUT)

  set_microros_transports();
  state = WAITING_AGENT;
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) destroy_entities();
      break;
    case AGENT_CONNECTED:
      // Kiem tra ket noi moi 1 s; chi coi la mat ket noi khi 3 lan lien tiep that bai
      EXECUTE_EVERY_N_MS(1000, checkAgent(););
      if (state == AGENT_CONNECTED) rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      break;
    case AGENT_DISCONNECTED:
      stopMotors();
      led_mode = 0; renderLeds();
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }

  digitalWrite(STATUS_LED, state == AGENT_CONNECTED ? HIGH : LOW);

  // Mat lenh toc do -> dung motor
  if (last_cmd_ms != 0 && millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    stopMotors();
    last_cmd_ms = 0;
  }

  updateBlink();
}
