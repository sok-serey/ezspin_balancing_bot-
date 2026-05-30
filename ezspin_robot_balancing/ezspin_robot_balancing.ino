#include <SimpleFOC.h>
#include "balancer_pinouts.h"
#include "imu.h"
#include "can.h"
#include "BluetoothSerial.h"

BluetoothSerial bluetooth;

// ─────────────────────────────────────────────────────────────────────────────
// BALANCING ENABLE FLAG
// Set via CAN ID 0x010 (defined in can.h):
//   data[0] = 0x01 → enable  (R2 on gamepad)
//   data[0] = 0x00 → disable (L2 on gamepad)
// Serial command: 'E' = enable, 'D' = disable
// ─────────────────────────────────────────────────────────────────────────────
volatile bool balancing_enabled = true;

// ─────────────────────────────────────────────────────────────────────────────
// JOYSTICK cmd_vel — received via CAN ID 0x020
//   cmd_vel_linear  = linear.x  [m/s]  — forward/back
//   cmd_vel_angular = angular.z [rad/s] — turn left/right
//
// How they affect balancing:
//   linear  → added to velocity setpoint in outer loop
//             robot leans forward/back to achieve the commanded speed
//   angular → voltage difference between motors for turning
//             motor1 gets +turn, motor2 gets -turn
// ─────────────────────────────────────────────────────────────────────────────
volatile float cmd_vel_linear  = 0.0f;
volatile float cmd_vel_angular = 0.0f;

// Scale factors — tune to match your robot's speed
#define CMD_VEL_LINEAR_SCALE  15.0f   // 1.0 m/s cmd → 1.0 rad/s velocity setpoint
#define CMD_VEL_ANGULAR_SCALE 1.5f   // 1.0 rad/s cmd → 0.5 V difference between motors

// ─────────────────────────────────────────────────────────────────────────────
// LOW PASS FILTER
// ─────────────────────────────────────────────────────────────────────────────
struct MyLPF {
  float Tf;
  float _y;
  unsigned long _prev_us;

  MyLPF(float tf) : Tf(tf), _y(0.0f), _prev_us(0) {}

  float operator()(float x) {
    unsigned long now = micros();
    float dt = (float)(now - _prev_us) * 1e-6f;
    if (_prev_us == 0 || dt <= 0.0f || dt > 0.5f) {
      _prev_us = now; _y = x; return _y;
    }
    _prev_us = now;
    _y = _y + (x - _y) * dt / (Tf + dt);
    return _y;
  }

  void reset() { _y = 0.0f; _prev_us = 0; }
};

// ─────────────────────────────────────────────────────────────────────────────
// PID CONTROLLER
// Constructor order: (Kp, Ki, Kd, integral_limit, output_limit)
// D term on error — matches SimpleFOC pid.cpp exactly
// ─────────────────────────────────────────────────────────────────────────────
struct MyPID {
  float Kp, Ki, Kd;
  float integral_limit;
  float output_limit;
  float _integral;
  float _prev_error;
  unsigned long _prev_us;

  MyPID(float kp, float ki, float kd, float integ_lim, float out_lim)
    : Kp(kp), Ki(ki), Kd(kd),
      integral_limit(integ_lim), output_limit(out_lim),
      _integral(0.0f), _prev_error(0.0f), _prev_us(0) {}

  float operator()(float error) {
    unsigned long now = micros();
    float dt = (float)(now - _prev_us) * 1e-6f;
    if (_prev_us == 0 || dt <= 0.0f || dt > 0.5f) {
      _prev_us = now; _prev_error = error; return 0.0f;
    }
    _prev_us = now;
    float P = Kp * error;
    _integral += Ki * error * dt;
    _integral  = constrain(_integral, -integral_limit, integral_limit);
    float D = Kd * (error - _prev_error) / dt;
    _prev_error = error;
    return constrain(P + _integral + D, -output_limit, output_limit);
  }

  void reset() {
    _integral = 0.0f; _prev_error = 0.0f; _prev_us = 0;
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// ENCODERS & MOTORS
// NOTE: named 'motor' and 'motor2' to match can.cpp extern
// ─────────────────────────────────────────────────────────────────────────────
MagneticSensorSPI encoder1 = MagneticSensorSPI(AS5147_SPI, 15);
MagneticSensorSPI encoder2 = MagneticSensorSPI(AS5147_SPI, 16);

BLDCMotor motor  = BLDCMotor(11);
BLDCMotor motor2 = BLDCMotor(11);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(MOT1_A, MOT1_B, MOT1_C, MOT1_EN);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(MOT2_A, MOT2_B, MOT2_C, MOT2_EN);

// ─────────────────────────────────────────────────────────────────────────────
// CAN motor targets — defined here, extern'd in can.h
// ─────────────────────────────────────────────────────────────────────────────
float       target_position  = 0.0f;
float       target_velocity  = 0.0f;
float       target_torque    = 0.0f;
MessageType current_mode     = CONTROL_TORQUE;

float       target_position2 = 0.0f;
float       target_velocity2 = 0.0f;
float       target_torque2   = 0.0f;
MessageType current_mode2    = CONTROL_TORQUE;

// ─────────────────────────────────────────────────────────────────────────────
// CONTROLLERS  (Kp, Ki, Kd, integral_limit, output_limit)
// ─────────────────────────────────────────────────────────────────────────────
MyPID pid_stb(18.0f, 60.0f, 0.4f, 50000.0f, 6.0f      );  // inner: full PID
MyPID pid_vel(0.01f, 0.03f, 0.0f, 10000.0f, _PI/10.0f );  // outer: PI

MyLPF lpf_pitch_cmd(0.07f);

float target_pitch = 0.0f;

// ── CAN encoder feedback timing ───────────────────────────────────────────────
static unsigned long lastFeedbackUs = 0;
const  unsigned long FEEDBACK_US    = 10000;  // 100 Hz

// ─────────────────────────────────────────────────────────────────────────────
// STOP MOTORS — zero torque + reset all controllers
// ─────────────────────────────────────────────────────────────────────────────
void stopMotors() {
  motor.target  = 0.0f;
  motor2.target = 0.0f;
  pid_stb.reset();
  pid_vel.reset();
  lpf_pitch_cmd.reset();
  target_pitch = 0.0f;
  Serial.println(F("[BAL] DISABLED — motors zeroed"));
  bluetooth.println(F("[BAL] DISABLED — motors zeroed"));
}

void enableBalancing() {
  pid_stb.reset();
  pid_vel.reset();
  lpf_pitch_cmd.reset();
  target_pitch = 0.0f;
  balancing_enabled = true;
  Serial.println(F("[BAL] ENABLED — balancing started"));
  bluetooth.println(F("[BAL] ENABLED — balancing started"));
}

// ─────────────────────────────────────────────────────────────────────────────
// TUNING  (Serial Monitor or Bluetooth terminal)
//   E       → enable balancing
//   D       → disable balancing
//   SP18    → pid_stb Kp      SI60   → pid_stb Ki     SD0.4  → pid_stb Kd
//   VP0.01  → pid_vel Kp      VI0.03 → pid_vel Ki
//   FC0.07  → lpf_pitch_cmd Tf
//   ?       → print all gains
// ─────────────────────────────────────────────────────────────────────────────
void printGains(Stream& port) {
  port.printf("stb  Kp=%.3f Ki=%.3f Kd=%.4f  integ_lim=%.0f out_lim=%.2f\n",
              pid_stb.Kp, pid_stb.Ki, pid_stb.Kd,
              pid_stb.integral_limit, pid_stb.output_limit);
  port.printf("vel  Kp=%.4f Ki=%.4f  integ_lim=%.0f out_lim=%.4f\n",
              pid_vel.Kp, pid_vel.Ki,
              pid_vel.integral_limit, pid_vel.output_limit);
  port.printf("lpf  cmd_Tf=%.3f\n", lpf_pitch_cmd.Tf);
  port.printf("enabled=%d  lin=%.3f  ang=%.3f\n",
              balancing_enabled, (float)cmd_vel_linear, (float)cmd_vel_angular);
}

void handleTuning(Stream& port) {
  if (!port.available()) return;
  String line = port.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("E")) { enableBalancing();                       return; }
  if (line.equalsIgnoreCase("D")) { balancing_enabled = false; stopMotors(); return; }
  if (line[0] == '?')            { printGains(Serial); printGains(bluetooth); return; }
  if (line.length() < 3) return;

  char  grp   = toupper(line[0]);
  char  param = toupper(line[1]);
  float val   = line.substring(2).toFloat();

  if      (grp == 'S' && param == 'P') pid_stb.Kp = val;
  else if (grp == 'S' && param == 'I') pid_stb.Ki = val;
  else if (grp == 'S' && param == 'D') pid_stb.Kd = val;
  else if (grp == 'V' && param == 'P') pid_vel.Kp = val;
  else if (grp == 'V' && param == 'I') { pid_vel.Ki = val; pid_vel.reset(); }
  else if (grp == 'F' && param == 'C') lpf_pitch_cmd.Tf = val;

  printGains(Serial);
  printGains(bluetooth);
}

// ─────────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(250000);
  bluetooth.begin("FOCBalancer");
  _delay(1000);

  if (!initIMU()) {
    Serial.println(F("IMU error - halting!"));
    while (true);
  }
  _delay(1000);

  initCAN();

  encoder1.init();
  encoder2.init();
  motor.linkSensor(&encoder1);
  motor2.linkSensor(&encoder2);

  driver1.voltage_power_supply = 8;
  driver1.init();
  motor.linkDriver(&driver1);

  driver2.voltage_power_supply = 8;
  driver2.init();
  motor2.linkDriver(&driver2);

  motor.controller  = MotionControlType::torque;
  motor2.controller = MotionControlType::torque;

  motor.useMonitoring(Serial);
  motor2.useMonitoring(Serial);

  motor.init();
  motor2.init();
  motor.initFOC();
  motor2.initFOC();

  Serial.println(F("Ready — send 'E' to enable, 'D' to disable"));
  Serial.println(F("SP/SI/SD  VP/VI  FC  ?=gains"));
  bluetooth.println(F("SP/SI/SD  VP/VI  FC  ?=gains"));
}

// ─────────────────────────────────────────────────────────────────────────────
// MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  // ── FOC — must run as fast as possible ───────────────────────────────────
  motor.loopFOC();
  motor2.loopFOC();

  motor.move();
  motor2.move();

  // ── CAN receive — handles enable + cmd_vel + motor commands ──────────────
  receiveCANMessages();

  // ── IMU — update cache every loop ────────────────────────────────────────
  loopIMU();

  if (cache_valid) {
    cache_valid = false;

    // Always send IMU so OrangePi can monitor angle even when disabled
    sendIMUData();

    if (!balancing_enabled) {
      motor.target  = 0.0f;
      motor2.target = 0.0f;

    } else {
      float pitch   = getPitchIMU();
      float avg_vel = (motor.shaft_velocity + motor2.shaft_velocity) / 2.0f;

      // ── Outer loop: PI velocity → target pitch ──────────────────────────
      // cmd_vel_linear shifts the velocity setpoint:
      //   linear > 0 = move forward → robot leans forward slightly
      //   linear < 0 = move backward → robot leans back slightly
      float vel_setpoint = cmd_vel_linear * CMD_VEL_LINEAR_SCALE;
      float cmd    = pid_vel(avg_vel - vel_setpoint);
      target_pitch = lpf_pitch_cmd(cmd);
      target_pitch = constrain(target_pitch, -0.35f, 0.35f);

      // ── Inner loop: PID pitch error → base voltage ──────────────────────
      float voltage = pid_stb(target_pitch - pitch);

      // ── Turning: add angular difference between motors ──────────────────
      // cmd_vel_angular > 0 = turn left  → motor1 faster, motor2 slower
      // cmd_vel_angular < 0 = turn right → motor2 faster, motor1 slower
      float turn = cmd_vel_angular * CMD_VEL_ANGULAR_SCALE;

      motor.target  = constrain(voltage + turn, -6.0f, 6.0f);
      motor2.target = constrain(voltage - turn, -6.0f, 6.0f);
    }
  }

  // ── CAN encoder feedback at 100 Hz ───────────────────────────────────────
  unsigned long now = micros();
  if (now - lastFeedbackUs >= FEEDBACK_US) {
    lastFeedbackUs = now;
    sendFeedback();
    sendFeedback2();
  }

  // ── Serial / Bluetooth tuning ─────────────────────────────────────────────
  handleTuning(Serial);
  handleTuning(bluetooth);
}
