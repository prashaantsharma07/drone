#ifndef MIXER
#define MIXER

#include <Arduino.h>
#include "PID.h"   // Roll_PID_Output, Pitch_PID_Output, Yaw_PID_Output
#include "Motor.h" // setMotorPWM()

// ---- Frame layout (confirmed: schematic MOT_1..4 + your spin test) ----
// Motor 1 = front-right, CW
// Motor 2 = front-left,  CCW
// Motor 3 = rear-left,   CW
// Motor 4 = rear-right,  CCW

// ---- Sign convention - UNVERIFIED, confirm before arming with props on ----
// Assumes +AngleRoll = right side down, +AnglePitch = nose up,
// +Yaw_PID_Output = commands clockwise rotation viewed from above.
// See verification steps below. If an axis fights instead of correcting,
// negate that PID_Output's sign in the four lines inside mixAndWrite().

float Throttle_Input = 1100; 

const int MOTOR_MIN_ARMED = 1050; 
const int MOTOR_MAX = 1900;       // headroom so a PID correction never saturates flat against 2000

void mixAndWrite() {
  float m1 = Throttle_Input - Pitch_PID_Output - Roll_PID_Output - Yaw_PID_Output; // front-right, CW
  float m2 = Throttle_Input - Pitch_PID_Output + Roll_PID_Output + Yaw_PID_Output; // front-left,  CCW
  float m3 = Throttle_Input + Pitch_PID_Output + Roll_PID_Output - Yaw_PID_Output; // rear-left,   CW
  float m4 = Throttle_Input + Pitch_PID_Output - Roll_PID_Output + Yaw_PID_Output; // rear-right,  CCW

  m1 = constrain(m1, MOTOR_MIN_ARMED, MOTOR_MAX);
  m2 = constrain(m2, MOTOR_MIN_ARMED, MOTOR_MAX);
  m3 = constrain(m3, MOTOR_MIN_ARMED, MOTOR_MAX);
  m4 = constrain(m4, MOTOR_MIN_ARMED, MOTOR_MAX);

  setMotorPWM(1, (uint32_t)m1);
  setMotorPWM(2, (uint32_t)m2);
  setMotorPWM(3, (uint32_t)m3);
  setMotorPWM(4, (uint32_t)m4);
}

#endif
