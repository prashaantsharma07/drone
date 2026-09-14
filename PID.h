#ifndef PID
#define PID

#include <Arduino.h>
#include "mpu6050.h" // for dt, AngleRoll, AnglePitch, RateRollDegS, RatePitchDegS, RateYawDegS


float Desired_Roll_Angle  = 0;  
float Desired_Pitch_Angle = 0;  
float Desired_Yaw_Rate    = 0;  

//Trim (small offsets to null out physical imbalance)
float Roll_Trim  = 0.0;
float Pitch_Trim = 0.0;

//Angle-loop gains (outer loop, Roll/Pitch only)
float P_Roll_Angle = 5,  I_Roll_Angle = 0.0, D_Roll_Angle = 0.1;
float P_Pitch_Angle = P_Roll_Angle, I_Pitch_Angle = I_Roll_Angle, D_Pitch_Angle = D_Roll_Angle;

//Rate-loop gains (inner loop, Roll/Pitch/Yaw)
float P_Roll_Rate = 0.8, I_Roll_Rate = 0.0, D_Roll_Rate = 0.1;
float P_Pitch_Rate = P_Roll_Rate, I_Pitch_Rate = I_Roll_Rate, D_Pitch_Rate = D_Roll_Rate;
float P_Yaw_Rate = 5, I_Yaw_Rate = 0.0, D_Yaw_Rate = 0.1;


float Roll_Angle_Error = 0, Prev_Roll_Angle_Error = 0, Roll_Angle_I_term = 0, Desired_Roll_Rate = 0;
float Pitch_Angle_Error = 0, Prev_Pitch_Angle_Error = 0, Pitch_Angle_I_term = 0, Desired_Pitch_Rate = 0;


float Roll_Rate_Error = 0, Prev_Roll_Rate_Error = 0, Roll_Rate_I_term = 0, Roll_PID_Output = 0;
float Pitch_Rate_Error = 0, Prev_Pitch_Rate_Error = 0, Pitch_Rate_I_term = 0, Pitch_PID_Output = 0;
float Yaw_Rate_Error = 0, Prev_Yaw_Rate_Error = 0, Yaw_Rate_I_term = 0, Yaw_PID_Output = 0;

const float PID_LIMIT = 400; // clamp - keep well inside the 1000-2000 range 

void calculateAnglePID() {
  // Roll
  Roll_Angle_Error = Desired_Roll_Angle - AngleRoll - Roll_Trim;
  float p = P_Roll_Angle * Roll_Angle_Error;
  Roll_Angle_I_term += I_Roll_Angle * Roll_Angle_Error * dt;
  Roll_Angle_I_term = constrain(Roll_Angle_I_term, -PID_LIMIT, PID_LIMIT);
  float d = D_Roll_Angle * ((Roll_Angle_Error - Prev_Roll_Angle_Error) / dt);
  Desired_Roll_Rate = constrain(p + Roll_Angle_I_term + d, -PID_LIMIT, PID_LIMIT);
  Prev_Roll_Angle_Error = Roll_Angle_Error;

  // Pitch
  Pitch_Angle_Error = Desired_Pitch_Angle - AnglePitch - Pitch_Trim;
  p = P_Pitch_Angle * Pitch_Angle_Error;
  Pitch_Angle_I_term += I_Pitch_Angle * Pitch_Angle_Error * dt;
  Pitch_Angle_I_term = constrain(Pitch_Angle_I_term, -PID_LIMIT, PID_LIMIT);
  d = D_Pitch_Angle * ((Pitch_Angle_Error - Prev_Pitch_Angle_Error) / dt);
  Desired_Pitch_Rate = constrain(p + Pitch_Angle_I_term + d, -PID_LIMIT, PID_LIMIT);
  Prev_Pitch_Angle_Error = Pitch_Angle_Error;

  // Yaw: no angle loop - Desired_Yaw_Rate is set directly by the app
}

void calculateRatePID() {
  // Roll
  Roll_Rate_Error = Desired_Roll_Rate - RateRollDegS;
  float p = P_Roll_Rate * Roll_Rate_Error;
  Roll_Rate_I_term += I_Roll_Rate * Roll_Rate_Error * dt;
  Roll_Rate_I_term = constrain(Roll_Rate_I_term, -PID_LIMIT, PID_LIMIT);
  float d = D_Roll_Rate * ((Roll_Rate_Error - Prev_Roll_Rate_Error) / dt);
  Roll_PID_Output = constrain(p + Roll_Rate_I_term + d, -PID_LIMIT, PID_LIMIT);
  Prev_Roll_Rate_Error = Roll_Rate_Error;

  // Pitch
  Pitch_Rate_Error = Desired_Pitch_Rate - RatePitchDegS;
  p = P_Pitch_Rate * Pitch_Rate_Error;
  Pitch_Rate_I_term += I_Pitch_Rate * Pitch_Rate_Error * dt;
  Pitch_Rate_I_term = constrain(Pitch_Rate_I_term, -PID_LIMIT, PID_LIMIT);
  d = D_Pitch_Rate * ((Pitch_Rate_Error - Prev_Pitch_Rate_Error) / dt);
  Pitch_PID_Output = constrain(p + Pitch_Rate_I_term + d, -PID_LIMIT, PID_LIMIT);
  Prev_Pitch_Rate_Error = Pitch_Rate_Error;

  // Yaw
  Yaw_Rate_Error = Desired_Yaw_Rate - RateYawDegS;
  p = P_Yaw_Rate * Yaw_Rate_Error;
  Yaw_Rate_I_term += I_Yaw_Rate * Yaw_Rate_Error * dt;
  Yaw_Rate_I_term = constrain(Yaw_Rate_I_term, -PID_LIMIT, PID_LIMIT);
  d = D_Yaw_Rate * ((Yaw_Rate_Error - Prev_Yaw_Rate_Error) / dt);
  Yaw_PID_Output = constrain(p + Yaw_Rate_I_term + d, -PID_LIMIT, PID_LIMIT);
  Prev_Yaw_Rate_Error = Yaw_Rate_Error;
}

// Call this on arm/disarm and whenever throttle drops near zero, or the
// integral terms will have wound up from ground handling and kick on takeoff.
void resetPID() {
  Roll_Angle_I_term = Pitch_Angle_I_term = 0;
  Roll_Rate_I_term = Pitch_Rate_I_term = Yaw_Rate_I_term = 0;
  Prev_Roll_Angle_Error = Prev_Pitch_Angle_Error = 0;
  Prev_Roll_Rate_Error = Prev_Pitch_Rate_Error = Prev_Yaw_Rate_Error = 0;
}
#endif
