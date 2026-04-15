#ifndef PID_H
#define PID_H

/**
 * PID Controller for angular rate and angle control.
 *
 * Uses derivative-on-measurement to avoid derivative kick on setpoint changes.
 *
 * IMPORTANT sign convention:
 *   error     = target - measured                          (normal error)
 *   d(error)  ≈ -d(measured)   for constant target
 *   D_term    = -Kd * d(measured)/dt                      (must SUBTRACT)
 *   output    = P + I - D                                  <-- correct
 *
 * Using "+D" with derivative-on-measurement is positive feedback and will
 * cause twitching / instability whenever Kd != 0.
 */
class PIDController {
public:
  float Kp, Ki, Kd;
  float integral = 0.0f;
  float lastError = 0.0f;
  unsigned long lastTime = 0;
  float integralLimit = 500.0f;  // Prevent integral wind-up

  // last computed terms for diagnostics
  float lastP = 0.0f;
  float lastI = 0.0f;
  float lastD = 0.0f;

  // for derivative-on-measurement filtering
  float lastMeasured = 0.0f;
  float dFilterCoef = 0.1f;           // [0..1] smoothing coefficient (lower = more smoothing)
  float lastDerivative = 0.0f;

  /**
   * Constructor
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   */
  PIDController(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f)
    : Kp(kp), Ki(ki), Kd(kd) {}

  /**
   * Update PID controller
   * @param target  Setpoint (deg for angle loop, deg/s for rate loop)
   * @param measured Measured value in the same units
   * @return Control output (microseconds, ±500 range)
   */
  float update(float target, float measured) {
    float error = target - measured;
    unsigned long now = micros();

    // Initialize time on first call
    if (lastTime == 0) {
      lastTime = now;
      lastError = error;
      lastMeasured = measured;
      return 0.0f;
    }

    // Calculate dt in seconds
    float dt = (now - lastTime) / 1e6f;
    lastTime = now;

    // Skip if dt is out of sane range
    if (dt < 0.0001f || dt > 0.1f) {
      return 0.0f;
    }

    // Small deadband to avoid chasing sensor noise
    if (fabs(error) < 0.5f) {
      error = 0.0f;
    }
    

    // Proportional term
    float P = Kp * error;

    // Integral term with anti-windup
    integral += Ki * error * dt;
    if (integral > integralLimit) integral = integralLimit;
    if (integral < -integralLimit) integral = -integralLimit;
    float I = integral;

    // Derivative-on-measurement: track slope of measured value.
    // d(measured)/dt is computed here; it is subtracted below because
    // d(error)/dt = -d(measured)/dt for a constant setpoint.
    float rawDerivative = (measured - lastMeasured) / dt;
    lastMeasured = measured;

    // Low-pass filter the derivative (simple IIR)
    float derivative = dFilterCoef * rawDerivative + (1.0f - dFilterCoef) * lastDerivative;
    lastDerivative = derivative;

    // FIX: D is computed from measured slope (not error slope), so it must
    // be SUBTRACTED.  Using "+D" here would be positive feedback and causes
    // the observed random twitching whenever Kd != 0.
    float D = Kd * derivative;


    // Store for diagnostics
    lastP = P;
    lastI = I;
    lastD = D;
    lastError = error;

    // Correct combination: subtract D because we used derivative-on-measurement
    float output = P + I - D;

    // Clamp output to ±500 microseconds
    if (output > 500.0f) output = 500.0f;
    if (output < -500.0f) output = -500.0f;

    return output;
  }

  /**
   * Reset the PID controller state
   */
  void reset() {
    integral = 0.0f;
    lastError = 0.0f;
    lastTime = 0;
    lastMeasured = 0.0f;
    lastDerivative = 0.0f;
  }
};

#endif  // PID_H