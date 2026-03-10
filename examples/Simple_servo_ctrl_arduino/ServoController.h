#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

/*
===============================================================================
File:         ServoController.h
Author:       Alejandro Alonso Puig + GPT
Date:         2026-03-10
License:      Apache 2.0
-------------------------------------------------------------------------------
Description:

Header file for a generic Arduino servo-control library based on a managed
single-servo abstraction.

This library is intended to provide a reusable control layer for hobby servos
in educational, maker, and robotics applications. It supports:

- One object per servo
- Trapezoidal motion profile
- Speed control expressed as % of real physical max speed
- Acceleration control expressed as % of a global software reference
- Optional analog feedback from the servo internal potentiometer
- Optional no-movement fault detection
- Query methods for state, position, velocity, PWM, and diagnostics

This file only declares the API and internal data members. Implementation is
provided separately in ServoController.cpp.

Important design notes:

1) max_speed_degps is stored per servo in ServoConfig because it depends on the
   real actuator or joint, including any gearbox or reduction stage.

2) acceleration uses a global software reference (ACCEL_MAX_DEGPS2), because
   hobby-servo datasheets rarely provide a physically meaningful acceleration
   value. Therefore, acceleration percentages are interpreted as:
       active_accel = ACCEL_MAX_DEGPS2 * (accel_pct / 100)

3) The library is generic and does NOT manage:
   - power MOSFETs
   - subsystem enable/disable hardware
   - ROS / Xicro communication
   - global robot-level safety logic
   Those responsibilities are expected to remain in the application sketch.

===============================================================================
*/

#include <Arduino.h>
#include <Servo.h>

// ============================================================================
// Global library tuning constants
// ============================================================================
//
// These constants define the generic behaviour of the library itself.
// They are not servo-specific and therefore do not belong in ServoConfig.
//

// Software reference maximum acceleration.
// 100 % commanded acceleration means this value.
// This is a motion-profile design parameter, not a measured physical property
// taken from a servo datasheet.
static constexpr float ACCEL_MAX_DEGPS2 = 800.0f;

// Angular tolerance used to decide that a servo has effectively reached
// its target. This helps avoid endless tiny corrections due to quantization
// or numerical effects in the profile generator.
static constexpr float TARGET_EPS_DEG = 0.2f;

// Minimum elapsed time between effective update() computations.
// If update() is called faster than this, the call can be ignored.
// This protects the profile logic from extremely small dt values.
static constexpr unsigned long MIN_UPDATE_INTERVAL_MS = 5;

// Fault detection arming threshold.
// If the tracking error is below this value, the library assumes that no
// significant movement is being demanded and therefore stall detection is
// not armed.
static constexpr float FAULT_ARM_ERROR_DEG = 3.0f;

// Minimum measured motion that counts as "real movement" when feedback exists.
// If the servo should be moving but measured angle changes by less than this
// amount for too long, a fault may be raised.
static constexpr float FAULT_MOVE_EPS_DEG = 0.25f;

// Maximum allowed duration of the "should move but does not move" condition
// before latching a fault.
static constexpr unsigned long FAULT_TIMEOUT_MS = 600;

// Special value used in ServoConfig to indicate that the servo has no analog
// feedback input connected.
static constexpr int NO_FEEDBACK_PIN = -1;

// ============================================================================
// ServoState
// ============================================================================
//
// Logical runtime state of one ServoController instance.
//
// UNINITIALIZED:
//   The object exists, but it has not yet been correctly synchronized to a
//   known starting position.
//
// IDLE:
//   The servo is operational but not currently executing a relevant motion.
//   This state may be used after resetFault() or for future extensions.
//
// MOVING:
//   The motion profile is actively moving the internal command toward target.
//
// HOLDING:
//   The servo has reached target (within tolerance) and is holding position.
//
// FAULT:
//   A latched no-movement / fault condition has been detected.
//

enum class ServoState {
    UNINITIALIZED,
    IDLE,
    MOVING,
    HOLDING,
    FAULT
};

// ============================================================================
// ServoConfig
// ============================================================================
//
// Static per-servo configuration structure.
//
// This structure is meant to be defined externally by the application sketch,
// typically in a configuration table containing one entry per servo.
//
// The library copies this configuration internally during begin().
//
// The fields are divided conceptually into:
//
// - Identification
// - PWM pin assignment
// - Physical calibrated servo range
// - Application-level allowed range
// - Rest angle
// - PWM calibration
// - Real physical maximum speed
// - Default motion percentages
// - Optional feedback ADC calibration
// - Direction inversion
// - Fault detection enable flag
//

struct ServoConfig {
    const char* name;                 // Short logical servo name for debugging, logs, or mapping.

    int pwm_pin;                      // Arduino digital pin used to output PWM to the servo signal wire.

    float servo_min_deg;              // Lower calibrated physical angle used for PWM / feedback conversion.
    float servo_max_deg;              // Upper calibrated physical angle used for PWM / feedback conversion.

    float allowed_min_deg;            // Lower application-specific allowed motion limit.
    float allowed_max_deg;            // Upper application-specific allowed motion limit.

    float rest_deg;                   // Default rest / park angle used by the application.

    int pwm_min_us;                   // Pulse width corresponding to servo_min_deg.
    int pwm_max_us;                   // Pulse width corresponding to servo_max_deg.

    float max_speed_degps;            // Real physical maximum speed of the actuator or joint in deg/s.

    uint8_t default_speed_pct;        // Default motion speed as % of max_speed_degps.
    uint8_t default_accel_pct;        // Default motion acceleration as % of ACCEL_MAX_DEGPS2.

    int feedback_adc_pin;             // Analog pin for feedback, or NO_FEEDBACK_PIN if absent.
    int fb_adc_at_servo_min_deg;      // ADC reading measured at servo_min_deg.
    int fb_adc_at_servo_max_deg;      // ADC reading measured at servo_max_deg.

    bool inverted;                    // If true, logical angle direction is inverted internally.
    bool fault_detection_enabled;     // If true, no-movement fault detection is enabled when feedback exists.
};

// ============================================================================
// ServoController
// ============================================================================
//
// Main single-servo controller class.
//
// Philosophy:
// - One ServoController object = one physical servo
// - The application provides static configuration via ServoConfig
// - The application periodically calls update()
// - The controller internally manages target, command, velocity, PWM output,
//   optional feedback reading, and optional fault detection
//
// Position-related concepts used internally:
//
// targetDeg:
//   Final requested angle.
//
// commandDeg:
//   Internal instantaneous reference generated by the trapezoidal profile.
//
// measuredDeg:
//   Real angle derived from analog feedback, if available.
//
// currentDeg:
//   "Official" current position exposed to the application.
//   With feedback  -> currentDeg = measuredDeg
//   Without feedback -> currentDeg = commandDeg (estimated)
//
// Important limitation:
// - Without feedback, the controller cannot know true physical position.
// - In that case it behaves in open loop with an internal estimate.
//

class ServoController {
public:
    // ------------------------------------------------------------------------
    // Construction
    // ------------------------------------------------------------------------
    //
    // Creates an empty controller object in a safe default state.
    // Real initialization happens later in begin().
    //
    ServoController();

    // ------------------------------------------------------------------------
    // Initialization and synchronization
    // ------------------------------------------------------------------------

    // begin(config)
    //
    // Stores the provided configuration, attaches the Servo object to the PWM
    // pin, initializes internal runtime variables, and leaves the object ready
    // for later synchronization.
    //
    // After begin(), the object is configured but not necessarily synchronized
    // to the real physical position of the servo. For that reason, the state
    // typically remains UNINITIALIZED until syncToFeedback() or syncToAngle()
    // is called.
    //
    void begin(const ServoConfig& config);

    // syncToFeedback()
    //
    // Synchronizes the internal state to the real physical position read from
    // analog feedback.
    //
    // Returns:
    //   true  -> synchronization succeeded
    //   false -> no feedback exists, or the object is not ready
    //
    // Typical use:
    //   after powering a subsystem that contains servos with feedback, to avoid
    //   any sudden jump at startup.
    //
    bool syncToFeedback();

    // syncToAngle(angleDeg)
    //
    // Synchronizes the internal state to a specified logical angle.
    //
    // Typical use:
    //   - servos without feedback
    //   - initialization to rest position
    //   - controlled reinitialization
    //
    // This sets target, command, current position, and velocity consistently.
    //
    void syncToAngle(float angleDeg);

    // ------------------------------------------------------------------------
    // Motion commands
    // ------------------------------------------------------------------------

    // setTarget(angleDeg)
    //
    // Sets a new logical target angle using the default speed and acceleration
    // percentages defined in ServoConfig.
    //
    // The angle is clamped to the allowed application range.
    //
    void setTarget(float angleDeg);

    // setTarget(angleDeg, speedPct, accelPct)
    //
    // Sets a new logical target angle while explicitly overriding motion speed
    // and acceleration percentages for the current move.
    //
    // The given percentages are clamped internally to [1..100].
    // This does NOT modify the persistent defaults stored in ServoConfig.
    //
    void setTarget(float angleDeg, uint8_t speedPct, uint8_t accelPct);

    // moveToRest()
    //
    // Convenience method that commands the servo to its configured rest angle.
    //
    void moveToRest();

    // stop()
    //
    // Gracefully stops the profile by turning the current position into the
    // active target and forcing profile velocity to zero.
    //
    void stop();

    // ------------------------------------------------------------------------
    // Fault handling
    // ------------------------------------------------------------------------

    // resetFault()
    //
    // Clears a latched fault condition and returns the controller to an
    // operational state.
    //
    // Important:
    // - this does NOT power-cycle the servo
    // - this does NOT automatically re-synchronize to feedback
    // The application sketch remains responsible for higher-level recovery.
    //
    void resetFault();

    // ------------------------------------------------------------------------
    // Periodic execution
    // ------------------------------------------------------------------------

    // update()
    //
    // Main runtime method. Must be called periodically from loop().
    //
    // update() is responsible for:
    // - computing dt
    // - reading feedback if available
    // - updating current position
    // - advancing the trapezoidal motion profile
    // - generating PWM
    // - writing PWM to the servo
    // - checking fault conditions if enabled
    // - updating the logical state
    //
    void update();

    // ------------------------------------------------------------------------
    // State queries
    // ------------------------------------------------------------------------

    // Returns the current logical servo state.
    ServoState getState() const;

    // Returns true if a fault has been latched.
    bool hasFault() const;

    // Returns true if the servo is currently in MOVING state.
    bool isMoving() const;

    // Returns true if current position is within target tolerance.
    bool isAtTarget() const;

    // Returns true if the servo has valid analog feedback configured.
    bool hasFeedback() const;

    // ------------------------------------------------------------------------
    // Position queries
    // ------------------------------------------------------------------------

    // Returns the final requested target angle.
    float getTargetDeg() const;

    // Returns the internal motion-profile command angle currently being sent.
    float getCommandDeg() const;

    // Returns the current official servo position:
    // - measured position if feedback exists
    // - estimated position otherwise
    float getCurrentDeg() const;

    // Returns the measured position when feedback exists.
    // If no feedback exists, returns the current estimated position.
    float getMeasuredDeg() const;

    // Returns tracking error defined as:
    //   commandDeg - currentDeg
    //
    // This is meaningful as a real tracking error when feedback exists.
    float getErrorDeg() const;

    // ------------------------------------------------------------------------
    // Telemetry queries
    // ------------------------------------------------------------------------

    // Returns current profile velocity in deg/s.
    float getVelocityDegps() const;

    // Returns the last PWM pulse width sent to the servo.
    int getPwmUs() const;

    // Returns the last ADC feedback reading, or -1 if feedback does not exist.
    int getFeedbackAdc() const;

    // ------------------------------------------------------------------------
    // Utility access
    // ------------------------------------------------------------------------

    // Returns the logical servo name from configuration.
    const char* getName() const;

    // Returns a const reference to the stored configuration.
    const ServoConfig& getConfig() const;

private:
    // ------------------------------------------------------------------------
    // Static configuration
    // ------------------------------------------------------------------------
    //
    // Internal copy of the application-provided configuration.
    //
    ServoConfig _cfg;

    // ------------------------------------------------------------------------
    // Hardware resource
    // ------------------------------------------------------------------------
    //
    // Servo library instance plus attached-state flag.
    //
    Servo _servo;
    bool _attached;

    // ------------------------------------------------------------------------
    // Motion state
    // ------------------------------------------------------------------------
    //
    // _targetDeg:
    //   final goal requested by the application
    //
    // _commandDeg:
    //   instantaneous motion-profile reference
    //
    // _currentDeg:
    //   official current position exposed by the class
    //
    // _velocityDegps:
    //   current profile velocity (may be signed internally)
    //
    // _activeSpeedPct / _activeAccelPct:
    //   active percentages for the current motion
    //
    // _activeSpeedDegps / _activeAccelDegps2:
    //   resolved physical magnitudes used directly by updateMotionProfile()
    //
    float _targetDeg;
    float _commandDeg;
    float _currentDeg;
    float _velocityDegps;

    uint8_t _activeSpeedPct;
    uint8_t _activeAccelPct;

    float _activeSpeedDegps;
    float _activeAccelDegps2;

    // ------------------------------------------------------------------------
    // Feedback state
    // ------------------------------------------------------------------------
    //
    // _hasFeedback:
    //   cached boolean derived from configuration
    //
    // _lastFeedbackAdc:
    //   last raw ADC reading used for feedback
    //
    // _measuredDeg:
    //   last real measured angle (if feedback exists)
    //
    // _prevMeasuredDeg:
    //   previous measured angle, used for no-movement fault detection
    //
    bool _hasFeedback;
    int _lastFeedbackAdc;
    float _measuredDeg;
    float _prevMeasuredDeg;

    // ------------------------------------------------------------------------
    // PWM / output state
    // ------------------------------------------------------------------------
    //
    // Last pulse width that was computed and sent to the servo.
    //
    int _lastPwmUs;

    // ------------------------------------------------------------------------
    // Timing
    // ------------------------------------------------------------------------
    //
    // _lastUpdateMs:
    //   timestamp of the last effective update()
    //
    // _faultStartMs:
    //   timestamp at which the current suspected no-movement condition began
    //
    unsigned long _lastUpdateMs;
    unsigned long _faultStartMs;

    // ------------------------------------------------------------------------
    // Logical state / safety
    // ------------------------------------------------------------------------
    //
    // _state:
    //   current logical runtime state
    //
    // _faultLatched:
    //   latched safety flag; once true, motion is blocked until resetFault()
    //
    ServoState _state;
    bool _faultLatched;

    // ------------------------------------------------------------------------
    // Internal helpers
    // ------------------------------------------------------------------------

    // Clamps an angle to the application-safe allowed range.
    float clampAngleToAllowedRange(float angleDeg) const;

    // Clamps an angle to the calibrated physical servo range.
    float clampAngleToServoRange(float angleDeg) const;

    // Clamps a percentage to [1..100].
    uint8_t clampPercent(uint8_t pct) const;

    // Converts speed percentage into a physical speed in deg/s based on the
    // configured per-servo max_speed_degps.
    float percentToSpeedDegps(uint8_t speedPct) const;

    // Converts acceleration percentage into a physical acceleration in deg/s²
    // based on the global software reference ACCEL_MAX_DEGPS2.
    float percentToAccelDegps2(uint8_t accelPct) const;

    // Converts a raw feedback ADC value into a logical servo angle.
    float adcToDeg(int adcValue) const;

    // Converts a logical servo angle into PWM microseconds.
    int degToPwmUs(float angleDeg) const;

    // Applies inversion when mapping a logical angle to a physical angle.
    float applyDirectionIfNeeded(float logicalDeg) const;

    // Reverses inversion when mapping a physical angle back to a logical angle.
    float undoDirectionIfNeeded(float physicalDeg) const;

    // Updates current position from analog feedback.
    void updateCurrentPositionFromFeedback();

    // Updates current position without feedback (estimated open-loop mode).
    void updateCurrentPositionWithoutFeedback();

    // Advances the trapezoidal motion profile using dt in seconds.
    void updateMotionProfile(float dtSeconds);

    // Evaluates no-movement fault conditions.
    void updateFaultDetection(unsigned long nowMs);

    // Returns true if fault detection should currently be evaluated.
    bool shouldCheckFault() const;

    // Returns true if current position is sufficiently close to target.
    bool hasReachedTarget() const;

    // Writes a PWM pulse to the servo if attached.
    void writeServoPwm(int pwmUs);

    // Updates logical state according to current runtime variables.
    void setStateFromRuntime();
};

#endif