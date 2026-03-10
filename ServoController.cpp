/*
===============================================================================
File:         ServoController.cpp v1.0.0
Author:       Alejandro Alonso Puig + GPT
Date:         2026-03-10
License:      Apache 2.0
-------------------------------------------------------------------------------
Description:

Implementation file for the generic Arduino ServoController library.

This class manages one servo instance and provides:

- trapezoidal motion profiling
- speed control as % of real physical max speed
- acceleration control as % of a global software reference
- optional analog feedback from the servo internal potentiometer
- optional no-movement fault detection
- telemetry getters for command, current position, PWM, ADC, etc.

Design philosophy:

- This class only manages one servo locally.
- With feedback:
    currentDeg = measuredDeg
- Without feedback:
    currentDeg = commandDeg

Important notes:

1) This first implementation does NOT power-cycle or detach the servo on fault.
   It only latches FAULT and stops motion generation. Higher-level action is
   expected from the application sketch.

2) Fault detection is only meaningful when real feedback exists and the config
   explicitly enables it.

===============================================================================
*/

#include "ServoController.h"

// ============================================================================
// Constructor
// ============================================================================

ServoController::ServoController()
    : _cfg{},
      _servo(),
      _attached(false),
      _targetDeg(0.0f),
      _commandDeg(0.0f),
      _currentDeg(0.0f),
      _velocityDegps(0.0f),
      _activeSpeedPct(100),
      _activeAccelPct(100),
      _activeSpeedDegps(0.0f),
      _activeAccelDegps2(0.0f),
      _hasFeedback(false),
      _lastFeedbackAdc(-1),
      _measuredDeg(0.0f),
      _prevMeasuredDeg(0.0f),
      _lastPwmUs(1500),
      _lastUpdateMs(0),
      _faultStartMs(0),
      _state(ServoState::UNINITIALIZED),
      _faultLatched(false)
{
}

// ============================================================================
// Public API
// ============================================================================

void ServoController::begin(const ServoConfig& config) {
    // Store a local copy of the configuration so the object is self-contained.
    _cfg = config;

    // Resolve whether feedback exists from the configured ADC pin.
    _hasFeedback = (_cfg.feedback_adc_pin != NO_FEEDBACK_PIN);

    // Load default motion percentages, clamped to the valid range.
    _activeSpeedPct = clampPercent(_cfg.default_speed_pct);
    _activeAccelPct = clampPercent(_cfg.default_accel_pct);

    // Convert percentages into physical magnitudes used directly by the profile.
    _activeSpeedDegps = percentToSpeedDegps(_activeSpeedPct);
    _activeAccelDegps2 = percentToAccelDegps2(_activeAccelPct);

    // Initialize logical position state to rest_deg.
    // This is only an internal starting point; the application may later
    // call syncToFeedback() or syncToAngle() explicitly.
    float initDeg = clampAngleToAllowedRange(_cfg.rest_deg);

    _targetDeg = initDeg;
    _commandDeg = initDeg;
    _currentDeg = initDeg;
    _measuredDeg = initDeg;
    _prevMeasuredDeg = initDeg;

    _velocityDegps = 0.0f;

    _lastFeedbackAdc = -1;
    _lastPwmUs = degToPwmUs(_commandDeg);

    _faultLatched = false;
    _faultStartMs = 0;
    _lastUpdateMs = millis();

    // Attach the servo using calibrated pulse width bounds.
    _servo.attach(_cfg.pwm_pin, _cfg.pwm_min_us, _cfg.pwm_max_us);
    _attached = true;

    // Send an initial PWM corresponding to the internal starting angle.
    writeServoPwm(_lastPwmUs);

    // Position is not yet guaranteed to be synchronized with reality.
    _state = ServoState::UNINITIALIZED;
}

bool ServoController::syncToFeedback() {
    // Feedback synchronization only makes sense if:
    // - the servo has feedback configured
    // - the servo object is attached
    if (!_hasFeedback || !_attached) {
        return false;
    }

    // Read real position from feedback and make it the internal truth.
    updateCurrentPositionFromFeedback();

    // Align target and command to measured current position, so no jump occurs.
    _targetDeg = clampAngleToAllowedRange(_currentDeg);
    _commandDeg = _targetDeg;
    _velocityDegps = 0.0f;

    // Reset fault timing state.
    _faultStartMs = 0;

    // Emit PWM matching the synchronized command.
    _lastPwmUs = degToPwmUs(_commandDeg);
    writeServoPwm(_lastPwmUs);

    _state = ServoState::HOLDING;
    return true;
}

void ServoController::syncToAngle(float angleDeg) {
    // Clamp requested sync angle to the allowed application range.
    float clamped = clampAngleToAllowedRange(angleDeg);

    // Force all internal position variables to the same aligned value.
    _targetDeg = clamped;
    _commandDeg = clamped;
    _currentDeg = clamped;
    _measuredDeg = clamped;
    _prevMeasuredDeg = clamped;

    _velocityDegps = 0.0f;
    _faultStartMs = 0;

    // If PWM is already attached, reflect the synchronized state physically.
    if (_attached) {
        _lastPwmUs = degToPwmUs(_commandDeg);
        writeServoPwm(_lastPwmUs);
    }

    _state = ServoState::HOLDING;
}

void ServoController::setTarget(float angleDeg) {
    // Reuse the overload using configured default percentages.
    setTarget(angleDeg, _cfg.default_speed_pct, _cfg.default_accel_pct);
}

void ServoController::setTarget(float angleDeg, uint8_t speedPct, uint8_t accelPct) {
    // Once a fault is latched, the controller should ignore new motion commands
    // until resetFault() is called explicitly.
    if (_faultLatched) {
        return;
    }

    // Clamp target to the application-safe range.
    _targetDeg = clampAngleToAllowedRange(angleDeg);

    // Store and resolve the active motion parameters for THIS move.
    _activeSpeedPct = clampPercent(speedPct);
    _activeAccelPct = clampPercent(accelPct);

    _activeSpeedDegps = percentToSpeedDegps(_activeSpeedPct);
    _activeAccelDegps2 = percentToAccelDegps2(_activeAccelPct);

    // If target is not already effectively reached, enter MOVING state.
    // We intentionally do NOT reset commandDeg or velocityDegps here, so that
    // if a new target arrives mid-motion, the profile continues smoothly.
    if (!hasReachedTarget()) {
        _state = ServoState::MOVING;
    } else {
        _state = ServoState::HOLDING;
    }
}

void ServoController::moveToRest() {
    setTarget(_cfg.rest_deg);
}

void ServoController::stop() {
    // Graceful logical stop:
    // - turn current position into the new target
    // - align command to current
    // - zero the profile velocity
    _targetDeg = clampAngleToAllowedRange(_currentDeg);
    _commandDeg = _targetDeg;
    _velocityDegps = 0.0f;

    _lastPwmUs = degToPwmUs(_commandDeg);
    writeServoPwm(_lastPwmUs);

    _state = ServoState::HOLDING;
}

void ServoController::resetFault() {
    // Clear fault latch and timing state, but do NOT automatically resync.
    // The application remains responsible for any required higher-level recovery.
    _faultLatched = false;
    _faultStartMs = 0;
    _velocityDegps = 0.0f;

    // If still attached, return to a calm operational state.
    if (_state == ServoState::FAULT) {
        _state = ServoState::IDLE;
    }
}

void ServoController::update() {
    // If not attached, there is nothing meaningful to do.
    if (!_attached) {
        return;
    }

    unsigned long nowMs = millis();
    unsigned long elapsedMs = nowMs - _lastUpdateMs;

    // Ignore excessively fast calls. This avoids tiny dt values.
    if (elapsedMs < MIN_UPDATE_INTERVAL_MS) {
        return;
    }

    _lastUpdateMs = nowMs;
    float dtSeconds = elapsedMs / 1000.0f;

    // If fault is latched, keep state updated but do not generate further motion.
    // We still refresh feedback/current position if possible for telemetry.
    if (_faultLatched) {
        if (_hasFeedback) {
            updateCurrentPositionFromFeedback();
        } else {
            updateCurrentPositionWithoutFeedback();
        }

        _state = ServoState::FAULT;
        return;
    }

    // Refresh current position before computing new profile state.
    if (_hasFeedback) {
        updateCurrentPositionFromFeedback();
    } else {
        updateCurrentPositionWithoutFeedback();
    }

    // Keep target inside the allowed range at all times.
    _targetDeg = clampAngleToAllowedRange(_targetDeg);

    // Advance internal reference using the trapezoidal motion profile.
    updateMotionProfile(dtSeconds);

    // In open-loop mode, the official current position is the commanded position.
    if (!_hasFeedback) {
        updateCurrentPositionWithoutFeedback();
    }

    // Convert the new internal command to PWM and send it.
    _lastPwmUs = degToPwmUs(_commandDeg);
    writeServoPwm(_lastPwmUs);

    // Evaluate fault conditions if feedback-based detection is enabled.
    updateFaultDetection(nowMs);

    // Derive logical runtime state from the updated variables.
    setStateFromRuntime();
}

ServoState ServoController::getState() const {
    return _state;
}

bool ServoController::hasFault() const {
    return _faultLatched;
}

bool ServoController::isMoving() const {
    return _state == ServoState::MOVING;
}

bool ServoController::isAtTarget() const {
    return hasReachedTarget();
}

bool ServoController::hasFeedback() const {
    return _hasFeedback;
}

float ServoController::getTargetDeg() const {
    return _targetDeg;
}

float ServoController::getCommandDeg() const {
    return _commandDeg;
}

float ServoController::getCurrentDeg() const {
    return _currentDeg;
}

float ServoController::getMeasuredDeg() const {
    // For convenience, if no feedback exists return current estimated position.
    return _hasFeedback ? _measuredDeg : _currentDeg;
}

float ServoController::getErrorDeg() const {
    // Defined as internal command minus official current position.
    return _commandDeg - _currentDeg;
}

float ServoController::getVelocityDegps() const {
    return _velocityDegps;
}

int ServoController::getPwmUs() const {
    return _lastPwmUs;
}

int ServoController::getFeedbackAdc() const {
    return _hasFeedback ? _lastFeedbackAdc : -1;
}

const char* ServoController::getName() const {
    return _cfg.name;
}

const ServoConfig& ServoController::getConfig() const {
    return _cfg;
}

// ============================================================================
// Internal helpers
// ============================================================================

float ServoController::clampAngleToAllowedRange(float angleDeg) const {
    if (angleDeg < _cfg.allowed_min_deg) return _cfg.allowed_min_deg;
    if (angleDeg > _cfg.allowed_max_deg) return _cfg.allowed_max_deg;
    return angleDeg;
}

float ServoController::clampAngleToServoRange(float angleDeg) const {
    if (angleDeg < _cfg.servo_min_deg) return _cfg.servo_min_deg;
    if (angleDeg > _cfg.servo_max_deg) return _cfg.servo_max_deg;
    return angleDeg;
}

uint8_t ServoController::clampPercent(uint8_t pct) const {
    // Note: uint8_t can never be < 0, so we only need to clamp the upper end
    // after enforcing the lower bound logically.
    if (pct < 1) return 1;
    if (pct > 100) return 100;
    return pct;
}

float ServoController::percentToSpeedDegps(uint8_t speedPct) const {
    float pct = clampPercent(speedPct) / 100.0f;
    float speed = _cfg.max_speed_degps * pct;

    // Keep strictly positive to avoid degenerate cases.
    float minSpeed = _cfg.max_speed_degps * 0.01f;
    if (speed < minSpeed) {
        speed = minSpeed;
    }

    return speed;
}

float ServoController::percentToAccelDegps2(uint8_t accelPct) const {
    float pct = clampPercent(accelPct) / 100.0f;
    float accel = ACCEL_MAX_DEGPS2 * pct;

    // Keep strictly positive to avoid degenerate stop-profile math.
    float minAccel = ACCEL_MAX_DEGPS2 * 0.01f;
    if (accel < minAccel) {
        accel = minAccel;
    }

    return accel;
}

float ServoController::adcToDeg(int adcValue) const {
    // Convert a raw ADC reading into a physical angle using two-point
    // calibration between:
    // - fb_adc_at_servo_min_deg -> servo_min_deg
    // - fb_adc_at_servo_max_deg -> servo_max_deg
    long adcMin = _cfg.fb_adc_at_servo_min_deg;
    long adcMax = _cfg.fb_adc_at_servo_max_deg;

    // Protect against invalid calibration.
    if (adcMax == adcMin) {
        return clampAngleToAllowedRange(_cfg.servo_min_deg);
    }

    float ratio = (float)(adcValue - adcMin) / (float)(adcMax - adcMin);
    float physicalDeg = _cfg.servo_min_deg + ratio * (_cfg.servo_max_deg - _cfg.servo_min_deg);

    // Keep within the calibrated physical range.
    physicalDeg = clampAngleToServoRange(physicalDeg);

    // Convert physical angle back into logical angle if direction is inverted.
    float logicalDeg = undoDirectionIfNeeded(physicalDeg);

    // Finally keep it within the servo range.
    logicalDeg = clampAngleToServoRange(logicalDeg);
    return logicalDeg;
}

int ServoController::degToPwmUs(float angleDeg) const {
    // Convert logical angle to physical angle if inversion is configured.
    float logicalDeg = clampAngleToServoRange(angleDeg);
    float physicalDeg = applyDirectionIfNeeded(logicalDeg);

    float spanDeg = _cfg.servo_max_deg - _cfg.servo_min_deg;
    if (spanDeg == 0.0f) {
        return _cfg.pwm_min_us;
    }

    float ratio = (physicalDeg - _cfg.servo_min_deg) / spanDeg;
    float pwm = _cfg.pwm_min_us + ratio * (_cfg.pwm_max_us - _cfg.pwm_min_us);

    // Clamp to the calibrated PWM range.
    if (pwm < _cfg.pwm_min_us) pwm = _cfg.pwm_min_us;
    if (pwm > _cfg.pwm_max_us) pwm = _cfg.pwm_max_us;

    return (int)(pwm + 0.5f);
}

float ServoController::applyDirectionIfNeeded(float logicalDeg) const {
    logicalDeg = clampAngleToServoRange(logicalDeg);

    if (!_cfg.inverted) {
        return logicalDeg;
    }

    // Mirror around the calibrated servo range.
    return _cfg.servo_max_deg - (logicalDeg - _cfg.servo_min_deg);
}

float ServoController::undoDirectionIfNeeded(float physicalDeg) const {
    physicalDeg = clampAngleToServoRange(physicalDeg);

    if (!_cfg.inverted) {
        return physicalDeg;
    }

    // Reverse of applyDirectionIfNeeded().
    return _cfg.servo_max_deg - (physicalDeg - _cfg.servo_min_deg);
}

void ServoController::updateCurrentPositionFromFeedback() {
    // Save the previous measured angle for movement-detection logic.
    _prevMeasuredDeg = _measuredDeg;

    // Read raw ADC and convert to logical degrees.
    _lastFeedbackAdc = analogRead(_cfg.feedback_adc_pin);
    _measuredDeg = adcToDeg(_lastFeedbackAdc);

    // With feedback, measured position becomes the official current position.
    _currentDeg = _measuredDeg;
}

void ServoController::updateCurrentPositionWithoutFeedback() {
    // In open-loop mode, the best available estimate of current position is
    // simply the commanded reference.
    _currentDeg = _commandDeg;
}

void ServoController::updateMotionProfile(float dtSeconds) {
    // Compute signed and absolute distance from current internal command
    // to the final target.
    float dist = _targetDeg - _commandDeg;
    float dabs = fabs(dist);

    // If close enough, snap to target and stop.
    if (dabs <= TARGET_EPS_DEG) {
        _commandDeg = _targetDeg;
        _velocityDegps = 0.0f;
        return;
    }

    // Direction of motion.
    float dir = (dist >= 0.0f) ? 1.0f : -1.0f;

    // Stopping-speed constraint:
    // vStop = sqrt(2 * a * d)
    //
    // This is the maximum speed that still allows braking to zero within the
    // remaining distance dabs, assuming deceleration magnitude = active accel.
    float vStop = sqrtf(2.0f * _activeAccelDegps2 * dabs);

    // The allowed speed is limited both by the commanded max speed and by the
    // stop constraint near target.
    float vAllowed = min(_activeSpeedDegps, vStop);

    // Ramp speed magnitude upward by accel * dt.
    // Natural deceleration happens because vAllowed shrinks as target approaches.
    float vMag = fabs(_velocityDegps);
    vMag += _activeAccelDegps2 * dtSeconds;

    if (vMag > vAllowed) {
        vMag = vAllowed;
    }

    _velocityDegps = dir * vMag;

    // Integrate position.
    float delta = _velocityDegps * dtSeconds;

    // Avoid overshoot.
    if (fabs(delta) > dabs) {
        _commandDeg = _targetDeg;
        _velocityDegps = 0.0f;
    } else {
        _commandDeg += delta;
    }

    // Keep command within allowed application range.
    _commandDeg = clampAngleToAllowedRange(_commandDeg);
}

void ServoController::updateFaultDetection(unsigned long nowMs) {
    // Fault detection only makes sense if:
    // - feedback exists
    // - fault detection is enabled
    // - no fault is currently latched
    if (!shouldCheckFault()) {
        _faultStartMs = 0;
        return;
    }

    // Arm fault logic only if the servo should clearly be moving.
    float trackingError = fabs(_commandDeg - _currentDeg);
    if (trackingError < FAULT_ARM_ERROR_DEG) {
        _faultStartMs = 0;
        return;
    }

    // Compare current measured angle with previous measured angle.
    float measuredMotion = fabs(_measuredDeg - _prevMeasuredDeg);

    if (measuredMotion < FAULT_MOVE_EPS_DEG) {
        // Not enough real motion observed. Start or continue the no-motion timer.
        if (_faultStartMs == 0) {
            _faultStartMs = nowMs;
        } else if ((nowMs - _faultStartMs) >= FAULT_TIMEOUT_MS) {
            _faultLatched = true;
            _state = ServoState::FAULT;
            _velocityDegps = 0.0f;
        }
    } else {
        // Real motion detected -> clear no-motion timer.
        _faultStartMs = 0;
    }
}

bool ServoController::shouldCheckFault() const {
    return _hasFeedback && _cfg.fault_detection_enabled && !_faultLatched;
}

bool ServoController::hasReachedTarget() const {
    return fabs(_targetDeg - _currentDeg) <= TARGET_EPS_DEG;
}

void ServoController::writeServoPwm(int pwmUs) {
    if (_attached) {
        _servo.writeMicroseconds(pwmUs);
    }
}

void ServoController::setStateFromRuntime() {
    if (_faultLatched) {
        _state = ServoState::FAULT;
        return;
    }

    // If target is reached and profile velocity is effectively zero,
    // the servo is considered to be holding position.
    if (fabs(_targetDeg - _currentDeg) <= TARGET_EPS_DEG &&
        fabs(_velocityDegps) <= 0.01f) {
        _state = ServoState::HOLDING;
    } else {
        _state = ServoState::MOVING;
    }
}