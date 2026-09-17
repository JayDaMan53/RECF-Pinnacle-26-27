#include "vex.h"
#include "chainbar-pid.h"
#include <cmath>

namespace {
double clampChainbarValue(double value, double low, double high) {
  return value < low ? low : (value > high ? high : value);
}
}

ChainbarPID::ChainbarPID(const ChainbarPIDConfig& config) : config_(config) {}

void ChainbarPID::zeroAt(double knownDegrees) {
  stop();
  const double motorDegrees = knownDegrees * config_.motorDegreesPerChainbarDegree;
  left_chainbar.setPosition(motorDegrees, vex::degrees);
  right_chainbar.setPosition(motorDegrees, vex::degrees);
}

double ChainbarPID::position() const {
  // Both encoders must increase when raising the chainbar.
  return (left_chainbar.position(vex::degrees) +
          right_chainbar.position(vex::degrees)) /
         (2.0 * config_.motorDegreesPerChainbarDegree);
}

double ChainbarPID::target() const { return target_; }

void ChainbarPID::setTarget(double degrees) {
  target_ = clampChainbarValue(degrees, config_.minDegrees, config_.maxDegrees);
  integral_ = 0.0;
  targetElapsedSeconds_ = 0.0;
  if (config_.motorDegreesPerChainbarDegree <= 0.0) {
    stop();
    return;
  }
  previousPosition_ = position();
  enabled_ = true;
}

void ChainbarPID::move(int direction) {
  if (direction == 0) return;
  if (config_.motorDegreesPerChainbarDegree <= 0.0) {
    stop();
    return;
  }

  // A full-range temporary target creates enough error for the PID to use its
  // available voltage. Driver control calls hold() as soon as the button is
  // released, so this target does not remain latched.
  target_ = direction > 0 ? config_.maxDegrees : config_.minDegrees;
  targetElapsedSeconds_ = 0.0;
  if (!enabled_) {
    integral_ = 0.0;
    previousPosition_ = position();
    enabled_ = true;
  }
}

void ChainbarPID::hold() { setTarget(position()); }

void ChainbarPID::update(double dtSeconds) {
  if (!enabled_) return;
  if (dtSeconds <= 0.0 || !std::isfinite(dtSeconds)) return;

  const double ratio = config_.motorDegreesPerChainbarDegree;
  const double leftPosition = left_chainbar.position(vex::degrees) / ratio;
  const double rightPosition = right_chainbar.position(vex::degrees) / ratio;
  const double current = (leftPosition + rightPosition) / 2.0;
  targetElapsedSeconds_ += dtSeconds;
  if (config_.targetTimeoutSeconds > 0.0 &&
      targetElapsedSeconds_ >= config_.targetTimeoutSeconds) {
    target_ = current;
    stop();
    return;
  }
  const double error = target_ - current;
  // Derivative on measurement avoids a kick when the target changes.
  const double velocity = (current - previousPosition_) / dtSeconds;
  previousPosition_ = current;
  if (error * integral_ < 0.0) integral_ = 0.0;

  double nextIntegral = 0.0;
  if (config_.kI > 0.0) {
    const double integralLimit = std::fabs(config_.maxIntegralVoltage) / config_.kI;
    nextIntegral = clampChainbarValue(integral_ + error * dtSeconds,
                         -integralLimit, integralLimit);
  }
  const double maxVoltage = clampChainbarValue(std::fabs(config_.maxVoltage), 0.0, 12.0);
  const double pd = config_.kP * error - config_.kD * velocity;
  const double candidate = pd + config_.kI * nextIntegral;
  // Do not accumulate integral while pushing farther into output saturation
  // or against a configured travel limit.
  const bool atLimit = (current <= config_.minDegrees && candidate < 0.0) ||
                       (current >= config_.maxDegrees && candidate > 0.0);
  if (!atLimit && (std::fabs(candidate) <= maxVoltage || candidate * error < 0.0)) {
    integral_ = nextIntegral;
  }
  const double output = clampChainbarValue(pd + config_.kI * integral_,
                                           -maxVoltage, maxVoltage);

  // Give each motor its own output so the average-position PID cannot hide a
  // mismatch. A leading side is slowed while the lagging side is helped.
  const double maxSyncVoltage = clampChainbarValue(
      std::fabs(config_.maxSyncVoltage), 0.0, maxVoltage);
  const double syncCorrection = clampChainbarValue(
      config_.syncKP * (leftPosition - rightPosition),
      -maxSyncVoltage, maxSyncVoltage);
  double leftOutput = clampChainbarValue(output - syncCorrection,
                                         -maxVoltage, maxVoltage);
  double rightOutput = clampChainbarValue(output + syncCorrection,
                                          -maxVoltage, maxVoltage);

  if ((leftPosition <= config_.minDegrees && leftOutput < 0.0) ||
      (leftPosition >= config_.maxDegrees && leftOutput > 0.0)) {
    leftOutput = 0.0;
  }
  if ((rightPosition <= config_.minDegrees && rightOutput < 0.0) ||
      (rightPosition >= config_.maxDegrees && rightOutput > 0.0)) {
    rightOutput = 0.0;
  }
  left_chainbar.spin(vex::fwd, leftOutput, vex::voltageUnits::volt);
  right_chainbar.spin(vex::fwd, rightOutput, vex::voltageUnits::volt);
}

void ChainbarPID::stop() {
  enabled_ = false;
  integral_ = 0.0;
  targetElapsedSeconds_ = 0.0;
  chainbar.stop(vex::hold);
}
