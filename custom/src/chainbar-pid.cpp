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
  if (config_.motorDegreesPerChainbarDegree <= 0.0) {
    stop();
    return;
  }
  previousPosition_ = position();
  enabled_ = true;
}

void ChainbarPID::update(double dtSeconds) {
  if (!enabled_) return;
  if (dtSeconds <= 0.0 || !std::isfinite(dtSeconds)) return;

  const double current = position();
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
  double output = clampChainbarValue(pd + config_.kI * integral_, -maxVoltage, maxVoltage);
  if ((current <= config_.minDegrees && output < 0.0) ||
      (current >= config_.maxDegrees && output > 0.0)) output = 0.0;
  chainbar.spin(vex::fwd, output, vex::voltageUnits::volt);
}

void ChainbarPID::stop() {
  enabled_ = false;
  integral_ = 0.0;
  chainbar.stop(vex::hold);
}
