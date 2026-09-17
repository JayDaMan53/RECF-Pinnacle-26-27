#ifndef CHAINBAR_PID_H
#define CHAINBAR_PID_H

// All positions are chainbar degrees relative to a manually established zero.
// Gains use seconds: kP volts/degree, kI volts/(degree*second),
// kD volts/(degree/second). Defaults must be tuned on your robot.
struct ChainbarPIDConfig {
  double kP = 0.08;
  double kI = 0.0;
  double kD = 0.005;
  double maxVoltage = 12.0;
  double maxIntegralVoltage = 1.0;
  double syncKP = 0.20;          // Volts per degree of left/right mismatch.
  double maxSyncVoltage = 3.0;   // Maximum per-motor synchronization correction.
  double motorDegreesPerChainbarDegree = 1.0; // External gear ratio; EDIT.
  double minDegrees = -360.0;   // EDIT to match physical travel.
  double maxDegrees = 0.0;  // EDIT to match physical travel.
  double targetTimeoutSeconds = 2.0;
};

class ChainbarPID {
 public:
  explicit ChainbarPID(const ChainbarPIDConfig& config);
  // Call ONLY after physically placing the bar at a known angle.
  // This resets both motor encoders and disables the controller.
  void zeroAt(double knownDegrees = 0.0);
  // Enables control and resets PID history. Target is limited to configured
  // travel and gives up after targetTimeoutSeconds (0 disables the timeout).
  void setTarget(double degrees);
  // Moves in one direction while a driver-control button is held. Use 1 to
  // raise and -1 to lower; configured travel limits are still enforced.
  void move(int direction);
  // Captures the current position so releasing a move button stops the motion.
  void hold();
  double position() const;
  double target() const;
  // Call once each control cycle, with its duration in seconds.
  // Keep calling at the target to hold it. No background task is created.
  void update(double dtSeconds);
  // Call before taking over with manual motor commands.
  void stop();

 private:
  ChainbarPIDConfig config_;
  double target_ = 0.0;
  double integral_ = 0.0;
  double previousPosition_ = 0.0;
  double targetElapsedSeconds_ = 0.0;
  bool enabled_ = false;
};

#endif
