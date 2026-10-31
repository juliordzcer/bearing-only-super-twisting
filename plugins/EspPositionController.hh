// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
#ifndef GZ_SIM_SYSTEMS_ESPPOSITIONCONTROLLER_HH_
#define GZ_SIM_SYSTEMS_ESPPOSITIONCONTROLLER_HH_

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/float_v.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <memory>
#include <mutex>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>


namespace gz
{
namespace sim
{
namespace systems
{

// Matches firmware PidObject (position_controller.c uses no D-filter/DerOnMeas)
struct PosPidObject
{
  double kp{0}, ki{0}, kd{0};
  double iLimit{5000.0};
  double dt{0.01};

  double desired{0};
  double error{0}, prevError{0};
  double integ{0}, deriv{0};

  void init(double _kp, double _ki, double _kd, double _dt)
  {
    kp = _kp; ki = _ki; kd = _kd; dt = _dt;
    reset();
  }

  void reset()
  {
    error = prevError = integ = deriv = 0;
    desired = 0;
  }

  double update(double measured)
  {
    error = desired - measured;
    integ += error * dt;
    integ = std::clamp(integ, -iLimit, iLimit);
    deriv = (error - prevError) / dt;
    prevError = error;
    return kp * error + ki * integ + kd * deriv;
  }
};

/// Plugin replicating firmware commander + position_controller.c.
///
/// Setpoint topics (one per firmware commander type):
///   /{ns}/cmd_position  (gz.msgs.Pose)     — Type 7: abs x,y,z + yaw from quaternion
///   /{ns}/cmd_stop      (gz.msgs.Boolean)  — Type 0: kill thrust
///   /{ns}/cmd_vel_world (gz.msgs.Twist)    — Type 8: lin=vxyz_world [m/s], ang.z=yawRate [deg/s]
///   /{ns}/cmd_hover     (gz.msgs.Twist)    — Type 10: lin.xy=vxy_body, lin.z=z_abs, ang.z=yawRate
///   /{ns}/cmd_althold   (gz.msgs.Twist)    — Type 4: ang.xy=roll/pitch [deg], ang.z=yawRate, lin.z=zVel [m/s]
///   /{ns}/cmd_zdist     (gz.msgs.Twist)    — Type 9: ang.xy=roll/pitch [deg], ang.z=yawRate, lin.z=z_abs
///   /{ns}/pos_pid_gains (gz.msgs.Float_V)  — runtime PID tuning (12 or 18 floats + optional thrustBase)
///
/// Output topics:
///   /{ns}/cmd_attitude  (gz.msgs.Twist)    — to attitude/SMC controller
///   /{ns}/position_state (gz.msgs.Float_V) — telemetry for optimizer
///
class EspPositionController : public System,
                               public ISystemConfigure,
                               public ISystemPreUpdate
{
public:
  EspPositionController() = default;
  ~EspPositionController() override = default;

  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &_eventMgr) override;

  void PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm) override;

private:
  // Firmware commander type equivalents
  enum class SpMode : uint8_t { STOP, POSITION, VEL_WORLD, HOVER, ALTHOLD, ZDISTANCE };

  struct GazeboSetpoint {
    SpMode mode{SpMode::STOP};
    // Type 7 — POSITION
    double x{0}, y{0}, z{0.5}, yaw{0};      // [m,m,m,deg]
    // Type 8 — VEL_WORLD
    double vxw{0}, vyw{0}, vzw{0};           // world-frame [m/s]
    // Type 10 — HOVER (body-frame xy vel + abs z)
    double vxb{0}, vyb{0};                   // body-frame [m/s]
    double zAbs{0.5};                        // [m] (also used by ZDISTANCE)
    // Types 4,9 — direct roll/pitch
    double rollCmd{0}, pitchCmd{0};          // [deg]
    double zVel{0};                          // [m/s] (ALTHOLD only)
    // Shared yaw rate (HOVER, VEL_WORLD, ALTHOLD, ZDISTANCE)
    double yawRate{0};                       // [deg/s]
  };

  void OnCmdPosition(const gz::msgs::Pose &_msg);
  void OnCmdStop(const gz::msgs::Boolean &_msg);
  void OnCmdVelWorld(const gz::msgs::Twist &_msg);
  void OnCmdHover(const gz::msgs::Twist &_msg);
  void OnCmdAlthold(const gz::msgs::Twist &_msg);
  void OnCmdZdist(const gz::msgs::Twist &_msg);
  void OnPidGains(const gz::msgs::Float_V &_msg);
  void initPids();
  void resetAllPids();

  Model model{kNullEntity};
  Entity bodyEntity{kNullEntity};
  std::string comLinkName{"body"};

  // Position PID (outer): pos error → vel setpoint
  PosPidObject pidX, pidY, pidZ;

  // Velocity PID (inner): vel error → thrust/attitude
  PosPidObject pidVX, pidVY, pidVZ;

  // Firmware-matching defaults
  double rpLimit{20.0};
  double rpLimitOverhead{1.10};
  double xyVelMax{1.0};
  double zVelMax{5.0};
  double velMaxOverhead{1.20};
  double thrustBase{42000.0};
  double thrustMin{2000.0};
  static constexpr double thrustScale{1000.0};

  double controlFrequency{50.0};
  std::chrono::steady_clock::duration controlPeriod;
  std::chrono::steady_clock::duration lastControlTime{0};

  GazeboSetpoint setpoint;
  std::mutex cmdMutex;

  // Internal state for rate/integration modes
  double yawDesired{0};
  double hoverTargetX{0}, hoverTargetY{0};
  SpMode lastMode{SpMode::STOP};

  gz::transport::Node node;
  gz::transport::Node::Publisher cmdAttPub;
  gz::transport::Node::Publisher statePub;
  bool initialized{false};

  std::mutex gainsMutex;
  bool gainsUpdated{false};
};

}  // namespace systems
}  // namespace sim
}  // namespace gz

#endif
