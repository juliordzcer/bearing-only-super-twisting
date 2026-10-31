// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
#ifndef GZ_SIM_SYSTEMS_ESPATTITUDECONTROLLER_HH_
#define GZ_SIM_SYSTEMS_ESPATTITUDECONTROLLER_HH_

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/float_v.pb.h>
#include <mutex>
#include <string>
#include <cmath>

namespace gz
{
namespace sim
{
namespace systems
{

// Butterworth 2nd-order low-pass filter (matches firmware lpf2p)
struct Lpf2p
{
  double b0{0}, b1{0}, b2{0};
  double a1{0}, a2{0};
  double x1{0}, x2{0};
  double y1{0}, y2{0};

  void init(double sampleFreq, double cutoffFreq)
  {
    double fr = sampleFreq / cutoffFreq;
    double ohm = std::tan(M_PI / fr);
    double c = 1.0 + 2.0 * std::cos(M_PI / 4.0) * ohm + ohm * ohm;
    b0 = ohm * ohm / c;
    b1 = 2.0 * b0;
    b2 = b0;
    a1 = 2.0 * (ohm * ohm - 1.0) / c;
    a2 = (1.0 - 2.0 * std::cos(M_PI / 4.0) * ohm + ohm * ohm) / c;
    x1 = x2 = y1 = y2 = 0.0;
  }

  double apply(double sample)
  {
    double y = b0 * sample + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1;  x1 = sample;
    y2 = y1;  y1 = y;
    return y;
  }

  void reset() { x1 = x2 = y1 = y2 = 0.0; }
};

struct PidObject
{
  double kp{0}, ki{0}, kd{0};
  double iLimit{5000.0};
  double outputLimit{0};  // 0 = disabled
  double dt{0.002};

  double desired{0};
  double error{0}, prevError{0};
  double integ{0}, deriv{0};
  double outP{0}, outI{0}, outD{0};
  double prevMeasured{0};
  bool derOnMeasurement{false};

  bool enableDFilter{false};
  Lpf2p dFilter;

  void init(double _kp, double _ki, double _kd, double _dt)
  {
    kp = _kp; ki = _ki; kd = _kd; dt = _dt;
    reset();
  }

  void reset()
  {
    error = prevError = integ = deriv = 0;
    outP = outI = outD = 0;
    prevMeasured = 0;
  }

  void resetWithMeasured(double measured)
  {
    reset();
    prevMeasured = measured;
  }

  void enableDFilterBW(double sampleFreq, double cutoffFreq)
  {
    enableDFilter = true;
    dFilter.init(sampleFreq, cutoffFreq);
  }

  double update(double measured, bool updateError)
  {
    if (updateError)
    {
      error = desired - measured;
    }

    outP = kp * error;

    integ += error * dt;
    integ = std::clamp(integ, -iLimit, iLimit);
    outI = ki * integ;

    if (derOnMeasurement)
      deriv = -(measured - prevMeasured) / dt;
    else
      deriv = (error - prevError) / dt;

    if (enableDFilter)
      deriv = dFilter.apply(deriv);

    outD = kd * deriv;

    prevError = error;
    prevMeasured = measured;

    double output = outP + outI + outD;
    if (outputLimit > 0)
      output = std::clamp(output, -outputLimit, outputLimit);
    return output;
  }
};

class EspAttitudeController : public System,
                              public ISystemConfigure,
                              public ISystemPreUpdate
{
public:
  EspAttitudeController() = default;
  ~EspAttitudeController() override = default;

  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &_eventMgr) override;

  void PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm) override;

private:
  void OnCmdAttitude(const gz::msgs::Twist &_msg);
  void OnPidGains(const gz::msgs::Float_V &_msg);
  void initPids();
  void resetAllPids();

  Model model{kNullEntity};
  Entity bodyEntity{kNullEntity};
  std::string comLinkName{"body"};

  // Attitude PID (outer loop)
  PidObject pidRoll, pidPitch, pidYaw;

  // Rate PID (inner loop)
  PidObject pidRollRate, pidPitchRate, pidYawRate;

  double controlFrequency{500.0};
  std::chrono::steady_clock::duration controlPeriod;
  std::chrono::steady_clock::duration lastControlTime{0};

  // Setpoint
  struct CmdAttitude
  {
    double thrust{0};
    double rollDes{0};
    double pitchDes{0};
    double yawDes{0};
  };
  CmdAttitude cmd;
  std::mutex cmdMutex;

  gz::transport::Node node;
  gz::transport::Node::Publisher controlCmdPub;
  gz::transport::Node::Publisher statePub;
  bool initialized{false};

  double thrustBase{42000.0};  // matches firmware for ESP-drone (59g) — used for time-regression recovery

  // Dynamic gains update
  std::mutex gainsMutex;
  bool gainsUpdated{false};
};

}
}
}

#endif
