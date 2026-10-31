// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
#include "EspAttitudeController.hh"

#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>

#include <algorithm>
#include <cmath>

using namespace gz;
using namespace sim;
using namespace systems;

static constexpr double kDeg2Rad = M_PI / 180.0;
static constexpr double kRad2Deg = 180.0 / M_PI;

void EspAttitudeController::initPids()
{
  double dt = 1.0 / this->controlFrequency;

  pidRoll.init(5.3, 2.5, 0.0, dt);
  pidRoll.iLimit = 20.0;

  pidPitch.init(5.3, 2.5, 0.0, dt);
  pidPitch.iLimit = 20.0;

  pidYaw.init(6.0, 1.0, 0.35, dt);
  pidYaw.iLimit = 360.0;

  pidRollRate.init(190.0, 440.0, 2.6, dt);
  pidRollRate.iLimit = 33.3;
  pidRollRate.derOnMeasurement = true;
  pidRollRate.enableDFilterBW(this->controlFrequency, 30.0);

  pidPitchRate.init(190.0, 440.0, 2.6, dt);
  pidPitchRate.iLimit = 33.3;
  pidPitchRate.derOnMeasurement = true;
  pidPitchRate.enableDFilterBW(this->controlFrequency, 30.0);

  pidYawRate.init(120.0, 16.7, 0.0, dt);
  pidYawRate.iLimit = 166.7;
  pidYawRate.derOnMeasurement = true;
  pidYawRate.enableDFilterBW(this->controlFrequency, 30.0);
}

void EspAttitudeController::resetAllPids()
{
  pidRoll.reset();
  pidPitch.reset();
  pidYaw.reset();
  pidRollRate.reset();
  pidRollRate.dFilter.reset();
  pidPitchRate.reset();
  pidPitchRate.dFilter.reset();
  pidYawRate.reset();
  pidYawRate.dFilter.reset();
}

void EspAttitudeController::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &)
{
  this->model = Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    gzerr << "EspAttitudeController: invalid model entity\n";
    return;
  }

  if (_sdf->HasElement("comLinkName"))
    this->comLinkName = _sdf->Get<std::string>("comLinkName");
  if (_sdf->HasElement("controlFrequency"))
    this->controlFrequency = _sdf->Get<double>("controlFrequency");
  if (_sdf->HasElement("thrustBase"))
    this->thrustBase = _sdf->Get<double>("thrustBase");

  this->initPids();

  auto readPid = [&](const std::string &prefix, PidObject &pid)
  {
    if (_sdf->HasElement(prefix + "_kp"))
      pid.kp = _sdf->Get<double>(prefix + "_kp");
    if (_sdf->HasElement(prefix + "_ki"))
      pid.ki = _sdf->Get<double>(prefix + "_ki");
    if (_sdf->HasElement(prefix + "_kd"))
      pid.kd = _sdf->Get<double>(prefix + "_kd");
    if (_sdf->HasElement(prefix + "_iLimit"))
      pid.iLimit = _sdf->Get<double>(prefix + "_iLimit");
  };

  readPid("roll", pidRoll);
  readPid("pitch", pidPitch);
  readPid("yaw", pidYaw);
  readPid("rollRate", pidRollRate);
  readPid("pitchRate", pidPitchRate);
  readPid("yawRate", pidYawRate);

  std::string ns;
  if (_sdf->HasElement("robotNamespace"))
    ns = _sdf->Get<std::string>("robotNamespace");
  else
    ns = this->model.Name(_ecm);

  std::string cmdTopic = "/" + ns + "/cmd_attitude";
  this->node.Subscribe(cmdTopic, &EspAttitudeController::OnCmdAttitude, this);

  std::string gainsTopic = "/" + ns + "/pid_gains";
  this->node.Subscribe(gainsTopic, &EspAttitudeController::OnPidGains, this);

  std::string motorTopic = "/" + ns + "/control_cmd";
  this->controlCmdPub = this->node.Advertise<gz::msgs::Twist>(motorTopic);

  std::string stateTopic = "/" + ns + "/controller_state";
  this->statePub = this->node.Advertise<gz::msgs::Float_V>(stateTopic);

  std::chrono::duration<double> period{1.0 / this->controlFrequency};
  this->controlPeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

  gzmsg << "EspAttitudeController on " << ns
        << " @ " << this->controlFrequency << " Hz\n";
}

void EspAttitudeController::OnCmdAttitude(const gz::msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->cmdMutex);
  this->cmd.thrust = _msg.linear().z();
  this->cmd.rollDes = _msg.angular().x();
  this->cmd.pitchDes = _msg.angular().y();
  this->cmd.yawDes = _msg.angular().z();
}

void EspAttitudeController::OnPidGains(const gz::msgs::Float_V &_msg)
{
  std::lock_guard<std::mutex> lock(this->gainsMutex);
  int n = _msg.data_size();
  if (n < 9)
  {
    gzwarn << "EspAttitudeController: pid_gains needs >= 9 floats, got " << n << "\n";
    return;
  }

  // Rate PID gains: [rollRate_kp, ki, kd, pitchRate_kp, ki, kd, yawRate_kp, ki, kd]
  pidRollRate.kp  = _msg.data(0);
  pidRollRate.ki  = _msg.data(1);
  pidRollRate.kd  = _msg.data(2);
  pidPitchRate.kp = _msg.data(3);
  pidPitchRate.ki = _msg.data(4);
  pidPitchRate.kd = _msg.data(5);
  pidYawRate.kp   = _msg.data(6);
  pidYawRate.ki   = _msg.data(7);
  pidYawRate.kd   = _msg.data(8);

  // Attitude PID gains (optional, indices 9-17)
  if (n >= 18)
  {
    pidRoll.kp  = _msg.data(9);
    pidRoll.ki  = _msg.data(10);
    pidRoll.kd  = _msg.data(11);
    pidPitch.kp = _msg.data(12);
    pidPitch.ki = _msg.data(13);
    pidPitch.kd = _msg.data(14);
    pidYaw.kp   = _msg.data(15);
    pidYaw.ki   = _msg.data(16);
    pidYaw.kd   = _msg.data(17);
  }

  this->gainsUpdated = true;
  gzmsg << "PID gains updated: rollRate=[" << pidRollRate.kp << ","
        << pidRollRate.ki << "," << pidRollRate.kd << "]\n";
}

void EspAttitudeController::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (!this->initialized)
  {
    this->bodyEntity = this->model.LinkByName(_ecm, this->comLinkName);
    if (this->bodyEntity == kNullEntity)
    {
      gzerr << "EspAttitudeController: link [" << this->comLinkName
             << "] not found\n";
      return;
    }
    Link bodyLink(this->bodyEntity);
    bodyLink.EnableVelocityChecks(_ecm, true);
    this->initialized = true;
  }

  // Apply pending gains update + reset PID state
  {
    std::lock_guard<std::mutex> lock(this->gainsMutex);
    if (this->gainsUpdated)
    {
      this->resetAllPids();
      this->gainsUpdated = false;
    }
  }

  // Detect time regression (e.g. after gz_world_reset) and reset tracking
  if (_info.simTime < this->lastControlTime)
  {
    this->lastControlTime = std::chrono::steady_clock::duration{0};
    this->resetAllPids();
    std::lock_guard<std::mutex> lock(this->cmdMutex);
    this->cmd.thrust = this->thrustBase;  // hover thrust avoids motor spin-up transient
    this->cmd.rollDes = 0;
    this->cmd.pitchDes = 0;
    this->cmd.yawDes = 0;
  }

  // Throttle to controlFrequency
  auto elapsed = _info.simTime - this->lastControlTime;
  if (elapsed < this->controlPeriod)
    return;
  this->lastControlTime = _info.simTime;

  Link bodyLink(this->bodyEntity);
  auto poseOpt = bodyLink.WorldPose(_ecm);
  auto angVelOpt = bodyLink.WorldAngularVelocity(_ecm);
  if (!poseOpt || !angVelOpt)
    return;

  auto &pose = poseOpt.value();
  auto &angVelWorld = angVelOpt.value();

  double rollActual = pose.Rot().Roll() * kRad2Deg;
  double pitchActual = pose.Rot().Pitch() * kRad2Deg;
  double yawActual = pose.Rot().Yaw() * kRad2Deg;

  math::Vector3d angVelBody = pose.Rot().Inverse().RotateVector(angVelWorld);
  double rollRateActual = angVelBody.X() * kRad2Deg;
  double pitchRateActual = angVelBody.Y() * kRad2Deg;
  double yawRateActual = angVelBody.Z() * kRad2Deg;

  CmdAttitude setpoint;
  {
    std::lock_guard<std::mutex> lock(this->cmdMutex);
    setpoint = this->cmd;
  }

  // === Attitude PID (outer loop) ===
  pidRoll.desired = setpoint.rollDes;
  double rollRateDesired = pidRoll.update(rollActual, true);

  pidPitch.desired = setpoint.pitchDes;
  double pitchRateDesired = pidPitch.update(pitchActual, true);

  double yawError = setpoint.yawDes - yawActual;
  if (yawError > 180.0)
    yawError -= 360.0;
  else if (yawError < -180.0)
    yawError += 360.0;
  pidYaw.error = yawError;
  double yawRateDesired = pidYaw.update(yawActual, false);

  // === Rate PID (inner loop) ===
  pidRollRate.desired = rollRateDesired;
  double rollOutput = pidRollRate.update(rollRateActual, true);

  pidPitchRate.desired = pitchRateDesired;
  double pitchOutput = pidPitchRate.update(pitchRateActual, true);

  pidYawRate.desired = yawRateDesired;
  double yawOutput = pidYawRate.update(yawRateActual, true);

  rollOutput = std::clamp(rollOutput, -32768.0, 32767.0);
  pitchOutput = std::clamp(pitchOutput, -32768.0, 32767.0);
  yawOutput = std::clamp(yawOutput, -32768.0, 32767.0);

  double thrust = setpoint.thrust;

  // Publish raw actuator signals to EspPowerDistribution for motor mixing
  gz::msgs::Twist controlCmdMsg;
  controlCmdMsg.mutable_linear()->set_z(thrust);
  controlCmdMsg.mutable_angular()->set_x(rollOutput);
  controlCmdMsg.mutable_angular()->set_y(pitchOutput);
  controlCmdMsg.mutable_angular()->set_z(yawOutput);
  this->controlCmdPub.Publish(controlCmdMsg);

  // Publish controller state for optimizer:
  // [sim_time_s, roll, pitch, yaw, rollRate, pitchRate, yawRate]
  double simTimeSec = std::chrono::duration<double>(_info.simTime).count();
  gz::msgs::Float_V stateMsg;
  stateMsg.add_data(static_cast<float>(simTimeSec));
  stateMsg.add_data(static_cast<float>(rollActual));
  stateMsg.add_data(static_cast<float>(pitchActual));
  stateMsg.add_data(static_cast<float>(yawActual));
  stateMsg.add_data(static_cast<float>(rollRateActual));
  stateMsg.add_data(static_cast<float>(pitchRateActual));
  stateMsg.add_data(static_cast<float>(yawRateActual));
  this->statePub.Publish(stateMsg);
}

GZ_ADD_PLUGIN(EspAttitudeController, System, ISystemConfigure, ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(EspAttitudeController,
    "gz::sim::systems::EspAttitudeController")
