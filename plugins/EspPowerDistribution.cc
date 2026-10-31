// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
#include "EspPowerDistribution.hh"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

#include <algorithm>
#include <cmath>

using namespace gz;
using namespace sim;
using namespace systems;

void EspPowerDistribution::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &)
{
  model = Model(_entity);
  if (!model.Valid(_ecm))
  {
    gzerr << "EspPowerDistribution: invalid model entity\n";
    return;
  }

  std::string ns = _sdf->HasElement("robotNamespace")
                   ? _sdf->Get<std::string>("robotNamespace")
                   : model.Name(_ecm);

  node.Subscribe("/" + ns + "/control_cmd",
                 &EspPowerDistribution::OnControlCmd, this);

  motorPub = node.Advertise<gz::msgs::Actuators>("/" + ns + "/motor_command");

  if (_sdf->HasElement("max_pwm_rate"))
    max_pwm_rate_ = _sdf->Get<double>("max_pwm_rate");

  initialized = true;
  gzmsg << "EspPowerDistribution on " << ns << "\n";
}

void EspPowerDistribution::OnControlCmd(const gz::msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lk(cmdMutex);
  cmd.thrust = _msg.linear().z();
  cmd.roll   = _msg.angular().x();
  cmd.pitch  = _msg.angular().y();
  cmd.yaw    = _msg.angular().z();
}

void EspPowerDistribution::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &)
{
  if (_info.paused || !initialized) return;

  ControlCmd c;
  { std::lock_guard<std::mutex> lk(cmdMutex); c = cmd; }

  // Halve roll/pitch to preserve yaw authority (matches firmware power_distribution.c)
  double roll  = c.roll  / 2.0;
  double pitch = c.pitch / 2.0;
  double yaw   = c.yaw;
  double thrust= c.thrust;

  // X-frame mixing (plugin sign convention, same as previous EspAttitudeController)
  double motors[4] = {
    thrust - roll - pitch + yaw,   // M1 front-right
    thrust - roll + pitch - yaw,   // M2 rear-right
    thrust + roll + pitch + yaw,   // M3 rear-left
    thrust + roll - pitch - yaw    // M4 front-left
  };

  // Shift all motors up if any goes below zero (preserves torque shape)
  double minM = *std::min_element(motors, motors + 4);
  if (minM < 0.0) { for (auto &m : motors) m -= minM; }
  for (auto &m : motors) m = std::clamp(m, 0.0, 65535.0);

  if (!motors_initialized_) {
    for (int i = 0; i < 4; ++i) prev_motors_[i] = motors[i];
    motors_initialized_ = true;
  } else {
    for (int i = 0; i < 4; ++i) {
      motors[i] = std::clamp(motors[i],
                             prev_motors_[i] - max_pwm_rate_,
                             prev_motors_[i] + max_pwm_rate_);
      prev_motors_[i] = motors[i];
    }
  }

  gz::msgs::Actuators msg;
  for (double m : motors) msg.add_velocity(m);
  motorPub.Publish(msg);
}

GZ_ADD_PLUGIN(EspPowerDistribution, System, ISystemConfigure, ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(EspPowerDistribution,
    "gz::sim::systems::EspPowerDistribution")
