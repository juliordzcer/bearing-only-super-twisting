// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
#ifndef GZ_SIM_SYSTEMS_ESPPOWERDISTRIBUTION_HH_
#define GZ_SIM_SYSTEMS_ESPPOWERDISTRIBUTION_HH_

#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/actuators.pb.h>
#include <mutex>
#include <string>

namespace gz
{
namespace sim
{
namespace systems
{

/*
 * EspPowerDistribution — X-frame motor mixer.
 *
 * Replicates firmware power_distribution.c:
 *   roll  /= 2, pitch /= 2  (halved to preserve yaw authority)
 *   m1 = T - roll - pitch + yaw   (front-right)
 *   m2 = T - roll + pitch - yaw   (rear-right)
 *   m3 = T + roll + pitch + yaw   (rear-left)
 *   m4 = T + roll - pitch - yaw   (front-left)
 * Min-shift anti-saturation: if any motor < 0, shift all up uniformly.
 *
 * Topics
 *   sub  /{ns}/control_cmd    (gz.msgs.Twist)
 *          linear.z  = thrust  [PWM 0–65535]
 *          angular.x = roll    [raw, −32000…32000]
 *          angular.y = pitch   [raw, −32000…32000]
 *          angular.z = yaw     [raw, −32000…32000]
 *   pub  /{ns}/motor_command  (gz.msgs.Actuators)  — per-motor PWM [m1,m2,m3,m4]
 */
class EspPowerDistribution : public System,
                             public ISystemConfigure,
                             public ISystemPreUpdate
{
public:
  EspPowerDistribution()  = default;
  ~EspPowerDistribution() override = default;

  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &) override;

  void PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm) override;

private:
  void OnControlCmd(const gz::msgs::Twist &_msg);

  Model model{kNullEntity};

  struct ControlCmd
  {
    double thrust{0}, roll{0}, pitch{0}, yaw{0};
  };
  ControlCmd cmd;
  std::mutex cmdMutex;

  gz::transport::Node node;
  gz::transport::Node::Publisher motorPub;

  bool initialized{false};

  // Option C: per-motor PWM rate limiter
  double prev_motors_[4]{0, 0, 0, 0};
  double max_pwm_rate_{5000.0};
  bool motors_initialized_{false};
};

}
}
}

#endif
