// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
#ifndef GZ_SIM_SYSTEMS_ESPMOTORMODEL_HH_
#define GZ_SIM_SYSTEMS_ESPMOTORMODEL_HH_

#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <mutex>
#include <string>

namespace gz
{
namespace sim
{
namespace systems
{

class EspMotorModel : public System,
                      public ISystemConfigure,
                      public ISystemPreUpdate
{
public:
  EspMotorModel() = default;
  ~EspMotorModel() override = default;

  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &_eventMgr) override;

  void PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm) override;

private:
  void OnActuatorMsg(const gz::msgs::Actuators &_msg);
  void OnMotorEnable(const gz::msgs::Boolean &_msg);

  Model model{kNullEntity};
  Entity jointEntity{kNullEntity};
  Entity linkEntity{kNullEntity};
  Entity parentLinkEntity{kNullEntity};

  std::string jointName;
  std::string linkName;
  int actuatorNumber{0};
  int turningDirection{1};  // +1 CCW, -1 CW

  double motorConstant{1.4e-07};
  double momentConstant{0.005964552};
  double maxRotVelocity{2618.0};
  double rotorDragCoefficient{8.06428e-05};
  double rollingMomentCoefficient{1e-07};
  double timeConstant{0.0125};

  double rotorVelocity{0.0};
  double cmdInput{0.0};
  double hoverPwm{26300.0};
  bool motorsEnabled{false};
  std::mutex cmdMutex;
  std::chrono::steady_clock::duration lastSimTime{0};

  gz::transport::Node node;
  bool initialized{false};
};

}
}
}

#endif
