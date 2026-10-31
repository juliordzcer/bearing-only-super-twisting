// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
#include "EspMotorModel.hh"

#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/ParentLinkName.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>

#include <algorithm>
#include <cmath>

using namespace gz;
using namespace sim;
using namespace systems;

void EspMotorModel::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &)
{
  this->model = Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    gzerr << "EspMotorModel: invalid model entity\n";
    return;
  }

  if (_sdf->HasElement("jointName"))
    this->jointName = _sdf->Get<std::string>("jointName");
  else
  {
    gzerr << "EspMotorModel: <jointName> required\n";
    return;
  }

  if (_sdf->HasElement("linkName"))
    this->linkName = _sdf->Get<std::string>("linkName");
  else
  {
    gzerr << "EspMotorModel: <linkName> required\n";
    return;
  }

  if (_sdf->HasElement("actuator_number"))
    this->actuatorNumber = _sdf->Get<int>("actuator_number");

  if (_sdf->HasElement("turningDirection"))
  {
    auto dir = _sdf->Get<std::string>("turningDirection");
    this->turningDirection = (dir == "cw") ? -1 : 1;
  }

  if (_sdf->HasElement("motorConstant"))
    this->motorConstant = _sdf->Get<double>("motorConstant");
  if (_sdf->HasElement("momentConstant"))
    this->momentConstant = _sdf->Get<double>("momentConstant");
  if (_sdf->HasElement("maxRotVelocity"))
    this->maxRotVelocity = _sdf->Get<double>("maxRotVelocity");
  if (_sdf->HasElement("rotorDragCoefficient"))
    this->rotorDragCoefficient = _sdf->Get<double>("rotorDragCoefficient");
  if (_sdf->HasElement("rollingMomentCoefficient"))
    this->rollingMomentCoefficient = _sdf->Get<double>("rollingMomentCoefficient");
  if (_sdf->HasElement("timeConstant"))
    this->timeConstant = _sdf->Get<double>("timeConstant");
  if (_sdf->HasElement("hoverPwm"))
    this->hoverPwm = _sdf->Get<double>("hoverPwm");

  this->rotorVelocity = (this->hoverPwm / 65535.0) * this->maxRotVelocity;
  this->cmdInput = this->hoverPwm;

  std::string ns;
  if (_sdf->HasElement("robotNamespace"))
    ns = _sdf->Get<std::string>("robotNamespace");
  else
    ns = this->model.Name(_ecm);

  std::string topic = "/" + ns + "/motor_command";
  this->node.Subscribe(topic, &EspMotorModel::OnActuatorMsg, this);

  std::string enableTopic = "/" + ns + "/motor_enable";
  this->node.Subscribe(enableTopic, &EspMotorModel::OnMotorEnable, this);

  gzmsg << "EspMotorModel [" << this->jointName << "] actuator "
        << this->actuatorNumber << " on topic " << topic << "\n";
}

void EspMotorModel::OnActuatorMsg(const gz::msgs::Actuators &_msg)
{
  std::lock_guard<std::mutex> lock(this->cmdMutex);
  if (this->actuatorNumber < _msg.velocity_size())
    this->cmdInput = _msg.velocity(this->actuatorNumber);
}

void EspMotorModel::OnMotorEnable(const gz::msgs::Boolean &_msg)
{
  std::lock_guard<std::mutex> lock(this->cmdMutex);
  this->motorsEnabled = _msg.data();
  if (!this->motorsEnabled)
    this->cmdInput = 0.0;
}

void EspMotorModel::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (!this->initialized)
  {
    this->jointEntity = this->model.JointByName(_ecm, this->jointName);
    this->linkEntity = this->model.LinkByName(_ecm, this->linkName);

    if (this->jointEntity == kNullEntity || this->linkEntity == kNullEntity)
    {
      gzerr << "EspMotorModel: joint [" << this->jointName
             << "] or link [" << this->linkName << "] not found\n";
      return;
    }

    // Body link is sibling, not parent. Find via joint's parent link.
    auto jointParentComp =
        _ecm.Component<components::ParentLinkName>(this->jointEntity);
    if (jointParentComp)
    {
      this->parentLinkEntity =
          this->model.LinkByName(_ecm, jointParentComp->Data());
    }
    if (this->parentLinkEntity == kNullEntity)
    {
      // Fallback: assume "body"
      this->parentLinkEntity = this->model.LinkByName(_ecm, "body");
    }
    if (this->parentLinkEntity == kNullEntity)
    {
      gzerr << "EspMotorModel: parent link not found for joint ["
             << this->jointName << "]\n";
      return;
    }

    Link parentLink(this->parentLinkEntity);
    parentLink.EnableVelocityChecks(_ecm, true);

    this->initialized = true;
  }

  // Detect time regression (e.g. after gz_world_reset): restore hover state
  // instead of zeroing to avoid the spin-up transient that causes integral windup.
  if (_info.simTime < this->lastSimTime)
  {
    this->rotorVelocity = (this->hoverPwm / 65535.0) * this->maxRotVelocity;
    std::lock_guard<std::mutex> lock(this->cmdMutex);
    this->cmdInput = this->hoverPwm;
  }
  this->lastSimTime = _info.simTime;

  // Kill motors if disabled
  {
    std::lock_guard<std::mutex> lock(this->cmdMutex);
    if (!this->motorsEnabled)
    {
      this->cmdInput = 0.0;
      this->rotorVelocity = 0.0;
    }
  }

  const double dt = std::chrono::duration<double>(_info.dt).count();
  if (dt <= 0.0)
    return;

  double cmd;
  {
    std::lock_guard<std::mutex> lock(this->cmdMutex);
    cmd = this->cmdInput;
  }

  // PWM (0–65535) → normalized (0–1) → target angular velocity
  double normalized = std::clamp(cmd / 65535.0, 0.0, 1.0);
  double targetVelocity = normalized * this->maxRotVelocity;

  // First-order motor dynamics
  double alpha = dt / (this->timeConstant + dt);
  this->rotorVelocity += alpha * (targetVelocity - this->rotorVelocity);

  // Thrust: F = motorConstant * ω²
  double omega = this->rotorVelocity;
  double thrust = this->motorConstant * omega * omega;

  // Drag torque: opposes rotation direction
  double dragTorque = this->momentConstant * thrust *
                      static_cast<double>(this->turningDirection);

  // Rotor drag and rolling moment are modelled but not yet applied as separate wrench.
  (void)(this->rotorDragCoefficient * omega * omega);
  (void)(this->rollingMomentCoefficient * omega * omega);

  // Get link world pose to transform force to world frame
  Link link(this->linkEntity);
  auto poseOpt = link.WorldPose(_ecm);
  if (!poseOpt)
  {
    link.EnableVelocityChecks(_ecm, true);
    return;
  }

  auto &pose = poseOpt.value();
  math::Vector3d zAxis = pose.Rot().RotateVector(math::Vector3d::UnitZ);

  // Force along rotor axis (Z-up in link frame)
  math::Vector3d forceWorld = zAxis * thrust;

  // Torques in world frame
  // Drag torque around rotor axis (reaction torque on body)
  math::Vector3d torqueWorld = zAxis * (-dragTorque);

  // Apply wrench to parent link (body)
  Link parentLink(this->parentLinkEntity);

  // Thrust at offset position creates pitch/roll torque: τ += r × F
  auto bodyPoseOpt = parentLink.WorldPose(_ecm);
  if (bodyPoseOpt)
  {
    math::Vector3d r = pose.Pos() - bodyPoseOpt.value().Pos();
    torqueWorld += r.Cross(forceWorld);
  }

  parentLink.AddWorldWrench(_ecm, forceWorld, torqueWorld);

  // Spin the propeller joint visually
  Joint joint(this->jointEntity);
  double visualVel = omega * static_cast<double>(this->turningDirection);
  auto velCmd = _ecm.Component<components::JointVelocityCmd>(this->jointEntity);
  if (!velCmd)
    _ecm.CreateComponent(this->jointEntity,
        components::JointVelocityCmd({visualVel}));
  else
    velCmd->Data() = {visualVel};
}

GZ_ADD_PLUGIN(EspMotorModel, System, ISystemConfigure, ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(EspMotorModel, "gz::sim::systems::EspMotorModel")
