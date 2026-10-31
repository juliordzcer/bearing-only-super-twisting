// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
#include "EspPositionController.hh"

#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>

#include <sstream>

using namespace gz;
using namespace sim;
using namespace systems;

static constexpr double kDeg2Rad = M_PI / 180.0;
static constexpr double kRad2Deg = 180.0 / M_PI;

void EspPositionController::initPids()
{
  double dt = 1.0 / this->controlFrequency;

  pidX.init(1.9, 0.1, 0.0, dt);
  pidY.init(1.9, 0.1, 0.0, dt);
  pidZ.init(1.6, 0.5, 0.0, dt);

  pidVX.init(25.0, 1.0, 0.0, dt);
  pidVY.init(25.0, 1.0, 0.0, dt);
  pidVZ.init(22.0, 15.0, 0.0, dt);

  double xyLim = xyVelMax * velMaxOverhead / pidX.ki;
  double zLim  = zVelMax  * velMaxOverhead / pidZ.ki;
  double rpLim = rpLimit  * rpLimitOverhead / pidVX.ki;
  double vzLim = (65535.0 / 2.0 / thrustScale) / pidVZ.ki;

  pidX.iLimit = xyLim;  pidY.iLimit = xyLim;
  pidZ.iLimit = zLim;
  pidVX.iLimit = rpLim; pidVY.iLimit = rpLim;
  pidVZ.iLimit = vzLim;
}

void EspPositionController::resetAllPids()
{
  pidX.reset(); pidY.reset(); pidZ.reset();
  pidVX.reset(); pidVY.reset(); pidVZ.reset();
}

void EspPositionController::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &)
{
  this->model = Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    gzerr << "EspPositionController: invalid model entity\n";
    return;
  }

  if (_sdf->HasElement("comLinkName"))
    this->comLinkName = _sdf->Get<std::string>("comLinkName");
  if (_sdf->HasElement("controlFrequency"))
    this->controlFrequency = _sdf->Get<double>("controlFrequency");

  auto readOpt = [&](const std::string &name, double &val) {
    if (_sdf->HasElement(name)) val = _sdf->Get<double>(name);
  };
  readOpt("rpLimit", rpLimit);
  readOpt("xyVelMax", xyVelMax);
  readOpt("zVelMax", zVelMax);
  readOpt("thrustBase", thrustBase);
  readOpt("thrustMin", thrustMin);

  this->initPids();

  auto readPid = [&](const std::string &prefix, PosPidObject &pid) {
    if (_sdf->HasElement(prefix + "_kp")) pid.kp = _sdf->Get<double>(prefix + "_kp");
    if (_sdf->HasElement(prefix + "_ki")) pid.ki = _sdf->Get<double>(prefix + "_ki");
    if (_sdf->HasElement(prefix + "_kd")) pid.kd = _sdf->Get<double>(prefix + "_kd");
  };
  readPid("posX", pidX);  readPid("posY", pidY);  readPid("posZ", pidZ);
  readPid("velX", pidVX); readPid("velY", pidVY); readPid("velZ", pidVZ);

  std::string ns;
  if (_sdf->HasElement("robotNamespace"))
    ns = _sdf->Get<std::string>("robotNamespace");
  else
    ns = this->model.Name(_ecm);

  // Setpoint subscriptions (one per firmware commander type)
  this->node.Subscribe("/" + ns + "/cmd_position",
      &EspPositionController::OnCmdPosition, this);
  this->node.Subscribe("/" + ns + "/cmd_stop",
      &EspPositionController::OnCmdStop, this);
  this->node.Subscribe("/" + ns + "/cmd_vel_world",
      &EspPositionController::OnCmdVelWorld, this);
  this->node.Subscribe("/" + ns + "/cmd_hover",
      &EspPositionController::OnCmdHover, this);
  this->node.Subscribe("/" + ns + "/cmd_althold",
      &EspPositionController::OnCmdAlthold, this);
  this->node.Subscribe("/" + ns + "/cmd_zdist",
      &EspPositionController::OnCmdZdist, this);

  this->node.Subscribe("/" + ns + "/pos_pid_gains",
      &EspPositionController::OnPidGains, this);

  this->cmdAttPub = this->node.Advertise<gz::msgs::Twist>("/" + ns + "/cmd_attitude");
  this->statePub  = this->node.Advertise<gz::msgs::Float_V>("/" + ns + "/position_state");

  std::chrono::duration<double> period{1.0 / this->controlFrequency};
  this->controlPeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

  gzmsg << "EspPositionController on " << ns
        << " @ " << this->controlFrequency << " Hz"
        << " thrustBase=" << thrustBase << "\n";
}

// ── Callbacks ─────────────────────────────────────────────────────────────────

void EspPositionController::OnCmdPosition(const gz::msgs::Pose &_msg)
{
  std::lock_guard<std::mutex> lock(this->cmdMutex);
  this->setpoint.mode = SpMode::POSITION;
  this->setpoint.x = _msg.position().x();
  this->setpoint.y = _msg.position().y();
  this->setpoint.z = _msg.position().z();
  double qw = _msg.orientation().w();
  double qz = _msg.orientation().z();
  this->setpoint.yaw = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz) * kRad2Deg;
}

void EspPositionController::OnCmdStop(const gz::msgs::Boolean &)
{
  std::lock_guard<std::mutex> lock(this->cmdMutex);
  this->setpoint.mode = SpMode::STOP;
}

// Type 8: world-frame velocity + yaw rate
// lin.x=vx_world [m/s], lin.y=vy_world [m/s], lin.z=vz_world [m/s], ang.z=yawRate [deg/s]
void EspPositionController::OnCmdVelWorld(const gz::msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->cmdMutex);
  this->setpoint.mode    = SpMode::VEL_WORLD;
  this->setpoint.vxw     = _msg.linear().x();
  this->setpoint.vyw     = _msg.linear().y();
  this->setpoint.vzw     = _msg.linear().z();
  this->setpoint.yawRate = _msg.angular().z();
}

// Type 10: body-frame xy velocity + absolute z + yaw rate
// lin.x=vx_body [m/s], lin.y=vy_body [m/s], lin.z=z_abs [m], ang.z=yawRate [deg/s]
void EspPositionController::OnCmdHover(const gz::msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->cmdMutex);
  this->setpoint.mode    = SpMode::HOVER;
  this->setpoint.vxb     = _msg.linear().x();
  this->setpoint.vyb     = _msg.linear().y();
  this->setpoint.zAbs    = _msg.linear().z();
  this->setpoint.yawRate = _msg.angular().z();
}

// Type 4: direct roll/pitch + z velocity + yaw rate
// ang.x=roll [deg], ang.y=pitch [deg], ang.z=yawRate [deg/s], lin.z=zVel [m/s]
void EspPositionController::OnCmdAlthold(const gz::msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->cmdMutex);
  this->setpoint.mode     = SpMode::ALTHOLD;
  this->setpoint.rollCmd  = _msg.angular().x();
  this->setpoint.pitchCmd = _msg.angular().y();
  this->setpoint.yawRate  = _msg.angular().z();
  this->setpoint.zVel     = _msg.linear().z();
}

// Type 9: direct roll/pitch + absolute z + yaw rate
// ang.x=roll [deg], ang.y=pitch [deg], ang.z=yawRate [deg/s], lin.z=z_abs [m]
void EspPositionController::OnCmdZdist(const gz::msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->cmdMutex);
  this->setpoint.mode     = SpMode::ZDISTANCE;
  this->setpoint.rollCmd  = _msg.angular().x();
  this->setpoint.pitchCmd = _msg.angular().y();
  this->setpoint.yawRate  = _msg.angular().z();
  this->setpoint.zAbs     = _msg.linear().z();
}

void EspPositionController::OnPidGains(const gz::msgs::Float_V &_msg)
{
  std::lock_guard<std::mutex> lock(this->gainsMutex);
  int n = _msg.data_size();
  if (n < 12)
  {
    gzwarn << "EspPositionController: pos_pid_gains needs >= 12 floats, got " << n << "\n";
    return;
  }

  pidX.kp = _msg.data(0);  pidX.ki = _msg.data(1);  pidX.kd = _msg.data(2);
  pidY.kp = _msg.data(3);  pidY.ki = _msg.data(4);  pidY.kd = _msg.data(5);
  pidZ.kp = _msg.data(6);  pidZ.ki = _msg.data(7);  pidZ.kd = _msg.data(8);

  if (n >= 18)
  {
    pidVX.kp = _msg.data(9);  pidVX.ki = _msg.data(10); pidVX.kd = _msg.data(11);
    pidVY.kp = _msg.data(12); pidVY.ki = _msg.data(13); pidVY.kd = _msg.data(14);
    pidVZ.kp = _msg.data(15); pidVZ.ki = _msg.data(16); pidVZ.kd = _msg.data(17);
  }
  else
  {
    pidVX.kp = _msg.data(9);  pidVX.ki = _msg.data(10); pidVX.kd = _msg.data(11);
    pidVY.kp = pidVX.kp;      pidVY.ki = pidVX.ki;      pidVY.kd = pidVX.kd;
  }

  if (n >= 19)
    thrustBase = _msg.data(18);

  if (pidX.ki > 1e-6)
  { double l = xyVelMax * velMaxOverhead / pidX.ki; pidX.iLimit = l; pidY.iLimit = l; }
  if (pidZ.ki > 1e-6)
    pidZ.iLimit = zVelMax * velMaxOverhead / pidZ.ki;
  if (pidVX.ki > 1e-6)
  { double l = rpLimit * rpLimitOverhead / pidVX.ki; pidVX.iLimit = l; pidVY.iLimit = l; }
  if (pidVZ.ki > 1e-6)
    pidVZ.iLimit = (65535.0 / 2.0 / thrustScale) / pidVZ.ki;

  this->gainsUpdated = true;
}

// ── PreUpdate ─────────────────────────────────────────────────────────────────

void EspPositionController::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (!this->initialized)
  {
    this->bodyEntity = this->model.LinkByName(_ecm, this->comLinkName);
    if (this->bodyEntity == kNullEntity)
    {
      gzerr << "EspPositionController: link [" << this->comLinkName << "] not found\n";
      return;
    }
    Link(this->bodyEntity).EnableVelocityChecks(_ecm, true);
    this->initialized = true;
  }

  {
    std::lock_guard<std::mutex> lock(this->gainsMutex);
    if (this->gainsUpdated)
    {
      this->resetAllPids();
      this->gainsUpdated = false;
    }
  }

  if (_info.simTime < this->lastControlTime)
  {
    this->lastControlTime = std::chrono::steady_clock::duration{0};
    this->resetAllPids();
    this->lastMode = SpMode::STOP;
  }

  if (_info.simTime - this->lastControlTime < this->controlPeriod)
    return;
  this->lastControlTime = _info.simTime;

  Link bodyLink(this->bodyEntity);
  auto poseOpt   = bodyLink.WorldPose(_ecm);
  auto linVelOpt = bodyLink.WorldLinearVelocity(_ecm);
  if (!poseOpt || !linVelOpt)
    return;

  const auto &pose = poseOpt.value();
  const auto &linVelWorld = linVelOpt.value();

  double px = pose.Pos().X();
  double py = pose.Pos().Y();
  double pz = pose.Pos().Z();
  double vx = linVelWorld.X();
  double vy = linVelWorld.Y();
  double vz = linVelWorld.Z();
  double yawActual = pose.Rot().Yaw() * kRad2Deg;

  GazeboSetpoint sp;
  { std::lock_guard<std::mutex> lock(this->cmdMutex); sp = this->setpoint; }

  const double dt = 1.0 / this->controlFrequency;

  // Reset PIDs and re-initialize integration state on mode transition
  if (sp.mode != this->lastMode)
  {
    this->resetAllPids();
    this->yawDesired   = yawActual;
    this->hoverTargetX = px;
    this->hoverTargetY = py;
    this->lastMode     = sp.mode;
  }

  double yawRad = yawActual * kDeg2Rad;
  double cosyaw = std::cos(yawRad);
  double sinyaw = std::sin(yawRad);

  // Current state in body frame (firmware convention)
  double body_x  =  px * cosyaw + py * sinyaw;
  double body_y  = -px * sinyaw + py * cosyaw;
  double body_vx =  vx * cosyaw + vy * sinyaw;
  double body_vy = -vx * sinyaw + vy * cosyaw;

  double rollOut{0}, pitchOut{0}, thrust{0};
  double yawOut{this->yawDesired};

  switch (sp.mode)
  {
    // ── Type 0: STOP ──────────────────────────────────────────────────────────
    case SpMode::STOP:
    {
      gz::msgs::Twist m;
      m.mutable_linear()->set_z(0.0);
      m.mutable_angular()->set_x(0.0);
      m.mutable_angular()->set_y(0.0);
      m.mutable_angular()->set_z(yawActual);
      this->cmdAttPub.Publish(m);
      return;
    }

    // ── Type 7: POSITION ──────────────────────────────────────────────────────
    case SpMode::POSITION:
    {
      double sx =  sp.x * cosyaw + sp.y * sinyaw;
      double sy = -sp.x * sinyaw + sp.y * cosyaw;

      pidX.desired = sx;
      double velSpX = std::clamp(pidX.update(body_x),
          -xyVelMax * velMaxOverhead, xyVelMax * velMaxOverhead);
      pidY.desired = sy;
      double velSpY = std::clamp(pidY.update(body_y),
          -xyVelMax * velMaxOverhead, xyVelMax * velMaxOverhead);
      pidZ.desired = sp.z;
      double velSpZ = std::clamp(pidZ.update(pz),
          -zVelMax * velMaxOverhead, zVelMax * velMaxOverhead);

      pidVX.desired = velSpX;
      pitchOut = std::clamp(pidVX.update(body_vx), -rpLimit, rpLimit);
      pidVY.desired = velSpY;
      rollOut = -std::clamp(pidVY.update(body_vy), -rpLimit, rpLimit);
      pidVZ.desired = velSpZ;
      double thrustRaw = std::clamp(pidVZ.update(vz), -32.767, 32.767);
      thrust = std::clamp(thrustRaw * thrustScale + thrustBase, thrustMin, 65535.0);

      yawOut = sp.yaw;
      break;
    }

    // ── Type 8: VEL_WORLD ─────────────────────────────────────────────────────
    case SpMode::VEL_WORLD:
    {
      // Bypass outer position PIDs; convert world setpoint to body frame
      pidX.reset(); pidY.reset(); pidZ.reset();

      pidVX.desired =  sp.vxw * cosyaw + sp.vyw * sinyaw;
      pitchOut = std::clamp(pidVX.update(body_vx), -rpLimit, rpLimit);
      pidVY.desired = -sp.vxw * sinyaw + sp.vyw * cosyaw;
      rollOut = -std::clamp(pidVY.update(body_vy), -rpLimit, rpLimit);
      pidVZ.desired = sp.vzw;
      double thrustRaw = std::clamp(pidVZ.update(vz), -32.767, 32.767);
      thrust = std::clamp(thrustRaw * thrustScale + thrustBase, thrustMin, 65535.0);

      this->yawDesired += sp.yawRate * dt;
      yawOut = this->yawDesired;
      break;
    }

    // ── Type 10: HOVER ────────────────────────────────────────────────────────
    case SpMode::HOVER:
    {
      // Body-frame velocity → integrate world target → outer pos PID (firmware isVelHoverMode)
      double vxw_cmd =  sp.vxb * cosyaw - sp.vyb * sinyaw;
      double vyw_cmd =  sp.vxb * sinyaw + sp.vyb * cosyaw;
      this->hoverTargetX += vxw_cmd * dt;
      this->hoverTargetY += vyw_cmd * dt;

      double sx =  this->hoverTargetX * cosyaw + this->hoverTargetY * sinyaw;
      double sy = -this->hoverTargetX * sinyaw + this->hoverTargetY * cosyaw;

      pidX.desired = sx;
      double velSpX = std::clamp(pidX.update(body_x),
          -xyVelMax * velMaxOverhead, xyVelMax * velMaxOverhead);
      pidY.desired = sy;
      double velSpY = std::clamp(pidY.update(body_y),
          -xyVelMax * velMaxOverhead, xyVelMax * velMaxOverhead);
      pidZ.desired = sp.zAbs;
      double velSpZ = std::clamp(pidZ.update(pz),
          -zVelMax * velMaxOverhead, zVelMax * velMaxOverhead);

      pidVX.desired = velSpX;
      pitchOut = std::clamp(pidVX.update(body_vx), -rpLimit, rpLimit);
      pidVY.desired = velSpY;
      rollOut = -std::clamp(pidVY.update(body_vy), -rpLimit, rpLimit);
      pidVZ.desired = velSpZ;
      double thrustRaw = std::clamp(pidVZ.update(vz), -32.767, 32.767);
      thrust = std::clamp(thrustRaw * thrustScale + thrustBase, thrustMin, 65535.0);

      this->yawDesired += sp.yawRate * dt;
      yawOut = this->yawDesired;
      break;
    }

    // ── Type 4: ALTHOLD ───────────────────────────────────────────────────────
    case SpMode::ALTHOLD:
    {
      // Direct roll/pitch; Z velocity fed straight to inner Z PID
      pidX.reset(); pidY.reset(); pidZ.reset(); pidVX.reset(); pidVY.reset();

      rollOut  = std::clamp(sp.rollCmd,  -rpLimit, rpLimit);
      pitchOut = std::clamp(sp.pitchCmd, -rpLimit, rpLimit);
      pidVZ.desired = sp.zVel;
      double thrustRaw = std::clamp(pidVZ.update(vz), -32.767, 32.767);
      thrust = std::clamp(thrustRaw * thrustScale + thrustBase, thrustMin, 65535.0);

      this->yawDesired += sp.yawRate * dt;
      yawOut = this->yawDesired;
      break;
    }

    // ── Type 9: ZDISTANCE ─────────────────────────────────────────────────────
    case SpMode::ZDISTANCE:
    {
      // Direct roll/pitch; absolute Z through outer+inner Z PIDs
      pidX.reset(); pidY.reset(); pidVX.reset(); pidVY.reset();

      rollOut  = std::clamp(sp.rollCmd,  -rpLimit, rpLimit);
      pitchOut = std::clamp(sp.pitchCmd, -rpLimit, rpLimit);
      pidZ.desired = sp.zAbs;
      double velSpZ = std::clamp(pidZ.update(pz),
          -zVelMax * velMaxOverhead, zVelMax * velMaxOverhead);
      pidVZ.desired = velSpZ;
      double thrustRaw = std::clamp(pidVZ.update(vz), -32.767, 32.767);
      thrust = std::clamp(thrustRaw * thrustScale + thrustBase, thrustMin, 65535.0);

      this->yawDesired += sp.yawRate * dt;
      yawOut = this->yawDesired;
      break;
    }
  }

  // Publish to attitude/SMC controller
  gz::msgs::Twist cmdMsg;
  cmdMsg.mutable_linear()->set_z(thrust);
  cmdMsg.mutable_angular()->set_x(rollOut);
  cmdMsg.mutable_angular()->set_y(pitchOut);
  cmdMsg.mutable_angular()->set_z(yawOut);
  this->cmdAttPub.Publish(cmdMsg);

  // Telemetry for optimizer
  double simTimeSec = std::chrono::duration<double>(_info.simTime).count();
  gz::msgs::Float_V stateMsg;
  stateMsg.add_data(static_cast<float>(simTimeSec));
  stateMsg.add_data(static_cast<float>(px));
  stateMsg.add_data(static_cast<float>(py));
  stateMsg.add_data(static_cast<float>(pz));
  stateMsg.add_data(static_cast<float>(vx));
  stateMsg.add_data(static_cast<float>(vy));
  stateMsg.add_data(static_cast<float>(vz));
  stateMsg.add_data(static_cast<float>(yawActual));
  double tgtX{0}, tgtY{0}, tgtZ{0};
  if (sp.mode == SpMode::POSITION)
  { tgtX = sp.x; tgtY = sp.y; tgtZ = sp.z; }
  else if (sp.mode == SpMode::HOVER || sp.mode == SpMode::ZDISTANCE)
  { tgtX = this->hoverTargetX; tgtY = this->hoverTargetY; tgtZ = sp.zAbs; }
  stateMsg.add_data(static_cast<float>(tgtX));
  stateMsg.add_data(static_cast<float>(tgtY));
  stateMsg.add_data(static_cast<float>(tgtZ));
  this->statePub.Publish(stateMsg);
}

GZ_ADD_PLUGIN(EspPositionController, System, ISystemConfigure, ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(EspPositionController,
    "gz::sim::systems::EspPositionController")
