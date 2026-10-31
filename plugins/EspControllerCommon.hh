// Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <gz/sim/System.hh>
#include <cmath>
#include <algorithm>

namespace gz { namespace sim { namespace systems {

// Butterworth 2nd-order low-pass filter — matches firmware lpf2p
struct Lpf2p
{
  double b0{0}, b1{0}, b2{0}, a1{0}, a2{0};
  double x1{0}, x2{0}, y1{0}, y2{0};

  void init(double sampleFreq, double cutoffFreq)
  {
    double fr  = sampleFreq / cutoffFreq;
    double ohm = std::tan(M_PI / fr);
    double c   = 1.0 + 2.0*std::cos(M_PI/4.0)*ohm + ohm*ohm;
    b0 = ohm*ohm/c; b1 = 2.0*b0; b2 = b0;
    a1 = 2.0*(ohm*ohm - 1.0)/c;
    a2 = (1.0 - 2.0*std::cos(M_PI/4.0)*ohm + ohm*ohm)/c;
    x1 = x2 = y1 = y2 = 0.0;
  }

  double apply(double s)
  {
    double y = b0*s + b1*x1 + b2*x2 - a1*y1 - a2*y2;
    x2=x1; x1=s; y2=y1; y1=y; return y;
  }

  void reset() { x1 = x2 = y1 = y2 = 0.0; }
};

// Outer attitude PID: angle (deg) -> rate setpoint (deg/s)
// Matches firmware attitudeControllerCorrectAttitudePID() for roll/pitch/yaw outer loop.
struct AttPidObject
{
  double kp{0}, ki{0}, kd{0};
  double iLimit{5000.0};
  double dt{0.002};
  double desired{0}, error{0}, prevError{0}, integ{0}, deriv{0};
  double prevMeasured{0};

  void init(double _kp, double _ki, double _kd, double _dt)
  { kp=_kp; ki=_ki; kd=_kd; dt=_dt; reset(); }

  void reset()
  { error=prevError=integ=deriv=prevMeasured=0.0; desired=0.0; }

  void resetWithMeasured(double m) { reset(); prevMeasured=m; }

  // updateError=true  → error = desired - measured
  // updateError=false → use caller-set error field (yaw wrap case)
  double update(double measured, bool updateError)
  {
    if (updateError) error = desired - measured;
    double out = kp * error;
    integ += error * dt;
    integ = std::clamp(integ, -iLimit, iLimit);
    out += ki * integ;
    deriv = (error - prevError) / dt;
    out += kd * deriv;
    prevError = error;
    prevMeasured = measured;
    return out;
  }
};

// Attitude setpoint received from EspPositionController via cmd_attitude topic
struct CmdAttitude
{
  double thrust{0}, rollDes{0}, pitchDes{0}, yawDes{0};
};

inline double sgn(double x) { return (x > 0.0) ? 1.0 : (x < 0.0) ? -1.0 : 0.0; }

inline double wrapYaw180(double e)
{
  if (e >  180.0) e -= 360.0;
  else if (e < -180.0) e += 360.0;
  return e;
}

}}} // namespace gz::sim::systems
