/*
 ** Copyright 2024 Robotic Systems Lab - ETH Zurich:
 ** Remo Diethelm, Christian Gehring, Samuel Bachmann, Philipp Leeman, Lennart Nachtigall, Jonas Junger, Jan Preisig,
 ** Fabian Tischhauser, Johannes Pankert
 ** Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions
 *are met:
 **
 ** 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 **
 ** 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
 *documentation and/or other materials provided with the distribution.
 **
 ** 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from
 *this software without specific prior written permission.
 **
 ** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// std
#include <cmath>
#include <sstream>

// rsl_drive_sdk
#include "rsl_drive_sdk/State.hpp"


namespace rsl_drive_sdk
{


State::State() {}

State::~State() {}

const std::chrono::high_resolution_clock::time_point & State::getStamp() const
{
  return stamp_;
}

void State::setStamp(const std::chrono::high_resolution_clock::time_point & stamp)
{
  stamp_ = stamp;
}

const Statusword & State::getStatusword() const
{
  return statusword_;
}

void State::setStatusword(const Statusword & statusword)
{
  statusword_ = statusword;
}

double State::getCurrent() const
{
  return current_;
}

void State::setCurrent(const double current)
{
  current_ = current;
}

double State::getGearPosition() const
{
  return gearPosition_;
}

void State::setGearPosition(const double gearPosition)
{
  gearPosition_ = gearPosition;
}

double State::getGearVelocity() const
{
  return gearVelocity_;
}

void State::setGearVelocity(const double gearVelocity)
{
  gearVelocity_ = gearVelocity;
}

double State::getJointPosition() const
{
  return jointPosition_;
}

void State::setJointPosition(const double jointPosition)
{
  jointPosition_ = jointPosition;
}

double State::getJointVelocity() const
{
  return jointVelocity_;
}

void State::setJointVelocity(const double jointVelocity)
{
  jointVelocity_ = jointVelocity;
}

double State::getJointAcceleration() const
{
  return jointAcceleration_;
}

void State::setJointAcceleration(const double jointAcceleration)
{
  jointAcceleration_ = jointAcceleration;
}

double State::getJointTorque() const
{
  return jointTorque_;
}

void State::setJointTorque(const double jointTorque)
{
  jointTorque_ = jointTorque;
}

const Imu & State::getImu() const
{
  return imu_;
}

void State::setImu(const Imu & imu)
{
  imu_ = imu;
}

bool State::isValid() const
{
  return (
      //!stamp_.isZero() &&
    !statusword_.isEmpty() &&
    std::isfinite(current_) &&
    std::isfinite(gearPosition_) &&
    std::isfinite(gearVelocity_) &&
    std::isfinite(jointPosition_) &&
    std::isfinite(jointVelocity_) &&
    std::isfinite(jointTorque_) &&
      //std::isfinite(!imu_.time_.isZero()) &&
    std::isfinite(imu_.acceleration_x) &&
    std::isfinite(imu_.acceleration_y)) &&
         std::isfinite(imu_.acceleration_z) &&
         std::isfinite(imu_.angle_velocity_x) &&
         std::isfinite(imu_.angle_velocity_y) &&
         std::isfinite(imu_.angle_velocity_z);
}

std::string State::asString(const std::string & prefix) const
{
  std::stringstream ss;
  //ss << prefix << "Stamp: "  << stamp_ << std::endl;
  ss << prefix << "Statusword: " << statusword_ << std::endl;
  ss << prefix << "Current: " << current_ << std::endl;
  ss << prefix << "Gear position: " << gearPosition_ << std::endl;
  ss << prefix << "Gear velocity: " << gearVelocity_ << std::endl;
  ss << prefix << "Joint position: " << jointPosition_ << std::endl;
  ss << prefix << "Joint velocity: " << jointVelocity_ << std::endl;
  ss << prefix << "Joint acceleration: " << jointAcceleration_ << std::endl;
  ss << prefix << "Joint torque: " << jointTorque_ << std::endl;
  ss << prefix << "IMU:" << std::endl;
  //ss << prefix << prefix << "Stamp: " << imu_.time_ << std::endl;
  //ss << prefix << prefix << "Linear acceleration: " << imu_.linearAcceleration_ << std::endl;
  //ss << prefix << prefix << "Angular velocity: " << imu_.angularVelocity_;
  return ss.str();
}

std::ostream & operator<<(std::ostream & out, const State & state)
{
  out << "State:" << std::endl;
  out << state.asString("  ");
  return out;
}

} // rsl_drive_sdk
