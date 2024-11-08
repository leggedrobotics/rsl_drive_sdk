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
#include "rsl_drive_sdk/Command.hpp"
#include <chrono>


namespace rsl_drive_sdk
{


Command::Command() {}

Command::~Command() {}

const std::chrono::high_resolution_clock::time_point & Command::getStamp() const
{
  return stamp_;
}

void Command::setStamp(const std::chrono::high_resolution_clock::time_point & stamp)
{
  stamp_ = stamp;
}

mode::ModeEnum Command::getModeEnum() const
{
  return modeEnum_;
}

void Command::setModeEnum(const mode::ModeEnum modeEnum)
{
  modeEnum_ = modeEnum;
}

double Command::getCurrent() const
{
  return current_;
}

void Command::setCurrent(const double current)
{
  current_ = current;
}

double Command::getMotorPosition() const
{
  return motorPosition_;
}

void Command::setMotorPosition(const double motorPosition)
{
  motorPosition_ = motorPosition;
}

double Command::getMotorVelocity() const
{
  return motorVelocity_;
}

void Command::setMotorVelocity(const double motorVelocity)
{
  motorVelocity_ = motorVelocity;
}

double Command::getGearPosition() const
{
  return gearPosition_;
}

void Command::setGearPosition(const double gearPosition)
{
  gearPosition_ = gearPosition;
}

double Command::getGearVelocity() const
{
  return gearVelocity_;
}

void Command::setGearVelocity(const double gearVelocity)
{
  gearVelocity_ = gearVelocity;
}

double Command::getJointPosition() const
{
  return jointPosition_;
}

void Command::setJointPosition(const double jointPosition)
{
  jointPosition_ = jointPosition;
}

double Command::getJointVelocity() const
{
  return jointVelocity_;
}

void Command::setJointVelocity(const double jointVelocity)
{
  jointVelocity_ = jointVelocity;
}

double Command::getJointTorque() const
{
  return jointTorque_;
}

void Command::setJointTorque(const double jointTorque)
{
  jointTorque_ = jointTorque;
}

mode::PidGainsF & Command::getPidGains()
{
  return pidGains_;
}

const mode::PidGainsF & Command::getPidGains() const
{
  return pidGains_;
}

void Command::setPidGains(const mode::PidGainsF & pidGains)
{
  pidGains_ = pidGains;
}

bool Command::isValid() const
{
  return

    modeEnum_ != mode::ModeEnum::NA &&
    std::isfinite(current_) &&
    std::isfinite(motorPosition_) &&
    std::isfinite(motorVelocity_) &&
    std::isfinite(gearPosition_) &&
    std::isfinite(gearVelocity_) &&
    std::isfinite(jointPosition_) &&
    std::isfinite(jointVelocity_) &&
    std::isfinite(jointTorque_) &&
    pidGains_.isValid();
}

std::string Command::asString(const std::string & prefix) const
{
  std::stringstream ss;
  //ss << prefix << "Stamp: "  << stamp_ << std::endl;
  ss << prefix << "Mode: " << modeEnum_ << std::endl;
  ss << prefix << "Current: " << current_ << std::endl;
  ss << prefix << "Motor position: " << motorPosition_ << std::endl;
  ss << prefix << "Motor velocity: " << motorVelocity_ << std::endl;
  ss << prefix << "Gear position: " << gearPosition_ << std::endl;
  ss << prefix << "Gear velocity: " << gearVelocity_ << std::endl;
  ss << prefix << "Joint position: " << jointPosition_ << std::endl;
  ss << prefix << "Joint velocity: " << jointVelocity_ << std::endl;
  ss << prefix << "Joint torque: " << jointTorque_ << std::endl;
  ss << prefix << "PID gains: " << pidGains_;
  return ss.str();
}

std::ostream & operator<<(std::ostream & out, const Command & command)
{
  out << "Command:" << std::endl;
  out << command.asString("  ") << std::endl;
  return out;
}


} // rsl_drive_sdk
