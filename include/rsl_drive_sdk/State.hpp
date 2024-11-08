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
#pragma once


// std
#include <ostream>

// any measurements

#include <chrono>

// rsl_drive_sdk
#include "rsl_drive_sdk/Statusword.hpp"


namespace rsl_drive_sdk
{

struct Imu
{
  double acceleration_x;
  double acceleration_y;
  double acceleration_z;

  double angle_velocity_x;
  double angle_velocity_y;
  double angle_velocity_z;

};


//! State of the Drive.
class State
{
protected:
  //! Time when the state was read out.
  std::chrono::high_resolution_clock::time_point stamp_;
  //! Statusword.
  Statusword statusword_ {};
  //! Current [A].
  double current_ = 0.0;
  //! Gear position [rad].
  double gearPosition_ = 0.0;
  //! Gear velocity [rad/s].
  double gearVelocity_ = 0.0;
  //! Joint position [rad].
  double jointPosition_ = 0.0;
  //! Joint velocity [rad/s].
  double jointVelocity_ = 0.0;
  //! Joint velocity [rad/s²].
  double jointAcceleration_ = 0.0;
  //! Joint torque [Nm].
  double jointTorque_ = 0.0;
  //! IMU.
  Imu imu_ {};

public:
  State();
  virtual ~State();

  const  std::chrono::high_resolution_clock::time_point & getStamp() const;
  void setStamp(const std::chrono::high_resolution_clock::time_point & stamp);

  const Statusword & getStatusword() const;
  void setStatusword(const Statusword & statusword);

  double getCurrent() const;
  void setCurrent(const double current);

  double getGearPosition() const;
  void setGearPosition(const double gearPosition);

  double getGearVelocity() const;
  void setGearVelocity(const double gearVelocity);

  double getJointPosition() const;
  void setJointPosition(const double jointPosition);

  double getJointVelocity() const;
  void setJointVelocity(const double jointVelocity);

  double getJointAcceleration() const;
  void setJointAcceleration(const double jointAcceleration);

  double getJointTorque() const;
  void setJointTorque(const double jointTorque);

  const Imu & getImu() const;
  void setImu(const Imu & imu);

  /*!
   * Check if the state is valid:
   * - Stamp is non-zero
   * - Statusword is valid
   * - None of the values is Inf or NaN
   * @return True if valid.
   */
  bool isValid() const;

  virtual std::string asString(const std::string & prefix = "") const;
};

std::ostream & operator<<(std::ostream & out, const State & state);


} // rsl_drive_sdk
