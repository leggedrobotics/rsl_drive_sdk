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
// rsl_drive_sdk
#include "rsl_drive_sdk/Statusword.hpp"


namespace rsl_drive_sdk
{


Statusword::Data::Data() {}

Statusword::Data::Data(const uint32_t data)
: all_(data) {}

bool Statusword::Data::operator==(const Data & other)
{
  return all_ == other.all_;
}

bool Statusword::Data::operator!=(const Data & other)
{
  return !(*this == other);
}


Statusword::Statusword() {}

Statusword::Statusword(const Statusword & statusword)
: stamp_(statusword.getStamp()),
  data_(statusword.getData()) {}

Statusword::Statusword(uint32_t data)
{
  setData(data);
}

Statusword::~Statusword() {}

Statusword & Statusword::operator=(const Statusword & statusword)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  stamp_ = statusword.getStamp();
  data_ = statusword.getData();
  return *this;
}

bool Statusword::isEmpty() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return stamp_ == TimePoint();
}

double Statusword::getAge() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const Duration age = std::chrono::system_clock::now() - stamp_;
  return age.count();
}

Statusword::TimePoint Statusword::getStamp() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return stamp_;
}

void Statusword::setData(const uint32_t data)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  stamp_ = std::chrono::system_clock::now();
  data_ = Data(data);
}

uint32_t Statusword::getData() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.all_;
}

Statusword::Data & Statusword::getDataField()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_;
}

fsm::StateEnum Statusword::getStateEnum() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return fsm::stateIdToEnum(data_.bits_.stateId_);
}

void Statusword::setStateEnum(const fsm::StateEnum stateEnum)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  stamp_ = std::chrono::system_clock::now();
  data_.bits_.stateId_ = fsm::stateEnumToId(stateEnum);
}

mode::ModeEnum Statusword::getModeEnum() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return mode::modeIdToEnum(data_.bits_.modeId_);
}

void Statusword::setModeEnum(const mode::ModeEnum modeEnum)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  stamp_ = std::chrono::system_clock::now();
  data_.bits_.modeId_ = mode::modeEnumToId(modeEnum);
}

void Statusword::getMessages(
  std::vector<std::string> & infos,
  std::vector<std::string> & warnings,
  std::vector<std::string> & errors,
  std::vector<std::string> & fatals) const
{
  Statusword previousStatusword;
  return getMessagesDiff(previousStatusword, infos, warnings, errors, fatals);
}

void Statusword::getMessagesDiff(
  Statusword & previousStatusword,
  std::vector<std::string> & infos,
  std::vector<std::string> & warnings,
  std::vector<std::string> & errors,
  std::vector<std::string> & fatals) const
{
  // Warnings.
  if (!previousStatusword.hasWarningHighTemperatureBridge() && hasWarningHighTemperatureBridge()) {
    warnings.push_back("Warning: High power stage temperature.");
  } else if (previousStatusword.hasWarningHighTemperatureBridge() &&
    !hasWarningHighTemperatureBridge())
  {
    infos.push_back("Warning disappeared: High power stage temperature.");
  }
  if (!previousStatusword.hasWarningHighTemperatureStator() && hasWarningHighTemperatureStator()) {
    warnings.push_back("Warning: High electric motor temperature.");
  } else if (previousStatusword.hasWarningHighTemperatureStator() &&
    !hasWarningHighTemperatureStator())
  {
    infos.push_back("Warning disappeared: High electric motor temperature.");
  }
  if (!previousStatusword.hasWarningHighTemperatureCpu() && hasWarningHighTemperatureCpu()) {
    warnings.push_back("Warning: High CPU temperature.");
  } else if (previousStatusword.hasWarningHighTemperatureCpu() && !hasWarningHighTemperatureCpu()) {
    infos.push_back("Warning disappeared: High CPU temperature.");
  }
  if (!previousStatusword.hasWarningEncoderOutlierMotor() && hasWarningEncoderOutlierMotor()) {
    warnings.push_back("Warning: Motor encoder outlier.");
  } else if (previousStatusword.hasWarningEncoderOutlierMotor() &&
    !hasWarningEncoderOutlierMotor())
  {
    infos.push_back("Warning disappeared: Motor encoder outlier.");
  }
  if (!previousStatusword.hasWarningEncoderOutlierGear() && hasWarningEncoderOutlierGear()) {
    warnings.push_back("Warning: Gear encoder outlier.");
  } else if (previousStatusword.hasWarningEncoderOutlierGear() && !hasWarningEncoderOutlierGear()) {
    infos.push_back("Warning disappeared: Gear encoder outlier.");
  }
  if (!previousStatusword.hasWarningEncoderOutlierJoint() && hasWarningEncoderOutlierJoint()) {
    warnings.push_back("Warning: Joint encoder outlier.");
  } else if (previousStatusword.hasWarningEncoderOutlierJoint() &&
    !hasWarningEncoderOutlierJoint())
  {
    infos.push_back("Warning disappeared: Joint encoder outlier.");
  }
  if (!previousStatusword.hasWarningIncompleteCalibration() && hasWarningIncompleteCalibration()) {
    warnings.push_back("Warning: Incomplete calibration.");
  } else if (previousStatusword.hasWarningIncompleteCalibration() &&
    !hasWarningIncompleteCalibration())
  {
    infos.push_back("Warning disappeared: Incomplete calibration.");
  }
  if (!previousStatusword.hasWarningEncoderCrcGear() && hasWarningEncoderCrcGear()) {
    warnings.push_back("Warning: Invalid gear encoder CRC.");
  } else if (previousStatusword.hasWarningEncoderCrcGear() && !hasWarningEncoderCrcGear()) {
    infos.push_back("Warning disappeared: Invalid gear encoder CRC.");
  }
  if (!previousStatusword.hasWarningEncoderCrcJoint() && hasWarningEncoderCrcJoint()) {
    warnings.push_back("Warning: Invalid joint encoder CRC.");
  } else if (previousStatusword.hasWarningEncoderCrcJoint() && !hasWarningEncoderCrcJoint()) {
    infos.push_back("Warning disappeared: Invalid joint encoder CRC.");
  }

  // Errors.
  if (!previousStatusword.hasErrorInvalidJointTorque() && hasErrorInvalidJointTorque()) {
    errors.push_back("Error: Invalid joint torque.");
  } else if (previousStatusword.hasErrorInvalidJointTorque() && !hasErrorInvalidJointTorque()) {
    infos.push_back("Error disappeared: Invalid joint torque.");
  }
  if (!previousStatusword.hasErrorPdoTimeout() && hasErrorPdoTimeout()) {
    errors.push_back("Error: PDO timeout.");
  } else if (previousStatusword.hasErrorPdoTimeout() && !hasErrorPdoTimeout()) {
    infos.push_back("Error disappeared: PDO timeout.");
  }
  if (!previousStatusword.hasErrorInvalidElecZeroOffset() && hasErrorInvalidElecZeroOffset()) {
    errors.push_back("Error: Direct Drive Elec Zero Offset Invalid.");
  } else if (previousStatusword.hasErrorInvalidElecZeroOffset() &&
    !hasErrorInvalidElecZeroOffset())
  {
    infos.push_back("Error disappeared:  Direct Drive Elec Zero Offset Invalid");
  }
  if (!previousStatusword.hasErrorMaxVelocityExceeded() && hasErrorMaxVelocityExceeded()) {
    errors.push_back("Error: Maximum Velocity Exceeded.");
  } else if (previousStatusword.hasErrorMaxVelocityExceeded() && !hasErrorMaxVelocityExceeded()) {
    infos.push_back("Error disappeared:  Maximum Velocity Exceeded.");
  }
  if (!previousStatusword.hasErrorJointPositionLimitsSoft() && hasErrorJointPositionLimitsSoft()) {
    errors.push_back("Error: Reached soft joint position limits.");
  } else if (previousStatusword.hasErrorJointPositionLimitsSoft() &&
    !hasErrorJointPositionLimitsSoft())
  {
    infos.push_back("Error disappeared: Reached soft joint position limits.");
  }

  // Fatals.
  if (!previousStatusword.hasFatalOvertemperatureBridge() && hasFatalOvertemperatureBridge()) {
    fatals.push_back("Fatal: Power stage overtemperature.");
  } else if (previousStatusword.hasFatalOvertemperatureBridge() &&
    !hasFatalOvertemperatureBridge())
  {
    infos.push_back("Fatal disappeared: Power stage overtemperature.");
  }
  if (!previousStatusword.hasFatalOvertemperatureStator() && hasFatalOvertemperatureStator()) {
    fatals.push_back("Fatal: Electric motor overtemperature.");
  } else if (previousStatusword.hasFatalOvertemperatureStator() &&
    !hasFatalOvertemperatureStator())
  {
    infos.push_back("Fatal disappeared: Electric motor overtemperature.");
  }
  if (!previousStatusword.hasFatalOvertemperatureCpu() && hasFatalOvertemperatureCpu()) {
    fatals.push_back("Fatal: CPU overtemperature.");
  } else if (previousStatusword.hasFatalOvertemperatureCpu() && !hasFatalOvertemperatureCpu()) {
    infos.push_back("Fatal disappeared: CPU overtemperature.");
  }
  if (!previousStatusword.hasFatalJointPositionLimitsHard() && hasFatalJointPositionLimitsHard()) {
    fatals.push_back("Fatal: Reached hard joint position limits.");
  } else if (previousStatusword.hasFatalJointPositionLimitsHard() &&
    !hasFatalJointPositionLimitsHard())
  {
    infos.push_back("Fatal disappeared: Reached hard joint position limits.");
  }
  if (!previousStatusword.hasFatalMotorEncoder() && hasFatalMotorEncoder()) {
    fatals.push_back("Fatal: Motor encoder not working.");
  } else if (previousStatusword.hasFatalMotorEncoder() && !hasFatalMotorEncoder()) {
    infos.push_back("Fatal disappeared: Motor encoder not working.");
  }
  if (!previousStatusword.hasFatalCurrentSensor() && hasFatalCurrentSensor()) {
    fatals.push_back("Fatal: Current sensor not working.");
  } else if (previousStatusword.hasFatalCurrentSensor() && !hasFatalCurrentSensor()) {
    infos.push_back("Fatal disappeared: Current sensor not working.");
  }
  if (!previousStatusword.hasFatalOvervoltage() && hasFatalOvervoltage()) {
    fatals.push_back("Fatal: Drive input overvoltage.");
  } else if (previousStatusword.hasFatalOvervoltage() && !hasFatalOvervoltage()) {
    infos.push_back("Fatal disappeared: Drive input overvoltage.");
  }
  if (!previousStatusword.hasFatalUndervoltage() && hasFatalUndervoltage()) {
    fatals.push_back("Fatal: Drive input undervoltage.");
  } else if (previousStatusword.hasFatalUndervoltage() && !hasFatalUndervoltage()) {
    infos.push_back("Fatal disappeared: Drive input undervoltage.");
  }
  if (!previousStatusword.hasFatalJointMotorEncoder() && hasFatalJointMotorEncoder()) {
    fatals.push_back("Fatal: Either motor or joint encoder are in a fatal condition.");
  } else if (previousStatusword.hasFatalJointMotorEncoder() && !hasFatalJointMotorEncoder()) {
    infos.push_back("Fatal disappeared: Either motor or joint encoder are in a fatal condition.");
  }
}

bool Statusword::hasWarningHighTemperatureBridge() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.warningOvertemperatureBridge_;
}

bool Statusword::hasWarningHighTemperatureStator() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.warningOvertemperatureStator_;
}

bool Statusword::hasWarningHighTemperatureCpu() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.warningOvertemperatureCpu_;
}

bool Statusword::hasWarningEncoderOutlierMotor() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.warningEncoderOutlierMotor_;
}

bool Statusword::hasWarningEncoderOutlierGear() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.warningEncoderOutlierGear_;
}

bool Statusword::hasWarningEncoderOutlierJoint() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.warningEncoderOutlierJoint_;
}

bool Statusword::hasErrorInvalidJointTorque() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.errorInvalidJointTorque_;
}

bool Statusword::hasErrorPdoTimeout() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.errorPdoTimeout_;
}

bool Statusword::hasFatalOvertemperatureBridge() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.fatalOvertemperatureBridge_;
}

bool Statusword::hasFatalOvertemperatureStator() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.fatalOvertemperatureStator_;
}

bool Statusword::hasFatalOvertemperatureCpu() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.fatalOvertemperatureCpu_;
}

bool Statusword::hasErrorInvalidElecZeroOffset() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.error_direct_drive_elec_zero_offset_invalid;
}

bool Statusword::hasErrorMaxVelocityExceeded() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.error_max_velocity_exceeded;
}

bool Statusword::hasErrorJointPositionLimitsSoft() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.errorJointPositionLimitsSoft_;
}

bool Statusword::hasFatalJointPositionLimitsHard() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.errorJointPositionLimitsHard_;
}

bool Statusword::hasWarningIncompleteCalibration() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.warningIncompleteCalibration_;
}

bool Statusword::hasWarningEncoderCrcGear() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.warningEncoderCrcGear_;
}

bool Statusword::hasWarningEncoderCrcJoint() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.warningEncoderCrcJoint_;
}

bool Statusword::hasFatalMotorEncoder() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.fatalMotorEncoder_;
}

bool Statusword::hasFatalCurrentSensor() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.fatalCurrentSensor_;
}

bool Statusword::hasFatalOvervoltage() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.fatalOvervoltage_;
}

bool Statusword::hasFatalUndervoltage() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.fatalUndervoltage_;
}

bool Statusword::hasFatalJointMotorEncoder() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.fatal_joint_motor_encoder;
}

std::ostream & operator<<(std::ostream & os, const Statusword & statusword)
{
  for (uint32_t i = 8 * sizeof(uint32_t); i > uint32_t(0); i--) {
    os << ((statusword.getData() & (uint32_t(1) << (i - uint32_t(1)))) ? "1" : "0");
  }
  return os;
}


} // rsl_drive_sdk
