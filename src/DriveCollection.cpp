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
#include <pthread.h>

// rsl_drive_sdk
#include "rsl_drive_sdk/DriveCollection.hpp"
#include "rsl_drive_sdk/DriveCalibrator.hpp"

namespace rsl_drive_sdk
{

DriveCollection::DriveCollection(const DrivesVector & drives)
: shutdownRequested_(false),
  drives_(drives)
{
#ifndef NDEBUG
  MELO_WARN_STREAM("CMake Build Type is 'Debug'. Change to 'Release' for better performance.");
#endif
}

DriveCollection::~DriveCollection() {}

bool DriveCollection::addDrive(const DriveEthercatDevice::SharedPtr & drive)
{
  const std::string name = drive->getName();
  if (driveExists(name)) {
    MELO_ERROR_STREAM("Cannot add Drive with name '" << name << "', because it already exists.");
    return false;
  }
  drives_.push_back(drive);
  return true;
}

bool DriveCollection::driveExists(const std::string & name) const
{
  for (const auto & rsl_drive_sdk : drives_) {
    if (rsl_drive_sdk->getName() == name) {
      return true;
    }
  }
  return false;
}

DriveEthercatDevice::SharedPtr DriveCollection::getDrive(const std::string & name) const
{
  for (auto & rsl_drive_sdk : drives_) {
    if (rsl_drive_sdk->getName() == name) {
      return rsl_drive_sdk;
    }
  }
  return DriveEthercatDevice::SharedPtr();
}

DriveCollection::DrivesVector DriveCollection::getDrives() const
{
  return drives_;
}

size_t DriveCollection::getNumberOfDrives() const
{
  return drives_.size();
}

bool DriveCollection::setGoalStatesEnum(
  const fsm::StateEnum goalStateEnum,
  const bool reachStates,
  const double timeout,
  const double checkingFrequency)
{
  for (const auto & rsl_drive_sdk : drives_) {
    rsl_drive_sdk->setFSMGoalState(goalStateEnum, false, 0.0, 100.0);
  }

  if (!reachStates) {
    return true;
  }

  const double timeStep = 1.0 / checkingFrequency;
  double timeSlept = 0.0;
  while (true) {
    if (timeSlept > timeout) {
      return false;
    }
    bool goalStatesHaveBeenReached = true;
    for (const auto & rsl_drive_sdk : drives_) {
      goalStatesHaveBeenReached = goalStatesHaveBeenReached &&
        rsl_drive_sdk->goalStateHasBeenReached();
    }
    if (goalStatesHaveBeenReached) {
      return true;
    }
    thread_sleep(timeStep);
    timeSlept += timeStep;
  }
}

void DriveCollection::clearGoalStatesEnum()
{
  for (const auto & rsl_drive_sdk : drives_) {
    rsl_drive_sdk->clearGoalStateEnum();
  }
}

bool DriveCollection::allDevicesAreConnected() const
{
  for (const auto & rsl_drive_sdk : drives_) {
    if (rsl_drive_sdk->deviceIsMissing()) {
      return false;
    }
  }
  return true;
}

bool DriveCollection::allDevicesAreInTheState(const fsm::StateEnum stateEnum) const
{
  if (getNumberOfDrives() == 0) {
    return false;
  }

  for (const auto & rsl_drive_sdk : drives_) {
    if (rsl_drive_sdk->getActiveStateEnum() != stateEnum) {
      return false;
    }
  }


  return true;
}

bool DriveCollection::allDevicesAreInTheSameState(fsm::StateEnum & stateEnum) const
{
  if (getNumberOfDrives() == 0) {
    stateEnum = fsm::StateEnum::NA;
    return true;
  }

  bool stateEnumSet = false;
  for (const auto & rsl_drive_sdk : drives_) {
    if (!stateEnumSet) {
      stateEnum = rsl_drive_sdk->getActiveStateEnum();
      stateEnumSet = true;
    } else {
      if (stateEnum != rsl_drive_sdk->getActiveStateEnum()) {
        stateEnum = fsm::StateEnum::NA;
        return false;
      }
    }
  }
  return true;
}

bool DriveCollection::allDevicesAreInTheMode(const mode::ModeEnum modeEnum) const
{
  if (getNumberOfDrives() == 0) {
    return false;
  }

  for (const auto & rsl_drive_sdk : drives_) {
    if (rsl_drive_sdk->getStatusword().getModeEnum() != modeEnum) {
      return false;
    }
  }
  return true;
}

bool DriveCollection::allDevicesAreInTheSameMode(mode::ModeEnum & modeEnum) const
{
  if (getNumberOfDrives() == 0) {
    modeEnum = mode::ModeEnum::NA;
    return true;
  }

  bool modeEnumSet = false;
  for (const auto & rsl_drive_sdk : drives_) {
    if (!modeEnumSet) {
      modeEnum = rsl_drive_sdk->getStatusword().getModeEnum();
      modeEnumSet = true;
    } else {
      if (modeEnum != rsl_drive_sdk->getStatusword().getModeEnum()) {
        modeEnum = mode::ModeEnum::NA;
        return false;
      }
    }
  }
  return true;
}

bool DriveCollection::noDeviceIsInErrorState() const
{
  for (const auto & rsl_drive_sdk : drives_) {
    if (rsl_drive_sdk->deviceIsInErrorState()) {
      return false;
    }
  }

  return true;
}

bool DriveCollection::noDeviceIsInFatalState() const
{
  for (const auto & rsl_drive_sdk : drives_) {
    if (rsl_drive_sdk->deviceIsInFatalState()) {
      return false;
    }
  }

  return true;
}

bool DriveCollection::allDevicesAreWithinJointPositionLimitsSoft() const
{
  for (const auto & rsl_drive_sdk : drives_) {
    if (!rsl_drive_sdk->isWithinJointPositionLimitsSoft()) {
      return false;
    }
  }
  return true;
}

bool DriveCollection::allDevicesAreWithinJointPositionLimitsHard() const
{
  for (const auto & rsl_drive_sdk : drives_) {
    if (!rsl_drive_sdk->isWithinJointPositionLimitsHard()) {
      return false;
    }
  }
  return true;
}

bool DriveCollection::calibrate(
  const std::string & deviceName, const calibration::CalibrationModeEnum calibrationModeEnum,
  const bool gearAndJointEncoderHomingAbsolute,
  const double gearAndJointEncoderHomingNewJointPosition)
{
  auto rsl_drive_sdk = getDrive(deviceName);
  DriveCalibrator calibrator(rsl_drive_sdk);
  return calibrator.calibrate(calibrationModeEnum, gearAndJointEncoderHomingAbsolute,
      gearAndJointEncoderHomingNewJointPosition);
}

void DriveCollection::sendControlwords(const uint16_t controlwordId)
{
  for (const auto & rsl_drive_sdk : drives_) {
    rsl_drive_sdk->setControlword(controlwordId);
  }
}

void DriveCollection::stageDisables()
{
  for (const auto & rsl_drive_sdk : drives_) {
    Command command;
    command.setModeEnum(mode::ModeEnum::Disable);
    rsl_drive_sdk->setCommand(command);
  }
}

void DriveCollection::stageFreezes()
{
  for (const auto & rsl_drive_sdk : drives_) {
    Command command;
    command.setModeEnum(mode::ModeEnum::Freeze);
    rsl_drive_sdk->setCommand(command);
  }
}

void DriveCollection::stageZeroJointTorques()
{
  for (const auto & rsl_drive_sdk : drives_) {
    Command command;
    command.setModeEnum(mode::ModeEnum::JointTorque);
    rsl_drive_sdk->setCommand(command);
  }
}

void DriveCollection::setCommands(const std::vector<Command> & commands)
{
  assert(commands.size() == drives_.size());
  unsigned int i = 0;
  for (const auto & rsl_drive_sdk : drives_) {
    rsl_drive_sdk->setCommand(commands[i]);
    i++;
  }
}


} // rsl_drive_sdk
