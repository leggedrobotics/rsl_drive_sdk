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

#include "rsl_drive_sdk/DriveCalibrator.hpp"
#include <thread>
#include <chrono>
namespace rsl_drive_sdk
{

DriveCalibrator::DriveCalibrator(DriveEthercatDevice::SharedPtr drive)
:_drive(drive)
{

}

bool DriveCalibrator::calibrate(
  const calibration::CalibrationModeEnum calibrationModeEnum,
  const bool gearAndJointEncoderHomingAbsolute,
  const double gearAndJointEncoderHomingNewJointPosition)
{
    //1. Check if drive is calibrate state
  if(_drive->getActiveStateEnum() != fsm::StateEnum::Calibrate) {
    MELO_ERROR_STREAM("Device: " << _drive->getName() << ":" << _drive->getAddress() <<
        " needs to be in calibrate state for calibration, is in: " << _drive->getActiveStateEnum());
    return false;
  }
  using cme = calibration::CalibrationModeEnum;
  switch (calibrationModeEnum) {
    case cme::GearAndJointEncoderHoming:
      {
        /* First send new joint position (it will be wrapped in the firmware if outside of [-pi, pi)).
             * There are two ways of calibrating the position offset:
             * Absolute: The new joint position will be set as the current.
             * Relative: The new joint position will be set as the new zero.
             */
        double newJointPosition = gearAndJointEncoderHomingNewJointPosition;
        if (!gearAndJointEncoderHomingAbsolute) {
          ReadingExtended reading;
          _drive->getReading(reading);
          newJointPosition += reading.getState().getJointPosition();
        }
        _drive->sendCalibrationGearAndJointEncoderHomingNewJointPosition(newJointPosition);
        [[fallthrough]];
      }
    case cme::FrictionEstimation: [[fallthrough]];
    case cme::ImuGyroscopeDcBias: [[fallthrough]];
    case cme::MotorEncoderOffset: [[fallthrough]];
    case cme::GearJointEncoderOffset: [[fallthrough]];
    case cme::MotorEncoderParameters:
      {
        MELO_INFO_STREAM("Starting calibration: " << calibrationModeEnum);
        _drive->startCalibration(calibrationModeEnum);
        bool calib_running = true;
        size_t runs = 0;
        while(calib_running) {
          _drive->calibrationIsRunning(calib_running);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          runs++;
          if(runs > 4000) { // More than 240s
            return false;
          }
        }
        return true;
      }
      break;
    default:
      return false;
  }
}

}
