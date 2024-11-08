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
#include "rsl_drive_sdk/calibration/CalibrationState.hpp"


namespace rsl_drive_sdk
{
namespace calibration
{


bool CalibrationState::getSingleCalibrationState(
  const CalibrationModeEnum calibrationModeEnum) const
{
  switch (calibrationModeEnum) {
    case CalibrationModeEnum::MotorEncoderOffset:
      return static_cast<bool>(single_.motorEncoderOffset_);
    case CalibrationModeEnum::MotorEncoderParameters:
      return static_cast<bool>(single_.motorEncoderParameters_);
    case CalibrationModeEnum::GearJointEncoderOffset:
      return static_cast<bool>(single_.gearJointEncoderOffset_);
    case CalibrationModeEnum::GearAndJointEncoderHoming:
      return static_cast<bool>(single_.gearAndJointEncoderHoming_);
    case CalibrationModeEnum::ImuGyroscopeDcBias:
      return static_cast<bool>(single_.imuGyroscopeDcBias_);
    case CalibrationModeEnum::SpringStiffness:
      return static_cast<bool>(single_.springStiffness_);
    case CalibrationModeEnum::FrictionEstimation:
      return static_cast<bool>(single_.frictionEstimation_);
    case CalibrationModeEnum::AksimGearCalibState:
      return static_cast<bool>(single_.aksimGear_);
    case CalibrationModeEnum::AksimJointCalibState:
      return static_cast<bool>(single_.aksimJoint_);

    default:
      return false;
  }
}

/*!
 * Set the state of a single calibration.
 * @param calibrationModeEnum Calibration mode enumerator.
 * @param isCalibrated    The new state of the calibration.
 */
void CalibrationState::setSingleCalibrationState(
  const CalibrationModeEnum calibrationModeEnum,
  const bool isCalibrated)
{
  switch (calibrationModeEnum) {
    case CalibrationModeEnum::MotorEncoderOffset:
      single_.motorEncoderOffset_ = isCalibrated;
      break;
    case CalibrationModeEnum::MotorEncoderParameters:
      single_.motorEncoderParameters_ = isCalibrated;
      break;
    case CalibrationModeEnum::GearJointEncoderOffset:
      single_.gearJointEncoderOffset_ = isCalibrated;
      break;
    case CalibrationModeEnum::GearAndJointEncoderHoming:
      single_.gearAndJointEncoderHoming_ = isCalibrated;
      break;
    case CalibrationModeEnum::ImuGyroscopeDcBias:
      single_.imuGyroscopeDcBias_ = isCalibrated;
      break;
    case CalibrationModeEnum::SpringStiffness:
      single_.springStiffness_ = isCalibrated;
      break;
    case CalibrationModeEnum::FrictionEstimation:
      single_.frictionEstimation_ = isCalibrated;
      break;
    default:
      break;
  }
}


} // calibration
} // rsl_drive_sdk
