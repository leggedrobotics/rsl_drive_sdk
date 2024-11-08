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
#include "rsl_drive_sdk/calibration/CalibrationModeEnum.hpp"
#include "rsl_drive_sdk/common/ObjectDictionary.hpp"


namespace rsl_drive_sdk
{
namespace calibration
{


uint16_t calibrationModeEnumToId(const CalibrationModeEnum calibrationModeEnum)
{
  if (calibrationModeEnum == CalibrationModeEnum::FrictionEstimation) {
    return OD_CALIB_MODE_ID_VAL_FRICTION_ESTIMATION;
  }
  if (calibrationModeEnum == CalibrationModeEnum::GearAndJointEncoderHoming) {
    return OD_CALIB_MODE_ID_VAL_GEAR_AND_JOINT_ENCODER_HOMING;
  }
  if (calibrationModeEnum == CalibrationModeEnum::GearJointEncoderOffset) {
    return OD_CALIB_MODE_ID_VAL_GEAR_JOINT_ENCODER_OFFSET;
  }
  if (calibrationModeEnum == CalibrationModeEnum::ImuGyroscopeDcBias) {
    return OD_CALIB_MODE_ID_VAL_IMU_GYROSCOPE_DC_BIAS;
  }
  if (calibrationModeEnum == CalibrationModeEnum::MotorEncoderOffset) {
    return OD_CALIB_MODE_ID_VAL_MOTOR_ENCODER_OFFSET;
  }
  if (calibrationModeEnum == CalibrationModeEnum::MotorEncoderParameters) {
    return OD_CALIB_MODE_ID_VAL_MOTOR_ENCODER_PARAMETERS;
  }
  if (calibrationModeEnum == CalibrationModeEnum::SpringStiffness) {
    return OD_CALIB_MODE_ID_VAL_SPRING_STIFFNESS;
  }
  if (calibrationModeEnum == CalibrationModeEnum::AksimSelfCalibMode) {
    return OD_CALIB_MODE_ID_VAL_AKSIM_SELF_CALIB;
  }
  return OD_CALIB_MODE_ID_VAL_IDLE;
}

CalibrationModeEnum calibrationModeIdToEnum(const uint16_t calibrationModeId)
{
  if (calibrationModeId == OD_CALIB_MODE_ID_VAL_FRICTION_ESTIMATION) {
    return CalibrationModeEnum::FrictionEstimation;
  }
  if (calibrationModeId == OD_CALIB_MODE_ID_VAL_GEAR_AND_JOINT_ENCODER_HOMING) {
    return CalibrationModeEnum::GearAndJointEncoderHoming;
  }
  if (calibrationModeId == OD_CALIB_MODE_ID_VAL_GEAR_JOINT_ENCODER_OFFSET) {
    return CalibrationModeEnum::GearJointEncoderOffset;
  }
  if (calibrationModeId == OD_CALIB_MODE_ID_VAL_IMU_GYROSCOPE_DC_BIAS) {
    return CalibrationModeEnum::ImuGyroscopeDcBias;
  }
  if (calibrationModeId == OD_CALIB_MODE_ID_VAL_MOTOR_ENCODER_OFFSET) {
    return CalibrationModeEnum::MotorEncoderOffset;
  }
  if (calibrationModeId == OD_CALIB_MODE_ID_VAL_MOTOR_ENCODER_PARAMETERS) {
    return CalibrationModeEnum::MotorEncoderParameters;
  }
  if (calibrationModeId == OD_CALIB_MODE_ID_VAL_SPRING_STIFFNESS) {
    return CalibrationModeEnum::SpringStiffness;
  }
  if (calibrationModeId == OD_CALIB_MODE_ID_VAL_AKSIM_SELF_CALIB) {
    return CalibrationModeEnum::AksimSelfCalibMode;
  }
  return CalibrationModeEnum::NA;
}

std::string calibrationModeEnumToName(const CalibrationModeEnum calibrationModeEnum)
{
  if (calibrationModeEnum == CalibrationModeEnum::FrictionEstimation) {
    return RSL_DRIVE_CALIB_MODE_NAME_SHORT_FRICTION_ESTIMATION;
  }
  if (calibrationModeEnum == CalibrationModeEnum::GearAndJointEncoderHoming) {
    return RSL_DRIVE_CALIB_MODE_NAME_SHORT_GEAR_AND_JOINT_ENC_HOMING;
  }
  if (calibrationModeEnum == CalibrationModeEnum::GearJointEncoderOffset) {
    return RSL_DRIVE_CALIB_MODE_NAME_SHORT_GEAR_JOINT_ENC_OFFSET;
  }
  if (calibrationModeEnum == CalibrationModeEnum::GravityCompensation) {
    return RSL_DRIVE_CALIB_MODE_NAME_SHORT_GRAVITY_COMPENSATION;
  }
  if (calibrationModeEnum == CalibrationModeEnum::ImuGyroscopeDcBias) {
    return RSL_DRIVE_CALIB_MODE_NAME_SHORT_IMU_GYRO_DC_BIAS;
  }
  if (calibrationModeEnum == CalibrationModeEnum::MotorEncoderOffset) {
    return RSL_DRIVE_CALIB_MODE_NAME_SHORT_MOTOR_ENC_OFFSET;
  }
  if (calibrationModeEnum == CalibrationModeEnum::MotorEncoderParameters) {
    return RSL_DRIVE_CALIB_MODE_NAME_SHORT_MOTOR_ENC_PARAMS;
  }
  if (calibrationModeEnum == CalibrationModeEnum::SafeJointVelocity) {
    return RSL_DRIVE_CALIB_MODE_NAME_SHORT_SAFE_JOINT_VEL;
  }
  if (calibrationModeEnum == CalibrationModeEnum::SpringStiffness) {
    return RSL_DRIVE_CALIB_MODE_NAME_SHORT_SPRING_STIFFNESS;
  }
  if (calibrationModeEnum == CalibrationModeEnum::AksimSelfCalibMode) {
    return RSL_DRIVE_CALIB_MODE_NAME_SHORT_AKSIM_SELF_CALIB;
  }
  return RSL_DRIVE_CALIB_MODE_NAME_SHORT_NA;
}

CalibrationModeEnum calibrationModeNameToEnum(const std::string & calibrationModeName)
{
  if (calibrationModeName == RSL_DRIVE_CALIB_MODE_NAME_SHORT_FRICTION_ESTIMATION) {
    return CalibrationModeEnum::FrictionEstimation;
  }
  if (calibrationModeName == RSL_DRIVE_CALIB_MODE_NAME_SHORT_GEAR_AND_JOINT_ENC_HOMING) {
    return CalibrationModeEnum::GearAndJointEncoderHoming;
  }
  if (calibrationModeName == RSL_DRIVE_CALIB_MODE_NAME_SHORT_GEAR_JOINT_ENC_OFFSET) {
    return CalibrationModeEnum::GearJointEncoderOffset;
  }
  if (calibrationModeName == RSL_DRIVE_CALIB_MODE_NAME_SHORT_GRAVITY_COMPENSATION) {
    return CalibrationModeEnum::GravityCompensation;
  }
  if (calibrationModeName == RSL_DRIVE_CALIB_MODE_NAME_SHORT_IMU_GYRO_DC_BIAS) {
    return CalibrationModeEnum::ImuGyroscopeDcBias;
  }
  if (calibrationModeName == RSL_DRIVE_CALIB_MODE_NAME_SHORT_MOTOR_ENC_OFFSET) {
    return CalibrationModeEnum::MotorEncoderOffset;
  }
  if (calibrationModeName == RSL_DRIVE_CALIB_MODE_NAME_SHORT_MOTOR_ENC_PARAMS) {
    return CalibrationModeEnum::MotorEncoderParameters;
  }
  if (calibrationModeName == RSL_DRIVE_CALIB_MODE_NAME_SHORT_SAFE_JOINT_VEL) {
    return CalibrationModeEnum::SafeJointVelocity;
  }
  if (calibrationModeName == RSL_DRIVE_CALIB_MODE_NAME_SHORT_SPRING_STIFFNESS) {
    return CalibrationModeEnum::SpringStiffness;
  }
  if (calibrationModeName == RSL_DRIVE_CALIB_MODE_NAME_SHORT_AKSIM_SELF_CALIB) {
    return CalibrationModeEnum::AksimSelfCalibMode;
  }
  return CalibrationModeEnum::NA;
}

std::ostream & operator<<(std::ostream & out, const CalibrationModeEnum calibrationModeEnum)
{
  return out << calibrationModeEnumToName(calibrationModeEnum);
}


} // calibration
} // rsl_drive_sdk
