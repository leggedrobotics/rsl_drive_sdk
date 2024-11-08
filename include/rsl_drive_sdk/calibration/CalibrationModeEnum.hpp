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
#include <cstdint>
#include <string>


namespace rsl_drive_sdk
{
namespace calibration
{


//! Calibration mode short names.
#define RSL_DRIVE_CALIB_MODE_NAME_SHORT_FRICTION_ESTIMATION           ("FricEst")
#define RSL_DRIVE_CALIB_MODE_NAME_SHORT_GEAR_AND_JOINT_ENC_HOMING     ("GJEHoming")
#define RSL_DRIVE_CALIB_MODE_NAME_SHORT_GEAR_JOINT_ENC_OFFSET         ("GJEOffset")
#define RSL_DRIVE_CALIB_MODE_NAME_SHORT_GRAVITY_COMPENSATION          ("GravComp")
#define RSL_DRIVE_CALIB_MODE_NAME_SHORT_IMU_GYRO_DC_BIAS              ("GyroBias")
#define RSL_DRIVE_CALIB_MODE_NAME_SHORT_MOTOR_ENC_OFFSET              ("MEOffset")
#define RSL_DRIVE_CALIB_MODE_NAME_SHORT_MOTOR_ENC_PARAMS              ("MEParams")
#define RSL_DRIVE_CALIB_MODE_NAME_SHORT_NA                            ("N/A")
#define RSL_DRIVE_CALIB_MODE_NAME_SHORT_SAFE_JOINT_VEL                ("SafeJVel")
#define RSL_DRIVE_CALIB_MODE_NAME_SHORT_SPRING_STIFFNESS              ("SprStiff")
#define RSL_DRIVE_CALIB_MODE_NAME_SHORT_AKSIM_SELF_CALIB              ("AksimSelfCal")

//! Calibration mode enumerators for type safe usage.
enum class CalibrationModeEnum
{
  FrictionEstimation,
  GearAndJointEncoderHoming,
  GearJointEncoderOffset,
  GravityCompensation,
  ImuGyroscopeDcBias,
  MotorEncoderOffset,
  MotorEncoderParameters,
  NA,
  SafeJointVelocity,
  SpringStiffness,
  AksimSelfCalibMode, // For initializing the overall calibration
  AksimGearCalibState, // For reading the gear-Aksim's calibration state
  AksimJointCalibState // For reading the joint-Aksim's calibration state
};

/*!
 * Convert a calibration mode enumerator to an ID.
 * @param calibrationModeEnum Calibration mode enumerator.
 * @return Calibration mode ID.
 */
uint16_t calibrationModeEnumToId(const CalibrationModeEnum calibrationModeEnum);

/*!
 * Convert a calibration mode ID to an enumerator.
 * @param calibrationModeId Calibration mode ID.
 * @return Calibration mode enumerator.
 */
CalibrationModeEnum calibrationModeIdToEnum(const uint16_t calibrationModeId);

/*!
 * Convert a calibration mode enumerator to a human readable string (GUI, etc.).
 * @param calibrationModeEnum Calibration mode enumerator.
 * @return Human readable string.
 */
std::string calibrationModeEnumToName(const CalibrationModeEnum calibrationModeEnum);

/*!
 * Convert a human readable string (GUI, etc.) to a calibration mode enumerator.
 * @param calibrationModeName Human readable string.
 * @return Calibration mode enumerator.
 */
CalibrationModeEnum calibrationModeNameToEnum(const std::string & calibrationModeName);

std::ostream & operator<<(std::ostream & out, const CalibrationModeEnum calibrationModeEnum);


} // calibration
} // rsl_drive_sdk
