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
#include "rsl_drive_sdk/mode/ModeEnum.hpp"


namespace rsl_drive_sdk
{
namespace mode
{


uint16_t modeEnumToId(const ModeEnum modeEnum)
{
  if (modeEnum == ModeEnum::Current) {
    return RSL_DRIVE_MODE_ID_CURRENT;
  }
  if (modeEnum == ModeEnum::JointPositionTorque) {
    return RSL_DRIVE_MODE_ID_JOINT_POS_TOR;
  }
  if (modeEnum == ModeEnum::Disable) {
    return RSL_DRIVE_MODE_ID_DISABLE;
  }
  if (modeEnum == ModeEnum::Freeze) {
    return RSL_DRIVE_MODE_ID_FREEZE;
  }
  if (modeEnum == ModeEnum::GearPosition) {
    return RSL_DRIVE_MODE_ID_GEAR_POS;
  }
  if (modeEnum == ModeEnum::GearVelocity) {
    return RSL_DRIVE_MODE_ID_GEAR_VEL;
  }
  if (modeEnum == ModeEnum::JointPosition) {
    return RSL_DRIVE_MODE_ID_JOINT_POS;
  }
  if (modeEnum == ModeEnum::JointPositionVelocity) {
    return RSL_DRIVE_MODE_ID_JOINT_POS_VEL;
  }
  if (modeEnum == ModeEnum::JointPositionVelocityTorque) {
    return RSL_DRIVE_MODE_ID_JOINT_POS_VEL_TOR;
  }
  if (modeEnum == ModeEnum::JointPositionVelocityTorquePidGains) {
    return RSL_DRIVE_MODE_ID_JOINT_POS_VEL_TOR_PID;
  }
  if (modeEnum == ModeEnum::JointTorque) {
    return RSL_DRIVE_MODE_ID_JOINT_TOR;
  }
  if (modeEnum == ModeEnum::JointVelocity) {
    return RSL_DRIVE_MODE_ID_JOINT_VEL;
  }
  if (modeEnum == ModeEnum::MotorPosition) {
    return RSL_DRIVE_MODE_ID_MOTOR_POS;
  }
  if (modeEnum == ModeEnum::MotorVelocity) {
    return RSL_DRIVE_MODE_ID_MOTOR_VEL;
  }
  return RSL_DRIVE_MODE_ID_NA;
}

ModeEnum modeIdToEnum(const uint16_t modeId)
{
  if (modeId == RSL_DRIVE_MODE_ID_CURRENT) {
    return ModeEnum::Current;
  }
  if (modeId == RSL_DRIVE_MODE_ID_JOINT_POS_TOR) {
    return ModeEnum::JointPositionTorque;
  }
  if (modeId == RSL_DRIVE_MODE_ID_DISABLE) {
    return ModeEnum::Disable;
  }
  if (modeId == RSL_DRIVE_MODE_ID_FREEZE) {
    return ModeEnum::Freeze;
  }
  if (modeId == RSL_DRIVE_MODE_ID_GEAR_POS) {
    return ModeEnum::GearPosition;
  }
  if (modeId == RSL_DRIVE_MODE_ID_GEAR_VEL) {
    return ModeEnum::GearVelocity;
  }
  if (modeId == RSL_DRIVE_MODE_ID_JOINT_POS) {
    return ModeEnum::JointPosition;
  }
  if (modeId == RSL_DRIVE_MODE_ID_JOINT_POS_VEL) {
    return ModeEnum::JointPositionVelocity;
  }
  if (modeId == RSL_DRIVE_MODE_ID_JOINT_POS_VEL_TOR) {
    return ModeEnum::JointPositionVelocityTorque;
  }
  if (modeId == RSL_DRIVE_MODE_ID_JOINT_POS_VEL_TOR_PID) {
    return ModeEnum::JointPositionVelocityTorquePidGains;
  }
  if (modeId == RSL_DRIVE_MODE_ID_JOINT_TOR) {
    return ModeEnum::JointTorque;
  }
  if (modeId == RSL_DRIVE_MODE_ID_JOINT_VEL) {
    return ModeEnum::JointVelocity;
  }
  if (modeId == RSL_DRIVE_MODE_ID_MOTOR_POS) {
    return ModeEnum::MotorPosition;
  }
  if (modeId == RSL_DRIVE_MODE_ID_MOTOR_VEL) {
    return ModeEnum::MotorVelocity;
  }
  return ModeEnum::NA;
}

std::string modeEnumToName(const ModeEnum modeEnum)
{
  if (modeEnum == ModeEnum::Current) {
    return RSL_DRIVE_MODE_NAME_CURRENT;
  }
  if (modeEnum == ModeEnum::JointPositionTorque) {
    return RSL_DRIVE_MODE_NAME_JOINT_POS_TOR;
  }
  if (modeEnum == ModeEnum::Disable) {
    return RSL_DRIVE_MODE_NAME_DISABLE;
  }
  if (modeEnum == ModeEnum::Freeze) {
    return RSL_DRIVE_MODE_NAME_FREEZE;
  }
  if (modeEnum == ModeEnum::GearPosition) {
    return RSL_DRIVE_MODE_NAME_GEAR_POS;
  }
  if (modeEnum == ModeEnum::GearVelocity) {
    return RSL_DRIVE_MODE_NAME_GEAR_VEL;
  }
  if (modeEnum == ModeEnum::JointPosition) {
    return RSL_DRIVE_MODE_NAME_JOINT_POS;
  }
  if (modeEnum == ModeEnum::JointPositionVelocity) {
    return RSL_DRIVE_MODE_NAME_JOINT_POS_VEL;
  }
  if (modeEnum == ModeEnum::JointPositionVelocityTorque) {
    return RSL_DRIVE_MODE_NAME_JOINT_POS_VEL_TOR;
  }
  if (modeEnum == ModeEnum::JointPositionVelocityTorquePidGains) {
    return RSL_DRIVE_MODE_NAME_JOINT_POS_VEL_TOR_PID;
  }
  if (modeEnum == ModeEnum::JointTorque) {
    return RSL_DRIVE_MODE_NAME_JOINT_TOR;
  }
  if (modeEnum == ModeEnum::JointVelocity) {
    return RSL_DRIVE_MODE_NAME_JOINT_VEL;
  }
  if (modeEnum == ModeEnum::MotorPosition) {
    return RSL_DRIVE_MODE_NAME_MOTOR_POS;
  }
  if (modeEnum == ModeEnum::MotorVelocity) {
    return RSL_DRIVE_MODE_NAME_MOTOR_VEL;
  }
  return RSL_DRIVE_MODE_NAME_NA;
}

std::string modeEnumToShortName(const ModeEnum modeEnum)
{
  if (modeEnum == ModeEnum::Current) {
    return RSL_DRIVE_MODE_NAME_SHORT_CURRENT;
  }
  if (modeEnum == ModeEnum::JointPositionTorque) {
    return RSL_DRIVE_MODE_NAME_SHORT_JOINT_POS_TOR;
  }
  if (modeEnum == ModeEnum::Disable) {
    return RSL_DRIVE_MODE_NAME_SHORT_DISABLE;
  }
  if (modeEnum == ModeEnum::Freeze) {
    return RSL_DRIVE_MODE_NAME_SHORT_FREEZE;
  }
  if (modeEnum == ModeEnum::GearPosition) {
    return RSL_DRIVE_MODE_NAME_SHORT_GEAR_POS;
  }
  if (modeEnum == ModeEnum::GearVelocity) {
    return RSL_DRIVE_MODE_NAME_SHORT_GEAR_VEL;
  }
  if (modeEnum == ModeEnum::JointPosition) {
    return RSL_DRIVE_MODE_NAME_SHORT_JOINT_POS;
  }
  if (modeEnum == ModeEnum::JointPositionVelocity) {
    return RSL_DRIVE_MODE_NAME_SHORT_JOINT_POS_VEL;
  }
  if (modeEnum == ModeEnum::JointPositionVelocityTorque) {
    return RSL_DRIVE_MODE_NAME_SHORT_JOINT_POS_VEL_TOR;
  }
  if (modeEnum == ModeEnum::JointPositionVelocityTorquePidGains) {
    return RSL_DRIVE_MODE_NAME_SHORT_JOINT_POS_VEL_TOR_PID;
  }
  if (modeEnum == ModeEnum::JointTorque) {
    return RSL_DRIVE_MODE_NAME_SHORT_JOINT_TOR;
  }
  if (modeEnum == ModeEnum::JointVelocity) {
    return RSL_DRIVE_MODE_NAME_SHORT_JOINT_VEL;
  }
  if (modeEnum == ModeEnum::MotorPosition) {
    return RSL_DRIVE_MODE_NAME_SHORT_MOTOR_POS;
  }
  if (modeEnum == ModeEnum::MotorVelocity) {
    return RSL_DRIVE_MODE_NAME_SHORT_MOTOR_VEL;
  }
  return RSL_DRIVE_MODE_NAME_SHORT_NA;
}

ModeEnum modeNameToEnum(const std::string & string)
{
  if (string == RSL_DRIVE_MODE_NAME_CURRENT) {
    return ModeEnum::Current;
  }
  if (string == RSL_DRIVE_MODE_NAME_JOINT_POS_TOR) {
    return ModeEnum::JointPositionTorque;
  }
  if (string == RSL_DRIVE_MODE_NAME_DISABLE) {
    return ModeEnum::Disable;
  }
  if (string == RSL_DRIVE_MODE_NAME_FREEZE) {
    return ModeEnum::Freeze;
  }
  if (string == RSL_DRIVE_MODE_NAME_GEAR_POS) {
    return ModeEnum::GearPosition;
  }
  if (string == RSL_DRIVE_MODE_NAME_GEAR_VEL) {
    return ModeEnum::GearVelocity;
  }
  if (string == RSL_DRIVE_MODE_NAME_JOINT_POS) {
    return ModeEnum::JointPosition;
  }
  if (string == RSL_DRIVE_MODE_NAME_JOINT_POS_VEL) {
    return ModeEnum::JointPositionVelocity;
  }
  if (string == RSL_DRIVE_MODE_NAME_JOINT_POS_VEL_TOR) {
    return ModeEnum::JointPositionVelocityTorque;
  }
  if (string == RSL_DRIVE_MODE_NAME_JOINT_POS_VEL_TOR_PID) {
    return ModeEnum::JointPositionVelocityTorquePidGains;
  }
  if (string == RSL_DRIVE_MODE_NAME_JOINT_TOR) {
    return ModeEnum::JointTorque;
  }
  if (string == RSL_DRIVE_MODE_NAME_JOINT_VEL) {
    return ModeEnum::JointVelocity;
  }
  if (string == RSL_DRIVE_MODE_NAME_MOTOR_POS) {
    return ModeEnum::MotorPosition;
  }
  if (string == RSL_DRIVE_MODE_NAME_MOTOR_VEL) {
    return ModeEnum::MotorVelocity;
  }
  return ModeEnum::NA;
}

std::ostream & operator<<(std::ostream & out, const ModeEnum modeEnum)
{
  return out << modeEnumToName(modeEnum);
}


} // mode
} // rsl_drive_sdk
