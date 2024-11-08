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
#include "rsl_drive_sdk/fsm/StateEnum.hpp"


namespace rsl_drive_sdk
{
namespace fsm
{


uint16_t stateEnumToId(const StateEnum stateEnum)
{
  if (stateEnum == StateEnum::Calibrate) {
    return RSL_DRIVE_STATE_ID_CALIBRATE;
  }
  if (stateEnum == StateEnum::ColdStart) {
    return RSL_DRIVE_STATE_ID_COLDSTART;
  }
  if (stateEnum == StateEnum::Configure) {
    return RSL_DRIVE_STATE_ID_CONFIGURE;
  }
  if (stateEnum == StateEnum::ControlOp) {
    return RSL_DRIVE_STATE_ID_CONTROLOP;
  }
  if (stateEnum == StateEnum::DeviceMissing) {
    return RSL_DRIVE_STATE_ID_DEVICE_MISSING;
  }
  if (stateEnum == StateEnum::Error) {
    return RSL_DRIVE_STATE_ID_ERROR;
  }
  if (stateEnum == StateEnum::Fatal) {
    return RSL_DRIVE_STATE_ID_FATAL;
  }
  if (stateEnum == StateEnum::MotorOp) {
    return RSL_DRIVE_STATE_ID_MOTOROP;
  }
  if (stateEnum == StateEnum::MotorPreOp) {
    return RSL_DRIVE_STATE_ID_MOTORPREOP;
  }
  if (stateEnum == StateEnum::Standby) {
    return RSL_DRIVE_STATE_ID_STANDBY;
  }
  if (stateEnum == StateEnum::WarmStart) {
    return RSL_DRIVE_STATE_ID_WARMSTART;
  }
  return RSL_DRIVE_STATE_ID_NA;
}

StateEnum stateIdToEnum(uint16_t stateId)
{
  if (stateId == RSL_DRIVE_STATE_ID_CALIBRATE) {
    return StateEnum::Calibrate;
  }
  if (stateId == RSL_DRIVE_STATE_ID_COLDSTART) {
    return StateEnum::ColdStart;
  }
  if (stateId == RSL_DRIVE_STATE_ID_CONFIGURE) {
    return StateEnum::Configure;
  }
  if (stateId == RSL_DRIVE_STATE_ID_CONTROLOP) {
    return StateEnum::ControlOp;
  }
  if (stateId == RSL_DRIVE_STATE_ID_DEVICE_MISSING) {
    return StateEnum::DeviceMissing;
  }
  if (stateId == RSL_DRIVE_STATE_ID_ERROR) {
    return StateEnum::Error;
  }
  if (stateId == RSL_DRIVE_STATE_ID_FATAL) {
    return StateEnum::Fatal;
  }
  if (stateId == RSL_DRIVE_STATE_ID_MOTOROP) {
    return StateEnum::MotorOp;
  }
  if (stateId == RSL_DRIVE_STATE_ID_MOTORPREOP) {
    return StateEnum::MotorPreOp;
  }
  if (stateId == RSL_DRIVE_STATE_ID_STANDBY) {
    return StateEnum::Standby;
  }
  if (stateId == RSL_DRIVE_STATE_ID_WARMSTART) {
    return StateEnum::WarmStart;
  }
  return StateEnum::NA;
}

std::string stateEnumToName(const StateEnum stateEnum)
{
  if (stateEnum == StateEnum::Calibrate) {
    return RSL_DRIVE_STATE_NAME_CALIBRATE;
  }
  if (stateEnum == StateEnum::ColdStart) {
    return RSL_DRIVE_STATE_NAME_COLDSTART;
  }
  if (stateEnum == StateEnum::Configure) {
    return RSL_DRIVE_STATE_NAME_CONFIGURE;
  }
  if (stateEnum == StateEnum::ControlOp) {
    return RSL_DRIVE_STATE_NAME_CONTROLOP;
  }
  if (stateEnum == StateEnum::DeviceMissing) {
    return RSL_DRIVE_STATE_NAME_DEVICE_MISSING;
  }
  if (stateEnum == StateEnum::Error) {
    return RSL_DRIVE_STATE_NAME_ERROR;
  }
  if (stateEnum == StateEnum::Fatal) {
    return RSL_DRIVE_STATE_NAME_FATAL;
  }
  if (stateEnum == StateEnum::MotorOp) {
    return RSL_DRIVE_STATE_NAME_MOTOROP;
  }
  if (stateEnum == StateEnum::MotorPreOp) {
    return RSL_DRIVE_STATE_NAME_MOTORPREOP;
  }
  if (stateEnum == StateEnum::Standby) {
    return RSL_DRIVE_STATE_NAME_STANDBY;
  }
  if (stateEnum == StateEnum::WarmStart) {
    return RSL_DRIVE_STATE_NAME_WARMSTART;
  }
  return RSL_DRIVE_STATE_NAME_NA;
}

StateEnum stateNameToEnum(const std::string & string)
{
  if (string == RSL_DRIVE_STATE_NAME_CALIBRATE) {
    return StateEnum::Calibrate;
  }
  if (string == RSL_DRIVE_STATE_NAME_COLDSTART) {
    return StateEnum::ColdStart;
  }
  if (string == RSL_DRIVE_STATE_NAME_CONFIGURE) {
    return StateEnum::Configure;
  }
  if (string == RSL_DRIVE_STATE_NAME_CONTROLOP) {
    return StateEnum::ControlOp;
  }
  if (string == RSL_DRIVE_STATE_NAME_DEVICE_MISSING) {
    return StateEnum::DeviceMissing;
  }
  if (string == RSL_DRIVE_STATE_NAME_ERROR) {
    return StateEnum::Error;
  }
  if (string == RSL_DRIVE_STATE_NAME_FATAL) {
    return StateEnum::Fatal;
  }
  if (string == RSL_DRIVE_STATE_NAME_MOTOROP) {
    return StateEnum::MotorOp;
  }
  if (string == RSL_DRIVE_STATE_NAME_MOTORPREOP) {
    return StateEnum::MotorPreOp;
  }
  if (string == RSL_DRIVE_STATE_NAME_STANDBY) {
    return StateEnum::Standby;
  }
  if (string == RSL_DRIVE_STATE_NAME_WARMSTART) {
    return StateEnum::WarmStart;
  }
  return StateEnum::NA;
}

std::ostream & operator<<(std::ostream & out, const StateEnum stateEnum)
{
  return out << stateEnumToName(stateEnum);
}


} // fsm
} // rsl_drive_sdk
