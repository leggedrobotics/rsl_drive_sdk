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
#include <iostream>
#include <string>


namespace rsl_drive_sdk
{
namespace fsm
{


// FSM state IDs.
// NOTE: This mapping must be equal to the mapping in the firmware.
#define RSL_DRIVE_STATE_ID_CALIBRATE             (4)
#define RSL_DRIVE_STATE_ID_COLDSTART             (1)
#define RSL_DRIVE_STATE_ID_CONFIGURE             (3)
#define RSL_DRIVE_STATE_ID_CONTROLOP             (7)
#define RSL_DRIVE_STATE_ID_DEVICE_MISSING        (11)
#define RSL_DRIVE_STATE_ID_ERROR                 (8)
#define RSL_DRIVE_STATE_ID_FATAL                 (9)
#define RSL_DRIVE_STATE_ID_MOTOROP               (6)
#define RSL_DRIVE_STATE_ID_MOTORPREOP            (10)
#define RSL_DRIVE_STATE_ID_NA                    (0)
#define RSL_DRIVE_STATE_ID_STANDBY               (5)
#define RSL_DRIVE_STATE_ID_WARMSTART             (2)

// FSM state names.
#define RSL_DRIVE_STATE_NAME_CALIBRATE           ("Calibrate")
#define RSL_DRIVE_STATE_NAME_COLDSTART           ("ColdStart")
#define RSL_DRIVE_STATE_NAME_CONFIGURE           ("Configure")
#define RSL_DRIVE_STATE_NAME_CONTROLOP           ("ControlOp")
#define RSL_DRIVE_STATE_NAME_DEVICE_MISSING      ("DeviceMissing")
#define RSL_DRIVE_STATE_NAME_ERROR               ("Error")
#define RSL_DRIVE_STATE_NAME_FATAL               ("Fatal")
#define RSL_DRIVE_STATE_NAME_MOTOROP             ("MotorOp")
#define RSL_DRIVE_STATE_NAME_MOTORPREOP          ("MotorPreOp")
#define RSL_DRIVE_STATE_NAME_NA                  ("N/A")
#define RSL_DRIVE_STATE_NAME_STANDBY             ("Standby")
#define RSL_DRIVE_STATE_NAME_WARMSTART           ("WarmStart")

// FSM state enumerators.
enum class StateEnum
{
  Calibrate,
  ColdStart,
  Configure,
  ControlOp,
  DeviceMissing,
  Error,
  Fatal,
  MotorOp,
  MotorPreOp,
  NA,
  Standby,
  WarmStart
};

uint16_t stateEnumToId(const StateEnum stateEnum);
StateEnum stateIdToEnum(uint16_t stateId);

std::string stateEnumToName(const StateEnum stateEnum);
StateEnum stateNameToEnum(const std::string & string);

std::ostream & operator<<(std::ostream & out, const StateEnum stateEnum);


} // fsm
} // rsl_drive_sdk
