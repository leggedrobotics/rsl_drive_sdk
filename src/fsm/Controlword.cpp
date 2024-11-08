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
#include "rsl_drive_sdk/fsm/Controlword.hpp"


namespace rsl_drive_sdk
{
namespace fsm
{

std::string controlwordIdToString(const uint16_t controlwordId)
{
  if (controlwordId == RSL_DRIVE_CW_ID_CALIBRATE_TO_CONFIGURE) {
    return RSL_DRIVE_CW_NAME_CALIBRATE_TO_CONFIGURE;
  }
  if (controlwordId == RSL_DRIVE_CW_ID_CLEAR_ERRORS_TO_MOTOR_OP) {
    return RSL_DRIVE_CW_NAME_CLEAR_ERRORS_TO_MOTOR_OP;
  }
  if (controlwordId == RSL_DRIVE_CW_ID_CLEAR_ERRORS_TO_STANDBY) {
    return RSL_DRIVE_CW_NAME_CLEAR_ERRORS_TO_STANDBY;
  }
  if (controlwordId == RSL_DRIVE_CW_ID_CONFIGURE_TO_CALIBRATE) {
    return RSL_DRIVE_CW_NAME_CONFIGURE_TO_CALIBRATE;
  }
  if (controlwordId == RSL_DRIVE_CW_ID_CONFIGURE_TO_STANDBY) {
    return RSL_DRIVE_CW_NAME_CONFIGURE_TO_STANDBY;
  }
  if (controlwordId == RSL_DRIVE_CW_ID_CONTROL_OP_TO_MOTOR_OP) {
    return RSL_DRIVE_CW_NAME_CONTROL_OP_TO_MOTOR_OP;
  }
  if (controlwordId == RSL_DRIVE_CW_ID_CONTROL_OP_TO_STANDBY) {
    return RSL_DRIVE_CW_NAME_CONTROL_OP_TO_STANDBY;
  }
  if (controlwordId == RSL_DRIVE_CW_ID_MOTOR_OP_TO_CONTROL_OP) {
    return RSL_DRIVE_CW_NAME_MOTOR_OP_TO_CONTROL_OP;
  }
  if (controlwordId == RSL_DRIVE_CW_ID_MOTOR_OP_TO_STANDBY) {
    return RSL_DRIVE_CW_NAME_MOTOR_OP_TO_STANDBY;
  }
  if (controlwordId == RSL_DRIVE_CW_ID_STANDBY_TO_CONFIGURE) {
    return RSL_DRIVE_CW_NAME_STANDBY_TO_CONFIGURE;
  }
  if (controlwordId == RSL_DRIVE_CW_ID_STANDBY_TO_MOTOR_PREOP) {
    return RSL_DRIVE_CW_NAME_STANDBY_TO_MOTOR_PREOP;
  }
  if (controlwordId == RSL_DRIVE_CW_ID_WARM_RESET) {
    return RSL_DRIVE_CW_NAME_WARM_RESET;
  }
  return RSL_DRIVE_CW_NAME_NA;
}

uint16_t controlwordStringToId(const std::string & controlwordString)
{
  if (controlwordString == RSL_DRIVE_CW_NAME_CALIBRATE_TO_CONFIGURE) {
    return RSL_DRIVE_CW_ID_CALIBRATE_TO_CONFIGURE;
  }
  if (controlwordString == RSL_DRIVE_CW_NAME_CLEAR_ERRORS_TO_MOTOR_OP) {
    return RSL_DRIVE_CW_ID_CLEAR_ERRORS_TO_MOTOR_OP;
  }
  if (controlwordString == RSL_DRIVE_CW_NAME_CLEAR_ERRORS_TO_STANDBY) {
    return RSL_DRIVE_CW_ID_CLEAR_ERRORS_TO_STANDBY;
  }
  if (controlwordString == RSL_DRIVE_CW_NAME_CONFIGURE_TO_CALIBRATE) {
    return RSL_DRIVE_CW_ID_CONFIGURE_TO_CALIBRATE;
  }
  if (controlwordString == RSL_DRIVE_CW_NAME_CONFIGURE_TO_STANDBY) {
    return RSL_DRIVE_CW_ID_CONFIGURE_TO_STANDBY;
  }
  if (controlwordString == RSL_DRIVE_CW_NAME_CONTROL_OP_TO_MOTOR_OP) {
    return RSL_DRIVE_CW_ID_CONTROL_OP_TO_MOTOR_OP;
  }
  if (controlwordString == RSL_DRIVE_CW_NAME_CONTROL_OP_TO_STANDBY) {
    return RSL_DRIVE_CW_ID_CONTROL_OP_TO_STANDBY;
  }
  if (controlwordString == RSL_DRIVE_CW_NAME_MOTOR_OP_TO_CONTROL_OP) {
    return RSL_DRIVE_CW_ID_MOTOR_OP_TO_CONTROL_OP;
  }
  if (controlwordString == RSL_DRIVE_CW_NAME_MOTOR_OP_TO_STANDBY) {
    return RSL_DRIVE_CW_ID_MOTOR_OP_TO_STANDBY;
  }
  if (controlwordString == RSL_DRIVE_CW_NAME_STANDBY_TO_CONFIGURE) {
    return RSL_DRIVE_CW_ID_STANDBY_TO_CONFIGURE;
  }
  if (controlwordString == RSL_DRIVE_CW_NAME_STANDBY_TO_MOTOR_PREOP) {
    return RSL_DRIVE_CW_ID_STANDBY_TO_MOTOR_PREOP;
  }
  if (controlwordString == RSL_DRIVE_CW_NAME_WARM_RESET) {
    return RSL_DRIVE_CW_ID_WARM_RESET;
  }
  return RSL_DRIVE_CW_ID_NA;
}

} // fsm
} // rsl_drive_sdk
