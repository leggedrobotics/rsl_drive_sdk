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
#include "rsl_drive_sdk/fsm/StateControlOp.hpp"
#include "rsl_drive_sdk/Drive.hpp"


namespace rsl_drive_sdk
{
namespace fsm
{


StateControlOp::StateControlOp(
  DriveEthercatDevice & rsl_drive_sdk,
  std::atomic<StateEnum> & goalStateEnum)
: StateBase(rsl_drive_sdk, goalStateEnum, StateEnum::ControlOp,
    {{StateEnum::Configure, RSL_DRIVE_CW_ID_CONTROL_OP_TO_STANDBY},
      {StateEnum::Standby, RSL_DRIVE_CW_ID_CONTROL_OP_TO_STANDBY},
      {StateEnum::Calibrate, RSL_DRIVE_CW_ID_CONTROL_OP_TO_STANDBY},
      {StateEnum::MotorOp, RSL_DRIVE_CW_ID_CONTROL_OP_TO_MOTOR_OP}}) {}

StateControlOp::~StateControlOp() {}

void StateControlOp::enterDerived()
{
//  rsl_drive_sdk_.getCommunicationInterface()->configureHeartBeat(false);
}

void StateControlOp::leaveDerived()
{
  /* If auto stage last command is enabled, a freeze has to overwrite the staged command.
   * Otherwise a potentially unsafe staged command will again be executed next time ControlOp
   * is reached.
   */
  if (rsl_drive_sdk_.getConfiguration().getAutoStageLastCommand()) {
    rsl_drive_sdk_.stageFreeze();
  }

//  rsl_drive_sdk_.getCommunicationInterface()->configureHeartBeat(true);
}


} // fsm
} // rsl_drive_sdk
