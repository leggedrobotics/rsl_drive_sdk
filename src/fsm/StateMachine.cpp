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
#include "message_logger/message_logger.hpp"
#include "rsl_drive_sdk/fsm/StateCalibrate.hpp"
#include "rsl_drive_sdk/fsm/StateColdStart.hpp"
#include "rsl_drive_sdk/fsm/StateConfigure.hpp"
#include "rsl_drive_sdk/fsm/StateControlOp.hpp"
#include "rsl_drive_sdk/fsm/StateDeviceMissing.hpp"
#include "rsl_drive_sdk/fsm/StateError.hpp"
#include "rsl_drive_sdk/fsm/StateFatal.hpp"
#include "rsl_drive_sdk/fsm/StateMachine.hpp"
#include "rsl_drive_sdk/fsm/StateMotorOp.hpp"
#include "rsl_drive_sdk/fsm/StateStandby.hpp"
#include "rsl_drive_sdk/fsm/StateMotorPreOp.hpp"
#include "rsl_drive_sdk/fsm/StateWarmStart.hpp"
#include "rsl_drive_sdk/Drive.hpp"


namespace rsl_drive_sdk
{
namespace fsm
{


StateMachine::StateMachine(DriveEthercatDevice & rsl_drive_sdk)
: rsl_drive_sdk_(rsl_drive_sdk)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  activeStateEnum_ = StateEnum::NA;
  goalStateEnum_ = StateEnum::NA;
  addState(StateBasePtr(new StateCalibrate(rsl_drive_sdk_, goalStateEnum_)));
  addState(StateBasePtr(new StateColdStart(rsl_drive_sdk_, goalStateEnum_)));
  addState(StateBasePtr(new StateConfigure(rsl_drive_sdk_, goalStateEnum_)));
  addState(StateBasePtr(new StateControlOp(rsl_drive_sdk_, goalStateEnum_)));
  addState(StateBasePtr(new StateDeviceMissing(rsl_drive_sdk_, goalStateEnum_)));
  addState(StateBasePtr(new StateError(rsl_drive_sdk_, goalStateEnum_)));
  addState(StateBasePtr(new StateFatal(rsl_drive_sdk_, goalStateEnum_)));
  addState(StateBasePtr(new StateMotorOp(rsl_drive_sdk_, goalStateEnum_)));
  addState(StateBasePtr(new StateMotorPreOp(rsl_drive_sdk_, goalStateEnum_)));
  addState(StateBasePtr(new StateStandby(rsl_drive_sdk_, goalStateEnum_)));
  addState(StateBasePtr(new StateWarmStart(rsl_drive_sdk_, goalStateEnum_)));
}

StateMachine::~StateMachine() {}

void StateMachine::updateActiveState(const StateEnum newActiveStateEnum)
{
  // Check if the active state changed.
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto activeStateIt = states_.find(activeStateEnum_);
  StateBasePtr activeState = (activeStateIt ==
    states_.end()) ? StateBasePtr() : activeStateIt->second;
  if (newActiveStateEnum != activeStateEnum_) {
    auto newActiveStateIt = states_.find(newActiveStateEnum);
    if (newActiveStateIt == states_.end()) {
      MELO_WARN_STREAM("New active FSM state '" << stateEnumToName(newActiveStateEnum) <<
            "' has not been found in the list of states.");
    } else {
      if (activeState) {
        activeState->leave();
      }
      activeState = newActiveStateIt->second;
      activeStateEnum_ = newActiveStateEnum;
      activeState->enter();
    }
  }

  // Update the active state.
  if (activeState) {
    activeState->update();
  }
}

StateEnum StateMachine::getActiveStateEnum() const
{
  return activeStateEnum_;
}

StateEnum StateMachine::getGoalStateEnum() const
{
  return goalStateEnum_;
}

bool StateMachine::goalStateHasBeenReached() const
{
  return goalStateEnum_ == StateEnum::NA;
}

void StateMachine::setGoalStateEnum(const StateEnum goalStateEnum)
{
  if (goalStateEnum == StateEnum::NA) {
    return;
  }
  if (goalStateEnum == StateEnum::DeviceMissing) {
    MELO_WARN_STREAM("Cannot set goal FSM state to DeviceMissing.");
    return;
  }
  if (goalStateEnum == StateEnum::Error ||
    goalStateEnum == StateEnum::Fatal)
  {
    MELO_WARN_STREAM("Cannot set goal FSM state to Error or Fatal.");
    return;
  }
  if (goalStateEnum == StateEnum::ColdStart ||
    goalStateEnum == StateEnum::WarmStart ||
    goalStateEnum == StateEnum::MotorPreOp)
  {
    MELO_WARN_STREAM(
          "Cannot set goal FSM state to ColdStart, WarmStart or MotorPreOp (auto-transition states).");
    return;
  }
  if (goalStateEnum == getActiveStateEnum()) {
    MELO_DEBUG_STREAM("Device is already in goal state.");
    return;
  }

  MELO_DEBUG_STREAM("[" << rsl_drive_sdk_.getName() << "]Setting goal FSM state to '" <<
        stateEnumToName(goalStateEnum) << "'.");
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  goalStateEnum_ = goalStateEnum;
}

void StateMachine::clearGoalStateEnum()
{
  goalStateEnum_ = StateEnum::NA;
  MELO_DEBUG_STREAM("The goal FSM state has been cleared.");
}

std::string StateMachine::getName() const
{
  return rsl_drive_sdk_.getName();
}

void StateMachine::addState(const StateBasePtr & state)
{
  states_.insert({state->getStateEnum(), state});
}


} // fsm
} // rsl_drive_sdk
