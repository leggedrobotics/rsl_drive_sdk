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
#include "rsl_drive_sdk/fsm/StateBase.hpp"
#include "rsl_drive_sdk/Drive.hpp"


namespace rsl_drive_sdk
{
namespace fsm
{


StateBase::StateBase(
  DriveEthercatDevice & rsl_drive_sdk,
  std::atomic<StateEnum> & goalStateEnum,
  const StateEnum stateEnum,
  const std::map<StateEnum, uint8_t> & goalStateEnumToControlword)
: rsl_drive_sdk_(rsl_drive_sdk),
  goalStateEnum_(goalStateEnum),
  stateEnum_(stateEnum),
  name_(stateEnumToName(stateEnum)),
  goalStateEnumToControlword_(goalStateEnumToControlword) {}

StateBase::~StateBase() {}

StateEnum StateBase::getStateEnum() const
{
  return stateEnum_;
}

void StateBase::enter()
{
  enterBase();
  this->enterDerived();
}

void StateBase::update()
{
  updateBase();
  this->updateDerived();
}

void StateBase::leave()
{
  this->leaveDerived();
  leaveBase();
}

std::string StateBase::getName()
{
  return rsl_drive_sdk_.getName();
}

void StateBase::enterBase()
{
  MELO_INFO_STREAM("[" << rsl_drive_sdk_.getName() << "]: " << "Entered FSM state '" << name_ <<
        "'.");
  enteredCounter_++;
  controlwordSentForState_ = StateEnum::NA;

  // Reset the goal state enum if it has been reached
  if (stateEnum_ == goalStateEnum_) {
    MELO_DEBUG_STREAM("Reached goal FSM state.");
    goalStateEnum_ = StateEnum::NA;
  }
}

void StateBase::updateBase()
{
  if (!isDone_) {
    return;
  }

  // Send a controlword if a goal state is set and it has not been sent yet.
  if (goalStateEnum_ == StateEnum::NA) {
    return;
  }
  if (controlwordSentForState_ == goalStateEnum_) {
    return;

  }
  auto it = goalStateEnumToControlword_.find(goalStateEnum_);
  if (it == goalStateEnumToControlword_.end()) {
    return;
  }

  rsl_drive_sdk_.setControlword(it->second);
  //rsl_drive_sdk_.sendControlword(it->second);
  controlwordSentForState_ = goalStateEnum_;
}

void StateBase::leaveBase()
{
  // Reset controlword.
  rsl_drive_sdk_.resetControlword();

  MELO_DEBUG_STREAM("Left FSM state '" << name_ << "'.");
}


} // fsm
} // rsl_drive_sdk
