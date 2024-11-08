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
#include <atomic>
#include <memory>
#include <map>
#include <string>

// rsl_drive_sdk
#include "rsl_drive_sdk/fsm/Controlword.hpp"
#include "rsl_drive_sdk/fsm/StateEnum.hpp"

namespace rsl_drive_sdk
{

class DriveEthercatDevice;

namespace fsm
{


class StateBase
{
public:
  bool isDone_ = true;

protected:
  DriveEthercatDevice & rsl_drive_sdk_;
  std::atomic<StateEnum> & goalStateEnum_;

  StateEnum stateEnum_;
  std::string name_;
  unsigned int enteredCounter_ = 0;

  std::map<StateEnum, uint8_t> goalStateEnumToControlword_;
  StateEnum controlwordSentForState_ = StateEnum::NA;

protected:
  StateBase(
    DriveEthercatDevice & rsl_drive_sdk,
    std::atomic<StateEnum> & goalStateEnum,
    const StateEnum stateEnum,
    const std::map<StateEnum, uint8_t> & goalStateEnumToControlword = {});
  virtual ~StateBase();

public:
  StateEnum getStateEnum() const;

  void enter();
  void update();
  void leave();

protected:
  std::string getName();

  void enterBase();
  virtual void enterDerived() {}
  void updateBase();
  virtual void updateDerived() {}
  void leaveBase();
  virtual void leaveDerived() {}
};

using StateBasePtr = std::shared_ptr<StateBase>;


} // fsm
} // rsl_drive_sdk
