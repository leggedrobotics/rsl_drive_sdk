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


// rsl_drive_sdk
#include "rsl_drive_sdk/Command.hpp"
#include "rsl_drive_sdk/State.hpp"
#include "rsl_drive_sdk/StateExtended.hpp"


namespace rsl_drive_sdk
{


//! Reading of the Drive containing arbitrary command and state information.
template<typename CommandT, typename StateT>
class ReadingT
{
public:
  using Command = CommandT;
  using State = StateT;

protected:
  //! Command which was sent to the Drive.
  Command commanded_;
  //! Current state of the Drive.
  State state_;

public:
  ReadingT() {}
  virtual ~ReadingT() {}

  const Command & getCommanded() const
  {
    return commanded_;
  }

  Command & getCommanded()
  {
    return commanded_;
  }

  void setCommanded(const Command & commanded)
  {
    commanded_ = commanded;
  }

  const State & getState() const
  {
    return state_;
  }

  State & getState()
  {
    return state_;
  }

  void setState(const State & state)
  {
    state_ = state;
  }

  std::string asString(const std::string & prefix = "") const
  {
    std::stringstream ss;
    ss << prefix << commanded_.asString("  ") << std::endl;
    ss << prefix << state_.asString("  ") << std::endl;
    return ss.str();
  }
};

template<typename CommandT, typename StateT>
std::ostream & operator<<(std::ostream & out, const ReadingT<CommandT, StateT> & reading)
{
  out << "Reading:" << std::endl;
  out << reading.asString("  ") << std::endl;
  return out;
}

//! Reading of the Drive containing the command and the state.
using Reading = ReadingT<Command, State>;


} // rsl_drive_sdk
