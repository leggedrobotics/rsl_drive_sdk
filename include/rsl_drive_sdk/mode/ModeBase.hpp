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
#include <memory>
#include <mutex>
#include <optional>

// rsl_drive_sdk
#include "rsl_drive_sdk/mode/ModeEnum.hpp"
#include "rsl_drive_sdk/mode/PidGains.hpp"


namespace rsl_drive_sdk
{
namespace mode
{


class ModeBase
{
protected:
  mutable std::recursive_mutex mutex_;

  ModeEnum modeEnum_;

  std::optional<PidGainsF> pidGains_;

public:
  bool controlCurrent_ = false;
  bool controlMotorPosition_ = false;
  bool controlMotorVelocity_ = false;
  bool controlGearPosition_ = false;
  bool controlGearVelocity_ = false;
  bool controlJointPosition_ = false;
  bool controlJointVelocity_ = false;
  bool controlJointTorque_ = false;
  bool customGains_ = false;

protected:
  ModeBase(const ModeEnum modeEnum);
  virtual ~ModeBase();

public:
  ModeEnum getModeEnum() const;
  std::string getName() const;

  void setPidGains(const PidGainsF & pidGains);
  std::optional<PidGainsF> getPidGains() const;
};

using ModeBasePtr = std::shared_ptr<ModeBase>;


} // mode
} // rsl_drive_sdk
