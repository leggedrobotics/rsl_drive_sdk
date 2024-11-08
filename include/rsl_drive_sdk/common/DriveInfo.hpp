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
#include <string>
#include <cstdint>

// rsl_drive_sdk
#include "rsl_drive_sdk/common/Version.hpp"

namespace rsl_drive_sdk
{


class DriveInfo
{
protected:
  std::string serialNumber_;

  std::string model_;
  std::string name_;
  uint16_t id_ = 0;

  common::Version bootloaderVersion_;
  common::Version firmwareVersion_;

public:
  DriveInfo();
  DriveInfo(
    const std::string & serialNumber,
    const std::string & model,
    const std::string & name,
    const uint16_t id,
    const common::Version & bootloaderVersion,
    const common::Version & firmwareVersion);
  virtual ~DriveInfo();

  std::string & getSerialNumber();
  const std::string & getSerialNumber() const;

  std::string & getModel();
  const std::string & getModel() const;

  std::string & getName();
  const std::string & getName() const;

  uint16_t & getId();
  uint16_t getId() const;

  common::Version & getBootloaderVersion();
  const common::Version & getBootloaderVersion() const;

  common::Version & getFirmwareVersion();
  const common::Version & getFirmwareVersion() const;
};

std::ostream & operator<<(std::ostream & ostream, const DriveInfo & driveInfo);


} // rsl_drive_sdk
