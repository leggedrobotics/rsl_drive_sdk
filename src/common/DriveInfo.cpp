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
#include "rsl_drive_sdk/common/DriveInfo.hpp"


namespace rsl_drive_sdk
{


DriveInfo::DriveInfo() {}

DriveInfo::DriveInfo(
  const std::string & serialNumber,
  const std::string & model,
  const std::string & name,
  const uint16_t id,
  const common::Version & bootloaderVersion,
  const common::Version & firmwareVersion)
: serialNumber_(serialNumber),
  model_(model),
  name_(name),
  id_(id),
  bootloaderVersion_(bootloaderVersion),
  firmwareVersion_(firmwareVersion) {}

DriveInfo::~DriveInfo() {}

std::string & DriveInfo::getSerialNumber()
{
  return serialNumber_;
}

const std::string & DriveInfo::getSerialNumber() const
{
  return serialNumber_;
}

std::string & DriveInfo::getModel()
{
  return model_;
}

const std::string & DriveInfo::getModel() const
{
  return model_;
}

std::string & DriveInfo::getName()
{
  return name_;
}

const std::string & DriveInfo::getName() const
{
  return name_;
}

uint16_t & DriveInfo::getId()
{
  return id_;
}

uint16_t DriveInfo::getId() const
{
  return id_;
}

common::Version & DriveInfo::getBootloaderVersion()
{
  return bootloaderVersion_;
}

const common::Version & DriveInfo::getBootloaderVersion() const
{
  return bootloaderVersion_;
}

common::Version & DriveInfo::getFirmwareVersion()
{
  return firmwareVersion_;
}

const common::Version & DriveInfo::getFirmwareVersion() const
{
  return firmwareVersion_;
}

std::ostream & operator<<(std::ostream & ostream, const DriveInfo & driveInfo)
{
  ostream << "Serial number:      " << driveInfo.getSerialNumber() << std::endl;
  ostream << "Model:              " << driveInfo.getModel() << std::endl;
  ostream << "Name:               " << driveInfo.getName() << std::endl;
  ostream << "ID:                 " << driveInfo.getId() << std::endl;
  ostream << "Bootloader version: " << driveInfo.getBootloaderVersion() << std::endl;
  ostream << "Firmware version:   " << driveInfo.getFirmwareVersion();
  return ostream;
}


} // rsl_drive_sdk
