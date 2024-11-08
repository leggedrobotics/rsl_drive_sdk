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
// std
#include <sstream>

// rsl_drive_sdk
#include "rsl_drive_sdk/common/Version.hpp"


namespace rsl_drive_sdk
{
namespace common
{


Version::Version() {}

Version::Version(const int major, const int minor, const int patch)
: major_(major),
  minor_(minor),
  patch_(patch) {}

Version::Version(const std::string & string)
{
  sscanf(string.c_str(), "%d.%d.%d", &major_, &minor_, &patch_);
}

Version & Version::fromString(const std::string & string)
{
  *this = Version(string);
  return *this;
}

std::string Version::toString() const
{
  std::stringstream ss;
  ss << major_ << "." << minor_ << "." << patch_;
  return ss.str();
}

bool Version::operator==(const Version & other) const
{
  return
    major_ == other.major_ &&
    minor_ == other.minor_ &&
    patch_ == other.patch_;
}

bool Version::operator!=(const Version & other) const
{
  return !(*this == other);
}

bool Version::operator<(const Version & other) const
{
  if (major_ < other.major_) {
    return true;
  } else if (major_ > other.major_) {
    return false;
  } else if (minor_ < other.minor_) {
    return true;
  } else if (minor_ > other.minor_) {
    return false;
  } else if (patch_ < other.patch_) {
    return true;
  } else {
    return false;
  }
}

bool Version::operator<=(const Version & other) const
{
  return  *this == other || *this < other;
}

bool Version::operator>(const Version & other) const
{
  return !(*this <= other);
}

bool Version::operator>=(const Version & other) const
{
  return !(*this < other);
}

std::ostream & operator<<(std::ostream & ostream, const Version & version)
{
  ostream << version.toString();
  return ostream;
}


} // common
} // rsl_drive_sdk
