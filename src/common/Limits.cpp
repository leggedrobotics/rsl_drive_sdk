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
#include "rsl_drive_sdk/common/Limits.hpp"
#include "message_logger/message_logger.hpp"

namespace rsl_drive_sdk
{
namespace common
{


Limits::Limits() {}

Limits::Limits(const double min, const double max)
: min_(min),
  max_(max)
{
  if (max < min) {
    MELO_WARN_STREAM("The interval is an empty set, since max (" << max <<
          ") is smaller than min (" << min << ").");
  }
}

Limits::~Limits() {}

bool Limits::areInf() const
{
  return (min_ == 0.0) && (max_ == 0.0);
}

double Limits::min() const
{
  return min_;
}

double & Limits::min()
{
  return min_;
}

double Limits::max() const
{
  return max_;
}

double & Limits::max()
{
  return max_;
}

bool Limits::liesWithin(const double value) const
{
  return areInf() || ((min_ <= value) && (value <= max_));
}

Limits Limits::operator+(const double offset) const
{
  return areInf() ? *this : Limits(min_ + offset, max_ + offset);
}

Limits & Limits::operator+=(const double offset)
{
  *this = *this + offset;
  return *this;
}

Limits Limits::operator-(const double offset) const
{
  return *this + -offset;
}

Limits & Limits::operator-=(const double offset)
{
  *this = *this - offset;
  return *this;
}

Limits Limits::operator*(const double scaling) const
{
  if (areInf()) {
    return *this;
  } else if (scaling >= 0) {
    return Limits(min_ * scaling, max_ * scaling);
  } else {
    return Limits(max_ * scaling, min_ * scaling);
  }
}

Limits & Limits::operator*=(const double scaling)
{
  *this = *this * scaling;
  return *this;
}


} // common
} // rsl_drive_sdk
