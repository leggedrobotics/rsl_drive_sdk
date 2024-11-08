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
#include <cmath>
#include <ostream>

namespace rsl_drive_sdk
{
namespace mode
{


template<typename T>
class PidGains
{
protected:
  T p_ = 0.0;
  T i_ = 0.0;
  T d_ = 0.0;

public:
  PidGains() {}

  PidGains(T p, T i, T d)
  : p_(p),
    i_(i),
    d_(d) {}

  virtual ~PidGains() {}

  T getP() const
  {
    return p_;
  }

  T & getP()
  {
    return p_;
  }

  void setP(T p)
  {
    p_ = p;
  }

  T getI() const
  {
    return i_;
  }

  T & getI()
  {
    return i_;
  }

  void setI(T i)
  {
    i_ = i;
  }

  T getD() const
  {
    return d_;
  }

  T & getD()
  {
    return d_;
  }

  void setD(T d)
  {
    d_ = d;
  }

  PidGains operator*(const double scaling) const
  {
    return PidGains(p_ * scaling, i_ * scaling, d_ * scaling);
  }

  PidGains & operator*=(const double scaling)
  {
    *this = *this * scaling;
    return *this;
  }

  bool isValid() const
  {
    return
      std::isfinite(p_) &&
      std::isfinite(i_) &&
      std::isfinite(d_);
  }
};

template<typename T>
std::ostream & operator<<(std::ostream & out, const PidGains<T> & command)
{
  return out << command.getP() << ", " << command.getI() << ", " << command.getD();
}

using PidGainsF = PidGains<float>;
using PidGainsD = PidGains<double>;


} // mode
} // rsl_drive_sdk
