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
#include <limits>


namespace rsl_drive_sdk
{
namespace common
{


//! Class implementing limits.
class Limits
{
protected:
  // The limits are an interval [min, max].
  // If min == max == 0.0, it is infinity, [-inf, inf].

  //! Lower limit.
  double min_ = 0.0;
  //! Upper limit.
  double max_ = 0.0;

public:
  /*!
   * Constructor, setting limits to infinity.
   */
  Limits();

  /*!
   * Constructor setting the limits
   * @param min Lower limit.
   * @param max Upper limit.
   */
  Limits(const double min, const double max);

  /*!
   * Destructor.
   */
  virtual ~Limits();

  /*!
   * Check if the limits are infinity.
   * @return True iff the limits are infinity.
   */
  bool areInf() const;

  /*!
   * Get the lower limit.
   * @return Lower limit.
   */
  double min() const;

  /*!
   * Get the lower limit by reference.
   * @return Reference to the lower limit.
   */
  double & min();

  /*!
   * Get the upper limit.
   * @return Upper limit.
   */
  double max() const;

  /*!
   * Get the upper limit by reference.
   * @return Reference to the upper limit.
   */
  double & max();

  /*!
   * Check if a value lies within the limits.
   * @param value Value.
   * @return True iff the value lies within the limits.
   */
  bool liesWithin(const double value) const;

  /*!
   * Create new shifted the limits.
   * @param offset Offset to shift.
   * @return Shifted limits.
   */
  Limits operator+(const double offset) const;

  /*!
   * Shift limits by an offset.
   * @param offset Offset.
   * @return Shifted limits.
   */
  Limits & operator+=(const double offset);

  /*!
   * Create new shifted the limits.
   * @param offset Offset to shift.
   * @return Shifted limits.
   */
  Limits operator-(const double offset) const;

  /*!
   * Shift limits by an offset.
   * @param offset Offset.
   * @return Reference to shifted limits.
   */
  Limits & operator-=(const double offset);

  /*!
   * Create new scaled limits.
   * @param scaling Scaling factor.
   * @return Scaled limits.
   */
  Limits operator*(const double scaling) const;

  /*!
   * Scale limits by a factor.
   * @param scaling Scaling factor.
   * @return Reference to scaled limits.
   */
  Limits & operator*=(const double scaling);
};


} // common
} // rsl_drive_sdk
