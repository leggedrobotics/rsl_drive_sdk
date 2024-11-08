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
#include <chrono>
#include <string>
#include <vector>

// rsl_drive_sdk
#include <rsl_drive_sdk/common/Macros.hpp>


namespace rsl_drive_sdk
{


//! Simple templated log class, used for calibration.
template<typename DataT>
class Log
{
protected:
  //! Type of a time point.
  using TimePoint = std::chrono::system_clock::time_point;

  //! Name of the log for referencing.
  std::string name_;
  //! Time stamp of the log.
  TimePoint stamp_;
  //! Logged data.
  std::vector<DataT> data_;

public:
  /*!
   * Constructor.
   * @param name Name of the log.
   */
  Log(const std::string name)
  : name_(name),
    stamp_(std::chrono::system_clock::now()) {}

  /*!
   * Destructor.
   */
  virtual ~Log() {}

  /*!
   * Get the name of the log.
   * @return Name of the log.
   */
  const std::string & getName() const
  {
    return name_;
  }

  /*!
   * Get the time stamp of the log.
   * @return Time stamp of the log.
   */
  const TimePoint & getStamp() const
  {
    return stamp_;
  }

  /*!
   * Get the logged data.
   * @return Logged data.
   */
  const std::vector<DataT> & getData() const
  {
    return data_;
  }

  /*!
   * Add data to log.
   * @param data Data to add.
   */
  void addData(const DataT & data)
  {
    data_.push_back(data);
  }
};


} // rsl_drive_sdk
