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
#include <map>

// rsl_drive_sdk
#include "rsl_drive_sdk/common/Log.hpp"


namespace rsl_drive_sdk
{
namespace common
{


//! Simple templated logger class, used for calibration.
template<typename DataT>
class Logger
{
protected:
  //! Type of a log.
  using LogT = Log<DataT>;
  //! Type of a map of logs.
  using LogTMap = std::map<std::string, LogT>;
  //! Type of a pointer to a log.
  using LogTPtr = LogT *;

  //! Map of logs.
  LogTMap logs_;
  //! Pointer to the active log. The null-pointer indicates no active log.
  LogTPtr activeLog_ = nullptr;

public:
  /*!
   * Constructor.
   */
  Logger() {}

  /*!
   * Destructor.
   */
  virtual ~Logger() {}

  /*!
   * Check if logging is active.
   * @return True if logging is active.
   */
  bool logIsActive() const
  {
    return static_cast<bool>(activeLog_);
  }

  /*!
   * Get a log by name.
   * @param name Name of the log.
   * @return Log.
   */
  LogT getLog(const std::string & name) const
  {
    const auto it = logs_.find(name);
    if (it == logs_.end()) {
      RSL_DRIVE_ERROR("Log with name '" << name << "' does not exist.");
      return LogT("");
    }
    return it->second;
  }

  /*!
   * Start a new log. Skips if a log with the given name already exists.
   * @param name Name of the new log.
   */
  void startLog(const std::string & name)
  {
    if (logIsActive()) {
      RSL_DRIVE_ERROR("Log with name '" << activeLog_->getName() <<
            "' is still active, cannot start a new one.");
      return;
    }

    const auto result = logs_.insert({name, LogT(name)});
    if (!result.second) {
      RSL_DRIVE_ERROR("Log with name '" << name << "' exists already, will not overwrite it.");
      return;
    }

    activeLog_ = &result.first->second;
    RSL_DRIVE_INFO("Started log with name '" << activeLog_->getName() << "'.");
  }

  /*!
   * Add data to the active log.
   * @param data Data.
   */
  void addDataToLog(const DataT & data)
  {
    if (!logIsActive()) {
      RSL_DRIVE_ERROR("No log is active, cannot add data.");
      return;
    }
    activeLog_->addData(data);
  }

  /*!
   * Stop the active log.
   */
  void stopLog()
  {
    if (!logIsActive()) {
      RSL_DRIVE_DEBUG("No log is active, cannot stop it.");
      return;
    }
    RSL_DRIVE_INFO("Stopped log with name '" << activeLog_->getName() << "'.");
    activeLog_ = nullptr;
  }

  /*!
   * Clear all logs.
   */
  void clearLogs()
  {
    if (logIsActive()) {
      stopLog();
    }

    logs_.clear();
  }
};


} // common
} // rsl_drive_sdk
