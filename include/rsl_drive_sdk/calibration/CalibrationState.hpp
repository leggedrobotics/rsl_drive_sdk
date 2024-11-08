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
#include <cstdint>

// rsl_drive_sdk
#include "rsl_drive_sdk/calibration/CalibrationModeEnum.hpp"


namespace rsl_drive_sdk
{
namespace calibration
{


/*!
 * Calibration state bits. Can be used to indicate e.g. which calibrations have been
 * done or which calibrations should be reset.
 * Note: The bit-ordering must be the same as in the firmware; motordrive_types.h
 * Note: The aksim self-calibration states are only supported in Drive 3.
 */
struct CalibrationStateBits
{
  uint16_t motorEncoderOffset_ : 1;
  uint16_t motorEncoderParameters_ : 1;
  uint16_t gearJointEncoderOffset_ : 1;
  uint16_t gearAndJointEncoderHoming_ : 1;
  uint16_t frictionEstimation_ : 1;
  uint16_t imuGyroscopeDcBias_ : 1;
  uint16_t springStiffness_ : 1;
  uint16_t aksimGear_ : 1;
  uint16_t aksimJoint_ : 1;
  uint16_t unused_ : 7;
};

//! Calibration state union.
union CalibrationState{
  //! Bitwise access.
  CalibrationStateBits single_;
  //! Complete access.
  uint16_t all_ = 0;

  /*!
   * Check if a calibration has been done.
   * @param calibrationModeEnum Calibration mode enumerator.
   * @return True, iff a calibration has been done.
   */
  bool getSingleCalibrationState(const CalibrationModeEnum calibrationModeEnum) const;

  /*!
   * Set the state of a single calibration.
   * @param calibrationModeEnum Calibration mode enumerator.
   * @param isCalibrated    The new state of the calibration.
   */
  void setSingleCalibrationState(const CalibrationModeEnum calibrationModeEnum,
    const bool isCalibrated);
};


} // calibration
} // rsl_drive_sdk
