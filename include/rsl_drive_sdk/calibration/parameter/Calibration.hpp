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


// rsl_drive_sdk
#include "rsl_drive_sdk/calibration/parameter/FrictionEstimation.hpp"
#include "rsl_drive_sdk/calibration/parameter/GearAndJointEncoderHoming.hpp"
#include "rsl_drive_sdk/calibration/parameter/GearJointEncoderOffset.hpp"
#include "rsl_drive_sdk/calibration/parameter/ImuGyroscopeDcBias.hpp"
#include "rsl_drive_sdk/calibration/parameter/MotorEncoderOffset.hpp"
#include "rsl_drive_sdk/calibration/parameter/MotorEncoderParameters.hpp"
#include "rsl_drive_sdk/calibration/parameter/SpringStiffness.hpp"


namespace rsl_drive_sdk
{
namespace calibration
{
namespace parameter
{


/*!
 * Calibration class containing all calibration parameters.
 */
struct Calibration
{
  //! Motor encoder offset calibration parameters.
  MotorEncoderOffset motorEncoderOffset_;
  //! Motor encoder parameters calibration parameters.
  MotorEncoderParameters motorEncoderParameters_;
  //! Gear/joint encoder offset calibration parameters.
  GearJointEncoderOffset gearJointEncoderOffset_;
  //! Gear and joint encoder calibration parameters.
  GearAndJointEncoderHoming gearAndJointEncoderHoming_;
  //! IMU gyroscope DC bias calibration parameters.
  ImuGyroscopeDcBias imuGyroscopeDcBias_;
  //! Spring stiffness calibration parameters.
  SpringStiffness springStiffness_;
  //! Friction estimation calibration parameters.
  FrictionEstimation frictionEstimation_;

  /*!
   * Comparison operator.
   * @param other Other calibration parameters.
   * @return True if equal.
   */
  bool operator==(const Calibration & other) const;
};


} // parameter
} // calibration
} // rsl_drive_sdk
