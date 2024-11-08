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
#include "rsl_drive_sdk/fsm/StateConfigure.hpp"
#include "rsl_drive_sdk/Drive.hpp"


namespace rsl_drive_sdk
{
namespace fsm
{


StateConfigure::StateConfigure(
  DriveEthercatDevice & rsl_drive_sdk,
  std::atomic<StateEnum> & goalStateEnum)
: StateBase(rsl_drive_sdk, goalStateEnum, StateEnum::Configure,
    {{StateEnum::Calibrate, RSL_DRIVE_CW_ID_CONFIGURE_TO_CALIBRATE},
      {StateEnum::Standby, RSL_DRIVE_CW_ID_CONFIGURE_TO_STANDBY},
      {StateEnum::MotorOp, RSL_DRIVE_CW_ID_CONFIGURE_TO_STANDBY},
      {StateEnum::ControlOp, RSL_DRIVE_CW_ID_CONFIGURE_TO_STANDBY}}) {}

StateConfigure::~StateConfigure() {}

void StateConfigure::enterDerived()
{
  isDone_ = false;
}

void StateConfigure::updateDerived()
{
  if (isDone_) {
    return;
  }

  if (step_ == 0) {
    if (rsl_drive_sdk_.getConfiguration().getErrorStateBehavior()) {
      rsl_drive_sdk_.setErrorStateBehavior(
          rsl_drive_sdk_.getConfiguration().getErrorStateBehavior().value());
    }
    step_++;
    return;
  }

  if (step_ == 1) {
    if (rsl_drive_sdk_.getConfiguration().getMaxCurrent()) {
      rsl_drive_sdk_.setMaxCurrent(
          rsl_drive_sdk_.getConfiguration().getMaxCurrent().value());
    }
    step_++;
    return;
  }

  if (step_ == 2) {
    if (rsl_drive_sdk_.getConfiguration().getMaxMotorVelocity()) {
      rsl_drive_sdk_.setMaxMotorVelocity(
          rsl_drive_sdk_.getConfiguration().getMaxMotorVelocity().value());
    }
    step_++;
    return;
  }

  if (step_ == 3) {
    if (rsl_drive_sdk_.getConfiguration().getDirection()) {
      rsl_drive_sdk_.setDirection(
          rsl_drive_sdk_.getConfiguration().getDirection().value());
    }
    step_++;
    return;
  }

  if (step_ == 4) {
    if (rsl_drive_sdk_.getConfiguration().getJointPositionLimitsSoft()) {
      rsl_drive_sdk_.setJointPositionLimitsSoft(
          rsl_drive_sdk_.getConfiguration().getJointPositionLimitsSoft().value());
    }
    step_++;
    return;
  }

  if (step_ == 5) {
    if (rsl_drive_sdk_.getConfiguration().getJointPositionLimitsHard()) {
      rsl_drive_sdk_.setJointPositionLimitsHard(
          rsl_drive_sdk_.getConfiguration().getJointPositionLimitsHard().value());
    }
    step_++;
    return;
  }

  if (step_ == 6) {
    if (rsl_drive_sdk_.getConfiguration().getImuEnable()) {
      rsl_drive_sdk_.setImuEnable(rsl_drive_sdk_.getConfiguration().getImuEnable().value());
    }
    step_++;
    return;
  }

  if (step_ == 7) {
    if (rsl_drive_sdk_.getConfiguration().getImuAccelerometerRange()) {
      rsl_drive_sdk_.setImuAccelerometerRange(
          rsl_drive_sdk_.getConfiguration().getImuAccelerometerRange().value());
    }
    step_++;
    return;
  }

  if (step_ == 8) {
    if (rsl_drive_sdk_.getConfiguration().getImuGyroscopeRange()) {
      rsl_drive_sdk_.setImuGyroscopeRange(
          rsl_drive_sdk_.getConfiguration().getImuGyroscopeRange().value());
    }
    step_++;
    return;
  }

  if (step_ == 9) {
    if (rsl_drive_sdk_.getConfiguration().getMaxJointTorque()) {
      rsl_drive_sdk_.setMaxJointTorque(
          rsl_drive_sdk_.getConfiguration().getMaxJointTorque().value());
    }
    step_++;
    return;
  }

  if (step_ == 10) {
    if (rsl_drive_sdk_.getConfiguration().getCurrentIntegratorSaturation()) {
      rsl_drive_sdk_.setCurrentIntegratorSaturation(
          rsl_drive_sdk_.getConfiguration().getCurrentIntegratorSaturation().value());
    }
    step_++;
    return;
  }

  if (step_ == 11) {
    if (rsl_drive_sdk_.getConfiguration().getJointTorqueIntegratorSaturation()) {
      rsl_drive_sdk_.setJointTorqueIntegratorSaturation(
          rsl_drive_sdk_.getConfiguration().getJointTorqueIntegratorSaturation().value());
    }
    step_++;
    return;
  }

  if (step_ == 12) {
    if (rsl_drive_sdk_.getConfiguration().getFanMode()) {
      rsl_drive_sdk_.setFanMode(
          rsl_drive_sdk_.getConfiguration().getFanMode().value());
    }
    step_++;
    return;
  }

  if (step_ == 13) {
    if (rsl_drive_sdk_.getConfiguration().getFanIntensity()) {
      rsl_drive_sdk_.setFanIntensity(
          rsl_drive_sdk_.getConfiguration().getFanIntensity().value());
    }
    step_++;
    return;
  }

  if (step_ == 14) {
    if (rsl_drive_sdk_.getConfiguration().getFanLowerTemperature()) {
      rsl_drive_sdk_.setFanLowerTemperature(
          rsl_drive_sdk_.getConfiguration().getFanLowerTemperature().value());
    }
    step_++;
    return;
  }

  if (step_ == 15) {
    if (rsl_drive_sdk_.getConfiguration().getFanUpperTemperature()) {
      rsl_drive_sdk_.setFanUpperTemperature(
          rsl_drive_sdk_.getConfiguration().getFanUpperTemperature().value());
    }
    step_++;
    return;
  }

  if (step_ == 16) {
    if (rsl_drive_sdk_.getConfiguration().getMaxFreezeCurrent()) {
      rsl_drive_sdk_.setMaxFreezeCurrent(
          rsl_drive_sdk_.getConfiguration().getMaxFreezeCurrent().value());
    }
    step_++;
    step_mode_ = rsl_drive_sdk_.getConfiguration().getModes().begin();
    return;
  }

  if (step_ == 17) {
    const auto & modes = rsl_drive_sdk_.getConfiguration().getModes();

    if (step_mode_ != modes.end()) {

     // std::advance(modeIt, stepMode_);
      if (step_mode_->second->getPidGains()) {
        rsl_drive_sdk_.setControlGains(step_mode_->first,
              step_mode_->second->getPidGains().value());
      }
      step_mode_++;
      return;
    } else {
      step_++;
    }
  }

  if (step_ == 18) {
    if (rsl_drive_sdk_.getConfiguration().getGearJointVelocityFilterType()) {
      rsl_drive_sdk_.setGearJointVelocityFilterType(
            rsl_drive_sdk_.getConfiguration().getGearJointVelocityFilterType().value());
    }
    step_++;
    return;
  }

  if (step_ == 19) {
    if (rsl_drive_sdk_.getConfiguration().getGearJointVelocityKfNoiseVariance()) {
      rsl_drive_sdk_.setGearJointVelocityKfNoiseVariance(
            rsl_drive_sdk_.getConfiguration().getGearJointVelocityKfNoiseVariance().value());
    }
    step_++;
    return;
  }

  if (step_ == 20) {
    if (rsl_drive_sdk_.getConfiguration().getGearJointVelocityKfLambda2()) {
      rsl_drive_sdk_.setGearJointVelocityKfLambda2(
            rsl_drive_sdk_.getConfiguration().getGearJointVelocityKfLambda2().value());
    }
    step_++;
    return;
  }

  if (step_ == 21) {
    if (rsl_drive_sdk_.getConfiguration().getGearJointVelocityKfGamma()) {
      rsl_drive_sdk_.setGearJointVelocityKfGamma(
            rsl_drive_sdk_.getConfiguration().getGearJointVelocityKfGamma().value());
    }
    step_++;
    return;
  }

  if (step_ == 22) {
    if (rsl_drive_sdk_.getConfiguration().getGearJointVelocityEmaAlpha()) {
      rsl_drive_sdk_.setGearJointVelocityEmaAlpha(
            rsl_drive_sdk_.getConfiguration().getGearJointVelocityEmaAlpha().value());
    }
    step_++;
    return;
  }

  if (step_ == 23) {
    if (rsl_drive_sdk_.getConfiguration().getJointVelocityForAccelerationFilterType()) {
      rsl_drive_sdk_.setJointVelocityForAccelerationFilterType(
            rsl_drive_sdk_.getConfiguration().getJointVelocityForAccelerationFilterType().value());
    }
    step_++;
    return;
  }

  if (step_ == 24) {
    if (rsl_drive_sdk_.getConfiguration().getJointVelocityForAccelerationKfNoiseVariance()) {
      rsl_drive_sdk_.setJointVelocityForAccelerationKfNoiseVariance(
            rsl_drive_sdk_.getConfiguration().getJointVelocityForAccelerationKfNoiseVariance().value());
    }
    step_++;
    return;
  }

  if (step_ == 25) {
    if (rsl_drive_sdk_.getConfiguration().getJointVelocityForAccelerationKfLambda2()) {
      rsl_drive_sdk_.setJointVelocityForAccelerationKfLambda2(
            rsl_drive_sdk_.getConfiguration().getJointVelocityForAccelerationKfLambda2().value());
    }
    step_++;
    return;
  }

  if (step_ == 26) {
    if (rsl_drive_sdk_.getConfiguration().getJointVelocityForAccelerationKfGamma()) {
      rsl_drive_sdk_.setJointVelocityForAccelerationKfGamma(
            rsl_drive_sdk_.getConfiguration().getJointVelocityForAccelerationKfGamma().value());
    }
    step_++;
    return;
  }

  if (step_ == 27) {
    if (rsl_drive_sdk_.getConfiguration().getJointVelocityForAccelerationEmaAlpha()) {
      rsl_drive_sdk_.setJointVelocityForAccelerationEmaAlpha(
            rsl_drive_sdk_.getConfiguration().getJointVelocityForAccelerationEmaAlpha().value());
    }
    step_++;
    return;
  }

  if (step_ == 28) {
    if (rsl_drive_sdk_.getConfiguration().getJointAccelerationFilterType()) {
      rsl_drive_sdk_.setJointAccelerationFilterType(
            rsl_drive_sdk_.getConfiguration().getJointAccelerationFilterType().value());
    }
    step_++;
    return;
  }

  if (step_ == 29) {
    if (rsl_drive_sdk_.getConfiguration().getJointAccelerationKfNoiseVariance()) {
      rsl_drive_sdk_.setJointAccelerationKfNoiseVariance(
            rsl_drive_sdk_.getConfiguration().getJointAccelerationKfNoiseVariance().value());
    }
    step_++;
    return;
  }

  if (step_ == 30) {
    if (rsl_drive_sdk_.getConfiguration().getJointAccelerationKfLambda2()) {
      rsl_drive_sdk_.setJointAccelerationKfLambda2(
            rsl_drive_sdk_.getConfiguration().getJointAccelerationKfLambda2().value());
    }
    step_++;
    return;
  }

  if (step_ == 31) {
    if (rsl_drive_sdk_.getConfiguration().getJointAccelerationKfGamma()) {
      rsl_drive_sdk_.setJointAccelerationKfGamma(
            rsl_drive_sdk_.getConfiguration().getJointAccelerationKfGamma().value());
    }
    step_++;
    return;
  }

  if (step_ == 32) {
    if (rsl_drive_sdk_.getConfiguration().getJointAccelerationEmaAlpha()) {
      rsl_drive_sdk_.setJointAccelerationEmaAlpha(
            rsl_drive_sdk_.getConfiguration().getJointAccelerationEmaAlpha().value());
    }
    step_++;
    return;
  }
  if(step_ == 33) {
    if (rsl_drive_sdk_.getConfiguration().getDGainFilterCutoffFrequency()) {
      rsl_drive_sdk_.setDGainFilterCutoffFrequency(
            rsl_drive_sdk_.getConfiguration().getDGainFilterCutoffFrequency().value());
    }
    step_++;
    return;
  }

  isDone_ = true;
}


} // fsm
} // rsl_drive_sdk
