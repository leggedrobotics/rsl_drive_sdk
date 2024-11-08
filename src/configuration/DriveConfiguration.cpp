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
#include "rsl_drive_sdk/configuration/DriveConfiguration.hpp"
#include "rsl_drive_sdk/mode/ModeCurrent.hpp"

#include "rsl_drive_sdk/mode/ModeDisable.hpp"
#include "rsl_drive_sdk/mode/ModeFreeze.hpp"
#include "rsl_drive_sdk/mode/ModeGearPosition.hpp"
#include "rsl_drive_sdk/mode/ModeGearVelocity.hpp"
#include "rsl_drive_sdk/mode/ModeJointPosition.hpp"
#include "rsl_drive_sdk/mode/ModeJointPositionVelocity.hpp"
#include "rsl_drive_sdk/mode/ModeJointPositionVelocityTorque.hpp"
#include "rsl_drive_sdk/mode/ModeJointPositionVelocityTorquePidGains.hpp"
#include "rsl_drive_sdk/mode/ModeJointTorque.hpp"
#include "rsl_drive_sdk/mode/ModeJointVelocity.hpp"
#include "rsl_drive_sdk/mode/ModeMotorPosition.hpp"
#include "rsl_drive_sdk/mode/ModeMotorVelocity.hpp"
#include "rsl_drive_sdk/mode/ModeJointPositionTorque.hpp"

namespace rsl_drive_sdk
{
namespace configuration
{


Configuration::Configuration()
: errorStateBehavior_(0),
  maxCurrent_(28.0),
  maxMotorVelocity_(680.7),
  direction_(1),
  imuEnable_(true),
  imuAccelerometerRange_(1),
  imuGyroscopeRange_(1),
  fake_(false)
{

    //std::lock_guard<std::recursive_mutex> lock(mutex_);
  addMode(mode::ModeBasePtr(new mode::ModeCurrent()));
  addMode(mode::ModeBasePtr(new mode::ModeJointPositionTorque()));
  addMode(mode::ModeBasePtr(new mode::ModeDisable()));
  addMode(mode::ModeBasePtr(new mode::ModeFreeze()));
  addMode(mode::ModeBasePtr(new mode::ModeGearPosition()));
  addMode(mode::ModeBasePtr(new mode::ModeGearVelocity()));
  addMode(mode::ModeBasePtr(new mode::ModeJointPosition()));
  addMode(mode::ModeBasePtr(new mode::ModeJointPositionVelocity()));
  addMode(mode::ModeBasePtr(new mode::ModeJointPositionVelocityTorque()));
  addMode(mode::ModeBasePtr(new mode::ModeJointPositionVelocityTorquePidGains()));
  addMode(mode::ModeBasePtr(new mode::ModeJointTorque()));
  addMode(mode::ModeBasePtr(new mode::ModeJointVelocity()));
  addMode(mode::ModeBasePtr(new mode::ModeMotorPosition()));
  addMode(mode::ModeBasePtr(new mode::ModeMotorVelocity()));
}

Configuration::Configuration(const Configuration & other)
{
  *this = other;
}

Configuration & Configuration::operator=(const Configuration & other)
{

  maxCommandAge_ = other.getMaxCommandAge();
  autoStageLastCommand_ = other.getAutoStageLastCommand();
  setReadingToNanOnDisconnect_ = other.getSetReadingToNanOnDisconnect();
  errorStateBehavior_ = other.getErrorStateBehavior();

  maxCurrent_ = other.getMaxCurrent();
  maxFreezeCurrent_ = other.getMaxFreezeCurrent();
  maxMotorVelocity_ = other.getMaxMotorVelocity();
  maxJointTorque_ = other.getMaxJointTorque();
  currentIntegratorSaturation_ = other.getCurrentIntegratorSaturation();
  jointTorqueIntegratorSaturation_ = other.getJointTorqueIntegratorSaturation();

  direction_ = other.getDirection();

  jointPositionLimitsSdk_ = other.getJointPositionLimitsSdk();
  jointPositionLimitsSoft_ = other.getJointPositionLimitsSoft();
  jointPositionLimitsHard_ = other.getJointPositionLimitsHard();

  for (const auto & jointPositionConfiguration : other.getJointPositionConfigurations()) {
    addJointPositionConfiguration(jointPositionConfiguration.first,
          jointPositionConfiguration.second);
  }

  imuEnable_ = other.getImuEnable();
  imuAccelerometerRange_ = other.getImuAccelerometerRange();
  imuGyroscopeRange_ = other.getImuGyroscopeRange();

  fanMode_ = other.getFanMode();
  fanIntensity_ = other.getFanIntensity();
  fanLowerTemperature_ = other.getFanLowerTemperature();
  fanUpperTemperature_ = other.getFanUpperTemperature();


  for (const auto & mode : other.getModes()) {
    if (mode.second->getPidGains()) {
      getMode(mode.first)->setPidGains(mode.second->getPidGains().value());
    }
  }

  goalStateEnumStartup_ = other.getGoalStateEnumStartup();
  goalStateEnumShutdown_ = other.getGoalStateEnumShutdown();

  gearJointVelocityFilterType_ = other.getGearJointVelocityFilterType();
  gearJointVelocityKfNoiseVariance_ = other.getGearJointVelocityKfNoiseVariance();
  gearJointVelocityKfLambda2_ = other.getGearJointVelocityKfLambda2();
  gearJointVelocityKfGamma_ = other.getGearJointVelocityKfGamma();
  gearJointVelocityEmaAlpha_ = other.getGearJointVelocityEmaAlpha();

  jointVelocityForAccelerationFilterType_ = other.getJointVelocityForAccelerationFilterType();
  jointVelocityForAccelerationKfNoiseVariance_ =
    other.getJointVelocityForAccelerationKfNoiseVariance();
  jointVelocityForAccelerationKfLambda2_ = other.getJointVelocityForAccelerationKfLambda2();
  jointVelocityForAccelerationKfGamma_ = other.getJointVelocityForAccelerationKfGamma();
  jointVelocityForAccelerationEmaAlpha_ = other.getJointVelocityForAccelerationEmaAlpha();

  jointAccelerationFilterType_ = other.getJointAccelerationFilterType();
  jointAccelerationKfNoiseVariance_ = other.getJointAccelerationKfNoiseVariance();
  jointAccelerationKfLambda2_ = other.getJointAccelerationKfLambda2();
  jointAccelerationKfGamma_ = other.getJointAccelerationKfGamma();
  jointAccelerationEmaAlpha_ = other.getJointAccelerationEmaAlpha();
  dgainFilterCutoffFrequency_ = other.getDGainFilterCutoffFrequency();
  fake_ = other.fake_;
  return *this;
}


void Configuration::setMaxCommandAge(const double maxCommandAge)
{
  if (maxCommandAge < 0) {
    MELO_ERROR("The max command age must be within [0.0, .inf) s.");
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  maxCommandAge_ = maxCommandAge;
}

double Configuration::getMaxCommandAge() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return maxCommandAge_;
}

void Configuration::setAutoStageLastCommand(const bool keepSendingLastCommand)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  autoStageLastCommand_ = keepSendingLastCommand;
}

bool Configuration::getAutoStageLastCommand() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return autoStageLastCommand_;
}

void Configuration::setSetReadingToNanOnDisconnect(const bool setReadingToNanOnDisconnect)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  setReadingToNanOnDisconnect_ = setReadingToNanOnDisconnect;
}

bool Configuration::getSetReadingToNanOnDisconnect() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return setReadingToNanOnDisconnect_;
}

void Configuration::setErrorStateBehavior(const uint16_t errorStateBehavior)
{
  if (errorStateBehavior > 1) {
    MELO_ERROR("The error state behavior must be 0 or 1.");
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  errorStateBehavior_ = errorStateBehavior;
}

std::optional<uint16_t> Configuration::getErrorStateBehavior() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return errorStateBehavior_;
}

void Configuration::setMaxCurrent(const double maxCurrent)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  maxCurrent_ = maxCurrent;
}

std::optional<double> Configuration::getMaxCurrent() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return maxCurrent_;
}

void Configuration::setMaxFreezeCurrent(const double current)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  maxFreezeCurrent_ = current;
}

std::optional<double> Configuration::getMaxFreezeCurrent() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return maxFreezeCurrent_;
}

void Configuration::setMaxMotorVelocity(const double maxMotorVelocity)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  maxMotorVelocity_ = maxMotorVelocity;
}

std::optional<double> Configuration::getMaxMotorVelocity() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return maxMotorVelocity_;
}

void Configuration::setMaxJointTorque(const double maxJointTorque)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  maxJointTorque_ = maxJointTorque;
}

std::optional<double> Configuration::getMaxJointTorque() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return maxJointTorque_;
}

void Configuration::setCurrentIntegratorSaturation(const double saturation)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  currentIntegratorSaturation_ = saturation;
}

std::optional<double> Configuration::getCurrentIntegratorSaturation() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return currentIntegratorSaturation_;
}

void Configuration::setJointTorqueIntegratorSaturation(const double saturation)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointTorqueIntegratorSaturation_ = saturation;
}

std::optional<double> Configuration::getJointTorqueIntegratorSaturation() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointTorqueIntegratorSaturation_;
}

void Configuration::setDirection(const int16_t direction)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  direction_ = direction;
}

std::optional<int16_t> Configuration::getDirection() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return direction_;
}

void Configuration::setJointPositionLimitsSdk(const common::Limits & limits)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointPositionLimitsSdk_ = limits;
}

std::optional<common::Limits> Configuration::getJointPositionLimitsSdk() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointPositionLimitsSdk_;
}

void Configuration::setJointPositionLimitsSoft(const common::Limits & limits)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointPositionLimitsSoft_ = limits;
}

std::optional<common::Limits> Configuration::getJointPositionLimitsSoft() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointPositionLimitsSoft_;
}

void Configuration::setJointPositionLimitsHard(const common::Limits & limits)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointPositionLimitsHard_ = limits;
}

std::optional<common::Limits> Configuration::getJointPositionLimitsHard() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointPositionLimitsHard_;
}

void Configuration::addJointPositionConfiguration(
  const std::string & jointPositionConfigurationName, double jointPositionConfigurationValue)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointPositionConfigurations_.insert({jointPositionConfigurationName,
      jointPositionConfigurationValue});
}

bool Configuration::getJointPositionConfigurationValue(
  const std::string & jointPositionConfigurationName,
  double & jointPositionConfigurationValue) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = jointPositionConfigurations_.find(jointPositionConfigurationName);
  if (it == jointPositionConfigurations_.end()) {
    return false;
  }
  jointPositionConfigurationValue = it->second;
  return true;
}

std::map<std::string, double> Configuration::getJointPositionConfigurations() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointPositionConfigurations_;
}

void Configuration::setImuEnable(const bool enable)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  imuEnable_ = enable;
}

std::optional<bool> Configuration::getImuEnable() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return imuEnable_;
}

void Configuration::setImuAccelerometerRange(const uint32_t range)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  imuAccelerometerRange_ = range;
}

std::optional<uint32_t> Configuration::getImuAccelerometerRange() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return imuAccelerometerRange_;
}

void Configuration::setImuGyroscopeRange(const uint32_t range)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  imuGyroscopeRange_ = range;
}

std::optional<uint32_t> Configuration::getImuGyroscopeRange() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return imuGyroscopeRange_;
}

void Configuration::setFanMode(const uint32_t mode)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  fanMode_ = mode;
}

std::optional<uint32_t> Configuration::getFanMode() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return fanMode_;
}

void Configuration::setFanIntensity(const uint32_t intensity)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  fanIntensity_ = intensity;
}

std::optional<uint32_t> Configuration::getFanIntensity() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return fanIntensity_;
}

void Configuration::setFanLowerTemperature(const float temperature)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  fanLowerTemperature_ = temperature;
}

std::optional<float> Configuration::getFanLowerTemperature() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return fanLowerTemperature_;
}

void Configuration::setFanUpperTemperature(const float temperature)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  fanUpperTemperature_ = temperature;
}

std::optional<float> Configuration::getFanUpperTemperature() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return fanUpperTemperature_;
}

void Configuration::setGearJointVelocityFilterType(const uint32_t type)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  gearJointVelocityFilterType_ = type;
}

std::optional<uint32_t> Configuration::getGearJointVelocityFilterType() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return gearJointVelocityFilterType_;
}

void Configuration::setGearJointVelocityKfNoiseVariance(const float variance)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  gearJointVelocityKfNoiseVariance_ = variance;
}

std::optional<float> Configuration::getGearJointVelocityKfNoiseVariance() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return gearJointVelocityKfNoiseVariance_;
}

void Configuration::setGearJointVelocityKfLambda2(const float lambda)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  gearJointVelocityKfLambda2_ = lambda;
}

std::optional<float> Configuration::getGearJointVelocityKfLambda2() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return gearJointVelocityKfLambda2_;
}

void Configuration::setGearJointVelocityKfGamma(const float gamma)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  gearJointVelocityKfGamma_ = gamma;
}

std::optional<float> Configuration::getGearJointVelocityKfGamma() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return gearJointVelocityKfGamma_;
}

void Configuration::setGearJointVelocityEmaAlpha(const float alpha)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  gearJointVelocityEmaAlpha_ = alpha;
}

std::optional<float> Configuration::getGearJointVelocityEmaAlpha() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return gearJointVelocityEmaAlpha_;
}

void Configuration::setJointVelocityForAccelerationFilterType(const uint32_t type)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointVelocityForAccelerationFilterType_ = type;
}

std::optional<uint32_t> Configuration::getJointVelocityForAccelerationFilterType() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointVelocityForAccelerationFilterType_;
}

void Configuration::setJointVelocityForAccelerationKfNoiseVariance(const float variance)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointVelocityForAccelerationKfNoiseVariance_ = variance;
}

std::optional<float> Configuration::getJointVelocityForAccelerationKfNoiseVariance() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointVelocityForAccelerationKfNoiseVariance_;
}

void Configuration::setJointVelocityForAccelerationKfLambda2(const float lambda)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointVelocityForAccelerationKfLambda2_ = lambda;
}

std::optional<float> Configuration::getJointVelocityForAccelerationKfLambda2() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointVelocityForAccelerationKfLambda2_;
}

void Configuration::setJointVelocityForAccelerationKfGamma(const float gamma)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointVelocityForAccelerationKfGamma_ = gamma;
}

std::optional<float> Configuration::getJointVelocityForAccelerationKfGamma() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointVelocityForAccelerationKfGamma_;
}

void Configuration::setJointVelocityForAccelerationEmaAlpha(const float alpha)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointVelocityForAccelerationEmaAlpha_ = alpha;
}

std::optional<float> Configuration::getJointVelocityForAccelerationEmaAlpha() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointVelocityForAccelerationEmaAlpha_;
}

void Configuration::setJointAccelerationFilterType(const uint32_t type)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointAccelerationFilterType_ = type;
}

std::optional<uint32_t> Configuration::getJointAccelerationFilterType() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointAccelerationFilterType_;
}

void Configuration::setJointAccelerationKfNoiseVariance(const float variance)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointAccelerationKfNoiseVariance_ = variance;
}

std::optional<float> Configuration::getJointAccelerationKfNoiseVariance() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointAccelerationKfNoiseVariance_;
}

void Configuration::setJointAccelerationKfLambda2(const float lambda)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointAccelerationKfLambda2_ = lambda;
}

std::optional<float> Configuration::getJointAccelerationKfLambda2() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointAccelerationKfLambda2_;
}

void Configuration::setJointAccelerationKfGamma(const float gamma)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointAccelerationKfGamma_ = gamma;
}

std::optional<float> Configuration::getJointAccelerationKfGamma() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointAccelerationKfGamma_;
}

void Configuration::setJointAccelerationEmaAlpha(const float alpha)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointAccelerationEmaAlpha_ = alpha;
}

std::optional<float> Configuration::getJointAccelerationEmaAlpha() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointAccelerationEmaAlpha_;
}

void Configuration::setDGainFilterCutoffFrequency(const float freq)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  dgainFilterCutoffFrequency_ = freq;
}

std::optional<float> Configuration::getDGainFilterCutoffFrequency() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return dgainFilterCutoffFrequency_;
}

mode::ModeBasePtr Configuration::getMode(const mode::ModeEnum modeEnum) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = modes_.find(modeEnum);


  if (it == modes_.end()) {
    MELO_ERROR_STREAM("Mode does not exist: " << modeEnum);
    return mode::ModeBasePtr();
  }

  return it->second;
}

std::map<mode::ModeEnum, mode::ModeBasePtr> Configuration::getModes() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return modes_;
}

void Configuration::setGoalStateEnumStartup(const fsm::StateEnum goalStateEnumStartup)
{
  switch (goalStateEnumStartup) {
    case fsm::StateEnum::ColdStart:
    case fsm::StateEnum::DeviceMissing:
    case fsm::StateEnum::Error:
    case fsm::StateEnum::Fatal:
    case fsm::StateEnum::MotorPreOp:
    case fsm::StateEnum::WarmStart:
      MELO_ERROR("Invalid startup goal state enum.");
      return;
    default:
      break;
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  goalStateEnumStartup_ = goalStateEnumStartup;
}

fsm::StateEnum Configuration::getGoalStateEnumStartup() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return goalStateEnumStartup_;
}

void Configuration::setGoalStateEnumShutdown(const fsm::StateEnum goalStateEnumShutdown)
{
  switch (goalStateEnumShutdown) {
    case fsm::StateEnum::ColdStart:
    case fsm::StateEnum::ControlOp:
    case fsm::StateEnum::DeviceMissing:
    case fsm::StateEnum::Error:
    case fsm::StateEnum::Fatal:
    case fsm::StateEnum::MotorPreOp:
    case fsm::StateEnum::WarmStart:
      MELO_ERROR("Invalid shutdown goal state enum.");
      return;
    default:
      break;
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  goalStateEnumShutdown_ = goalStateEnumShutdown;
}

fsm::StateEnum Configuration::getGoalStateEnumShutdown() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return goalStateEnumShutdown_;
}

void Configuration::setFake(const bool fake)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  fake_ = fake;
}

bool Configuration::getFake() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return fake_;
}

void Configuration::addMode(const mode::ModeBasePtr & mode)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  modes_.insert({mode->getModeEnum(), mode});
}


} // configuration
} // rsl_drive_sdk
