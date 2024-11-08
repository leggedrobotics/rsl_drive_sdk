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
#include <climits>
#include <cstdint>
#include <mutex>
#include <string>
#include <optional>

// yaml tools
#include <yaml-cpp/yaml.h>

// rsl_drive_sdk
#include "rsl_drive_sdk/common/Limits.hpp"
#include "message_logger/message_logger.hpp"
#include "rsl_drive_sdk/fsm/StateEnum.hpp"
#include "rsl_drive_sdk/mode/ModeBase.hpp"


namespace rsl_drive_sdk
{
namespace configuration
{


class Configuration
{
protected:
  //! Mutex to protect this class from simultaneous access.
  mutable std::recursive_mutex mutex_;

  /*!
   * If the command's age exceeds this duration, it is not sent.
   * Usage:   SDK
   * Unit:    s
   * Range:   [0.0, .inf)
   * Default: .inf
   */
  double maxCommandAge_ = std::numeric_limits<double>::infinity();

  /*!
   * The commands are sent continuously to the Drive. If the parameter is true,
   * the last command is automatically staged.
   * Usage:   SDK
   * Unit:    -
   * Range:   {true, false}
   * Default: true
   */
  bool autoStageLastCommand_ = true;

  /*!
   * If this parameter is true, the reading values are set to NaN when a drive
   * disconnects. Otherwise the last values which were received are kept.
   * Usage:   SDK
   * Unit:    -
   * Range:   {true, false}
   * Default: true
   */
  bool setReadingToNanOnDisconnect_ = true;

  /*!
   * This value defines the Drive's behavior in the Error state.
   * 0 = freeze the motor
   * 1 = disable the motor
   * Usage:   Firmware
   * Unit:    -
   * Range:   {0, 1}
   * Default: Value stored on device.
   */
  std::optional<uint16_t> errorStateBehavior_;

  /*!
   * Maximal current the motor may apply.
   * Used in all control modes.
   * Usage:   Firmware
   * Unit:    A
   * Range:   > 0.0
   * Default: Value stored on device.
   */
  std::optional<double> maxCurrent_;

  /*!
   * Sets the current limit for the freeze controller.
   * Usage:   Firmware
   * Unit:    A
   * Range:   > 0.0
   * Default: Value stored on device.
   */
  std::optional<double> maxFreezeCurrent_;

  /*!
   * Maximal velocity of the motor.
   * Only used in MotorVelocity control mode.
   * Usage:   Firmware
   * Unit:    rad/s
   * Range:   > 0.0
   * Default: Value stored on device.
   */
  std::optional<double> maxMotorVelocity_;

  /*!
   * Maximal joint torque the actuator may apply.
   * Only used in series elastic actuator control modes.
   * Usage:   Firmware
   * Unit:    Nm
   * Range:   > 0.0
   * Default: Value stored on device.
   */
  std::optional<double> maxJointTorque_;

  /*!
   * Integrator saturation the current controller is limited to.
   * Usage:   Firmware
   * Unit:    A
   * Range:   > 0.0
   * Default: Value stored on device.
   */
  std::optional<double> currentIntegratorSaturation_;

  /*!
   * Integrator saturation the joint torque controller is limited to.
   * Usage:   Firmware
   * Unit:    Nm
   * Range:   > 0.0
   * Default: Value stored on device.
   */
  std::optional<double> jointTorqueIntegratorSaturation_;

  /*!
   * This value defines the Drive's direction of rotation. If changed, the zero gear and
   * joint zero positions remain the same, but position, velocity, torque and current
   * values flip signs.
   * -1 = clockwise
   *  1 = counter-clockwise
   * Usage:   Firmware
   * Unit:    -
   * Range:   {-1, 1}
   * Default: Value stored on device.
   */
  std::optional<int16_t> direction_;

  /*!
   * Minimal/maximal value of the SDK joint position limits. If violated, user
   * defined error callback is called. The Drive is not affected by this limits.
   * If both values are set to 0.0, the limits are open.
   * Usage:   SDK
   * Unit:    rad
   * Range:   (-.inf, .inf)
   * Default: 0.0
   */
  std::optional<common::Limits> jointPositionLimitsSdk_;

  /*!
   * Minimal/maximal value of the soft joint position limits. If violated,
   * the device switches to the Error state (freeze resp. disable the motor).
   * If both values are set to 0.0, the limits are open.
   * Usage:   Firmware
   * Unit:    rad
   * Range:   (-.inf, .inf)
   * Default: Values stored on device.
   */
  std::optional<common::Limits> jointPositionLimitsSoft_;

  /*!
   * Minimal/maximal value of the hard joint position limits. If violated,
   * the device switches to the Fatal state (disable the motor).
   * If both values are set to 0.0, the limits are open.
   * Usage:   Firmware
   * Unit:    rad
   * Range:   (-.inf, .inf)
   * Default: Values stored on device.
   */
  std::optional<common::Limits> jointPositionLimitsHard_;

  /*!
   * List containing the joint position configurations for this device.
   * Usage:   SDK
   * Unit:    -
   * Range:   -
   * Default: []
   */
  std::map<std::string, double> jointPositionConfigurations_;

  /*!
   * Enable/disable the IMU.
   * Usage:   Firmware
   * Unit:    -
   * Range:   {true, false}
   * Default: Value stored on device.
   */
  std::optional<bool> imuEnable_;

  /*!
   * Sets the range of the IMU's accelerometer.
   * 0 = +/- 2g
   * 1 = +/- 4g
   * 2 = +/- 8g
   * 3 = +/- 16g
   * Usage:   Firmware
   * Unit:    -
   * Range:   {0, 1, 2, 3}
   * Default: Value stored on device.
   */
  std::optional<uint32_t> imuAccelerometerRange_;

  /*!
   * Sets the range of the IMU's gyroscope.
   * 0 = +/- 250dps
   * 1 = +/- 500dps
   * 2 = +/- 1000dps
   * 3 = +/- 2000dps
   * Usage:   Firmware
   * Unit:    -
   * Range:   {0, 1, 2, 3}
   * Default: Value stored on device.
   */
  std::optional<uint32_t> imuGyroscopeRange_;

  /*!
   * Sets the fan mode.
   * 0 = OFF
   * 1 = Auto
   * 2 = Manual
   * Usage:   Firmware
   * Unit:    -
   * Range:   {0, 1, 2}
   * Default: Value stored on device.
   */
  std::optional<uint32_t> fanMode_;

  /*!
   * Sets the fan intensity.
   * 0 = OFF
   * 1 = Minimal intensity
   * 10= Maximum intensity
   * Usage:   Firmware
   * Unit:    -
   * Range:   {0, ..., 10}
   * Default: Value stored on device.
   */
  std::optional<uint32_t> fanIntensity_;

  /*!
   * Sets the fan lower temperature, this and the upper temperature are used
   * in the auto mode to calculate the intensity with linear interpolation.
   * Usage:   Firmware
   * Unit:    Degrees Celsius
   * Default: Value stored on device.
   */
  std::optional<float> fanLowerTemperature_;

  /*!
   * Sets the fan upper temperature, this and the lower temperature are used
   * in the auto mode to calculate the intensity with linear interpolation.
   * Usage:   Firmware
   * Unit:    Degrees Celsius
   * Default: Value stored on device.
   */
  std::optional<float> fanUpperTemperature_;

  /*!
   * Choose filter type for estimation of gear and joint velocity.
   * Usage:   Firmware
   * Unit:    -
   * Range:   {none, KF, EMA}
   * Default: Value stored on device.
   */
  std::optional<uint32_t> gearJointVelocityFilterType_;

  /*!
   * Sets noise variance R value used by the gear/joint velocity KF filter.
   * Usage:   Firmware
   * Unit:    -
   * Range:   {0.0, .inf}
   * Default: Value stored on device.
   */
  std::optional<float> gearJointVelocityKfNoiseVariance_;

  /*!
   * Sets lmabda^2 value used by the gear/joint velocity KF filter.
   * Usage:   Firmware
   * Unit:    -
   * Range:   {0.0, .inf}
   * Default: Value stored on device.
   */
  std::optional<float> gearJointVelocityKfLambda2_;

  /*!
   * Sets gamma value used by the gear/joint velocity KF filter.
   * Usage:   Firmware
   * Unit:    -
   * Range:   {0.0, .inf}
   * Default: Value stored on device.
   */
  std::optional<float> gearJointVelocityKfGamma_;

  /*!
   * Sets alpha value used by the gear/joint velocity EMA filter.
   * Usage:   Firmware
   * Unit:    -
   * Range:   {0.0, .inf}
   * Default: Value stored on device.
   */
  std::optional<float> gearJointVelocityEmaAlpha_;

  /*!
   * Choose filter type for estimation of joint velocity for acceleration.
   * Usage:   Firmware
   * Unit:    -
   * Range:   {none, KF, EMA}
   * Default: Value stored on device.
   */
  std::optional<uint32_t> jointVelocityForAccelerationFilterType_;

  /*!
   * Sets noise variance R value used by the joint velocity for acceleration KF filter.
   * Usage:   Firmware
   * Unit:    -
   * Range:   {0.0, .inf}
   * Default: Value stored on device.
   */
  std::optional<float> jointVelocityForAccelerationKfNoiseVariance_;

  /*!
   * Sets lmabda^2 value used by the joint velocity for acceleration KF filter.
   * Usage:   Firmware
   * Unit:    -
   * Range:   {0.0, .inf}
   * Default: Value stored on device.
   */
  std::optional<float> jointVelocityForAccelerationKfLambda2_;

  /*!
   * Sets gamma value used by the joint velocity for acceleration KF filter.
   * Usage:   Firmware
   * Unit:    -
   * Range:   {0.0, .inf}
   * Default: Value stored on device.
   */
  std::optional<float> jointVelocityForAccelerationKfGamma_;

  /*!
   * Sets alpha value used by the joint velocity for acceleration EMA filter.
   * Usage:   Firmware
   * Unit:    -
   * Range:   {0.0, .inf}
   * Default: Value stored on device.
   */
  std::optional<float> jointVelocityForAccelerationEmaAlpha_;

  /*!
   * Choose filter type for estimation of joint acceleration.
   * Usage:   Firmware
   * Unit:    -
   * Range:   {none, KF, EMA}
   * Default: Value stored on device.
   */
  std::optional<uint32_t> jointAccelerationFilterType_;

  /*!
   * Sets noise variance R value used by the joint acceleration KF filter.
   * Usage:   Firmware
   * Unit:    -
   * Range:   {0.0, .inf}
   * Default: Value stored on device.
   */
  std::optional<float> jointAccelerationKfNoiseVariance_;

  /*!
   * Sets lmabda^2 value used by the joint acceleration KF filter.
   * Usage:   Firmware
   * Unit:    -
   * Range:   {0.0, .inf}
   * Default: Value stored on device.
   */
  std::optional<float> jointAccelerationKfLambda2_;

  /*!
   * Sets gamma value used by the joint acceleration KF filter.
   * Usage:   Firmware
   * Unit:    -
   * Range:   {0.0, .inf}
   * Default: Value stored on device.
   */
  std::optional<float> jointAccelerationKfGamma_;

  /*!
   * Sets alpha value used by the joint acceleration KF filter.
   * Usage:   Firmware
   * Unit:    -
   * Range:   {0.0, .inf}
   * Default: Value stored on device.
   */
  std::optional<float> jointAccelerationEmaAlpha_;

  /*!
   * List of control modes containing the gains.
   * Usage:   Firmware
   * Unit:    -
   * Range:   -
   * Default: Complete list, but no gains set.
   */
  std::map<mode::ModeEnum, mode::ModeBasePtr> modes_;


  std::optional<float> dgainFilterCutoffFrequency_;

  /*!
   * Goal FSM states which Drive tries to  transition to when the SDK is started up
   * or shut down.
   * Note that in case of a shutdown, the Drive is not allowed to remain in the
   * ControlOp state, as losing the communication results in a transition to the Error state.
   * Calibrate = Go to the Calibrate state.
   * Configure = Go to the Configure state.
   * ControlOp = Go to the ControlOp state (startup only).
   * MotorOp   = Go to the MotorOp state.
   * NA        = Remain in the current state. If the current state is ControlOp and a shutdown is
   *             requested, the device will go to MotorOp.
   * Standby   = Go to the Standby state.
   * Usage:   SDK
   * Unit:    -
   * Range:   {Calibrate, Configure, ControlOp, MotorOp, NA, Standby}
   * Default: NA
   */
  fsm::StateEnum goalStateEnumStartup_ = fsm::StateEnum::Configure;
  fsm::StateEnum goalStateEnumShutdown_ = fsm::StateEnum::Configure;

  /**
   * @brief fake_ true indicates this is a fake rsl_drive_sdk that doesnt get added to a communication manager
   */
  bool fake_;

public:
  Configuration();
  Configuration(const Configuration & other);
  virtual ~Configuration() = default;

  Configuration & operator=(const Configuration & other);

  void fromFile(const std::string & path);
  void fromYamlNode(const YAML::Node & yamlNode);

  void setMaxCommandAge(const double maxCommandAge);
  double getMaxCommandAge() const;

  void setAutoStageLastCommand(const bool keepSendingLastCommand);
  bool getAutoStageLastCommand() const;

  void setSetReadingToNanOnDisconnect(const bool setReadingToNanOnDisconnect);
  bool getSetReadingToNanOnDisconnect() const;

  void setErrorStateBehavior(const uint16_t errorStateBehavior);
  std::optional<uint16_t> getErrorStateBehavior() const;

  void setMaxCurrent(const double maxCurrent);
  std::optional<double> getMaxCurrent() const;

  void setMaxFreezeCurrent(const double current);
  std::optional<double> getMaxFreezeCurrent() const;

  void setMaxMotorVelocity(const double maxMotorVelocity);
  std::optional<double> getMaxMotorVelocity() const;

  void setMaxJointTorque(const double maxJointTorque);
  std::optional<double> getMaxJointTorque() const;

  void setCurrentIntegratorSaturation(const double saturation);
  std::optional<double> getCurrentIntegratorSaturation() const;

  void setJointTorqueIntegratorSaturation(const double saturation);
  std::optional<double> getJointTorqueIntegratorSaturation() const;

  void setDirection(const int16_t direction);
  std::optional<int16_t> getDirection() const;

  void setJointPositionLimitsSdk(const common::Limits & limits);
  std::optional<common::Limits> getJointPositionLimitsSdk() const;

  void setJointPositionLimitsSoft(const common::Limits & limits);
  std::optional<common::Limits> getJointPositionLimitsSoft() const;

  void setJointPositionLimitsHard(const common::Limits & limits);
  std::optional<common::Limits> getJointPositionLimitsHard() const;

  void addJointPositionConfiguration(
    const std::string & jointPositionConfigurationName,
    const double jointPositionConfigurationValue);
  bool getJointPositionConfigurationValue(
    const std::string & jointPositionConfigurationName,
    double & jointPositionConfigurationValue) const;
  std::map<std::string, double> getJointPositionConfigurations() const;

  void setImuEnable(const bool enable);
  std::optional<bool> getImuEnable() const;

  void setImuAccelerometerRange(const uint32_t range);
  std::optional<uint32_t> getImuAccelerometerRange() const;

  void setImuGyroscopeRange(const uint32_t range);
  std::optional<uint32_t> getImuGyroscopeRange() const;

  void setFanMode(const uint32_t mode);
  std::optional<uint32_t> getFanMode() const;

  void setFanIntensity(const uint32_t intensity);
  std::optional<uint32_t> getFanIntensity() const;

  void setFanLowerTemperature(const float temperature);
  std::optional<float> getFanLowerTemperature() const;

  void setFanUpperTemperature(const float temperature);
  std::optional<float> getFanUpperTemperature() const;

  void setGearJointVelocityFilterType(const uint32_t type);
  std::optional<uint32_t> getGearJointVelocityFilterType() const;

  void setGearJointVelocityKfNoiseVariance(const float variance);
  std::optional<float> getGearJointVelocityKfNoiseVariance() const;

  void setGearJointVelocityKfLambda2(const float lambda);
  std::optional<float> getGearJointVelocityKfLambda2() const;

  void setGearJointVelocityKfGamma(const float gamma);
  std::optional<float> getGearJointVelocityKfGamma() const;

  void setGearJointVelocityEmaAlpha(const float alpha);
  std::optional<float> getGearJointVelocityEmaAlpha() const;

  void setJointVelocityForAccelerationFilterType(const uint32_t type);
  std::optional<uint32_t> getJointVelocityForAccelerationFilterType() const;

  void setJointVelocityForAccelerationKfNoiseVariance(const float variance);
  std::optional<float> getJointVelocityForAccelerationKfNoiseVariance() const;

  void setJointVelocityForAccelerationKfLambda2(const float lambda);
  std::optional<float> getJointVelocityForAccelerationKfLambda2() const;

  void setJointVelocityForAccelerationKfGamma(const float gamma);
  std::optional<float> getJointVelocityForAccelerationKfGamma() const;

  void setJointVelocityForAccelerationEmaAlpha(const float alpha);
  std::optional<float> getJointVelocityForAccelerationEmaAlpha() const;

  void setJointAccelerationFilterType(const uint32_t type);
  std::optional<uint32_t> getJointAccelerationFilterType() const;

  void setJointAccelerationKfNoiseVariance(const float variance);
  std::optional<float> getJointAccelerationKfNoiseVariance() const;

  void setJointAccelerationKfLambda2(const float lambda);
  std::optional<float> getJointAccelerationKfLambda2() const;

  void setJointAccelerationKfGamma(const float gamma);
  std::optional<float> getJointAccelerationKfGamma() const;

  void setJointAccelerationEmaAlpha(const float alpha);
  std::optional<float> getJointAccelerationEmaAlpha() const;

  void setDGainFilterCutoffFrequency(const float freq);
  std::optional<float> getDGainFilterCutoffFrequency() const;

  mode::ModeBasePtr getMode(const mode::ModeEnum modeEnum) const;
  std::map<mode::ModeEnum, mode::ModeBasePtr> getModes() const;

  void setGoalStateEnumStartup(const fsm::StateEnum goalStateEnumStartup);
  fsm::StateEnum getGoalStateEnumStartup() const;

  void setGoalStateEnumShutdown(const fsm::StateEnum goalStateEnumShutdown);
  fsm::StateEnum getGoalStateEnumShutdown() const;

  void setFake(const bool fake);
  bool getFake() const;

protected:
  void addMode(const mode::ModeBasePtr & mode);
};


} // configuration
} // rsl_drive_sdk
