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
#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>

// rsl_drive_sdk

// soem_interface
#include <ethercat_sdk_master/EthercatDevice.hpp>


// rsl_drive_sdk ethercat
#include "rsl_drive_sdk/PdoTypeEnum.hpp"
#include "rsl_drive_sdk/Statusword.hpp"
#include "rsl_drive_sdk/StateExtended.hpp"
#include "rsl_drive_sdk/ReadingExtended.hpp"
#include "rsl_drive_sdk/common/Version.hpp"
#include "rsl_drive_sdk/common/ObjectDictionary.hpp"
#include "rsl_drive_sdk/common/BuildInfo.h"
#include "rsl_drive_sdk/common/FirmwareInfo.hpp"
#include "rsl_drive_sdk/common/Limits.hpp"
#include "rsl_drive_sdk/calibration/CalibrationState.hpp"
#include "rsl_drive_sdk/calibration/CalibrationModeEnum.hpp"
#include "rsl_drive_sdk/calibration/CalibrationTypeEnum.hpp"
#include "rsl_drive_sdk/calibration/parameter/Calibration.hpp"
#include "rsl_drive_sdk/configuration/DriveConfiguration.hpp"
#include "rsl_drive_sdk/fsm/StateMachine.hpp"
#include "rsl_drive_sdk/usings.hpp"


namespace rsl_drive_sdk
{

class DriveEthercatDevice : public ecat_master::EthercatDevice {
protected:
  using RecMutex = std::recursive_mutex;
  using RecLock = std::lock_guard<RecMutex>;

  static constexpr double rpmPerRadS_ = 60.0 / (2.0 * M_PI);

  static constexpr double milliScaling_ = 1000.0;
  static constexpr double kiloScaling_ = 0.001;

  static constexpr double timeScaling_ = milliScaling_;   // ms
  static constexpr double gainScaling_ = 1.0;
  static constexpr double temperatureScaling_ = 100.0;   // °cC (and offset)
  static constexpr double motorVoltageScaling_ = 100.0;   // cV
  static constexpr double motorCurrentScaling_ = 1.0;   // A
  static constexpr double motorPositionScaling_ = 1.0;   // rad
  static constexpr double motorVelocityScaling_ = kiloScaling_ * rpmPerRadS_; // krpm
  static constexpr double gearPositionScaling_ = 1.0;   // W
  static constexpr double gearVelocityScaling_ = 1.0;   // W
  static constexpr double jointPositionScaling_ = 1.0;   // rad
  static constexpr double jointVelocityScaling_ = rpmPerRadS_;   // rpm
  static constexpr double jointAccelerationScaling_ = rpmPerRadS_;   // rpm/s
  static constexpr double jointTorqueScaling_ = 1.0;   // Nm

  static constexpr double timeScalingInv_ = 1.0 / timeScaling_;
  static constexpr double gainScalingInv_ = 1.0 / gainScaling_;
  static constexpr double temperatureScalingInv_ = 1.0 / temperatureScaling_;
  static constexpr double motorVoltageScalingInv_ = 1.0 / motorVoltageScaling_;
  static constexpr double motorCurrentScalingInv_ = 1.0 / motorCurrentScaling_;
  static constexpr double motorPositionScalingInv_ = 1.0 / motorPositionScaling_;
  static constexpr double motorVelocityScalingInv_ = 1.0 / motorVelocityScaling_;
  static constexpr double gearPositionScalingInv_ = 1.0 / gearPositionScaling_;
  static constexpr double gearVelocityScalingInv_ = 1.0 / gearVelocityScaling_;
  static constexpr double jointPositionScalingInv_ = 1.0 / jointPositionScaling_;
  static constexpr double jointVelocityScalingInv_ = 1.0 / jointVelocityScaling_;
  static constexpr double jointAccelerationScalingInv_ = 1.0 / jointAccelerationScaling_;
  static constexpr double jointTorqueScalingInv_ = 1.0 / jointTorqueScaling_;

  static constexpr double temperatureOffset_ = -55.0;


  using PdoInfos = std::map<PdoTypeEnum, PdoInfo>;

  static std::map<rsl_drive_sdk::mode::ModeEnum, uint16_t> modeEnumToOdIndex_;

  PdoInfos pdoInfos_;
  std::atomic<PdoTypeEnum> pdoTypeEnum_;
  std::atomic<PdoTypeEnum> currentPdoTypeEnum_;

  mutable RecMutex readingMutex_;
  rsl_drive_sdk::ReadingExtended reading_;

  mutable RecMutex commandMutex_;
  rsl_drive_sdk::Command command_;

  mutable RecMutex controlwordIdMutex_;
  uint16_t controlwordId_ = 0;

  mutable RecMutex mutex_;

  bool isRtdlRunning_ = false;

  rsl_drive_sdk::configuration::Configuration config_;


    //! State machine of the device.
  fsm::StateMachine stateMachine_;

  void updateProcessReading();
  bool statuswordRequested_ = false;
  void requestAndSetStatusword();

  void setStatusword(Statusword & statusword);

  Statusword statusword_;

  std::multimap<int, ReadingCb, std::greater<int>> readingCbs_;
    //! Prioritized callbacks called when an error appeared.
  std::multimap<int, ErrorCb, std::greater<int>> errorCbs_;
    //! Prioritized callbacks called when an error disappeared.
  std::multimap<int, ErrorRecoveredCb, std::greater<int>> errorRecoveredCbs_;
    //! Prioritized callbacks called when an fatal appeared.
  std::multimap<int, FatalCb, std::greater<int>> fatalCbs_;
    //! Prioritized callbacks called when an fatal disappeared.
  std::multimap<int, FatalRecoveredCb, std::greater<int>> fatalRecoveredCbs_;
    //! Prioritized callbacks called when the device disconnected.
  std::multimap<int, DeviceDisconnectedCb, std::greater<int>> deviceDisconnectedCbs_;
    //! Prioritized callbacks called when the device reconnected.
  std::multimap<int, DeviceReconnectedCb, std::greater<int>> deviceReconnectedCbs_;

public:
  typedef std::shared_ptr<DriveEthercatDevice> SharedPtr;
  static SharedPtr deviceFromFile(
    const std::string & fileName, const std::string & name,
    const uint32_t address, const PdoTypeEnum pdoTypeEnum);
  DriveEthercatDevice(
    const uint32_t address, const std::string & name,
    const PdoTypeEnum pdoTypeEnum);
  virtual ~DriveEthercatDevice() = default;

  uint32_t getAddress() const;
  configuration::Configuration & getConfiguration() {return config_;}
  void applyConfiguration(const configuration::Configuration & config);
  bool startup() override;
  void updateRead() override;
  void updateWrite() override;
  void shutdown() override;
  void preShutdown() override;

  void stageFreeze();
  bool deviceIsMissing() const;

  void setState(const uint16_t state);
  bool waitForState(const uint16_t state);


  PdoTypeEnum getCurrentPdoTypeEnum() const;
  PdoInfo getCurrentPdoInfo() const override;
  bool receivesGearAndJointEncoderTicks() const;
  std::string getName() const override;

    //State machine.
  fsm::StateEnum getActiveStateEnum() const;
  fsm::StateEnum getGoalStateEnum() const;
  bool goalStateHasBeenReached() const;
  bool setFSMGoalState(
    const fsm::StateEnum goalStateEnum, const bool reachState,
    const double timeout, const double checkingFrequency);
  void clearGoalStateEnum();

    // Drive info.
  bool getDriveModel(std::string & model);
  bool getDriveInfoSerialNumber(std::string & serialNumber);
  bool setDriveInfoSerialNumber(const std::string & serialNumber);
  bool getDriveInfoName(std::string & name);
  bool setDriveInfoName(const std::string & name);
  bool getDriveInfoId(uint16_t & id);
  bool setDriveInfoId(const uint16_t id);
  bool getDriveInfoBootloaderVersion(rsl_drive_sdk::common::Version & bootloaderVersion);
  bool setDriveInfoBootloaderVersion(const rsl_drive_sdk::common::Version & bootloaderVersion);
  bool getDriveInfoFirmwareVersion(rsl_drive_sdk::common::Version & firmwareVersion);
  bool getDriveFirmwareInfo(rsl_drive_sdk::common::FirmwareInfo & firmwareInfo);
  bool getGearboxRatio(float & ratio);
  bool getBuildInfo(rsl_drive_sdk::common::BuildInfo & buildInfo);
  bool getDriveType(std::string & type);

    // Flash storage.
  bool eraseFlashStorage();
  bool resetFlashStorageSections(const uint16_t flashStorageSections);

    // Calibration.
  bool getCalibrationState(
    const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum,
    rsl_drive_sdk::calibration::CalibrationState & calibrationState);
  bool sendCalibrationGearAndJointEncoderHomingNewJointPosition(const double newJointPosition);
  bool startCalibration(const rsl_drive_sdk::calibration::CalibrationModeEnum calibrationModeEnum);
  bool calibrationIsRunning(bool & running);
  bool getCalibrationMotorEncoderOffset(
    const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum,
    rsl_drive_sdk::calibration::parameter::MotorEncoderOffset & motorEncoderOffset);
  bool getCalibrationMotorEncoderParameters(
    const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum,
    rsl_drive_sdk::calibration::parameter::MotorEncoderParameters & motorEncoderParameters);
  bool getCalibrationGearJointEncoderOffset(
    const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum,
    rsl_drive_sdk::calibration::parameter::GearJointEncoderOffset & gearJointEncoderOffset);
  bool setCalibrationGearAndJointEncoderHoming(
    const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum,
    const rsl_drive_sdk::calibration::parameter::GearAndJointEncoderHoming &
    gearAndJointEncoderHoming);
  bool getCalibrationGearJointEncoderOffsetConstant(int32_t & constant);
  bool setCalibrationGearJointEncoderOffsetConstant(const int32_t constant);
  bool getCalibrationGearAndJointEncoderHoming(
    const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum,
    rsl_drive_sdk::calibration::parameter::GearAndJointEncoderHoming & gearAndJointEncoderHoming);
  bool getCalibrationImuGyroscopeDcBias(
    const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum,
    rsl_drive_sdk::calibration::parameter::ImuGyroscopeDcBias & imuGyroscopeDcBias);
  bool setCalibrationSpringStiffness(
    const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum,
    const rsl_drive_sdk::calibration::parameter::SpringStiffness & springStiffness);
  bool getCalibrationSpringStiffness(
    const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum,
    rsl_drive_sdk::calibration::parameter::SpringStiffness & springStiffness);
  bool setCalibrationFrictionEstimation(
    const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum,
    const rsl_drive_sdk::calibration::parameter::FrictionEstimation & frictionEstimation);
  bool getCalibrationFrictionEstimation(
    const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum,
    rsl_drive_sdk::calibration::parameter::FrictionEstimation & frictionEstimation);
  bool resetCustomCalibrationsToFactory(
    const rsl_drive_sdk::calibration::CalibrationState calibrationState);
  bool writeFactoryCalibration();

    // Configuration.
  bool getMaxCurrent(double & maxCurrent);
  bool setMaxCurrent(const double maxCurrent);
  bool getMaxFreezeCurrent(double & current);
  bool setMaxFreezeCurrent(const double current);
  bool getMaxMotorVelocity(double & maxMotorVelocity);
  bool setMaxMotorVelocity(const double maxMotorVelocity);
  bool getMaxJointTorque(double & maxJointTorque);
  bool setMaxJointTorque(const double maxJointTorque);
  bool getCurrentIntegratorSaturation(double & saturation);
  bool setCurrentIntegratorSaturation(const double saturation);
  bool getJointTorqueIntegratorSaturation(double & saturation);
  bool setJointTorqueIntegratorSaturation(const double saturation);
  bool getDirection(int16_t & direction);
  bool setDirection(const int16_t direction);
  bool getJointPositionLimitsSoft(rsl_drive_sdk::common::Limits & limits);
  bool setJointPositionLimitsSoft(const rsl_drive_sdk::common::Limits & limits);
  bool getJointPositionLimitsHard(rsl_drive_sdk::common::Limits & limits);
  bool setJointPositionLimitsHard(const rsl_drive_sdk::common::Limits & limits);
  bool getControlGains(
    const rsl_drive_sdk::mode::ModeEnum mode,
    rsl_drive_sdk::mode::PidGainsF & pidGains);
  bool setControlGains(
    const rsl_drive_sdk::mode::ModeEnum mode,
    const rsl_drive_sdk::mode::PidGainsF & pidGains);
  bool getErrorStateBehavior(uint16_t & errorStateBehavior);
  bool setErrorStateBehavior(const uint16_t errorStateBehavior);
  bool getImuEnable(bool & enabled);
  bool setImuEnable(const bool enable);
  bool getImuAccelerometerRange(uint32_t & range);
  bool setImuAccelerometerRange(const uint32_t range);
  bool getImuGyroscopeRange(uint32_t & range);
  bool setImuGyroscopeRange(const uint32_t range);
  bool getFanMode(uint32_t & mode);
  bool setFanMode(const uint32_t mode);
  bool getFanIntensity(uint32_t & intensity);
  bool setFanIntensity(const uint32_t intensity);
  bool getFanLowerTemperature(float & temperature);
  bool setFanLowerTemperature(const float temperature);
  bool getFanUpperTemperature(float & temperature);
  bool setFanUpperTemperature(const float temperature);
  bool setBrakeMode(const bool mode);
  bool getBrakeMode(bool & mode);
  bool setBrakeDuty(const float d);
  bool getBrakeDuty(float & d);
  bool getGearJointVelocityFilterType(uint32_t & type);
  bool setGearJointVelocityFilterType(const uint32_t type);
  bool getGearJointVelocityKfNoiseVariance(float & variance);
  bool setGearJointVelocityKfNoiseVariance(const float variance);
  bool getGearJointVelocityKfLambda2(float & lambda);
  bool setGearJointVelocityKfLambda2(const float lambda);
  bool getGearJointVelocityKfGamma(float & gamma);
  bool setGearJointVelocityKfGamma(const float gamma);
  bool getGearJointVelocityIirAlpha(float & alpha);
  bool setGearJointVelocityIirAlpha(const float alpha);
  bool getGearJointVelocityEmaAlpha(float & alpha);
  bool setGearJointVelocityEmaAlpha(const float alpha);
  bool getJointVelocityForAccelerationFilterType(uint32_t & type);
  bool setJointVelocityForAccelerationFilterType(const uint32_t type);
  bool getJointVelocityForAccelerationKfNoiseVariance(float & variance);
  bool setJointVelocityForAccelerationKfNoiseVariance(const float variance);
  bool getJointVelocityForAccelerationKfLambda2(float & lambda);
  bool setJointVelocityForAccelerationKfLambda2(const float lambda);
  bool getJointVelocityForAccelerationKfGamma(float & gamma);
  bool setJointVelocityForAccelerationKfGamma(const float gamma);
  bool getJointVelocityForAccelerationIirAlpha(float & alpha);
  bool setJointVelocityForAccelerationIirAlpha(const float alpha);
  bool getJointVelocityForAccelerationEmaAlpha(float & alpha);
  bool setJointVelocityForAccelerationEmaAlpha(const float alpha);
  bool getJointAccelerationFilterType(uint32_t & type);
  bool setJointAccelerationFilterType(const uint32_t type);
  bool getJointAccelerationKfNoiseVariance(float & variance);
  bool setJointAccelerationKfNoiseVariance(const float variance);
  bool getJointAccelerationKfLambda2(float & lambda);
  bool setJointAccelerationKfLambda2(const float lambda);
  bool getJointAccelerationKfGamma(float & gamma);
  bool setJointAccelerationKfGamma(const float gamma);
  bool getJointAccelerationIirAlpha(float & alpha);
  bool setJointAccelerationIirAlpha(const float alpha);
  bool getJointAccelerationEmaAlpha(float & alpha);
  bool setJointAccelerationEmaAlpha(const float alpha);
  bool writeConfiguration();
  bool setDGainFilterCutoffFrequency(const float freq);
  bool getDGainFilterCutoffFrequency(float & freq);

    // Status.
  bool getStatuswordSdo(rsl_drive_sdk::Statusword & statusword);
  rsl_drive_sdk::Statusword getStatusword();

    // Control.
  void setControlword(const uint16_t controlwordId);
  void resetControlword();
  void setCommand(const rsl_drive_sdk::Command & command);
  void getReading(rsl_drive_sdk::ReadingExtended & reading);

    // RTDL (Real time data logging).
  bool getRtdlEnabled(bool & enabled);
  bool setRtdlEnable(const bool enable);
  bool setRtdlCommand(const uint16_t command);
  bool getRtdlStatus(uint16_t & status);
  bool getRtdlLoggingFrequency(uint16_t & frequency);
  bool setRtdlLoggingFrequency(const uint16_t frequency);
  bool getRtdlStreamingFrequency(uint16_t & frequency);
  bool setRtdlStreamingFrequency(const uint16_t frequency);
  bool getRtdlLastTimestamp(uint64_t & timestamp);

    // Data logging:
  bool clearLoggedData();
  bool refreshLoggedData();

  bool readOperatingTimes(uint64_t & timeTotal, uint64_t & timeActive);
  bool readJointTravel(double & jointTravel);


  bool getCalibrationTypeId(uint16_t & calibrationTypeId);
  bool setCalibrationTypeId(const uint16_t calibrationTypeId);

  bool getLockStatus(uint32_t & status);
  bool sendPassword(const std::string & password);

  bool getCalibrationTypeEnum(
    rsl_drive_sdk::calibration::CalibrationTypeEnum & calibrationTypeEnum);
  bool setCalibrationTypeEnum(
    const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum);

    /*!
    * Add a callback called when a reading has been received.
    * @param readingCb Callback.
    * @param priority Priority.
    */
  void addReadingCb(const ReadingCb & readingCb, const int priority = 0);

    /*!
    * Add a callback called when an error appeared.
    * @param readingCb Callback.
    * @param priority Priority.
    */
  void addErrorCb(const ErrorCb & errorCb, const int priority = 0);

    /*!
    * Add a callback called when an error disappeared.
    * @param readingCb Callback.
    * @param priority Priority.
    */
  void addErrorRecoveredCb(const ErrorRecoveredCb & errorRecoveredCb, const int priority = 0);

    /*!
    * Add a callback called when a fatal appeared.
    * @param readingCb Callback.
    * @param priority Priority.
    */
  void addFatalCb(const FatalCb & fatalCb, const int priority = 0);

    /*!
    * Add a callback called when a fatal disappeared.
    * @param readingCb Callback.
    * @param priority Priority.
    */
  void addFatalRecoveredCb(const FatalRecoveredCb & fatalRecoveredCb, const int priority = 0);

    /*!
    * Add a callback called when the device disconnected.
    * @param readingCb Callback.
    * @param priority Priority.
    */
  void addDeviceDisconnectedCb(
    const DeviceDisconnectedCb & deviceDisconnectedCb,
    const int priority = 0);

    /*!
    * Add a callback called when a the device reconnected.
    * @param readingCb Callback.
    * @param priority Priority.
    */
  void addDeviceReconnectedCb(
    const DeviceReconnectedCb & deviceReconnectedCb,
    const int priority = 0);

    /*!
    * Call the prioritized error callbacks.
    */
  void errorCb();

    /*!
    * Call the prioritized error recovered callbacks.
    */
  void errorRecoveredCb();

    /*!
    * Check if the device is in the error state.
    * @return True if the device is in the error state.
    */
  bool deviceIsInErrorState();

    /*!
    * Call the prioritized fatal callbacks.
    */
  void fatalCb();

    /*!
    * Call the prioritized fatal recovered callbacks.
    */
  void fatalRecoveredCb();

    /*!
    * Check if the device is in the fatal state.
    * @return True if the device is in the fatal state.
    */
  bool deviceIsInFatalState();

    /*!
    * Call the device disconnected callbacks.
    */
  void deviceDisconnectedCb();

    /*!
    * Call the device reconnected callbacks.
    */
  void deviceReconnectedCb();

    /*!
     * Check whether the joint position is within the soft limits.
     * @True if the device is within the soft limits.
     */
  bool isWithinJointPositionLimitsSoft() const;

    /*!
     * Check whether the joint position is within the hard limits.
     * @True if the device is within the hard limits.
     */
  bool isWithinJointPositionLimitsHard() const;

    //@}

protected:
  bool configurePdo(const PdoTypeEnum pdoTypeEnum);


  template<typename T>
  bool read_and_apply_configuration_object(
    std::function<std::optional<T>(void)> config_getter, std::function<void(T)> config_setter,
    std::function<bool(T &)> drive_getter, std::function<bool(T)> drive_setter)
  {
    bool success = false;
    if(config_getter().isSet()) {
      success &= drive_setter(config_getter());
    } else {
      T val;
      success &= drive_getter(val);
      config_setter(val);
    }

    return success;
  }

    // custom handling of string SDOs
  bool sendSdoReadStringCustom(const uint16_t index, std::string & value);
  bool sendSdoWriteStringCustom(const uint16_t index, const std::string & value);
};


} // rsl_drive_sdk_ethercat
