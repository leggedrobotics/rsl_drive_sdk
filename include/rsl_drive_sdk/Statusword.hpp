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
#include <mutex>
#include <vector>

// rsl_drive_sdk
#include "message_logger/message_logger.hpp"
#include "rsl_drive_sdk/mode/ModeEnum.hpp"
#include "rsl_drive_sdk/fsm/StateEnum.hpp"


namespace rsl_drive_sdk
{


class Statusword
{
public:
  struct DataBits // See drive firmware: motordrive_def.h
  {
    uint32_t stateId_ : 4; // FSM Status: Bits 0-3
    uint32_t modeId_ : 4; // Operation Mode: 4-7

    uint32_t warningOvertemperatureBridge_ : 1;
    uint32_t warningOvertemperatureStator_ : 1;
    uint32_t warningOvertemperatureCpu_ : 1;
    uint32_t warningEncoderOutlierMotor_ : 1;
    uint32_t warningEncoderOutlierGear_ : 1;
    uint32_t warningEncoderOutlierJoint_ : 1;

    uint32_t errorInvalidJointTorque_ : 1;
    uint32_t errorPdoTimeout_ : 1;

    uint32_t fatalOvertemperatureBridge_ : 1;
    uint32_t fatalOvertemperatureStator_ : 1;
    uint32_t fatalOvertemperatureCpu_ : 1;

    uint32_t error_direct_drive_elec_zero_offset_invalid : 1;
    uint32_t error_max_velocity_exceeded : 1;


    uint32_t errorJointPositionLimitsSoft_ : 1;
    uint32_t errorJointPositionLimitsHard_ : 1;

    uint32_t warningIncompleteCalibration_ : 1;

    uint32_t warningEncoderCrcGear_ : 1;
    uint32_t warningEncoderCrcJoint_ : 1;

    uint32_t fatalMotorEncoder_ : 1;
    uint32_t fatalCurrentSensor_ : 1;

    uint32_t fatalOvervoltage_ : 1;
    uint32_t fatalUndervoltage_ : 1;

    uint32_t unused_ : 1;

    uint32_t fatal_joint_motor_encoder : 1;
  };

  union Data{
    DataBits bits_;
    uint32_t all_ = 0;

    Data();
    Data(const uint32_t data);

    bool operator==(const Data & other);
    bool operator!=(const Data & other);
  };

protected:
  using TimePoint = std::chrono::system_clock::time_point;
  using Duration = std::chrono::duration<double>;

  mutable std::recursive_mutex mutex_;
  TimePoint stamp_;
  Data data_;

public:
  Statusword();
  Statusword(const Statusword & statusword);
  explicit Statusword(const uint32_t data);
  virtual ~Statusword();

  Statusword & operator=(const Statusword & statusword);

  bool isEmpty() const;

  double getAge() const;
  TimePoint getStamp() const;

  void setData(const uint32_t data);
  uint32_t getData() const;

  Data & getDataField();

  fsm::StateEnum getStateEnum() const;
  void setStateEnum(const fsm::StateEnum stateEnum);

  mode::ModeEnum getModeEnum() const;
  void setModeEnum(const mode::ModeEnum modeEnum);

  void getMessages(
    std::vector<std::string> & infos,
    std::vector<std::string> & warnings,
    std::vector<std::string> & errors,
    std::vector<std::string> & fatals) const;
  void getMessagesDiff(
    Statusword & previousStatusword,
    std::vector<std::string> & infos,
    std::vector<std::string> & warnings,
    std::vector<std::string> & errors,
    std::vector<std::string> & fatals) const;

  bool hasWarningHighTemperatureBridge() const;
  bool hasWarningHighTemperatureStator() const;
  bool hasWarningHighTemperatureCpu() const;
  bool hasWarningEncoderOutlierMotor() const;
  bool hasWarningEncoderOutlierGear() const;
  bool hasWarningEncoderOutlierJoint() const;
  bool hasErrorInvalidJointTorque() const;
  bool hasErrorPdoTimeout() const;
  bool hasFatalOvertemperatureBridge() const;
  bool hasFatalOvertemperatureStator() const;
  bool hasFatalOvertemperatureCpu() const;
  bool hasErrorInvalidElecZeroOffset() const;
  bool hasErrorMaxVelocityExceeded() const;
  bool hasErrorJointPositionLimitsSoft() const;
  bool hasFatalJointPositionLimitsHard() const;
  bool hasWarningIncompleteCalibration() const;
  bool hasWarningEncoderCrcGear() const;
  bool hasWarningEncoderCrcJoint() const;
  bool hasFatalMotorEncoder() const;
  bool hasFatalCurrentSensor() const;
  bool hasFatalOvervoltage() const;
  bool hasFatalUndervoltage() const;
  bool hasFatalJointMotorEncoder() const;
};

std::ostream & operator<<(std::ostream & os, const Statusword & statusword);


} // rsl_drive_sdk
