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
#include "rsl_drive_sdk/thread_sleep.hpp"

// rsl_drive_sdk ethercat
#include "rsl_drive_sdk/Drive.hpp"
#include "rsl_drive_sdk/PDO/RxPdo.hpp"
#include "rsl_drive_sdk/PDO/TxPdoA.hpp"
#include "rsl_drive_sdk/PDO/TxPdoB.hpp"
#include "rsl_drive_sdk/PDO/TxPdoC.hpp"
#include "rsl_drive_sdk/PDO/TxPdoD.hpp"
#include "rsl_drive_sdk/PDO/TxPdoE.hpp"
#include "rsl_drive_sdk/common/ObjectDictionary.hpp"
#include "rsl_drive_sdk/configuration/DriveConfigurationParser.hpp"
#include <soem_rsl/ethercattype.h>
namespace rsl_drive_sdk
{


std::map<rsl_drive_sdk::mode::ModeEnum, uint16_t> DriveEthercatDevice::modeEnumToOdIndex_ = {
  {rsl_drive_sdk::mode::ModeEnum::Current, OD_GAINS_CURRENT_CTRL_ID},
  {rsl_drive_sdk::mode::ModeEnum::MotorPosition, OD_GAINS_MOTOR_POSITION_CTRL_ID},
  {rsl_drive_sdk::mode::ModeEnum::MotorVelocity, OD_GAINS_MOTOR_VELOCITY_CTRL_ID},
  {rsl_drive_sdk::mode::ModeEnum::GearPosition, OD_GAINS_GEAR_POSITION_CTRL_ID},
  {rsl_drive_sdk::mode::ModeEnum::GearVelocity, OD_GAINS_GEAR_VELOCITY_CTRL_ID},
  {rsl_drive_sdk::mode::ModeEnum::JointPosition, OD_GAINS_JOINT_POSITION_CTRL_ID},
  {rsl_drive_sdk::mode::ModeEnum::JointPositionTorque, OD_GAINS_JOINT_POSITION_CTRL_ID},
  {rsl_drive_sdk::mode::ModeEnum::JointVelocity, OD_GAINS_JOINT_VELOCITY_CTRL_ID},
  {rsl_drive_sdk::mode::ModeEnum::JointTorque, OD_GAINS_JOINT_TORQUE_CTRL_ID},
  {rsl_drive_sdk::mode::ModeEnum::JointPositionVelocity, OD_GAINS_JOINT_POSITION_VELOCITY_CTRL_ID},
  {rsl_drive_sdk::mode::ModeEnum::JointPositionVelocityTorque,
    OD_GAINS_JOINT_POSITION_VELOCITY_TORQUE_CTRL_ID},
  {rsl_drive_sdk::mode::ModeEnum::JointPositionVelocityTorquePidGains,
    OD_GAINS_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS_CTRL_ID}
};

DriveEthercatDevice::SharedPtr DriveEthercatDevice::deviceFromFile(
  const std::string & fileName,
  const std::string & name, const uint32_t address, const PdoTypeEnum pdoTypeEnum)
{
  auto rsl_drive_sdk = std::make_shared<DriveEthercatDevice>(address, name, pdoTypeEnum);
  const auto configuration = DriveConfigurationParser::fromFile(fileName);
  rsl_drive_sdk->applyConfiguration(configuration);
  return rsl_drive_sdk;
}

DriveEthercatDevice::DriveEthercatDevice(
  const uint32_t address, const std::string & name,
  const PdoTypeEnum pdoTypeEnum)
:pdoTypeEnum_(pdoTypeEnum),
  currentPdoTypeEnum_(PdoTypeEnum::NA),
  stateMachine_(*this)
{
  address_ = address;
  name_ = name;

  command_.setModeEnum(mode::ModeEnum::Disable);
  PdoInfo pdoInfo;

  pdoInfo.rxPdoId_ = OD_DSP402_RX_PDO_SID_VAL_A;
  pdoInfo.txPdoId_ = OD_DSP402_TX_PDO_SID_VAL_A;
  pdoInfo.rxPdoSize_ = sizeof(RxPdo);
  pdoInfo.txPdoSize_ = sizeof(TxPdoA);
  pdoInfo.moduleId_ = 0x00119800;
  pdoInfos_.insert({PdoTypeEnum::A, pdoInfo});

  pdoInfo.rxPdoId_ = OD_DSP402_RX_PDO_SID_VAL_B;
  pdoInfo.txPdoId_ = OD_DSP402_TX_PDO_SID_VAL_B;
  pdoInfo.rxPdoSize_ = sizeof(RxPdo);
  pdoInfo.txPdoSize_ = sizeof(TxPdoB);
  pdoInfo.moduleId_ = 0x00219800;
  pdoInfos_.insert({PdoTypeEnum::B, pdoInfo});

  pdoInfo.rxPdoId_ = OD_DSP402_RX_PDO_SID_VAL_C;
  pdoInfo.txPdoId_ = OD_DSP402_TX_PDO_SID_VAL_C;
  pdoInfo.rxPdoSize_ = sizeof(RxPdo);
  pdoInfo.txPdoSize_ = sizeof(TxPdoC);
  pdoInfo.moduleId_ = 0x00319800;
  pdoInfos_.insert({PdoTypeEnum::C, pdoInfo});

  pdoInfo.rxPdoId_ = OD_DSP402_RX_PDO_SID_VAL_D;
  pdoInfo.txPdoId_ = OD_DSP402_TX_PDO_SID_VAL_D;
  pdoInfo.rxPdoSize_ = sizeof(RxPdo);
  pdoInfo.txPdoSize_ = sizeof(TxPdoD);
  pdoInfo.moduleId_ = 0x00419800;
  pdoInfos_.insert({PdoTypeEnum::D, pdoInfo});

    //Note: E Type is special as it uses the D type rx pdo
  pdoInfo.rxPdoId_ = OD_DSP402_RX_PDO_SID_VAL_D;
  pdoInfo.txPdoId_ = OD_DSP402_TX_PDO_SID_VAL_E;
  pdoInfo.rxPdoSize_ = sizeof(RxPdo);
  pdoInfo.txPdoSize_ = sizeof(TxPdoE);
  pdoInfo.moduleId_ = 0x00519800;
  pdoInfos_.insert({PdoTypeEnum::E, pdoInfo});
}

uint32_t DriveEthercatDevice::getAddress() const
{
  return address_;
}


void DriveEthercatDevice::applyConfiguration(const configuration::Configuration & config)
{
  MELO_DEBUG("Applying configuration");
  config_ = config;
}


std::string DriveEthercatDevice::getName() const
{
  return name_;
}


fsm::StateEnum DriveEthercatDevice::getActiveStateEnum() const
{
  return stateMachine_.getActiveStateEnum();
}

fsm::StateEnum DriveEthercatDevice::getGoalStateEnum() const
{
  return stateMachine_.getGoalStateEnum();
}

bool DriveEthercatDevice::goalStateHasBeenReached() const
{
  return stateMachine_.goalStateHasBeenReached();
}
bool DriveEthercatDevice::setFSMGoalState(
  const fsm::StateEnum goalStateEnum,
  const bool reachState,
  const double timeout,
  const double checkingFrequency)
{
  stateMachine_.setGoalStateEnum(goalStateEnum);

  if (!reachState) {
    return true;
  }

  const double timeStep = 1.0 / checkingFrequency;
  double timeSlept = 0.0;
  while (true) {
    if (timeSlept > timeout) {
      return false;
    }
    if (goalStateHasBeenReached()) {
      return true;
    }
    thread_sleep(timeStep);
    timeSlept += timeStep;
  }
}

void DriveEthercatDevice::clearGoalStateEnum()
{
  stateMachine_.clearGoalStateEnum();
}

bool DriveEthercatDevice::startup()
{
  if(!bus_->waitForState(EC_STATE_PRE_OP, address_, 50)) {
    MELO_ERROR_STREAM("[rsl_drive_sdk:Drive::startup] '" << name_ <<
        "' did not reach PRE_OP state.");
  }
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Startup: " << getName() );

  std::string model;
  getDriveModel(model);
  MELO_INFO("[" + getName() + "] model: " + model)
            // Configure PDO setup
  if (!configurePdo(pdoTypeEnum_)) {
    MELO_ERROR("Could not configure pdo mapping");
    return false;
  }

  setFSMGoalState(config_.getGoalStateEnumStartup(), false, 0, 0);

    // This sets the mode::ModeEnum of command_ to mode::ModeEnum::Freeze
    // This means that the drives freeze after startup if and only if:
    // - ControlOp state is reached.
    // - No other command with a ModeEnum that requests another mode is staged
    //   after startup.
    // - no other state than ControlOp is requested directly after or before startup.
  stageFreeze();

  return true;
}
void DriveEthercatDevice::updateProcessReading()
{

    // Get reading.

  std::lock_guard<std::recursive_mutex> lock(readingMutex_);

    // Update statusword.
  Statusword statusword(reading_.getState().getStatusword());
  setStatusword(statusword);
  statuswordRequested_ = false;

    // execute reading callbacks
  for (const auto & readingCb : readingCbs_) {
    readingCb.second(getName(), reading_);
  }


  if (deviceIsMissing()) {
    Statusword statusword;
    statusword.setStateEnum(fsm::StateEnum::DeviceMissing);

    setStatusword(statusword);
  } else {
    if (statusword_.isEmpty() /* || statusword_.getAge() > getConfiguration().getMaxStatuswordAge()*/)
    {                                                                                                      // TODO Delete statusword request over SDO?
            // Device is not missing but statusword is empty or outdated.
      requestAndSetStatusword();
    }
    if (statusword_.isEmpty()) {
            // Device is not missing but statusword is still empty.
      return;
    }
  }

  const fsm::StateEnum activeState = statusword_.getStateEnum();
  if (activeState == fsm::StateEnum::NA) {
    MELO_DEBUG("The FSM state is not available.");
    return;
  }
  stateMachine_.updateActiveState(activeState);
}

void DriveEthercatDevice::requestAndSetStatusword()
{
  if (!statuswordRequested_) {
    MELO_DEBUG_STREAM("Requesting statusword over SDO ...");
        //requestStatusword();
    statuswordRequested_ = true;
  } else {
    Statusword statusword;
    if (getStatuswordSdo(statusword)) {
      MELO_DEBUG_STREAM("Received statusword over SDO.");
      setStatusword(statusword);
      statuswordRequested_ = false;
    }
  }
}

void DriveEthercatDevice::setStatusword(Statusword & statusword)
{
    // If the stamp has not changed, we assume it is again the same statusword.
  if (statusword.getStamp() == statusword_.getStamp()) {
    return;
  }

    // Check if statusword contains new data.
  if (statusword_.isEmpty() ||
    statusword.getData() != statusword_.getData())
  {
    MELO_DEBUG_STREAM("Received new statusword (" << statusword << ").");
    std::vector<std::string> infos;
    std::vector<std::string> warnings;
    std::vector<std::string> errors;
    std::vector<std::string> fatals;
    statusword.getMessagesDiff(statusword_, infos, warnings, errors, fatals);
    for (const std::string & info : infos) {
      MELO_INFO_STREAM("[" << getName() << "]: " << info);
    }
    for (const std::string & warning : warnings) {
      MELO_WARN_STREAM("[" << getName() << "]: " << warning);
    }
    for (const std::string & error : errors) {
      MELO_ERROR_STREAM("[" << getName() << "]: " << error);
    }
    for (const std::string & fatal : fatals) {
      MELO_ERROR_STREAM("[" << getName() << "]: " << fatal);
    }

    if (statusword.getModeEnum() != statusword_.getModeEnum()) {
      MELO_DEBUG_STREAM("[" << getName() << "]Changed mode to '" <<
          mode::modeEnumToName(statusword.getModeEnum()) << "'.");
    }
  }

    // Always update statusword to set new time stamp.
  statusword_ = statusword;
}

void DriveEthercatDevice::updateRead()
{


  const std::chrono::high_resolution_clock::time_point updateStamp = bus_->getUpdateReadStamp();
  switch (getCurrentPdoTypeEnum()) {
    case PdoTypeEnum::A:
      {
        TxPdoA txPdoA;
        bus_->readTxPdo(address_, txPdoA);

        RecLock lockReading(readingMutex_);
        reading_.getState().setStamp(updateStamp);
        reading_.getState().setStatusword(rsl_drive_sdk::Statusword(txPdoA.statusword_));
        reading_.getState().setTemperature(temperatureScalingInv_ * txPdoA.measuredTemperature_ +
          temperatureOffset_);
        reading_.getState().setVoltage(motorVoltageScalingInv_ * txPdoA.measuredMotorVoltage_);
        reading_.getState().setMotorPosition(motorPositionScalingInv_ *
          txPdoA.measuredMotorPosition_);
        reading_.getState().setGearPosition(gearPositionScalingInv_ * txPdoA.measuredGearPosition_);
        reading_.getState().setJointPosition(jointPositionScalingInv_ *
          txPdoA.measuredJointPosition_);
        reading_.getState().setCurrent(motorCurrentScalingInv_ * txPdoA.measuredMotorCurrent_);
        reading_.getState().setMotorVelocity(motorVelocityScalingInv_ *
          txPdoA.measuredMotorVelocity_);
        reading_.getState().setGearVelocity(gearVelocityScalingInv_ * txPdoA.measuredGearVelocity_);
        reading_.getState().setJointVelocity(jointVelocityScalingInv_ *
          txPdoA.measuredJointVelocity_);
        reading_.getState().setJointAcceleration(jointAccelerationScalingInv_ *
          txPdoA.measuredJointAcceleration_);
        reading_.getState().setJointTorque(jointTorqueScalingInv_ * txPdoA.measuredJointTorque_);
      }
      break;
    case PdoTypeEnum::B:
      {
        TxPdoB txPdoB;
        bus_->readTxPdo(address_, txPdoB);

        RecLock lockReading(readingMutex_);
        reading_.getState().setStamp(updateStamp);
        reading_.getState().setStatusword(rsl_drive_sdk::Statusword(txPdoB.statusword_));
        reading_.getState().setTemperature(temperatureScalingInv_ * txPdoB.measuredTemperature_ +
          temperatureOffset_);
        reading_.getState().setVoltage(motorVoltageScalingInv_ * txPdoB.measuredMotorVoltage_);
        reading_.getState().setMotorPosition(motorPositionScalingInv_ *
          txPdoB.measuredMotorPosition_);
        reading_.getState().setGearPosition(gearPositionScalingInv_ * txPdoB.measuredGearPosition_);
        reading_.getState().setJointPosition(jointPositionScalingInv_ *
          txPdoB.measuredJointPosition_);
        reading_.getState().setCurrent(motorCurrentScalingInv_ * txPdoB.measuredMotorCurrent_);
        reading_.getState().setMotorVelocity(motorVelocityScalingInv_ *
          txPdoB.measuredMotorVelocity_);
        reading_.getState().setGearVelocity(gearVelocityScalingInv_ * txPdoB.measuredGearVelocity_);
        reading_.getState().setJointVelocity(jointVelocityScalingInv_ *
          txPdoB.measuredJointVelocity_);
        reading_.getState().setJointAcceleration(jointAccelerationScalingInv_ *
          txPdoB.measuredJointAcceleration_);
        reading_.getState().setJointTorque(jointTorqueScalingInv_ * txPdoB.measuredJointTorque_);

        Imu imu;
        imu.acceleration_x = txPdoB.measuredImuLinearAccelerationX_;
        imu.acceleration_y = txPdoB.measuredImuLinearAccelerationY_;
        imu.acceleration_z = txPdoB.measuredImuLinearAccelerationZ_;
        imu.angle_velocity_x = txPdoB.measuredImuAngularVelocityX_;
        imu.angle_velocity_y = txPdoB.measuredImuAngularVelocityY_;
        imu.angle_velocity_z = txPdoB.measuredImuAngularVelocityZ_;
        // 7th IMU value is not read out at the moment.
        reading_.getState().setImu(imu);
      }
      break;
    case PdoTypeEnum::C:
      {
        TxPdoC txPdoC;
        bus_->readTxPdo(address_, txPdoC);

        RecLock lockReading(readingMutex_);
        reading_.getState().setStamp(updateStamp);
        reading_.getState().setStatusword(rsl_drive_sdk::Statusword(txPdoC.statusword_));
        reading_.getState().setTemperature(temperatureScalingInv_ * txPdoC.measuredTemperature_ +
          temperatureOffset_);
        reading_.getState().setVoltage(motorVoltageScalingInv_ * txPdoC.measuredMotorVoltage_);
        reading_.getState().setMotorPosition(motorPositionScalingInv_ *
          txPdoC.measuredMotorPosition_);
        reading_.getState().setGearPosition(gearPositionScalingInv_ * txPdoC.measuredGearPosition_);
        reading_.getState().setJointPosition(jointPositionScalingInv_ *
          txPdoC.measuredJointPosition_);
        reading_.getState().setCurrent(motorCurrentScalingInv_ * txPdoC.measuredMotorCurrent_);
        reading_.getState().setMotorVelocity(motorVelocityScalingInv_ *
          txPdoC.measuredMotorVelocity_);
        reading_.getState().setGearVelocity(gearVelocityScalingInv_ * txPdoC.measuredGearVelocity_);
        reading_.getState().setJointVelocity(jointVelocityScalingInv_ *
          txPdoC.measuredJointVelocity_);
        reading_.getState().setJointAcceleration(jointAccelerationScalingInv_ *
          txPdoC.measuredJointAcceleration_);
        reading_.getState().setJointTorque(jointTorqueScalingInv_ * txPdoC.measuredJointTorque_);
        reading_.getState().setGearPositionTicks(txPdoC.measuredGearPositionTicks_);
        reading_.getState().setJointPositionTicks(txPdoC.measuredJointPositionTicks_);
        reading_.getState().setTimestamp(txPdoC.timestamp_);
        reading_.getState().setDesiredCurrentD(txPdoC.desiredCurrentD_);
        reading_.getState().setMeasuredCurrentD(txPdoC.measuredCurrentD_);
        reading_.getState().setDesiredCurrentQ(txPdoC.desiredCurrentQ_);
        reading_.getState().setMeasuredCurrentQ(txPdoC.measuredCurrentQ_);
        reading_.getState().setAlpha(txPdoC.alpha_);
        reading_.getState().setBeta(txPdoC.beta_);
        reading_.getState().setDutyCycleU(txPdoC.dutyCycleU_);
        reading_.getState().setDutyCycleV(txPdoC.dutyCycleV_);
        reading_.getState().setDutyCycleW(txPdoC.dutyCycleW_);
        reading_.getState().setMeasuredCurrentPhaseU(txPdoC.measuredCurrentPhaseU_);
        reading_.getState().setMeasuredCurrentPhaseV(txPdoC.measuredCurrentPhaseV_);
        reading_.getState().setMeasuredCurrentPhaseW(txPdoC.measuredCurrentPhaseW_);
        reading_.getState().setMeasuredVoltagePhaseU(txPdoC.measuredVoltagePhaseU_);
        reading_.getState().setMeasuredVoltagePhaseV(txPdoC.measuredVoltagePhaseV_);
        reading_.getState().setMeasuredVoltagePhaseW(txPdoC.measuredVoltagePhaseW_);
        reading_.getState().setMeasuredApparentPower(txPdoC.measuredCurrentPhaseU_ *
          txPdoC.measuredVoltagePhaseU_ + txPdoC.measuredCurrentPhaseV_ *
          txPdoC.measuredVoltagePhaseV_ + txPdoC.measuredCurrentPhaseW_ *
          txPdoC.measuredVoltagePhaseW_);
        reading_.getState().setMeasuredInputCurrent(reading_.getState().getMeasuredApparentPower() /
          txPdoC.measuredMotorVoltage_);
        reading_.getCommanded().setStamp(updateStamp);
        reading_.getCommanded().setCurrent(txPdoC.desiredCurrentQ_);
        reading_.getCommanded().setMotorVelocity(motorVelocityScalingInv_ *
          txPdoC.desiredMotorVelocity_);
        reading_.getCommanded().setGearPosition(gearPositionScalingInv_ *
          txPdoC.desiredGearPosition_);
        reading_.getCommanded().setGearVelocity(gearVelocityScalingInv_ *
          txPdoC.desiredGearVelocity_);
        reading_.getCommanded().setJointPosition(jointPositionScalingInv_ *
          txPdoC.desiredJointPosition_);
        reading_.getCommanded().setJointVelocity(jointVelocityScalingInv_ *
          txPdoC.desiredJointVelocity_);
        reading_.getCommanded().setJointTorque(jointTorqueScalingInv_ * txPdoC.desiredJointTorque_);


        Imu imu;
        imu.acceleration_x = txPdoC.measuredImuLinearAccelerationX_;
        imu.acceleration_y = txPdoC.measuredImuLinearAccelerationY_;
        imu.acceleration_z = txPdoC.measuredImuLinearAccelerationZ_;
        imu.angle_velocity_x = txPdoC.measuredImuAngularVelocityX_;
        imu.angle_velocity_y = txPdoC.measuredImuAngularVelocityY_;
        imu.angle_velocity_z = txPdoC.measuredImuAngularVelocityZ_;
        reading_.getState().setImu(imu);
      }
      break;
    case PdoTypeEnum::D:
      {

        TxPdoD txPdoD;
        bus_->readTxPdo(address_, txPdoD);

        RecLock lockReading(readingMutex_);
        reading_.getState().setStamp(updateStamp);
        reading_.getState().setStatusword(rsl_drive_sdk::Statusword(txPdoD.statusword_));
        reading_.getState().setTemperature(temperatureScalingInv_ * txPdoD.measuredTemperature_ +
          temperatureOffset_);
        reading_.getState().setVoltage(motorVoltageScalingInv_ * txPdoD.measuredMotorVoltage_);
        reading_.getState().setMotorPosition(motorPositionScalingInv_ *
          txPdoD.measuredMotorPosition_);
        reading_.getState().setGearPosition(gearPositionScalingInv_ * txPdoD.measuredGearPosition_);
        reading_.getState().setJointPosition(jointPositionScalingInv_ *
          txPdoD.measuredJointPosition_);
        reading_.getState().setCurrent(motorCurrentScalingInv_ * txPdoD.measuredMotorCurrent_);
        reading_.getState().setMotorVelocity(motorVelocityScalingInv_ *
          txPdoD.measuredMotorVelocity_);
        reading_.getState().setGearVelocity(gearVelocityScalingInv_ * txPdoD.measuredGearVelocity_);
        reading_.getState().setJointVelocity(jointVelocityScalingInv_ *
          txPdoD.measuredJointVelocity_);
        reading_.getState().setJointAcceleration(jointAccelerationScalingInv_ *
          txPdoD.measuredJointAcceleration_);
        reading_.getState().setJointTorque(jointTorqueScalingInv_ * txPdoD.measuredJointTorque_);
        reading_.getState().setGearPositionTicks(txPdoD.measuredGearPositionTicks_);
        reading_.getState().setJointPositionTicks(txPdoD.measuredJointPositionTicks_);
        reading_.getState().setTimestamp(txPdoD.timestamp_);
        reading_.getState().setDesiredCurrentD(txPdoD.desiredCurrentD_);
        reading_.getState().setMeasuredCurrentD(txPdoD.measuredCurrentD_);
        reading_.getState().setDesiredCurrentQ(txPdoD.desiredCurrentQ_);
        reading_.getState().setMeasuredCurrentQ(txPdoD.measuredCurrentQ_);
        reading_.getState().setMeasuredApparentPower(txPdoD.measuredCurrentPhaseU_ *
          txPdoD.measuredVoltagePhaseU_ + txPdoD.measuredCurrentPhaseV_ *
          txPdoD.measuredVoltagePhaseV_ + txPdoD.measuredCurrentPhaseW_ *
          txPdoD.measuredVoltagePhaseW_);
        reading_.getState().setMeasuredInputCurrent(reading_.getState().getMeasuredApparentPower() /
          txPdoD.measuredMotorVoltage_);
        reading_.getState().setMeasuredCurrentPhaseU(txPdoD.measuredCurrentPhaseU_);
        reading_.getState().setMeasuredCurrentPhaseV(txPdoD.measuredCurrentPhaseV_);
        reading_.getState().setMeasuredCurrentPhaseW(txPdoD.measuredCurrentPhaseW_);
        reading_.getState().setMeasuredVoltagePhaseU(txPdoD.measuredVoltagePhaseU_);
        reading_.getState().setMeasuredVoltagePhaseV(txPdoD.measuredVoltagePhaseV_);
        reading_.getState().setMeasuredVoltagePhaseW(txPdoD.measuredVoltagePhaseW_);
      }
      break;
    case PdoTypeEnum::E:
      {
        TxPdoE txPdoE;
        bus_->readTxPdo(address_, txPdoE);

        RecLock lockReading(readingMutex_);
        reading_.getState().setStamp(updateStamp);
        reading_.getState().setStatusword(rsl_drive_sdk::Statusword(txPdoE.statusword_));
        reading_.getState().setTemperature(temperatureScalingInv_ * txPdoE.measuredTemperature_ +
          temperatureOffset_);
        reading_.getState().setVoltage(motorVoltageScalingInv_ * txPdoE.measuredMotorVoltage_);
        reading_.getState().setMotorPosition(motorPositionScalingInv_ *
          txPdoE.measuredMotorPosition_);
        reading_.getState().setGearPosition(gearPositionScalingInv_ * txPdoE.measuredGearPosition_);
        reading_.getState().setJointPosition(jointPositionScalingInv_ *
          txPdoE.measuredJointPosition_);
        reading_.getState().setCurrent(motorCurrentScalingInv_ * txPdoE.measuredMotorCurrent_);
        reading_.getState().setMotorVelocity(motorVelocityScalingInv_ *
          txPdoE.measuredMotorVelocity_);
        reading_.getState().setGearVelocity(gearVelocityScalingInv_ * txPdoE.measuredGearVelocity_);
        reading_.getState().setJointVelocity(jointVelocityScalingInv_ *
          txPdoE.measuredJointVelocity_);
        reading_.getState().setJointAcceleration(jointAccelerationScalingInv_ *
          txPdoE.measuredJointAcceleration_);
        reading_.getState().setJointTorque(jointTorqueScalingInv_ * txPdoE.measuredJointTorque_);
        reading_.getState().setGearPositionTicks(txPdoE.measuredGearPositionTicks_);
        reading_.getState().setJointPositionTicks(txPdoE.measuredJointPositionTicks_);
        reading_.getState().setTimestamp(txPdoE.timestamp_);
        reading_.getState().setDesiredCurrentD(txPdoE.desiredCurrentD_);
        reading_.getState().setMeasuredCurrentD(txPdoE.measuredCurrentD_);
        reading_.getState().setDesiredCurrentQ(txPdoE.desiredCurrentQ_);
        reading_.getState().setMeasuredCurrentQ(txPdoE.measuredCurrentQ_);
        reading_.getState().setAlpha(txPdoE.alpha_);
        reading_.getState().setBeta(txPdoE.beta_);
        reading_.getState().setDutyCycleU(txPdoE.dutyCycleU_);
        reading_.getState().setDutyCycleV(txPdoE.dutyCycleV_);
        reading_.getState().setDutyCycleW(txPdoE.dutyCycleW_);
        reading_.getState().setMeasuredCurrentPhaseU(txPdoE.measuredCurrentPhaseU_);
        reading_.getState().setMeasuredCurrentPhaseV(txPdoE.measuredCurrentPhaseV_);
        reading_.getState().setMeasuredCurrentPhaseW(txPdoE.measuredCurrentPhaseW_);
        reading_.getState().setMeasuredVoltagePhaseU(txPdoE.measuredVoltagePhaseU_);
        reading_.getState().setMeasuredVoltagePhaseV(txPdoE.measuredVoltagePhaseV_);
        reading_.getState().setMeasuredVoltagePhaseW(txPdoE.measuredVoltagePhaseW_);
        reading_.getState().setMeasuredApparentPower(txPdoE.measuredCurrentPhaseU_ *
          txPdoE.measuredVoltagePhaseU_ + txPdoE.measuredCurrentPhaseV_ *
          txPdoE.measuredVoltagePhaseV_ + txPdoE.measuredCurrentPhaseW_ *
          txPdoE.measuredVoltagePhaseW_);
        reading_.getState().setMeasuredInputCurrent(reading_.getState().getMeasuredApparentPower() /
          txPdoE.measuredMotorVoltage_);
        reading_.getCommanded().setStamp(updateStamp);
        reading_.getCommanded().setCurrent(txPdoE.desiredCurrentQ_);
        reading_.getCommanded().setMotorVelocity(motorVelocityScalingInv_ *
          txPdoE.desiredMotorVelocity_);
        reading_.getCommanded().setGearPosition(gearPositionScalingInv_ *
          txPdoE.desiredGearPosition_);
        reading_.getCommanded().setGearVelocity(gearVelocityScalingInv_ *
          txPdoE.desiredGearVelocity_);
        reading_.getCommanded().setJointPosition(jointPositionScalingInv_ *
          txPdoE.desiredJointPosition_);
        reading_.getCommanded().setJointVelocity(jointVelocityScalingInv_ *
          txPdoE.desiredJointVelocity_);
        reading_.getCommanded().setJointTorque(jointTorqueScalingInv_ * txPdoE.desiredJointTorque_);
        Imu imu;
        imu.acceleration_x = txPdoE.measuredImuLinearAccelerationX_;
        imu.acceleration_y = txPdoE.measuredImuLinearAccelerationY_;
        imu.acceleration_z = txPdoE.measuredImuLinearAccelerationZ_;
        imu.angle_velocity_x = txPdoE.measuredImuAngularVelocityX_;
        imu.angle_velocity_y = txPdoE.measuredImuAngularVelocityY_;
        imu.angle_velocity_z = txPdoE.measuredImuAngularVelocityZ_;
        reading_.getState().setImu(imu);

        reading_.getState().setCoilTemp1(txPdoE.measuredCoilTemp1_ / 1000.0);
        reading_.getState().setCoilTemp2(txPdoE.measuredCoilTemp2_ / 1000.0);
        reading_.getState().setCoilTemp3(txPdoE.measuredCoilTemp3_ / 1000.0);

      }
      break;
    default:
      break;
  }

    // Save the command in the reading.
  if (!isRtdlRunning_) {
    RecLock lockReading(readingMutex_);
    RecLock lockCommand(commandMutex_);
    reading_.setCommanded(command_);
  }

  updateProcessReading();
}

void DriveEthercatDevice::updateWrite()
{
  RecLock lock(commandMutex_);
  const rsl_drive_sdk::mode::ModeEnum modeEnum = command_.getModeEnum();

  if(modeEnum == rsl_drive_sdk::mode::ModeEnum::NA) {
    return;
  }

  const uint16_t modeOfOperation = rsl_drive_sdk::mode::modeEnumToId(modeEnum);
  const auto & mode = config_.getMode(modeEnum);
  if (!mode) {
    return;
  }

  RxPdo rxPdo;
  {
    RecLock lock(controlwordIdMutex_);
    rxPdo.controlword_ = controlwordId_;
  }
  rxPdo.modeOfOperation_ = modeOfOperation;
  rxPdo.desiredMotorCurrent_ = motorCurrentScaling_ * command_.getCurrent();
  if (mode->controlMotorVelocity_) {
    rxPdo.desiredVelocity_ = motorVelocityScaling_ * command_.getMotorVelocity();
  } else if (mode->controlGearVelocity_) {
    rxPdo.desiredVelocity_ = gearVelocityScaling_ * command_.getGearVelocity();
  } else if (mode->controlJointVelocity_) {
    rxPdo.desiredVelocity_ = jointVelocityScaling_ * command_.getJointVelocity();
  }
  rxPdo.desiredJointTorque_ = jointTorqueScaling_ * command_.getJointTorque();
  if (mode->controlMotorPosition_) {
    rxPdo.desiredPosition_ = motorPositionScaling_ * command_.getMotorPosition();
  } else if (mode->controlGearPosition_) {
    rxPdo.desiredPosition_ = gearPositionScaling_ * command_.getGearPosition();
  } else if (mode->controlJointPosition_) {
    rxPdo.desiredPosition_ = jointPositionScaling_ * command_.getJointPosition();
  }
  rxPdo.controlParameterA_ = gainScaling_ * command_.getPidGains().getP();
  rxPdo.controlParameterB_ = gainScaling_ * command_.getPidGains().getI();
  rxPdo.controlParameterC_ = gainScaling_ * command_.getPidGains().getD();
  rxPdo.controlParameterD_ = gainScaling_ * 0.0;

  bus_->writeRxPdo(address_, rxPdo);

}

void DriveEthercatDevice::shutdown()
{
    // setGoalStateEnum(rsl_drive_sdk::fsm::StateEnum::Standby, true, 100000,1000);

}

void DriveEthercatDevice::preShutdown()
{
  auto shutdown_state = config_.getGoalStateEnumShutdown();
  if(shutdown_state == fsm::StateEnum::NA) {
    shutdown_state = fsm::StateEnum::Configure;
  }

  MELO_INFO_STREAM("Pre shutdown: " << shutdown_state);

  if (setFSMGoalState(shutdown_state, true, 10, 10)) {
    MELO_INFO("Pre shutdown finished");
  } else {
    MELO_ERROR_STREAM(
        "[DriveEthercatDevice::preShutdown] shutdown state not reached for rsl_drive_sdk '" <<
        getName() << "'");
  }
}

bool DriveEthercatDevice::deviceIsMissing() const
{
  return false;   // TODO
}

void DriveEthercatDevice::setState(const uint16_t state)
{
  RecLock lock(mutex_);
  bus_->setState(state, address_);
}

bool DriveEthercatDevice::waitForState(const uint16_t state)
{
  RecLock lock(mutex_);
  return bus_->waitForState(state, address_);
}


PdoTypeEnum DriveEthercatDevice::getCurrentPdoTypeEnum() const
{
  return currentPdoTypeEnum_;
}

DriveEthercatDevice::PdoInfo DriveEthercatDevice::getCurrentPdoInfo() const
{
  return pdoInfos_.at(currentPdoTypeEnum_);
}

bool DriveEthercatDevice::getDriveModel(std::string & model)
{
  return sendSdoReadVisibleString(OD_DSP402_MODEL_ID, 0, model);
}

bool DriveEthercatDevice::getDriveInfoSerialNumber(std::string & serialNumber)
{
  return sendSdoReadStringCustom(OD_DRIVE_INFO_HARDWARE_SERIAL_NUMBER_ID, serialNumber);
}

bool DriveEthercatDevice::setDriveInfoSerialNumber(const std::string & serialNumber)
{
  return sendSdoWriteStringCustom(OD_DRIVE_INFO_HARDWARE_SERIAL_NUMBER_ID, serialNumber);
}

bool DriveEthercatDevice::getDriveInfoName(std::string & name)
{
  return sendSdoReadStringCustom(OD_DRIVE_INFO_DRIVE_NAME_ID, name);
}

bool DriveEthercatDevice::setDriveInfoName(const std::string & name)
{
  return sendSdoWriteStringCustom(OD_DRIVE_INFO_DRIVE_NAME_ID, name);
}

bool DriveEthercatDevice::getDriveInfoId(uint16_t & id)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_DRIVE_INFO_DRIVE_ID_ID, 0x00, false, id);
}

bool DriveEthercatDevice::setDriveInfoId(const uint16_t id)
{
  RecLock lock(mutex_);
  return sendSdoWrite(OD_DRIVE_INFO_DRIVE_ID_ID, 0x00, false, id);
}

bool DriveEthercatDevice::getDriveInfoBootloaderVersion(
  rsl_drive_sdk::common::Version & bootloaderVersion)
{
  std::string bootloaderVersionString;
  if (!sendSdoReadStringCustom(OD_DRIVE_INFO_BOOTLOADER_VERSION_ID, bootloaderVersionString)) {
    return false;
  }
  bootloaderVersion.fromString(bootloaderVersionString);
  return true;
}

bool DriveEthercatDevice::setDriveInfoBootloaderVersion(
  const rsl_drive_sdk::common::Version & bootloaderVersion)
{
  return sendSdoWriteStringCustom(OD_DRIVE_INFO_BOOTLOADER_VERSION_ID,
      bootloaderVersion.toString());
}

bool DriveEthercatDevice::getDriveInfoFirmwareVersion(
  rsl_drive_sdk::common::Version & firmwareVersion)
{
  std::string firmwareVersionString;
  if (!sendSdoReadStringCustom(OD_DRIVE_INFO_FIRMWARE_VERSION_ID, firmwareVersionString)) {
    return false;
  }
  firmwareVersion.fromString(firmwareVersionString);
  return true;
}

bool DriveEthercatDevice::getDriveFirmwareInfo(rsl_drive_sdk::common::FirmwareInfo & firmwareInfo)
{
  uint8_t infoRaw[128];
  uint8_t numberOfSubindexes = 0;
  bool ok = false;

  for (int j = 0; j < 20; ++j) {
    if (sendSdoRead(OD_DRIVE_FIRMWARE_INFO_ID, OD_DRIVE_FIRMWARE_INFO_SID_0, false,
        numberOfSubindexes))
    {
      ok = true;
      break;
    }
  }
  if (!ok) {
    MELO_ERROR_STREAM("Could not read subindex 0 of index 0x7075.");
    return false;
  }

  const uint8_t nChars = numberOfSubindexes * sizeof(uint32_t) / sizeof(char);
  if (nChars != 128) {
    MELO_ERROR_STREAM("Firmware info array has the wrong length (" << (int)nChars << ").");
    return false;
  }

  for (uint8_t i = 0; i < numberOfSubindexes; i++) {
    ok = false;
    for (int j = 0; j < 20; ++j) {
      rsl_drive_sdk::thread_sleep(0.0001);
      if (sendSdoRead(OD_DRIVE_FIRMWARE_INFO_ID, OD_DRIVE_FIRMWARE_INFO_SID_DATA + i, false,
        ((uint32_t *)infoRaw)[i]))
      {
        ok = true;
        break;
      }
    }
    if (!ok) {
      MELO_ERROR_STREAM("Error during read of the firmware info.");
      return false;
    }
  }

  firmwareInfo.infoVersion = infoRaw[0];

  if (firmwareInfo.infoVersion == 3) {
        // Get version.
    std::stringstream versionStream;
    for (int i = 0; i < 3; ++i) {
      versionStream << (int)infoRaw[1 + i];
      if (i < 2) {
        versionStream << ".";
      }
    }
    firmwareInfo.version = versionStream.str();

        // Get fw hash.
    std::stringstream hashStream;
    for (int i = 0; i < 16; ++i) {
      if (infoRaw[4 + i] == '\0') {
        break;
      }
      hashStream       << std::setfill('0') << std::setw(sizeof(uint8_t) * 2)
                       << std::hex << (int)infoRaw[4 + i];
    }
    firmwareInfo.fwHash = hashStream.str();

        // Get channel id.
    std::stringstream channelIdStream;
    for (int i = 0; i < 16; ++i) {
      if (infoRaw[20 + i] == '\0') {
        break;
      }
      channelIdStream       << std::setfill('0') << std::setw(sizeof(uint8_t) * 2)
                            << std::hex << (int)infoRaw[20 + i];
    }
    firmwareInfo.channelId = channelIdStream.str();

        // Get channel tid.
    std::stringstream channelTidStream;
    for (int i = 0; i < 16; ++i) {
      if (infoRaw[36 + i] == '\0') {
        break;
      }
      channelTidStream << infoRaw[36 + i];
    }
    firmwareInfo.channelTid = channelTidStream.str();

        // Get serial number.
    std::stringstream serialNumberStream;
    for (int i = 0; i < 16; ++i) {
      if (infoRaw[52 + i] == '\0') {
        break;
      }
      serialNumberStream << infoRaw[52 + i];
    }
    firmwareInfo.serialNumber = serialNumberStream.str();

        // Get key id.
    std::stringstream keyIdStream;
    for (int i = 0; i < 16; ++i) {
      keyIdStream       << std::setfill('0') << std::setw(sizeof(uint8_t) * 2)
                        << std::hex << (int)infoRaw[68 + i];
      if (i == 3 || i == 5 || i == 7 || i == 9) {
        keyIdStream << "-";
      }
    }
    firmwareInfo.keyId = keyIdStream.str();
  } else {
    MELO_ERROR_STREAM(
                    "Firmware info version ("
                    << (int)firmwareInfo.infoVersion
                    << ") is not compatible with this SDK version. "
                       "Please update the SDK.")
    return false;
  }

  return true;
}
bool DriveEthercatDevice::getBuildInfo(rsl_drive_sdk::common::BuildInfo & buildInfo)
{
  bool success = true;
  success &= sendSdoReadVisibleString(OD_BUILDINFO_ID, 0x01, buildInfo.buildDate);
  success &= sendSdoReadVisibleString(OD_BUILDINFO_ID, 0x02, buildInfo.gitTag);
  success &= sendSdoReadVisibleString(OD_BUILDINFO_ID, 0x03, buildInfo.gitHash);

  rsl_drive_sdk::common::capabilities_u u;
  success &= sendSdoRead(OD_BUILDINFO_ID + 1, 0x00, false, u.raw);
  buildInfo.caps = u.cap;

  return success;
}

bool DriveEthercatDevice::getDriveType(std::string & type)
{
  return  sendSdoReadVisibleString(OD_DRIVETYPE_ID, 0x01, type);
}
bool DriveEthercatDevice::getGearboxRatio(float & ratio)
{
  return sendSdoReadFloat(OD_GEARBOX_RATIO_ID, 0x00, false, ratio);
}

bool DriveEthercatDevice::eraseFlashStorage()
{
  RecLock lock(mutex_);
  MELO_INFO_STREAM("Erasing flash storage.");
  return sendSdoWrite(OD_FLASH_ERASE_ID, 0x00, false,
      static_cast<uint16_t>(OD_FLASH_ERASE_ID_VAL_RUN));
}

bool DriveEthercatDevice::resetFlashStorageSections(const uint16_t flashStorageSections)
{
  RecLock lock(mutex_);
  MELO_INFO_STREAM("Resetting flash storage sections (" << flashStorageSections << ").");
  return sendSdoWrite(OD_FLASH_RESET_ID, 0x00, false, flashStorageSections);
}

bool DriveEthercatDevice::getCalibrationState(
  const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum,
  rsl_drive_sdk::calibration::CalibrationState & calibrationState)
{
  RecLock lock(mutex_);
  rsl_drive_sdk::calibration::CalibrationTypeEnum previousCalibrationTypeEnum =
    rsl_drive_sdk::calibration::CalibrationTypeEnum::NA;
  if (!getCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  if (!setCalibrationTypeEnum(calibrationTypeEnum)) {
    return false;
  }
  const bool success = sendSdoRead(OD_CALIB_STATES_ID, 0x00, false, calibrationState.all_);
  if (!setCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  return success;
}

bool DriveEthercatDevice::sendCalibrationGearAndJointEncoderHomingNewJointPosition(
  const double newJointPosition)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Sending homing calibration new joint position.");
  return sendSdoWrite(OD_CALIB_GEAR_AND_JOINT_ENCODER_HOMING_NEW_JOINT_POSITION_ID, 0x00, false,
      static_cast<float>(jointPositionScaling_ * newJointPosition));
}

bool DriveEthercatDevice::startCalibration(
  const rsl_drive_sdk::calibration::CalibrationModeEnum calibrationModeEnum)
{
  RecLock lock(mutex_);
  MELO_INFO_STREAM("Starting calibration '" <<
      rsl_drive_sdk::calibration::calibrationModeEnumToName(calibrationModeEnum) << "'.");
  return sendSdoWrite(OD_CALIB_MODE_ID, 0x00, false,
      rsl_drive_sdk::calibration::calibrationModeEnumToId(calibrationModeEnum));
}

bool DriveEthercatDevice::calibrationIsRunning(bool & running)
{
  RecLock lock(mutex_);
  uint16_t calibrationMode = 0;
  if (!sendSdoRead(OD_CALIB_MODE_ID, 0x00, false, calibrationMode)) {
    return false;
  }
  running = (calibrationMode != OD_CALIB_MODE_ID_VAL_IDLE);
  return true;
}

bool DriveEthercatDevice::getCalibrationMotorEncoderOffset(
  const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum,
  rsl_drive_sdk::calibration::parameter::MotorEncoderOffset & motorEncoderOffset)
{
  RecLock lock(mutex_);
  rsl_drive_sdk::calibration::CalibrationTypeEnum previousCalibrationTypeEnum =
    rsl_drive_sdk::calibration::CalibrationTypeEnum::NA;
  if (!getCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  if (!setCalibrationTypeEnum(calibrationTypeEnum)) {
    return false;
  }
  const bool success = sendSdoRead(OD_CALIB_MOTOR_ENCODER_PARAMS_ID,
      OD_CALIB_MOTOR_ENCODER_PARAMS_SID_OFFSET, false, motorEncoderOffset.value_);
  if (!setCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  return success;
}

bool DriveEthercatDevice::getCalibrationMotorEncoderParameters(
  const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum,
  rsl_drive_sdk::calibration::parameter::MotorEncoderParameters & motorEncoderParameters)
{
  RecLock lock(mutex_);
  rsl_drive_sdk::calibration::CalibrationTypeEnum previousCalibrationTypeEnum =
    rsl_drive_sdk::calibration::CalibrationTypeEnum::NA;
  if (!getCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  if (!setCalibrationTypeEnum(calibrationTypeEnum)) {
    return false;
  }
  bool success = true;
  success &= sendSdoRead(OD_CALIB_MOTOR_ENCODER_PARAMS_ID, OD_CALIB_MOTOR_ENCODER_PARAMS_SID_DGAIN,
      false, motorEncoderParameters.dGain_);
  success &= sendSdoRead(OD_CALIB_MOTOR_ENCODER_PARAMS_ID, OD_CALIB_MOTOR_ENCODER_PARAMS_SID_DOFFS,
      false, motorEncoderParameters.dOffs_);
  success &= sendSdoRead(OD_CALIB_MOTOR_ENCODER_PARAMS_ID, OD_CALIB_MOTOR_ENCODER_PARAMS_SID_DOFFC,
      false, motorEncoderParameters.dOffc_);
  success &= sendSdoRead(OD_CALIB_MOTOR_ENCODER_PARAMS_ID, OD_CALIB_MOTOR_ENCODER_PARAMS_SID_DPH,
      false, motorEncoderParameters.dPh_);
  success &= sendSdoRead(OD_CALIB_MOTOR_ENCODER_PARAMS_ID, OD_CALIB_MOTOR_ENCODER_PARAMS_SID_AGAIN,
      false, motorEncoderParameters.aGain_);
  success &= sendSdoRead(OD_CALIB_MOTOR_ENCODER_PARAMS_ID, OD_CALIB_MOTOR_ENCODER_PARAMS_SID_AOFFS,
      false, motorEncoderParameters.aOffs_);
  success &= sendSdoRead(OD_CALIB_MOTOR_ENCODER_PARAMS_ID, OD_CALIB_MOTOR_ENCODER_PARAMS_SID_AOFFC,
      false, motorEncoderParameters.aOffc_);
  if (!setCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  return success;
}

bool DriveEthercatDevice::getCalibrationGearJointEncoderOffset(
  const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum,
  rsl_drive_sdk::calibration::parameter::GearJointEncoderOffset & gearJointEncoderOffset)
{
  RecLock lock(mutex_);
  rsl_drive_sdk::calibration::CalibrationTypeEnum previousCalibrationTypeEnum =
    rsl_drive_sdk::calibration::CalibrationTypeEnum::NA;
  if (!getCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  if (!setCalibrationTypeEnum(calibrationTypeEnum)) {
    return false;
  }
  bool success = true;
  success &= sendSdoRead(OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_ID,
      OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_CONSTANT, false, gearJointEncoderOffset.constant_);
  success &= sendSdoRead(OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_ID,
      OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_SIN1_AMPLITUDE, false,
      gearJointEncoderOffset.sin1Amplitude_);
  success &= sendSdoRead(OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_ID,
      OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_SIN1_PHASESHIFT, false,
      gearJointEncoderOffset.sin1Phaseshift_);
  success &= sendSdoRead(OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_ID,
      OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_SIN2_AMPLITUDE, false,
      gearJointEncoderOffset.sin2Amplitude_);
  success &= sendSdoRead(OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_ID,
      OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_SIN2_PHASESHIFT, false,
      gearJointEncoderOffset.sin2Phaseshift_);
  if (!setCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  return success;
}

bool DriveEthercatDevice::setCalibrationGearAndJointEncoderHoming(
  [[maybe_unused]] const calibration::CalibrationTypeEnum calibrationTypeEnum,
  const calibration::parameter::GearAndJointEncoderHoming & gearAndJointEncoderHoming)
{
  RecLock lock(mutex_);
  bool success = true;
  success &= sendSdoWrite(OD_CALIB_GEAR_AND_JOINT_ENCODER_HOMING_TICKS_ID,
      OD_CALIB_GEAR_AND_JOINT_ENCODER_HOMING_TICKS_SID_GEAR, false,
      gearAndJointEncoderHoming.gearEncoderRawTicks_);
  success &= sendSdoWrite(OD_CALIB_GEAR_AND_JOINT_ENCODER_HOMING_TICKS_ID,
      OD_CALIB_GEAR_AND_JOINT_ENCODER_HOMING_TICKS_SID_JOINT, false,
      gearAndJointEncoderHoming.jointEncoderRawTicks_);
  return success;
}

bool DriveEthercatDevice::getCalibrationGearAndJointEncoderHoming(
  const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum,
  rsl_drive_sdk::calibration::parameter::GearAndJointEncoderHoming & gearAndJointEncoderHoming)
{
  RecLock lock(mutex_);
  rsl_drive_sdk::calibration::CalibrationTypeEnum previousCalibrationTypeEnum =
    rsl_drive_sdk::calibration::CalibrationTypeEnum::NA;
  if (!getCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  if (!setCalibrationTypeEnum(calibrationTypeEnum)) {
    return false;
  }
  bool success = true;
  success &= sendSdoRead(OD_CALIB_GEAR_AND_JOINT_ENCODER_HOMING_TICKS_ID,
      OD_CALIB_GEAR_AND_JOINT_ENCODER_HOMING_TICKS_SID_GEAR, false,
      gearAndJointEncoderHoming.gearEncoderRawTicks_);
  success &= sendSdoRead(OD_CALIB_GEAR_AND_JOINT_ENCODER_HOMING_TICKS_ID,
      OD_CALIB_GEAR_AND_JOINT_ENCODER_HOMING_TICKS_SID_JOINT, false,
      gearAndJointEncoderHoming.jointEncoderRawTicks_);
  if (!setCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  return success;
}

bool DriveEthercatDevice::getCalibrationImuGyroscopeDcBias(
  const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum,
  rsl_drive_sdk::calibration::parameter::ImuGyroscopeDcBias & imuGyroscopeDcBias)
{
  RecLock lock(mutex_);
  rsl_drive_sdk::calibration::CalibrationTypeEnum previousCalibrationTypeEnum =
    rsl_drive_sdk::calibration::CalibrationTypeEnum::NA;
  if (!getCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  if (!setCalibrationTypeEnum(calibrationTypeEnum)) {
    return false;
  }
  bool success = true;
  success &= sendSdoRead(OD_CALIB_IMU_GYROSCOPE_DC_BIAS_ID, OD_CALIB_IMU_GYROSCOPE_DC_BIAS_SID_X,
      false, imuGyroscopeDcBias.x_);
  success &= sendSdoRead(OD_CALIB_IMU_GYROSCOPE_DC_BIAS_ID, OD_CALIB_IMU_GYROSCOPE_DC_BIAS_SID_Y,
      false, imuGyroscopeDcBias.y_);
  success &= sendSdoRead(OD_CALIB_IMU_GYROSCOPE_DC_BIAS_ID, OD_CALIB_IMU_GYROSCOPE_DC_BIAS_SID_Z,
      false, imuGyroscopeDcBias.z_);
  if (!setCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  return success;
}

bool DriveEthercatDevice::setCalibrationSpringStiffness(
  [[maybe_unused]] const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum,
  const rsl_drive_sdk::calibration::parameter::SpringStiffness & springStiffness)
{
  RecLock lock(mutex_);
  bool success = true;
  success &= sendSdoWrite(OD_CALIB_SPRING_STIFFNESS_ID, OD_CALIB_SPRING_STIFFNESS_SID_NEG, false,
      springStiffness.neg_);
  success &= sendSdoWrite(OD_CALIB_SPRING_STIFFNESS_ID, OD_CALIB_SPRING_STIFFNESS_SID_POS, false,
      springStiffness.pos_);
  return success;
}

bool DriveEthercatDevice::getCalibrationSpringStiffness(
  const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum,
  rsl_drive_sdk::calibration::parameter::SpringStiffness & springStiffness)
{
  RecLock lock(mutex_);
  rsl_drive_sdk::calibration::CalibrationTypeEnum previousCalibrationTypeEnum =
    rsl_drive_sdk::calibration::CalibrationTypeEnum::NA;
  if (!getCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  if (!setCalibrationTypeEnum(calibrationTypeEnum)) {
    return false;
  }
  bool success = true;
  success &= sendSdoRead(OD_CALIB_SPRING_STIFFNESS_ID, OD_CALIB_SPRING_STIFFNESS_SID_NEG, false,
      springStiffness.neg_);
  success &= sendSdoRead(OD_CALIB_SPRING_STIFFNESS_ID, OD_CALIB_SPRING_STIFFNESS_SID_POS, false,
      springStiffness.pos_);
  if (!setCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  return success;
}

bool DriveEthercatDevice::setCalibrationFrictionEstimation(
  [[maybe_unused]] const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum,
  const rsl_drive_sdk::calibration::parameter::FrictionEstimation & frictionEstimation)
{
  RecLock lock(mutex_);
  bool success = true;
  success &= sendSdoWrite(OD_CALIB_FRICTION_ESTIMATION_ID,
      OD_CALIB_FRICTION_ESTIMATION_SID_BREAK_AWAY_FRICTION, false,
      frictionEstimation.breakAwayFriction_);
  success &= sendSdoWrite(OD_CALIB_FRICTION_ESTIMATION_ID,
      OD_CALIB_FRICTION_ESTIMATION_SID_BREAK_AWAY_FRICTION_BAND, false,
      frictionEstimation.breakAwayFrictionBand_);
  success &= sendSdoWrite(OD_CALIB_FRICTION_ESTIMATION_ID,
      OD_CALIB_FRICTION_ESTIMATION_SID_VISCOUS_FRICTION_COEFF_NEG, false,
      frictionEstimation.viscousFrictionCoeffNeg_);
  success &= sendSdoWrite(OD_CALIB_FRICTION_ESTIMATION_ID,
      OD_CALIB_FRICTION_ESTIMATION_SID_VISCOUS_FRICTION_COEFF_POS, false,
      frictionEstimation.viscousFrictionCoeffPos_);
  return success;
}

bool DriveEthercatDevice::getCalibrationFrictionEstimation(
  const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum,
  rsl_drive_sdk::calibration::parameter::FrictionEstimation & frictionEstimation)
{
  RecLock lock(mutex_);
  rsl_drive_sdk::calibration::CalibrationTypeEnum previousCalibrationTypeEnum =
    rsl_drive_sdk::calibration::CalibrationTypeEnum::NA;
  if (!getCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  if (!setCalibrationTypeEnum(calibrationTypeEnum)) {
    return false;
  }
  bool success = true;
  success &= sendSdoRead(OD_CALIB_FRICTION_ESTIMATION_ID,
      OD_CALIB_FRICTION_ESTIMATION_SID_BREAK_AWAY_FRICTION, false,
      frictionEstimation.breakAwayFriction_);
  success &= sendSdoRead(OD_CALIB_FRICTION_ESTIMATION_ID,
      OD_CALIB_FRICTION_ESTIMATION_SID_BREAK_AWAY_FRICTION_BAND, false,
      frictionEstimation.breakAwayFrictionBand_);
  success &= sendSdoRead(OD_CALIB_FRICTION_ESTIMATION_ID,
      OD_CALIB_FRICTION_ESTIMATION_SID_VISCOUS_FRICTION_COEFF_NEG, false,
      frictionEstimation.viscousFrictionCoeffNeg_);
  success &= sendSdoRead(OD_CALIB_FRICTION_ESTIMATION_ID,
      OD_CALIB_FRICTION_ESTIMATION_SID_VISCOUS_FRICTION_COEFF_POS, false,
      frictionEstimation.viscousFrictionCoeffPos_);
  if (!setCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  return success;
}

bool DriveEthercatDevice::getCalibrationGearJointEncoderOffsetConstant(int32_t & constant)
{
  return sendSdoRead(OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_ID,
                       OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_CONSTANT,
                       false, constant);
}

bool DriveEthercatDevice::setCalibrationGearJointEncoderOffsetConstant(const int32_t constant)
{
  return sendSdoWrite(OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_ID,
                        OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_CONSTANT,
                        false, constant);
}

bool DriveEthercatDevice::resetCustomCalibrationsToFactory(
  const rsl_drive_sdk::calibration::CalibrationState calibrationState)
{
  RecLock lock(mutex_);
  MELO_INFO_STREAM("Resetting custom calibrations to factory. Fields: " << calibrationState.all_);
  return sendSdoWrite(OD_CALIB_FACTORY_TO_CUSTOM_ID, 0x00, false, calibrationState.all_);
}

bool DriveEthercatDevice::writeFactoryCalibration()
{
  RecLock lock(mutex_);
  MELO_INFO_STREAM("Writing factory calibration to flash.");
  return sendSdoWrite(OD_CALIB_CUSTOM_TO_FACTORY_ID, 0x00, false,
      static_cast<uint16_t>(OD_CALIB_CUSTOM_TO_FACTORY_ID_VAL_RUN));
}

bool DriveEthercatDevice::getMaxCurrent(double & maxCurrent)
{
  RecLock lock(mutex_);
  float maxCurrentFloat = 0.0;
  if (!sendSdoRead(OD_DSP402_CURRENT_MAX_ID, 0x00, false, maxCurrentFloat)) {
    return false;
  }
  maxCurrent = motorCurrentScalingInv_ * maxCurrentFloat;
  return true;
}

bool DriveEthercatDevice::setMaxCurrent(const double maxCurrent)
{
  MELO_DEBUG_STREAM("Setting max current (" << maxCurrent << " A).");
  RecLock lock(mutex_);
  return sendSdoWrite(OD_DSP402_CURRENT_MAX_ID, 0x00, false,
      static_cast<float>(motorCurrentScaling_ * maxCurrent));
}

bool DriveEthercatDevice::getMaxFreezeCurrent(double & current)
{
  RecLock lock(mutex_);
  float currentFloat = 0.0;
  if (!sendSdoRead(OD_CONFIG_MAX_FREEZE_CURRENT, 0x00, false, currentFloat)) {
    return false;
  }
  current = motorCurrentScalingInv_ * currentFloat;
  return true;
}

bool DriveEthercatDevice::setMaxFreezeCurrent(const double current)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting freeze current limit (" << current << ").");
  return sendSdoWrite(OD_CONFIG_MAX_FREEZE_CURRENT, 0x00, false,
      static_cast<float>(motorCurrentScaling_ * current));
}

bool DriveEthercatDevice::clearLoggedData()
{
  MELO_DEBUG_STREAM("Erasing logged data.");
    // Subindex 2 erases the data
  return sendSdoWriteUInt64(OD_DATALOGGING, 2, false, (uint64_t)1);
}

bool DriveEthercatDevice::refreshLoggedData()
{
  MELO_DEBUG_STREAM("Refreshing logged data.");
    // Subindex 1 refreshes the data
  return sendSdoWriteUInt64(OD_DATALOGGING, 1, false, (uint64_t)1);
}


bool DriveEthercatDevice::getMaxMotorVelocity(double & maxMotorVelocity)
{
  RecLock lock(mutex_);
  float maxMotorVelocityFloat = 0.0;
  if (!sendSdoRead(OD_DSP402_MOTOR_VELOCITY_MAX_ID, 0x00, false, maxMotorVelocityFloat)) {
    return false;
  }
  maxMotorVelocity = motorVelocityScalingInv_ * maxMotorVelocityFloat;
  return true;
}

bool DriveEthercatDevice::setMaxMotorVelocity(const double maxMotorVelocity)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting max motor velocity (" << maxMotorVelocity << " rad/s).");
  return sendSdoWrite(OD_DSP402_MOTOR_VELOCITY_MAX_ID, 0x00, false,
      static_cast<float>(motorVelocityScaling_ * maxMotorVelocity));
}

bool DriveEthercatDevice::getMaxJointTorque(double & maxJointTorque)
{
  RecLock lock(mutex_);
  float maxJointTorqueFloat = 0.0;
  if (!sendSdoRead(OD_DSP402_JOINT_TORQUE_MAX_ID, 0x00, false, maxJointTorqueFloat)) {
    return false;
  }
  maxJointTorque = jointTorqueScalingInv_ * maxJointTorqueFloat;
  return true;
}

bool DriveEthercatDevice::setMaxJointTorque(const double maxJointTorque)
{
  MELO_DEBUG_STREAM("Setting max joint torque (" << maxJointTorque << " Nm).");
  RecLock lock(mutex_);
  return sendSdoWrite(OD_DSP402_JOINT_TORQUE_MAX_ID, 0x00, false,
      static_cast<float>(jointTorqueScaling_ * maxJointTorque));
}

bool DriveEthercatDevice::getCurrentIntegratorSaturation(double & saturation)
{
  RecLock lock(mutex_);
  float saturationFloat = 0.0;
  if (!sendSdoRead(OD_CONFIG_CURRENT_INTEGRATOR_SATURATION, 0x00, false, saturationFloat)) {
    return false;
  }
  saturation = saturationFloat;
  return true;
}

bool DriveEthercatDevice::setCurrentIntegratorSaturation(const double saturation)
{
  MELO_DEBUG_STREAM("Setting current integrator saturation (" << saturation << " A).");
  RecLock lock(mutex_);
  return sendSdoWrite(OD_CONFIG_CURRENT_INTEGRATOR_SATURATION, 0x00, false,
      static_cast<float>(saturation));
}

bool DriveEthercatDevice::getJointTorqueIntegratorSaturation(double & saturation)
{
  RecLock lock(mutex_);
  float saturationFloat = 0.0;
  if (!sendSdoRead(OD_CONFIG_JOINT_TORQUE_INTEGRATOR_SATURATION, 0x00, false, saturationFloat)) {
    return false;
  }
  saturation = saturationFloat;
  return true;
}

bool DriveEthercatDevice::setJointTorqueIntegratorSaturation(const double saturation)
{
  MELO_DEBUG_STREAM("Setting joint torque integrator saturation (" << saturation << " Nm).");
  RecLock lock(mutex_);
  return sendSdoWrite(OD_CONFIG_JOINT_TORQUE_INTEGRATOR_SATURATION, 0x00, false,
      static_cast<float>(saturation));
}

bool DriveEthercatDevice::getDirection(int16_t & direction)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_DIRECTION_ID, 0x00, false, direction);
}

bool DriveEthercatDevice::setDirection(const int16_t direction)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting direction (" << direction << ").");
  return sendSdoWrite(OD_CONTROL_DIRECTION_ID, 0x00, false, direction);
}

bool DriveEthercatDevice::getJointPositionLimitsSoft(rsl_drive_sdk::common::Limits & limits)
{
  RecLock lock(mutex_);
  bool success = true;
  success &= sendSdoRead(OD_DSP402_SOFT_JOINT_POSITION_LIMIT_ID,
      OD_DSP402_SOFT_JOINT_POSITION_LIMIT_SID_MIN, false, limits.min());
  success &= sendSdoRead(OD_DSP402_SOFT_JOINT_POSITION_LIMIT_ID,
      OD_DSP402_SOFT_JOINT_POSITION_LIMIT_SID_MAX, false, limits.max());
  limits *= jointPositionScalingInv_;
  return success;
}

bool DriveEthercatDevice::setJointPositionLimitsSoft(const rsl_drive_sdk::common::Limits & limits)
{
  RecLock lock(mutex_);

  MELO_INFO_STREAM("Setting soft joint position limits ([" << limits.min() << ", " <<
      limits.max() << "] rad).");

  const auto direction = config_.getDirection();
  rsl_drive_sdk::common::Limits adjustedLimits = limits;
  if (direction) {
    adjustedLimits *= direction.value();
  }
  bool success = true;
  success &= sendSdoWrite(OD_DSP402_SOFT_JOINT_POSITION_LIMIT_ID,
      OD_DSP402_SOFT_JOINT_POSITION_LIMIT_SID_MIN, false,
      static_cast<double>(jointPositionScaling_ * adjustedLimits.min()));
  success &= sendSdoWrite(OD_DSP402_SOFT_JOINT_POSITION_LIMIT_ID,
      OD_DSP402_SOFT_JOINT_POSITION_LIMIT_SID_MAX, false,
      static_cast<double>(jointPositionScaling_ * adjustedLimits.max()));
  return success;
}

bool DriveEthercatDevice::getJointPositionLimitsHard(rsl_drive_sdk::common::Limits & limits)
{
  RecLock lock(mutex_);
  bool success = true;
  success &= sendSdoRead(OD_DSP402_HARD_JOINT_POSITION_LIMIT_ID,
      OD_DSP402_HARD_JOINT_POSITION_LIMIT_SID_MIN, false, limits.min());
  success &= sendSdoRead(OD_DSP402_HARD_JOINT_POSITION_LIMIT_ID,
      OD_DSP402_HARD_JOINT_POSITION_LIMIT_SID_MAX, false, limits.max());
  limits *= jointPositionScalingInv_;
  return success;
}

bool DriveEthercatDevice::setJointPositionLimitsHard(const rsl_drive_sdk::common::Limits & limits)
{
  RecLock lock(mutex_);

  MELO_DEBUG_STREAM("Setting hard joint position limits ([" << limits.min() << ", " <<
      limits.max() << "] rad).");
  const auto direction = config_.getDirection();
  rsl_drive_sdk::common::Limits adjustedLimits = limits;
  if (direction) {
    adjustedLimits *= direction.value();
  }
  bool success = true;
  success &= sendSdoWrite(OD_DSP402_HARD_JOINT_POSITION_LIMIT_ID,
      OD_DSP402_HARD_JOINT_POSITION_LIMIT_SID_MIN, false,
      static_cast<double>(jointPositionScaling_ * adjustedLimits.min()));
  success &= sendSdoWrite(OD_DSP402_HARD_JOINT_POSITION_LIMIT_ID,
      OD_DSP402_HARD_JOINT_POSITION_LIMIT_SID_MAX, false,
      static_cast<double>(jointPositionScaling_ * adjustedLimits.max()));
  return success;
}

bool DriveEthercatDevice::getControlGains(
  const rsl_drive_sdk::mode::ModeEnum mode,
  rsl_drive_sdk::mode::PidGainsF & pidGains)
{
  auto odIndex = modeEnumToOdIndex_.find(mode);
  if (odIndex == modeEnumToOdIndex_.end()) {
    MELO_ERROR_STREAM("Getting control gains is not supported for mode '" <<
        rsl_drive_sdk::mode::modeEnumToName(mode) << "'.");
    return false;
  }

  RecLock lock(mutex_);
  bool success = true;
  success &= sendSdoRead(odIndex->second, OD_GAINS_COMMON_SID_P, false, pidGains.getP());
  success &= sendSdoRead(odIndex->second, OD_GAINS_COMMON_SID_I, false, pidGains.getI());
  success &= sendSdoRead(odIndex->second, OD_GAINS_COMMON_SID_D, false, pidGains.getD());
  pidGains *= gainScalingInv_;
  return success;
}

bool DriveEthercatDevice::setControlGains(
  rsl_drive_sdk::mode::ModeEnum mode,
  const rsl_drive_sdk::mode::PidGainsF & pidGains)
{
  auto odIndex = modeEnumToOdIndex_.find(mode);
  if (odIndex == modeEnumToOdIndex_.end()) {
    MELO_ERROR_STREAM("Setting control gains is not supported for mode '" <<
        rsl_drive_sdk::mode::modeEnumToName(mode) << "'.");
    return false;
  }

  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting control gains for mode '" <<
      rsl_drive_sdk::mode::modeEnumToName(mode) << "'.");
  bool success = true;
  success &= sendSdoWrite(odIndex->second, OD_GAINS_COMMON_SID_P, false,
      static_cast<float>(gainScaling_ * pidGains.getP()));
  success &= sendSdoWrite(odIndex->second, OD_GAINS_COMMON_SID_I, false,
      static_cast<float>(gainScaling_ * pidGains.getI()));
  success &= sendSdoWrite(odIndex->second, OD_GAINS_COMMON_SID_D, false,
      static_cast<float>(gainScaling_ * pidGains.getD()));
  return success;
}

bool DriveEthercatDevice::getErrorStateBehavior(uint16_t & errorStateBehavior)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_FSM_ERROR_BEHAVIOR_ID, 0x00, false, errorStateBehavior);
}

bool DriveEthercatDevice::setErrorStateBehavior(const uint16_t errorStateBehavior)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting error state behavior (" << errorStateBehavior << ").");
  return sendSdoWrite(OD_FSM_ERROR_BEHAVIOR_ID, 0x00, false, errorStateBehavior);
}

bool DriveEthercatDevice::getImuEnable(bool & enabled)
{
  RecLock lock(mutex_);
  uint32_t enabledInt = 0;
  bool isOk = sendSdoRead(OD_CONTROL_IMU_CONFIG_ID, OD_CONTROL_IMU_CONFIG_SID_ENABLE, false,
      enabledInt);
  enabled = (enabledInt == 1);
  return isOk;
}

bool DriveEthercatDevice::setImuEnable(const bool enable)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM((enable ? "En" : "Dis") << "abling IMU.");
  return sendSdoWrite(OD_CONTROL_IMU_CONFIG_ID, OD_CONTROL_IMU_CONFIG_SID_ENABLE, false,
           (uint32_t)(enable ? 1 : 0));
}

bool DriveEthercatDevice::getImuAccelerometerRange(uint32_t & range)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_IMU_CONFIG_ID, OD_CONTROL_IMU_CONFIG_SID_ACCELEROMETER_RANGE, false,
      range);
}

bool DriveEthercatDevice::setImuAccelerometerRange(const uint32_t range)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting IMU accelerometer range (" << range << ").");
  return sendSdoWrite(OD_CONTROL_IMU_CONFIG_ID, OD_CONTROL_IMU_CONFIG_SID_ACCELEROMETER_RANGE,
      false, range);
}

bool DriveEthercatDevice::getImuGyroscopeRange(uint32_t & range)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_IMU_CONFIG_ID, OD_CONTROL_IMU_CONFIG_SID_GYROSCOPE_RANGE, false,
      range);
}

bool DriveEthercatDevice::setImuGyroscopeRange(const uint32_t range)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting IMU gyroscope range (" << range << ").");
  return sendSdoWrite(OD_CONTROL_IMU_CONFIG_ID, OD_CONTROL_IMU_CONFIG_SID_GYROSCOPE_RANGE, false,
      range);
}

bool DriveEthercatDevice::getFanMode(uint32_t & mode)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_FAN_ID, OD_CONTROL_FAN_SID_MODE, false, mode);
}

bool DriveEthercatDevice::setFanMode(const uint32_t mode)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting fan mode (" << mode << ").");
  return sendSdoWrite(OD_CONTROL_FAN_ID, OD_CONTROL_FAN_SID_MODE, false, mode);
}

bool DriveEthercatDevice::getFanIntensity(uint32_t & intensity)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_FAN_ID, OD_CONTROL_FAN_SID_INTENSITY, false, intensity);
}

bool DriveEthercatDevice::setFanIntensity(const uint32_t intensity)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting fan intensity (" << intensity << ").");
  return sendSdoWrite(OD_CONTROL_FAN_ID, OD_CONTROL_FAN_SID_INTENSITY, false, intensity);
}

bool DriveEthercatDevice::getFanLowerTemperature(float & temperature)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_FAN_ID, OD_CONTROL_FAN_SID_LOWER_TEMPERATURE, false, temperature);
}

bool DriveEthercatDevice::setFanLowerTemperature(const float temperature)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting fan lower temperature (" << temperature << ").");
  return sendSdoWrite(OD_CONTROL_FAN_ID, OD_CONTROL_FAN_SID_LOWER_TEMPERATURE, false, temperature);
}

bool DriveEthercatDevice::getFanUpperTemperature(float & temperature)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_FAN_ID, OD_CONTROL_FAN_SID_UPPER_TEMPERATURE, false, temperature);
}

bool DriveEthercatDevice::setFanUpperTemperature(const float temperature)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting fan upper temperature (" << temperature << ").");
  return sendSdoWrite(OD_CONTROL_FAN_ID, OD_CONTROL_FAN_SID_UPPER_TEMPERATURE, false, temperature);
}

bool DriveEthercatDevice::setBrakeMode(const bool mode)
{
  RecLock lock(mutex_);
  if (mode) {
    MELO_DEBUG_STREAM("Enabling Brake Mode")
    if(!sendSdoWrite(OD_CONTROL_BRAKE_ID, OD_CONTROL_BRAKE_SID_CTRL, false,
        OD_CONTROL_BRAKE_SID_CTRL_VAL_ENABLE))
    {
      MELO_ERROR_STREAM("Could not enable brake. Fan enabled?");
      return false;
    } else {
      MELO_DEBUG_STREAM("Brake output active.")
      return true;
    }
  } else {
    MELO_DEBUG_STREAM("Disabling Brake Mode")
    if(!sendSdoWrite(OD_CONTROL_BRAKE_ID, OD_CONTROL_BRAKE_SID_CTRL, false,
        OD_CONTROL_BRAKE_SID_CTRL_VAL_DISABLE))
    {
      MELO_ERROR_STREAM("Could not disable brake.");
      return false;
    } else {
      MELO_DEBUG_STREAM("Brake output disabled.")
      return true;
    }
  }
}

bool DriveEthercatDevice::getBrakeMode(bool & mode)
{
  RecLock lock(mutex_);
  uint16_t m;
  if (!sendSdoRead(OD_CONTROL_BRAKE_ID, OD_CONTROL_BRAKE_SID_CTRL, false, m)) {
    return false;
  }
  if (m == 0) {
    mode = false;
  } else if (m == 1) {
    mode = true;
  } else {
    MELO_ERROR_STREAM("Brake mode not recognized. Received: " << m)
    mode = false;
    return false;
  }
  return true;
}

bool DriveEthercatDevice::setBrakeDuty(const float d)
{
    // Expected duty cycle range: 0...1
  RecLock lock(mutex_);
  float d_lim;
  if (d > 1.0f) {
    d_lim = 1.0f;
  } else if (d < 0.0f) {
    d_lim = 0.0f;
  } else {
    d_lim = d;
  }
  uint16_t d_disc = (uint16_t)(65535.0f * d_lim);
  return sendSdoWrite(OD_CONTROL_BRAKE_ID, OD_CONTROL_BRAKE_SID_DUTY, false, d_disc);
}

bool DriveEthercatDevice::getBrakeDuty(float & d)
{
  RecLock lock(mutex_);
  uint16_t d_disc;
  if(!sendSdoRead(OD_CONTROL_BRAKE_ID, OD_CONTROL_BRAKE_SID_DUTY, false, d_disc)) {
    return false;
  }
  d = (float)d_disc / 65535.0f;
  return true;
}

bool DriveEthercatDevice::getGearJointVelocityFilterType(uint32_t & type)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_GEAR_JOINT_VELOCITY_ID,
      OD_FILTER_GEAR_JOINT_VELOCITY_SID_FILTER_TYPE, false, type);
}

bool DriveEthercatDevice::setGearJointVelocityFilterType(const uint32_t type)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting gear joint velicity filter type (" << type << ").");
  return sendSdoWrite(OD_FILTER_GEAR_JOINT_VELOCITY_ID,
      OD_FILTER_GEAR_JOINT_VELOCITY_SID_FILTER_TYPE, false, type);
}

bool DriveEthercatDevice::getGearJointVelocityKfNoiseVariance(float & variance)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_GEAR_JOINT_VELOCITY_ID,
      OD_FILTER_GEAR_JOINT_VELOCITY_SID_KF_NOISE_VARIANCE, false, variance);
}

bool DriveEthercatDevice::setGearJointVelocityKfNoiseVariance(const float variance)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting gear joint velicity KF noies variance (" << variance << ").");
  return sendSdoWrite(OD_FILTER_GEAR_JOINT_VELOCITY_ID,
      OD_FILTER_GEAR_JOINT_VELOCITY_SID_KF_NOISE_VARIANCE, false, variance);
}

bool DriveEthercatDevice::getGearJointVelocityKfLambda2(float & lambda)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_GEAR_JOINT_VELOCITY_ID,
      OD_FILTER_GEAR_JOINT_VELOCITY_SID_KF_LAMBDA_2, false, lambda);
}

bool DriveEthercatDevice::setGearJointVelocityKfLambda2(const float lambda)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting gear joint velicity KF lambda^2 (" << lambda << ").");
  return sendSdoWrite(OD_FILTER_GEAR_JOINT_VELOCITY_ID,
      OD_FILTER_GEAR_JOINT_VELOCITY_SID_KF_LAMBDA_2, false, lambda);
}

bool DriveEthercatDevice::getGearJointVelocityKfGamma(float & gamma)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_GEAR_JOINT_VELOCITY_ID, OD_FILTER_GEAR_JOINT_VELOCITY_SID_KF_GAMMA,
      false, gamma);
}

bool DriveEthercatDevice::setGearJointVelocityKfGamma(const float gamma)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting gear joint velicity KF gamma (" << gamma << ").");
  return sendSdoWrite(OD_FILTER_GEAR_JOINT_VELOCITY_ID, OD_FILTER_GEAR_JOINT_VELOCITY_SID_KF_GAMMA,
      false, gamma);
}

bool DriveEthercatDevice::getGearJointVelocityIirAlpha(float & alpha)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_GEAR_JOINT_VELOCITY_ID, OD_FILTER_GEAR_JOINT_VELOCITY_SID_IIR_ALPHA,
      false, alpha);
}

bool DriveEthercatDevice::setGearJointVelocityIirAlpha(const float alpha)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting gear joint velicity IIR alpha (" << alpha << ").");
  return sendSdoWrite(OD_FILTER_GEAR_JOINT_VELOCITY_ID, OD_FILTER_GEAR_JOINT_VELOCITY_SID_IIR_ALPHA,
      false, alpha);
}

bool DriveEthercatDevice::getGearJointVelocityEmaAlpha(float & alpha)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_GEAR_JOINT_VELOCITY_ID, OD_FILTER_GEAR_JOINT_VELOCITY_SID_EMA_ALPHA,
      false, alpha);
}

bool DriveEthercatDevice::setGearJointVelocityEmaAlpha(const float alpha)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting gear joint velicity IIR alpha (" << alpha << ").");
  return sendSdoWrite(OD_FILTER_GEAR_JOINT_VELOCITY_ID, OD_FILTER_GEAR_JOINT_VELOCITY_SID_EMA_ALPHA,
      false, alpha);
}

bool DriveEthercatDevice::getJointVelocityForAccelerationFilterType(uint32_t & type)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID,
      OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_FILTER_TYPE, false, type);
}

bool DriveEthercatDevice::setJointVelocityForAccelerationFilterType(const uint32_t type)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting joint velocity for acceleration filter type (" << type << ").");
  return sendSdoWrite(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID,
      OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_FILTER_TYPE, false, type);
}

bool DriveEthercatDevice::getJointVelocityForAccelerationKfNoiseVariance(float & variance)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID,
      OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_KF_NOISE_VARIANCE, false, variance);
}

bool DriveEthercatDevice::setJointVelocityForAccelerationKfNoiseVariance(const float variance)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting joint velocity for acceleration KF noies variance (" << variance <<
      ").");
  return sendSdoWrite(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID,
      OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_KF_NOISE_VARIANCE, false, variance);
}

bool DriveEthercatDevice::getJointVelocityForAccelerationKfLambda2(float & lambda)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID,
      OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_KF_LAMBDA_2, false, lambda);
}

bool DriveEthercatDevice::setJointVelocityForAccelerationKfLambda2(const float lambda)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting joint velocity for acceleration KF lambda^2 (" << lambda << ").");
  return sendSdoWrite(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID,
      OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_KF_LAMBDA_2, false, lambda);
}

bool DriveEthercatDevice::getJointVelocityForAccelerationKfGamma(float & gamma)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID,
      OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_KF_GAMMA, false, gamma);
}

bool DriveEthercatDevice::setJointVelocityForAccelerationKfGamma(const float gamma)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting joint velocity for acceleration KF gamma (" << gamma << ").");
  return sendSdoWrite(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID,
      OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_KF_GAMMA, false, gamma);
}

bool DriveEthercatDevice::getJointVelocityForAccelerationIirAlpha(float & alpha)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID,
      OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_IIR_ALPHA, false, alpha);
}

bool DriveEthercatDevice::setJointVelocityForAccelerationIirAlpha(const float alpha)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting joint velocity for acceleration KF gamma (" << alpha << ").");
  return sendSdoWrite(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID,
      OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_IIR_ALPHA, false, alpha);
}


bool DriveEthercatDevice::getJointVelocityForAccelerationEmaAlpha(float & alpha)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID,
      OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_EMA_ALPHA, false, alpha);
}

bool DriveEthercatDevice::setJointVelocityForAccelerationEmaAlpha(const float alpha)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting joint velocity for acceleration IIR alpha (" << alpha << ").");
  return sendSdoWrite(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID,
      OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_EMA_ALPHA, false, alpha);
}

bool DriveEthercatDevice::getJointAccelerationFilterType(uint32_t & type)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID,
      OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_FILTER_TYPE, false, type);
}

bool DriveEthercatDevice::setJointAccelerationFilterType(const uint32_t type)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting joint acceleration filter type (" << type << ").");
  return sendSdoWrite(OD_FILTER_JOINT_ACCELERATION_ID, OD_FILTER_JOINT_ACCELERATION_SID_FILTER_TYPE,
      false, type);
}

bool DriveEthercatDevice::getJointAccelerationKfNoiseVariance(float & variance)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_ACCELERATION_ID,
      OD_FILTER_JOINT_ACCELERATION_SID_KF_NOISE_VARIANCE, false, variance);
}

bool DriveEthercatDevice::setJointAccelerationKfNoiseVariance(const float variance)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting joint acceleration KF noies variance (" << variance << ").");
  return sendSdoWrite(OD_FILTER_JOINT_ACCELERATION_ID,
      OD_FILTER_JOINT_ACCELERATION_SID_KF_NOISE_VARIANCE, false, variance);
}

bool DriveEthercatDevice::getJointAccelerationKfLambda2(float & lambda)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_ACCELERATION_ID, OD_FILTER_JOINT_ACCELERATION_SID_KF_LAMBDA_2,
      false, lambda);
}

bool DriveEthercatDevice::setJointAccelerationKfLambda2(const float lambda)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting joint acceleration KF lambda^2 (" << lambda << ").");
  return sendSdoWrite(OD_FILTER_JOINT_ACCELERATION_ID, OD_FILTER_JOINT_ACCELERATION_SID_KF_LAMBDA_2,
      false, lambda);
}

bool DriveEthercatDevice::getJointAccelerationKfGamma(float & gamma)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_ACCELERATION_ID, OD_FILTER_JOINT_ACCELERATION_SID_KF_GAMMA,
      false, gamma);
}

bool DriveEthercatDevice::setJointAccelerationKfGamma(const float gamma)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting joint acceleration KF gamma (" << gamma << ").");
  return sendSdoWrite(OD_FILTER_JOINT_ACCELERATION_ID, OD_FILTER_JOINT_ACCELERATION_SID_KF_GAMMA,
      false, gamma);
}

bool DriveEthercatDevice::getJointAccelerationIirAlpha(float & alpha)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_ACCELERATION_ID, OD_FILTER_JOINT_ACCELERATION_SID_IIR_ALPHA,
      false, alpha);
}

bool DriveEthercatDevice::setJointAccelerationIirAlpha(const float alpha)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting joint acceleration IIR Alpha (" << alpha << ").");
  return sendSdoWrite(OD_FILTER_JOINT_ACCELERATION_ID, OD_FILTER_JOINT_ACCELERATION_SID_IIR_ALPHA,
      false, alpha);
}


bool DriveEthercatDevice::getJointAccelerationEmaAlpha(float & alpha)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_ACCELERATION_ID, OD_FILTER_JOINT_ACCELERATION_SID_EMA_ALPHA,
      false, alpha);
}

bool DriveEthercatDevice::setJointAccelerationEmaAlpha(const float alpha)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting joint acceleration IIR alpha (" << alpha << ").");
  return sendSdoWrite(OD_FILTER_JOINT_ACCELERATION_ID, OD_FILTER_JOINT_ACCELERATION_SID_EMA_ALPHA,
      false, alpha);
}

bool DriveEthercatDevice::writeConfiguration()
{
  RecLock lock(mutex_);
  MELO_INFO_STREAM("Writing configuration to flash.");
  return sendSdoWrite(OD_FLASH_WRITE_CONFIGURATION_ID, 0x00, false,
      static_cast<uint16_t>(OD_FLASH_WRITE_CONFIGURATION_ID_VAL_RUN));
}

bool DriveEthercatDevice::setDGainFilterCutoffFrequency(const float freq)
{
  RecLock lock(mutex_);
  return sendSdoWrite(OD_FILTER_D_GAIN, 0, false, freq);
}

bool DriveEthercatDevice::getDGainFilterCutoffFrequency(float & freq)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_D_GAIN, 0, false, freq);
}

bool DriveEthercatDevice::getStatuswordSdo(rsl_drive_sdk::Statusword & statusword)
{
  RecLock lock(mutex_);
  uint32_t data = 0;
  if (!sendSdoRead(OD_DSP402_STATUS_WORD_ID, 0x00, false, data)) {
    return false;
  }
  statusword.setData(data);
  return true;
}

rsl_drive_sdk::Statusword DriveEthercatDevice::getStatusword()
{
  return statusword_;
}


void DriveEthercatDevice::setControlword(const uint16_t controlwordId)
{
  MELO_DEBUG_STREAM("Setting controlword (" << controlwordId << ").");
  RecLock lock(controlwordIdMutex_);
  controlwordId_ = controlwordId;
}

void DriveEthercatDevice::resetControlword()
{
  MELO_DEBUG_STREAM("Resetting controlword.");
  RecLock lock(controlwordIdMutex_);
  controlwordId_ = 0;
}

void DriveEthercatDevice::stageFreeze()
{

  Command command;
  command.setModeEnum(mode::ModeEnum::Freeze);
  setCommand(command);

}

void DriveEthercatDevice::setCommand(const rsl_drive_sdk::Command & command)
{
  RecLock lock(commandMutex_);
  command_ = command;
}

void DriveEthercatDevice::getReading(rsl_drive_sdk::ReadingExtended & reading)
{
  RecLock lock(readingMutex_);
  reading = reading_;
}

bool DriveEthercatDevice::getRtdlEnabled(bool & enabled)
{
  RecLock lock(mutex_);
  uint16_t value;
  bool isOk = sendSdoRead(OD_CONTROL_RTDL_CONTROL_ID, OD_CONTROL_RTDL_CONTROL_SID_ENABLE, false,
      value);
  enabled = (value == OD_CONTROL_RTDL_CONTROL_SID_ENABLE_VAL_ENABLE);
  return isOk;
}

bool DriveEthercatDevice::setRtdlEnable(const bool enable)
{
  RecLock lock(mutex_);
  MELO_INFO_STREAM((enable ? "En" : "Dis") << "abling RTDL.");
  bool isOk = sendSdoWrite(OD_CONTROL_RTDL_CONTROL_ID, OD_CONTROL_IMU_CONFIG_SID_ENABLE, false,
      (enable ? OD_CONTROL_RTDL_CONTROL_SID_ENABLE_VAL_ENABLE :
      OD_CONTROL_RTDL_CONTROL_SID_ENABLE_VAL_DISABLE));
  if (isOk) {
    isRtdlRunning_ = enable;
  }
  return isOk;
}

bool DriveEthercatDevice::setRtdlCommand(const uint16_t command)
{
  RecLock lock(mutex_);
  if (command > 3) {
    MELO_ERROR_STREAM("Invalid RTDL command (" << command << ").");
    return false;
  }
  MELO_DEBUG_STREAM("Setting RTDL command (" << command << ").");
  return sendSdoWrite(OD_CONTROL_RTDL_CONTROL_ID, OD_CONTROL_RTDL_CONTROL_SID_COMMAND, false,
      command);
}

bool DriveEthercatDevice::getRtdlStatus(uint16_t & status)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_RTDL_CONTROL_ID, OD_CONTROL_RTDL_CONTROL_SID_STATUS, false, status);
}

bool DriveEthercatDevice::getRtdlLoggingFrequency(uint16_t & frequency)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_RTDL_CONTROL_ID, OD_CONTROL_RTDL_CONTROL_SID_LOGGING_FREQUENCY,
      false, frequency);
}

bool DriveEthercatDevice::setRtdlLoggingFrequency(const uint16_t frequency)
{
  RecLock lock(mutex_);
  if (frequency > 1) {
    MELO_ERROR_STREAM("Invalid RTDL logging frequency (" << frequency << ").");
    return false;
  }
  MELO_DEBUG_STREAM("Setting RTDL logging frequency (" << frequency << ").");
  return sendSdoWrite(OD_CONTROL_RTDL_CONTROL_ID, OD_CONTROL_RTDL_CONTROL_SID_LOGGING_FREQUENCY,
      false, frequency);
}

bool DriveEthercatDevice::getRtdlStreamingFrequency(uint16_t & frequency)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_RTDL_CONTROL_ID, OD_CONTROL_RTDL_CONTROL_SID_STREAMING_FREQUENCY,
      false, frequency);
}

bool DriveEthercatDevice::setRtdlStreamingFrequency(const uint16_t frequency)
{
  RecLock lock(mutex_);
  MELO_DEBUG_STREAM("Setting RTDL streaming frequency (" << frequency << " Hz).");
  return sendSdoWrite(OD_CONTROL_RTDL_CONTROL_ID, OD_CONTROL_RTDL_CONTROL_SID_STREAMING_FREQUENCY,
      false, frequency);
}

bool DriveEthercatDevice::getRtdlLastTimestamp(uint64_t & timestamp)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_RTDL_LAST_TIMESTAMP_ID, 0x0, false, timestamp);
}

bool DriveEthercatDevice::configurePdo(const PdoTypeEnum pdoTypeEnum)
{
  if (pdoTypeEnum == PdoTypeEnum::NA) {
    MELO_ERROR_STREAM("Invalid EtherCAT PDO Type.");
    return false;
  }

    // If PDO setup is already active, return.
  if (pdoTypeEnum == getCurrentPdoTypeEnum()) {
    return true;
  }

  {
    RecLock lock(mutex_);
    if (!sendSdoWrite(OD_SWITCH_PDO_ID, OD_SWITCH_PDO_SID, false,
        pdoInfos_.at(pdoTypeEnum).moduleId_))
    {
      return false;
    }
  }

  currentPdoTypeEnum_ = pdoTypeEnum;
  return true;
}


bool DriveEthercatDevice::getCalibrationTypeEnum(
  rsl_drive_sdk::calibration::CalibrationTypeEnum & calibrationTypeEnum)
{
  uint16_t id;
  if(getCalibrationTypeId(id)) {
    calibrationTypeEnum = rsl_drive_sdk::calibration::calibrationTypeIdToEnum(id);
    return true;
  }
  return false;
}

bool DriveEthercatDevice::setCalibrationTypeEnum(
  const rsl_drive_sdk::calibration::CalibrationTypeEnum calibrationTypeEnum)
{
  return setCalibrationTypeId(rsl_drive_sdk::calibration::calibrationTypeEnumToId(
      calibrationTypeEnum));
}

bool DriveEthercatDevice::getCalibrationTypeId(uint16_t & calibrationTypeId)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_CALIB_SELECTION_ID, 0x00, false, calibrationTypeId);
}

bool DriveEthercatDevice::setCalibrationTypeId(const uint16_t calibrationTypeId)
{
  RecLock lock(mutex_);
  return sendSdoWrite(OD_CALIB_SELECTION_ID, 0x00, false, calibrationTypeId);
}

bool DriveEthercatDevice::getLockStatus(uint32_t & status)
{
  RecLock lock(mutex_);
  return sendSdoRead(OD_VARIOUS_PASSWORD_ID, 3, false, status);
}

bool DriveEthercatDevice::sendPassword(const std::string & password)
{
  RecLock lock(mutex_);
  return sendSdoWriteGeneric(std::to_string(OD_VARIOUS_PASSWORD_ID),
                               "1", "string", password);
}

// Add callbacks
void DriveEthercatDevice::addReadingCb(const ReadingCb & readingCb, const int priority)
{
  readingCbs_.insert({priority, readingCb});
}
void DriveEthercatDevice::addErrorCb(const ErrorCb & errorCb, const int priority)
{
  errorCbs_.insert({priority, errorCb});
}
void DriveEthercatDevice::addErrorRecoveredCb(
  const ErrorRecoveredCb & errorRecoveredCb,
  const int priority)
{
  errorRecoveredCbs_.insert({priority, errorRecoveredCb});
}
void DriveEthercatDevice::addFatalCb(const FatalCb & fatalCb, const int priority)
{
  fatalCbs_.insert({priority, fatalCb});
}
void DriveEthercatDevice::addFatalRecoveredCb(
  const FatalRecoveredCb & fatalRecoveredCb,
  const int priority)
{
  fatalRecoveredCbs_.insert({priority, fatalRecoveredCb});
}
void DriveEthercatDevice::addDeviceDisconnectedCb(
  const DeviceDisconnectedCb & deviceDisconnectedCb,
  const int priority)
{
  deviceDisconnectedCbs_.insert({priority, deviceDisconnectedCb});
}
void DriveEthercatDevice::addDeviceReconnectedCb(
  const DeviceReconnectedCb & deviceReconnectedCb,
  const int priority)
{
  deviceReconnectedCbs_.insert({priority, deviceReconnectedCb});
}

// execute the added callbacks
void DriveEthercatDevice::errorCb()
{
  for (const auto & errorCb : errorCbs_) {
    errorCb.second(getName());
  }
}

void DriveEthercatDevice::errorRecoveredCb()
{
  clearGoalStateEnum();
  for (const auto & errorRecoveredCb : errorRecoveredCbs_) {
    errorRecoveredCb.second(getName());
  }
}

bool DriveEthercatDevice::deviceIsInErrorState()
{
  return statusword_.getStateEnum() == fsm::StateEnum::Error;
}

void DriveEthercatDevice::fatalCb()
{
  for (const auto & fatalCb : fatalCbs_) {
    fatalCb.second(getName());
  }
}

void DriveEthercatDevice::fatalRecoveredCb()
{
  clearGoalStateEnum();
  for (const auto & fatalRecoveredCb : fatalRecoveredCbs_) {
    fatalRecoveredCb.second(getName());
  }
}

bool DriveEthercatDevice::deviceIsInFatalState()
{
  return statusword_.getStateEnum() == fsm::StateEnum::Fatal;
}

void DriveEthercatDevice::deviceDisconnectedCb()
{
  statuswordRequested_ = false;
  clearGoalStateEnum();
  for (const auto & deviceDisconnectedCb : deviceDisconnectedCbs_) {
    deviceDisconnectedCb.second(getName());
  }

    // Set statusword and reading accordingly.
    //  statusword_.resetData();
  ReadingExtended reading;
  const std::chrono::high_resolution_clock::time_point stamp =
    std::chrono::high_resolution_clock::now();

  {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    StateExtended & state = reading_.getState();
    state.setStamp(stamp);
    state.setStatusword(statusword_);
    if (getConfiguration().getSetReadingToNanOnDisconnect()) {
      static constexpr auto _NAN_ = std::numeric_limits<double>::quiet_NaN();
            // State.
      state.setCurrent(_NAN_);
      state.setGearPosition(_NAN_);
      state.setGearVelocity(_NAN_);
      state.setJointPosition(_NAN_);
      state.setJointVelocity(_NAN_);
      state.setJointAcceleration(_NAN_);
      state.setJointTorque(_NAN_);

            // Extended state.
      state.setMotorPosition(_NAN_);
      state.setMotorVelocity(_NAN_);
      state.setGearPositionTicks(0);
      state.setJointPositionTicks(0);
      state.setTemperature(_NAN_);
      state.setVoltage(_NAN_);
      state.setTimestamp(0);
      state.setDesiredCurrentD(_NAN_);
      state.setMeasuredCurrentD(_NAN_);
      state.setDesiredCurrentQ(_NAN_);
      state.setMeasuredCurrentQ(_NAN_);
      state.setMeasuredCurrentPhaseU(_NAN_);
      state.setMeasuredVoltagePhaseU(_NAN_);
      state.setMeasuredCurrentPhaseV(_NAN_);
      state.setMeasuredVoltagePhaseV(_NAN_);
      state.setMeasuredCurrentPhaseW(_NAN_);
      state.setMeasuredVoltagePhaseW(_NAN_);

    }
    reading = reading_;
  }

    // External reading callbacks.
  for (const auto & readingCb : readingCbs_) {
    readingCb.second(getName(), reading);
  }
}

void DriveEthercatDevice::deviceReconnectedCb()
{
  setFSMGoalState(getConfiguration().getGoalStateEnumStartup(), false, 5.0, 100.0);
  for (const auto & deviceReconnectedCb : deviceReconnectedCbs_) {
    deviceReconnectedCb.second(getName());
  }
}

bool DriveEthercatDevice::isWithinJointPositionLimitsSoft() const
{
  return statusword_.hasErrorJointPositionLimitsSoft();
}

bool DriveEthercatDevice::isWithinJointPositionLimitsHard() const
{
  return statusword_.hasFatalJointPositionLimitsHard();
}

bool DriveEthercatDevice::sendSdoReadStringCustom(const uint16_t index, std::string & value)
{
  const uint8_t lenSubindex = 0;
  const uint8_t dataSubindex = 1;

    // Read number of subindices.
  uint8_t nSubindex = 0;
  if (!sendSdoRead(index, lenSubindex, false, nSubindex)) {
    return false;
  }

    // Read char casted as an uint32 array.
  const uint8_t nChars = nSubindex * sizeof(uint32_t) / sizeof(char);
  std::vector<char> arrayChar(nChars + 1, 0);

  for (uint8_t i = 0; i < nSubindex; i++) {
    if (!sendSdoRead(index, dataSubindex + i, false, ((uint32_t *)arrayChar.data())[i])) {  //NOTE: This might lead to alignment issues on ARM platforms
      return false;
    }
  }
  arrayChar[nChars] = '\0';

    // Transform char array to string.
  value = std::string(arrayChar.begin(), arrayChar.end());
  return true;
}

bool DriveEthercatDevice::sendSdoWriteStringCustom(const uint16_t index, const std::string & value)
{
  const uint8_t lenSubindex = 0;
  const uint8_t dataSubindex = 1;

    // Read number of subindices.
  uint8_t nSubindex = 0;
  if (!sendSdoRead(index, lenSubindex, false, nSubindex)) {
    return false;
  }

    // Check if the string is not too big.
  const uint8_t nChars = nSubindex * sizeof(uint32_t) / sizeof(char);
  if (value.size() > nChars) {
    MELO_ERROR_STREAM("[DriveEthercatDevice::sendSdoWriteStringCustom] (Drive '"
                          << name_ << "') String is too big ("
                          << value.size() << " > " << (uint16_t)nChars
                          << "), cannot be written.");
    return false;
  }

    // Transform string to char array.
  std::vector<char> arrayChar(nChars, 0);
  for (uint8_t i = 0; i < nChars; i++) {
    arrayChar[i] = (i < value.size()) ? value[i] : '\0';
  }

    // Write char casted as an uint32 array.
  for (uint8_t i = 0; i < nSubindex; i++) {
    if (!sendSdoWrite(index, dataSubindex + i, false, ((uint32_t *)arrayChar.data())[i])) {  //NOTE: This might lead to alignment issues on ARM platforms
      return false;
    }
  }

  return true;
}


} // rsl_drive_sdk_ethercat
