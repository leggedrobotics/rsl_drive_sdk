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
#include <mutex>
#include <unistd.h>
#include <unordered_map>
#include <assert.h>

// rsl_drive_sdk
#include "rsl_drive_sdk/Drive.hpp"
#include "rsl_drive_sdk/thread_sleep.hpp"

// message logger
#include <message_logger/message_logger.hpp>


namespace rsl_drive_sdk
{

//! Class implementing a manager for multiple Drives.
class DriveCollection
{
public:
  typedef std::vector<DriveEthercatDevice::SharedPtr> DrivesVector;
  typedef std::shared_ptr<DriveCollection> DriveCollectionPtr;

  //! Flag indicating whether a shutdown has been requested.
  std::atomic<bool> shutdownRequested_;

  //! List of Drives.
  DrivesVector drives_;

public:
  /*!
   * @name Setup and configuration
   *
   * Methods to set up and configure the manager.
   */
  //@{

  /*!
   * Constructor.
   */
  DriveCollection(const DrivesVector & drives);

  /*!
   * Destructor.
   */
  virtual ~DriveCollection();


  /*!
   * Add an rsl_drive_sdk to the vector of rsl_drive_sdks.
   * @param pointer to the new rsl_drive_sdk
   * @return True if successful.
   */
  bool addDrive(const DriveEthercatDevice::SharedPtr & drive);

  /*!
   * Check if a Drive with a given name exists.
   * @param name Name of the Frive.
   * @return True if existing.
   */
  bool driveExists(const std::string & name) const;

  /*!
   * Get an Drive by name. If unsure, first check if the Drive exists with rsl_drive_sdkExists(..).
   * @param name Name of the Drive.
   * @return Pointer to the Drive.
   */
  DriveEthercatDevice::SharedPtr getDrive(const std::string & name) const;

  /*!
   * Get all Drives.
   * @return List of all Drives.
   */
  DrivesVector getDrives() const;

  /*!
   * Get the number of Drives.
   * @return Number of Drives.
   */
  size_t getNumberOfDrives() const;

  //@}

  /*!
   * @name Execution
   *
   * Methods to start up, update and shut down the manager.
   */
  //@{

  /*!
   * @name State Machine
   *
   * Methods to interact with the device state machines.
   */
  //@{

  /*!
   * Set the goal state of the state machines of all devices.
   * @param goalStateEnum     Goal state.
   * @param reachState        True: Block until the devices are in the goal state, False: return after setting the goal state.
   * @param timeout           Maximal blocking time.
   * @param checkingFrequency Frequency at which is checked, whether the devices have reached the goal state.
   * @return True if non-blocking, or blocking and the devices reached the goal state within the timeout.
   */
  bool setGoalStatesEnum(
    const fsm::StateEnum goalStateEnum,
    const bool reachStates = false,
    const double timeout = 5.0,
    const double checkingFrequency = 100.0);

  /*!
   * Clear the current goal state of the state machines of all devices.
   */
  void clearGoalStatesEnum();

  /*!
   * Check if all devices are connected.
   * @return True if all devices are connected.
   */
  bool allDevicesAreConnected() const;

  /*!
   * Check if all devices are in a given state.
   * @param stateEnum State to check.
   * @return True if all devices are in the given state.
   */
  bool allDevicesAreInTheState(const fsm::StateEnum stateEnum) const;

  /*!
   * Check if all devices if the are in the same state and get it.
   * @param stateEnum Return argument, will contain the common state. NA if not all devices are in the same state.
   * @return True if all devices are in the same state.
   */
  bool allDevicesAreInTheSameState(fsm::StateEnum & stateEnum) const;

  /*!
   * Check if all devices are in a given mode.
   * @param modeEnum Mode to check.
   * @return True if all devices are in the given mode.
   */
  bool allDevicesAreInTheMode(const mode::ModeEnum modeEnum) const;

  /*!
   * Check if all devices if the are in the same mode and get it.
   * @param modeEnum Return argument, will contain the common mode. NA if not all devices are in the same mode.
   * @return True if all devices are in the same mode.
   */
  bool allDevicesAreInTheSameMode(mode::ModeEnum & modeEnum) const;

  /*!
   * Check if no device is in the Error state.
   * @return True if no device is in the Error state.
   */
  bool noDeviceIsInErrorState() const;

  /*!
   * Check if no device is in the Fatal state.
   * @return True if no device is in the Fatal state.
   */
  bool noDeviceIsInFatalState() const;

  /*!
   * Check if all devices are within the soft joint position limits.
   * @return True if all devices are within the soft joint position limits.
   */
  bool allDevicesAreWithinJointPositionLimitsSoft() const;

  /*!
   * Check if all devices are within the hard joint position limits.
   * @return True if all devices are within the hard joint position limits.
   */
  bool allDevicesAreWithinJointPositionLimitsHard() const;

  //@}

  /*!
   * @name Calibration
   *
   * Methods to interface with the device calibrations.
   */
  //@{

  /*!
   * Run a calibration for a given device.
   * @param deviceName                                Name of the device to run the calibration.
   * @param calibrationModeEnum                       Mode of the calibration.
   * @param gearAndJointEncoderHomingAbsolute         In case of the gear and joint encoder homing calibration, this flag indicates whether it shall be done absolute or relative.
   * @param gearAndJointEncoderHomingNewJointPosition In case of the gear and joint encoder homing calibration, this is the new joint position.
   */
  virtual bool calibrate(
    const std::string & deviceName,
    const calibration::CalibrationModeEnum calibrationModeEnum,
    const bool gearAndJointEncoderHomingAbsolute = true,
    const double gearAndJointEncoderHomingNewJointPosition = 0.0);

  //@}

  /*!
   * @name Control
   *
   * Methods to run controllers on the devices.
   */
  //@{

  /*!
   * Calls Drive::sendControlwords for all drives.
   * @param controlwordId ID of the controlword to send.
   */
  void sendControlwords(const uint16_t controlwordId);

  /*!
   * Calls Drive::stageDisable for all drives.
   */
  void stageDisables();

  /*!
   * Calls Drive::stageFreezes for all drives.
   */
  void stageFreezes();

  /*!
   * Calls Drive::stageZeroJointTorques for all drives.
   */
  void stageZeroJointTorques();

  /*!
   * Calls Drive::setCommand for all drives.
   * @param commands Commands to stage for each Drive. Order and size must match the the list of Drives.
   */
  void setCommands(const std::vector<Command> & commands);

  //@}

  /*!
   * @name Debugging
   *
   * Methods for debugging.
   */
  //@{

  /*!
   * Print Drive infos for debugging.
   */
  void printInfo() const;

  //@}

};


} // rsl_drive_sdk
