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
#include <iostream>
#include <string>


namespace rsl_drive_sdk
{
namespace mode
{


// Mode IDs.
// This mapping must be equal to the firmware mapping.
#define RSL_DRIVE_MODE_ID_CURRENT                              (3)  // Track current
#define RSL_DRIVE_MODE_ID_JOINT_POS_TOR                        (16) // Track position + feedforward torque
#define RSL_DRIVE_MODE_ID_DISABLE                              (2)  // Disable motor
#define RSL_DRIVE_MODE_ID_FREEZE                               (1)  // Freeze motor
#define RSL_DRIVE_MODE_ID_GEAR_POS                             (6)  // Track gear position
#define RSL_DRIVE_MODE_ID_GEAR_VEL                             (7)  // Track gear velocity
#define RSL_DRIVE_MODE_ID_JOINT_POS                            (8)  // Track joint position
#define RSL_DRIVE_MODE_ID_JOINT_POS_VEL                        (11) // Track joint position with feedforward velocity
#define RSL_DRIVE_MODE_ID_JOINT_POS_VEL_TOR                    (12) // Track joint position with feedforward velocity and torque
#define RSL_DRIVE_MODE_ID_JOINT_POS_VEL_TOR_PID                (13) // Track joint position with feedforward velocity and torque using custom joint position gains
#define RSL_DRIVE_MODE_ID_JOINT_TOR                            (10) // Track joint torque
#define RSL_DRIVE_MODE_ID_JOINT_VEL                            (9)  // Track joint velocity
#define RSL_DRIVE_MODE_ID_MOTOR_POS                            (4)  // Track motor position
#define RSL_DRIVE_MODE_ID_MOTOR_VEL                            (5)  // Track motor velocity
#define RSL_DRIVE_MODE_ID_NA                                   (0)  // Not available

// Mode names.
#define RSL_DRIVE_MODE_NAME_CURRENT                            ("Current")
#define RSL_DRIVE_MODE_NAME_JOINT_POS_TOR                      ("JointPositionTorque")
#define RSL_DRIVE_MODE_NAME_DISABLE                            ("Disable")
#define RSL_DRIVE_MODE_NAME_FREEZE                             ("Freeze")
#define RSL_DRIVE_MODE_NAME_GEAR_POS                           ("GearPosition")
#define RSL_DRIVE_MODE_NAME_GEAR_VEL                           ("GearVelocity")
#define RSL_DRIVE_MODE_NAME_JOINT_POS                          ("JointPosition")
#define RSL_DRIVE_MODE_NAME_JOINT_POS_VEL                      ("JointPositionVelocity")
#define RSL_DRIVE_MODE_NAME_JOINT_POS_VEL_TOR                  ("JointPositionVelocityTorque")
#define RSL_DRIVE_MODE_NAME_JOINT_POS_VEL_TOR_PID              ( \
    "JointPositionVelocityTorquePidGains")
#define RSL_DRIVE_MODE_NAME_JOINT_TOR                          ("JointTorque")
#define RSL_DRIVE_MODE_NAME_JOINT_VEL                          ("JointVelocity")
#define RSL_DRIVE_MODE_NAME_MOTOR_POS                          ("MotorPosition")
#define RSL_DRIVE_MODE_NAME_MOTOR_VEL                          ("MotorVelocity")
#define RSL_DRIVE_MODE_NAME_NA                                 ("N/A")

// Mode short names.
#define RSL_DRIVE_MODE_NAME_SHORT_CURRENT                      ("C")
#define RSL_DRIVE_MODE_NAME_SHORT_JOINT_POS_TOR                ("JPT")
#define RSL_DRIVE_MODE_NAME_SHORT_DISABLE                      ("D")
#define RSL_DRIVE_MODE_NAME_SHORT_FREEZE                       ("F")
#define RSL_DRIVE_MODE_NAME_SHORT_GEAR_POS                     ("GP")
#define RSL_DRIVE_MODE_NAME_SHORT_GEAR_VEL                     ("GV")
#define RSL_DRIVE_MODE_NAME_SHORT_JOINT_POS                    ("JP")
#define RSL_DRIVE_MODE_NAME_SHORT_JOINT_POS_VEL                ("JPV")
#define RSL_DRIVE_MODE_NAME_SHORT_JOINT_POS_VEL_TOR            ("JPVT")
#define RSL_DRIVE_MODE_NAME_SHORT_JOINT_POS_VEL_TOR_PID        ("JPVTPID")
#define RSL_DRIVE_MODE_NAME_SHORT_JOINT_TOR                    ("JT")
#define RSL_DRIVE_MODE_NAME_SHORT_JOINT_VEL                    ("JV")
#define RSL_DRIVE_MODE_NAME_SHORT_MOTOR_POS                    ("MP")
#define RSL_DRIVE_MODE_NAME_SHORT_MOTOR_VEL                    ("MV")
#define RSL_DRIVE_MODE_NAME_SHORT_NA                           ("N/A")

// Mode enumerators.
enum class ModeEnum
{
  Current,
  JointPositionTorque,
  Disable,
  Freeze,
  GearPosition,
  GearVelocity,
  JointPosition,
  JointPositionVelocity,
  JointPositionVelocityTorque,
  JointPositionVelocityTorquePidGains,
  JointTorque,
  JointVelocity,
  MotorPosition,
  MotorVelocity,
  NA
};


uint16_t modeEnumToId(const ModeEnum modeEnum);
ModeEnum modeIdToEnum(const uint16_t modeId);

std::string modeEnumToName(const ModeEnum modeEnum);
std::string modeEnumToShortName(const ModeEnum modeEnum);
ModeEnum modeNameToEnum(const std::string & string);

std::ostream & operator<<(std::ostream & out, const ModeEnum modeEnum);


} // mode
} // rsl_drive_sdk
