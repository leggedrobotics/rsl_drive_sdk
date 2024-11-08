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
#include <string>


namespace rsl_drive_sdk
{
namespace fsm
{


// Controlword IDs.
// Note: This mapping must be equal to the mapping in the firmware.
#define RSL_DRIVE_CW_ID_CALIBRATE_TO_CONFIGURE              (0x05)
#define RSL_DRIVE_CW_ID_CLEAR_ERRORS_TO_MOTOR_OP            (0x02)
#define RSL_DRIVE_CW_ID_CLEAR_ERRORS_TO_STANDBY             (0x0C)
#define RSL_DRIVE_CW_ID_CONFIGURE_TO_CALIBRATE              (0x06)
#define RSL_DRIVE_CW_ID_CONFIGURE_TO_STANDBY                (0x04)
#define RSL_DRIVE_CW_ID_CONTROL_OP_TO_MOTOR_OP              (0x09)
#define RSL_DRIVE_CW_ID_CONTROL_OP_TO_STANDBY               (0x0B)
#define RSL_DRIVE_CW_ID_MOTOR_OP_TO_CONTROL_OP              (0x0A)
#define RSL_DRIVE_CW_ID_MOTOR_OP_TO_STANDBY                 (0x07)
#define RSL_DRIVE_CW_ID_NA                                  (0x00)
#define RSL_DRIVE_CW_ID_STANDBY_TO_CONFIGURE                (0x03)
#define RSL_DRIVE_CW_ID_STANDBY_TO_MOTOR_PREOP              (0x08)
#define RSL_DRIVE_CW_ID_WARM_RESET                          (0x01)

// Controlword names.
#define RSL_DRIVE_CW_NAME_CALIBRATE_TO_CONFIGURE            ("Calibrate to Configure")
#define RSL_DRIVE_CW_NAME_CLEAR_ERRORS_TO_MOTOR_OP          ("Clear Errors to MotorOp")
#define RSL_DRIVE_CW_NAME_CLEAR_ERRORS_TO_STANDBY           ("Clear Errors to Standby")
#define RSL_DRIVE_CW_NAME_CONFIGURE_TO_CALIBRATE            ("Configure to Calibrate")
#define RSL_DRIVE_CW_NAME_CONFIGURE_TO_STANDBY              ("Configure to Standby")
#define RSL_DRIVE_CW_NAME_CONTROL_OP_TO_MOTOR_OP            ("ControlOp to MotorOp")
#define RSL_DRIVE_CW_NAME_CONTROL_OP_TO_STANDBY             ("ControlOp to Standby")
#define RSL_DRIVE_CW_NAME_MOTOR_OP_TO_CONTROL_OP            ("MotorOp to ControlOp")
#define RSL_DRIVE_CW_NAME_MOTOR_OP_TO_STANDBY               ("MotorOp to Standby")
#define RSL_DRIVE_CW_NAME_NA                                ("N/A")
#define RSL_DRIVE_CW_NAME_STANDBY_TO_CONFIGURE              ("Standby to Configure")
#define RSL_DRIVE_CW_NAME_STANDBY_TO_MOTOR_PREOP            ("Standby to MotorPreOp")
#define RSL_DRIVE_CW_NAME_WARM_RESET                        ("Warm Reset")

std::string controlwordIdToString(const uint16_t controlwordId);
uint16_t controlwordStringToId(const std::string & controlwordString);


} // fsm
} // rsl_drive_sdk
