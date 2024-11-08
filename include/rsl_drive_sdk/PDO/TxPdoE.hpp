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


namespace rsl_drive_sdk
{


struct TxPdoE
{
  uint32_t statusword_ = 0;
  uint16_t measuredTemperature_ = 0;
  uint16_t measuredMotorVoltage_ = 0;
  double measuredMotorPosition_ = 0.0;
  double measuredGearPosition_ = 0.0;
  double measuredJointPosition_ = 0.0;
  float measuredMotorCurrent_ = 0.0;
  float measuredMotorVelocity_ = 0.0;
  float measuredGearVelocity_ = 0.0;
  float measuredJointVelocity_ = 0.0;
  float measuredJointAcceleration_ = 0.0;
  float measuredJointTorque_ = 0.0;
  int32_t measuredGearPositionTicks_ = 0;
  int32_t measuredJointPositionTicks_ = 0;
  uint64_t timestamp_ = 0;
  float desiredCurrentD_ = 0.0;
  float measuredCurrentD_ = 0.0;
  float desiredCurrentQ_ = 0.0;
  float measuredCurrentQ_ = 0.0;
  float alpha_ = 0.0;
  float beta_ = 0.0;
  float dutyCycleU_ = 0.0;
  float dutyCycleV_ = 0.0;
  float dutyCycleW_ = 0.0;
  float measuredCurrentPhaseU_ = 0.0;
  float measuredCurrentPhaseV_ = 0.0;
  float measuredCurrentPhaseW_ = 0.0;
  float measuredVoltagePhaseU_ = 0.0;
  float measuredVoltagePhaseV_ = 0.0;
  float measuredVoltagePhaseW_ = 0.0;
  float desiredMotorVelocity_ = 0.0;
  double desiredGearPosition_ = 0.0;
  float desiredGearVelocity_ = 0.0;
  double desiredJointPosition_ = 0.0;
  float desiredJointVelocity_ = 0.0;
  float desiredJointTorque_ = 0.0;
  float measuredImuLinearAccelerationX_ = 0.0;
  float measuredImuLinearAccelerationY_ = 0.0;
  float measuredImuLinearAccelerationZ_ = 0.0;
  float measuredImuAngularVelocityX_ = 0.0;
  float measuredImuAngularVelocityY_ = 0.0;
  float measuredImuAngularVelocityZ_ = 0.0;
  float measuredImuAngularVelocityW_ = 0.0;
  int32_t measuredCoilTemp1_ = 0.0;
  int32_t measuredCoilTemp2_ = 0.0;
  int32_t measuredCoilTemp3_ = 0.0;
} __attribute__((packed));


} // rsl_drive_sdk_ethercat
