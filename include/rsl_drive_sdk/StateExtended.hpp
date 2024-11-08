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


// rsl_drive_sdk
#include "rsl_drive_sdk/State.hpp"


namespace rsl_drive_sdk
{


//! Extended state of the Drive, containing additional information to the state.
class StateExtended : public State
{
protected:
  //! Motor position [rad].
  double motorPosition_ = 0.0;
  //! Motor velocity [rad/s].
  double motorVelocity_ = 0.0;
  //! Position of the spring on the gear side [ticks].
  int32_t gearPositionTicks_ = 0;
  //! Position of the spring on the joint side [ticks].
  int32_t jointPositionTicks_ = 0;
  //! Temperature of the actuator [°C].
  double temperature_ = 0.0;
  //! Measured input voltage [V].
  double voltage_ = 0.0;
  //! Time on the Drive [1/(25e6s)].
  uint64_t timestamp_ = 0;
  //! D part of the desired current [A].
  double desiredCurrentD_ = 0.0;
  //! D part of the measured current [A].
  double measuredCurrentD_ = 0.0;
  //! Q part of the desired current [A].
  double desiredCurrentQ_ = 0.0;
  //! Q part of the measured current [A].
  double measuredCurrentQ_ = 0.0;
  //! Alpha from Inverse Park Transform.
  double alpha_ = 0.0;
  //! Beta from Inverse Park Transform.
  double beta_ = 0.0;
  //! U phase of the measured current [A].
  double measuredCurrentPhaseU_ = 0.0;
  //! U phase of the measured voltage [V].
  double measuredVoltagePhaseU_ = 0.0;
  //! V phase of the measured current [A].
  double measuredCurrentPhaseV_ = 0.0;
  //! V phase of the measured voltage [V].
  double measuredVoltagePhaseV_ = 0.0;
  //! W phase of the measured current [A].
  double measuredCurrentPhaseW_ = 0.0;
  //! W phase of the measured voltage [V].
  double measuredVoltagePhaseW_ = 0.0;
  //! U duty cycle.
  double dutyCycleU_ = 0.0;
  //! V duty cycle.
  double dutyCycleV_ = 0.0;
  //! W duty cycle.
  double dutyCycleW_ = 0.0;

  float coilTemp1_ = 0.0;
  float coilTemp2_ = 0.0;
  float coilTemp3_ = 0.0;

  //Apparent power in [W]
  double measuredApparentPower = 0.0;
  //Apparent input current [A]
  double measuredInputCurrent = 0.0;

public:
  StateExtended();
  virtual ~StateExtended();

  double getMotorPosition() const;
  void setMotorPosition(const double motorPosition);

  double getMotorVelocity() const;
  void setMotorVelocity(const double motorVelocity);

  int32_t getGearPositionTicks() const;
  void setGearPositionTicks(const int32_t gearPositionTicks);

  int32_t getJointPositionTicks() const;
  void setJointPositionTicks(const int32_t jointPositionTicks);

  double getTemperature() const;
  void setTemperature(const double temperature);

  double getVoltage() const;
  void setVoltage(const double voltage);

  uint64_t getTimestamp() const;
  void setTimestamp(const uint64_t timestamp);

  double getDesiredCurrentD() const;
  void setDesiredCurrentD(const double current);

  double getMeasuredCurrentD() const;
  void setMeasuredCurrentD(const double current);

  double getDesiredCurrentQ() const;
  void setDesiredCurrentQ(const double current);

  double getMeasuredCurrentQ() const;
  void setMeasuredCurrentQ(const double current);

  double getAlpha() const;
  void setAlpha(const double alpha);

  double getBeta() const;
  void setBeta(const double beta);

  double getMeasuredCurrentPhaseU() const;
  void setMeasuredCurrentPhaseU(const double current);

  double getMeasuredCurrentPhaseV() const;
  void setMeasuredCurrentPhaseV(const double current);

  double getMeasuredCurrentPhaseW() const;
  void setMeasuredCurrentPhaseW(const double current);

  double getMeasuredVoltagePhaseU() const;
  void setMeasuredVoltagePhaseU(const double voltage);

  double getMeasuredVoltagePhaseV() const;
  void setMeasuredVoltagePhaseV(const double voltage);

  double getMeasuredVoltagePhaseW() const;
  void setMeasuredVoltagePhaseW(const double voltage);

  double getDutyCycleU() const;
  void setDutyCycleU(const double dutyCycle);

  double getDutyCycleV() const;
  void setDutyCycleV(const double dutyCycle);

  double getDutyCycleW() const;
  void setDutyCycleW(const double dutyCycle);

  double getCoilTemp1() const;
  void setCoilTemp1(const double coilTemp);

  double getCoilTemp2() const;
  void setCoilTemp2(const double coilTemp);

  double getCoilTemp3() const;
  void setCoilTemp3(const double coilTemp);


  virtual std::string asString(const std::string & prefix = "") const;
  double getMeasuredApparentPower() const;
  void setMeasuredApparentPower(double value);
  double getMeasuredInputCurrent() const;
  void setMeasuredInputCurrent(double value);
};

std::ostream & operator<<(std::ostream & out, const StateExtended & stateExtended);


} // rsl_drive_sdk
