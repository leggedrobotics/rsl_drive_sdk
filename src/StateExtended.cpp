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
// std
#include <cmath>
#include <sstream>

// rsl_drive_sdk
#include "rsl_drive_sdk/StateExtended.hpp"


namespace rsl_drive_sdk
{


double StateExtended::getMeasuredApparentPower() const
{
  return measuredApparentPower;
}

void StateExtended::setMeasuredApparentPower(double value)
{
  measuredApparentPower = value;
}

double StateExtended::getMeasuredInputCurrent() const
{
  return measuredInputCurrent;
}

void StateExtended::setMeasuredInputCurrent(double value)
{
  measuredInputCurrent = value;
}

StateExtended::StateExtended()
: State() {}

StateExtended::~StateExtended() {}

double StateExtended::getMotorPosition() const
{
  return motorPosition_;
}

void StateExtended::setMotorPosition(const double motorPosition)
{
  motorPosition_ = motorPosition;
}

double StateExtended::getMotorVelocity() const
{
  return motorVelocity_;
}

void StateExtended::setMotorVelocity(const double motorVelocity)
{
  motorVelocity_ = motorVelocity;
}

int32_t StateExtended::getGearPositionTicks() const
{
  return gearPositionTicks_;
}

void StateExtended::setGearPositionTicks(const int32_t gearPositionTicks)
{
  gearPositionTicks_ = gearPositionTicks;
}

int32_t StateExtended::getJointPositionTicks() const
{
  return jointPositionTicks_;
}

void StateExtended::setJointPositionTicks(const int32_t jointPositionTicks)
{
  jointPositionTicks_ = jointPositionTicks;
}

double StateExtended::getTemperature() const
{
  return temperature_;
}

void StateExtended::setTemperature(const double temperature)
{
  temperature_ = temperature;
}

double StateExtended::getVoltage() const
{
  return voltage_;
}

void StateExtended::setVoltage(const double voltage)
{
  voltage_ = voltage;
}

uint64_t StateExtended::getTimestamp() const
{
  return timestamp_;
}

void StateExtended::setTimestamp(const uint64_t timestamp)
{
  timestamp_ = timestamp;
}

double StateExtended::getDesiredCurrentD() const
{
  return desiredCurrentD_;
}

void StateExtended::setDesiredCurrentD(const double current)
{
  desiredCurrentD_ = current;
}

double StateExtended::getMeasuredCurrentD() const
{
  return measuredCurrentD_;
}

void StateExtended::setMeasuredCurrentD(const double current)
{
  measuredCurrentD_ = current;
}

double StateExtended::getDesiredCurrentQ() const
{
  return desiredCurrentQ_;
}

void StateExtended::setDesiredCurrentQ(const double current)
{
  desiredCurrentQ_ = current;
}

double StateExtended::getMeasuredCurrentQ() const
{
  return measuredCurrentQ_;
}

void StateExtended::setMeasuredCurrentQ(const double current)
{
  measuredCurrentQ_ = current;
}

double StateExtended::getAlpha() const
{
  return alpha_;
}

void StateExtended::setAlpha(const double alpha)
{
  alpha_ = alpha;
}

double StateExtended::getBeta() const
{
  return beta_;
}

void StateExtended::setBeta(const double beta)
{
  beta_ = beta;
}

double StateExtended::getMeasuredCurrentPhaseU() const
{
  return measuredCurrentPhaseU_;
}

void StateExtended::setMeasuredCurrentPhaseU(const double current)
{
  measuredCurrentPhaseU_ = current;
}

double StateExtended::getMeasuredCurrentPhaseV() const
{
  return measuredCurrentPhaseV_;
}

void StateExtended::setMeasuredCurrentPhaseV(const double current)
{
  measuredCurrentPhaseV_ = current;
}

double StateExtended::getMeasuredCurrentPhaseW() const
{
  return measuredCurrentPhaseW_;
}

void StateExtended::setMeasuredCurrentPhaseW(const double current)
{
  measuredCurrentPhaseW_ = current;
}

double StateExtended::getMeasuredVoltagePhaseU() const
{
  return measuredVoltagePhaseU_;
}

void StateExtended::setMeasuredVoltagePhaseU(const double voltage)
{
  measuredVoltagePhaseU_ = voltage;
}

double StateExtended::getMeasuredVoltagePhaseV() const
{
  return measuredVoltagePhaseV_;
}

void StateExtended::setMeasuredVoltagePhaseV(const double voltage)
{
  measuredVoltagePhaseV_ = voltage;
}

double StateExtended::getMeasuredVoltagePhaseW() const
{
  return measuredVoltagePhaseW_;
}

void StateExtended::setMeasuredVoltagePhaseW(const double voltage)
{
  measuredVoltagePhaseW_ = voltage;
}

double StateExtended::getDutyCycleU() const
{
  return dutyCycleU_;
}

void StateExtended::setDutyCycleU(const double dutyCycle)
{
  dutyCycleU_ = dutyCycle;
}

double StateExtended::getDutyCycleV() const
{
  return dutyCycleV_;
}

void StateExtended::setDutyCycleV(const double dutyCycle)
{
  dutyCycleV_ = dutyCycle;
}

double StateExtended::getDutyCycleW() const
{
  return dutyCycleW_;
}

void StateExtended::setDutyCycleW(const double dutyCycle)
{
  dutyCycleW_ = dutyCycle;
}

double StateExtended::getCoilTemp1() const
{
  return coilTemp1_;
}

void StateExtended::setCoilTemp1(const double coilTemp)
{
  coilTemp1_ = coilTemp;
}

double StateExtended::getCoilTemp2() const
{
  return coilTemp2_;
}

void StateExtended::setCoilTemp2(const double coilTemp)
{
  coilTemp2_ = coilTemp;
}

double StateExtended::getCoilTemp3() const
{
  return coilTemp3_;
}

void StateExtended::setCoilTemp3(const double coilTemp)
{
  coilTemp3_ = coilTemp;
}


std::string StateExtended::asString(const std::string & prefix) const
{
  std::stringstream ss;
  ss << prefix << static_cast<const State &>(*this);
  ss << prefix << "Motor position: " << motorPosition_ << std::endl;
  ss << prefix << "Motor velocity: " << motorVelocity_ << std::endl;
  ss << prefix << "Gear position ticks: " << gearPositionTicks_ << std::endl;
  ss << prefix << "Joint position ticks: " << jointPositionTicks_ << std::endl;
  ss << prefix << "Temperature: " << temperature_ << std::endl;
  ss << prefix << "Voltage: " << voltage_ << std::endl;
  ss << prefix << "Timestamp: " << timestamp_ << std::endl;
  ss << prefix << "Desired current D: " << desiredCurrentD_ << std::endl;
  ss << prefix << "Measured current D: " << measuredCurrentD_ << std::endl;
  ss << prefix << "Desired current Q: " << desiredCurrentQ_ << std::endl;
  ss << prefix << "Measured current Q: " << measuredCurrentQ_ << std::endl;
  ss << prefix << "Alpha: " << alpha_ << std::endl;
  ss << prefix << "Beta: " << beta_ << std::endl;
  ss << prefix << "Measured current phase U: " << measuredCurrentPhaseU_ << std::endl;
  ss << prefix << "Measured current phase V: " << measuredCurrentPhaseV_ << std::endl;
  ss << prefix << "Measured current phase W: " << measuredCurrentPhaseW_ << std::endl;
  ss << prefix << "Measured voltage phase U: " << measuredVoltagePhaseU_ << std::endl;
  ss << prefix << "Measured voltage phase V: " << measuredVoltagePhaseV_ << std::endl;
  ss << prefix << "Measured voltage phase W: " << measuredVoltagePhaseW_ << std::endl;
  ss << prefix << "Duty cycle U: " << dutyCycleU_ << std::endl;
  ss << prefix << "Duty cycle V: " << dutyCycleV_ << std::endl;
  ss << prefix << "Duty cycle W: " << dutyCycleW_ << std::endl;
  ss << prefix << "Temperature coil 1:" << coilTemp1_ << std::endl;
  ss << prefix << "Temperature coil 2:" << coilTemp2_ << std::endl;
  ss << prefix << "Temperature coil 3:" << coilTemp3_ << std::endl;
  ss << prefix << "Apparent power: " << measuredApparentPower << std::endl;
  ss << prefix << "Input current: " << measuredInputCurrent << std::endl;
  return ss.str();
}

std::ostream & operator<<(std::ostream & out, const StateExtended & stateExtended)
{
  out << "State extended:" << std::endl;
  out << stateExtended.asString("  ");
  return out;
}


} // rsl_drive_sdk
