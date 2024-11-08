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

// rsl_drive_sdk
#include "rsl_drive_sdk/common/ObjectDictionary.hpp"


namespace rsl_drive_sdk
{
namespace calibration
{


//! Calibration type enumerators for type safe usage.
enum class CalibrationTypeEnum
{
  Custom,
  Factory,
  NA
};

/*!
 * Convert a calibration type enumerator to an ID.
 * @param calibrationTypeEnum Calibration type enumerator.
 * @return Calibration type ID.
 */
uint16_t calibrationTypeEnumToId(const CalibrationTypeEnum calibrationTypeEnum);

/*!
 * Convert a calibration type ID to an enumerator.
 * @param calibrationTypeId Calibration type ID.
 * @return Calibration type enumerator.
 */
CalibrationTypeEnum calibrationTypeIdToEnum(const uint16_t calibrationTypeId);

/*!
 * Convert a calibration type enumerator to a human readable string (GUI, etc.).
 * @param calibrationTypeEnum Calibration type enumerator.
 * @return Human readable string.
 */
std::string calibrationTypeEnumToName(const CalibrationTypeEnum calibrationTypeEnum);

/*!
 * Convert a human readable string (GUI, etc.) to a calibration type enumerator.
 * @param calibrationTypeName Human readable string.
 * @return Calibration type enumerator.
 */
CalibrationTypeEnum calibrationTypeNameToEnum(const std::string & calibrationTypeName);


} // calibration
} // rsl_drive_sdk
