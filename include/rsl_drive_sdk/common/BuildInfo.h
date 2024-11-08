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


#include <iostream>
#include <string>
#include <cstdint>


namespace  rsl_drive_sdk {
  namespace  common {


    struct capabilities_t
    {
      uint32_t experimental:1;
      uint32_t async_spi:1;
      uint32_t async_i2c:1;
      uint32_t watchdog:1;
      uint32_t shunt_05:1;
      uint32_t timing_measurements:1;
      uint32_t fieldweakening:1;
    };

    union capabilities_u{
      capabilities_t cap;
      uint32_t raw;
    };


    struct BuildInfo
    {
      std::string buildDate = "";
      std::string gitTag = "";
      std::string gitHash = "";
      capabilities_t caps;
    };

  }

}
