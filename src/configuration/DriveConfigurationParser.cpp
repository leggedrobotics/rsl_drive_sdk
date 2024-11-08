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
#include "rsl_drive_sdk/configuration/DriveConfigurationParser.hpp"


namespace rsl_drive_sdk
{

configuration::Configuration DriveConfigurationParser::fromFile(const std::string & path)
{
  YAML::Node yamlNode = YAML::LoadFile(path);
  return fromYamlNode(yamlNode);
}

configuration::Configuration DriveConfigurationParser::fromYamlNode(const YAML::Node & yamlNode)
{
  configuration::Configuration config;


  if (yamlNode["max_command_age"].IsDefined()) {
    config.setMaxCommandAge(
                    yamlNode["max_command_age"].as<double>());
  }
  if (yamlNode["auto_stage_last_command"].IsDefined()) {
    config.setAutoStageLastCommand(
                    yamlNode["auto_stage_last_command"].as<bool>());
  }
  if (yamlNode["set_reading_to_nan_on_disconnect"].IsDefined()) {
    config.setSetReadingToNanOnDisconnect(
                    yamlNode["set_reading_to_nan_on_disconnect"].as<bool>());
  }
  if (yamlNode["error_state_behavior"].IsDefined()) {
    config.setErrorStateBehavior(
                    yamlNode["error_state_behavior"].as<uint16_t>());
  }
  if (yamlNode["max_current"].IsDefined()) {
    config.setMaxCurrent(
                    yamlNode["max_current"].as<double>());
  }
  if (yamlNode["max_freeze_current"].IsDefined()) {
    config.setMaxFreezeCurrent(
                    yamlNode["max_freeze_current"].as<double>());
  }
  if (yamlNode["max_motor_velocity"].IsDefined()) {
    config.setMaxMotorVelocity(
                    yamlNode["max_motor_velocity"].as<double>());
  }
  if (yamlNode["max_joint_torque"].IsDefined()) {
    config.setMaxJointTorque(
                    yamlNode["max_joint_torque"].as<double>());
  }
  if (yamlNode["current_integrator_saturation"].IsDefined()) {
    config.setCurrentIntegratorSaturation(
                    yamlNode["current_integrator_saturation"].as<double>());
  }
  if (yamlNode["joint_torque_integrator_saturation"].IsDefined()) {
    config.setJointTorqueIntegratorSaturation(
                    yamlNode["joint_torque_integrator_saturation"].as<double>());
  }
  if (yamlNode["direction"].IsDefined()) {
    config.setDirection(
                    yamlNode["direction"].as<int16_t>());
  }
  if (yamlNode["joint_position_limits"].IsDefined()) {
    if (yamlNode["joint_position_limits"]["sdk"].IsDefined()) {
      config.setJointPositionLimitsSdk(common::Limits(
                                          yamlNode["joint_position_limits"]["sdk"]["min"].as<double>(),
                    yamlNode["joint_position_limits"]["sdk"]["max"].as<double>()));
    }
    if (yamlNode["joint_position_limits"]["soft"].IsDefined()) {
      config.setJointPositionLimitsSoft(common::Limits(
                                           yamlNode["joint_position_limits"]["soft"]["min"].as<
            double>(),
                    yamlNode["joint_position_limits"]["soft"]["max"].as<double>()));
    }
    if (yamlNode["joint_position_limits"]["hard"].IsDefined()) {
      config.setJointPositionLimitsHard(common::Limits(
                                           yamlNode["joint_position_limits"]["hard"]["min"].as<
            double>(),
                    yamlNode["joint_position_limits"]["hard"]["max"].as<double>()));
    }
  }
  if (yamlNode["joint_position_configurations"].IsDefined()) {
    for (unsigned int i = 0; i < yamlNode["joint_position_configurations"].size(); i++) {
      config.addJointPositionConfiguration(
                        yamlNode["joint_position_configurations"][i]["name"].as<std::string>(),
                    yamlNode["joint_position_configurations"][i]["value"].as<double>());
    }
  }
  if (yamlNode["imu"].IsDefined()) {
    if (yamlNode["imu"]["enable"].IsDefined()) {
      config.setImuEnable(
                        yamlNode["imu"]["enable"].as<bool>());
    }
    if (yamlNode["imu"]["accelerometer_range"].IsDefined()) {
      config.setImuAccelerometerRange(
                        yamlNode["imu"]["accelerometer_range"].as<uint32_t>());
    }
    if (yamlNode["imu"]["gyroscope_range"].IsDefined()) {
      config.setImuGyroscopeRange(
                        yamlNode["imu"]["gyroscope_range"].as<uint32_t>());
    }
  }
  if (yamlNode["fan"].IsDefined()) {
    if (yamlNode["fan"]["mode"].IsDefined()) {
      config.setFanMode(
                        yamlNode["fan"]["mode"].as<uint32_t>());
    }
    if (yamlNode["fan"]["intensity"].IsDefined()) {
      config.setFanIntensity(
                        yamlNode["fan"]["intensity"].as<uint32_t>());
    }
    if (yamlNode["fan"]["lower_temperature"].IsDefined()) {
      config.setFanLowerTemperature(
                        yamlNode["fan"]["lower_temperature"].as<float>());
    }
    if (yamlNode["fan"]["upper_temperature"].IsDefined()) {
      config.setFanUpperTemperature(
                        yamlNode["fan"]["upper_temperature"].as<float>());
    }
  }
  if (yamlNode["modes"].IsDefined()) {
    for (unsigned int i = 0; i < yamlNode["modes"].size(); i++) {
      const std::string modeName = yamlNode["modes"][i]["name"].as<std::string>();
      mode::ModeBasePtr mode = config.getMode(mode::modeNameToEnum(modeName));
      if (!mode) {
        MELO_ERROR_STREAM("Mode name '" << modeName << "' does not exist, cannot set gains.");
        continue;
      }
      mode->setPidGains(mode::PidGainsF(
                                  yamlNode["modes"][i]["gains"]["p"].as<double>(),
                    yamlNode["modes"][i]["gains"]["i"].as<double>(),
                    yamlNode["modes"][i]["gains"]["d"].as<double>()));
    }
  }
  if (yamlNode["goal_states"].IsDefined()) {
    if (yamlNode["goal_states"]["startup"].IsDefined()) {
      config.setGoalStateEnumStartup(fsm::stateNameToEnum(
                                        yamlNode["goal_states"]["startup"].as<std::string>()));
    }
    if (yamlNode["goal_states"]["shutdown"].IsDefined()) {
      config.setGoalStateEnumShutdown(fsm::stateNameToEnum(
                                         yamlNode["goal_states"]["shutdown"].as<std::string>()));
    }
  }
  if (yamlNode["gear_joint_velocity_filter"].IsDefined()) {
    if(yamlNode["gear_joint_velocity_filter"]["type"].IsDefined()) {
      config.setGearJointVelocityFilterType(
          yamlNode["gear_joint_velocity_filter"]["type"].as<uint32_t>());
    }
    if(yamlNode["gear_joint_velocity_filter"]["kf_noise_variance"].IsDefined()) {
      config.setGearJointVelocityKfNoiseVariance(
          yamlNode["gear_joint_velocity_filter"]["kf_noise_variance"].as<double>());
    }
    if(yamlNode["gear_joint_velocity_filter"]["kf_lambda_2"].IsDefined()) {
      config.setGearJointVelocityKfLambda2(
          yamlNode["gear_joint_velocity_filter"]["kf_lambda_2"].as<double>());
    }
    if(yamlNode["gear_joint_velocity_filter"]["kf_gamma"].IsDefined()) {
      config.setGearJointVelocityKfGamma(
          yamlNode["gear_joint_velocity_filter"]["kf_gamma"].as<double>());
    }
    if(yamlNode["gear_joint_velocity_filter"]["ema_alpha"].IsDefined()) {
      config.setGearJointVelocityEmaAlpha(
          yamlNode["gear_joint_velocity_filter"]["ema_alpha"].as<double>());
    }
  }
  if (yamlNode["joint_velocity_filter_for_acceleration"].IsDefined()) {
    if(yamlNode["joint_velocity_filter_for_acceleration"]["type"].IsDefined()) {
      config.setJointVelocityForAccelerationFilterType(
          yamlNode["joint_velocity_filter_for_acceleration"]["type"].as<uint32_t>());
    }
    if(yamlNode["joint_velocity_filter_for_acceleration"]["kf_noise_variance"].IsDefined()) {
      config.setJointVelocityForAccelerationKfNoiseVariance(
          yamlNode["joint_velocity_filter_for_acceleration"]["kf_noise_variance"].as<double>());
    }
    if(yamlNode["joint_velocity_filter_for_acceleration"]["kf_lambda_2"].IsDefined()) {
      config.setJointVelocityForAccelerationKfLambda2(
          yamlNode["joint_velocity_filter_for_acceleration"]["kf_lambda_2"].as<double>());
    }
    if(yamlNode["joint_velocity_filter_for_acceleration"]["kf_gamma"].IsDefined()) {
      config.setJointVelocityForAccelerationKfGamma(
          yamlNode["joint_velocity_filter_for_acceleration"]["kf_gamma"].as<double>());
    }
    if(yamlNode["joint_velocity_filter_for_acceleration"]["ema_alpha"].IsDefined()) {
      config.setJointVelocityForAccelerationEmaAlpha(
          yamlNode["joint_velocity_filter_for_acceleration"]["ema_alpha"].as<double>());
    }
  }
  if (yamlNode["joint_acceleration_filter"].IsDefined()) {
    if(yamlNode["joint_acceleration_filter"]["type"].IsDefined()) {
      config.setJointAccelerationFilterType(
          yamlNode["joint_acceleration_filter"]["type"].as<uint32_t>());
    }
    if(yamlNode["joint_acceleration_filter"]["kf_noise_variance"].IsDefined()) {
      config.setJointAccelerationKfNoiseVariance(
          yamlNode["joint_acceleration_filter"]["kf_noise_variance"].as<double>());
    }
    if(yamlNode["joint_acceleration_filter"]["kf_lambda_2"].IsDefined()) {
      config.setJointAccelerationKfLambda2(
          yamlNode["joint_acceleration_filter"]["kf_lambda_2"].as<double>());
    }
    if(yamlNode["joint_acceleration_filter"]["kf_gamma"].IsDefined()) {
      config.setJointAccelerationKfGamma(
          yamlNode["joint_acceleration_filter"]["kf_gamma"].as<double>());
    }
    if(yamlNode["joint_acceleration_filter"]["ema_alpha"].IsDefined()) {
      config.setJointAccelerationEmaAlpha(
          yamlNode["joint_acceleration_filter"]["ema_alpha"].as<double>());
    }
  }
  if (yamlNode["fake"].IsDefined()) {
    config.setFake(yamlNode["fake"].as<bool>());
  }
  if(yamlNode["dgainFilterCutoffFrequency"].IsDefined()) {
    config.setDGainFilterCutoffFrequency(yamlNode["dgainFilterCutoffFrequency"].as<double>());
  }
  return config;
}

}
