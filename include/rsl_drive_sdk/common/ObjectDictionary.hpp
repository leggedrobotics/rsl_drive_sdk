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


/*!
 * Object dictionary:
 *   ID = index (16 bit)
 *   SID = sub-index (8 bit)
 *   BIT = bit index
 *   VAL = pre-defined values
 */


// DSP402
#define OD_DSP402_MODEL_ID                                                     (uint16_t(0x1008))
#define OD_DSP402_RX_PDO_ID                                                    (uint16_t(0x1C12))
#define OD_DSP402_RX_PDO_SID                                                   (uint8_t(0x01))
#define OD_DSP402_RX_PDO_SID_VAL_A                                             (uint16_t(0x1600))
#define OD_DSP402_RX_PDO_SID_VAL_B                                             (uint16_t(0x1601))
#define OD_DSP402_RX_PDO_SID_VAL_C                                             (uint16_t(0x1602))
#define OD_DSP402_RX_PDO_SID_VAL_D                                             (uint16_t(0x1603))
#define OD_DSP402_TX_PDO_ID                                                    (uint16_t(0x1C13))
#define OD_DSP402_TX_PDO_SID                                                   (uint8_t(0x01))
#define OD_DSP402_TX_PDO_SID_VAL_A                                             (uint16_t(0x1A00))
#define OD_DSP402_TX_PDO_SID_VAL_B                                             (uint16_t(0x1A01))
#define OD_DSP402_TX_PDO_SID_VAL_C                                             (uint16_t(0x1A02))
#define OD_DSP402_TX_PDO_SID_VAL_D                                             (uint16_t(0x1A03))
#define OD_DSP402_TX_PDO_SID_VAL_E                                            (uint16_t(0x1A04))
#define OD_DSP402_MEASURED_TEMPERATURE_ID                                      (uint16_t(0x2002))
#define OD_DSP402_MEASURED_MOTOR_VOLTAGE_ID                                    (uint16_t(0x2003))
#define OD_DSP402_MEASURED_GEAR_POSITION_ID                                    (uint16_t(0x2005))
#define OD_DSP402_MEASURED_JOINT_POSITION_ID                                   (uint16_t(0x2006))
#define OD_DSP402_MEASURED_MOTOR_CURRENT_ID                                    (uint16_t(0x2007))
#define OD_DSP402_MEASURED_GEAR_VELOCITY_ID                                    (uint16_t(0x2009))
#define OD_DSP402_MEASURED_JOINT_VELOCITY_ID                                   (uint16_t(0x200A))
#define OD_DSP402_MEASURED_JOINT_ACCELERATION_ID                               (uint16_t(0x200B))
#define OD_DSP402_DESIRED_MOTOR_CURRENT_ID                                     (uint16_t(0x2020))
#define OD_DSP402_CONTROL_PARAMETER_A_ID                                       (uint16_t(0x2024))
#define OD_DSP402_CONTROL_PARAMETER_B_ID                                       (uint16_t(0x2025))
#define OD_DSP402_CONTROL_PARAMETER_C_ID                                       (uint16_t(0x2026))
#define OD_DSP402_CONTROL_PARAMETER_D_ID                                       (uint16_t(0x2027))
#define OD_DSP402_ERROR_CODE_ID                                                (uint16_t(0x603F))
#define OD_DSP402_CONTROL_WORD_ID                                              (uint16_t(0x6040))
#define OD_DSP402_STATUS_WORD_ID                                               (uint16_t(0x6041))
#define OD_DSP402_MAX_COMMUNICATION_TIMEOUT_ID                                 (uint16_t(0x6046))
#define OD_DSP402_QUICK_STOP_OPTION_CODE_ID                                    (uint16_t(0x605A))
#define OD_DSP402_SHUTDOWN_OPTION_CODE_ID                                      (uint16_t(0x605B))
#define OD_DSP402_DISABLE_OPERATION_OPTION_CODE_ID                             (uint16_t(0x605C))
#define OD_DSP402_FAULT_REACTION_CODE_ID                                       (uint16_t(0x605E))
#define OD_DSP402_MODES_OF_OPERATION_ID                                        (uint16_t(0x6060))
#define OD_DSP402_MODES_OF_OPERATION_DISPLAY_ID                                (uint16_t(0x6061))
#define OD_DSP402_MEASURED_MOTOR_POSITION_ID                                   (uint16_t(0x6064))
#define OD_DSP402_MEASURED_MOTOR_VELOCITY_ID                                   (uint16_t(0x606C))
#define OD_DSP402_DESIRED_JOINT_TORQUE_ID                                      (uint16_t(0x6071))
#define OD_DSP402_JOINT_TORQUE_MAX_ID                                          (uint16_t(0x6072))
#define OD_DSP402_CURRENT_MAX_ID                                               (uint16_t(0x6073))
#define OD_DSP402_MEASURED_JOINT_TORQUE_ID                                     (uint16_t(0x6077))
#define OD_DSP402_DESIRED_JOINT_POSITION_ID                                    (uint16_t(0x607A))
#define OD_DSP402_SOFT_JOINT_POSITION_LIMIT_ID                                 (uint16_t(0x607B))
#define OD_DSP402_SOFT_JOINT_POSITION_LIMIT_SID_MIN                            (uint8_t(0x01))
#define OD_DSP402_SOFT_JOINT_POSITION_LIMIT_SID_MAX                            (uint8_t(0x02))
#define OD_DSP402_HARD_JOINT_POSITION_LIMIT_ID                                 (uint16_t(0x607D))
#define OD_DSP402_HARD_JOINT_POSITION_LIMIT_SID_MIN                            (uint8_t(0x01))
#define OD_DSP402_HARD_JOINT_POSITION_LIMIT_SID_MAX                            (uint8_t(0x02))
#define OD_DSP402_MOTOR_VELOCITY_MAX_ID                                        (uint16_t(0x6080))
#define OD_DSP402_QUICK_STOP_DECLARATION_ID                                    (uint16_t(0x6085))
#define OD_DSP402_INTERPOLATION_TIME_PERIOD_ID                                 (uint16_t(0x60C2))
#define OD_DSP402_DESIRED_JOINT_VELOCITY_ID                                    (uint16_t(0x60FF))
#define OD_DSP402_SUPPORTED_DRIVE_MODES_ID                                     (uint16_t(0x6502))

// Gains
#define OD_GAINS_CURRENT_CTRL_ID                                               (uint16_t(0x7000))
#define OD_GAINS_MOTOR_POSITION_CTRL_ID                                        (uint16_t(0x7001))
#define OD_GAINS_MOTOR_VELOCITY_CTRL_ID                                        (uint16_t(0x7002))
#define OD_GAINS_GEAR_POSITION_CTRL_ID                                         (uint16_t(0x7003))
#define OD_GAINS_GEAR_VELOCITY_CTRL_ID                                         (uint16_t(0x7004))
#define OD_GAINS_JOINT_POSITION_CTRL_ID                                        (uint16_t(0x7005))
#define OD_GAINS_JOINT_VELOCITY_CTRL_ID                                        (uint16_t(0x7006))
#define OD_GAINS_JOINT_TORQUE_CTRL_ID                                          (uint16_t(0x7007))
#define OD_GAINS_JOINT_POSITION_VELOCITY_TORQUE_CTRL_ID                        (uint16_t(0x7008))
#define OD_GAINS_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS_CTRL_ID              (uint16_t(0x7009))
#define OD_GAINS_JOINT_POSITION_VELOCITY_CTRL_ID                               (uint16_t(0x7010))
#define OD_GAINS_COMMON_SID_P                                                  (uint8_t(0x01))
#define OD_GAINS_COMMON_SID_I                                                  (uint8_t(0x02))
#define OD_GAINS_COMMON_SID_D                                                  (uint8_t(0x03))

// Extended PDO
#define OD_EXT_PDO_MEASURED_GEAR_POSITION_TICKS_ID                             (uint16_t(0x7020))
#define OD_EXT_PDO_MEASURED_JOINT_POSITION_TICKS_ID                            (uint16_t(0x7021))
#define OD_EXT_PDO_TIMESTAMP_ID                                                (uint16_t(0x7022))
#define OD_EXT_PDO_DESIRED_CURRENT_D_ID                                        (uint16_t(0x7023))
#define OD_EXT_PDO_MEASURED_CURRENT_D_ID                                       (uint16_t(0x7024))
#define OD_EXT_PDO_DESIRED_CURRENT_Q_ID                                        (uint16_t(0x7025))
#define OD_EXT_PDO_MEASURED_CURRENT_Q_ID                                       (uint16_t(0x7026))
#define OD_EXT_PDO_MEASURED_CURRENT_PHASE_U_ID                                 (uint16_t(0x7027))
#define OD_EXT_PDO_MEASURED_VOLTAGE_PHASE_U_ID                                 (uint16_t(0x7028))
#define OD_EXT_PDO_MEASURED_CURRENT_PHASE_V_ID                                 (uint16_t(0x7029))
#define OD_EXT_PDO_MEASURED_VOLTAGE_PHASE_V_ID                                 (uint16_t(0x702A))
#define OD_EXT_PDO_MEASURED_CURRENT_PHASE_W_ID                                 (uint16_t(0x702B))
#define OD_EXT_PDO_MEASURED_VOLTAGE_PHASE_W_ID                                 (uint16_t(0x702C))
#define OD_EXT_PDO_VALUE_1_ID                                                  (uint16_t(0x702D))
#define OD_EXT_PDO_VALUE_2_ID                                                  (uint16_t(0x702E))
#define OD_EXT_PDO_VALUE_3_ID                                                  (uint16_t(0x702F))
#define OD_EXT_PDO_VALUE_4_ID                                                  (uint16_t(0x7030))
#define OD_EXT_PDO_VALUE_5_ID                                                  (uint16_t(0x7031))
#define OD_EXT_PDO_VALUE_6_ID                                                  (uint16_t(0x7032))

// Calibration
#define OD_CALIB_MODE_ID                                                       (uint16_t(0x7050))
#define OD_CALIB_MODE_ID_VAL_IDLE                                              (uint16_t(0x0000))
#define OD_CALIB_MODE_ID_VAL_MOTOR_ENCODER_OFFSET                              (uint16_t(0x0001))
#define OD_CALIB_MODE_ID_VAL_MOTOR_ENCODER_PARAMETERS                          (uint16_t(0x0002))
#define OD_CALIB_MODE_ID_VAL_GEAR_JOINT_ENCODER_OFFSET                         (uint16_t(0x0003))
#define OD_CALIB_MODE_ID_VAL_GEAR_AND_JOINT_ENCODER_HOMING                     (uint16_t(0x0004))
#define OD_CALIB_MODE_ID_VAL_GEAR_JOINT_ENCODER_OFFSET_END                     (uint16_t(0x0005))
#define OD_CALIB_MODE_ID_VAL_IMU_GYROSCOPE_DC_BIAS                             (uint16_t(0x0006))
#define OD_CALIB_MODE_ID_VAL_FRICTION_ESTIMATION                               (uint16_t(0x0007))
#define OD_CALIB_MODE_ID_VAL_FRICTION_ESTIMATION_END                           (uint16_t(0x0008))
#define OD_CALIB_MODE_ID_VAL_SPRING_STIFFNESS                                  (uint16_t(0x0009))
#define OD_CALIB_MODE_ID_VAL_SPRING_STIFFNESS_END                              (uint16_t(0x000A))
#define OD_CALIB_MODE_ID_VAL_AKSIM_SELF_CALIB                                  (uint16_t(0x000B))
#define OD_CALIB_STATES_ID                                                     (uint16_t(0x7051))
#define OD_CALIB_FACTORY_TO_CUSTOM_ID                                          (uint16_t(0x7052))
#define OD_CALIB_CUSTOM_TO_FACTORY_ID                                          (uint16_t(0x7053))
#define OD_CALIB_CUSTOM_TO_FACTORY_ID_VAL_IDLE                                 (uint16_t(0x0000))
#define OD_CALIB_CUSTOM_TO_FACTORY_ID_VAL_RUN                                  (uint16_t(0x0001))
#define OD_CALIB_GEAR_AND_JOINT_ENCODER_HOMING_NEW_JOINT_POSITION_ID           (uint16_t(0x7054))
#define OD_CALIB_GEAR_AND_JOINT_ENCODER_HOMING_TICKS_ID                        (uint16_t(0x7055))
#define OD_CALIB_GEAR_AND_JOINT_ENCODER_HOMING_TICKS_SID_GEAR                  (uint8_t(0x01))
#define OD_CALIB_GEAR_AND_JOINT_ENCODER_HOMING_TICKS_SID_JOINT                 (uint8_t(0x02))
#define OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_ID                                  (uint16_t(0x7056))
#define OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_CONSTANT                        (uint8_t(0x01))
#define OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_SIN1_AMPLITUDE                  (uint8_t(0x02))
#define OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_SIN1_PHASESHIFT                 (uint8_t(0x03))
#define OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_SIN2_AMPLITUDE                  (uint8_t(0x04))
#define OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_SIN2_PHASESHIFT                 (uint8_t(0x05))
#define OD_CALIB_SPRING_STIFFNESS_ID                                           (uint16_t(0x7057))
#define OD_CALIB_SPRING_STIFFNESS_SID_NEG                                      (uint8_t(0x01))
#define OD_CALIB_SPRING_STIFFNESS_SID_POS                                      (uint8_t(0x02))
#define OD_CALIB_MOTOR_ENCODER_PARAMS_ID                                       (uint16_t(0x7058))
#define OD_CALIB_MOTOR_ENCODER_PARAMS_SID_OFFSET                               (uint8_t(0x01))
#define OD_CALIB_MOTOR_ENCODER_PARAMS_SID_DGAIN                                (uint8_t(0x02))
#define OD_CALIB_MOTOR_ENCODER_PARAMS_SID_DOFFS                                (uint8_t(0x03))
#define OD_CALIB_MOTOR_ENCODER_PARAMS_SID_DOFFC                                (uint8_t(0x04))
#define OD_CALIB_MOTOR_ENCODER_PARAMS_SID_DPH                                  (uint8_t(0x05))
#define OD_CALIB_MOTOR_ENCODER_PARAMS_SID_AGAIN                                (uint8_t(0x06))
#define OD_CALIB_MOTOR_ENCODER_PARAMS_SID_AOFFS                                (uint8_t(0x07))
#define OD_CALIB_MOTOR_ENCODER_PARAMS_SID_AOFFC                                (uint8_t(0x08))
#define OD_CALIB_FRICTION_ESTIMATION_ID                                        (uint16_t(0x7059))
#define OD_CALIB_FRICTION_ESTIMATION_SID_BREAK_AWAY_FRICTION                   (uint8_t(0x01))
#define OD_CALIB_FRICTION_ESTIMATION_SID_BREAK_AWAY_FRICTION_BAND              (uint8_t(0x02))
#define OD_CALIB_FRICTION_ESTIMATION_SID_VISCOUS_FRICTION_COEFF_NEG            (uint8_t(0x03))
#define OD_CALIB_FRICTION_ESTIMATION_SID_VISCOUS_FRICTION_COEFF_POS            (uint8_t(0x04))
#define OD_CALIB_SELECTION_ID                                                  (uint16_t(0x705A))
#define OD_CALIB_SELECTION_ID_VAL_CUSTOM                                       (uint16_t(0x0000))
#define OD_CALIB_SELECTION_ID_VAL_FACTORY                                      (uint16_t(0x0001))
#define OD_CALIB_IMU_GYROSCOPE_DC_BIAS_ID                                      (uint16_t(0x705B))
#define OD_CALIB_IMU_GYROSCOPE_DC_BIAS_SID_X                                   (uint8_t(0x01))
#define OD_CALIB_IMU_GYROSCOPE_DC_BIAS_SID_Y                                   (uint8_t(0x02))
#define OD_CALIB_IMU_GYROSCOPE_DC_BIAS_SID_Z                                   (uint8_t(0x03))

// Drive info
#define OD_DRIVE_INFO_FIRMWARE_VERSION_ID                                      (uint16_t(0x7070))
#define OD_DRIVE_INFO_BOOTLOADER_VERSION_ID                                    (uint16_t(0x7071))
#define OD_DRIVE_INFO_HARDWARE_SERIAL_NUMBER_ID                                (uint16_t(0x7072))
#define OD_DRIVE_INFO_DRIVE_NAME_ID                                            (uint16_t(0x7073))
#define OD_DRIVE_INFO_DRIVE_ID_ID                                              (uint16_t(0x7074))
#define OD_DRIVE_FIRMWARE_INFO_ID                                              (uint16_t(0x7075))
#define OD_DRIVE_FIRMWARE_INFO_SID_0                                           (uint8_t(0x00))
#define OD_DRIVE_FIRMWARE_INFO_SID_DATA                                        (uint8_t(0x01))
#define OD_GEARBOX_RATIO_ID                                                    (uint16_t(0x7076))
#define OD_BUILDINFO_ID                                                        (uint16_t(0x7078))
#define OD_DRIVETYPE_ID                                                        (uint16_t(0x7077))

// Flash
#define OD_FLASH_ERASE_ID                                                      (uint16_t(0x70A0))
#define OD_FLASH_ERASE_ID_VAL_IDLE                                             (uint16_t(0x0000))
#define OD_FLASH_ERASE_ID_VAL_RUN                                              (uint16_t(0x0001))
#define OD_FLASH_RESET_ID                                                      (uint16_t(0x70A1))
#define OD_FLASH_RESET_ID_VAL_RESET_CALIBRATION                                (uint16_t(0x0000))
#define OD_FLASH_RESET_ID_VAL_RESET_CONFIGURATION                              (uint16_t(0x0001))
#define OD_FLASH_WRITE_CONFIGURATION_ID                                        (uint16_t(0x70A2))
#define OD_FLASH_WRITE_CONFIGURATION_ID_VAL_IDLE                               (uint16_t(0x0000))
#define OD_FLASH_WRITE_CONFIGURATION_ID_VAL_RUN                                (uint16_t(0x0001))

// FSM
#define OD_FSM_ERROR_BEHAVIOR_ID                                               (uint16_t(0x70C0))
#define OD_FSM_ERROR_BEHAVIOR_ID_VAL_FREEZE                                    (uint16_t(0x0000))
#define OD_FSM_ERROR_BEHAVIOR_ID_VAL_DISABLE_BRIDGE                            (uint16_t(0x0001))

// Filter
#define OD_FILTER_GEAR_JOINT_VELOCITY_ID                                       (uint16_t(0x70E0))
#define OD_FILTER_GEAR_JOINT_VELOCITY_SID_FILTER_TYPE                          (uint8_t(0x01))
#define OD_FILTER_GEAR_JOINT_VELOCITY_SID_FILTER_TYPE_VAL_DISABLED             (uint32_t( \
    0x00000000))
#define OD_FILTER_GEAR_JOINT_VELOCITY_SID_FILTER_TYPE_VAL_KALMAN               (uint32_t( \
    0x00000001))
#define OD_FILTER_GEAR_JOINT_VELOCITY_SID_FILTER_TYPE_VAL_IIR                  (uint32_t( \
    0x00000002))
#define OD_FILTER_GEAR_JOINT_VELOCITY_SID_FILTER_TYPE_VAL_EMA                  (uint32_t( \
    0x00000002))
#define OD_FILTER_GEAR_JOINT_VELOCITY_SID_KF_NOISE_VARIANCE                    (uint8_t(0x02))
#define OD_FILTER_GEAR_JOINT_VELOCITY_SID_KF_LAMBDA_2                          (uint8_t(0x03))
#define OD_FILTER_GEAR_JOINT_VELOCITY_SID_KF_GAMMA                             (uint8_t(0x04))
#define OD_FILTER_GEAR_JOINT_VELOCITY_SID_IIR_ALPHA                            (uint8_t(0x05))
#define OD_FILTER_GEAR_JOINT_VELOCITY_SID_EMA_ALPHA                            (uint8_t(0x05))
#define OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID                           (uint16_t(0x70E1))
#define OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_FILTER_TYPE              (uint8_t(0x01))
#define OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_FILTER_TYPE_VAL_DISABLED (uint32_t( \
    0x00000000))
#define OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_FILTER_TYPE_VAL_KALMAN   (uint32_t( \
    0x00000001))
#define OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_FILTER_TYPE_VAL_IIR      (uint32_t( \
    0x00000002))
#define OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_FILTER_TYPE_VAL_EMA      (uint32_t( \
    0x00000002))
#define OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_KF_NOISE_VARIANCE        (uint8_t(0x02))
#define OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_KF_LAMBDA_2              (uint8_t(0x03))
#define OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_KF_GAMMA                 (uint8_t(0x04))
#define OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_IIR_ALPHA                (uint8_t(0x05))
#define OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_EMA_ALPHA                (uint8_t(0x05))
#define OD_FILTER_JOINT_ACCELERATION_ID                                        (uint16_t(0x70E2))
#define OD_FILTER_JOINT_ACCELERATION_SID_FILTER_TYPE                           (uint8_t(0x01))
#define OD_FILTER_JOINT_ACCELERATION_SID_FILTER_TYPE_VAL_DISABLED              (uint32_t( \
    0x00000000))
#define OD_FILTER_JOINT_ACCELERATION_SID_FILTER_TYPE_VAL_KALMAN                (uint32_t( \
    0x00000001))
#define OD_FILTER_JOINT_ACCELERATION_SID_FILTER_TYPE_VAL_IIR                   (uint32_t( \
    0x00000002))
#define OD_FILTER_JOINT_ACCELERATION_SID_FILTER_TYPE_VAL_EMA                   (uint32_t( \
    0x00000002))
#define OD_FILTER_JOINT_ACCELERATION_SID_KF_NOISE_VARIANCE                     (uint8_t(0x02))
#define OD_FILTER_JOINT_ACCELERATION_SID_KF_LAMBDA_2                           (uint8_t(0x03))
#define OD_FILTER_JOINT_ACCELERATION_SID_KF_GAMMA                              (uint8_t(0x04))
#define OD_FILTER_JOINT_ACCELERATION_SID_IIR_ALPHA                             (uint8_t(0x05))
#define OD_FILTER_JOINT_ACCELERATION_SID_EMA_ALPHA                             (uint8_t(0x05))
#define OD_FILTER_D_GAIN                                                       (uint16_t(0x7604))

// Control
#define OD_CONTROL_DIRECTION_ID                                                (uint16_t(0x7100))
#define OD_CONTROL_RTDL_CONTROL_ID                                             (uint16_t(0x7101))
#define OD_CONTROL_RTDL_CONTROL_SID_ENABLE                                     (uint8_t(0x01))
#define OD_CONTROL_RTDL_CONTROL_SID_ENABLE_VAL_DISABLE                         (uint16_t(0x00))
#define OD_CONTROL_RTDL_CONTROL_SID_ENABLE_VAL_ENABLE                          (uint16_t(0x01))
#define OD_CONTROL_RTDL_CONTROL_SID_COMMAND                                    (uint8_t(0x02))
#define OD_CONTROL_RTDL_CONTROL_SID_COMMAND_VAL_STOP                           (uint16_t(0x00))
#define OD_CONTROL_RTDL_CONTROL_SID_COMMAND_VAL_RESET                          (uint16_t(0x01))
#define OD_CONTROL_RTDL_CONTROL_SID_COMMAND_VAL_LOG                            (uint16_t(0x02))
#define OD_CONTROL_RTDL_CONTROL_SID_COMMAND_VAL_STREAM                         (uint16_t(0x03))
#define OD_CONTROL_RTDL_CONTROL_SID_STATUS                                     (uint8_t(0x03))
#define OD_CONTROL_RTDL_CONTROL_SID_STATUS_VAL_IDLE                            (uint16_t(0x00))
#define OD_CONTROL_RTDL_CONTROL_SID_STATUS_VAL_LOGGING                         (uint16_t(0x01))
#define OD_CONTROL_RTDL_CONTROL_SID_STATUS_VAL_STREAMING                       (uint16_t(0x02))
#define OD_CONTROL_RTDL_CONTROL_SID_LOGGING_FREQUENCY                          (uint8_t(0x04))
#define OD_CONTROL_RTDL_CONTROL_SID_LOGGING_FREQUENCY_VAL_2_5_Hz               (uint16_t(0x00))
#define OD_CONTROL_RTDL_CONTROL_SID_LOGGING_FREQUENCY_VAL_10_Hz                (uint16_t(0x01))
#define OD_CONTROL_RTDL_CONTROL_SID_STREAMING_FREQUENCY                        (uint8_t(0x05))
#define OD_CONTROL_RTDL_LAST_TIMESTAMP_ID                                      (uint16_t(0x7103))
#define OD_CONTROL_IMU_CONFIG_ID                                               (uint16_t(0x7105))
#define OD_CONTROL_IMU_CONFIG_SID_ENABLE                                       (uint8_t(0x01))
#define OD_CONTROL_IMU_CONFIG_SID_ACCELEROMETER_RANGE                          (uint8_t(0x02))
#define OD_CONTROL_IMU_CONFIG_SID_GYROSCOPE_RANGE                              (uint8_t(0x03))
#define OD_CONTROL_FAN_ID                                                      (uint16_t(0x7106))
#define OD_CONTROL_FAN_SID_MODE                                                (uint8_t(0x1))
#define OD_CONTROL_FAN_SID_INTENSITY                                           (uint8_t(0x2))
#define OD_CONTROL_FAN_SID_LOWER_TEMPERATURE                                   (uint8_t(0x3))
#define OD_CONTROL_FAN_SID_UPPER_TEMPERATURE                                   (uint8_t(0x4))
#define OD_CONTROL_BRAKE_ID                                                    (uint16_t(0x7107))
#define OD_CONTROL_BRAKE_SID_CTRL                                              (uint8_t(0x1))
#define OD_CONTROL_BRAKE_SID_DUTY                                              (uint8_t(0x2))
#define OD_CONTROL_BRAKE_SID_CTRL_VAL_ENABLE                                   (uint16_t(0x1))
#define OD_CONTROL_BRAKE_SID_CTRL_VAL_DISABLE                                  (uint16_t(0x0))

// Various
#define OD_VARIOUS_PASSWORD_ID                                                 (uint16_t(0x7200))

// Configuration
#define OD_CONFIG_CURRENT_INTEGRATOR_SATURATION                                (uint16_t(0x7600))
#define OD_CONFIG_JOINT_TORQUE_INTEGRATOR_SATURATION                           (uint16_t(0x7601))
#define OD_CONFIG_MAX_FREEZE_CURRENT                                           (uint16_t(0x7603))

// Data Logging
#define OD_DATALOGGING                                                         (uint16_t(0x7800))
#define OD_DATALOGGING_HIST_EDGES                                              (uint16_t(0x7801))

#define OD_SWITCH_PDO_ID                                                       (uint16_t(0xF030))
#define OD_SWITCH_PDO_SID                                                      (uint8_t(0x01))
