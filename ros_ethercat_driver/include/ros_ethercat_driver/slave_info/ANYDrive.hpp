/*
 *  ANYDrive.hpp
 *  Descriotion:
 *
 *  Created on: May, 20, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#pragma once
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>



typedef enum ANYDriveFSMState{
  COLD_START  = 1,
  WARM_START  = 2,
  CONFIGURE   = 3,
  CALIBRATE   = 4,
  STANDBY     = 5,
  MOTOR_OP    = 6,
  CONTROL_OP  = 7,
  ERROR       = 8,
  FATAL       = 9,
  MOTOR_PRE_OP= 10,
}ANYDriveFSMState;

typedef enum ANYDriveModeOfOperation{
  FREEZE                              = 1,
  DISABLE                             = 2,
  CURRENT                             = 3,
  MOTOR_POSITION                      = 4,
  MOTOR_VELOCITY                      = 5,
  GEAR_POSITION                       = 6,
  GEAR_VELOCITY                       = 7,
  JOINT_POSITION                      = 8,
  JOINT_VELOCITY                      = 9,
  JOINT_TORQUE                        = 10,
  JOINT_POSITION_VELOCITY             = 11,
  JOINT_POSITION_VELOCITY_TORQUE      = 12,
  JOINT_POSITION_VELOCITY_TORQUE_PID  = 13,

}ANYDriveModeOfOperation;

typedef enum ANYDriveControlWord{
  WARM_RESET                = 1,
  CLEAR_ERRORS_TO_MOTOR_OP  = 2,
  STANDBY_TO_CONFIGURE      = 3,
  CONFIGURE_TO_STANDBY      = 4,
  CALIBRATE_TO_CONFIGURE    = 5,
  CONFIGURE_TO_CALIBRATE    = 6,
  MOTOR_OP_TO_STANDBY       = 7,
  STANDBY_TO_MOTOR_PRE_OP   = 8,
  CONTROL_OP_TO_MOTOR_OP    = 9,
  MOTOR_OP_TO_CONTROL_OP    = 10,
  CONTROL_OP_TO_STANDBY     = 11,
  CLEAR_ERRORS_TO_STANDBY   = 12,
}ANYDriveControlWord;

//typedef enum PDOTYPE{
//  RPDO_A = 0x16000001,
//  TPDO_A = 0x1A000001,
//  RPDO_B = 0x16010001,
//  TPDO_B = 0x1A010001,
//  RPDO_C = 0x16020001,
//  TPDO_C = 0x1A020001,
//  RPDO_D = 0x16030001,
//  TPDO_D = 0x1A030001,
//}PDOTYPE;

//! WSHY: EtherCAT datagrams of ANYDrive of Type A, refer to the ANYDrive document Page 36
struct ANYDriveRPDOA{
  uint16_t control_word;         // uint16    0x6040
  uint16_t mode_of_operation;   // float C    0x6060
  int32_t desired_motor_current;  // float A  0x2020
  uint32_t desired_velocity; // float krpm    0x60FF
  int32_t desired_joint_torque; // float NM   0x6071
  int64_t desired_position; // double rad     0x607A
  int32_t control_parameter_a;  // float      0x2024
  int32_t control_parameter_b; // float       0x2025
  int32_t control_parameter_c; //float        0x2026
  int32_t control_parameter_d;   // float     0x2027
};


struct ANYDriveTPDOA{
  uint32_t state;         // uint32           0x6061 4bit FSM stste, 4bit MOP, 24bit,warnnings, errors, fatals
  uint16_t temperature;   // float C          0x2002
  uint16_t voltage;       // float V          0x2003
  int64_t motor_position; // double rad       0x6064
  int64_t gear_position;  // double rad       0x2005
  int64_t joint_position; // double rad       0x2006
  int32_t motor_current;  // float A          0x2007
  int32_t motor_velocity; // float krpm       0x606C
  int32_t gear_velocity;  // float rpm        0x2009
  int32_t joint_velocity; // float rpm        0x200A
  int32_t joint_acceleration; //float rpm/s   0x200B
  int32_t joint_torque;   // float Nm         0x6077
};

//template<class T>
//class ANYDrivePDO
//{
//  std::vector<T> feedbacks;
//  std::vector<T> commands;
//};
