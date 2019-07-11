/*
 *  goldentwitter.hpp
 *  Descriotion:
 *
 *  Created on: Jue, 5, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#ifndef GOLDENTWITTER_HPP
#define GOLDENTWITTER_HPP


#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>

#define TWITTER_GEAR_RATIO 18.0625
#define TWITTER_ENCODER_RES 262144
#define TWITTER_CURRENT_TORQUE_RATIO 16.67 //1000 means max current 25[Amp]



typedef enum GoldenTwitterState{ //bit 0-3,5,6; result & 0b01101111
  NOT_READY_TO_SWITCH_ON  = 0b00000000,
  SWITCH_ON_DISABLED      = 0b01000000,
  READY_TO_SWITCH_ON      = 0b00100001,
  SWITCHED_ON             = 0b00100011,
  OP_ENABLED              = 0b00100111,
  QUICK_STOP_ACTIVED      = 0b00000111,
  FAULT                   = 0b00001000,
}GoldenTwitterState;

typedef enum GoldenTwitterControlWord{// bit 0-3,7, result &0b10001111
  SHUT_DOWN             = 0b00000110,//6
  SWITCH_ON             = 0b00000111,//7
  SWITCH_ON_AND_ENABLE  = 0b00001111,//15
  QUICK_STOP            = 0b00000010,
  DISABLE_OP            = 0b00000111,
  ENABLE_OP             = 0b00001111,
  DISABLE_VOLTAGE       = 0b00000000,
  FAULT_RESET           = 0b10000000, //upward trigger 128
}GoldenTwitterControlWord;

typedef enum GoldenTwitterModeOfOperation{
  PROFILE_POSITION                    = 1,
  PROFILE_VELOCITY                    = 3,
  PROFILE_TORQUE                      = 4,
  HOMING                              = 6,
  SYNC_POSITION                      = 8,
  SYNC_VELOCITY                      = 9,
  SYNC_TORQUE                        = 10,
}GoldenTwitterModeOfOperation;



//typedef enum PDOTYPE{
//  RPDO_A = 0x16000001,
//  TPDO_A = 0x1A000001,
//  RPDO_B = 0x16010001,
//  TPDO_B = 0x1A010001,
//  RPDO_C = 0x16020001,
//  TPDO_C = 0x1A020001,
//  RPDO_D = 0x16030001,
//  TPDO_D = 0x1A030001,
//  RPDO_E = 0x16040001,
//  TPDO_E = 0x1A040001,
//  RPDO_F = 0x16050001,
////  TPDO_F = 0x1A050001,
//  RPDO_G = 0x16060001,
////  TPDO_G = 0x1A060001,
//}PDOTYPE;

//! WSHY: EtherCAT datagrams of GoldenTwitter of Type A, refer to the GoldenTwitter document Page 36
struct GoldenTwitterRPDOA{
  int32_t target_position;  // double rad     0x607A
  uint32_t digital_outputs; //                 0x60FE
  uint16_t control_word;         // uint16    0x6040
};

struct GoldenTwitterRPDOB{
  int32_t target_velocity;  // double rad     0x60FF
  uint16_t control_word;         // uint16    0x6040
};

struct GoldenTwitterRPDOC{
  int16_t target_torque;  // double rad     0x6071
  uint16_t control_word;         // uint16    0x6040
};

struct GoldenTwitterRPDOD{
  int32_t target_position;  // double rad     0x607A
  uint32_t digital_outputs; //                 0x60FE
  int32_t velocity_offset;  // double rad     0x60B1
  uint16_t control_word;         // uint16    0x6040
};

struct GoldenTwitterRPDOE{
  int32_t target_position;  // double rad     0x607A
  int32_t target_velocity;  // double rad     0x60FF
  uint16_t max_torque;      //                0x6072
  uint16_t control_word;         // uint16    0x6040
};

struct GoldenTwitterRPDOF{
  int32_t target_position;  // double rad     0x607A
  int32_t target_velocity;  // double rad     0x60FF
  int16_t target_torque;  // double rad       0x6071
  uint16_t max_torque;      //                0x6072
  uint16_t control_word;         // uint16    0x6040
  uint16_t mode_of_operation;   // float C    0x6060
};

struct GoldenTwitterRPDOG{
  int32_t target_position;  // double rad     0x607A
  uint32_t digital_outputs; //                 0x60FE
  int32_t target_velocity;  // double rad     0x60FF
  int32_t velocity_offset;  // double rad     0x60B1
  int16_t torque_offset;   // float C         0x60B2
  uint16_t control_word;         // uint16    0x6040
};


struct GoldenTwitterTPDOA{
  int32_t motor_position; // double rad       0x6064
  uint32_t digital_inputs; //                 0x60FD
  uint16_t status_word;   // uint16           0x6041
};

struct GoldenTwitterTPDOB{
  int32_t motor_position; // double rad       0x6064
  int32_t motro_position_demand; //           0x606B
  int16_t motor_torque_demand;//              0x6074
  uint16_t status_word;   // uint16           0x6041
};

struct GoldenTwitterTPDOC{
  int32_t motor_position; // double rad       0x6064
  int16_t motor_torque;//                     0x6077
  uint16_t status_word;   // uint16           0x6041
  int8_t mode_of_opreration; //              0x6061
};

struct GoldenTwitterTPDOD{
  int32_t motor_position; // double rad       0x6064
  uint32_t digital_inputs; //                 0x60FD
  int32_t motor_velocity; // float krpm       0x606C
  uint16_t status_word;   // uint16           0x6041
};

struct GoldenTwitterTPDOE{
  int32_t motor_position; // double rad       0x6064
  int16_t position_error; //                  0x60F4
  int16_t motor_torque;//                     0x6077
  uint16_t status_word;   // uint16           0x6041
  int8_t mode_of_opreration; //              0x6061
};


struct GoldenTwitterTPDOF{
  int32_t motor_velocity; // cnts/sec     0x6069
  int32_t motor_position; // double rad       0x6064
  int16_t motor_torque;//                     0x6077
  uint16_t status_word;   // uint16           0x6041
  int8_t mode_of_opreration; //              0x6061
};

#endif // GOLDENTWITTER_HPP
