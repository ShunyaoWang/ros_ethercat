/*
 *  ros_ethercat_hardware_interface.hpp
 *  Descriotion:Ros EtherCAT Hardware Interface for RT control
 *
 *  Created on: MAY 20, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#pragma once
// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/robot_hw.h>
#include "hardware_interface/imu_sensor_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

#include "robot_state_interface.hpp"

#include "transmission_interface/transmission_info.h"
#include <transmission_interface/transmission_parser.h>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <ros/callback_queue.h>

// URDF
#include <urdf/model.h>

// Soem
#include "soem/soem/ethercat.h"

// slaves
#include "ros_ethercat_driver/slave_info/ANYDrive.hpp"
#include "ros_ethercat_driver/slave_info/goldentwitter.hpp"

//#include "ros_ethercat_driver/anydrive_test.h"
//free_gait
//#include "free_gait_msgs/RobotState.h"

#include "eigen3/Eigen/Eigen"
#include "unordered_map"

#define EC_TIMEOUTMON 500

namespace ros_ethercat_driver {

  typedef enum PDOTYPE{
    RPDO_A = 0x16000001,
    TPDO_A = 0x1A000001,
    RPDO_B = 0x16010001,
    TPDO_B = 0x1A010001,
    RPDO_C = 0x16020001,
    TPDO_C = 0x1A020001,
    RPDO_D = 0x16030001,
    TPDO_D = 0x1A030001,
    RPDO_E = 0x16040001,
    TPDO_E = 0x1A040001,
    RPDO_F = 0x16050001,
    TPDO_F = 0x1A050001,
    RPDO_G = 0x16060001,
    TPDO_G = 0x1A060001,
  }PDOTYPE;

  typedef union {
    int32_t int32data;
    uint32_t uint32data;
    float floatdata;
  }Bytes4Exchage;

  typedef union {
    int64_t int64data;
    uint64_t uint64data;
    double doubledata;
  }Bytes8Exchage;

  typedef enum {ANYDRIVE, GOLDENTWITTER, JUNCTION} SlaveType;

class RobotStateEtherCATHardwareInterface : public hardware_interface::RobotHW//, public hardware_interface::HardwareInterface
{

//  typedef std::unordered_map<PDOTYPE, struct ANYDriveRPDOA*> RPDO_Types;
//  typedef std::unordered_map<PDOTYPE, struct ANYDriveTPDOA*> TPDO_Types;
public:
  RobotStateEtherCATHardwareInterface();
  ~RobotStateEtherCATHardwareInterface();
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);

  void read(const ros::Time& time, const ros::Duration& period);

  void write(const ros::Time& time, const ros::Duration& period);

  virtual void eStopActive(const bool active);

  bool EtherCATInit();
  void EtherCATLoop();
  void EtherCATLoop(const ros::TimerEvent&);
  bool InitSlaves(const std::vector<SlaveType>& slave_type);
  bool DeInitSlaves(const std::vector<SlaveType>& slave_type);

  void EtherCATTimerThread();
  void shutdown();

  bool createEtherCATLoopThread();
  bool createEtherCATCheckThread();

  char* getSlaveName(int index);
  char* getSlaveECState(int index);
  uint16 getSlaveAddress(int index);
  char* getSlaveStatus(int index, const std::string& name);
  char* getSlaveMode(int index, const std::string& name);
  Eigen::Vector3d getJointFeedback(int index, const std::string& name);
  int getNumberOfSlaves();

  bool setPDOType(const std::string& name, const std::string& type);

  bool setSlaveCW(int id, const std::string& name, const std::string& control_word);

  bool setCommand(int id, const std::string& mode_of_operation, double command);
  bool setModeOfOperation(int id, const std::string& mode_of_operation);
  bool setControlMethod(const std::string& method);

  bool writeSDO(uint16 id, uint16 index, uint8 sub_index, void *value, int size);
  bool readSDO(uint16 id, uint16 index, uint8 sub_index, char* result);

//  void readJoints();
//  void writeJoints();
//  void readStates();
  char *ifname; // name of network card, default eno1;
protected:

  ros::NodeHandle node_handle_;

  ros::Timer EtherCATTimer_;
  ros::CallbackQueue EtherCATUpdateQueue_;

  boost::thread EtherCATTimerThread_;

  boost::thread EtherCATLoopThread_;

  boost::recursive_mutex r_mutex_;

  Bytes4Exchage exchage4bytes_;
  Bytes8Exchage exchage8bytes_;

  // Methods used to control a joint.
  enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID, STANCE_LEG, FREEZE};

  // Register the limits of the joint specified by joint_name and joint_handle. The limits are
  // retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
  // Return the joint's type, lower position limit, upper position limit, and effort limit.
  void registerJointLimits(const std::string& joint_name,
                           const hardware_interface::JointHandle& joint_handle,
                           const ControlMethod ctrl_method,
                           const ros::NodeHandle& joint_limit_nh,
                           const urdf::Model *const urdf_model,
                           int *const joint_type, double *const lower_limit,
                           double *const upper_limit, double *const effort_limit);

  unsigned int n_dof_;

  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;
  hardware_interface::ImuSensorInterface imu_interface_;
  hardware_interface::ImuSensorHandle::Data imu_data_;
  //!
  //! \brief robot_state_interface_, a robot state interface contain robot base state
  //! and joint states, see the robot_state_interface.hpp
  //!
  hardware_interface::RobotStateInterface robot_state_interface_;
  //!
  //! \brief robot_state_data_, robot_state data handle
  //!
  hardware_interface::RobotStateHandle::Data robot_state_data_;

  joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
  joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;

  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_effort_limits_;
  std::vector<ControlMethod> joint_control_methods_;
  std::vector<control_toolbox::Pid> pid_controllers_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_effort_command_;
  std::vector<double> joint_position_command_;
  std::vector<double> last_joint_position_command_;
  std::vector<double> joint_velocity_command_;

//  std::vector<gazebo::physics::JointPtr> sim_joints_;

//  gazebo::sensors::ContactSensorPtr contact_sensor_ptr;

//  gazebo::sensors::SensorPtr sensor_ptr;

//  gazebo::physics::LinkPtr base_link_ptr_, lf_foot_link_ptr_, rf_foot_link_ptr_, rh_foot_link_ptr_, lh_foot_link_ptr_;

  std::string physics_type_;

  // e_stop_active_ is true if the emergency stop is active.
  bool e_stop_active_, last_e_stop_active_;

  double pos_read[12], pos_write[12], vel_read[12], vel_write[12], eff_read[12],eff_write[12];
  double position[3], orinetation[4], linear_vel[3], angular_vel[3];
//  free_gait_msgs::RobotState actual_robot_state_;

  std::vector<transmission_interface::TransmissionInfo> transmissions_;

private:



//  char *ifname; // name of network card, default eno1;

  /** cache for EEPROM read functions */
  static uint8            ec_esibuf[EC_MAXEEPBUF];
  /** bitmap for filled cache buffer bytes */
  static uint32           ec_esimap[EC_MAXEEPBITMAP];
  /** current slave for EEPROM cache buffer */
  static ec_eringt        ec_elist;
  static ec_idxstackT     ec_idxstack;

  /** SyncManager Communication Type struct to store data of one slave */
  static ec_SMcommtypet   ec_SMcommtype[EC_MAX_MAPT];
  /** PDO assign struct to store data of one slave */
  static ec_PDOassignt    ec_PDOassign[EC_MAX_MAPT];
  /** PDO description struct to store data of one slave */
  static ec_PDOdesct      ec_PDOdesc[EC_MAX_MAPT];

  /** buffer for EEPROM SM data */
  static ec_eepromSMt     ec_SM;
  /** buffer for EEPROM FMMU data */
  static ec_eepromFMMUt   ec_FMMU;

  static char IOmap[4096];
  /** Main slave data array.
   *  Each slave found on the network gets its own record.
   *  ec_slave[0] is reserved for the master. Structure gets filled
   *  in by the configuration function ec_config().
   */
  static ec_slavet               ec_slave[EC_MAXSLAVE];
  /** number of slaves found on the network */
  static int                     ec_slavecount;
  /** slave group structure */
  static ec_groupt               ec_group[EC_MAXGROUP];

  /** Global variable TRUE if error available in error stack */
  static boolean                 EcatError;// = FALSE;

  static int64                   ec_DCtime;

  static ecx_portt               ecx_port;
  static ecx_redportt            ecx_redport;

  static ecx_contextt  ecx_context;

//  ecx_contextt  ecx_context = {
//      &ecx_port,          // .port          =
//      &ec_slave[0],       // .slavelist     =
//      &ec_slavecount,     // .slavecount    =
//      EC_MAXSLAVE,        // .maxslave      =
//      &ec_group[0],       // .grouplist     =
//      EC_MAXGROUP,        // .maxgroup      =
//      &ec_esibuf[0],      // .esibuf        =
//      &ec_esimap[0],      // .esimap        =
//      0,                  // .esislave      =
//      &ec_elist,          // .elist         =
//      &ec_idxstack,       // .idxstack      =
//      &EcatError,         // .ecaterror     =
//      0,                  // .DCtO          =
//      0,                  // .DCl           =
//      &ec_DCtime,         // .DCtime        =
//      &ec_SMcommtype[0],  // .SMcommtype    =
//      &ec_PDOassign[0],   // .PDOassign     =
//      &ec_PDOdesc[0],     // .PDOdesc       =
//      &ec_SM,             // .eepSM         =
//      &ec_FMMU,           // .eepFMMU       =
//      NULL//,               // .FOEhook()
////      NULL                // .EOEhook()
//  };

  int expectedWKC;
  boolean needlf;
  volatile int wkc;
  boolean inOP;
  uint8 currentgroup = 0;

  boost::thread EtherCATCheckThread_;
  void EtherCATCheck();

  //! WSHY: read and write data struct
//  struct ANYDriveTPDOA *feedback;
//  struct ANYDriveRPDOA *command;



  std::vector<struct ANYDriveTPDOA*> feedbacks_;
  std::vector<struct ANYDriveRPDOA*> commands_;

  std::vector<uint8 *> all_type_feedbacks;

  PDOTYPE ANYdriveTPDOType_, ANYdriveRPDOType_;
  PDOTYPE GoldenTwitterTPDOType_, GoldenTwitterRPDOType_;
  std::vector<SlaveType> slaves_type_;

  int motor_slave_num_;

};


typedef boost::shared_ptr<RobotStateEtherCATHardwareInterface> RobotStateEtherCATHardwareInterfacePtr;

};
