/*
 *  gazebo_state_hardware_interface.cpp
 *  Descriotion: based on the default_robot_hw_sim in gazebo_ros_control, and a
 *               imu sensor interface.
 *
 *  Created on: Mar 19, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#include "ros_ethercat_driver/hardware_interface/ros_ethercat_hardware_interface.hpp"
namespace
{

double clamp(const double val, const double min_val, const double max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

}

namespace ros_ethercat_driver
{

  /** cache for EEPROM read functions */
  uint8            RobotStateEtherCATHardwareInterface::ec_esibuf[EC_MAXEEPBUF];
  /** bitmap for filled cache buffer bytes */
  uint32           RobotStateEtherCATHardwareInterface::ec_esimap[EC_MAXEEPBITMAP];
  /** current slave for EEPROM cache buffer */
  ec_eringt        RobotStateEtherCATHardwareInterface::ec_elist;
  ec_idxstackT     RobotStateEtherCATHardwareInterface::ec_idxstack;

  /** SyncManager Communication Type struct to store data of one slave */
  ec_SMcommtypet   RobotStateEtherCATHardwareInterface::ec_SMcommtype[EC_MAX_MAPT];
  /** PDO assign struct to store data of one slave */
  ec_PDOassignt    RobotStateEtherCATHardwareInterface::ec_PDOassign[EC_MAX_MAPT];
  /** PDO description struct to store data of one slave */
  ec_PDOdesct      RobotStateEtherCATHardwareInterface::ec_PDOdesc[EC_MAX_MAPT];

  /** buffer for EEPROM SM data */
  ec_eepromSMt     RobotStateEtherCATHardwareInterface::ec_SM;
  /** buffer for EEPROM FMMU data */
  ec_eepromFMMUt   RobotStateEtherCATHardwareInterface::ec_FMMU;

  char RobotStateEtherCATHardwareInterface::IOmap[4096];
  /** Main slave data array.
   *  Each slave found on the network gets its own record.
   *  ec_slave[0] is reserved for the master. Structure gets filled
   *  in by the configuration function ec_config().
   */
  ec_slavet               RobotStateEtherCATHardwareInterface::ec_slave[EC_MAXSLAVE];
  /** number of slaves found on the network */
  int                     RobotStateEtherCATHardwareInterface::ec_slavecount;
  /** slave group structure */
  ec_groupt               RobotStateEtherCATHardwareInterface::ec_group[EC_MAXGROUP];

  /** Global variable TRUE if error available in error stack */
  boolean                 RobotStateEtherCATHardwareInterface::EcatError = FALSE;

  int64                   RobotStateEtherCATHardwareInterface::ec_DCtime;

  ecx_portt               RobotStateEtherCATHardwareInterface::ecx_port;
  ecx_redportt            RobotStateEtherCATHardwareInterface::ecx_redport;


  ecx_contextt  RobotStateEtherCATHardwareInterface::ecx_context = {
      &ecx_port,          // .port          =
      &ec_slave[0],       // .slavelist     =
      &ec_slavecount,     // .slavecount    =
      EC_MAXSLAVE,        // .maxslave      =
      &ec_group[0],       // .grouplist     =
      EC_MAXGROUP,        // .maxgroup      =
      &ec_esibuf[0],      // .esibuf        =
      &ec_esimap[0],      // .esimap        =
      0,                  // .esislave      =
      &ec_elist,          // .elist         =
      &ec_idxstack,       // .idxstack      =
      &EcatError,         // .ecaterror     =
      0,                  // .DCtO          =
      0,                  // .DCl           =
      &ec_DCtime,         // .DCtime        =
      &ec_SMcommtype[0],  // .SMcommtype    =
      &ec_PDOassign[0],   // .PDOassign     =
      &ec_PDOdesc[0],     // .PDOdesc       =
      &ec_SM,             // .eepSM         =
      &ec_FMMU,           // .eepFMMU       =
      NULL//,               // .FOEhook()
//      NULL                // .EOEhook()
  };

RobotStateEtherCATHardwareInterface::RobotStateEtherCATHardwareInterface()
{
  ANYdriveTPDOType_ = PDOTYPE::TPDO_A;
  ANYdriveRPDOType_ = PDOTYPE::RPDO_A;
  GoldenTwitterTPDOType_ = PDOTYPE::TPDO_F;
  GoldenTwitterRPDOType_ = PDOTYPE::RPDO_F;
  motor_slave_num_ = 0;
}

RobotStateEtherCATHardwareInterface::~RobotStateEtherCATHardwareInterface()
{
  ros::Duration delay(0.1);
  double start = ros::Time::now().toSec();
  while (!DeInitSlaves(slaves_type_)) {

      delay.sleep();
      double wait_time = ros::Time::now().toSec() - start;
      if(wait_time > 5.0)
        break;

    }
  ec_slave[0].state = EC_STATE_INIT;
  /* request INIT state for all slaves */
  ecx_writestate(&ecx_context, 0);
  ecx_close(&ecx_context);
  ROS_WARN("EtherCAT HardWare Interface Shutdown");
}

void RobotStateEtherCATHardwareInterface::shutdown()
{
  ros::Duration delay(0.1);
  double start = ros::Time::now().toSec();
  while (!DeInitSlaves(slaves_type_)) {

      delay.sleep();
      double wait_time = ros::Time::now().toSec() - start;
      if(wait_time > 5.0)
        break;
    }
  ec_slave[0].state = EC_STATE_INIT;
  /* request INIT state for all slaves */
  ecx_writestate(&ecx_context, 0);
  ecx_close(&ecx_context);
  ROS_WARN("EtherCAT HardWare Interface Shutdown");
}

bool RobotStateEtherCATHardwareInterface::loadParameters(ros::NodeHandle& nh)
{
  if(!nh.getParam("/motor_frictions/mu", motor_friction_mu))
    {
      ROS_ERROR("Can not load motor friction 'mu' parameters");
      return false;
    }
  if(!nh.getParam("/motor_frictions/bias", motor_friction_bias))
    {
      ROS_ERROR("Can not load motor friction 'bias' parameters");
      return false;
    }
  if(!nh.getParam("/motor_zeros/zero_offsets", motor_zero_offsets))
    {
      ROS_ERROR("Can not load motor zero 'zero_offsets' parameters");
      return false;
    }
  if(!nh.getParam("/motor_frictions/proportion", motor_friction_proportion))
    {
      ROS_ERROR("Can not load motor friction 'proportion' parameters");
      return false;
    }
  if(!nh.getParam("/motor_limits/max", motor_max_limits))
    {
      ROS_ERROR("Can not load motor 'motor_limits_max' parameters");
      return false;
    }
  if(!nh.getParam("/motor_limits/min", motor_min_limits))
    {
      ROS_ERROR("Can not load motor '/motor_limits/min' parameters");
      return false;
    }
  if(!nh.getParam("/motor_unused", motor_unused))
    {
      ROS_ERROR("Can not load motor '/motor_unused' parameters");
      return false;
    }

  if(!nh.getParam("/motor_direction", motor_directions))
    {
      ROS_ERROR("Can not load motor '/motor_direction' parameters");
      return false;
    }

  for(auto out:motor_friction_mu)
    std::cout<<out<<std::endl;
  std::string motor_friction_dir_str = ros::package::getPath("ros_ethercat_driver") + "/config";
  friction_of_pos.resize(motor_friction_mu.size());
  for(int i = 0;i<motor_friction_mu.size();i++)
    {
      std::ifstream data(motor_friction_dir_str + "/friction_of_pos_motor_"+std::to_string(i+1)+".txt");
      friction_of_pos[i].resize(360);
      for(int j = 0;j<360;j++)
        {
          data >> friction_of_pos[i][j];
//          friction_of_pos[i][j] *= motor_friction_proportion[i];
          if(friction_of_pos[i][j]<0)
            friction_of_pos[i][j]=0;
//          std::cout<<friction_of_pos[i][j]<<std::endl;
        }
      motor_disabled.push_back((bool)motor_unused[i]);
    }
  return true;
}

bool RobotStateEtherCATHardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{

//  sdf::ElementPtr sdf = parent_model->GetSDF();
////  sdf::ElementPtr sensor = sdf->GetElement("sensor");
//  std::cout<<"=========================================="<<sdf->GetElement("robotParam")<<std::endl;
//  contact_sensor_ptr.reset(new gazebo::sensors::ContactSensor);
//  contact_sensor_ptr->Load("ground_plane");
//  contact_sensor_ptr->SetActive(true);

  ROS_INFO("Initializing RobotStateEtherCATHardwareInterface");
  node_handle_ = root_nh;

  if(!loadParameters(node_handle_))
    {
      ROS_ERROR("Failed to Load parameters");
      return false;
    }
//  imu_data_.frame_id = "imu";
//  imu_interface_.registerHandle(hardware_interface::ImuSensorHandle(imu_data_));
  //! WSHY: initial data of robot state
  robot_state_data_.name = "base_controller";
  robot_state_data_.position = position;
  robot_state_data_.orientation = orinetation;
  robot_state_data_.linear_velocity = linear_vel;
  robot_state_data_.angular_velocity = angular_vel;
  robot_state_data_.joint_position_read = pos_read;
  robot_state_data_.joint_position_write = pos_write;
  robot_state_data_.joint_velocity_read = vel_read;
  robot_state_data_.joint_velocity_write = vel_write;
  robot_state_data_.joint_effort_read = eff_read;
  robot_state_data_.joint_effort_write = eff_write;
  robot_state_data_.foot_contact = foot_contact;
  //! WSHY: registerhandle pass the data point to the hardwareResourseManager and then
  //! the read() method update data which the pointer points to or write() the
  //! updated commmand
  robot_state_interface_.registerHandle(hardware_interface::RobotStateHandle(robot_state_data_));

  // getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
  // parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
//  const ros::NodeHandle joint_limit_nh(root_nh);

  // Add data from rosparam
  std::string net_card;
  if(!root_nh.getParam("ethercat_card_name", net_card))
    {
      ROS_ERROR("Can't find name of EtherCAT Card");
      return false;
    }
  ifname = (char *)net_card.c_str();

  std::string slave_type, pdo_type;
  if(!root_nh.getParam("slave_type", slave_type))
    {
      ROS_ERROR("Can't decide the Slave Type");
      return false;
    }
  if(!root_nh.getParam("pdo_type", pdo_type))
    {
      ROS_ERROR("Can't decide the PDO type");
      return false;
    }

//  if(!root_nh.getParam("use_jc06_junction_slave", pdo_type))
//    {
//      ROS_ERROR("Can't decide the 'use_jc06_junction_slave'.");
//      return false;
//    }

  std::string urdf_string;
  if(!root_nh.getParam("/robot_description", urdf_string))
    {
      ROS_ERROR("Failed to load urdf from robot_descriptions");
      return false;
    }
  transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);
  urdf::Model urdf_model;
  urdf_model.initString(urdf_string);
  const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

  std::string param_name = "/base_balance_controller/joints";
  if(!root_nh.getParam(param_name, joint_names_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << root_nh.getNamespace() << ").");
      return false;
    }
  n_dof_ = joint_names_.size();
//  n_dof_ = 1; //! WSHY: for single test
  // Resize vectors to our DOF
  joint_names_.resize(n_dof_);
  joint_types_.resize(n_dof_);
  joint_lower_limits_.resize(n_dof_);
  joint_upper_limits_.resize(n_dof_);
  joint_effort_limits_.resize(n_dof_);
  joint_control_methods_.resize(n_dof_);
  last_joint_control_methods_.resize(n_dof_);
  pid_controllers_.resize(n_dof_);
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_effort_command_.resize(n_dof_);
  joint_position_command_.resize(n_dof_);
  joint_velocity_command_.resize(n_dof_);

  status_word_.resize(n_dof_);
  // Initialize values
  const ros::NodeHandle joint_limit_nh(root_nh);

  hardware_interface::JointHandle joint_handle;
  for(unsigned int j = 0;j<n_dof_;j++)
    {

      // Check that this transmission has one joint
      if(transmissions_[j].joints_.size() == 0)
      {
        ROS_WARN_STREAM_NAMED("ros_ethercat_hardware_interface","Transmission " << transmissions_[j].name_
          << " has no associated joints.");
        continue;
      }
      else if(transmissions_[j].joints_.size() > 1)
      {
        ROS_WARN_STREAM_NAMED("ros_ethercat_hardware_interface","Transmission " << transmissions_[j].name_
          << " has more than one joint. Currently the default robot hardware simulation "
          << " interface only supports one.");
        continue;
      }

      std::vector<std::string> joint_interfaces = transmissions_[j].joints_[0].hardware_interfaces_;
      if (joint_interfaces.empty() &&
          !(transmissions_[j].actuators_.empty()) &&
          !(transmissions_[j].actuators_[0].hardware_interfaces_.empty()))
      {
        // TODO: Deprecate HW interface specification in actuators in ROS J
        joint_interfaces = transmissions_[j].actuators_[0].hardware_interfaces_;
        ROS_WARN_STREAM_NAMED("ros_ethercat_hardware_interface", "The <hardware_interface> element of tranmission " <<
          transmissions_[j].name_ << " should be nested inside the <joint> element, not <actuator>. " <<
          "The transmission will be properly loaded, but please update " <<
          "your robot model to remain compatible with future versions of the plugin.");
      }
      if (joint_interfaces.empty())
      {
        ROS_WARN_STREAM_NAMED("ros_ethercat_hardware_interface", "Joint " << transmissions_[j].joints_[0].name_ <<
          " of transmission " << transmissions_[j].name_ << " does not specify any hardware interface. " <<
          "Not adding it to the robot hardware simulation.");
        continue;
      }
      else if (joint_interfaces.size() > 1)
      {
        ROS_WARN_STREAM_NAMED("ros_ethercat_hardware_interface", "Joint " << transmissions_[j].joints_[0].name_ <<
          " of transmission " << transmissions_[j].name_ << " specifies multiple hardware interfaces. " <<
          "Currently the default robot hardware simulation interface only supports one. Using the first entry");
        //continue;
      }

      const std::string& hardware_interface = joint_interfaces.front();

      joint_position_[j] = 1.0;
      joint_velocity_[j] = 0.0;
      joint_effort_[j] = 1.0;  // N/m for continuous joints
      joint_effort_command_[j] = 0.0;
      joint_position_command_[j] = 0.0;
      joint_velocity_command_[j] = 0.0;

      // Create joint state interface for all joints
      js_interface_.registerHandle(hardware_interface::JointStateHandle(
          joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

//       if(hardware_interface == "EffortJointInterface" || hardware_interface == "hardware_interface/EffortJointInterface")
//         {
           // Create effort joint interface
           joint_control_methods_[j] = EFFORT;
           joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                          &joint_effort_command_[j]);
           //      hardware_interface::RobotStateHandle(js_interface_.getHandle(joint_names_[j],
           //                                                                   &joint_effort_command_[j]));
           robot_state_interface_.joint_effort_interfaces_.registerHandle(joint_handle);
           ej_interface_.registerHandle(joint_handle);
//         }
//       else if(hardware_interface == "VelocityJointInterface" || hardware_interface == "hardware_interface/VelocityJointInterface")
//         {
           // Create velocity joint interface
           joint_control_methods_[j] = VELOCITY;
           joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                          &joint_velocity_command_[j]);
           vj_interface_.registerHandle(joint_handle);
//         }
//       else if(hardware_interface == "PositionJointInterface" || hardware_interface == "hardware_interface/PositionJointInterface")

//         {
           // Create position joint interface
           joint_control_methods_[j] = POSITION;
           joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                          &joint_position_command_[j]);
           pj_interface_.registerHandle(joint_handle);
//         }
//       else
//       {
//         ROS_FATAL_STREAM_NAMED("ros_ethercat_hardware_interface","No matching hardware interface found for '"
//           << hardware_interface << "' while loading interfaces for " << joint_names_[j] );
//         return false;
//       }
           joint_control_methods_[j] = FREEZE;
       const ros::NodeHandle joint_limit_nh(root_nh);
       registerJointLimits(joint_names_[j], joint_handle, joint_control_methods_[j],
                           joint_limit_nh, urdf_model_ptr,
                           &joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
                           &joint_effort_limits_[j]);
    }



  // Register interfaces
  //! WSHY: the controller pass the interface in it
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&vj_interface_);
//  registerInterface(&imu_interface_);
  registerInterface(&robot_state_interface_);

  // Initialize the emergency stop code.
  e_stop_active_ = false;
  last_e_stop_active_ = false;


  setPDOType(slave_type, pdo_type);

  feedbacks_.resize(n_dof_);
  commands_.resize(n_dof_);
//  anydriveTest(ifname);
  if(!EtherCATInit()){
     ROS_ERROR("Failed to initialize EtherCAT Conmunication !!!!!!!!!");
     return false;
  }

  createEtherCATCheckThread();
  //! WSHY: EtherCAT LWR/LDR Loop, update struct data for slave, then the read&write set
  //! the data in struct to robot_state_handle
//  EtherCATLoopThread_ = boost::thread(boost::bind(&RobotStateEtherCATHardwareInterface::EtherCATLoop, this));

  ros::TimerOptions timer_options(ros::Duration(0.002),
                                  boost::bind(&RobotStateEtherCATHardwareInterface::EtherCATLoop, this, _1),
                                  &EtherCATUpdateQueue_, false, false);
  EtherCATTimer_ = node_handle_.createTimer(timer_options);
  EtherCATTimerThread_ = boost::thread(boost::bind(&RobotStateEtherCATHardwareInterface::EtherCATTimerThread, this));
  EtherCATTimer_.start();

  ros::Duration delay(0.1);
  while (!InitSlaves(slaves_type_)) {

      delay.sleep();
    }
  ROS_INFO("Successfully Initialize EtherCAT Hardware Interface");
  return true;
}

bool RobotStateEtherCATHardwareInterface::DeInitSlaves(const std::vector<SlaveType>& slave_type)
{
 bool success = false;
 int disable_count = 0;
// ROS_INFO("disconnect %d slave",slaves_type_.size());
 for(int i=0;i<ec_slavecount;i++)
   {
     ROS_INFO("disconnect %d slave",i);
    switch (slave_type[i]) {
      case SlaveType::ANYDRIVE:
        {

          switch (ANYdriveTPDOType_) {
            case PDOTYPE::TPDO_A:
              {
                ANYDriveRPDOA* driver_command;
                ANYDriveTPDOA* driver_feedback;
  //              for(int i=0;i<ec_slavecount;i++)
  //                {
                ROS_INFO("disconnect %d slave",i);
                driver_feedback = (struct ANYDriveTPDOA*)ec_slave[i+1].inputs;
                driver_command = (struct ANYDriveRPDOA*)ec_slave[i+1].outputs;
                switch ((driver_feedback->state<<28)>>28) {
                  case ANYDriveFSMState::CONFIGURE:
                    driver_command->control_word = ANYDriveControlWord::CONFIGURE_TO_STANDBY;
                    break;
                  case ANYDriveFSMState::STANDBY:
                    driver_command->control_word = 0;
                    disable_count++;
                    std::cout<<"disable_count : "<<disable_count<<std::endl;
                    break;
                  case ANYDriveFSMState::MOTOR_OP:
                    driver_command->control_word = ANYDriveControlWord::MOTOR_OP_TO_STANDBY;
                    break;
                  case ANYDriveFSMState::CONTROL_OP:
                    driver_command->control_word = ANYDriveControlWord::CONTROL_OP_TO_STANDBY;
                    driver_command->mode_of_operation = ANYDriveModeOfOperation::FREEZE;
                    break;
                  }
                if(disable_count == motor_slave_num_)
                  {
                    success = true;
                    break;
                  }
                std::cout<<"motor slave count : "<<motor_slave_num_<<std::endl;

  //              }
                 success = false;
                 break;
              }
            default:
              {
                ANYDriveRPDOA* driver_command;
                ANYDriveTPDOA* driver_feedback;
  //              for(int i=0;i<ec_slavecount;i++)
  //                {
                    driver_feedback = (struct ANYDriveTPDOA*)ec_slave[i+1].inputs;
                    driver_command = (struct ANYDriveRPDOA*)ec_slave[i+1].outputs;
                    switch ((driver_feedback->state<<28)>>28) {
                      case ANYDriveFSMState::CONFIGURE:
                        driver_command->control_word = ANYDriveControlWord::CONFIGURE_TO_STANDBY;
                        break;
                      case ANYDriveFSMState::STANDBY:
                        driver_command->control_word = 0;
                        disable_count++;
                        break;
                      case ANYDriveFSMState::MOTOR_OP:
                        driver_command->control_word = ANYDriveControlWord::MOTOR_OP_TO_STANDBY;
                        break;
                      case ANYDriveFSMState::CONTROL_OP:
                        driver_command->control_word = ANYDriveControlWord::CONTROL_OP_TO_STANDBY;
                        driver_command->mode_of_operation = ANYDriveModeOfOperation::FREEZE;
                        break;
                      }
                    if(disable_count == motor_slave_num_)
                      {
                        success = true;
                        break;
                      }

  //              }
              success = false;
           break;
              }
            }
          break;
        }
      case SlaveType::GOLDENTWITTER:
        {
          switch (GoldenTwitterTPDOType_) {
            case PDOTYPE::TPDO_F:
              {
                GoldenTwitterRPDOF* driver_command;
                GoldenTwitterTPDOF* driver_feedback;
  //              for(int i=0;i<ec_slavecount;i++)
  //                {
                ROS_INFO("disconnect %d slave",i);
                driver_feedback = (struct GoldenTwitterTPDOF*)ec_slave[i+1].inputs;
                driver_command = (struct GoldenTwitterRPDOF*)ec_slave[i+1].outputs;
                switch ((driver_feedback->status_word & 0b01101111)) {
                  case GoldenTwitterState::NOT_READY_TO_SWITCH_ON:
                    driver_command->control_word = GoldenTwitterControlWord::SHUT_DOWN;
                    break;
                  case GoldenTwitterState::SWITCH_ON_DISABLED:
                    driver_command->control_word = GoldenTwitterControlWord::SHUT_DOWN;
                    break;
                  case GoldenTwitterState::READY_TO_SWITCH_ON:
                    driver_command->control_word = 0;
                    disable_count++;
                    std::cout<<"disable_count : "<<disable_count<<std::endl;
                    break;
                  case GoldenTwitterState::SWITCHED_ON:
                    driver_command->control_word = GoldenTwitterControlWord::SHUT_DOWN;
                    break;
                  case GoldenTwitterState::QUICK_STOP_ACTIVED:
                    driver_command->control_word = GoldenTwitterControlWord::DISABLE_VOLTAGE;
                    break;
//                  case GoldenTwitterState::FAULT:
//                    driver_command->control_word = GoldenTwitterControlWord::FAULT_RESET;
//                    break;
                  case GoldenTwitterState::OP_ENABLED:
                    driver_command->control_word = GoldenTwitterControlWord::SHUT_DOWN;
                    break;
                  }
                if(disable_count == motor_slave_num_)
                  {
                    success = true;
                    break;
                  }
                std::cout<<"motor slave count : "<<motor_slave_num_<<std::endl;

  //              }
                 success = false;
                 break;
              }
            }
          break;
        }
      case SlaveType::JUNCTION:
        {
          break;
        }
      }
  }
 if(!success)
   return false;
 return true;
}

bool RobotStateEtherCATHardwareInterface::setSlaveCW(int id, const std::string& name,
                                                       const std::string& control_word)
{
  SlaveType type = slaves_type_[id];
  if(type == SlaveType::ANYDRIVE)
    {
      //! WSHY: for control word
      uint16_t cw = 0;
      if(control_word == "Warm Reset")
        {
          ROS_INFO("%s", control_word.c_str());
          cw = ANYDriveControlWord::WARM_RESET;
        }
      else if(control_word == "Clear Errors to MotorOp")
        {
          ROS_INFO("%s", control_word.c_str());
          cw = ANYDriveControlWord::WARM_RESET;
        }
      else if(control_word == "Standby to Configure")
        {
          ROS_INFO("%s", control_word.c_str());
          cw = ANYDriveControlWord::STANDBY_TO_CONFIGURE;
        }
      else if(control_word == "Configure to Standby")
        {
          ROS_INFO("%s", control_word.c_str());
          cw = ANYDriveControlWord::CONFIGURE_TO_STANDBY;
        }
      else if(control_word == "MotorOp to Standby")
            {
              ROS_INFO("%s", control_word.c_str());
          cw = ANYDriveControlWord::MOTOR_OP_TO_STANDBY;
            }
      else if(control_word == "Standby to MotorPreOp")
            {
              ROS_INFO("%s", control_word.c_str());
          cw = ANYDriveControlWord::STANDBY_TO_MOTOR_PRE_OP;
            }
      else if(control_word == "ControlOp to MotorOp")
            {
              ROS_INFO("%s", control_word.c_str());
          cw = ANYDriveControlWord::CONTROL_OP_TO_MOTOR_OP;
            }
      else if(control_word == "MotorOp to ControlOp")
            {
              ROS_INFO("%s", control_word.c_str());
          cw = ANYDriveControlWord::MOTOR_OP_TO_CONTROL_OP;
            }
      else if(control_word == "Clear Errors to Standby")
            {
              ROS_INFO("%s", control_word.c_str());
          cw = ANYDriveControlWord::CLEAR_ERRORS_TO_STANDBY;
            }
      switch (ANYdriveRPDOType_) {
        case PDOTYPE::RPDO_A:
          {
            ANYDriveRPDOA* driver_command;
            driver_command = (struct ANYDriveRPDOA*)ec_slave[id+1].outputs;
            driver_command->control_word = cw;
            break;
          }
        default:
          {
            ANYDriveRPDOA* driver_command;
            driver_command = (struct ANYDriveRPDOA*)ec_slave[id+1].outputs;
            driver_command->control_word = cw;
            break;
          }
        }


//      //! WSHY: for mode of operation
//      if(mode_of_operation == "Position")
//        commands_[id]->mode_of_operation = ANYDriveModeOfOperation::JOINT_POSITION;
//      else if(mode_of_operation == "Velocity")
//        commands_[id]->mode_of_operation = ANYDriveModeOfOperation::JOINT_VELOCITY;
//      else if(mode_of_operation == "Torque")
//        commands_[id]->mode_of_operation = ANYDriveModeOfOperation::JOINT_TORQUE;
//      else if(mode_of_operation == "Freeze")
//        commands_[id]->mode_of_operation = ANYDriveModeOfOperation::FREEZE;
//      else if(mode_of_operation == "Disable")
//        commands_[id]->mode_of_operation = ANYDriveModeOfOperation::DISABLE;

      return true;

    }
  if(type == SlaveType::GOLDENTWITTER)
    {
      //! WSHY: for control word
      uint16_t cw = 0;
      if(control_word == "Shut Down")
        {
          ROS_INFO("%s", control_word.c_str());
          cw = GoldenTwitterControlWord::SHUT_DOWN;
        }
      else if (control_word == "Switch On") {
          ROS_INFO("%s", control_word.c_str());
          cw = GoldenTwitterControlWord::SWITCH_ON;
        }
      else if (control_word == "Switch On and Enable") {
          ROS_INFO("%s", control_word.c_str());
          cw = GoldenTwitterControlWord::SWITCH_ON_AND_ENABLE;
        }
      else if (control_word == "Quick Stop") {
          ROS_INFO("%s", control_word.c_str());
          cw = GoldenTwitterControlWord::QUICK_STOP;
        }
      else if (control_word == "Fault Reset") {
          ROS_INFO("%s", control_word.c_str());
          cw = GoldenTwitterControlWord::FAULT_RESET;
        }
      else if (control_word == "Disable Operation") {
          ROS_INFO("%s", control_word.c_str());
          cw = GoldenTwitterControlWord::DISABLE_OP;
        }
      else if (control_word == "Enable Operation") {
          ROS_INFO("%s", control_word.c_str());
          cw = GoldenTwitterControlWord::ENABLE_OP;
        }
      else if (control_word == "Disable Voltage") {
          ROS_INFO("%s", control_word.c_str());
          cw = GoldenTwitterControlWord::DISABLE_VOLTAGE;
        }
      switch (GoldenTwitterRPDOType_) {
        case PDOTYPE::RPDO_F:
          {
            GoldenTwitterRPDOF* driver_command;
            driver_command = (struct GoldenTwitterRPDOF*)ec_slave[id+1].outputs;
            driver_command->control_word = cw;
            break;
          }
        default:
          {
            GoldenTwitterRPDOF* driver_command;
            driver_command = (struct GoldenTwitterRPDOF*)ec_slave[id+1].outputs;
            driver_command->control_word = cw;
            break;
          }
        }
      return true;
    }
  return false;

}

bool RobotStateEtherCATHardwareInterface::setCommand(int id, const std::string& mode_of_operation, double command)
{
  SlaveType type = slaves_type_[id];
  int motor_count = id;
  for(int i = 0; i<id;i++)
    {
      if(slaves_type_[i] == SlaveType::JUNCTION)
        motor_count --;
    }


  if(type == SlaveType::ANYDRIVE)
    {
      switch (ANYdriveRPDOType_) {
        case PDOTYPE::RPDO_A:
          {
            ANYDriveRPDOA* driver_command;
            driver_command = (struct ANYDriveRPDOA*)ec_slave[id+1].outputs;
            //! WSHY: for mode of operation
            if(mode_of_operation == "Position")
              {
                driver_command->mode_of_operation = ANYDriveModeOfOperation::JOINT_POSITION;
                exchage8bytes_.doubledata = command;
                driver_command->desired_position = exchage8bytes_.int64data;
              }
            else if(mode_of_operation == "Velocity")
              {
                driver_command->mode_of_operation = ANYDriveModeOfOperation::JOINT_VELOCITY;
                exchage4bytes_.floatdata = (float)command;
                driver_command->desired_velocity = exchage4bytes_.int32data;
              }
            else if(mode_of_operation == "Torque")
              {
                driver_command->mode_of_operation = ANYDriveModeOfOperation::JOINT_TORQUE;
                exchage4bytes_.floatdata = (float)command;
                driver_command->desired_joint_torque = exchage4bytes_.int32data;
              }
            else if(mode_of_operation == "Freeze")
              driver_command->mode_of_operation = ANYDriveModeOfOperation::FREEZE;
            else if(mode_of_operation == "Disable")
              driver_command->mode_of_operation = ANYDriveModeOfOperation::DISABLE;
            break;
          }
        default:
          {
            ANYDriveRPDOA* driver_command;
            driver_command = (struct ANYDriveRPDOA*)ec_slave[id+1].outputs;
            //! WSHY: for mode of operation
            if(mode_of_operation == "Position")
              {
                driver_command->mode_of_operation = ANYDriveModeOfOperation::JOINT_POSITION;
                exchage8bytes_.doubledata = command;
                driver_command->desired_position = exchage8bytes_.int64data;
              }
            else if(mode_of_operation == "Velocity")
              {
                driver_command->mode_of_operation = ANYDriveModeOfOperation::JOINT_VELOCITY;
                exchage4bytes_.floatdata = (float)command;
                driver_command->desired_velocity = exchage4bytes_.int32data;
              }
            else if(mode_of_operation == "Torque")
              {
                driver_command->mode_of_operation = ANYDriveModeOfOperation::JOINT_TORQUE;
                exchage4bytes_.floatdata = (float)command;
                driver_command->desired_joint_torque = exchage4bytes_.int32data;
              }
            else if(mode_of_operation == "Freeze")
              driver_command->mode_of_operation = ANYDriveModeOfOperation::FREEZE;
            else if(mode_of_operation == "Disable")
              driver_command->mode_of_operation = ANYDriveModeOfOperation::DISABLE;
          }

        }
    }
  if(type == SlaveType::GOLDENTWITTER)
    {
      switch (GoldenTwitterRPDOType_) {
        case PDOTYPE::RPDO_F:
          {
            GoldenTwitterRPDOF* driver_command;
            driver_command = (struct GoldenTwitterRPDOF*)ec_slave[id+1].outputs;
            GoldenTwitterTPDOF* driver_feedback;
            driver_feedback = (struct GoldenTwitterTPDOF*)ec_slave[id +1].inputs;
            //! WSHY: for mode of operation
            if(mode_of_operation == "Position")
              {

                if(driver_feedback->status_word >> 12 == 1)
                  {
                    ROS_INFO("set point is not available");
                    driver_command->control_word = 47;
                  }else if (driver_feedback->status_word >>12 == 0) {
                    ROS_INFO("set point is available");
                    driver_command->control_word = 63;
                  }
//                if(driver_feedback->status_word >> 10 == 1)
//                  {
//                    ROS_INFO("Target is reached");
//                    driver_command->control_word = 47;
//                  }else if (driver_feedback->status_word >> 10 == 0) {
//                    ROS_INFO("Target is  not reached");
//                    driver_command->control_word = 63;
//                  }
                driver_command->mode_of_operation = GoldenTwitterModeOfOperation::PROFILE_POSITION;
                command = motor_directions[motor_count] * command + motor_zero_offsets[motor_count];
                int32 command_cnts = int32(TWITTER_GEAR_RATIO * TWITTER_ENCODER_RES * command/(2 * M_PI));
                ROS_INFO("send position in motor cnts is : %d",command_cnts);
                driver_command->max_torque = 1000;
//                if(driver_command->target_position == command_cnts)
//                  {
//                    driver_command->control_word = 47;
//                  }else {
//                    driver_command->control_word = 63;
//                  }
                driver_command->target_position = command_cnts;//exchage8bytes_.int64data;

              }
            else if(mode_of_operation == "Velocity")
              {
                driver_command->mode_of_operation = GoldenTwitterModeOfOperation::SYNC_VELOCITY;
//                exchage4bytes_.floatdata = (float)command;
//                int32 vel_cnts = TWITTER_GEAR_RATIO * TWITTER_ENCODER_RES * command * 2 * M_PI/60;
                // rad/s
                int32 vel_cnts = motor_directions[motor_count] * TWITTER_GEAR_RATIO * TWITTER_ENCODER_RES * command / (2 * M_PI);
                driver_command->max_torque = 1000;
                driver_command->target_velocity = vel_cnts;//exchage4bytes_.int32data;
                ROS_INFO("send velocity in motor cnts is : %d",vel_cnts);
              }
            else if(mode_of_operation == "Torque")
              {
                double velocity = 60.0 * driver_feedback->motor_velocity/(TWITTER_ENCODER_RES*TWITTER_GEAR_RATIO*2*M_PI);
                double position_in_rad = 2 * M_PI * driver_feedback->motor_position/(TWITTER_ENCODER_RES*TWITTER_GEAR_RATIO);
                int position_in_degree = int(180*position_in_rad/M_PI);//int(360*fmod(position_in_rad,2*M_PI)/(2*M_PI));
                position_in_degree = position_in_degree%360;
                driver_command->mode_of_operation = GoldenTwitterModeOfOperation::SYNC_TORQUE;
//                exchage4bytes_.floatdata = (float)command;

                int16 motor_torque = motor_directions[motor_count] * int16(command*TWITTER_CURRENT_TORQUE_RATIO);//TWITTER_GEAR_RATIO);
                std::cout<<"motor_count: "<<motor_count<<std::endl;
                ROS_INFO("Torque compensation in position %d is %f",position_in_degree, (1.0 + friction_of_pos[motor_count][position_in_degree])*getMotorFrictionCompensation(motor_count, velocity));

                if(motor_torque>=0) //id-1 to consider first junction slave
                  motor_torque = motor_torque + (1.0 + friction_of_pos[motor_count][position_in_degree])*getMotorFrictionCompensation(motor_count, velocity);
                if(motor_torque<0)
                  motor_torque = motor_torque + (1.0 + friction_of_pos[motor_count][position_in_degree])*getMotorFrictionCompensation(motor_count, velocity);
                driver_command->target_torque = motor_torque;//exchage4bytes_.int32data;
                driver_command->max_torque = 1000;
                ROS_INFO("send torque in motor : %d", motor_torque);
              }
//            else if(mode_of_operation == "Freeze")
//              driver_command->mode_of_operation = ANYDriveModeOfOperation::FREEZE;
//            else if(mode_of_operation == "Disable")
//              driver_command->mode_of_operation = ANYDriveModeOfOperation::DISABLE;
            break;
          }
        default:
          {
            GoldenTwitterRPDOF* driver_command;
            driver_command = (struct GoldenTwitterRPDOF*)ec_slave[id+1].outputs;
            //! WSHY: for mode of operation
            if(mode_of_operation == "Position")
              {
                driver_command->mode_of_operation = GoldenTwitterModeOfOperation::PROFILE_POSITION;
                exchage8bytes_.doubledata = command;
                driver_command->target_position = exchage8bytes_.int64data;
              }
            else if(mode_of_operation == "Velocity")
              {
                driver_command->mode_of_operation = GoldenTwitterModeOfOperation::SYNC_VELOCITY;
                exchage4bytes_.floatdata = (float)command;
                driver_command->target_velocity = exchage4bytes_.int32data;
              }
            else if(mode_of_operation == "Torque")
              {
                driver_command->mode_of_operation = GoldenTwitterModeOfOperation::SYNC_TORQUE;
                exchage4bytes_.floatdata = (float)command;
                driver_command->target_torque = exchage4bytes_.int32data;
              }
            break;
          }

        }
    }
        return true;
}

bool RobotStateEtherCATHardwareInterface::setModeOfOperation(int id, const std::string& mode_of_operation)
{
  SlaveType type = slaves_type_[id];
  if(type == SlaveType::ANYDRIVE)
    {
      switch (ANYdriveRPDOType_) {
        case PDOTYPE::RPDO_A:
          {
            ANYDriveRPDOA* driver_command;
            driver_command = (struct ANYDriveRPDOA*)ec_slave[id+1].outputs;
            //! WSHY: for mode of operation
            if(mode_of_operation == "Position")
              {
                driver_command->mode_of_operation = ANYDriveModeOfOperation::JOINT_POSITION;
              }
            else if(mode_of_operation == "Velocity")
              {
                driver_command->mode_of_operation = ANYDriveModeOfOperation::JOINT_VELOCITY;
              }
            else if(mode_of_operation == "Torque")
              {
                driver_command->mode_of_operation = ANYDriveModeOfOperation::JOINT_TORQUE;
              }
            else if(mode_of_operation == "Freeze")
              driver_command->mode_of_operation = ANYDriveModeOfOperation::FREEZE;
            else if(mode_of_operation == "Disable")
              driver_command->mode_of_operation = ANYDriveModeOfOperation::DISABLE;
            break;
          }
        default:
          {
            ANYDriveRPDOA* driver_command;
            driver_command = (struct ANYDriveRPDOA*)ec_slave[id+1].outputs;
            //! WSHY: for mode of operation
            if(mode_of_operation == "Position")
              {
                driver_command->mode_of_operation = ANYDriveModeOfOperation::JOINT_POSITION;
              }
            else if(mode_of_operation == "Velocity")
              {
                driver_command->mode_of_operation = ANYDriveModeOfOperation::JOINT_VELOCITY;
              }
            else if(mode_of_operation == "Torque")
              {
                driver_command->mode_of_operation = ANYDriveModeOfOperation::JOINT_TORQUE;
              }
            else if(mode_of_operation == "Freeze")
              driver_command->mode_of_operation = ANYDriveModeOfOperation::FREEZE;
            else if(mode_of_operation == "Disable")
              driver_command->mode_of_operation = ANYDriveModeOfOperation::DISABLE;
          }
          return true;
        }
    }
  if(type == SlaveType::GOLDENTWITTER)
    {
      switch (GoldenTwitterRPDOType_) {
        case PDOTYPE::RPDO_F:
          {
            GoldenTwitterRPDOF* driver_command;
            driver_command = (struct GoldenTwitterRPDOF*)ec_slave[id+1].outputs;
            //! WSHY: for mode of operation
            if(mode_of_operation == "Position")
              {
                driver_command->mode_of_operation = GoldenTwitterModeOfOperation::PROFILE_POSITION;
              }
            else if(mode_of_operation == "Velocity")
              {
                driver_command->mode_of_operation = GoldenTwitterModeOfOperation::SYNC_VELOCITY;
              }
            else if(mode_of_operation == "Torque")
              {
                driver_command->mode_of_operation = GoldenTwitterModeOfOperation::SYNC_TORQUE;
              }
            else if (mode_of_operation == "Cyclic Position") {
                driver_command->mode_of_operation = GoldenTwitterModeOfOperation::SYNC_POSITION;
              }
//            else if(mode_of_operation == "Freeze")
//              driver_command->mode_of_operation = GoldenTwitterModeOfOperation::FREEZE;
//            else if(mode_of_operation == "Disable")
//              driver_command->mode_of_operation = GoldenTwitterModeOfOperation::DISABLE;
            break;
          }
        default:
          {
            GoldenTwitterRPDOF* driver_command;
            driver_command = (struct GoldenTwitterRPDOF*)ec_slave[id+1].outputs;
            //! WSHY: for mode of operation
            if(mode_of_operation == "Position")
              {
                driver_command->mode_of_operation = GoldenTwitterModeOfOperation::PROFILE_POSITION;
              }
            else if(mode_of_operation == "Velocity")
              {
                driver_command->mode_of_operation = GoldenTwitterModeOfOperation::SYNC_VELOCITY;
              }
            else if(mode_of_operation == "Torque")
              {
                driver_command->mode_of_operation = GoldenTwitterModeOfOperation::SYNC_TORQUE;
              }
            break;
          }
          return true;
        }
    }
  return false;
}
bool RobotStateEtherCATHardwareInterface::setControlMethod(const std::string& method)
{

  if(method == "Joint Position")
    {
      for(int i = 0;i<ec_slavecount;i++)
        {
          joint_control_methods_[i] = POSITION;
          setModeOfOperation(i, "Position");
        }
      return true;
    }
  if(method == "Joint Cyclic Position")
    {
      for(int i = 0;i<ec_slavecount;i++)
        {
          joint_control_methods_[i] = POSITION;
          setModeOfOperation(i, "Cyclic Position");
        }
      return true;
    }else if (method == "Joint Velocity") {
      for(int i = 0;i<ec_slavecount;i++)
        {
          joint_control_methods_[i] = VELOCITY;
          setModeOfOperation(i, "Velocity");
        }
      return true;
    }else if (method == "Joint Effort" || method == "Balance") {
      for(int i = 0;i<ec_slavecount;i++)
        {
          joint_control_methods_[i] = EFFORT;
          setModeOfOperation(i, "Torque");
        }
      return true;
    }
  return false;
}

bool RobotStateEtherCATHardwareInterface::InitSlaves(const std::vector<SlaveType>& slave_type)
{
 bool success = false;
 int init_count = 0;
 for(int i=0;i<ec_slavecount;i++)
   {
    switch (slave_type[i]) {
      case SlaveType::ANYDRIVE:
        {
          switch (ANYdriveTPDOType_) {
            case PDOTYPE::TPDO_A:
              {
                ANYDriveRPDOA* driver_command;
                ANYDriveTPDOA* driver_feedback;
  //              for(int i=0;i<ec_slavecount;i++)
  //                {
                driver_feedback = (struct ANYDriveTPDOA*)ec_slave[i+1].inputs;
                driver_command = (struct ANYDriveRPDOA*)ec_slave[i+1].outputs;
                switch ((driver_feedback->state<<28)>>28) {
                  case ANYDriveFSMState::ERROR:
                    driver_command->control_word = ANYDriveControlWord::CLEAR_ERRORS_TO_STANDBY;
                    break;
                  case ANYDriveFSMState::CONFIGURE:
                    driver_command->control_word = ANYDriveControlWord::CONFIGURE_TO_STANDBY;
                    break;
                  case ANYDriveFSMState::STANDBY:
                    driver_command->control_word = ANYDriveControlWord::STANDBY_TO_MOTOR_PRE_OP;
                    break;
                  case ANYDriveFSMState::MOTOR_OP:
                    driver_command->control_word = ANYDriveControlWord::MOTOR_OP_TO_CONTROL_OP;
                    break;
                  case ANYDriveFSMState::CONTROL_OP:
                    ROS_INFO("Switch to Control_OP");
                    driver_command->control_word = 0;
                    init_count++;
                    switch (joint_control_methods_[i]) {
                      case EFFORT:
                        driver_command->mode_of_operation = ANYDriveModeOfOperation::JOINT_TORQUE;
                        ROS_INFO("Switch to Joint Torque Mode");
                        break;
                      case VELOCITY:
                        driver_command->mode_of_operation = ANYDriveModeOfOperation::JOINT_VELOCITY;
                        ROS_INFO("Switch to Joint Velocity Mode");
                        break;
                      case POSITION:
                        driver_command->mode_of_operation = ANYDriveModeOfOperation::JOINT_POSITION;
                        ROS_INFO("Switch to Joint Position Mode ");
                        break;
                      case FREEZE:
                        driver_command->mode_of_operation =ANYDriveModeOfOperation::FREEZE;
                        ROS_INFO("Switch to Joint Position Mode ");
                        break;
                      }
                    ROS_INFO("Switch to Control_OP");
                    if(init_count==motor_slave_num_)
                      {
                        success = true;
                      }else{
                        success = false;
                      }
                  }

  //                }
  //              return false;

                  break;
              }
            default:
              {
                ANYDriveRPDOA* driver_command;
                ANYDriveTPDOA* driver_feedback;
  //              for(int i=0;i<ec_slavecount;i++)
  //                {
                    driver_feedback = (struct ANYDriveTPDOA*)ec_slave[i+1].inputs;
                    driver_command = (struct ANYDriveRPDOA*)ec_slave[i+1].outputs;
                    switch ((driver_feedback->state<<28)>>28) {
                      case ANYDriveFSMState::ERROR:
                        driver_command->control_word = ANYDriveControlWord::CLEAR_ERRORS_TO_STANDBY;
                        break;
                      case ANYDriveFSMState::CONFIGURE:
                        driver_command->control_word = ANYDriveControlWord::CONFIGURE_TO_STANDBY;
                        break;
                      case ANYDriveFSMState::STANDBY:
                        driver_command->control_word = ANYDriveControlWord::STANDBY_TO_MOTOR_PRE_OP;
                        break;
                      case ANYDriveFSMState::MOTOR_OP:
                        driver_command->control_word = ANYDriveControlWord::MOTOR_OP_TO_CONTROL_OP;
                        break;
                      case ANYDriveFSMState::CONTROL_OP:
                        ROS_INFO("Switch to Control_OP");
                        driver_command->control_word = 0;
                        init_count++;
                        switch (joint_control_methods_[i]) {
                          case EFFORT:
                            driver_command->mode_of_operation = ANYDriveModeOfOperation::JOINT_TORQUE;
                            ROS_INFO("Switch to Joint Torque Mode");
                            break;
                          case VELOCITY:
                            driver_command->mode_of_operation = ANYDriveModeOfOperation::JOINT_VELOCITY;
                            ROS_INFO("Switch to Joint Velocity Mode");
                            break;
                          case POSITION:
                            driver_command->mode_of_operation = ANYDriveModeOfOperation::JOINT_POSITION;
                            ROS_INFO("Switch to Joint Position Mode ");
                            break;

                          }
                        ROS_INFO("Switch to Control_OP");
                        if(init_count==motor_slave_num_)
                          {
                            success = true;
                          }else{
                            success = false;
                          }
                      }

  //                }
  //              return false;
//               success = false;
               break;
              }

            }

          break;
        }
      case SlaveType::GOLDENTWITTER:
        {
          switch (GoldenTwitterTPDOType_) {
            case PDOTYPE::TPDO_F:
              {
                GoldenTwitterRPDOF* driver_command;
                GoldenTwitterTPDOF* driver_feedback;
  //              for(int i=0;i<ec_slavecount;i++)
  //                {
                driver_feedback = (struct GoldenTwitterTPDOF*)ec_slave[i+1].inputs;
                driver_command = (struct GoldenTwitterRPDOF*)ec_slave[i+1].outputs;
                switch ((driver_feedback->status_word & 0b01101111)) {
                  case GoldenTwitterState::NOT_READY_TO_SWITCH_ON:
                    driver_command->control_word = GoldenTwitterControlWord::SHUT_DOWN;
                    break;
                  case GoldenTwitterState::SWITCH_ON_DISABLED:
                    driver_command->control_word = GoldenTwitterControlWord::SHUT_DOWN;
                    break;
                  case GoldenTwitterState::READY_TO_SWITCH_ON:
                    driver_command->control_word = GoldenTwitterControlWord::SWITCH_ON;
                    break;
                  case GoldenTwitterState::SWITCHED_ON:
                    driver_command->control_word = GoldenTwitterControlWord::ENABLE_OP;
                    break;
                  case GoldenTwitterState::QUICK_STOP_ACTIVED:
                    driver_command->control_word = GoldenTwitterControlWord::DISABLE_VOLTAGE;
                    break;
                  case GoldenTwitterState::FAULT:
                    driver_command->control_word = GoldenTwitterControlWord::FAULT_RESET;
                    break;
                  case GoldenTwitterState::OP_ENABLED:
                    ROS_INFO("Switch to Control_OP");
                    init_count++;
                    switch (joint_control_methods_[i]) {
                      case POSITION:
                        driver_command->mode_of_operation = GoldenTwitterModeOfOperation::PROFILE_POSITION;
                        ROS_INFO("Switch to Joint Torque Mode");
                        break;
                      case VELOCITY:
                        driver_command->mode_of_operation = GoldenTwitterModeOfOperation::SYNC_VELOCITY;
                        ROS_INFO("Switch to Joint Velocity Mode");
                        break;
                      case EFFORT:
                        driver_command->mode_of_operation = GoldenTwitterModeOfOperation::SYNC_TORQUE;
                        ROS_INFO("Switch to Joint Position Mode ");
                        break;
//                      case FREEZE:
//                        driver_command->mode_of_operation =ANYDriveModeOfOperation::FREEZE;
//                        ROS_INFO("Switch to Joint Position Mode ");
//                        break;
                      }
                    ROS_INFO("Switch to Control_OP");
                    if(init_count==motor_slave_num_)
                      {
                        success = true;
                      }else{
                        success = false;
                      }
                  }

  //                }
  //              return false;

                  break;
              }
            }

          break;
        }
      case SlaveType::JUNCTION:
        {
          break;
        }
      }
  }
  if(!success)
    return false;
  return true;
}

bool RobotStateEtherCATHardwareInterface::setPDOType(const std::string& name, const std::string& type)
{
  if(name == "ANYdrive")
    {
      if(type == "PDOA")
        {
          ANYdriveTPDOType_ = PDOTYPE::TPDO_A;
          ANYdriveRPDOType_ = PDOTYPE::RPDO_A;
        }
      if(type == "PDOB")
        {
          ANYdriveTPDOType_ = PDOTYPE::TPDO_B;
          ANYdriveRPDOType_ = PDOTYPE::RPDO_B;
        }
      if(type == "PDOC")
        {
          ANYdriveTPDOType_ = PDOTYPE::TPDO_F;
          ANYdriveRPDOType_ = PDOTYPE::RPDO_C;
        }
      if(type == "PDOD")
        {
          ANYdriveTPDOType_ = PDOTYPE::TPDO_D;
          ANYdriveRPDOType_ = PDOTYPE::RPDO_D;
        }
    }
  if(name == "Twitter")
    {
      if(type == "PDOA")
        {
          GoldenTwitterTPDOType_ = PDOTYPE::TPDO_F;
          GoldenTwitterRPDOType_ = PDOTYPE::RPDO_F;
        }
      if(type == "PDOB")
        {
          GoldenTwitterTPDOType_ = PDOTYPE::TPDO_B;
          GoldenTwitterRPDOType_ = PDOTYPE::RPDO_B;
        }
      if(type == "PDOC")
        {
          GoldenTwitterTPDOType_ = PDOTYPE::TPDO_F;
          GoldenTwitterRPDOType_ = PDOTYPE::RPDO_C;
        }
      if(type == "PDOD")
        {
          GoldenTwitterTPDOType_ = PDOTYPE::TPDO_D;
          GoldenTwitterRPDOType_ = PDOTYPE::RPDO_D;
        }
    }
}

bool RobotStateEtherCATHardwareInterface::writeSDO(uint16 id, uint16 index,
                                                   uint8 sub_index, void *value, int size)
{
  ROS_INFO("In SDO Write");
  std::cout<<"value to write :"<<value<<std::endl;
  uint32 data;
  memcpy(&data, value, size);
  std::cout<<"data to write :"<<data<<std::endl;
  ecx_SDOwrite(&ecx_context, id, index, sub_index, FALSE, size, &data, EC_TIMEOUTRXM);
  return true;

}
bool RobotStateEtherCATHardwareInterface::readSDO(uint16 id, uint16 index, uint8 sub_index, char* result)
{
  char usdo[128];
//  char sdo_value[1024] = " ";
  int l = sizeof(usdo) - 1;
  ecx_SDOread(&ecx_context, id, index, sub_index, FALSE, &l, &usdo, EC_TIMEOUTRXM);
  strcpy(result, usdo);
//  return sdo_value;
  return true;
}
bool RobotStateEtherCATHardwareInterface::EtherCATInit()
{
//  anydriveTest(ifname);
  ROS_INFO("Starting EtherCAT Init");
  currentgroup = 0;
  /* initialise SOEM, bind socket to ifname */
  if (ecx_init(&ecx_context,ifname))
  {
     printf("ec_init on %s succeeded.\n",ifname);
     /* find and auto-config slaves */


      if ( ecx_config_init(&ecx_context, FALSE) > 0 )
     {
        printf("%d slaves found and configured.\n",ec_slavecount);
        feedbacks_.resize(ec_slavecount);
        commands_.resize(ec_slavecount);
//         slaves_type_.resize(ec_slavecount);
        motor_slave_num_ = 0;
        slaves_type_.clear();


        char usdo[128];
        for (int i=1; i<=ec_slavecount; i++) {
            bool hasCA = ec_slave[i].CoEdetails & ECT_COEDET_SDOCA ? true : false;
            printf("Slave %d has CA? %s\n", i, ec_slave[i].CoEdetails & ECT_COEDET_SDOCA ? "true":"false" );

            char sdo_name[128] = " ";
            if(hasCA)
              {

                int l = sizeof(usdo) - 1;
                readSDO(i,0x1008,0,sdo_name);
//                ecx_SDOread(&ecx_context, i, 0x1008, 0, FALSE, &l, &usdo, EC_TIMEOUTRXM);
//                strcpy(sdo_name, usdo);

                printf("Read Device Name From SDO of %d is :%s", i, sdo_name);
              }
            int32 ob2;int os;
            os=sizeof(ob2);
            if(strcmp(ec_slave[i].name, "ANYdrive") == 0)
            {
               ROS_WARN("Configue ANYdrive RPDO");
               motor_slave_num_++;
               ob2 = ANYdriveRPDOType_;//PDOTYPE::RPDO_A;// 0x16030001;
               ecx_SDOwrite(&ecx_context, i, 0x1c12, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);
               slaves_type_.push_back(SlaveType::ANYDRIVE);
            }else if (strcmp(sdo_name, "Twitter") == 0) {
               ROS_WARN("Configue GoldenTwitter RPDO");
               strcpy(ec_slave[i].name, sdo_name);
               motor_slave_num_++;
               ob2 = GoldenTwitterRPDOType_;//PDOTYPE::RPDO_A;// 0x16030001;
               slaves_type_.push_back(SlaveType::GOLDENTWITTER);
               uint32 u32= 5000000;
               //! WSHY: set profile acc and deacc for PP mode
               writeSDO(i, 0x6081, 0, &u32, sizeof (u32));
               u32= 10000000;
               writeSDO(i, 0x6083, 0, &u32, sizeof (u32));
               writeSDO(i, 0x6084, 0, &u32, sizeof (u32));
               int16 i16 = 8;
               int8 i8 = 2;
               //! WSHY: set extrapolation timeout for cyclic mode
               writeSDO(i, 0x60C2, 1, &i8, sizeof (i8));
               writeSDO(i, 0x2F75, 0, &i16, sizeof (i16));

               ecx_SDOwrite(&ecx_context, i, 0x1c12, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);
            } else{
              ROS_WARN("Add A Junction Slave");
              slaves_type_.push_back(SlaveType::JUNCTION);
              continue;
            }

//            ecx_SDOwrite(&ecx_context, i, 0x1c12, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);


//            int32 rpdo = 0x1605;
//            ecx_SDOwrite(&ecx_context, i, 0x1c12, 1, FALSE, sizeof (rpdo), &rpdo, EC_TIMEOUTRXM);
//            rpdo = 0x16050002;
//            ecx_SDOwrite(&ecx_context, i, 0x1c12, 2, TRUE, sizeof (rpdo), &rpdo, EC_TIMEOUTRXM);
//            ob2 = 2;
//            ecx_SDOwrite(&ecx_context, i, 0x1c12, 0, FALSE, os, &ob2, EC_TIMEOUTRXM);
            os=sizeof(ob2); ob2 = ANYdriveTPDOType_;//PDOTYPE::TPDO_A;//0x1a030001;
             if(strcmp(ec_slave[i].name, "ANYdrive") == 0)
             {
                ROS_WARN("Configue ANYdrive TPDO");
                ob2 = ANYdriveTPDOType_;//PDOTYPE::TPDO_A;//0x1a030001;
                ecx_SDOwrite(&ecx_context, i, 0x1c13, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);
             }else if (strcmp(sdo_name, "Twitter") == 0) {
                ROS_WARN("Configue GoldenTwitter TPDO");
                ob2 = GoldenTwitterTPDOType_;//PDOTYPE::TPDO_A;//0x1a030001;
                //! WSHY: set 1c13:0 to 0 to disable sync manager, then set pdo assignment
                //! finally set 2 as the entrys of 1c13
                uint8 u8 = 0;
                ecx_SDOwrite(&ecx_context, i, 0x1c13, 0, FALSE, sizeof (u8), &u8, EC_TIMEOUTRXM);
                uint64 tpdo = 0x1A0F; // PDO of actual velocity 6069
                ecx_SDOwrite(&ecx_context, i, 0x1c13, 1, FALSE, sizeof (tpdo), &tpdo, EC_TIMEOUTRXM);
                tpdo = 0x1A02; // RPDO_C
                ecx_SDOwrite(&ecx_context, i, 0x1c13, 2, FALSE, sizeof (tpdo), &tpdo, EC_TIMEOUTRXM);
                u8 = 2;
                ecx_SDOwrite(&ecx_context, i, 0x1c13, 0, FALSE, sizeof (u8), &u8, EC_TIMEOUTRXM);

             }else{
                 continue;
               }
//            ecx_SDOwrite(&ecx_context, i, 0x1c13, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);
            //            ROS_WARN("psize is %d\n", sizeof (tpdo));
        }

        for (int i=1; i<=ec_slavecount; i++) {
            //! WSHY: the ANYDrive is not support LRW(Logical READ&WRITE) but, it can't read
            //! by the upload slave info, so we manually assigned the value to Block LRW, before
            //! ec_config_map set it to ec_context.
            ROS_INFO("EC group of %d is group %d", i, ec_slave[i].group);
            if(strcmp(ec_slave[i].name, "ANYdrive") == 0)
              {
                ec_slave[i].blockLRW = 1;
                ROS_INFO("Change LDR for ANYdrive");
              }
        }
        //! WSHY: map the all input and output of all slaves to a static array
        ecx_config_map_group(&ecx_context, &IOmap, 0);

        ecx_configdc(&ecx_context);

        expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
        printf("Calculated workcounter %d\n", expectedWKC);

        printf("Slaves mapped, state to SAFE_OP.\n");
        /* wait for all slaves to reach SAFE_OP state */
        ecx_statecheck(&ecx_context,0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

        printf("Attach data struct to slave inputs/outputs.\n");
        /* send one valid process data to make outputs in slaves happy*/
        ecx_send_processdata(&ecx_context);
        ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);

        for (int i=1; i<=ec_slavecount; i++) {
            // show slave info
            printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n Address: %d\n",
            i, ec_slave[i].name, ec_slave[i].Obits, ec_slave[i].Ibits,
            ec_slave[i].state, ec_slave[i].pdelay, ec_slave[i].hasdc, ec_slave[i].configadr);
            ec_slave[i].state = EC_STATE_OPERATIONAL;
            //! WSHY: assigned the inputs and outputs pointer to out read and write data struct
//            feedbacks_[i-1] = (struct ANYDriveTPDOA *)(ec_slave[i].inputs);
//            commands_[i-1] = (struct ANYDriveRPDOA *)(ec_slave[i].outputs);
//            all_type_feedbacks[i-1] = ec_slave[i].inputs;
            /* request OP state for all slaves */
            ecx_writestate(&ecx_context, i);
        }


//         ec_readstate();
        int chk = 40;
        /* wait for all slaves to reach OP state */
        do
        {
            ecx_send_processdata(&ecx_context);
            ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
            ecx_statecheck(&ecx_context, 0, EC_STATE_OPERATIONAL, 50000);
        }
        while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
        printf("Slaves set to 'EC_STATE_OPERATIONAL'.\n");
        if (ec_slave[0].state == EC_STATE_OPERATIONAL )
        {
            ROS_INFO("Success!!!");
            return true;
        }


      }
      else
      {
          printf("No slaves found!\n");
          return false;
      }
      ecx_close(&ecx_context);
    }
  else
  {
      printf("No socket connection on %s\nExcecute as root\n",ifname);
      return false;
  }
  ecx_close(&ecx_context);
  return false;

}

void RobotStateEtherCATHardwareInterface::EtherCATCheck()
{
  int slave;
  ros::Rate rate(100);
  while(ros::ok())
  {
      if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
      {
          if (needlf)
          {
             needlf = FALSE;
             printf("\n");
          }
          /* one ore more slaves are not responding */
          ec_group[currentgroup].docheckstate = FALSE;
          ecx_readstate(&ecx_context);
          for (slave = 1; slave <= ec_slavecount; slave++)
          {
             if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
             {
                ec_group[currentgroup].docheckstate = TRUE;
                if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                {
                   printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                   ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                   ecx_writestate(&ecx_context, slave);
                }
                else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                {
                   printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                   ec_slave[slave].state = EC_STATE_OPERATIONAL;
                   ecx_writestate(&ecx_context, slave);
                }
                else if(ec_slave[slave].state > 0x00)//EC_STATE_NONE)
                {
                   if (ecx_reconfig_slave(&ecx_context, slave, EC_TIMEOUTMON))
                   {
                      ec_slave[slave].islost = FALSE;
                      printf("MESSAGE : slave %d reconfigured\n",slave);
                   }
                }
                else if(!ec_slave[slave].islost)
                {
                   /* re-check state */
                   ecx_statecheck(&ecx_context, slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                   if (ec_slave[slave].state == 0x00)//EC_STATE_NONE)
                   {
                      ec_slave[slave].islost = TRUE;
                      printf("ERROR : slave %d lost\n",slave);
                   }
                }
             }
             if (ec_slave[slave].islost)
             {
                if(ec_slave[slave].state == 0x00)//EC_STATE_NONE)
                {
                   if (ecx_recover_slave(&ecx_context, slave, EC_TIMEOUTMON))
                   {
                      ec_slave[slave].islost = FALSE;
                      printf("MESSAGE : slave %d recovered\n",slave);
                   }
                }
                else
                {
                   ec_slave[slave].islost = FALSE;
                   printf("MESSAGE : slave %d found\n",slave);
                }
             }
          }
          if(!ec_group[currentgroup].docheckstate)
             printf("OK : all slaves resumed OPERATIONAL.\n");
      }
      osal_usleep(10000);
  }
}
bool RobotStateEtherCATHardwareInterface::createEtherCATLoopThread()
{
  EtherCATLoopThread_ = boost::thread(boost::bind(&RobotStateEtherCATHardwareInterface::EtherCATLoop, this));
  return true;
}

bool RobotStateEtherCATHardwareInterface::createEtherCATCheckThread()
{
  EtherCATCheckThread_ = boost::thread(boost::bind(&RobotStateEtherCATHardwareInterface::EtherCATCheck, this));
  return true;
}

void RobotStateEtherCATHardwareInterface::EtherCATLoop()
{
  ROS_INFO("Starting EtherCAT update loop");
  inOP = true;
  if (ec_slave[0].state == EC_STATE_OPERATIONAL )
  {

      ros::Rate rate(500);
      while (ros::ok()) {
          ecx_send_processdata(&ecx_context);
          wkc = ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
//          ROS_INFO("Worker Count is : %d", wkc);
          rate.sleep();
        }
    }
  ecx_close(&ecx_context);

}

void RobotStateEtherCATHardwareInterface::EtherCATLoop(const ros::TimerEvent&)
{
//  ROS_INFO("Starting EtherCAT update loop");
  inOP = true;
  if (ec_slave[0].state == EC_STATE_OPERATIONAL )
  {

          ecx_send_processdata(&ecx_context);
          wkc = ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
//          ROS_INFO("Worker Count is : %d", wkc);
    }

}

void RobotStateEtherCATHardwareInterface::EtherCATTimerThread()
{
  static const double timeout = 0.001;
  while (node_handle_.ok()) {
      EtherCATUpdateQueue_.callAvailable(ros::WallDuration(timeout));
    }
}

void RobotStateEtherCATHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  //! WSHY: copy the date in struct uodated in EtherCAT Loop, for thread safe
//  std::vector<ANYDriveTPDOA> driver_feedbacks;
  boost::recursive_mutex::scoped_lock lock(r_mutex_);
  int junction_count = 0;
  for(int i=0;i<ec_slavecount;i++)
    {
      if(slaves_type_[i] == SlaveType::ANYDRIVE)
        {
          int j = i - junction_count;
          switch (ANYdriveTPDOType_) {
            case PDOTYPE::TPDO_A:
              {
                ANYDriveTPDOA driver_feedback;
                //        for(int j=0;j<ec_slavecount;j++)
                //          {
                driver_feedback = (*(struct ANYDriveTPDOA*)ec_slave[i+1].inputs);

                exchage8bytes_.int64data = driver_feedback.joint_position;
                joint_position_[j-junction_count] = exchage8bytes_.doubledata;
                robot_state_data_.joint_position_read[j-junction_count] = exchage8bytes_.doubledata;

                exchage4bytes_.int32data = driver_feedback.joint_velocity;
                joint_velocity_[j-junction_count] = exchage4bytes_.floatdata;
                robot_state_data_.joint_velocity_read[j-junction_count] = exchage4bytes_.floatdata;

                exchage4bytes_.int32data = driver_feedback.joint_torque;
                joint_effort_[j-junction_count] = exchage4bytes_.floatdata;
                robot_state_data_.joint_effort_read[j-junction_count] = exchage4bytes_.floatdata;

                //          }
                break;
              }
            default:
              {
                ANYDriveTPDOA driver_feedback;
                //        for(int j=0;j<ec_slavecount;j++)
                //          {
                driver_feedback = (*(struct ANYDriveTPDOA*)ec_slave[i+1].inputs);

                exchage8bytes_.int64data = driver_feedback.joint_position;
                joint_position_[j] = exchage8bytes_.doubledata;
                robot_state_data_.joint_position_read[j] = exchage8bytes_.doubledata;

                exchage4bytes_.int32data = driver_feedback.joint_velocity;
                joint_velocity_[j] = exchage4bytes_.floatdata;
                robot_state_data_.joint_velocity_read[j] = exchage4bytes_.floatdata;

                exchage4bytes_.int32data = driver_feedback.joint_torque;
                joint_effort_[j] = exchage4bytes_.floatdata;
                robot_state_data_.joint_effort_read[j] = exchage4bytes_.floatdata;

                //          }
                break;
              }
            }
          continue;
        }
      if(slaves_type_[i] == SlaveType::GOLDENTWITTER)
        {
//          ROS_INFO("Read from Twitter");
          int j = i - junction_count;
          switch (GoldenTwitterTPDOType_) {
            case PDOTYPE::TPDO_F:
              {
                GoldenTwitterTPDOF* driver_feedback;
                driver_feedback = (struct GoldenTwitterTPDOF*)ec_slave[i+1].inputs;
                status_word_[j] = driver_feedback->status_word;
                double position = 2 * M_PI * driver_feedback->motor_position/(TWITTER_ENCODER_RES*TWITTER_GEAR_RATIO);
                position = motor_directions[j]*(position - motor_zero_offsets[j]);
                // rpm on load
//                double velocity = 60.0 * driver_feedback->motor_velocity/(TWITTER_ENCODER_RES*TWITTER_GEAR_RATIO*2*M_PI);
                // rad/s on load
                double velocity = motor_directions[j]*(2*M_PI * driver_feedback->motor_velocity/(TWITTER_ENCODER_RES*TWITTER_GEAR_RATIO));
                // percent of max torque
                double effort = motor_directions[j]*(driver_feedback->motor_torque/TWITTER_CURRENT_TORQUE_RATIO);

//                double last_joint_position = joint_position_[j];
                joint_position_[j] = position;
                robot_state_data_.joint_position_read[j] = position;

//                float joint_velocity = (joint_position_[j] - last_joint_position)/period.toSec();
                joint_velocity_[j] = velocity;
                robot_state_data_.joint_velocity_read[j] = velocity;

                joint_effort_[j] = effort;
                robot_state_data_.joint_effort_read[j] = effort;
                break;
              }
            default:
              {
                GoldenTwitterTPDOF driver_feedback;
                //        for(int j=0;j<ec_slavecount;j++)
                //          {
                driver_feedback = (*(struct GoldenTwitterTPDOF*)ec_slave[i+1].inputs);
                float last_joint_position = joint_position_[j];
                exchage8bytes_.int64data = driver_feedback.motor_position;
                joint_position_[j] = exchage8bytes_.doubledata;
                robot_state_data_.joint_position_read[j] = exchage8bytes_.doubledata;

                float joint_velocity = (joint_position_[j] - last_joint_position)/period.toSec();
                joint_velocity_[j] = joint_velocity;
                robot_state_data_.joint_velocity_read[j] = joint_velocity;
//                exchage4bytes_.int32data = driver_feedback.joint_velocity;
//                joint_velocity_[j] = exchage4bytes_.floatdata;
//                robot_state_data_.joint_velocity_read[j] = exchage4bytes_.floatdata;

                exchage4bytes_.int32data = driver_feedback.motor_torque;
                joint_effort_[j] = exchage4bytes_.floatdata;
                robot_state_data_.joint_effort_read[j] = exchage4bytes_.floatdata;

                //          }
                break;
              }
            }
          continue;
        }
      junction_count++;
    }
  lock.unlock();
//  boost::recursive_mutex::scoped_lock lock(r_mutex_);
//  for(int i=0;i<ec_slavecount;i++)
//    driver_feedbacks.push_back(*feedbacks_[i]);
//  lock.unlock();

//  real_time_factor = 0.55;
//  for(unsigned int j=0; j < ec_slavecount; j++)
//  {
//      exchage8bytes_.int64data = driver_feedbacks[j].joint_position;
//      joint_position_[j] = exchage8bytes_.doubledata;
//      robot_state_data_.joint_position_read[j] = exchage8bytes_.doubledata;

//      exchage4bytes_.int32data = driver_feedbacks[j].joint_velocity;
//      joint_velocity_[j] = exchage4bytes_.floatdata;
//      robot_state_data_.joint_velocity_read[j] = exchage4bytes_.floatdata;

//      exchage4bytes_.int32data = driver_feedbacks[j].joint_torque;
//      joint_effort_[j] = exchage4bytes_.floatdata;
//      robot_state_data_.joint_effort_read[j] = exchage4bytes_.floatdata;
//  }

  /****************
* TODO(Shunyao) : update Imu data
****************/

  /****************
* TODO(Shunyao) : update leg control method according to foot contact info
****************/
//  robot_state_data_.foot_contact[0] =

//  for(unsigned int i = 0; i<contact_sensor_ptr->Contacts().contact_size();i++)
//    {

//    }

//  ROS_INFO("Finish Read from EtherCAT");
}

void RobotStateEtherCATHardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
  // If the E-stop is active, joints controlled by position commands will maintain their positions.
  if (e_stop_active_)
  {
    if (!last_e_stop_active_)
    {
      last_joint_position_command_ = joint_position_;
      last_e_stop_active_ = true;
    }
    joint_position_command_ = last_joint_position_command_;
  }
  else
  {
    last_e_stop_active_ = false;
  }

  ej_sat_interface_.enforceLimits(period);
  ej_limits_interface_.enforceLimits(period);
  pj_sat_interface_.enforceLimits(period);
  pj_limits_interface_.enforceLimits(period);
  vj_sat_interface_.enforceLimits(period);
  vj_limits_interface_.enforceLimits(period);

//  std::vector<ANYDriveRPDOA> driver_commands;
//  driver_commands.resize(n_dof_);
  int junction_count = 0;
  boost::recursive_mutex::scoped_lock lock(r_mutex_);
  for(int i = 0;i<ec_slavecount;i++)
    {
      if(slaves_type_[i] == SlaveType::ANYDRIVE)
        {
          int j = i - junction_count;
          switch (ANYdriveRPDOType_) {
            case PDOTYPE::RPDO_A:
              {
                ANYDriveRPDOA* driver_command;
                //        for(int j = 0;j<ec_slavecount;j++)
                //          {
                driver_command = (struct ANYDriveRPDOA*)ec_slave[i+1].outputs;
                switch (joint_control_methods_[j])
                  {
                  case EFFORT:
                    {
                      const double effort = e_stop_active_ ? 0 : joint_effort_command_[j-junction_count];
                      exchage4bytes_.floatdata = effort;
                      ROS_INFO("recieved joint '%d' torque command%f\n", j-junction_count, effort);
                      driver_command->desired_joint_torque = exchage4bytes_.int32data;
                      break;
                    }
                  case POSITION:
                    {
                      exchage8bytes_.doubledata = joint_position_command_[j-junction_count];
                      driver_command->desired_position = exchage8bytes_.int64data;
                      ROS_INFO("recieved joint '%d' position command%f\n", j-junction_count, joint_position_command_[j-junction_count]);
                      break;
                    }
                  case VELOCITY:
                    {
                      const double vel = e_stop_active_ ? 0 : joint_velocity_command_[j-junction_count];
                      exchage4bytes_.floatdata = vel;
                      driver_command->desired_velocity = exchage4bytes_.int32data;
                      ROS_INFO("recieved joint '%d' velocity command%f\n", j-junction_count, joint_velocity_command_[j-junction_count]);
                      break;
                    }
                  }
                //            ec_slave[j+1].outputs = (uint8 *)&driver_command;
                //          }
                break;
              }
            default:
              {
                ANYDriveRPDOA* driver_command;
                //        for(int j = 0;j<ec_slavecount;j++)
                //          {
                driver_command = (struct ANYDriveRPDOA*)ec_slave[i+1].outputs;
                switch (joint_control_methods_[j])
                  {
                  case EFFORT:
                    {
                      const double effort = e_stop_active_ ? 0 : joint_effort_command_[j];
                      exchage4bytes_.floatdata = effort;
                      ROS_INFO("recieved joint '%d' torque command%f\n", j, effort);
                      driver_command->desired_joint_torque = exchage4bytes_.int32data;
                      break;
                    }
                  case POSITION:
                    {
                      exchage8bytes_.doubledata = joint_position_command_[j];
                      driver_command->desired_position = exchage8bytes_.int64data;
                      //          ROS_INFO("recieved joint '%d' position command%f\n", j, joint_position_command_[j]);
                      break;
                    }
                  case VELOCITY:
                    {
                      const double vel = e_stop_active_ ? 0 : joint_velocity_command_[j];
                      exchage4bytes_.floatdata = vel;
                      driver_command->desired_velocity = exchage4bytes_.int32data;
                      break;
                    }
                  }
                //            ec_slave[j+1].outputs = (uint8 *)&driver_command;
                //          }
                break;
              }
            }
          continue;
        }
      if(slaves_type_[i] == SlaveType::GOLDENTWITTER)
        {
//          ROS_INFO("Write to Twitter");
          int j = i - junction_count;
          switch (GoldenTwitterRPDOType_) {
            case PDOTYPE::RPDO_F:
              {
                GoldenTwitterRPDOF* driver_command;
                //        for(int j = 0;j<ec_slavecount;j++)
                //          {
                driver_command = (struct GoldenTwitterRPDOF*)ec_slave[i+1].outputs;
//                switch (driver_command->control_word) {
//                  case 0:
//                    driver_command->control_word = GoldenTwitterControlWord::SHUT_DOWN;
//                    break;
//                  case GoldenTwitterControlWord::SHUT_DOWN:
//                    driver_command->control_word = GoldenTwitterControlWord::SWITCH_ON;
//                    break;
//                  case GoldenTwitterControlWord::SWITCH_ON:
//                    driver_command->control_word = GoldenTwitterControlWord::ENABLE_OP;
//                    break;
//                  case GoldenTwitterControlWord::FAULT_RESET:
//                    driver_command->control_word = 0;
//                    break;
//                  }
//                motor_disabled[j] = (bool)motor_unused[j];
                if(!checkPositionLimits(j, joint_position_[j])&&!((bool)motor_unused[j]))
                  {
                    ROS_WARN("Joint %d Position is Out Of Limit!!!",j);
//                    std::cout<<"joint position of "<<j<<" is "<<joint_position_[j]<<std::endl;
//                    std::cout<<"joint limits of "<<j<<" is ("<<motor_max_limits[j]<<", "<<motor_min_limits[j]<<")"<<std::endl;
                    setCommand(i, "Velocity", 0);
                    if(fabs(joint_velocity_[j])<1)
                      {
                        motor_disabled[j] = true;
                      }
                  }else if(motor_disabled[j]&&!((bool)motor_unused[j])){
                    switch (joint_control_methods_[j]) {
                      case EFFORT:
                        setModeOfOperation(i, "Torque");
                        break;
                      case POSITION:
                        setModeOfOperation(i, "Position");
                        break;
                      case VELOCITY:
                        setModeOfOperation(i, "Velocity");
                        break;
                      }
                  }
                e_stop_active_ = motor_disabled[j];
                if(e_stop_active_)
                  {
                    driver_command->control_word = GoldenTwitterControlWord::DISABLE_OP;
//                    ROS_WARN("Motor %d E-stop is actived", j);
                    continue;
                  }else{
                    switch ((status_word_[j] & 0b01101111)) {
                      case GoldenTwitterState::NOT_READY_TO_SWITCH_ON:
                        driver_command->control_word = GoldenTwitterControlWord::SHUT_DOWN;
                        ROS_ERROR("Driver %d is NOT READY TO SWITCH ON",j);
                        break;
                      case GoldenTwitterState::SWITCH_ON_DISABLED:
                        driver_command->control_word = GoldenTwitterControlWord::SHUT_DOWN;
                        ROS_ERROR("Driver %d is SWITCH ON DISABLED",j);
                        break;
                      case GoldenTwitterState::READY_TO_SWITCH_ON:
                        driver_command->control_word = GoldenTwitterControlWord::SWITCH_ON;
                        ROS_ERROR("Driver %d is  READY TO SWITCH ON",j);
                        break;
                      case GoldenTwitterState::SWITCHED_ON:
                        driver_command->control_word = GoldenTwitterControlWord::ENABLE_OP;
                        break;
                      case GoldenTwitterState::OP_ENABLED:
    //                    ROS_INFO("Driver %d is OP ENABLED",j);
                        break;
                      case GoldenTwitterState::QUICK_STOP_ACTIVED:
                        driver_command->control_word = GoldenTwitterControlWord::SHUT_DOWN;
                        ROS_ERROR("Driver %d is QUICK STOP ACTIVED",j);
                        break;
                      case GoldenTwitterState::FAULT:
                        driver_command->control_word = GoldenTwitterControlWord::FAULT_RESET;
                        ROS_ERROR("Driver %d is FAULT",j);
                        break;
                      default:
                        ROS_ERROR("ERROR STATUS !!!!!");
                        break;

                    }
                  }

                switch (joint_control_methods_[j])
                  {
                  case EFFORT:
                    {
                      double velocity = joint_velocity_[j];
                      double position_in_rad = motor_directions[j]*joint_position_[j] + motor_zero_offsets[j];
                      double position_in_2pi = angles::normalize_angle_positive(position_in_rad);
                      int position_in_degree = int(180*position_in_2pi/M_PI);//int(360*fmod(position_in_rad,2*M_PI)/(2*M_PI));


//                      position_in_degree = position_in_degree%360;

                      const double effort = e_stop_active_ ? 0 : joint_effort_command_[j];
                      double command = motor_directions[j]*effort;
//                      ROS_INFO("recieved joint '%d' torque command%f\n", j, effort);
                      int16 motor_torque = int16(command*TWITTER_CURRENT_TORQUE_RATIO);

                      if(motor_torque==0) //id-1 to consider first junction slave
                        motor_torque = motor_torque + motor_directions[j]*(motor_friction_proportion[j] + friction_of_pos[j][position_in_degree])*getMotorFrictionCompensation(j+1, velocity);
                      else if(command > 0.0)
                        {
                          motor_torque = int16(command*TWITTER_CURRENT_TORQUE_RATIO);
                          motor_torque = motor_torque + motor_directions[j]*(motor_friction_proportion[j]+ friction_of_pos[j][position_in_degree])*getMotorFrictionCompensation(j+1, fabs(velocity));
                        }
                      else if (command < 0.0) {
                          motor_torque = int16(command*TWITTER_CURRENT_TORQUE_RATIO);
                          motor_torque = motor_torque + motor_directions[j]*(motor_friction_proportion[j] + friction_of_pos[j][position_in_degree])*getMotorFrictionCompensation(j+1, -fabs(velocity));
                        }
                      driver_command->target_torque = motor_torque;//exchage4bytes_.int32data;
                      driver_command->max_torque = 1000;
//                      ROS_INFO("send torque in motor : %d", motor_torque);
                      break;
                    }
                  case POSITION:
                    {
//                      GoldenTwitterTPDOF* driver_feedback;
//                      driver_feedback = (struct GoldenTwitterTPDOF*)ec_slave[i +1].inputs;

                      if(status_word_[j] >> 12 == 1)
                        {
                          ROS_INFO("set point is not available");
                          driver_command->control_word = 47;
                        }else if (status_word_[j] >>12 == 0) {
                          ROS_INFO("set point is available");
                          driver_command->control_word = 63;
                        }

                      double command = motor_directions[j] * joint_position_command_[j] + motor_zero_offsets[j];
                      int32 command_cnts = int32(TWITTER_GEAR_RATIO * TWITTER_ENCODER_RES * command/(2 * M_PI));
                      ROS_INFO("send position in motor cnts is : %d",command_cnts);
                      driver_command->target_position = command_cnts;
                      driver_command->max_torque = 1000;
                      ROS_INFO("recieved joint '%d' position command%f\n", j, motor_directions[j] * joint_position_command_[j]);
                      break;
                    }
                  case VELOCITY:
                    {
                      const double vel = e_stop_active_ ? 0 : joint_velocity_command_[j];
//                      int32 vel_cnts = TWITTER_GEAR_RATIO * TWITTER_ENCODER_RES * vel * 2 * M_PI/60;
                      int32 vel_cnts = motor_directions[j] * TWITTER_GEAR_RATIO * TWITTER_ENCODER_RES * vel / (2 * M_PI);
                      driver_command->target_velocity = vel_cnts;//exchage4bytes_.int32data;
                      driver_command->max_torque = 1000;
                      driver_command->control_word = GoldenTwitterControlWord::SWITCH_ON_AND_ENABLE;
                      ROS_INFO("send velocity in motor cnts is : %d",vel_cnts);
                      ROS_INFO("recieved joint '%d' velocity command%f\n", j, joint_velocity_command_[j]);
                      break;
                    }
                  }
                //            ec_slave[j+1].outputs = (uint8 *)&driver_command;
                //          }
                break;
              }
            default:
              {
                GoldenTwitterRPDOF* driver_command;
                //        for(int j = 0;j<ec_slavecount;j++)
                //          {
                driver_command = (struct GoldenTwitterRPDOF*)ec_slave[i+1].outputs;
                switch (joint_control_methods_[j])
                  {
                  case EFFORT:
                    {
                      const double effort = e_stop_active_ ? 0 : joint_effort_command_[j];
                      exchage4bytes_.floatdata = effort;
                      ROS_INFO("recieved joint '%d' torque command%f\n", j, effort);
                      driver_command->target_torque = exchage4bytes_.int32data;
                      break;
                    }
                  case POSITION:
                    {
                      exchage8bytes_.doubledata = joint_position_command_[j];
                      driver_command->target_position = exchage8bytes_.int64data;
                      //          ROS_INFO("recieved joint '%d' position command%f\n", j, joint_position_command_[j]);
                      break;
                    }
                  case VELOCITY:
                    {
                      const double vel = e_stop_active_ ? 0 : joint_velocity_command_[j];
                      exchage4bytes_.floatdata = vel;
                      driver_command->target_velocity = exchage4bytes_.int32data;
                      break;
                    }
                  }
                //            ec_slave[j+1].outputs = (uint8 *)&driver_command;
                //          }
                break;
              }
            }
          continue;
        }
      junction_count++;

    }
  lock.unlock();
//  boost::recursive_mutex::scoped_lock lock(r_mutex_);
//  //! WSHY: keep the unwrite data same as initial
//  for(int i = 0;i<commands_.size();i++)
//    driver_commands[i] = *commands_[i];
//  lock.unlock();


//  for(unsigned int j=0; j < n_dof_; j++)
//  {

//    switch (joint_control_methods_[j])
//    {
//      case EFFORT:
//        {
//          const double effort = e_stop_active_ ? 0 : joint_effort_command_[j];
//          exchage4bytes_.floatdata = effort;
//          ROS_INFO("recieved joint '%d' torque command%f\n", j, effort);
//          driver_commands[j].desired_joint_torque = exchage4bytes_.int32data;
//        break;
//        }
//      case POSITION:
//        {
//          exchage8bytes_.doubledata = joint_position_command_[j];
//          driver_commands[j].desired_position = exchage8bytes_.int64data;
////          ROS_INFO("recieved joint '%d' position command%f\n", j, joint_position_command_[j]);
//          break;
//        }
//      case VELOCITY:
//        {
//          const double vel = e_stop_active_ ? 0 : joint_velocity_command_[j];
//          exchage4bytes_.floatdata = vel;
//          driver_commands[j].desired_velocity = exchage4bytes_.int32data;
//          break;
//        }
//      }
//    }
////    boost::recursive_mutex::scoped_lock lock(r_mutex_);
//    lock.lock();


//    for(int i=0;i<ec_slavecount;i++)
//      {
//        ec_slave[i].outputs = (uint8 *)&driver_commands[i];
////        *commands_[i] = driver_commands[i];
////        exchage8bytes_.int64data = commands_[i]->desired_position;
////        ROS_INFO("write joint '%d' position command%f\n", i, exchage8bytes_.doubledata);
//      }
//    lock.unlock();
//    ROS_INFO("Finish Write from EtherCAT");

}


bool RobotStateEtherCATHardwareInterface::checkPositionLimits(const int index, const double position)
{
  if((position > motor_max_limits[index]) || (position < motor_min_limits[index]))
    {
      return false;
    }
  return true;
}

void RobotStateEtherCATHardwareInterface::eStopActive(const bool active)
{
  e_stop_active_ = active;
  for(int i = 0; i<motor_disabled.size(); i++)
    {
      motor_disabled[i] = active;
      if(motor_unused[i] == 1)
        motor_disabled[i] = (bool)motor_unused[i];
    }
}

// Register the limits of the joint specified by joint_name and joint_handle. The limits are
// retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
// Return the joint's type, lower position limit, upper position limit, and effort limit.
void RobotStateEtherCATHardwareInterface::registerJointLimits(const std::string& joint_name,
                         const hardware_interface::JointHandle& joint_handle,
                         const ControlMethod ctrl_method,
                         const ros::NodeHandle& joint_limit_nh,
                         const urdf::Model *const urdf_model,
                         int *const joint_type, double *const lower_limit,
                         double *const upper_limit, double *const effort_limit)
{
  *joint_type = urdf::Joint::UNKNOWN;
  *lower_limit = -std::numeric_limits<double>::max();
  *upper_limit = std::numeric_limits<double>::max();
  *effort_limit = std::numeric_limits<double>::max();

  joint_limits_interface::JointLimits limits;
  bool has_limits = false;
  joint_limits_interface::SoftJointLimits soft_limits;
  bool has_soft_limits = false;

  if (urdf_model != NULL)
  {
    const urdf::JointConstSharedPtr urdf_joint = urdf_model->getJoint(joint_name);
    if (urdf_joint != NULL)
    {
      *joint_type = urdf_joint->type;
      // Get limits from the URDF file.
      if (joint_limits_interface::getJointLimits(urdf_joint, limits))
        has_limits = true;
      if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
        has_soft_limits = true;
    }
  }
  // Get limits from the parameter server.
  if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
    has_limits = true;

  if (!has_limits)
    return;

  if (*joint_type == urdf::Joint::UNKNOWN)
  {
    // Infer the joint type.

    if (limits.has_position_limits)
    {
      *joint_type = urdf::Joint::REVOLUTE;
    }
    else
    {
      if (limits.angle_wraparound)
        *joint_type = urdf::Joint::CONTINUOUS;
      else
        *joint_type = urdf::Joint::PRISMATIC;
    }
  }

  if (limits.has_position_limits)
  {
    *lower_limit = limits.min_position;
    *upper_limit = limits.max_position;
  }
  if (limits.has_effort_limits)
    *effort_limit = limits.max_effort;

  if (has_soft_limits)
  {
    switch (ctrl_method)
    {
      case EFFORT:
        {
          const joint_limits_interface::EffortJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          ej_limits_interface_.registerHandle(limits_handle);
        }
        break;
      case POSITION:
        {
          const joint_limits_interface::PositionJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          pj_limits_interface_.registerHandle(limits_handle);
        }
        break;
      case VELOCITY:
        {
          const joint_limits_interface::VelocityJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          vj_limits_interface_.registerHandle(limits_handle);
        }
        break;
    }
  }
  else
  {
    switch (ctrl_method)
    {
      case EFFORT:
        {
          const joint_limits_interface::EffortJointSaturationHandle
            sat_handle(joint_handle, limits);
          ej_sat_interface_.registerHandle(sat_handle);
        }
        break;
      case POSITION:
        {
          const joint_limits_interface::PositionJointSaturationHandle
            sat_handle(joint_handle, limits);
          pj_sat_interface_.registerHandle(sat_handle);
        }
        break;
      case VELOCITY:
        {
          const joint_limits_interface::VelocityJointSaturationHandle
            sat_handle(joint_handle, limits);
          vj_sat_interface_.registerHandle(sat_handle);
        }
        break;
    }
  }
}


char* RobotStateEtherCATHardwareInterface::getSlaveName(int index)
{
  return ec_slave[index].name;
}
char* RobotStateEtherCATHardwareInterface::getSlaveECState(int index)
{
  switch (ec_slave[index].state) {
    case EC_STATE_INIT:
      return (char*)"EC_STATE_INIT";
    case EC_STATE_PRE_OP:
      return (char*)"EC_STATE_PRE_OP";
    case EC_STATE_BOOT:
      return (char*)"EC_STATE_BOOT";
    case EC_STATE_SAFE_OP:
      return (char*)"EC_STATE_SAFE_OP";
    case EC_STATE_OPERATIONAL:
      return (char*)"EC_STATE_OPERATIONAL";
    case EC_STATE_ERROR:
      return (char*)"EC_STATE_ERROR";
    }
}
uint16 RobotStateEtherCATHardwareInterface::getSlaveAddress(int index)
{
  return ec_slave[index].configadr;
}
char* RobotStateEtherCATHardwareInterface::getSlaveStatus(int index, const std::string& name)
{
  SlaveType type = slaves_type_[index];
  if(type == SlaveType::ANYDRIVE)
    {
      switch (ANYdriveTPDOType_) {
        case PDOTYPE::TPDO_A:
          {
            ANYDriveTPDOA* driver_feedback;
            driver_feedback = (struct ANYDriveTPDOA*)ec_slave[index+1].inputs;
            switch ((driver_feedback->state<<28)>>28) {
              case ANYDriveFSMState::CONFIGURE:
                return (char*)"Configure";
              case ANYDriveFSMState::STANDBY:
                return (char*)"Standby";
              case ANYDriveFSMState::MOTOR_OP:
                return (char*)"MotroOp";
              case ANYDriveFSMState::CONTROL_OP:
                return (char*)"ControlOp";
              case ANYDriveFSMState::MOTOR_PRE_OP:
                return (char*)"MotorPreOp";
              case ANYDriveFSMState::WARM_START:
                return (char*)"WarmStart";
              default:
                return (char*)"N/A";

            }
            break;
          }
        default:
          {
            ANYDriveTPDOA* driver_feedback;
            driver_feedback = (struct ANYDriveTPDOA*)ec_slave[index+1].inputs;
            switch ((driver_feedback->state<<28)>>28) {
              case ANYDriveFSMState::CONFIGURE:
                return (char*)"Configure";
              case ANYDriveFSMState::STANDBY:
                return (char*)"Standby";
              case ANYDriveFSMState::MOTOR_OP:
                return (char*)"MotroOp";
              case ANYDriveFSMState::CONTROL_OP:
                return (char*)"ControlOp";
              case ANYDriveFSMState::MOTOR_PRE_OP:
                return (char*)"MotorPreOp";
              case ANYDriveFSMState::WARM_START:
                return (char*)"WarmStart";
              default:
                return (char*)"N/A";

          }
            break;
        }

//      switch ((feedbacks_[index]->state<<28)>>28) {
//        case ANYDriveFSMState::CONFIGURE:
//          return (char*)"Configure";
//        case ANYDriveFSMState::STANDBY:
//          return (char*)"Standby";
//        case ANYDriveFSMState::MOTOR_OP:
//          return (char*)"MotroOp";
//        case ANYDriveFSMState::CONTROL_OP:
//          return (char*)"ControlOp";
//        case ANYDriveFSMState::MOTOR_PRE_OP:
//          return (char*)"MotorPreOp";
//        case ANYDriveFSMState::WARM_START:
//          return (char*)"WarmStart";
//        default:
//          return (char*)"N/A";
        }
    }
  if(type == SlaveType::GOLDENTWITTER)
    {
      switch (GoldenTwitterTPDOType_) {
        case PDOTYPE::TPDO_F:
          {
            GoldenTwitterTPDOF* driver_feedback;
            driver_feedback = (struct GoldenTwitterTPDOF*)ec_slave[index+1].inputs;
            switch ((driver_feedback->status_word & 0b01101111)) {
              case GoldenTwitterState::NOT_READY_TO_SWITCH_ON:
                return (char*)"Not Ready";
              case GoldenTwitterState::SWITCH_ON_DISABLED:
                return (char*)"Switch On Disabled";
              case GoldenTwitterState::READY_TO_SWITCH_ON:
                return (char*)"Ready To Switch On";
              case GoldenTwitterState::SWITCHED_ON:
                return (char*)"Switched On";
              case GoldenTwitterState::OP_ENABLED:
                return (char*)"Enable Operation";
              case GoldenTwitterState::QUICK_STOP_ACTIVED:
                return (char*)"Quick Stopped";
              case GoldenTwitterState::FAULT:
                return (char*)"Fault";
              default:
                return (char*)"N/A";

            }
            break;
          }
        default:
          {
            GoldenTwitterTPDOF* driver_feedback;
            driver_feedback = (struct GoldenTwitterTPDOF*)ec_slave[index+1].inputs;
            switch ((driver_feedback->status_word & 0b01101111)) {
              case GoldenTwitterState::NOT_READY_TO_SWITCH_ON:
                return (char*)"Not Ready";
              case GoldenTwitterState::SWITCH_ON_DISABLED:
                return (char*)"Switch On Disabled";
              case GoldenTwitterState::READY_TO_SWITCH_ON:
                return (char*)"Ready To Switch On";
              case GoldenTwitterState::SWITCHED_ON:
                return (char*)"Switched On";
              case GoldenTwitterState::OP_ENABLED:
                return (char*)"Enable Operation";
              case GoldenTwitterState::QUICK_STOP_ACTIVED:
                return (char*)"Quick Stopped";
              case GoldenTwitterState::FAULT:
                return (char*)"Fault";
              default:
                return (char*)"N/A";

            }
            break;
          }
        }
    }
  return (char*)"N/A";
}

char* RobotStateEtherCATHardwareInterface::getSlaveMode(int index, const std::string& name)
{
  SlaveType type = slaves_type_[index];
  if(type == SlaveType::ANYDRIVE)
    {
      switch (ANYdriveTPDOType_) {
        case PDOTYPE::TPDO_A:
          {
            ANYDriveTPDOA* driver_feedback;
            driver_feedback = (struct ANYDriveTPDOA*)ec_slave[index+1].inputs;

            switch ((driver_feedback->state<<24)>>28) {
              case ANYDriveModeOfOperation::FREEZE:
                return (char*)"Freeze";
              case ANYDriveModeOfOperation::DISABLE:
                return (char*)"Disable";
              case ANYDriveModeOfOperation::JOINT_POSITION:
                return (char*)"JointPosition";
              case ANYDriveModeOfOperation::JOINT_VELOCITY:
                return (char*)"JointVelocity";
              case ANYDriveModeOfOperation::JOINT_TORQUE:
                return (char*)"JointTorque";
              default:
                return (char*)"N/A";
              }
            break;
          }
        default:
          {
            ANYDriveTPDOA* driver_feedback;
            driver_feedback = (struct ANYDriveTPDOA*)ec_slave[index+1].inputs;

            switch ((driver_feedback->state<<24)>>28) {
              case ANYDriveModeOfOperation::FREEZE:
                return (char*)"Freeze";
              case ANYDriveModeOfOperation::DISABLE:
                return (char*)"Disable";
              case ANYDriveModeOfOperation::JOINT_POSITION:
                return (char*)"JointPosition";
              case ANYDriveModeOfOperation::JOINT_VELOCITY:
                return (char*)"JointVelocity";
              case ANYDriveModeOfOperation::JOINT_TORQUE:
                return (char*)"JointTorque";
              default:
                return (char*)"N/A";
              }
            break;
          }

      }
    }
  if(type == SlaveType::GOLDENTWITTER)
    {
      switch (GoldenTwitterTPDOType_) {
        case PDOTYPE::TPDO_F:
          {
            GoldenTwitterTPDOF* driver_feedback;
            driver_feedback = (struct GoldenTwitterTPDOF*)ec_slave[index+1].inputs;

            switch (driver_feedback->mode_of_opreration) {
              case GoldenTwitterModeOfOperation::PROFILE_POSITION:
                return (char*)"Profile Position";
              case GoldenTwitterModeOfOperation::PROFILE_VELOCITY:
                return (char*)"Profile Velocity";
              case GoldenTwitterModeOfOperation::PROFILE_TORQUE:
                return (char*)"Profiled Torque";
              case GoldenTwitterModeOfOperation::HOMING:
                return (char*)"Homing";
              case GoldenTwitterModeOfOperation::SYNC_POSITION:
                return (char*)"Synchronous Position";
              case GoldenTwitterModeOfOperation::SYNC_VELOCITY:
                return (char*)"Synchronous Velocity";
              case GoldenTwitterModeOfOperation::SYNC_TORQUE:
                return (char*)"Synchronous Torque";
              default:
                return (char*)"N/A";
              }
            break;
          }
        default:
          {
            GoldenTwitterTPDOF* driver_feedback;
            driver_feedback = (struct GoldenTwitterTPDOF*)ec_slave[index+1].inputs;

            switch (driver_feedback->mode_of_opreration) {
              case GoldenTwitterModeOfOperation::PROFILE_POSITION:
                return (char*)"Profile Position";
              case GoldenTwitterModeOfOperation::PROFILE_VELOCITY:
                return (char*)"Profile Velocity";
              case GoldenTwitterModeOfOperation::PROFILE_TORQUE:
                return (char*)"Profiled Torque";
              case GoldenTwitterModeOfOperation::HOMING:
                return (char*)"Homing";
              case GoldenTwitterModeOfOperation::SYNC_POSITION:
                return (char*)"Synchronous Position";
              case GoldenTwitterModeOfOperation::SYNC_VELOCITY:
                return (char*)"Synchronous Velocity";
              case GoldenTwitterModeOfOperation::SYNC_TORQUE:
                return (char*)"Synchronous Torque";
              default:
                return (char*)"N/A";
              }
            break;
          }

      }
    }
  return (char*)"N/A";
}

Eigen::Vector3d RobotStateEtherCATHardwareInterface::getJointFeedback(int index, const std::string& name)
{
  Eigen::Vector3d joint_state;
  SlaveType type = slaves_type_[index];
  int motor_count = index;
  for(int i = 0; i<index;i++)
    {
      if(slaves_type_[i] == SlaveType::JUNCTION)
        motor_count --;
    }

  if(type == SlaveType::ANYDRIVE)
    {
      switch (ANYdriveTPDOType_) {
        case PDOTYPE::TPDO_A:
          {
            ANYDriveTPDOA* driver_feedback;
            driver_feedback = (struct ANYDriveTPDOA*)ec_slave[index+1].inputs;
            exchage8bytes_.int64data = driver_feedback->joint_position;
            joint_state(0) = exchage8bytes_.doubledata;
            exchage4bytes_.int32data = driver_feedback->joint_velocity;
            joint_state(1) = exchage4bytes_.floatdata;
            exchage4bytes_.int32data = driver_feedback->joint_torque;
            joint_state(2) = exchage4bytes_.floatdata;
            break;
          }
        default:
          {
            ANYDriveTPDOA* driver_feedback;
            driver_feedback = (struct ANYDriveTPDOA*)ec_slave[index+1].inputs;
            exchage8bytes_.int64data = driver_feedback->joint_position;
            joint_state(0) = exchage8bytes_.doubledata;
            exchage4bytes_.int32data = driver_feedback->joint_velocity;
            joint_state(1) = exchage4bytes_.floatdata;
            exchage4bytes_.int32data = driver_feedback->joint_torque;
            joint_state(2) = exchage4bytes_.floatdata;
            break;
          }

        }
    }
  if(type == SlaveType::GOLDENTWITTER)
    {
      switch (GoldenTwitterTPDOType_) {
        case PDOTYPE::TPDO_F:
          {
            GoldenTwitterTPDOF* driver_feedback;
            driver_feedback = (struct GoldenTwitterTPDOF*)ec_slave[index+1].inputs;
            // in rad
            joint_state(0) = motor_directions[motor_count] * (2 * M_PI * driver_feedback->motor_position/(TWITTER_ENCODER_RES*TWITTER_GEAR_RATIO) - motor_zero_offsets[motor_count]);
            // rpm on load
//            joint_state(1) = 60.0 * driver_feedback->motor_velocity/(TWITTER_ENCODER_RES*TWITTER_GEAR_RATIO*2*M_PI);
            // rad/s on load
            joint_state(1)= motor_directions[motor_count] * 2*M_PI * driver_feedback->motor_velocity/(TWITTER_ENCODER_RES*TWITTER_GEAR_RATIO);

            // percent of max torque
            /****************
* TODO(Shunyao) : calibrate the torque and frictions
****************/
            joint_state(2) = motor_directions[motor_count] * driver_feedback->motor_torque/TWITTER_CURRENT_TORQUE_RATIO;
            break;
          }
        default:
          {
            GoldenTwitterTPDOF* driver_feedback;
            driver_feedback = (struct GoldenTwitterTPDOF*)ec_slave[index+1].inputs;
            // in rad
            joint_state(0) = 2 * M_PI * driver_feedback->motor_position/(TWITTER_ENCODER_RES*TWITTER_GEAR_RATIO);
            // rpm on load
            joint_state(1) = 60.0 * driver_feedback->motor_velocity/(TWITTER_ENCODER_RES*TWITTER_GEAR_RATIO*2*M_PI);
            // percent of max torque
            /****************
* TODO(Shunyao) : calibrate the torque and frictions
****************/
            joint_state(2) = driver_feedback->motor_torque/10.0;
            break;
          }

        }
    }
//  exchage8bytes_.int64data = feedbacks_[index]->joint_position;
//  joint_state(0) = exchage8bytes_.doubledata;
//  exchage4bytes_.int32data = feedbacks_[index]->joint_velocity;
//  joint_state(1) = exchage4bytes_.floatdata;
//  exchage4bytes_.int32data = feedbacks_[index]->joint_torque;
//  joint_state(2) = exchage4bytes_.floatdata;

  return joint_state;
}
int RobotStateEtherCATHardwareInterface::getNumberOfSlaves()
{
  return ec_slavecount;
}

double RobotStateEtherCATHardwareInterface::getMotorFrictionCompensation(int index, double velocity)
{
//  switch (index) {
//    case 1:
//      if(velocity>0)
//        return std::min(5.605 * velocity + 50.914, 100.0);
//      if(velocity<0)
//        return std::max(5.605 * velocity - 50.914, -100.0);
//    default:
//      return 50;
//    }

  if(velocity>0)
    return std::min(motor_friction_mu[index -1] * velocity + motor_friction_bias[index-1], 100.0);
  if(velocity<0)
    return std::max(motor_friction_mu[index -1] * velocity - motor_friction_bias[index - 1], -100.0);
  return 0;
}
}

PLUGINLIB_EXPORT_CLASS(ros_ethercat_driver::RobotStateEtherCATHardwareInterface, hardware_interface::RobotHW)
