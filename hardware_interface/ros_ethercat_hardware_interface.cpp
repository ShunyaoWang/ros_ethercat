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

}

RobotStateEtherCATHardwareInterface::~RobotStateEtherCATHardwareInterface()
{
  ros::Duration delay(0.1);
  while (!DeInitSlaves(SlaveType::ANYDRIVE)) {

      delay.sleep();
    }

  ecx_close(&ecx_context);
  ROS_WARN("EtherCAT HardWare Interface Shutdown");
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
  pid_controllers_.resize(n_dof_);
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_effort_command_.resize(n_dof_);
  joint_position_command_.resize(n_dof_);
  joint_velocity_command_.resize(n_dof_);
  // Initialize values

  hardware_interface::JointHandle joint_handle;
  for(unsigned int j = 0;j<n_dof_;j++)
    {
      joint_position_[j] = 1.0;
      joint_velocity_[j] = 0.0;
      joint_effort_[j] = 1.0;  // N/m for continuous joints
      joint_effort_command_[j] = 0.0;
      joint_position_command_[j] = 0.0;
      joint_velocity_command_[j] = 0.0;

      // Create joint state interface for all joints
      js_interface_.registerHandle(hardware_interface::JointStateHandle(
          joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

      // Create effort joint interface
      joint_control_methods_[j] = EFFORT;
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_effort_command_[j]);
//      hardware_interface::RobotStateHandle(js_interface_.getHandle(joint_names_[j],
//                                                                   &joint_effort_command_[j]));
      robot_state_interface_.joint_effort_interfaces_.registerHandle(joint_handle);
      ej_interface_.registerHandle(joint_handle);

      // Create position joint interface
      joint_control_methods_[j] = POSITION;
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_position_command_[j]);
      pj_interface_.registerHandle(joint_handle);

      // Create velocity joint interface
      joint_control_methods_[j] = VELOCITY;
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_velocity_command_[j]);
      vj_interface_.registerHandle(joint_handle);

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


  feedbacks_.resize(n_dof_);
  commands_.resize(n_dof_);
//  anydriveTest(ifname);
  if(!EtherCATInit()){
     ROS_ERROR("Failed to initialize EtherCAT Conmunication !!!!!!!!!");
     return false;
  }
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
  while (!InitSlaves(SlaveType::ANYDRIVE)) {

      delay.sleep();
    }
  ROS_INFO("Successfully Initialize EtherCAT Hardware Interface");
  return true;
}

bool RobotStateEtherCATHardwareInterface::DeInitSlaves(SlaveType slave_type)
{
  switch (slave_type) {
    case SlaveType::ANYDRIVE:
      {
        for(int i=0;i<n_dof_;i++)
          {
            switch ((feedbacks_[i]->state<<28)>>28) {
              case ANYDriveFSMState::CONFIGURE:
                commands_[i]->control_word = ANYDriveControlWord::CONFIGURE_TO_STANDBY;
                break;
              case ANYDriveFSMState::STANDBY:
                commands_[i]->control_word = 0;
                break;
              case ANYDriveFSMState::MOTOR_OP:
                commands_[i]->control_word = ANYDriveControlWord::MOTOR_OP_TO_STANDBY;
                break;
              case ANYDriveFSMState::CONTROL_OP:
                commands_[i]->control_word = ANYDriveControlWord::CONTROL_OP_TO_STANDBY;
                commands_[i]->mode_of_operation = ANYDriveModeOfOperation::FREEZE;
                if(i==n_dof_-1)
                  return true;
                break;
              }
            return false;
          }
        break;
      }
    case SlaveType::ELMOGOLDEN:
      {
        break;
      }
    }
  return true;
}

bool RobotStateEtherCATHardwareInterface::InitSlaves(SlaveType slave_type)
{
  switch (slave_type) {
    case SlaveType::ANYDRIVE:
      {
        for(int i=0;i<n_dof_;i++)
          {
            switch ((feedbacks_[i]->state<<28)>>28) {
              case ANYDriveFSMState::CONFIGURE:
                commands_[i]->control_word = ANYDriveControlWord::CONFIGURE_TO_STANDBY;
                break;
              case ANYDriveFSMState::STANDBY:
                commands_[i]->control_word = ANYDriveControlWord::STANDBY_TO_MOTOR_PRE_OP;
                break;
              case ANYDriveFSMState::MOTOR_OP:
                commands_[i]->control_word = ANYDriveControlWord::MOTOR_OP_TO_CONTROL_OP;
                break;
              case ANYDriveFSMState::CONTROL_OP:
                ROS_INFO("Switch to Control_OP");
                commands_[i]->control_word = 0;
                switch (joint_control_methods_[i]) {
                  case EFFORT:
                    commands_[i]->mode_of_operation = ANYDriveModeOfOperation::JOINT_TORQUE;
                    ROS_INFO("Switch to Joint Torque Mode");
                    break;
                  case VELOCITY:
                    commands_[i]->mode_of_operation = ANYDriveModeOfOperation::JOINT_VELOCITY;
                    ROS_INFO("Switch to Joint Velocity Mode");
                    break;
                  case POSITION:
                    commands_[i]->mode_of_operation = ANYDriveModeOfOperation::JOINT_POSITION;
                    ROS_INFO("Switch to Joint Position Mode ");
                    break;

                  }
                ROS_INFO("Switch to Control_OP");
                if(i==ec_slavecount-1)
                  return true;
              }
            return false;
          }
        break;
      }
    case SlaveType::ELMOGOLDEN:
      {
        break;
      }
    }
  return true;
}
bool RobotStateEtherCATHardwareInterface::EtherCATInit()
{
//  anydriveTest(ifname);
  ROS_INFO("Starting EtherCAT Init");
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

        for (int i=1; i<=ec_slavecount; i++) {
            printf("Slave %d has CA? %s\n", i, ec_slave[i].CoEdetails & ECT_COEDET_SDOCA ? "true":"false" );
        }

        for (int i=1; i<=ec_slavecount; i++) {
            //! WSHY: the ANYDrive is not support LRW(Logical READ&WRITE) but, it can't read
            //! by the upload slave info, so we manually assigned the value to Block LRW, before
            //! ec_config_map set it to ec_context.
            ec_slave[i].blockLRW = 1;
        }
        //! WSHY: map the all input and output of all slaves to a static array
        ecx_config_map_group(&ecx_context, &IOmap, 0);

        ecx_configdc(&ecx_context);

        printf("Slaves mapped, state to SAFE_OP.\n");
        /* wait for all slaves to reach SAFE_OP state */
        ecx_statecheck(&ecx_context,0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

        printf("Attach data struct to slave inputs/outputs.\n");
        /* send one valid process data to make outputs in slaves happy*/
        ecx_send_processdata(&ecx_context);
        ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);

        for (int i=1; i<=ec_slavecount; i++) {
            // show slave info
            printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
            i, ec_slave[i].name, ec_slave[i].Obits, ec_slave[i].Ibits,
            ec_slave[i].state, ec_slave[i].pdelay, ec_slave[i].hasdc);
            ec_slave[i].state = EC_STATE_OPERATIONAL;
            //! WSHY: assigned the inputs and outputs pointer to out read and write data struct
            feedbacks_[i-1] = (struct ANYDriveTPDO *)(ec_slave[i].inputs);
            commands_[i-1] = (struct ANYDriveRPDO *)(ec_slave[i].outputs);
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

void RobotStateEtherCATHardwareInterface::EtherCATLoop()
{
  ROS_INFO("Starting EtherCAT update loop");
  if (ec_slave[0].state == EC_STATE_OPERATIONAL )
  {
      ros::Rate rate(1000);
      while (ros::ok()) {
          ecx_send_processdata(&ecx_context);
          int wkc = ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);

          rate.sleep();
        }
    }
  ecx_close(&ecx_context);

}

void RobotStateEtherCATHardwareInterface::EtherCATLoop(const ros::TimerEvent&)
{
//  ROS_INFO("Starting EtherCAT update loop");
  if (ec_slave[0].state == EC_STATE_OPERATIONAL )
  {
          ecx_send_processdata(&ecx_context);
          int wkc = ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
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
  std::vector<ANYDriveTPDO> driver_feedbacks;
  boost::recursive_mutex::scoped_lock lock(r_mutex_);
  for(int i=0;i<feedbacks_.size();i++)
    driver_feedbacks.push_back(*feedbacks_[i]);
  lock.unlock();

//  real_time_factor = 0.55;
  for(unsigned int j=0; j < ec_slavecount; j++)
  {
      exchage8bytes_.int64data = driver_feedbacks[j].joint_position;
      joint_position_[j] = exchage8bytes_.doubledata;
      robot_state_data_.joint_position_read[j] = exchage8bytes_.doubledata;

      exchage4bytes_.int32data = driver_feedbacks[j].joint_velocity;
      joint_velocity_[j] = exchage4bytes_.floatdata;
      robot_state_data_.joint_velocity_read[j] = exchage4bytes_.floatdata;

      exchage4bytes_.int32data = driver_feedbacks[j].joint_torque;
      joint_effort_[j] = exchage4bytes_.floatdata;
      robot_state_data_.joint_effort_read[j] = exchage4bytes_.floatdata;
  }

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

  std::vector<ANYDriveRPDO> driver_commands;
  driver_commands.resize(n_dof_);



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

  for(unsigned int j=0; j < n_dof_; j++)
  {
    switch (joint_control_methods_[j])
    {
      case EFFORT:
        {
          const double effort = e_stop_active_ ? 0 : joint_effort_command_[j];
          exchage4bytes_.floatdata = effort;
          driver_commands[j].desired_joint_torque = exchage4bytes_.int32data;
        break;
        }
      case POSITION:
        {
          exchage8bytes_.doubledata = joint_position_command_[j];
          driver_commands[j].desired_position = exchage8bytes_.int64data;
          break;
        }
      case VELOCITY:
        {
          const double vel = e_stop_active_ ? 0 : joint_velocity_command_[j];
          exchage4bytes_.floatdata = vel;
          driver_commands[j].desired_velocity = exchage4bytes_.int32data;
          break;
        }
      }
    }
    boost::recursive_mutex::scoped_lock lock(r_mutex_);
    for(int i=0;i<commands_.size();i++)
      *commands_[i] = driver_commands[i] ;
    lock.unlock();
//    ROS_INFO("Finish Write from EtherCAT");

}

void RobotStateEtherCATHardwareInterface::eStopActive(const bool active)
{
  e_stop_active_ = active;
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

}

PLUGINLIB_EXPORT_CLASS(ros_ethercat_driver::RobotStateEtherCATHardwareInterface, hardware_interface::RobotHW)
