#include "ros_ethercat_driver/hardware_interface/ros_ethercat_hardware_interface.hpp"
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "hardware_test");
  ros::NodeHandle nh("~");
  ros_ethercat_driver::RobotStateEtherCATHardwareInterface EtherCAT_HW;
  EtherCAT_HW.ifname = argv[1];
  EtherCAT_HW.EtherCATInit();
  EtherCAT_HW.EtherCATLoop();
  return 0;
}
