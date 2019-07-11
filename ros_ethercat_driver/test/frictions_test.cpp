#include "ros/ros.h"
#include "controller_manager_msgs/SwitchController.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "boost/thread/thread.hpp"
#include "iostream"
#include "fstream"
#include "ros/package.h"

using namespace std;


bool velocity_change_flag = false;
double torque_sum = 0;
int record_count = 0;
std::vector<double> avarage_torque;
std::vector<double> test_velocity;

void jointFeedbackCallback(const sensor_msgs::JointState::ConstPtr& joint_states)
{
//  ROS_INFO("update");
  record_count ++;
  torque_sum = torque_sum + joint_states->effort[0]*16.67;
  if(velocity_change_flag)
    {
      avarage_torque.push_back(torque_sum/record_count);
      torque_sum = 0;
      record_count = 0;
      velocity_change_flag = false;
    }
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "friction_test");
  ros::NodeHandle nodehandle_("~");

  string record_dir = ros::package::getPath("ros_ethercat_driver");

  ofstream outfile(record_dir+"/test/friction_test_results.txt");

  ros::ServiceClient switchControllerClient_ = nodehandle_.serviceClient<controller_manager_msgs::SwitchController>
      ("/controller_manager/switch_controller", false);

  ros::Publisher eStopPublisher_ = nodehandle_.advertise<std_msgs::Bool>("/e_stop", 1);

  ros::Publisher jointVelocityCommandPublisher_ = nodehandle_.advertise<std_msgs::Float64MultiArray>("/all_joints_velocity_group_controller/command", 1);

  ros::Subscriber jointStateSubscriber_ = nodehandle_.subscribe("/joint_states", 1, jointFeedbackCallback);

  controller_manager_msgs::SwitchController controller_switch;

  controller_switch.request.start_controllers.push_back("all_joints_velocity_group_controller");
  controller_switch.request.stop_controllers.push_back("");
  controller_switch.request.strictness = controller_switch.request.STRICT;
  switchControllerClient_.call(controller_switch.request, controller_switch.response);

  std_msgs::Bool e_stop_msg;

  e_stop_msg.data = false;
  eStopPublisher_.publish(e_stop_msg);
  ros::Duration(0.5).sleep();
  eStopPublisher_.publish(e_stop_msg);
  ros::Duration(0.5).sleep();
  eStopPublisher_.publish(e_stop_msg);

  // Spin
  ros::AsyncSpinner spinner(1); // Use n threads
  spinner.start();

  std_msgs::Float64MultiArray joint_velocity_command;
  joint_velocity_command.data.resize(12);
  double step = 0.1;
  double velocity = 0.0;

  while (ros::ok()) {
      eStopPublisher_.publish(e_stop_msg);
      if(velocity>1&&velocity<5)
        step = 0.5;
      if(velocity>5)
        step = 1;
      velocity = velocity + step;
      test_velocity.push_back(velocity);
      double delay_time = 2*M_PI/velocity;
      ros::Duration delay(delay_time);
      joint_velocity_command.data[0] = velocity;

      jointVelocityCommandPublisher_.publish(joint_velocity_command);
      ROS_INFO("Run %f seconds With Velocity : %f ",delay_time, velocity);
      delay.sleep();
      velocity_change_flag = true;
      if(velocity>=10)
        {
          std::cout<<"Velocity  || Friction Torque"<<std::endl;
          outfile<<"Velocity, Friction Torque\r\n";
          for(int i = 0; i<avarage_torque.size();i++)
          {
            std::cout<<test_velocity[i]<<"  ||  "<<avarage_torque[i]<<std::endl;
            outfile<<test_velocity[i]<<", "<<avarage_torque[i]<<"\r\n";
          }
          outfile.close();
          break;
        }
    }




  return 0;
}
