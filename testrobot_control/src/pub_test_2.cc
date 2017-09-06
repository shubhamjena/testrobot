#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "testrobot_msg_stack/thrusterData6.h"
#include "testrobot_msg_stack/forceTorqueData.h"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_tester_2");

  ros::NodeHandle n;

  ros::Publisher _pub = n.advertise<testrobot_msg_stack::forceTorqueData>("cmd_force_torque", 10);

  ros::Rate loop_rate(10);

  int count = 0;
   testrobot_msg_stack::forceTorqueData msg;
  

  while (ros::ok())
  {
    for (int i = 0;i<6;i++)
      //msg[i]=10*i;

    msg.data[0] = 10;
    msg.data[1] = 100000;
    msg.data[2] = 20000;
    msg.data[3] = 400000;
    msg.data[4] = 100000;
    msg.data[5] = 1700000;
    
    _pub.publish(msg);
    
   //ROS_INFO(" published data [%f]",msg.data[0]);
 
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}


