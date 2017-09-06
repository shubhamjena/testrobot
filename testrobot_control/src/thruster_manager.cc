#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "testrobot_msg_stack/thrusterData6.h"
#include "testrobot_msg_stack/forceTorqueData.h"
#include "std_msgs/Float32.h"


 testrobot_msg_stack::thrusterData6 _output;

void cmd_force_torque_callback(const testrobot_msg_stack::forceTorqueDataConstPtr &msg)
{ 
  float f[6] = {0,0,0,0,0,0};
  float l1 = .4;               //dist_front_back_sway
  float l2 = .4;               // dist_front_back_heave 
  float f_max = 100000;          //max force deliverable by a thruster
  
  //testrobot_msg_stack::thrusterData6 _output;
  	
    f[0] = f[1] = msg->data[0]/2;             
    f[2] = (msg->data[1] + msg->data[5]*l1)/2;
    f[3] = (msg->data[1] - msg->data[5]*l1)/2;
    f[4] = (msg->data[2] + msg->data[4]*l2)/2;
    f[5] = (msg->data[2] - msg->data[4]*l2)/2;
  
 // scaling down forces by max force deliverable by a thruster
    
    for (int i= 0;i<5;i++)
         if(abs(f[i])> f_max)
            f[i] = f_max*(f[i])/abs(f[i]);
  
 //****logic is wrong. check matlab code of mobile robot

    
    for (int i= 0;i<5;i++)
        _output.data[i] = f[i]; 
 
}
//end of callback function




int main(int argc, char **argv)
{
  ros::init(argc, argv, "thruster_manager");

  ros::NodeHandle n1;
  
  ros::NodeHandle n2;

 // ros::Subscriber sub = n1.subscribe("cmd_force_torque",1, cmd_force_torque_callback); 
  
  //ROS_INFO("thruster_manager initialised");
  
  ros::Publisher _pub = n2.advertise<testrobot_msg_stack::thrusterData6>("gazebo_client/thruster_force", 1);
 
  ros::Rate looprate(10);
  
//debugging code.......................

        _output.data[0] = 10000 ;
        _output.data[1] = 10000 ;
        _output.data[2] = 0 ;
        _output.data[3] = 0 ;
        _output.data[4] = 0 ;
        _output.data[5] = 0 ;
         
//...........................   
 
 //while(ros::ok())
     {
        _pub.publish(_output);
   //     ros::spinOnce();
    //    looprate.sleep();
        ROS_INFO("published to thruster_force topic");
 
    }
  
    
  
  ros::spin();
     
  return 0;
}

