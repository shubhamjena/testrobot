#ifndef __TESTROBOT_DESCRIPTION_PLUGINS_THRUSTERPLUGIN_HH__
#define __TESTROBOT_DESCRIPTION_PLUGINS_THRUSTERPLUGIN_HH__
	
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "testrobot_msg_stack/thrusterData6.h"
#include "testrobot_msg_stack/forceTorqueData.h"
#include "std_msgs/Float32.h"


 namespace gazebo
{ 
  class TPA : public ModelPlugin
  {
  
    public: TPA() {}

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {    
         
         this->model = _model;
         this->link  =_model->GetLinks();
                 
        /*this->link[1]  =_model->GetLinks()[1];//surge_left  
         this->link[2]  =_model->GetLinks()[2];//surge_right
         this->link[3]  =_model->GetLinks()[3];//sway_front
         std::cerr<<link[2]->GetScopedName();
	 this->link[4]  =_model->GetLinks()[4];//sway_back
	 this->link[5]  =_model->GetLinks()[5];//depth_back
	 this->link[6]  =_model->GetLinks()[6];//depth_front
         
         physics::LinkPtr linkedin;
         linkedin =_model->GetLinks()[0];
         std::cerr<<linkedin->GetScopedName();        
        */

        //********* Initialize ros, if it has not already bee initialized******
        
        if (!ros::isInitialized())
          {
	    int argc = 0;
	    char **argv = NULL;
	    ros::init(argc, argv, "gazebo_client",ros::init_options::NoSigintHandler);
            std::cerr<<"\nROS NODE WORKING\n";
	   }

          /* Create our ROS node 
             this->rosNode.reset(new ros::NodeHandle("gazebo_client")); 


          subscribe to the topic thruster_force
            //ros::NodeHandle n;
            this->rosSub = this->rosNode->subscribe("thruster_force",1,&TPA::OnRosMsg,this);
            ROS_INFO("subscribed to thruster_force topic");
            //ros::Subscriber _sub = .subscribe("thruster_force",1,); 
         
            */	

       // Create our ROS node
          this->rosNode.reset(new ros::NodeHandle("gazebo_client")); 

       // Create a named topic, and subscribe to it.
          ros::SubscribeOptions so = 
          ros::SubscribeOptions::create<testrobot_msg_stack::thrusterData6>
          ("thruster_force" ,1,boost::bind(&TPA::OnRosMsg,
           this, _1),ros::VoidPtr(), &this->rosQueue);
          
           this->rosSub = this->rosNode->subscribe(so);
       
       // Spin up the queue helper thread.
          this->rosQueueThread = std::thread(std::bind(&TPA::QueueThread, this));

   }//**************end of load function***********************

   
        
//*****************************************************

     private: physics::ModelPtr model;
     private: physics::Link_V  link;
     private: int thrusterCount = 6;
  
  // A node use for ROS transport
     private: std::unique_ptr<ros::NodeHandle> rosNode;

  // A ROS subscriber
     private: ros::Subscriber rosSub;

  // A ROS callbackqueue that helps process messages
     private: ros::CallbackQueue rosQueue;

  // A thread the keeps running the rosQueue
     private: std::thread rosQueueThread;
  
  // 
  
//********** functions to handle an incoming message from ROS***********

// \param[in] _msg 6*1 force matrix to apply force to thrusters
   
   public: void OnRosMsg(const testrobot_msg_stack::thrusterData6ConstPtr &msg)
   {     
     for( int i = 0;i<this->thrusterCount;i++)
     { 
         math::Vector3 force(-(msg->data[i]),0,0);       
         
         this->link[i+1]->AddLinkForce(force);
     }
    
    ROS_INFO("applied force to MODEL");  
   
   }

// ROS helper function that processes messages
   private: void QueueThread()
   {
     static const double timeout = 0.01;
     while (this->rosNode->ok())
    {
     this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
   }


 
  };//**********end of class*********************

  
// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(TPA)

}
#endif	
