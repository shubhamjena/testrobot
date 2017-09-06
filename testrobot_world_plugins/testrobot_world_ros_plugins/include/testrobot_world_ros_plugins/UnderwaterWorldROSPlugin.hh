
#ifndef __CONSTANT_FLOW_ROS_PLUGIN_HH__
#define __CONSTANT_FLOW_ROS_PLUGIN_HH__

#include <map>
#include <string>

// Gazebo plugin
#include <testrobot_world_plugins/UnderwaterWorldPlugin.hh>

#include <boost/scoped_ptr.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <testrobot_msg_stack/SetCurrentModel.h>
#include <testrobot_msg_stack/GetCurrentModel.h>
#include <testrobot_msg_stack/SetCurrentVelocity.h>
#include <testrobot_msg_stack/SetCurrentDirection.h>
#include <testrobot_msg_stack/SetOriginSphericalCoord.h>
#include <testrobot_msg_stack/GetOriginSphericalCoord.h>

namespace testrobot_simulator_ros
{
  class UnderwaterWorldROSPlugin : public gazebo::UnderwaterWorldPlugin
  {
    /// \brief Class constructor
    public: UnderwaterWorldROSPlugin();

    /// \brief Class destructor
    public: virtual ~UnderwaterWorldROSPlugin();

    /// \brief Load module and read parameters from SDF.
    public: void Load(gazebo::physics::WorldPtr _world,
        sdf::ElementPtr _sdf);

    /// \brief Service call to update the parameters for the velocity
    /// Gauss-Markov process model
    public: bool UpdateCurrentVelocityModel(
        testrobot_msg_stack::SetCurrentModel::Request& _req,
        testrobot_msg_stack::SetCurrentModel::Response& _res);

    /// \brief Service call to update the parameters for the horizontal angle
    /// Gauss-Markov process model
    public: bool UpdateCurrentHorzAngleModel(
        testrobot_msg_stack::SetCurrentModel::Request& _req,
        testrobot_msg_stack::SetCurrentModel::Response& _res);

    /// \brief Service call to update the parameters for the vertical angle
    /// Gauss-Markov process model
    public: bool UpdateCurrentVertAngleModel(
        testrobot_msg_stack::SetCurrentModel::Request& _req,
        testrobot_msg_stack::SetCurrentModel::Response& _res);

    /// \brief Service call to read the parameters for the velocity
    /// Gauss-Markov process model
    public: bool GetCurrentVelocityModel(
        testrobot_msg_stack::GetCurrentModel::Request& _req,
        testrobot_msg_stack::GetCurrentModel::Response& _res);

    /// \brief Service call to read the parameters for the horizontal angle
    /// Gauss-Markov process model
    public: bool GetCurrentHorzAngleModel(
        testrobot_msg_stack::GetCurrentModel::Request& _req,
        testrobot_msg_stack::GetCurrentModel::Response& _res);

    /// \brief Service call to read the parameters for the vertical angle
    /// Gauss-Markov process model
    public: bool GetCurrentVertAngleModel(
        testrobot_msg_stack::GetCurrentModel::Request& _req,
        testrobot_msg_stack::GetCurrentModel::Response& _res);

    /// \brief Service call to update the mean value of the flow velocity
    public: bool UpdateCurrentVelocity(
        testrobot_msg_stack::SetCurrentVelocity::Request& _req,
        testrobot_msg_stack::SetCurrentVelocity::Response& _res);

    /// \brief Service call to update the mean value of the horizontal angle
    public: bool UpdateHorzAngle(
        testrobot_msg_stack::SetCurrentDirection::Request& _req,
        testrobot_msg_stack::SetCurrentDirection::Response& _res);

    /// \brief Service call to update the mean value of the vertical angle
    public: bool UpdateVertAngle(
        testrobot_msg_stack::SetCurrentDirection::Request& _req,
        testrobot_msg_stack::SetCurrentDirection::Response& _res);

    /// \brief Service call that returns the origin in WGS84 standard
    public: bool GetOriginSphericalCoord(
        testrobot_msg_stack::GetOriginSphericalCoord::Request& _req,
        testrobot_msg_stack::GetOriginSphericalCoord::Response& _res);

    /// \brief Service call that returns the origin in WGS84 standard
    public: bool SetOriginSphericalCoord(
        testrobot_msg_stack::SetOriginSphericalCoord::Request& _req,
        testrobot_msg_stack::SetOriginSphericalCoord::Response& _res);

    /// \brief Publishes ROS topics
    private: void PublishROSTopics();

    /// \brief All underwater world services
    private: std::map<std::string, ros::ServiceServer> worldServices;

    /// \brief Pointer to this ROS node's handle.
    private: boost::scoped_ptr<ros::NodeHandle> rosNode;

    /// \brief Connection for callbacks on update world.
    private: gazebo::event::ConnectionPtr rosPublishConnection;

    /// \brief Publisher for the flow velocity in the world frame
    private: ros::Publisher flowVelocityPub;

    /// \brief Period after which we should publish a message via ROS.
    private: gazebo::common::Time rosPublishPeriod;

    /// \brief Last time we published a message via ROS.
    private: gazebo::common::Time lastRosPublishTime;
  };
}

#endif  // __CONSTANT_FLOW_ROS_PLUGIN_HH__
