
#ifndef __TESTROBOT_DESCRIPTION_PLUGINS_UNDERWATEROBJECTROSPLUGIN_HH__
#define __TESTROBOT_DESCRIPTION_PLUGINS_UNDERWATEROBJECTROSPLUGIN_HH__

#include <string>

#include <testrobot_description_plugins/UnderwaterObjectPlugin.hh>

#include <boost/scoped_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <testrobot_msg_stack/SetUseGlobalCurrentVel.h>
#include <testrobot_msg_stack/UnderwaterObjectModel.h>
#include <testrobot_msg_stack/GetModelProperties.h>

#include <map>

namespace testrobot_simulator_ros
{
  class UnderwaterObjectROSPlugin : public gazebo::UnderwaterObjectPlugin
  {
    /// \brief Constructor
    public: UnderwaterObjectROSPlugin();

    /// \brief Destructor
    public: virtual ~UnderwaterObjectROSPlugin();

    /// \brief Load module and read parameters from SDF.
    public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Initialize Module.
    public: virtual void Init();

    /// \brief Reset Module.
    public: virtual void Reset();

    /// \brief Update the local current velocity, this data will be used only
    /// if the useGlobalCurrent flag is set to false.
    public: void UpdateLocalCurrentVelocity(
      const geometry_msgs::Vector3::ConstPtr &_msg);

    /// \brief Set the dynamic state efficiency factor
    public: bool SetUseGlobalCurrentVel(
      testrobot_msg_stack::SetUseGlobalCurrentVel::Request& _req,
      testrobot_msg_stack::SetUseGlobalCurrentVel::Response& _res);

    public: bool GetModelProperties(
      testrobot_msg_stack::GetModelProperties::Request& _req,
      testrobot_msg_stack::GetModelProperties::Response& _res);

    /// \brief Publish restoring force
    /// \param[in] _link Pointer to the link where the force information will
    /// be extracted from
    protected: virtual void PublishRestoringForce(
      gazebo::physics::LinkPtr _link);

    /// \brief Publish hydrodynamic wrenches
    /// \param[in] _link Pointer to the link where the force information will
    /// be extracted from
    protected: virtual void PublishHydrodynamicWrenches(
      gazebo::physics::LinkPtr _link);

    /// \brief Returns the wrench message for debugging topics
    /// \param[in] _force Force vector
    /// \param[in] _torque Torque vector
    /// \param[in] _output Stamped wrench message to be updated
    protected: virtual void GenWrenchMsg(
      gazebo::math::Vector3 _force, gazebo::math::Vector3 _torque,
      geometry_msgs::WrenchStamped &_output);

    /// \brief Sets the topics used for publishing the intermediate data during
    /// the simulation
    /// \param[in] _link Pointer to the link
    /// \param[in] _hydro Pointer to the hydrodynamic model
    protected: virtual void InitDebug(gazebo::physics::LinkPtr _link,
      gazebo::HydrodynamicModelPtr _hydro);

    /// \brief Publishes the current velocity marker
    protected: virtual void PublishCurrentVelocityMarker();

    /// \brief Publishes the state of the vehicle (is submerged)
    protected: virtual void PublishIsSubmerged();

    /// \brief Pointer to this ROS node's handle.
    private: boost::scoped_ptr<ros::NodeHandle> rosNode;

    /// \brief Subscriber to locally calculated current velocity
    private: ros::Subscriber subLocalCurVel;

    /// \brief Publisher for current actual thrust.
    private: std::map<std::string, ros::Publisher> rosHydroPub;

    /// \brief Map of services
    private: std::map<std::string, ros::ServiceServer> services;
  };
}

#endif  // __UNDERWATER_OBJECT_ROS_PLUGIN_HH__
