#ifndef FAST_JOINTS_PLUGIN_FAST_JOINTS_PLUGIN_HPP
#define FAST_JOINTS_PLUGIN_FAST_JOINTS_PLUGIN_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh> // Add this header
#include <gazebo/physics/Joint.hh> // Add this header
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace fast_joints_plugin {

class FastJointsPlugin : public gazebo::ModelPlugin {
public:
  FastJointsPlugin();
  virtual ~FastJointsPlugin();

  // Gazebo's virtual function called when the plugin is loaded
  virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

  // Gazebo's virtual function called on each simulation step
  virtual void Update(const gazebo::common::UpdateInfo& info);

private:
  gazebo::event::ConnectionPtr updateConnection;
  ros::Publisher jointStatesPublisher;
  double publishingFrequency;
  double publishingPeriod;
  ros::Time lastPublishTime;
  gazebo::physics::ModelPtr model;
};

}  // namespace fast_joints_plugin

#endif  // FAST_JOINTS_PLUGIN_FAST_JOINTS_PLUGIN_HPP
