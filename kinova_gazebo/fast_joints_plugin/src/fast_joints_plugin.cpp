#include "fast_joints_plugin.hpp"

namespace fast_joints_plugin {

FastJointsPlugin::FastJointsPlugin() {
}

FastJointsPlugin::~FastJointsPlugin() {
}

void FastJointsPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  // Initialize the plugin
  // Subscribe to topics, retrieve parameters, etc.

  // Attach the Update function to the simulation update event
  updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&FastJointsPlugin::Update, this, _1));

  // Initialize the ROS node and advertise the joint states publisher
  ros::NodeHandle nh;
  jointStatesPublisher = nh.advertise<sensor_msgs::JointState>("fast_joint_states", 1);

  // Set the publishing frequency
  publishingFrequency = 500.0;  // Set the desired publishing frequency in Hz
  publishingPeriod = 1.0 / publishingFrequency;
  lastPublishTime = ros::Time::now();
  this->model = model;
}

void FastJointsPlugin::Update(const gazebo::common::UpdateInfo& info) {
  // Perform actions on each simulation step
  // Publish joint states at the desired frequency

  // Check if it's time to publish the joint states
  ros::Time currentTime = ros::Time::now();
  if ((currentTime - lastPublishTime).toSec() >= publishingPeriod) {
    // Retrieve joint states
    gazebo::physics::Joint_V joints = model->GetJoints();

    // Create a JointState message
    sensor_msgs::JointState jointStatesMsg;
    jointStatesMsg.header.stamp = currentTime;

    for (const auto& joint : joints) {
      // Retrieve joint state information
      std::string jointName = joint->GetName();
      double position = joint->Position(0);
      double velocity = joint->GetVelocity(0);
      double effort = joint->GetForce(0);

      // Add joint state to the JointState message
      jointStatesMsg.name.push_back(jointName);
      jointStatesMsg.position.push_back(position);
      jointStatesMsg.velocity.push_back(velocity);
      jointStatesMsg.effort.push_back(effort);
    }

    // Publish the JointState message
    jointStatesPublisher.publish(jointStatesMsg);

    // Update the last publish time
    lastPublishTime = currentTime;
  }
}

GZ_REGISTER_MODEL_PLUGIN(FastJointsPlugin)

}  // namespace fast_joints_plugin
