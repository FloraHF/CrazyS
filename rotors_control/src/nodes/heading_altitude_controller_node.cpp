
#include "heading_altitude_controller_node.h"

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <ros/console.h>

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

HeadingAltitudeControllerNode::HeadingAltitudeControllerNode() 
    :status_(NOT_ACTIVE){

    ROS_INFO_ONCE("Started heading altitude controller");
    InitializeParams();

    ros::NodeHandle nh;

    // subscribers
    status_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_CONTROLLERSTATUS, 1,
                                  &HeadingAltitudeControllerNode::ControllerStatusCallback, this);
    trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
                                  &HeadingAltitudeControllerNode::MultiDofJointTrajectoryCallback, this);
    odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                  &HeadingAltitudeControllerNode::OdometryCallback, this);

    // publisher
    motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1);
}

HeadingAltitudeControllerNode::~HeadingAltitudeControllerNode(){}

void HeadingAltitudeControllerNode::InitializeParams() {

  heading_altitude_controller_.SetControllerGains();
  ROS_INFO_ONCE("[Heading Altitude Controller] Set controller gains and vehicle parameters");

  ros::NodeHandle pnh("~");
  bool dataStoringActive;
  double dataStoringTime;
  std::string user;

  if (pnh.getParam("playerID", id_)){
    ROS_INFO("Got param 'player ID': %s", id_.c_str());
    heading_altitude_controller_.SetPlayerID(id_);
  }
  else
     ROS_ERROR("Failed to get param 'playerID'");

  if (pnh.getParam("resID", res_id_)){
    ROS_INFO("Got param 'player ID': %s", res_id_.c_str());
    heading_altitude_controller_.SetResID(res_id_);
  }
  else
     ROS_ERROR("Failed to get param 'resID'");


  if (pnh.getParam("user_account", user)){
    ROS_INFO("Got param 'user_account': %s", user.c_str());
    heading_altitude_controller_.user_ = user;
  }
  else
     ROS_ERROR("Failed to get param 'user'");

  if (pnh.getParam("csvFilesStoring", dataStoringActive)){
    ROS_INFO("Got param 'csvFilesStoring': %d", dataStoringActive);
    heading_altitude_controller_.dataStoring_active_ = dataStoringActive;
  }
  else
     ROS_ERROR("Failed to get param 'csvFilesStoring'");

  if (pnh.getParam("csvFilesStoringTime", dataStoringTime)){
    ROS_INFO("Got param 'csvFilesStoringTime': %f", dataStoringTime);
    heading_altitude_controller_.dataStoringTime_ = dataStoringTime;
  }
  else
     ROS_ERROR("Failed to get param 'csvFilesStoringTime'");

  // heading_altitude_controller_.InitiateDataSaving();
  
}

void HeadingAltitudeControllerNode::ControllerStatusCallback(const std_msgs::StringConstPtr& status_msg) {
  heading_altitude_controller_.SetControllerStatus(status_msg->data.c_str());

  // initialize data saving after reveive first controller status message
  if (!heading_altitude_controller_.dataStoring_active_){
    if (heading_altitude_controller_.status_ == HEADING_ALTITUDE_CONTROL){
      heading_altitude_controller_.dataStoring_active_ = true;
    }
  }
  heading_altitude_controller_.InitiateDataSaving();

  // if (status_msg->data.c_str() != "not_active")
  // {
  //   std::cout << "active" << "\n";
  //   if (status_msg->data.c_str() == "acceleration_altitude"){
  //     status_ = HEADING_ALTITUDE_CONTROL;
  //     std::cout << "set status as heading altitude control " << status_msg->data.c_str() << "\n";
  //   }
  //   if (status_msg->data.c_str() == "waypoint"){
  //     status_ = WAYPOINT_CONTROL;
  //     std::cout << "set status as waypoint " << status_msg->data.c_str() << "\n";
  //   }
  // }
  // else
  //   std::cout << "not active" << "\n";
  //   status_ = NOT_ACTIVE;

}

void HeadingAltitudeControllerNode::MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);

  heading_altitude_controller_.SetTrajectoryPoint(eigen_reference);

}


void HeadingAltitudeControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

      EigenOdometry odometry;
      eigenOdometryFromMsg(odometry_msg, &odometry);
      heading_altitude_controller_.SetStateFromOdometry(odometry);

      // ROS_INFO("controller node status: %d", status_);

      if (heading_altitude_controller_.GetControllerStatus() != NOT_ACTIVE)
      {
        Eigen::Vector4d ref_rotor_velocities;
        heading_altitude_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

        mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
        actuator_msg->angular_velocities.clear();
        for (int i = 0; i < ref_rotor_velocities.size(); i++){
           actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
        }

        actuator_msg->header.stamp = odometry_msg->header.stamp;

        motor_velocity_reference_pub_.publish(actuator_msg);
      }
}

}

int main(int argc, char** argv){
    ros::init(argc, argv, "heading_altitude_controller");

    ros::NodeHandle nh2;

    rotors_control::HeadingAltitudeControllerNode heading_altitude_controller_node;

    ros::spin();

    return 0;
}