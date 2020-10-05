

#ifndef CRAYZFLIE_2_HEADING_ALTITUDE_CONTROLLER_NODE_H
#define CRAYZFLIE_2_HEADING_ALTITUDE_CONTROLLER_NODE_H

#include <string>

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/DroneState.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <ros/time.h>


#include "rotors_control/common.h"
#include "rotors_control/heading_altitude_controller.h"

#define NOT_ACTIVE                             -1  /* controller not active */
#define WAYPOINT_CONTROL                        0  /* When active, follow waypoint */
#define HEADING_ALTITUDE_CONTROL                1  /* When active, track attitude and altittude */

namespace rotors_control {

    class HeadingAltitudeControllerNode{
        
        public:
            HeadingAltitudeControllerNode();
            ~HeadingAltitudeControllerNode();

            void InitializeParams();

        private:
            //player id for data storing
            std::string id_;
            std::string res_id_;

            int status_;
            HeadingAltitudeController heading_altitude_controller_;

            //subscribers
            ros::Subscriber status_sub_;
            ros::Subscriber trajectory_sub_;
            ros::Subscriber odometry_sub_;

            //publisher
            ros::Publisher motor_velocity_reference_pub_;

            mav_msgs::EigenTrajectoryPointDeque commands_;
            std::deque<ros::Duration> command_waiting_times_;
            ros::Timer command_timer_;

            void ControllerStatusCallback(const std_msgs::StringConstPtr& status_msg);
            void MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);
            void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

    };
}

#endif // CRAYZFLIE_2_HEADING_ALTITUDE_CONTROLLER_NODE_H
