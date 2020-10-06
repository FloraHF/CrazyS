
#ifndef CRAZYFLIE_2_HEADING_ALTITUDE_CONTROLLER_H
#define CRAZYFLIE_2_HEADING_ALTITUDE_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
// #include <std_msgs/String.h>

#include <string>

#include <ros/time.h>

#include "common.h"
#include "parameters.h"
#include "stabilizer_types.h"
// #include "crazyflie_complementary_filter.h"
// #include "crazyflie_onboard_controller.h"
// #include "sensfusion6.h"
#include "controller_parameters.h"

#include <time.h>

#define NOT_ACTIVE                             -1  /* controller not active */
#define WAYPOINT_CONTROL                        0  /* When active, follow waypoint */
#define HEADING_ALTITUDE_CONTROL                1  /* When active, track attitude and altittude */

using namespace std;

namespace rotors_control {

    class HeadingAltitudeController{
        public:
            HeadingAltitudeController();
            ~HeadingAltitudeController();

            void SetPlayerID(const std::string& id);
            void SetResID(const std::string& res_id);

            void CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities);

            int GetControllerStatus();
            void SetControllerStatus(const std::string& status);
            void SetStateFromOdometry(const EigenOdometry& odometry);
            void SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory);
            void SetControllerGains();
            void InitiateDataSaving();

            PositionControllerParameters controller_parameters_;

            int status_;

            //data saving
            std::string user_;
            std::string id_;
            double dataStoringTime_;
            bool dataStoring_active_;

        private:
            std::string res_id_;

            EigenOdometry odometry_;
            mav_msgs::EigenTrajectoryPoint command_trajectory_; // command trajectory
            state_t state_;                                     // state feedback
            double t_;                                          // current time, computed from odometry

            // Callbacks
            ros::NodeHandle n_;
            ros::Timer timer_;
            void CallbackSaveData(const ros::TimerEvent& event);

            //data saving: state
            std::vector<string> listState_; // state: x, y, z, u, v, w, roll, pitch, yaw, p, q, r
            //data saving: commands 
            std::vector<string> listCommand_; // command: x, y, z, vx, vy, vz
            std::vector<string> listOmega_;
            std::vector<string> listPWM_;
            std::vector<string> listWrench_;
            std::vector<string> listCommandPQR_;
            std::vector<string> listCommandAttitude_;
            std::vector<string> listCommandYaw_;
            std::vector<string> listCommandUV_;

            //Integrator initial conditions
            double theta_command_ki_;
            double phi_command_ki_;
            double p_command_ki_;
            double q_command_ki_;
            double delta_phi_ki_;
            double delta_theta_ki_;
            double delta_psi_ki_;
            double r_command_ki_;
            double delta_omega_ki_;
            // double yaw_gain_ki_;

            // mixer parameter
            double rotor_arm_;
            Eigen::Matrix4f mixer_matrix_;
            double motor_constant_; 
            Eigen::Vector4f desired_wrench_;
            Eigen::Vector4f desired_rotor_lift_;
            Eigen::Vector4f PWM_sq_;

            // controller gains
            double drone_weight_;
            Eigen::Vector2f xy_gain_kp_, xy_gain_ki_;
            Eigen::Vector2f vxy_gain_kp_, vxy_gain_ki_;
            Eigen::Vector2f attitude_gain_kp_, attitude_gain_ki_;
            Eigen::Vector3f rate_gain_kp_, rate_gain_ki_, rate_gain_kd_; 
            double yaw_gain_kp_, yaw_gain_ki_;
            double hovering_gain_kp_, hovering_gain_ki_, hovering_gain_kd_;

            // error of previous step (for derivative term in pid)
            double p_error_prev_, q_error_prev_, r_error_prev_;

            void RateController(double* delta_phi, double* delta_theta, double* delta_psi);
            void AttitudeController(double* p_command, double* q_command);
            void ErrorBodyFrame(double* xe, double* ye) const;
            void HoveringController(double* delta_omega);
            void YawPositionController(double* r_command);
            void VelocityController(double* theta_command, double* phi_command);
            void XYController(double* vx_command, double* vy_command);
            void ControlMixer(double* PWM_1, double* PWM_2, double* PWM_3, double* PWM_4);
            void Quaternion2Euler(double* roll, double* pitch, double* yaw) const;
    };

}
#endif // CRAZYFLIE_2_HEADING_ALTITUDE_CONTROLLER_H