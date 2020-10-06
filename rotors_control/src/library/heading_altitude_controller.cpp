

#include "rotors_control/heading_altitude_controller.h"
#include "rotors_control/transform_datatypes.h"
#include "rotors_control/Matrix3x3.h"
#include "rotors_control/Quaternion.h"
#include "rotors_control/stabilizer_types.h"
// #include "rotors_control/sensfusion6.h"

#include <math.h>
#include <ros/ros.h>
#include <time.h>
#include <chrono>
#include <sys/stat.h>
#include <stdio.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iterator>

#include <nav_msgs/Odometry.h>
#include <ros/console.h>

// #define M_PI                                     3.14159265358979323846  /* pi [rad]*/
#define OMEGA_OFFSET                             6874  /* OMEGA OFFSET [PWM]*/
#define ANGULAR_MOTOR_COEFFICIENT                0.2685 /* ANGULAR_MOTOR_COEFFICIENT */
#define MOTORS_INTERCEPT                         426.24 /* MOTORS_INTERCEPT [rad/s]*/
#define MAX_PROPELLERS_ANGULAR_VELOCITY          2618 /* MAX PROPELLERS ANGULAR VELOCITY [rad/s]*/
#define MAX_R_DESIDERED                          3.4907 /* MAX R DESIDERED VALUE [rad/s]*/
#define MAX_THETA_COMMAND                        0.5236 /* MAX THETA COMMMAND [rad]*/
#define MAX_PHI_COMMAND                          0.5236 /* MAX PHI COMMAND [rad]*/
#define MAX_POS_DELTA_OMEGA                      1289 /* MAX POSITIVE DELTA OMEGA [PWM]*/
#define MAX_NEG_DELTA_OMEGA                      -1718 /* MAX NEGATIVE DELTA OMEGA [PWM]*/
#define SAMPLING_TIME                            0.01 /* SAMPLING TIME [s] */

// #define WAYPOINT_CONTROL                        0  /* When active, follow waypoint */
// #define HEADING_ALTITUDE_CONTROL                1  /* When active, track attitude and altittude */

namespace rotors_control{

HeadingAltitudeController::HeadingAltitudeController()
	 :theta_command_ki_(0),
    phi_command_ki_(0),
    yaw_gain_ki_(0),
    p_command_ki_(0),
    q_command_ki_(0),
    r_command_ki_(0),
    p_error_prev_(0),
    q_error_prev_(0),
    r_error_prev_(0),
    delta_phi_ki_(0),
    delta_theta_ki_(0),
    delta_psi_ki_(0),
    delta_omega_ki_(0),
    status_(NOT_ACTIVE),
    dataStoringTime_(0.01),
    dataStoring_active_(true),
    res_id_("0"){

		state_.angularAcc.x = 0; // Angular Acceleration x
		state_.angularAcc.y = 0; // Angular Acceleration y
		state_.angularAcc.z = 0; // Angular Acceleration z

		state_.attitude.roll = 0; // Roll
		state_.attitude.pitch = 0; // Pitch
		state_.attitude.yaw = 0; // Yaw

		state_.position.x = 0; // Position.x
		state_.position.y = 0; // Position.y
		state_.position.z = 0; // Position.z

		state_.angularVelocity.x = 0; // Angular velocity x
		state_.angularVelocity.y = 0; // Angular velocity y
		state_.angularVelocity.z = 0; // Angular velocity z

		state_.linearVelocity.x = 0; //Linear velocity x
		state_.linearVelocity.y = 0; //Linear velocity y
		state_.linearVelocity.z = 0; //Linear velocity z

		state_.attitudeQuaternion.x = 0; // Quaternion x
		state_.attitudeQuaternion.y = 0; // Quaternion y
		state_.attitudeQuaternion.z = 0; // Quaternion z
		state_.attitudeQuaternion.w = 0; // Quaternion w

    desired_rotor_lift_.setZero();
    desired_wrench_.setZero();

}

HeadingAltitudeController::~HeadingAltitudeController() {}


void HeadingAltitudeController::SetPlayerID(const std::string& id){
  id_ = id;
}

void HeadingAltitudeController::SetResID(const std::string& res_id){
  res_id_ = res_id;
}

// Reading parameters come frame launch file
void HeadingAltitudeController::InitiateDataSaving(){

  if(!dataStoring_active_){
    return;
  }

    ofstream fileState;
    ofstream fileCommand;
    ofstream fileOmega;
    ofstream filePWM;
    ofstream fileWrench;
    ofstream fileCommandPQR;
    ofstream fileCommandAttitude;
    ofstream fileCommandYaw;
    ofstream fileCommandUV;

    fileState.open("/home/" + user_ + "/mdmi_data/" + "/" + res_id_ + "/" + id_ + "/State.csv", std::ios_base::trunc);
    fileCommand.open("/home/" + user_ + "/mdmi_data/" + "/" + res_id_ + "/" + id_ + "/Command.csv",  std::ios_base::trunc);
    fileOmega.open("/home/" + user_ + "/mdmi_data/" + "/" + res_id_ + "/" + id_ + "/Omega.csv",  std::ios_base::trunc);
    filePWM.open("/home/" + user_ + "/mdmi_data/" + "/" + res_id_ + "/" + id_ + "/PWM.csv",  std::ios_base::trunc);
    fileWrench.open("/home/" + user_ + "/mdmi_data/" + "/" + res_id_ + "/" + id_ + "/Wrench.csv",  std::ios_base::trunc);
    fileCommandPQR.open("/home/" + user_ + "/mdmi_data/" + "/" + res_id_ + "/" + id_ + "/CommandPQR.csv",  std::ios_base::trunc);       
    fileCommandAttitude.open("/home/" + user_ + "/mdmi_data/" + "/" + res_id_ + "/" + id_ + "/CommandAttitude.csv",  std::ios_base::trunc); 
    fileCommandYaw.open("/home/" + user_ + "/mdmi_data/" + "/" + res_id_ + "/" + id_ + "/CommandYaw.csv",  std::ios_base::trunc);
    fileCommandUV.open("/home/" + user_ + "/mdmi_data/" + "/" + res_id_ + "/" + id_ + "/CommandUV.csv",  std::ios_base::trunc);

    fileState << "t,x,y,z,vx,vy,vz,roll,pitch,yaw,p,q,r\n";
    fileCommand << "t,x,y,z,vx,vy,vz\n";
    fileOmega << "t,omega_1,omega_2,omega_3,omega_4\n";
    filePWM << "t,PWM_1,PWM_2,PWM_3,PWM_4\n";
    fileWrench << "t,x,y,z,thrust\n";
    fileCommandPQR << "t,p,q,r\n";
    fileCommandAttitude << "t,roll,pitch,yaw\n";
    fileCommandYaw << "t,yaw\n";
    fileCommandUV << "t,x,y\n";

    fileState.close();
    fileCommand.close();
    fileOmega.close();
    filePWM.close();
    fileWrench.close();
    fileCommandPQR.close();
    fileCommandAttitude.close();
    fileCommandYaw.close();
    fileCommandUV.close();

    timer_ = n_.createTimer(ros::Duration(dataStoringTime_), &HeadingAltitudeController::CallbackSaveData, this, false, true);

    listState_.clear();
    listCommand_.clear();
    listOmega_.clear();
    listPWM_.clear();
    listWrench_.clear();
    listCommandPQR_.clear();
    listCommandAttitude_.clear();
    listCommandYaw_.clear();
    listCommandUV_.clear();

}

//The callback saves data come from simulation into csv files
void HeadingAltitudeController::CallbackSaveData(const ros::TimerEvent& event){

      if(!dataStoring_active_){
         return;
      }

      ofstream fileState;
      ofstream fileCommand;
      ofstream fileOmega;
      ofstream filePWM;
      ofstream fileWrench;
      ofstream fileCommandPQR;
      ofstream fileCommandAttitude;
      ofstream fileCommandYaw;
      ofstream fileCommandUV;

      ROS_INFO_ONCE("CallbackSavaData function is working");
      fileState.open("/home/" + user_ + "/mdmi_data/" + "/" + res_id_ + "/" + id_ + "/State.csv", std::ios_base::app);
      fileCommand.open("/home/" + user_ + "/mdmi_data/" + "/" + res_id_ + "/" + id_ + "/Command.csv",  std::ios_base::app);
      fileOmega.open("/home/" + user_ + "/mdmi_data/" + "/" + res_id_ + "/" + id_ + "/Omega.csv",  std::ios_base::app);
      filePWM.open("/home/" + user_ + "/mdmi_data/" + "/" + res_id_ + "/" + id_ + "/PWM.csv",  std::ios_base::app);
      fileWrench.open("/home/" + user_ + "/mdmi_data/" + "/" + res_id_ + "/" + id_ + "/Wrench.csv",  std::ios_base::app);
      fileCommandPQR.open("/home/" + user_ + "/mdmi_data/" + "/" + res_id_ + "/" + id_ + "/CommandPQR.csv",  std::ios_base::app);       
      fileCommandAttitude.open("/home/" + user_ + "/mdmi_data/" + "/" + res_id_ + "/" + id_ + "/CommandAttitude.csv",  std::ios_base::app); 
      fileCommandYaw.open("/home/" + user_ + "/mdmi_data/" + "/" + res_id_ + "/" + id_ + "/CommandYaw.csv",  std::ios_base::app);
      fileCommandUV.open("/home/" + user_ + "/mdmi_data/" + "/" + res_id_ + "/" + id_ + "/CommandUV.csv",  std::ios_base::app);

      // Saving control signals in a file
      for (unsigned n=0; n < listState_.size(); ++n) fileState << listState_.at( n );
      listState_.clear();

      for (unsigned n=0; n < listCommand_.size(); ++n) fileCommand << listCommand_.at( n ); 
      listCommand_.clear();

      for (unsigned n=0; n < listOmega_.size(); ++n) fileOmega << listOmega_.at( n );
      listOmega_.clear();

      for (unsigned n=0; n < listPWM_.size(); ++n) filePWM << listPWM_.at( n );
      listPWM_.clear();

      for (unsigned n=0; n < listWrench_.size(); ++n) fileWrench << listWrench_.at( n );
      listWrench_.clear();

      for (unsigned n=0; n < listCommandPQR_.size(); ++n) fileCommandPQR << listCommandPQR_.at( n );
      listCommandPQR_.clear();

      for (unsigned n=0; n < listCommandAttitude_.size(); ++n) fileCommandAttitude << listCommandAttitude_.at( n );
      listCommandAttitude_.clear();

      for (unsigned n=0; n < listCommandYaw_.size(); ++n) fileCommandYaw << listCommandYaw_.at( n );
      listCommandYaw_.clear();

      for (unsigned n=0; n < listCommandUV_.size(); ++n) fileCommandUV << listCommandUV_.at( n );
      listCommandUV_.clear();

      fileState.close();
      fileCommand.close();
      fileOmega.close();
      filePWM.close();
      fileWrench.close();
      fileCommandPQR.close();
      fileCommandAttitude.close();
      fileCommandYaw.close();
      fileCommandUV.close();

    }

void HeadingAltitudeController::SetControllerGains(){

      // ROS_INFO("controller gains");

      xy_gain_kp_ = Eigen::Vector2f(controller_parameters_.xy_gain_kp_.x(), controller_parameters_.xy_gain_kp_.y());
      xy_gain_ki_ = Eigen::Vector2f(controller_parameters_.xy_gain_ki_.x(), controller_parameters_.xy_gain_ki_.y());

      vxy_gain_kp_ = Eigen::Vector2f(controller_parameters_.vxy_gain_kp_.x(), controller_parameters_.vxy_gain_kp_.y());
      vxy_gain_ki_ = Eigen::Vector2f(controller_parameters_.vxy_gain_ki_.x(), controller_parameters_.vxy_gain_ki_.y());

      attitude_gain_kp_ = Eigen::Vector2f(controller_parameters_.attitude_gain_kp_.x(), controller_parameters_.attitude_gain_kp_.y());
      attitude_gain_ki_ = Eigen::Vector2f(controller_parameters_.attitude_gain_ki_.x(), controller_parameters_.attitude_gain_ki_.y());

      rate_gain_kp_ = Eigen::Vector3f(controller_parameters_.rate_gain_kp_.x(), controller_parameters_.rate_gain_kp_.y(), controller_parameters_.rate_gain_kp_.z());
      rate_gain_ki_ = Eigen::Vector3f(controller_parameters_.rate_gain_ki_.x(), controller_parameters_.rate_gain_ki_.y(), controller_parameters_.rate_gain_ki_.z());
      rate_gain_kd_ = Eigen::Vector3f(controller_parameters_.rate_gain_kd_.x(), controller_parameters_.rate_gain_kd_.y(), controller_parameters_.rate_gain_kd_.z());

      yaw_gain_kp_ = controller_parameters_.yaw_gain_kp_;
      yaw_gain_ki_ = controller_parameters_.yaw_gain_ki_;

      hovering_gain_kp_ = controller_parameters_.hovering_gain_kp_;
      hovering_gain_ki_ = controller_parameters_.hovering_gain_ki_;
      hovering_gain_kd_ = controller_parameters_.hovering_gain_kd_;

      motor_constant_ = controller_parameters_.motor_constant_;
      rotor_arm_      = controller_parameters_.rc_;
      mixer_matrix_   = controller_parameters_.mixer_matrix_;
      drone_weight_   = controller_parameters_.drone_mass_*9.81;
}

void HeadingAltitudeController::SetControllerStatus(const std::string& status) {
  // ROS_INFO("set controller status callback");
  // cout << "set controller status callback";
  if (status == "acceleration_altitude"){
    status_ = HEADING_ALTITUDE_CONTROL;
  }
  if (status == "waypoint"){
    status_ = WAYPOINT_CONTROL;
  }
  ROS_INFO("[Heading Altitude Controller] set controller status as: %d", status_);
}

int HeadingAltitudeController::GetControllerStatus(){
  return status_;
}

void HeadingAltitudeController::SetStateFromOdometry(const EigenOdometry& odometry) {

    odometry_ = odometry;

    double x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r;
    t_ = odometry_.timeStampSec + odometry_.timeStampNsec/1e9;

    x = state_.position.x = odometry.position[0];
    y = state_.position.y = odometry.position[1];
    z = state_.position.z = odometry.position[2];

    vx = state_.linearVelocity.x = odometry.velocity[0];
    vy = state_.linearVelocity.y = odometry.velocity[1];
    vz = state_.linearVelocity.z = odometry.velocity[2];

    state_.attitudeQuaternion.x = odometry.orientation.x();
    state_.attitudeQuaternion.y = odometry.orientation.y();
    state_.attitudeQuaternion.z = odometry.orientation.z();
    state_.attitudeQuaternion.w = odometry.orientation.w();

    p = state_.angularVelocity.x = odometry.angular_velocity[0];
    q = state_.angularVelocity.y = odometry.angular_velocity[1];
    r = state_.angularVelocity.z = odometry.angular_velocity[2];

    Quaternion2Euler(&roll, &pitch, &yaw);

    if (dataStoring_active_)
    {
      std::stringstream tempState;
      tempState << t_ << "," << x << "," << y << "," << z << "," << vx << "," << vy << "," << vz << ",";
      tempState << roll << "," << pitch << "," << yaw << "," << p << "," << q << "," << r << "\n";
      listState_.push_back(tempState.str());
    }

}


void HeadingAltitudeController::SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
    command_trajectory_= command_trajectory; //currently, only position, yaw and acceleration is used 
                                             //acceleration is only used when status_ == HEADING_ALTITUDE_CONTROL
    double x, y, z, vx, vy, vz;

    x = command_trajectory_.position_W[0];
    y = command_trajectory_.position_W[1];
    z = command_trajectory_.position_W[2];

    vx = command_trajectory_.velocity_W[0];
    vy = command_trajectory_.velocity_W[1];
    vz = command_trajectory_.velocity_W[2];

    if (dataStoring_active_)
    {
      std::stringstream tempCommand;
      tempCommand << t_ << "," << x << "," << y << "," << z << "," << vx << "," << vy << "," << vz << "\n";
      listCommand_.push_back(tempCommand.str());
    }    
}


void HeadingAltitudeController::CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities) {
    assert(rotor_velocities);

    double PWM_1, PWM_2, PWM_3, PWM_4;
    ControlMixer(&PWM_1, &PWM_2, &PWM_3, &PWM_4);

    double omega_1, omega_2, omega_3, omega_4;
    omega_1 = ((PWM_1 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);
    omega_2 = ((PWM_2 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);
    omega_3 = ((PWM_3 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);
    omega_4 = ((PWM_4 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);

    //The omega values are saturated considering physical constraints of the system
    if(!(omega_1 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_1 > 0)){
        if(omega_1 > MAX_PROPELLERS_ANGULAR_VELOCITY){
           omega_1 = MAX_PROPELLERS_ANGULAR_VELOCITY;
        }
        else{
           omega_1 = 0;
        }
    }        

    if(!(omega_2 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_2 > 0)){
        if(omega_2 > MAX_PROPELLERS_ANGULAR_VELOCITY){
           omega_2 = MAX_PROPELLERS_ANGULAR_VELOCITY;
        }           
        else{
           omega_2 = 0;
        }
    }        

    if(!(omega_3 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_3 > 0)){
        if(omega_3 > MAX_PROPELLERS_ANGULAR_VELOCITY){
           omega_3 = MAX_PROPELLERS_ANGULAR_VELOCITY;
        }           
        else{
           omega_3 = 0;
        }
    }        

    if(!(omega_4 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_4 > 0)){
        if(omega_4 > MAX_PROPELLERS_ANGULAR_VELOCITY){
           omega_4 = MAX_PROPELLERS_ANGULAR_VELOCITY;
        }           
        else{
           omega_4 = 0;
        }
    }
       
    *rotor_velocities = Eigen::Vector4d(omega_1, omega_2, omega_3, omega_4);

    if (dataStoring_active_)
    {
      std::stringstream tempOmega;
      tempOmega << t_ << "," << omega_1 << "," << omega_2 << "," << omega_3 << "," << omega_4 << "\n";
      listOmega_.push_back(tempOmega.str());
    }

}

void HeadingAltitudeController::ControlMixer(double* PWM_1, double* PWM_2, double* PWM_3, double* PWM_4) {
    assert(PWM_1);
    assert(PWM_2);
    assert(PWM_3);
    assert(PWM_4);

    double thrust;
    HoveringController(&thrust);

    double delta_phi, delta_theta, delta_psi;
    RateController(&delta_phi, &delta_theta, &delta_psi);

    desired_wrench_(0) = delta_phi;
    desired_wrench_(1) = delta_theta;
    desired_wrench_(2) = delta_psi;
    desired_wrench_(3) = thrust;
    desired_rotor_lift_ = mixer_matrix_ * desired_wrench_;

    for(int i=0 ; i < 4 ; i++){
      if(desired_rotor_lift_(i)<0){
        desired_rotor_lift_(i) = 0;
      }
      PWM_sq_(i) = (sqrt(desired_rotor_lift_(i)/motor_constant_) - MOTORS_INTERCEPT)/ANGULAR_MOTOR_COEFFICIENT;
      if(PWM_sq_(i)<0.0){
        PWM_sq_(i) = 0.0;
      }
    }

    *PWM_1 = PWM_sq_(0);
    *PWM_2 = PWM_sq_(1);
    *PWM_3 = PWM_sq_(2);
    *PWM_4 = PWM_sq_(3); 

    if (dataStoring_active_)
    {
      std::stringstream tempWrench;
      tempWrench << t_ << "," << desired_wrench_(0) << "," << desired_wrench_(1) << "," << desired_wrench_(2) << "," << desired_wrench_(3) << "\n";
      listWrench_.push_back(tempWrench.str());

      std::stringstream tempPWM;
      tempPWM << t_ << "," << *PWM_1 << "," << *PWM_2 << "," << *PWM_3 << "," << *PWM_4 << "\n";
      listPWM_.push_back(tempPWM.str());
    }   

}

void HeadingAltitudeController::HoveringController(double* thrust) {
    assert(thrust);

    double z_error, z_reference, dot_zeta;
    z_reference = command_trajectory_.position_W[2];
    z_error = z_reference - state_.position.z;

    // Velocity along z-axis from body to inertial frame
    double roll, pitch, yaw;
    Quaternion2Euler(&roll, &pitch, &yaw);

    // Needed because both angular and linear velocities are expressed in the aircraft body frame
     dot_zeta = -sin(pitch)*state_.linearVelocity.x + sin(roll)*cos(pitch)*state_.linearVelocity.y +
	            cos(roll)*cos(pitch)*state_.linearVelocity.z;

    double delta_omega, delta_omega_kp, delta_omega_kd;
    delta_omega_kp = hovering_gain_kp_ * z_error;
    delta_omega_ki_ = delta_omega_ki_ + (hovering_gain_ki_ * z_error * SAMPLING_TIME);
    delta_omega_kd = hovering_gain_kd_ * -dot_zeta;
    delta_omega = delta_omega_kp + delta_omega_ki_ + delta_omega_kd;

    *thrust = drone_weight_*(1.0 + delta_omega);

    // TODO: specify thresholds for delta_omega

    // if (dataStoring_active_)
    // {
    //   std::stringstream tempCommandZ;
    //   tempCommandZ << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << z_reference << "\n";
    //   listCommandZ_.push_back(tempCommandZ.str());

    //   std::stringstream tempZ;
    //   tempZ << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << state_.position.z << "\n";
    //   listZ_.push_back(tempZ.str());      
    // }

    //ROS_DEBUG("Delta_omega_kp: %f, Delta_omega_ki: %f, Delta_omega_kd: %f", delta_omega_kp, delta_omega_ki_, delta_omega_kd);
    //ROS_DEBUG("Z_error: %f, Delta_omega: %f", z_error, delta_omega);
    //ROS_DEBUG("Dot_zeta: %f", dot_zeta);
    //ROS_DEBUG("Omega: %f, delta_omega: %f", *omega, delta_omega);

}

void HeadingAltitudeController::RateController(double* delta_phi, double* delta_theta, double* delta_psi) {
    assert(delta_phi);
    assert(delta_theta);
    assert(delta_psi);

    double p, q, r;
    p = state_.angularVelocity.x;
    q = state_.angularVelocity.y;
    r = state_.angularVelocity.z;

    double p_command, q_command;
    AttitudeController(&p_command, &q_command);

    double r_command;
    YawPositionController(&r_command);

    double p_error, q_error, r_error;
    p_error = p_command - p;
    q_error = q_command - q;
    r_error = r_command - r;

    double delta_phi_kp, delta_theta_kp, delta_psi_kp, temp_delta_psi;
    double delta_phi_kd, delta_theta_kd, delta_psi_kd;
    delta_phi_kp = rate_gain_kp_.x() * p_error;
    delta_phi_ki_ = delta_phi_ki_ + (rate_gain_ki_.x() * p_error * SAMPLING_TIME);
    delta_phi_kd = rate_gain_kd_.x()*(p_error - p_error_prev_)/SAMPLING_TIME;
    *delta_phi = delta_phi_kp + delta_phi_ki_ + delta_phi_kd;

    delta_theta_kp = rate_gain_kp_.y() * q_error;
    delta_theta_ki_ = delta_theta_ki_ + (rate_gain_ki_.y() * q_error * SAMPLING_TIME);
    delta_theta_kd = rate_gain_kd_.y()*(q_error - q_error_prev_)/SAMPLING_TIME;
    *delta_theta = delta_theta_kp + delta_theta_ki_ + delta_theta_kd;

    delta_psi_kp = rate_gain_kp_.z() * r_error;
    delta_psi_ki_ = delta_psi_ki_ + (rate_gain_ki_.z() * r_error * SAMPLING_TIME);
    delta_psi_kd = rate_gain_kd_.z()*(r_error - r_error_prev_)/SAMPLING_TIME;
    temp_delta_psi = delta_psi_kp + delta_psi_ki_ + delta_psi_kd;

    p_error_prev_ = p_error;
    q_error_prev_ = q_error;
    r_error_prev_ = r_error;

    //TODO: satuation of delta_psi
    
    *delta_psi = temp_delta_psi;

    if (dataStoring_active_)
    {
      // std::stringstream tempPQ;
      // tempPQ << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << p << "," << q << "," << r << "\n";
      // listPQ_.push_back(tempPQ.str());

      std::stringstream tempCommandPQR;
      tempCommandPQR << t_ << ","  << p_command << "," << q_command << "," << r_command << "\n";
      listCommandPQR_.push_back(tempCommandPQR.str());
    }

}

void HeadingAltitudeController::YawPositionController(double* r_command) {
    assert(r_command);

    double roll, pitch, yaw;
    Quaternion2Euler(&roll, &pitch, &yaw);

    double yaw_error, yaw_reference;
    yaw_reference = command_trajectory_.getYaw();
    // yaw_reference = 0.0;
    yaw_error = yaw_reference - yaw;

    double r_command_kp;
    r_command_kp = yaw_gain_kp_ * yaw_error;
    r_command_ki_ = r_command_ki_ + (yaw_gain_ki_ * yaw_error * SAMPLING_TIME);
    *r_command = r_command_ki_ + r_command_kp;

   // R command value is saturated considering the aircraft physical constraints
    if(!(*r_command < MAX_R_DESIDERED && *r_command > -MAX_R_DESIDERED))
      if(*r_command > MAX_R_DESIDERED)
        *r_command = MAX_R_DESIDERED;
      else
        *r_command = -MAX_R_DESIDERED;

    if (dataStoring_active_)
    {
      // std::stringstream tempYaw;
      // tempYaw << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << yaw << "\n";
      // listYaw_.push_back(tempYaw.str());

      std::stringstream tempCommandYaw;
      tempCommandYaw << t_ << ","  << yaw_reference << "\n";
      listCommandYaw_.push_back(tempCommandYaw.str());
    }

}

void HeadingAltitudeController::AttitudeController(double* p_command, double* q_command) {
    assert(p_command);
    assert(q_command);

    double roll, pitch, yaw;
    Quaternion2Euler(&roll, &pitch, &yaw);

    // ROS_INFO("[heading altitude control] controller status: %d", status_);

    double theta_command, phi_command;
    // if (status_ == WAYPOINT_CONTROL)
    VelocityController(&theta_command, &phi_command);
    // theta_command = 0.1;
    // phi_command = 0.1;
    // if (status_ == HEADING_ALTITUDE_CONTROL){
    //  theta_command = command_trajectory_.acceleration_W.x();
    //  phi_command = -command_trajectory_.acceleration_W.y();
    //  // ROS_INFO("[heading altitude control] theta_command: %f, pitch: %f", theta_command, pitch);
    // }

    double phi_error, theta_error;
    phi_error = phi_command - roll;
    theta_error = theta_command - pitch;

    double p_command_kp, q_command_kp;
    p_command_kp = attitude_gain_kp_.x() * phi_error;
    p_command_ki_ = p_command_ki_ + (attitude_gain_ki_.x() * phi_error * SAMPLING_TIME);
    *p_command = p_command_kp + p_command_ki_;

    q_command_kp = attitude_gain_kp_.y() * theta_error;
    q_command_ki_ = q_command_ki_ + (attitude_gain_ki_.y() * theta_error * SAMPLING_TIME);
    *q_command = q_command_kp + q_command_ki_;

    if (dataStoring_active_)
    {
      // std::stringstream tempAttitude;
      // tempAttitude << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << roll << "," << pitch << "\n";
      // listAttitude_.push_back(tempAttitude.str());

      std::stringstream tempCommandAttitude;
      tempCommandAttitude << t_ << ","  << phi_command << "," << theta_command << "\n";
      listCommandAttitude_.push_back(tempCommandAttitude.str());
    }

}

void HeadingAltitudeController::VelocityController(double* theta_command, double* phi_command) {
    assert(theta_command);
    assert(phi_command);

    double v, u;
    u = state_.linearVelocity.x;
    v = state_.linearVelocity.y;

    double vx_command, vy_command;
    if (status_ == WAYPOINT_CONTROL)
      XYController(&vx_command, &vy_command);

    if (status_ == HEADING_ALTITUDE_CONTROL){
      double vx_e, vy_e, yaw, roll, pitch;
      Quaternion2Euler(&roll, &pitch, &yaw);

      vx_e = command_trajectory_.velocity_W.x();
      vy_e = command_trajectory_.velocity_W.y();
      vx_command = vx_e * cos(yaw) + vy_e * sin(yaw);
      vy_command = vy_e * cos(yaw) - vx_e * sin(yaw);
     // ROS_INFO("[heading altitude control] theta_command: %f, pitch: %f", theta_command, pitch);
    }

    double e_vx, e_vy;
    e_vx = vx_command - u;
    e_vy = vy_command - v;

    double theta_command_kp;
    theta_command_kp = vxy_gain_kp_.x() * e_vx;
    theta_command_ki_ = theta_command_ki_ + (vxy_gain_ki_.x() * e_vx * SAMPLING_TIME);
    *theta_command  = theta_command_kp + theta_command_ki_;

    double phi_command_kp;
    phi_command_kp = vxy_gain_kp_.y() * e_vy;
    phi_command_ki_ = phi_command_ki_ + (vxy_gain_ki_.y() * e_vy * SAMPLING_TIME);
    *phi_command  = phi_command_kp + phi_command_ki_;

    if (dataStoring_active_)
    {
      std::stringstream tempCommandUV;
      tempCommandUV << t_ << "," << vx_command << "," << vy_command << "\n";
      listCommandUV_.push_back(tempCommandUV.str());

      // std::stringstream tempBodyVXY;
      // tempBodyVXY << t_ << "," << u << "," << v << "\n";
      // listBodyVXY_.push_back(tempBodyVXY.str());
    }

    // Theta command is saturated considering the aircraft physical constraints
    if(!(*theta_command < MAX_THETA_COMMAND && *theta_command > -MAX_THETA_COMMAND))
       if(*theta_command > MAX_THETA_COMMAND)
          *theta_command = MAX_THETA_COMMAND;
       else
          *theta_command = -MAX_THETA_COMMAND;

    // Phi command is saturated considering the aircraft physical constraints
    if(!(*phi_command < MAX_PHI_COMMAND && *phi_command > -MAX_PHI_COMMAND))
       if(*phi_command > MAX_PHI_COMMAND)
          *phi_command = MAX_PHI_COMMAND;
       else
          *phi_command = -MAX_PHI_COMMAND;
}

void HeadingAltitudeController::XYController(double* vx_command, double* vy_command) {
    assert(vx_command);
    assert(vy_command);

    double xe, ye;
    ErrorBodyFrame(&xe, &ye);

    double vx_command_kp;
    vx_command_kp = xy_gain_kp_.x() * xe;
    *vx_command  = vx_command_kp;

    double vy_command_kp;
    vy_command_kp = xy_gain_kp_.y() * ye;
    *vy_command  = vy_command_kp;

    // below is used for controller gain tuning
    // if (dataStoring_active_)
    // {
    //   std::stringstream tempXY;
    //   tempXY << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << xe << "," << ye << "\n";
    //   listXY_.push_back(tempXY.str());
    // }    
}

void HeadingAltitudeController::ErrorBodyFrame(double* xe, double* ye) const {
    assert(xe);
    assert(ye);

    // X and Y reference coordinates
    double x_r = command_trajectory_.position_W[0];
    double y_r = command_trajectory_.position_W[1];

    // Position error
    double x_error_, y_error_;
    x_error_ = x_r - state_.position.x;
    y_error_ = y_r - state_.position.y;

    // The aircraft attitude (estimated or not, it depends by the employed controller)
    double yaw, roll, pitch;
    Quaternion2Euler(&roll, &pitch, &yaw);

    // Tracking error in the body frame
    *xe = x_error_ * cos(yaw) + y_error_ * sin(yaw);
    *ye = y_error_ * cos(yaw) - x_error_ * sin(yaw);

}

void HeadingAltitudeController::Quaternion2Euler(double* roll, double* pitch, double* yaw) const {
    assert(roll);
    assert(pitch);
    assert(yaw);

    // The estimated quaternion values
    double x, y, z, w;
    x = state_.attitudeQuaternion.x;
    y = state_.attitudeQuaternion.y;
    z = state_.attitudeQuaternion.z;
    w = state_.attitudeQuaternion.w;

    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    m.getRPY(*roll, *pitch, *yaw);

    ROS_DEBUG("Roll: %f, Pitch: %f, Yaw: %f", *roll, *pitch, *yaw);

}

}