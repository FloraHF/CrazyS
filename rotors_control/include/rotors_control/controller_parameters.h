/*
 * Copyright 2018 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Emanuele Aucone, University of Sannio in Benevento, Italy
 * Copyright 2018 Benjamin Rodriguez, MIT, USA
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CONTROLLER_PARAMETERS_H
#define CONTROLLER_PARAMETERS_H
#include <math.h>
// Default values for the position controller of the Crazyflie2. XYController [x,y], AttitudeController [phi,theta] 
//RateController [p,q,r], YawController[yaw], HoveringController[z]
static const Eigen::Vector2f kPDefaultXYController = Eigen::Vector2f(.45, .45);
static const Eigen::Vector2f kIDefaultXYController = Eigen::Vector2f(0.0, 0.0);

static const Eigen::Vector2f kPDefaultVXYController = Eigen::Vector2f(.123, -.123);
static const Eigen::Vector2f kIDefaultVXYController = Eigen::Vector2f(0.0, 0.0);

static const Eigen::Vector2f kPDefaultAttitudeController = Eigen::Vector2f(3.8, 3.64);
static const Eigen::Vector2f kIDefaultAttitudeController = Eigen::Vector2f(0.0, 0.0);
// static const Eigen::Vector2f kPDefaultAttitudeController = Eigen::Vector2f(0.0611, 0.0611);
// static const Eigen::Vector2f kIDefaultAttitudeController = Eigen::Vector2f(0.0349, 0.0349);

static const Eigen::Vector3f kPDefaultRateController = Eigen::Vector3f(.0005, .0005, .00041);
static const Eigen::Vector3f kIDefaultRateController = Eigen::Vector3f(.0, .0, .0);
static const Eigen::Vector3f kDDefaultRateController = Eigen::Vector3f(.00000, .00000, .0);
// static const Eigen::Vector3f kPDefaultRateController = Eigen::Vector3f(1000, 1000, 1000);
// static const Eigen::Vector3f kIDefaultRateController = Eigen::Vector3f(0, 0, 95.6839);

static const double kPDefaultYawController = 2.;
static const double kIDefaultYawController = 0;

static const double kPDefaultHoveringController = 0.3;
static const double kIDefaultHoveringController = 0.03;
static const double kDDefaultHoveringController = 0.5;


namespace rotors_control {

	class PositionControllerParameters {
	 public:
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	  PositionControllerParameters()
	      : xy_gain_kp_(kPDefaultXYController), 
	        xy_gain_ki_(kIDefaultXYController),
	      vxy_gain_kp_(kPDefaultVXYController), 
	      vxy_gain_ki_(kIDefaultVXYController),
          attitude_gain_kp_(kPDefaultAttitudeController),
          attitude_gain_ki_(kIDefaultAttitudeController),
          rate_gain_kp_(kPDefaultRateController),
          rate_gain_ki_(kIDefaultRateController),
          rate_gain_kd_(kDDefaultRateController),
          yaw_gain_kp_(kPDefaultYawController),
          yaw_gain_ki_(kIDefaultYawController),
          hovering_gain_kp_(kPDefaultHoveringController),
          hovering_gain_ki_(kIDefaultHoveringController),
          hovering_gain_kd_(kDDefaultHoveringController),
          rc_(0.046) {
          double sqrt_2 = 1.0/sqrt(2.0);
       
          Eigen::Vector4f a4( sqrt_2*rc_, -sqrt_2*rc_,  5.964552*pow(10,-3), 1);
          Eigen::Vector4f a3( sqrt_2*rc_,  sqrt_2*rc_, -5.964552*pow(10,-3), 1);
          Eigen::Vector4f a2(-sqrt_2*rc_,  sqrt_2*rc_,  5.964552*pow(10,-3), 1);
          Eigen::Vector4f a1(-sqrt_2*rc_, -sqrt_2*rc_, -5.964552*pow(10,-3), 1);

          Eigen::Matrix4f A;
          A.setZero();
	      A.block<4,1>(0,0) = a1; 	
	      A.block<4,1>(0,1) = a2;
	      A.block<4,1>(0,2) = a3;
	      A.block<4,1>(0,3) = a4;
	      mixer_matrix_.setZero();
	      mixer_matrix_ = A.inverse();
	      motor_constant_ = 1.28192*pow(10,-8);
	      drone_mass_ = 0.027;
	  }

	  Eigen::Vector2f xy_gain_kp_;
	  Eigen::Vector2f xy_gain_ki_;

	  Eigen::Vector2f vxy_gain_kp_;
	  Eigen::Vector2f vxy_gain_ki_;
	  
	  Eigen::Vector2f attitude_gain_kp_;
	  Eigen::Vector2f attitude_gain_ki_;
	  
	  Eigen::Vector3f rate_gain_kp_;
	  Eigen::Vector3f rate_gain_ki_;
	  Eigen::Vector3f rate_gain_kd_;
	  
	  double yaw_gain_kp_;
	  double yaw_gain_ki_;
	  
	  double hovering_gain_kp_;
	  double hovering_gain_ki_;
	  double hovering_gain_kd_;

	  Eigen::Matrix4f mixer_matrix_;
	  double rc_;
	  double motor_constant_;
	  double drone_mass_;

	};

}

#endif // CONTROLLER_PARAMETERS_H
