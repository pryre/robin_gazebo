/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#include "rotors/gazebo_motor_model.hpp"

namespace gazebo {

GazeboMotorModel::~GazeboMotorModel() {
	this->updateConnection_.reset();
	use_pid_ = false;
}

void GazeboMotorModel::InitializeParams() {
}

void GazeboMotorModel::Publish() {
	turning_velocity_msg_.data = joint_->GetVelocity( 0 );
	pub_motor_velocity_.publish( turning_velocity_msg_ );
}

void GazeboMotorModel::Load( physics::ModelPtr _model, sdf::ElementPtr _sdf ) {
	// Make sure the ROS node for Gazebo has already been initialized
	if ( !ros::isInitialized() ) {
		ROS_FATAL( "A ROS node for Gazebo has not been initialized, unable to load "
				   "plugin." );
		return;
	}

	model_ = _model;
	namespace_.clear();

	if ( _sdf->HasElement( "robotNamespace" ) )
		namespace_ = _sdf->GetElement( "robotNamespace" )->Get<std::string>();
	else
		gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";

	if ( _sdf->HasElement( "jointName" ) )
		joint_name_ = _sdf->GetElement( "jointName" )->Get<std::string>();
	else
		gzerr << "[gazebo_motor_model] Please specify a jointName, where the rotor "
				 "is attached.\n";
	// Get the pointer to the joint.
	joint_ = model_->GetJoint( joint_name_ );
	if ( joint_ == NULL )
		gzthrow( "[gazebo_motor_model] Couldn't find specified joint \""
			<< joint_name_ << "\"." );

	// setup joint control pid to control joint
	if ( _sdf->HasElement( "joint_control_pid" ) ) {
		sdf::ElementPtr pid = _sdf->GetElement( "joint_control_pid" );
		double p = 0.1;
		if ( pid->HasElement( "p" ) )
			p = pid->Get<double>( "p" );
		double i = 0;
		if ( pid->HasElement( "i" ) )
			i = pid->Get<double>( "i" );
		double d = 0;
		if ( pid->HasElement( "d" ) )
			d = pid->Get<double>( "d" );
		double iMax = 0;
		if ( pid->HasElement( "iMax" ) )
			iMax = pid->Get<double>( "iMax" );
		double iMin = 0;
		if ( pid->HasElement( "iMin" ) )
			iMin = pid->Get<double>( "iMin" );
		double cmdMax = 3;
		if ( pid->HasElement( "cmdMax" ) )
			cmdMax = pid->Get<double>( "cmdMax" );
		double cmdMin = -3;
		if ( pid->HasElement( "cmdMin" ) )
			cmdMin = pid->Get<double>( "cmdMin" );
		pid_.Init( p, i, d, iMax, iMin, cmdMax, cmdMin );
		use_pid_ = true;
	} else {
		use_pid_ = false;
	}

	if ( _sdf->HasElement( "linkName" ) )
		link_name_ = _sdf->GetElement( "linkName" )->Get<std::string>();
	else
		gzerr << "[gazebo_motor_model] Please specify a linkName of the rotor.\n";
	link_ = model_->GetLink( link_name_ );
	if ( link_ == NULL )
		gzthrow( "[gazebo_motor_model] Couldn't find specified link \"" << link_name_
																		<< "\"." );

	if ( _sdf->HasElement( "motorNumber" ) )
		motor_number_ = _sdf->GetElement( "motorNumber" )->Get<int>();
	else
		gzerr << "[gazebo_motor_model] Please specify a motorNumber.\n";

	if ( _sdf->HasElement( "turningDirection" ) ) {
		std::string turning_direction = _sdf->GetElement( "turningDirection" )->Get<std::string>();
		if ( turning_direction == "cw" )
			turning_direction_ = turning_direction::CW;
		else if ( turning_direction == "ccw" )
			turning_direction_ = turning_direction::CCW;
		else
			gzerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as "
					 "turningDirection.\n";
	} else
		gzerr << "[gazebo_motor_model] Please specify a turning direction ('cw' or "
				 "'ccw').\n";

	getSdfParam<std::string>( _sdf, "commandSubTopic", command_sub_topic_,
		command_sub_topic_ );
	getSdfParam<std::string>( _sdf, "motorSpeedPubTopic", motor_speed_pub_topic_,
		motor_speed_pub_topic_ );

	getSdfParam<double>( _sdf, "rotorDragCoefficient", rotor_drag_coefficient_,
		rotor_drag_coefficient_ );
	getSdfParam<double>( _sdf, "rollingMomentCoefficient",
		rolling_moment_coefficient_, rolling_moment_coefficient_ );
	getSdfParam<double>( _sdf, "maxRotVelocity", max_rot_velocity_,
		max_rot_velocity_ );
	getSdfParam<double>( _sdf, "motorConstant", motor_constant_, motor_constant_ );
	getSdfParam<double>( _sdf, "momentConstant", moment_constant_,
		moment_constant_ );

	getSdfParam<double>( _sdf, "timeConstantUp", time_constant_up_,
		time_constant_up_ );
	getSdfParam<double>( _sdf, "timeConstantDown", time_constant_down_,
		time_constant_down_ );
	getSdfParam<double>( _sdf, "rotorVelocitySlowdownSim",
		rotor_velocity_slowdown_sim_, 10 );

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	updateConnection_ = event::Events::ConnectWorldUpdateBegin(
		boost::bind( &GazeboMotorModel::OnUpdate, this, _1 ) );

	sub_command_ = nh_.subscribe<std_msgs::Float64MultiArray>(
		model_->GetName() + command_sub_topic_, 10,
		&GazeboMotorModel::VelocityCallback, this );
	pub_motor_velocity_ = nh_.advertise<std_msgs::Float64>(
		model_->GetName() + motor_speed_pub_topic_, 10 );

	// Create the first order filter.
	rotor_velocity_filter_.reset( new FirstOrderFilter<double>(
		time_constant_up_, time_constant_down_, ref_motor_rot_vel_ ) );

	ROS_INFO( "Loaded mantis motor model plugin [%i]!", motor_number_ + 1 );
}

// This gets called by the world update start event.
void GazeboMotorModel::OnUpdate( const common::UpdateInfo& _info ) {
	sampling_time_ = _info.simTime.Double() - prev_sim_time_;
	prev_sim_time_ = _info.simTime.Double();
	UpdateForcesAndMoments();
	Publish();
}

void GazeboMotorModel::VelocityCallback(
	const std_msgs::Float64MultiArray::ConstPtr& msg_in ) {
	if ( msg_in->data.size() < motor_number_ + 1 ) {
		std::cout << "You tried to access index " << motor_number_
				  << " of the MotorSpeed message array which is of size "
				  << msg_in->data.size() << "." << std::endl;
	} else
		ref_motor_rot_vel_ = std::min( static_cast<double>( msg_in->data[motor_number_] ),
			static_cast<double>( max_rot_velocity_ ) );
}

void GazeboMotorModel::UpdateForcesAndMoments() {
	motor_rot_vel_ = joint_->GetVelocity( 0 );
	if ( motor_rot_vel_ / ( 2 * M_PI ) > 1 / ( 2 * sampling_time_ ) ) {
		gzerr << "Aliasing on motor [" << motor_number_
			  << "] might occur. Consider making smaller simulation time steps or "
				 "raising the rotor_velocity_slowdown_sim_ param.\n";
	}
	double real_motor_velocity = motor_rot_vel_ * rotor_velocity_slowdown_sim_;
	double force = real_motor_velocity * real_motor_velocity * motor_constant_;

	// scale down force linearly with forward speed
	// XXX this has to be modelled better
	ignition::math::Vector3<double> body_velocity = link_->WorldLinearVel();
	double vel = body_velocity.Length();
	double scalar = 1 - vel / 25.0; // at 50 m/s the rotor will not produce any force anymore
	scalar = ignition::math::clamp( scalar, 0.0, 1.0 );
	// Apply a force to the link.
	link_->AddRelativeForce( ignition::math::Vector3<double>( 0, 0, force * scalar ) );

	// Forces from Philppe Martin's and Erwan Salaün's
	// 2010 IEEE Conference on Robotics and Automation paper
	// The True Role of Accelerometer Feedback in Quadrotor Control
	// - \omega * \lambda_1 * V_A^{\perp}
	ignition::math::Vector3<double> joint_axis = joint_->GlobalAxis( 0 );
	// ignition::math::Vector3 body_velocity = link_->GetWorldLinearVel();
	ignition::math::Vector3<double> body_velocity_perpendicular = body_velocity - ( body_velocity * joint_axis ) * joint_axis;
	ignition::math::Vector3<double> air_drag = -std::abs( real_motor_velocity ) * rotor_drag_coefficient_ * body_velocity_perpendicular;
	// Apply air_drag to link.
	link_->AddForce( air_drag );
	// Moments
	// Getting the parent link, such that the resulting torques can be applied to
	// it.
	physics::Link_V parent_links = link_->GetParentJointsLinks();
	// The tansformation from the parent_link to the link_.
	ignition::math::Pose3d pose_difference = link_->WorldCoGPose() - parent_links.at( 0 )->WorldCoGPose();
	ignition::math::Vector3<double> drag_torque( 0, 0, -turning_direction_ * force * moment_constant_ );
	// Transforming the drag torque into the parent frame to handle arbitrary
	// rotor orientations.
	ignition::math::Vector3<double> drag_torque_parent_frame = pose_difference.Rot().RotateVector( drag_torque );
	parent_links.at( 0 )->AddRelativeTorque( drag_torque_parent_frame );

	ignition::math::Vector3<double> rolling_moment;
	// - \omega * \mu_1 * V_A^{\perp}
	rolling_moment = -std::abs( real_motor_velocity ) * rolling_moment_coefficient_ * body_velocity_perpendicular;
	parent_links.at( 0 )->AddTorque( rolling_moment );
	// Apply the filter on the motor's velocity.
	double ref_motor_rot_vel;
	ref_motor_rot_vel = rotor_velocity_filter_->updateFilter( ref_motor_rot_vel_, sampling_time_ );

#if 0 // FIXME: disable PID for now, it does not play nice with the PX4 CI
      // system.
  if (use_pid_)
  {
    double err = joint_->GetVelocity(0) - turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_;
    double rotorForce = pid_.Update(err, sampling_time_);
    joint_->SetForce(0, rotorForce);
    // gzerr << "rotor " << joint_->GetName() << " : " << rotorForce << "\n";
  }
  else
  {
#if GAZEBO_MAJOR_VERSION >= 7
    // Not desirable to use SetVelocity for parts of a moving model
    // impact on rest of the dynamic system is non-physical.
    joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_);
#elif GAZEBO_MAJOR_VERSION >= 6
    // Not ideal as the approach could result in unrealistic impulses, and
    // is only available in ODE
    joint_->SetParam("fmax", 0, 2.0);
    joint_->SetParam("vel", 0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_);
#endif
  }
#else
	joint_->SetVelocity( 0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_ );
#endif /* if 0 */
}

GZ_REGISTER_MODEL_PLUGIN( GazeboMotorModel );
}
