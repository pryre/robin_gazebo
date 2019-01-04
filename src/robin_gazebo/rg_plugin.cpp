#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Float64MultiArray.h>
#include <mavros_msgs/RCOut.h>

#include <string>
#include <vector>
#include <math.h>

namespace gazebo
{
class RobinGazeboPlugin : public ModelPlugin
{

	private:
		physics::ModelPtr model_;	// Pointer to the model

		ros::NodeHandle nh_;

		//Model Odom
		ros::Timer timer_odom_;
		ros::Publisher pub_odom_;
		ros::Publisher pub_pose_;

		//PWM Motor
		ros::Subscriber sub_rc_out_;
		ros::Publisher pub_motor_velocity_;

		//Parameters
		uint16_t param_pwm_min_;
		uint16_t param_pwm_max_;
		double param_motor_vel_max_;

		std::string parent_name_;
		nav_msgs::Odometry msg_odom_;
		geometry_msgs::PoseStamped msg_pose_;

	public:
		RobinGazeboPlugin() : ModelPlugin(),
							  param_pwm_min_(1000),
							  param_pwm_max_(2000),
							  param_motor_vel_max_(1618),
							  parent_name_("map") {
		}

		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
			model_ = _parent;
			nh_ = ros::NodeHandle(model_->GetName());

			// Make sure the ROS node for Gazebo has already been initialized
			if( !ros::isInitialized() ) {
			  ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load plugin.");
			  return;
			}

			//PWM
			sub_rc_out_ = nh_.subscribe<mavros_msgs::RCOut>("command/motor_pwm", 100, &RobinGazeboPlugin::rc_out_cb, this );
			pub_motor_velocity_ = nh_.advertise<std_msgs::Float64MultiArray>("command/motor_velocity", 10);

			//Model Odom
			timer_odom_ = nh_.createTimer(ros::Duration(0.01), &RobinGazeboPlugin::callback_odom, this );
			pub_odom_ = nh_.advertise<nav_msgs::Odometry>( "state/odom", 10 );
			pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>( "state/pose", 10 );

			msg_odom_.header.frame_id = parent_name_;
			msg_odom_.child_frame_id = model_->GetName();
			msg_pose_.header.frame_id = parent_name_;

			ROS_INFO("Loaded robin gazebo plugin!");
		}

	private:
		int32_t int32_constrain(const int32_t i, const int32_t min, const int32_t max) {
			return (i < min) ? min : (i > max) ? max : i;
		}

		void rc_out_cb( const mavros_msgs::RCOut::ConstPtr& msg_in ) {
			std_msgs::Float64MultiArray msg_motor_velocity;
			msg_motor_velocity.data.resize( msg_in->channels.size() );

			for(int i=0; i<msg_in->channels.size(); i++){
				//Cut down ignored channels
				int pwm_raw = (msg_in->channels[i] != 0 ) && (msg_in->channels[i] != 65535) ? msg_in->channels[i] : 0;

				double pwm = int32_constrain( pwm_raw, param_pwm_min_, param_pwm_max_);

				double norm_ref = ( pwm - param_pwm_min_ ) / ( param_pwm_max_ - param_pwm_min_ );
				//double cmd_vel = norm_ref * param_motor_vel_max_;
				msg_motor_velocity.data[i] = norm_ref * param_motor_vel_max_;
			}

			pub_motor_velocity_.publish(msg_motor_velocity);
		}

		void callback_odom( const ros::TimerEvent& e ) {
			//Handle Odometry for ROS
			msg_odom_.header.stamp = e.current_real;

			msg_odom_.pose.pose.position.x = model_->GetWorldPose().pos.x;
			msg_odom_.pose.pose.position.y = model_->GetWorldPose().pos.y;
			msg_odom_.pose.pose.position.z = model_->GetWorldPose().pos.z;
			msg_odom_.pose.pose.orientation.w = model_->GetWorldPose().rot.w;
			msg_odom_.pose.pose.orientation.x = model_->GetWorldPose().rot.x;
			msg_odom_.pose.pose.orientation.y = model_->GetWorldPose().rot.y;
			msg_odom_.pose.pose.orientation.z = model_->GetWorldPose().rot.z;

			msg_odom_.twist.twist.linear.x = model_->GetRelativeLinearVel().x;
			msg_odom_.twist.twist.linear.y = model_->GetRelativeLinearVel().y;
			msg_odom_.twist.twist.linear.z = model_->GetRelativeLinearVel().z;
			msg_odom_.twist.twist.angular.x = model_->GetRelativeAngularVel().x;
			msg_odom_.twist.twist.angular.y = model_->GetRelativeAngularVel().y;
			msg_odom_.twist.twist.angular.z = model_->GetRelativeAngularVel().z;

			pub_odom_.publish(msg_odom_);

			//Derive Pose output for ease of use
			msg_pose_.header.stamp = e.current_real;
			msg_pose_.pose = msg_odom_.pose.pose;
			pub_pose_.publish(msg_pose_);
		}
};

GZ_REGISTER_MODEL_PLUGIN(RobinGazeboPlugin)
}
