#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Float64MultiArray.h>
#include <mavros_msgs/RCOut.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <eigen3/Eigen/Dense>

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
		mavros_msgs::RCOut rc_out_;

		//TF
		tf2_ros::TransformBroadcaster tfbr_;
		tf2_ros::StaticTransformBroadcaster tfsbr_;
		std::vector<geometry_msgs::TransformStamped> tf_props;

		//Parameters
		uint16_t param_pwm_min_;
		uint16_t param_pwm_max_;
		double param_motor_vel_max_;
		std::string param_prop_layout_;
		unsigned int motor_num_;
		std::vector<bool> motor_dir_;
		double base_arm_length_;
		double prop_rate_;

		std::string parent_name_;
		nav_msgs::Odometry msg_odom_;
		geometry_msgs::PoseStamped msg_pose_;

		std_msgs::Float64MultiArray msg_motor_velocity_;

	public:
		RobinGazeboPlugin() : ModelPlugin(),
							  param_pwm_min_(1000),
							  param_pwm_max_(2000),
							  param_motor_vel_max_(1618),
							  param_prop_layout_(""),
							  motor_num_(0),
							  base_arm_length_(0.250),
							  prop_rate_(0.18),
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

			nh_.param("prop_layout", param_prop_layout_, param_prop_layout_);
			nh_.param("base_arm_length", base_arm_length_, base_arm_length_);

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

			ROS_INFO("Robin gazebo preparing transforms...");

			//TF
			//Send static transforms for rviz model
			//TODO: Make more dynamic with params
			geometry_msgs::TransformStamped tf;
			tf.header.stamp = ros::Time::now();
			tf.transform.rotation.w = 1.0;

			//World Frame
			tf.header.frame_id = "world";
			tf.child_frame_id = parent_name_;
			tfsbr_.sendTransform(tf);

			//Base Link
			tf.header.frame_id = model_->GetName();
			tf.child_frame_id = "base_link";
			tfsbr_.sendTransform(tf);

			prepare_prop_viz();

			ROS_INFO("Loaded robin gazebo plugin!");
		}

	private:
		int32_t int32_constrain(const int32_t i, const int32_t min, const int32_t max) {
			return (i < min) ? min : (i > max) ? max : i;
		}

		void rc_out_cb( const mavros_msgs::RCOut::ConstPtr& msg_in ) {
			rc_out_ = *msg_in;

			msg_motor_velocity_.data.resize( rc_out_.channels.size() );

			for(int i=0; i<rc_out_.channels.size(); i++){
				//Cut down ignored channels
				int pwm_raw = (rc_out_.channels[i] != 0 ) && (rc_out_.channels[i] != 65535) ? rc_out_.channels[i] : 0;

				double pwm = int32_constrain( pwm_raw, param_pwm_min_, param_pwm_max_);

				double norm_ref = ( pwm - param_pwm_min_ ) / ( param_pwm_max_ - param_pwm_min_ );
				//double cmd_vel = norm_ref * param_motor_vel_max_;
				msg_motor_velocity_.data[i] = std::sqrt(norm_ref * param_motor_vel_max_ * param_motor_vel_max_);
			}

			pub_motor_velocity_.publish(msg_motor_velocity_);
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

			//Handle Pose for ROS
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

			//Handle TF things here as well
			geometry_msgs::TransformStamped tb;
			tb.header.frame_id = parent_name_;
			tb.header.stamp = e.current_real;
			tb.child_frame_id = model_->GetName();
			tb.transform.translation.x = msg_odom_.pose.pose.position.x;
			tb.transform.translation.y = msg_odom_.pose.pose.position.y;
			tb.transform.translation.z = msg_odom_.pose.pose.position.z;
			tb.transform.rotation = msg_odom_.pose.pose.orientation;
			tfbr_.sendTransform(tb);

			if( (motor_num_ > 0) && (motor_num_ < 9) ) {
				Eigen::Quaterniond r(Eigen::AngleAxisd(prop_rate_, Eigen::Vector3d::UnitZ()));

				for(int i=0; i<motor_num_; i++) {
					if(rc_out_.channels.size() >= motor_num_) {
						if(rc_out_.channels[i] > param_pwm_min_) {
							Eigen::Quaterniond q(tf_props[i].transform.rotation.w,
												 tf_props[i].transform.rotation.x,
												 tf_props[i].transform.rotation.y,
												 tf_props[i].transform.rotation.z);
							//Correct q for motor directions
							Eigen::Quaterniond mr = (motor_dir_[i]) ? r : r.inverse();
							tf_props[i].transform.rotation = quaternion_from_eig(mr*q);
						}
					}

					tf_props[i].header.stamp = e.current_real;
					tfbr_.sendTransform(tf_props[i]);
				}
			}
		}

		void prepare_prop_viz( void ) {
			//Propeller links
			if(param_prop_layout_.find("4") != std::string::npos) {
				motor_num_ = 4;
			} else if(param_prop_layout_.find("6") != std::string::npos) {
				motor_num_ = 6;
			}

			if(motor_num_ > 0) {
				tf_props.resize(motor_num_);
				motor_dir_.resize(motor_num_);
				Eigen::Vector3d arm = Eigen::Vector3d(base_arm_length_, 0.0, 0.046);

				//Prepare common values
				for(int i=0; i<motor_num_; i++) {
					tf_props[i].header.frame_id = model_->GetName();
					tf_props[i].child_frame_id = "link_rotor_" + std::to_string(i+1);
					tf_props[i].transform.rotation.w = 1.0;
				}

				if(param_prop_layout_ == "quad_p4") {
					Eigen::AngleAxisd rot(3.0*M_PI/2.0, Eigen::Vector3d::UnitZ());
					tf_props[0].transform.translation = vector_from_eig(rot*arm);
					rot.angle() = M_PI/2.0;
					tf_props[1].transform.translation = vector_from_eig(rot*arm);
					rot.angle() = 0.0;
					tf_props[2].transform.translation = vector_from_eig(rot*arm);
					rot.angle() = M_PI;
					tf_props[3].transform.translation = vector_from_eig(rot*arm);

					motor_dir_ = {1, 1, 0, 0};
				} else if(param_prop_layout_ == "quad_x4") {
					Eigen::AngleAxisd rot(7.0*M_PI/4.0, Eigen::Vector3d::UnitZ());
					tf_props[0].transform.translation = vector_from_eig(rot*arm);
					rot.angle() = 3.0*M_PI/4.0;
					tf_props[1].transform.translation = vector_from_eig(rot*arm);
					rot.angle() = M_PI/4.0;
					tf_props[2].transform.translation = vector_from_eig(rot*arm);
					rot.angle() = 5.0*M_PI/4.0;
					tf_props[3].transform.translation = vector_from_eig(rot*arm);

					motor_dir_ = {1, 1, 0, 0};
				} else if(param_prop_layout_ == "hex_p6") {
					Eigen::AngleAxisd rot(0.0, Eigen::Vector3d::UnitZ());
					tf_props[0].transform.translation = vector_from_eig(rot*arm);
					rot.angle() = M_PI;
					tf_props[1].transform.translation = vector_from_eig(rot*arm);
					rot.angle() = 2.0*M_PI/3.0;
					tf_props[2].transform.translation = vector_from_eig(rot*arm);
					rot.angle() = 5.0*M_PI/3.0;
					tf_props[3].transform.translation = vector_from_eig(rot*arm);
					rot.angle() = 1.0*M_PI/3.0;
					tf_props[4].transform.translation = vector_from_eig(rot*arm);
					rot.angle() = 4.0*M_PI/3.0;
					tf_props[5].transform.translation = vector_from_eig(rot*arm);

					motor_dir_ = {0, 1, 0, 1, 1, 0};
				} else if(param_prop_layout_ == "hex_x6") {
					Eigen::AngleAxisd rot(9.0*M_PI/6.0, Eigen::Vector3d::UnitZ());
					tf_props[0].transform.translation = vector_from_eig(rot*arm);
					rot.angle() = 3.0*M_PI/6.0;
					tf_props[1].transform.translation = vector_from_eig(rot*arm);
					rot.angle() = 1.0*M_PI/6.0;
					tf_props[2].transform.translation = vector_from_eig(rot*arm);
					rot.angle() = 7.0*M_PI/6.0;
					tf_props[3].transform.translation = vector_from_eig(rot*arm);
					rot.angle() = 11.0*M_PI/6.0;
					tf_props[4].transform.translation = vector_from_eig(rot*arm);
					rot.angle() = 5.0*M_PI/6.0;
					tf_props[5].transform.translation = vector_from_eig(rot*arm);

					motor_dir_ = {0, 1, 0, 1, 1, 0};
				}
			} else {
				ROS_ERROR("[ROBIN_GAZEBO] Unable to load motor configuration");
			}
		}


		geometry_msgs::Vector3 vector_from_eig(const Eigen::Vector3d &v) {
			geometry_msgs::Vector3 vec;

			vec.x = v.x();
			vec.y = v.y();
			vec.z = v.z();

			return vec;
		}

		geometry_msgs::Quaternion quaternion_from_eig(const Eigen::Quaterniond &q) {
			geometry_msgs::Quaternion quat;
			Eigen::Quaterniond qn = q.normalized();

			quat.w = qn.w();
			quat.x = qn.x();
			quat.y = qn.y();
			quat.z = qn.z();

			return quat;
		}
};

GZ_REGISTER_MODEL_PLUGIN(RobinGazeboPlugin)
}
