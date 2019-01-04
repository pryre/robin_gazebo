#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/HilSensor.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <eigen3/Eigen/Dense>

#include <string>
#include <iostream>

class RobinGazeboHilSensorViz {
	private:
		ros::NodeHandle nh_;

		ros::Subscriber sub_imu_;
		ros::Subscriber sub_odom_;
		ros::Subscriber sub_rc_;
		ros::Timer timer_viz_;
		ros::Publisher pub_hil_;

		//TF
		tf2_ros::TransformBroadcaster tfbr_;
		tf2_ros::StaticTransformBroadcaster tfsbr_;
		std::vector<geometry_msgs::TransformStamped> tf_props;

		mavros_msgs::RCOut msg_rc_;
		nav_msgs::Odometry msg_odom_;

		//Params
		bool armed_;
		std::string param_prop_layout_;
		unsigned int motor_num_;
		std::vector<bool> motor_dir_;
		double base_arm_length_;
		double prop_rate_;
		double viz_rate_;

	public:
		RobinGazeboHilSensorViz() :
			nh_(),
			param_prop_layout_(""),
			motor_num_(0),
			base_arm_length_(0.250),
			prop_rate_(0.57),
			viz_rate_(25.0) {

			nh_.param("viz_rate", viz_rate_, viz_rate_);
			nh_.param("prop_layout", param_prop_layout_, param_prop_layout_);
			nh_.param("base_arm_length", base_arm_length_, base_arm_length_);

			sub_odom_ = nh_.subscribe<nav_msgs::Odometry> ( "state/odom", 1, &RobinGazeboHilSensorViz::odom_cb, this );
			pub_hil_ = nh_.advertise<mavros_msgs::HilSensor>( "hil/imu_data", 100 );

			ROS_INFO("[HIL] waiting for odometry...");

			while( ros::ok() && msg_odom_.header.stamp == ros::Time(0) ) {
				ros::spinOnce();
				ros::Rate(viz_rate_).sleep();
			}

			if( ros::ok() ) {
				ROS_INFO("[HIL] preparing visualization transforms...");

				//TF
				//Send static transforms for rviz model
				/*
				geometry_msgs::TransformStamped tf;
				tf.header.stamp = ros::Time::now();
				tf.header.frame_id = msg_odom_.child_frame_id;
				tf.child_frame_id = msg_odom_.child_frame_id + "/base_link";
				tf.transform.rotation.w = 1.0;
				tfsbr_.sendTransform(tf);
				*/
				prepare_prop_viz();

				sub_imu_ = nh_.subscribe<sensor_msgs::Imu> ( "state/imu_data", 1, &RobinGazeboHilSensorViz::imu_cb, this );
				sub_rc_ = nh_.subscribe<mavros_msgs::RCOut> ( "command/motor_pwm", 1, &RobinGazeboHilSensorViz::rc_cb, this );
				timer_viz_ = nh_.createTimer(ros::Duration(1.0/viz_rate_), &RobinGazeboHilSensorViz::viz_cb, this );

				ROS_INFO("[HIL] Ready and running!");
			}
		}

		~RobinGazeboHilSensorViz() {
		}

		void rc_cb( const mavros_msgs::RCOut::ConstPtr& msg_in ) {
			msg_rc_ = *msg_in;
		}

		void odom_cb( const nav_msgs::Odometry::ConstPtr& msg_in ) {
			msg_odom_ = *msg_in;
		}


		void imu_cb( const sensor_msgs::Imu::ConstPtr& msg_in ) {
			mavros_msgs::HilSensor msg_out;

			msg_out.header = msg_in->header;
			msg_out.acc = msg_in->linear_acceleration;
			msg_out.gyro = msg_in->angular_velocity;

			//TODO: The rest (in another callback using "fields_updated" maybe?)
			msg_out.mag.x = 0.0;
			msg_out.mag.y = 0.0;
			msg_out.mag.z = 0.0;

			msg_out.abs_pressure = 0.0;
			msg_out.diff_pressure = 0.0;
			msg_out.pressure_alt = 0.0;

			msg_out.temperature = 0.0;

			//Updated IMU and gyro
			msg_out.fields_updated = 0b00000000000000000000000000111111;

			pub_hil_.publish(msg_out);
		}

		void viz_cb( const ros::TimerEvent& e ) {
			//Handle TF things here as well
			geometry_msgs::TransformStamped tb;
			tb.header.frame_id = msg_odom_.header.frame_id;
			tb.header.stamp = e.current_real;
			tb.child_frame_id = msg_odom_.child_frame_id + "/base_link";
			tb.transform.translation.x = msg_odom_.pose.pose.position.x;
			tb.transform.translation.y = msg_odom_.pose.pose.position.y;
			tb.transform.translation.z = msg_odom_.pose.pose.position.z;
			tb.transform.rotation = msg_odom_.pose.pose.orientation;
			tfbr_.sendTransform(tb);

			if( (motor_num_ > 0) && (motor_num_ < 9) ) {
				Eigen::Quaterniond r(Eigen::AngleAxisd(prop_rate_, Eigen::Vector3d::UnitZ()));

				for(int i=0; i<motor_num_; i++) {
					if(msg_rc_.channels.size() >= motor_num_) {
						if(msg_rc_.channels[i] > 1000) {
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
					tf_props[i].header.frame_id = msg_odom_.child_frame_id + "/base_link";
					tf_props[i].child_frame_id = msg_odom_.child_frame_id + "/link_rotor_" + std::to_string(i+1);
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
				ROS_ERROR("[HIL] Unable to load motor configuration");
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

int main(int argc, char** argv) {
	ros::init(argc, argv, "robin_gazebo_hil_sensor_viz");
	RobinGazeboHilSensorViz hil_viz;

	ros::spin();

	return 0;
}
