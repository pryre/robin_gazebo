#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <mavros_msgs/HilSensor.h>

#include <string>
#include <iostream>

class RobinGazeboHilSensor {
	private:
		ros::NodeHandle nh_;

		ros::Subscriber sub_imu_;
		ros::Publisher pub_hil_;

	public:
		RobinGazeboHilSensor() :
			nh_() {


			// Subscrive to input video feed and publish output video feed
			sub_imu_ = nh_.subscribe<sensor_msgs::Imu> ( "state/imu_data", 100, &RobinGazeboHilSensor::imu_cb, this );
			pub_hil_ = nh_.advertise<mavros_msgs::HilSensor>( "hil/imu_data", 100 );

			ROS_INFO("[HIL] Forwarding IMU sensor data!");
		}

		~RobinGazeboHilSensor() {
		}

		//We can use the camera info to correct for distortions, get image size, and other useful things
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
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "robin_gazebo_hil_sensor");
	RobinGazeboHilSensor hil;

	ros::spin();

	return 0;
}
