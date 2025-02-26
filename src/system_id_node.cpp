#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <Eigen/Dense> // For matrix operations
#include <tf/tf.h>     // For quaternion operations

using namespace std;
using namespace Eigen;



///please not while using this code 
// while using this please make sure you use a normal proportional controller for PX4 attitude controller
///please not while using this code 
// while using this please make sure you use a normal proportional controller for PX4 attitude controller
///please not while using this code 
// while using this please make sure you use a normal proportional controller for PX4 attitude controller
///please not while using this code 
// while using this please make sure you use a normal proportional controller for PX4 attitude controller


//rosbag record -o ./system_id_sin_wave.bag /mavros/local_position/pose /mavros/setpoint_raw/attitude for recording the data










// Global variables for storing data
vector<Vector2d> commands;
vector<Vector2d> responses;
double dt = 0.01; // Time step (assume 100 Hz)

// Callback for IMU data (to get roll and pitch)

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    double roll, pitch, yaw;

    // Extract roll and pitch from quaternion
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    ROS_INFO("IMU Callback Triggered - Roll: %f, Pitch: %f", roll, pitch);

    // Store the response (roll and pitch)
    
    responses.emplace_back(roll, pitch);
    
}

// Publish attitude commands
void publishCommands(ros::Publisher& pub) {
    static double time = 0.0;
    tf::Quaternion q;
    // Generate sinusoidal commands for roll and pitch
    
    // double theta_cmd = 0.01*sin(time);         //for sinusoidal signal
    // double phi_cmd = 0.1*cos(time);               //for sinusoidal signal



    // Chirp signal parameters
    double f0 = 0.1; // Initial frequency
    double f1 = 1.0; // Final frequency
    double T = 20.0; // Duration of the chirp

    // Generate chirp signal for roll and pitch
    double k = 5.0; // Controls how sharply frequency increases
    // double f_t = f0 + (f1 - f0) * (1 - exp(-k * time / T)); 
double phi_cmd = 0.0;
//double phi_cmd = 0.01*cos(2 * M_PI * (f0 + (f1 - f0) * time / (2 * T)) * time);
    double theta_cmd = 0.01*cos(2 * M_PI * (f0 + (f1 - f0) * time / (2 * T)) * time);
   // double theta_cmd = 0.5*cos(2 * M_PI * (f0 + (f1 - f0) * time / (2 * T)) * time);
    // double theta_cmd = 0.5 * cos(2 * M_PI * f_t * time);
    q.setRPY(phi_cmd, theta_cmd, 0.0);

    q.normalize();
    // Store the commands
    commands.emplace_back(phi_cmd, theta_cmd);

    // Publish the commands to MAVROS
    mavros_msgs::AttitudeTarget cmd_msg;
    cmd_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
        cmd_msg.orientation.x = q.x();
        cmd_msg.orientation.y = q.y();
        cmd_msg.orientation.z = q.z();
        cmd_msg.orientation.w = q.w();
        cmd_msg.thrust = 0.7; // Thrust is 0.7 for sin wave (normalized value)

    
    time += dt;
    if (time>30)
    {
        pub.publish(cmd_msg);
    }
    if (time>50)
    {
        time=30;

    }

    

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "mavros_system_identification_node");
    ros::NodeHandle nh;

    // Subscribers and publishers
    ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data", 100, imuCallback);
    
    ros::Publisher attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 100);

    ros::Rate loop_rate(100); // Run at 100 Hz

    while (ros::ok()) {
        publishCommands(attitude_pub);
        ROS_INFO("Commands size: %lu, Responses size: %lu", commands.size(), responses.size());


        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

