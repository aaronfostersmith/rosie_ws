#include <ros/ros.h>
#include "ros/console.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sweeper_interface/Imu_bytes.h>

class SubAndPub
{
public:
    SubAndPub()
    {
        imu_sub_ = n_.subscribe("/sweeper/imu", 10, &SubAndPub::ImuCallback, this);
        imu_pub_ = n_.advertise<sensor_msgs::Imu>("imu/data_raw", 100);
        mag_pub_ = n_.advertise<sensor_msgs::MagneticField>("imu/mag", 100);
    }
    
    void ImuCallback(const sweeper_interface::Imu_bytes &imub_input)
    {
       //create an objects for output
            sensor_msgs::Imu imu_output;
            sensor_msgs::MagneticField mag_output;

            //populate the output's headers
            imu_output.header.stamp = ros::Time::now();
            imu_output.header.frame_id = "imu_frame";
            mag_output.header.stamp = ros::Time::now();
            mag_output.header.frame_id = "imu_frame";

            //copy data from the input to the appropriate output fields
            //copy raw angular and linear imub_input
            imu_output.angular_velocity.x = imub_input.angular_velocity[0];
            imu_output.angular_velocity.y = imub_input.angular_velocity[1];
            imu_output.angular_velocity.z = imub_input.angular_velocity[2];
            imu_output.linear_acceleration.x = imub_input.linear_acceleration[0];
            imu_output.linear_acceleration.y = imub_input.linear_acceleration[1];
            imu_output.linear_acceleration.z = imub_input.linear_acceleration[2];

    /*
            //covariances
            imu_output.orientation_covariance = {{0.1,0.0,0.0},
                                 {0.0,0.1,0.0},
                                 {0.0,0.0,0.1}};

            imu_output.angular_velocity_covariance = {{0.1,0.0,0.0},
                                  {0.0,0.1,0.0},
                                  {0.0,0.0,0.1}};

            imu_output.linear_acceleration_covariance = {{0.1,0.0,0.0},
                                     {0.0,0.1,0.0},
                                     {0.0,0.0,0.1}};
    */
            //copy raw magnetic field data
            mag_output.magnetic_field.x = imub_input.magnetic_field[0];
            mag_output.magnetic_field.y = imub_input.magnetic_field[1];
            mag_output.magnetic_field.z = imub_input.magnetic_field[2];
           /* 
            //covariance
            mag_output.magnetic_field_covariance = {{0.1,0.0,0.0},
                                {0.0,0.1,0.0},
                                {0.0,0.0,0.1}};
            */
            //publish
            imu_pub_.publish(imu_output);
            mag_pub_.publish(mag_output);
    }
    
private:
    ros::NodeHandle n_;
    ros::Subscriber imu_sub_;
    ros::Publisher imu_pub_;
    ros::Publisher mag_pub_;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_builder");
        
    ros::spin();
    return 0;
}