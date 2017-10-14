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
        
        static bool init;
        //create an objects for output
        static sensor_msgs::Imu imu_output;
        static sensor_msgs::MagneticField mag_output;
        
        if(!init)
        {
                        mag_output.header.frame_id = "imu_link";
                        imu_output.header.frame_id = "imu_link";
 
            imu_output.orientation_covariance[0] = -1; //no orientation data produced
            imu_output.orientation_covariance[4] = 0.01;
            imu_output.orientation_covariance[8] = 0.01;

            imu_output.angular_velocity_covariance[0] =0.001;
            imu_output.angular_velocity_covariance[4] =0.001;
            imu_output.angular_velocity_covariance[8] =0.001;

            imu_output.linear_acceleration_covariance[0] =0.001;
            imu_output.linear_acceleration_covariance[4] =0.001;
            imu_output.linear_acceleration_covariance[8] =0.001;
                      
            mag_output.magnetic_field_covariance[0] =0.001;
            mag_output.magnetic_field_covariance[4] =0.001;
            mag_output.magnetic_field_covariance[8] =0.001;
            
            init =true;
        }
       

            //populate the output's headers
            imu_output.header.stamp = ros::Time::now();
            mag_output.header.stamp = ros::Time::now();

            //copy data from the input to the appropriate output fields
            //copy raw angular and linear imub_input
            imu_output.angular_velocity.x = imub_input.angular_velocity[0];
            imu_output.angular_velocity.y = imub_input.angular_velocity[1];
            imu_output.angular_velocity.z = imub_input.angular_velocity[2];
            
            imu_output.linear_acceleration.x = imub_input.linear_acceleration[0];
            imu_output.linear_acceleration.y = imub_input.linear_acceleration[1];
            imu_output.linear_acceleration.z = imub_input.linear_acceleration[2];

            //copy raw magnetic field data
            mag_output.magnetic_field.x = imub_input.magnetic_field[0];
            mag_output.magnetic_field.y = imub_input.magnetic_field[1];
            mag_output.magnetic_field.z = imub_input.magnetic_field[2];
      
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

	SubAndPub subAndPubObj;    

    ros::spin();
    return 0;
}
