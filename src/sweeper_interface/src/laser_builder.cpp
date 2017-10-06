#include <ros/ros.h>
#include "ros/console.h"

#include <sensor_msgs/LaserScan.h>
#include <sweeper_interface/laser_range.h>



class SubAndPub
{
public:
    SubAndPub()
    {
        //get params 
        if (!ros::param::get("~angle_min", _ls_angle_min))
        {
        _ls_angle_min = -3.14159265359;
            ROS_WARN("Failed to get angle_min param. Using 0 rad as default");
        }
         if (!ros::param::get("~angle_max", _ls_angle_max))
        {
        _ls_angle_max = 3.14159265359;
        ROS_WARN("Failed to get angle_max param. Using 2*pi rad as default");
        }
         if (!ros::param::get("~num_pts", _ls_num_pts))
        {
        _ls_num_pts= 32;
        ROS_WARN("Failed to get num_pts param. Using 64 pts as default");
        }
         if (!ros::param::get("~time_increment", _ls_time_increment))
        {
        _ls_time_increment = 0.025;
        ROS_WARN("Failed to get time_increment param. Using 0.03 seconds as default");
        }
    
        ros::NodeHandle n_;
        ros::Subscriber laser_sub_ = n_.subscribe("/sweeper/rawLaser", 10, &SubAndPub::LaserCallback, this);
        ros::Publisher ls_pub_ = n_.advertise<sensor_msgs::LaserScan>("/sweeper/scan", 100);
     }

        void LaserCallback(const sweeper_interface::laser_range &singleRange)
        {

                //containers for PCL and ROS data types

                static sensor_msgs::LaserScan ls_temp;
                static bool init = false;

                if(singleRange.seq <= _ls_num_pts)
                {
                    if (singleRange.seq == 1 || !init)
                    {
                    //set laserscan parameters
                        ls_temp.angle_min = _ls_angle_min;
                        ls_temp.angle_max = _ls_angle_max; //2*pi
                        ls_temp.angle_increment = (ls_temp.angle_max-ls_temp.angle_min)/_ls_num_pts; //pi/2
                        ls_temp.time_increment = _ls_time_increment;
                        ls_temp.scan_time = ls_temp.time_increment*(float)_ls_num_pts;
                        ls_temp.range_min = 0.02;
                        ls_temp.range_max = 1.005;
                        ls_temp.ranges.resize(_ls_num_pts); //4
                        ls_temp.header.stamp = ros::Time::now();
                    init = true;		    
                    }

                    //populate header
                    ls_temp.header.frame_id = "hokuyo";

                    //fill with laserScan with num_pts data points

                    //laserscan
                    ls_temp.ranges[singleRange.seq]= singleRange.range/1000.0;

                    if (singleRange.seq == _ls_num_pts)
                    {
                        ls_pub_.publish(ls_temp);
                    }
                }
        }
    
private:
ros::NodeHandle n_;
ros::Subscriber laser_sub_;
ros::Publisher ls_pub_;
double _ls_angle_min;
double _ls_angle_max;
int _ls_num_pts;
double _ls_time_increment;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_builder");
        
	SubAndPub subAndPubObj;    

    ros::spin();
    return 0;
}
