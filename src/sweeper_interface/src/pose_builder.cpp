//adapted from: https://answers.ros.org/question/11973/gathering-wheel-encoder-data-and-publishing-on-the-odom-topic/
/* twist and frame "odom" tf will be performed by robot_localization
*
*/

#include "ros/ros.h"
#include "ros/console.h"
#include <tf/transform_datatypes.h>
#include <sweeper_interface/Ticks.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Time current_time_encoder;
const double DistancePerCount = (3.14159265 * 0.060) / 224; //meters
const double WheelBase = 0.185; //meters
double x= 0;
double y= 0;
double th= 0;

double vx;
double vy;
double vth;

void WheelCallback(const sweeper_interface::Ticks &ticks)
{

    current_time_encoder = ticks.stamp;
    //calculate displacements
    double s_l = ticks.l*DistancePerCount;
    double s_r = ticks.r*DistancePerCount;
    double s_c = (s_r+s_l)/2;

    //estimate new position assuming small angles  
    x += s_c*cos(th); 
    y += s_c*sin(th); 
    th += (s_r-s_l)/WheelBase;
    //ROS_WARN("s_l = %d", s_l);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_publisher");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/sweeper/ticks", 100, WheelCallback);
    ros::Publisher _pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/odom/encPose", 50);   

    geometry_msgs::PoseWithCovarianceStamped t;
    t.header.frame_id = "odom";
    /* vector: x,y,z,rx,ry,rz
    
    {x*x , x*y , x*z , x*rx , x*ry , x*rz ,
     y*x , y*y , y*z , y*rx , y*ry , y*rz ,
     z*x , z*y , z*z , z*rx , z*ry , z*rz ,
     rx*x, rx*y, rx*z, rx*rx, rx*ry, rx*rz,
     ry*x, ry*y, ry*z, ry*rx, ry*ry, ry*rz,
     rz*x, rz*y, rz*z, rz*rx, rz*ry, rz*rz};
     
     x,y,rz have values, all else =0     
    {x*x , x*y , 0 , 0 , 0 , 0 ,
     y*x , y*y , 0 , 0 , 0 , 0 ,
     0   , 0   , 1 , 0 , 0 , 0 ,
     0   , 0   , 0 , 1 , 0 , 0,
     0   , 0   , 0 , 0 , 1 , 0,
     0   , 0   , 0 , 0 , 0 , rz*rz};
    */
    
    t.pose.covariance[0] = 0.001; 
    t.pose.covariance[7] = 0.001; 
    t.pose.covariance[35] = 0.001; 
    
    ros::Rate r(10.0);
    while(n.ok())
    {
        //since all odometry is 6DOF we'll need a quaternion created from yaw
        // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //time stamp
        t.header.stamp = current_time_encoder;
        //set the position
        t.pose.pose.position.x = x;
        t.pose.pose.position.y = y;
        t.pose.pose.position.z = 0.0;
        t.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);

        //covariance

        //publish the message
        _pub.publish(t);
        ros::spinOnce();
        r.sleep();
    }
}