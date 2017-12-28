// adapted from http://wiki.ros.org/differential_drive#twist_to_motors
/*
publishes: lwheel_velocity [m/s], rwheel_velocity [m/s] [Float32]
subscribes to: cmd_vel [twist]
parameters: base_width [m], publish_rate [hz]
*/
#include "ros/ros.h"
#include "ros/console.h"

#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

class TwistToVelocity
{
    public:
        TwistToVelocity()
        {
             //create publishers
            lwheel_vel_pub_ = n_.advertise<std_msgs::Float32>("lwheel_cmd_vel", 10);
            rwheel_vel_pub_ = n_.advertise<std_msgs::Float32>("rwheel_cmd_vel", 10);

            //create subscribers
            twist_sub_ = n_.subscribe("cmd_vel",1 , &TwistToVelocity::twistCB, this);
            
            //get parameters
            ros::param::param<double>("~base_width", base_width_, 0.150);
            ros::param::param<double>("~publish_rate", pub_rate_, 20);
            
            dx_ =0;
            dy_ =0;
            dr_ =0;        
            
        }
    
        void twistCB(const geometry_msgs::Twist &cmd_vel_msg)
        {
          dx_=  cmd_vel_msg.linear.x;
          dy_=  cmd_vel_msg.linear.y;
          dr_=  cmd_vel_msg.angular.z;
            
          std_msgs::Float32 left;
          std_msgs::Float32 right;
            
          left.data = (dx_-dr_*base_width_)/2;
          right.data = (dx_+dr_*base_width_)/2;
            
          lwheel_vel_pub_.publish(left);
          rwheel_vel_pub_.publish(right);

        }
        
        void publish_velocities()
        {
            
        }
    private:
    
    //node
    ros::NodeHandle n_;
    //pubs
    ros::Publisher lwheel_vel_pub_;
    ros::Publisher rwheel_vel_pub_;
    
    //subs
    ros::Subscriber twist_sub_;
    
    double base_width_, pub_rate_;
    double dx_, dy_, dr_;
};

int main(int argc, char **argv)
{

    //Initiate ROS
    ros::init(argc, argv, "twist_to_vel");

    //Create an object of class castto that will take care of everything
    TwistToVelocity TTVObj;

    ros::spin();

    return 0;
}