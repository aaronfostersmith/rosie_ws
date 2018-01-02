
#include "ros/ros.h"
#include "ros/console.h"

#include "math.h"
#include <std_msgs/Int16.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class PoseBuilder
{
    public:
        PoseBuilder()
        {
            //publishers
            pose_pub_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/odom/encPose", 10);
            
            //subscribers
            lwheel_sub_ = n_.subscribe("/lwheel_ticks",1, &PoseBuilder::lwheelCB, this);
            rwheel_sub_ = n_.subscribe("/rwheel_ticks",1, &PoseBuilder::rwheelCB, this);
            
            //get parameters and set default values
            ros::param::param<int>("~ticks_per_rev", ticks_per_rev_, 40*13);
            ros::param::param<int>("~encoder_min", encoder_min_, -32768);
            ros::param::param<int>("~encoder_max", encoder_max_, 32768);
            ros::param::param<int>("~backlash", backlash_, 7);
            ros::param::param<int>("~rate", pub_rate_, 10);
            ros::param::param<double>("~base_width", base_width_ , 0.150);
            ros::param::param<double>("~wheel_diameter", wheel_dia_, 0.075);

            old_ticks_l_ = 0;
            old_ticks_r_ = 0;
            wheel_latest_l_=0;
            wheel_latest_r_ =0;
            wheel_prev_r_ =0;
            wheel_prev_l_ = 0;
            wrap_mult_l_ =0;
            wrap_mult_r_=0;
            
            
            //do some one time calculations
            meters_per_tick_ = 3.14159265359*wheel_dia_/ticks_per_rev_;
            encoder_wrap_low_ = (encoder_max_ - encoder_min_)*0.3 + encoder_min_;
            encoder_wrap_high_ = (encoder_max_ - encoder_min_)*0.7 + encoder_min_;
            
            posePublisher();
        }
    
        void lwheelCB(const std_msgs::Int16 &new_ticks_l)
        {
            int enc = new_ticks_l.data;
             //deal with encoder ticks wraparound.
            if(enc<encoder_wrap_low_ && old_ticks_l_ > encoder_wrap_high_)
            {
                wrap_mult_l_ +=1;
            }
             if(enc>encoder_wrap_high_ && old_ticks_l_ < encoder_wrap_low_)
            {
                wrap_mult_l_ -=1;
            }
            
             old_ticks_l_ = enc;

            wheel_latest_l_ = enc+wrap_mult_l_*(encoder_max_-encoder_min_);
            
            //TODO: implement backlash compensation
        }
        void rwheelCB(const std_msgs::Int16 &new_ticks_r)
        {
            int enc = new_ticks_r.data;
              //deal with encoder ticks wraparound.
            if(enc<encoder_wrap_low_ && old_ticks_r_ > encoder_wrap_high_)
            {
                wrap_mult_r_ +=1;
            }
             if(enc>encoder_wrap_high_ && old_ticks_r_ < encoder_wrap_low_)
            {
                wrap_mult_r_ -=1;
            }
            
             old_ticks_r_ = enc;

            wheel_latest_r_ = enc+wrap_mult_r_*(encoder_max_-encoder_min_);
            
            //TODO: implement backlash compensation
        }
    
    private:
        void posePublisher()
        {
            geometry_msgs::PoseWithCovarianceStamped t;
            //set some of the data which doesn't change
            {t.pose.pose.position.z = 0.0;
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
            t.pose.covariance[35] = 0.001; }
            ros::Rate r(pub_rate_);
            while(n_.ok())
            {
                //since all odometry is 6DOF we'll need a quaternion created from yaw
                // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

                //time stamp
                t.header.stamp = ros::Time::now();
                //set the position
                t.pose.pose.position.x = x_;
                t.pose.pose.position.y = y_;
                t.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th_);

                //publish the message
                pose_pub_.publish(t);
                ros::spinOnce();
                r.sleep();
            }
        }
        void updatePose()
        {
            double d_left = (wheel_latest_l_ - wheel_prev_l_)*meters_per_tick_;
            double d_right = (wheel_latest_r_ - wheel_prev_r_)*meters_per_tick_;
            wheel_prev_l_ = wheel_latest_l_;
            wheel_prev_r_ = wheel_latest_r_;
            
            //center distance traveled is ~ avg of d_left and d_right
            double d_c = (d_left+d_right)/2;
            
            //calculate x,y, and theta values
            if (d_c != 0)
            {
                x_ += d_c*cos(th_);
                y_ += d_c*sin(th_);
            }
            th_ += (d_right-d_left)/base_width_;
            
            if (th_ > 6.28318530718){
                th_ -= 6.28318530718;
            } else if(th_ <0){
                th_+=6.28318530718;
            }
        }
    
    //node handle
    ros::NodeHandle n_;
        
    //pubs
    ros::Publisher pose_pub_;
        
    //subs
    ros::Subscriber lwheel_sub_;
    ros::Subscriber rwheel_sub_;
    
    //parameters
    int ticks_per_rev_, encoder_min_, encoder_max_, backlash_, pub_rate_;
    double base_width_, wheel_dia_;
    
    //tick variables
    int old_ticks_l_, old_ticks_r_;
    int wheel_latest_l_, wheel_latest_r_;
    int wheel_prev_l_, wheel_prev_r_;
    int encoder_wrap_low_, encoder_wrap_high_, wrap_mult_l_, wrap_mult_r_;
    double meters_per_tick_;
    
    //pose variables
    double x_, y_, th_;
  
};


int main(int argc, char **argv)
{

    //Initiate ROS
    ros::init(argc, argv, "Pose_Builder");

    //Create an object of class castto that will take care of everything
    PoseBuilder PBObj;

    ros::spin();

    return 0;
}
