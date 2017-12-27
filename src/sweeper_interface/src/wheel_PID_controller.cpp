//addapted from https://github.com/jfstepha/differential-drive/blob/master/scripts/pid_velocity.py

/*
* Receives the number of encoder ticks as std_msgs/int16 and estimates wheel velocity.
*  Uses PID to match wheel velocity to set point (std_msgs/Float32
*/

#include "ros/ros.h"
#include "ros/console.h"

#include "math.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

class WheelPID
{
    public:
        WheelPID()
        {
            //create publishers
            pwr_output_pub_ = n_.advertise<std_msgs::Float32>("~pwr_cmd_", 10);
            wheel_vel_pub_ = n_.advertise<std_msgs::Float32>("~wheel_vel",10);

            //create subscribers
            vel_setpt_sub_ = n_.subscribe("~wheel_cmd_vel",1 , &WheelPID::setPtCB, this);
            enc_ticks_sub_ = n_.subscribe("~encoder_ticks",1 , &WheelPID::encTicksCB, this);
            
            //get parameters
            n_.param<double>("Kp", Kp_, 1.0);
            n_.param<double>("Ki", Ki_, 0.0);
            n_.param<double>("Kd", Kd_, 0.0);
            n_.param<double>("pwr_out_min", pwr_out_min_, 0);
            n_.param<double>("pwr_out_max", pwr_out_max_, 255);
            n_.param<double>("wheel_dia", wheel_dia_, 0.06);
            n_.param<double>("ticks_per_rev", ticks_per_rev_, 1260);
            n_.param<int>("encoder_min", encoder_min_, -32768);
            n_.param<int>("encoder_max", encoder_max_, 32768);
            n_.param<double>("backlash", backlash_, 48);
            n_.param<double>("windup_limit", windup_limit_, 48);
            n_.param<double>("min_vel", min_vel_, 0.1);
            n_.param<double>("vel_cmd_timeout", vel_cmd_timeout_, 0.25);
            windup_limit_ = fabsf(windup_limit_);
            
            //initialize variables
            
            target_vel_ = 0;
            cur_vel_ = 0;
            pwr_out_ = 0;
            error_=0;
            integral_=0;
            derivative_ =0;
            prev_error_ = 0;
            old_ticks_ = 0;
            wheel_latest_ = 0;
            wheel_prev_ = 0;
            
            meters_per_tick_ = 3.14159265359*wheel_dia_/ticks_per_rev_;
            encoder_wrap_low_ = (encoder_max_ - encoder_min_)*0.3 + encoder_min_;
            encoder_wrap_high_ = (encoder_max_ - encoder_min_)*0.7 + encoder_min_;

        }
        void setPtCB (const std_msgs::Float32 &vel)
        {
            //update setpoint
            target_vel_ = vel.data;
        }
        void encTicksCB (const std_msgs::Int16 &new_ticks)
        {
            //deal with encoder ticks wraparound.
            if(new_ticks.data<encoder_wrap_low_ && old_ticks_ > encoder_wrap_high_)
            {
                //+1 to the multiplier
                wrap_mult_ +=1;
            }
             if(new_ticks.data>encoder_wrap_high_ && old_ticks_ < encoder_wrap_low_)
            {
                //-1 to the multiplier
                wrap_mult_ -=1;
            }
            wheel_prev_ = wheel_latest_;
            wheel_latest_ = new_ticks.data+wrap_mult_*(encoder_max_-encoder_min_)* meters_per_tick_;
            old_ticks_ = new_ticks.data;
            
            //estimate current velocity
            updateVel();
                      
            //update pid
            if(ros::Time::now().toSec() - last_vel_cmd_ > vel_cmd_timeout_)
            {
                target_vel_ = 0;
            }
            updatePID();
            
            //publish output pwr
            std_msgs::Float32 pwr_out_msg;
            pwr_out_msg.data = pwr_out_;
            pwr_output_pub_.publish(pwr_out_msg);
            
        }
    private:
        void updatePID()
        {
            //calculate PID_dt
            float now = ros::Time::now().toSec();
            float PID_dt = now-last_pid_update_;
            last_pid_update_ = now;
            
            //calculate PID terms
            error_ = target_vel_ - cur_vel_;
            integral_ = integral_+error_*PID_dt;
            //apply windup limit
            if (integral_ > windup_limit_)
            {
                integral_ = windup_limit_;
            }
            if (integral_ < -windup_limit_)
            {
                integral_ = -windup_limit_;
            }
            derivative_  = (error_ - prev_error_)/PID_dt;
            prev_error_ = error_;
            
            //calculate new output
            pwr_out_ = Kp_*error_ + Ki_*integral_ + Kd_*derivative_;
            
            //limit output to max/min values
            if (pwr_out_ > pwr_out_max_) 
            {
                pwr_out_ = pwr_out_max_;
            }
            if (pwr_out_ < pwr_out_min_) 
            {
                pwr_out_ = pwr_out_min_;
            }
            
            if (target_vel_ ==0)
            {
                pwr_out_ = 0;
            }
        }
    
        void updateVel()
        {
            //estimate velocity from meters_per_tick and dt
            float dt = ros::Time::now().toSec() - last_wheel_update_;
            last_wheel_update_  = ros::Time::now().toSec();
                
            if(wheel_latest_ == wheel_prev_) //low velocity
            {
                if(fabsf(meters_per_tick_)/dt < min_vel_)
                {
                    cur_vel_ = 0;
                }
            }else
            {
                cur_vel_ = (wheel_latest_ - wheel_prev_)*meters_per_tick_/dt;
            }
            std_msgs::Float32 vel_msg;
            vel_msg.data = cur_vel_;
            wheel_vel_pub_.publish(vel_msg);
        }
    
        //node handle
        ros::NodeHandle n_;
        
        //pubs
        ros::Publisher pwr_output_pub_;
        ros::Publisher wheel_vel_pub_;
        
        //subs
        ros::Subscriber vel_setpt_sub_;
        ros::Subscriber enc_ticks_sub_;
    
        //PID parameters
        double Kp_,Ki_,Kd_;
        double pwr_out_min_, pwr_out_max_, windup_limit_;
        double target_vel_, cur_vel_, pwr_out_;  //groundspeed setpoint r(t) [m/s], current groundspeed process variable y(t) [m/s], power output control variable u(t) [0-255]
        double error_, integral_, derivative_, prev_error_;
        int old_ticks_, wrap_mult_;
        double wheel_latest_, wheel_prev_;
        double vel_cmd_timeout_;
        double last_pid_update_, last_wheel_update_, last_vel_cmd_ ;
            
        //robot parameters
        double wheel_dia_, ticks_per_rev_, meters_per_tick_, backlash_, min_vel_; // wheel diameter [m], ticks per full wheel rotation [ticks], approximate backlash [ticks]
        int encoder_min_, encoder_max_, encoder_wrap_low_, encoder_wrap_high_;
    
};

int main(int argc, char **argv)
{

    //Initiate ROS
    ros::init(argc, argv, "wheel_PID_controller");

    //Create an object of class castto that will take care of everything
    WheelPID WPIDObj;

    ros::spin();


    return 0;
}