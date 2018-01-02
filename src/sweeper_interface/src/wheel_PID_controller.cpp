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
            pwr_output_pub_ = n_.advertise<std_msgs::Float32>("pwr_cmd", 10);
            wheel_vel_pub_ = n_.advertise<std_msgs::Float32>("wheel_vel",10);
            

            //create subscribers
            vel_setpt_sub_ = n_.subscribe("wheel_cmd_vel",1 , &WheelPID::setPtCB, this);
            enc_ticks_sub_ = n_.subscribe("encoder_ticks",1 , &WheelPID::encTicksCB, this);
      
            //get parameters
            ros::param::param<double>("~Kp", Kp_, 256.0);
            ros::param::param<double>("~Ki", Ki_, 1.0);
            ros::param::param<double>("~Kd", Kd_, 1.0);
            ros::param::param<double>("pwr_out_min", pwr_out_min_, -255);
            ros::param::param<double>("pwr_out_max", pwr_out_max_, 255);
            ros::param::param<double>("wheel_dia", wheel_dia_, 0.075);
            ros::param::param<double>("ticks_per_rev", ticks_per_rev_, 40*13);
            ros::param::param<int>("encoder_min", encoder_min_, -32768);
            ros::param::param<int>("encoder_max", encoder_max_, 32768);
            ros::param::param<double>("~windup_limit", windup_limit_, 255);
            ros::param::param<double>("~min_vel", min_vel_, 0.1);
            ros::param::param<double>("~vel_cmd_timeout", vel_cmd_timeout_, 0.5);
            
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
            last_pid_update_=0;
            last_wheel_update_ = 0;
            last_vel_cmd_=0;
            
            meters_per_tick_ = 3.14159265359*wheel_dia_/ticks_per_rev_;
            encoder_wrap_low_ = (encoder_max_ - encoder_min_)*0.3 + encoder_min_;
            encoder_wrap_high_ = (encoder_max_ - encoder_min_)*0.7 + encoder_min_;

        }
        void setPtCB (const std_msgs::Float32 &vel)
        {
            //update setpoint
            if(vel.data != target_vel_ )
            {
                ROS_DEBUG("Received new velocity: %f", vel.data);
                target_vel_ = vel.data;
                //integral_ = 0;
            }
            
            last_vel_cmd_ = ros::Time::now().toSec();
            
        }
        void encTicksCB (const std_msgs::Int16 &new_ticks)
        {
                       
            //deal with encoder ticks wraparound.
            if(new_ticks.data<encoder_wrap_low_ && old_ticks_ > encoder_wrap_high_)
            {
                //+1 to the multiplier
                wrap_mult_ +=1;
                ROS_DEBUG("New wrap_mult: %d", wrap_mult_);
            }
             if(new_ticks.data>encoder_wrap_high_ && old_ticks_ < encoder_wrap_low_)
            {
                //-1 to the multiplier
                wrap_mult_ -=1;
                ROS_DEBUG("New wrap_mult: %d", wrap_mult_);

            }
             old_ticks_ = new_ticks.data;

            wheel_latest_ = new_ticks.data+wrap_mult_*(encoder_max_-encoder_min_);
           
            
            //estimate current velocity
            updateVel();
                      
            //update pid
            if(ros::Time::now().toSec() - last_vel_cmd_ > vel_cmd_timeout_)
            {
                target_vel_ = 0;           
                integral_=0;
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
            double now = ros::Time::now().toSec();
            double PID_dt = now-last_pid_update_;

            if(PID_dt == 0)
            {
                ROS_ERROR("PID_dt == 0; Skipping loop. now = %f, last = %f", now, last_pid_update_);
                return;
            }
            last_pid_update_ = now;  
            
            //calculate PID terms
            error_ = target_vel_ - cur_vel_;
            integral_ = integral_+error_*PID_dt;
            //apply windup limit
            if (integral_ > windup_limit_)
            {
                integral_ = windup_limit_;
                ROS_DEBUG("Integral windup limit reached integral_: %f", integral_);
            }
            if (integral_ < -windup_limit_)
            {
                integral_ = -windup_limit_;
                ROS_DEBUG("Integral windup limit reached integral_: %f", integral_);
            }
            derivative_  = (error_ - prev_error_)/PID_dt;
            prev_error_ = error_;
            
            //calculate new output
            pwr_out_ = Kp_*error_ + Ki_*integral_ + Kd_*derivative_;
            ROS_DEBUG("error,integral,derivative: %f,%f,%f",error_, integral_, derivative_);

              //attempt to delinearize using experimentally derived velocity-voltage curve **note this is a real hacky way of doing this**
// probably PID control is no good for these motors due to a very non-linear relationship between PWM duty cycle and rpm
            bool is_positive = pwr_out_ > 0;
            pwr_out_ = 33.3376*exp(4.675*fabsf(pwr_out_));            
            if(!is_positive)
            {
		pwr_out_ = -pwr_out_;
	    }

            //limit output to max/min values
            if (pwr_out_ > pwr_out_max_) 
            {
                pwr_out_ = pwr_out_max_;
                ROS_DEBUG("power output limit reached pwr_out_max_: %f", pwr_out_max_);

            }
            if (pwr_out_ < pwr_out_min_) 
            {
                pwr_out_ = pwr_out_min_;
                ROS_DEBUG("power output limit reached pwr_out_max_: %f", pwr_out_max_);
            }
            
            if (target_vel_ ==0)
            {
                pwr_out_ = 0;
                integral_ = 0;
               
            }
        }

        void updateVel()
        {
            //estimate velocity from meters_per_tick and dt
            float dt = ros::Time::now().toSec() - last_wheel_update_;
            
                
            if(wheel_latest_ == wheel_prev_) //low velocity
            {
                if(meters_per_tick_/dt < min_vel_)
                {
                    cur_vel_ = 0;
                    //ROS_DEBUG("Estimated velocity is too low: setting velocity to 0");

                }
            }else
            {
                last_wheel_update_  = ros::Time::now().toSec();
                cur_vel_ = ((wheel_latest_ - wheel_prev_)*meters_per_tick_/dt);
                wheel_prev_ = wheel_latest_;
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
        int wheel_latest_, wheel_prev_;
        double vel_cmd_timeout_;
        double   last_pid_update_, last_wheel_update_, last_vel_cmd_ ;
          
        //robot parameters
        double wheel_dia_, ticks_per_rev_, meters_per_tick_, min_vel_; // wheel diameter [m], ticks per full wheel rotation [ticks], approximate backlash [ticks]
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
