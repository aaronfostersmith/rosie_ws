#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/LaserScan.h>

#include <sweeper_interface/laser_range.h>
#include <sweeper_interface/Imu_bytes.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>



//this class establishes subscribers and publishers for interpreting messages from Rosie
//there is a custom message for rosserial: minimal Imu message
//additionally, this class converts the Pose2D messages from optical sensor and encoders to /Odom msgs.
class InterpretMsgs
{
	
public:
    InterpretMsgs()
    {

        //create publishers
		ls_pub_ = n_.advertise<sensor_msgs::LaserScan>("/sweeper/scan", 100);
        imu_pub_ = n_.advertise<sensor_msgs::Imu>("imu/data_raw", 100);
        mag_pub_ = n_.advertise<sensor_msgs::MagneticField>("imu/mag", 100);
        enc_pub_ = n_.advertise<nav_msgs::Odometry>("/odom/enc", 100);
        mou_pub_ = n_.advertise<nav_msgs::Odometry>("/odom/mouse", 100);

        //subscriber
        laser_sub_ = n_.subscribe("/sweeper/rawLaser", 1, &InterpretMsgs::lasercallback, this);
        imu_sub_ = n_.subscribe("/sweeper/imu", 1, &InterpretMsgs::imucallback, this);
        enc_sub_ = n_.subscribe("/sweeper/encpose", 1, &InterpretMsgs::enccallback, this);
        mou_sub_ = n_.subscribe("/sweeper/mousepose", 1, &InterpretMsgs::moucallback, this);
        
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
        _ls_num_pts= 64;
        ROS_WARN("Failed to get num_pts param. Using 64 pts as default");
        }
         if (!ros::param::get("~time_increment", _ls_time_increment))
        {
        _ls_time_increment = 0.03;
        ROS_WARN("Failed to get time_increment param. Using 0.03 seconds as default");
        }
        
    }

    void lasercallback(const sweeper_interface::laser_range &raw)
    {
       
        //containers for PCL and ROS data types

		static sensor_msgs::LaserScan ls_temp;
        
          if (raw.seq == 0)
        {
            ls_temp.header.stamp = ros::Time::now();
		    ls_pub_.publish(ls_temp);
        }

        //populate header
        ls_temp.header.frame_id = "hokuyo";
	
		//set laserscan parameters
		ls_temp.angle_min = _ls_angle_min;
		ls_temp.angle_max = _ls_angle_max; //2*pi
		ls_temp.angle_increment = (ls_temp.angle_max-ls_temp.angle_min)/_ls_num_pts; //pi/2
		ls_temp.time_increment = _ls_time_increment;
		ls_temp.scan_time = ls_temp.time_increment*(float)_ls_num_pts;
		ls_temp.range_min = 0.03;
		ls_temp.range_max = 2.0;
		ls_temp.ranges.resize(_ls_num_pts); //4
		
		
        //fill with laserScan with num_pts data points

			//laserscan
		    ls_temp.ranges[raw.seq]= raw.range/1000.0;
    }

    void imucallback(const sweeper_interface::Imu_bytes &imub_input)
    {
        //create an objects for output
        sensor_msgs::Imu imu_output;
        sensor_msgs::MagneticField mag_output;

        //populate the output's headers
        imu_output.header.stamp = ros::Time::now();
        imu_output.header.frame_id = "base_link";
        mag_output.header.stamp = ros::Time::now();
        imu_output.header.frame_id = "base_link";

        //copy data from the input to the appropriate output fields
        //copy raw angular and linear imub_input
        imu_output.angular_velocity.x = imub_input.angular_velocity[0];
        imu_output.angular_velocity.y = imub_input.angular_velocity[1];
        imu_output.angular_velocity.z = imub_input.angular_velocity[2];
        imu_output.linear_acceleration.x = imub_input.linear_acceleration[0];
        imu_output.linear_acceleration.y = imub_input.linear_acceleration[1];
        imu_output.linear_acceleration.z = imub_input.linear_acceleration[2];
        /*	//covariances
        	imu_output.orientation_covariance = {{0.1,0.0,0.0},
        										 {0.0,0.1,0.0},
        										 {0.0,0.0,0.1}};
        	imu_output.angular_velocity_covariance = {{0.1,0.0,0.0},
        											  {0.0,0.1,0.0},
        											  {0.0,0.0,0.1}};
        	imu_output.linear_acceleration_covariance = {{0.1,0.0,0.0},
        												 {0.0,0.1,0.0},
        												 {0.0,0.0,0.1}};

        	//copy raw magnetic field data
        	mag_output.magnetic_field.x = imub_input.magnetic_field[0];
        	mag_output.magnetic_field.y = imub_input.magnetic_field[1];
        	mag_output.magnetic_field.z = imub_input.magnetic_field[2];
        	//covariance
        	mag_output.magnetic_field_covariance = {{0.1,0.0,0.0},
        											{0.0,0.1,0.0},
        											{0.0,0.0,0.1}};
        */
        //publish
        imu_pub_.publish(imu_output);
        mag_pub_.publish(mag_output);
    }

    void enccallback(const geometry_msgs::Pose2D &enc_input)
    {
        nav_msgs::Odometry enc_output;

        //populate header
        enc_output.header.stamp = ros::Time::now();
        enc_output.header.frame_id = "odom";
        //imu_output.header.child_frame_id = "/base_link";
        //copy over the data (ignore twist because it is derived data and Robot_Localization will not get any additional info if we calculate it here)
        enc_output.pose.pose.position.x = enc_input.x;
        enc_output.pose.pose.position.y = enc_input.y;
        //go from theta (an euler) to quaternion
        tf::Quaternion q = tf::createQuaternionFromRPY(0,0, enc_input.theta);
        enc_output.pose.pose.orientation.x = q.x();
        enc_output.pose.pose.orientation.y = q.y();
        enc_output.pose.pose.orientation.z = q.z();
        enc_output.pose.pose.orientation.w = q.w();

        /*	  //covariances
        	enc_output.pose.covariance = {0.1, 0, 0, 0, 0, 0,
                                             0, 0.1, 0, 0, 0, 0,
                                             0, 0, 0.1, 0, 0, 0,
                                             0, 0, 0, 0.1, 0, 0,
                                             0, 0, 0, 0, 0.1, 0,
                                             0, 0, 0, 0, 0, 0.1};

        	enc_output.twist.covariance = {0.1, 0, 0, 0, 0, 0,
                                              0, 0.1, 0, 0, 0, 0,
                                              0, 0, 0.1, 0, 0, 0,
                                              0, 0, 0, 0.1, 0, 0,
                                              0, 0, 0, 0, 0.1, 0,
                                              0, 0, 0, 0, 0, 0.1};
        */
        enc_pub_.publish(enc_output);

    }

    void moucallback(const geometry_msgs::Pose2D &mou_input)
    {
        nav_msgs::Odometry mou_output;

        //populate header
        mou_output.header.stamp = ros::Time::now();
        mou_output.header.frame_id = "odom";
        //imu_output.header.child_frame_id = "/base_link";
        //copy over the data (ignore twist because it is derived data and Robot_Localization will not get any additional info if we calculate it here)
        mou_output.pose.pose.position.x = mou_input.x;
        mou_output.pose.pose.position.y = mou_input.y;
        //go from theta (an euler) to quaternion
        tf::Quaternion q = tf::createQuaternionFromRPY(0,0, mou_input.theta);
        mou_output.pose.pose.orientation.x = q.x();
        mou_output.pose.pose.orientation.y = q.y();
        mou_output.pose.pose.orientation.z = q.z();
        mou_output.pose.pose.orientation.w = q.w();

        /*	  //covariances
        	mou_output.pose.covariance = {{0.1, 0, 0, 0, 0, 0},
        								  {0, 0.1, 0, 0, 0, 0},
        								  {0, 0, 0.1, 0, 0, 0},
        								  {0, 0, 0, 0.1, 0, 0},
        								   {0, 0, 0, 0, 0.1, 0},
        									{0, 0, 0, 0, 0, 0.1}};

        	enc_output.twist.covariance = {{0.1, 0, 0, 0, 0, 0},
                                             {0, 0.1, 0, 0, 0, 0},
                                              {0, 0, 0.1, 0, 0, 0},
                                              {0, 0, 0, 0.1, 0, 0},
                                              {0, 0, 0, 0, 0.1, 0},
                                              {0, 0, 0, 0, 0, 0.1}};
        */
        mou_pub_.publish(mou_output);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pc_pub_;
	ros::Publisher ls_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher mag_pub_;
    ros::Publisher enc_pub_;
    ros::Publisher mou_pub_;

    ros::Subscriber laser_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber enc_sub_;
    ros::Subscriber mou_sub_;
    
    //member params
    double _ls_angle_min;
    double _ls_angle_max;
    int _ls_num_pts;
    double _ls_time_increment;
    


};//End of class InterpretMsgs



int main(int argc, char **argv)
{

    //Initiate ROS
    ros::init(argc, argv, "rosie_interpreter");

    //Create an object of class castto that will take care of everything
    InterpretMsgs IMobj;

    ros::spin();


    return 0;
}
