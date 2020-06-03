#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>


class Obstacle_trajectory
{
	ros::NodeHandle nh;
    ros::Timer controlLoop;
    ros::Publisher primaryPosePub;
    ros::Publisher primaryTwistPub;
    ros::Subscriber advanceSub,syncSub;
    geometry_msgs::Twist primaryTwist;
	geometry_msgs::PoseStamped primaryPose;
    std::string bebopName;//namespace for the bebop    
    double loopRate, centerx,centery, radius1, radius2, sec, t, phase_shift, height;
    Eigen::Vector3d xd,xdDot;
	bool reverse,start_exp;
    
    ros::Time time_init, time_last;
    
	public:
		Obstacle_trajectory()
		{
			ros::NodeHandle nhp("~");
			nhp.param<double>("loopRate",loopRate,300); //Hz
			nhp.param<double>("primary_radius1",radius1,1); 
			nhp.param<double>("primary_radius2",radius2,2); 
			nhp.param<double>("primary_centerx",centerx,0); 
			nhp.param<double>("primary_centery",centery,0); 
			nhp.param<double>("primary_height",height,1); 
			nhp.param<double>("secs_per_rev",sec,20);
			nhp.param<double>("phase_shift",phase_shift,0);
			nhp.param<bool>("reverse",reverse,0);
			nhp.param<std::string>("bebopName",bebopName,"NONE");
			syncSub = nh.subscribe("/sync",1,&Obstacle_trajectory::syncCB,this);
			primaryTwistPub = nh.advertise<geometry_msgs::Twist>(bebopName+"/desTwist",1);
			primaryPosePub = nh.advertise<geometry_msgs::PoseStamped>(bebopName+"/desPose",1);
			controlLoop = nh.createTimer(ros::Duration(1.0/loopRate),&Obstacle_trajectory::trajectoryCB,this);
			xdDot << 0,0,0;
			xd << 0,0,0;
			start_exp = false;
			t = 0;	
		}
		void syncCB(const std_msgs::Bool msg)
		{
			start_exp = msg.data;
			if(start_exp)
			{
				time_init = ros::Time::now();
			}
		}
		void trajectoryCB(const ros::TimerEvent& )
		{
			if(!start_exp) return;
			
			ros::Time time_now = ros::Time::now();
			t = time_now.toSec() - time_init.toSec();

	
			double w = 2*M_PI/sec;
			if(reverse) w *= -1.0;
					
			xdDot << -radius1*w*sin(w*t+phase_shift), radius2*w*cos(w*t+phase_shift),w;
			xd << radius1*cos(w*t+phase_shift)+centerx,radius2*sin(w*t+phase_shift)+centery,atan2f(xdDot(1),xdDot(0));


			
			// publish primaryPose	
			Eigen::Quaterniond q(Eigen::AngleAxisd(xd(2),Eigen::Vector3d::UnitZ()));
				
			primaryPose.header.stamp = time_now;
			primaryPose.header.frame_id = "world";
			primaryPose.pose.position.x = xd(0);
			primaryPose.pose.position.y = xd(1);
			primaryPose.pose.position.z = height;
			primaryPose.pose.orientation.x = q.x();
			primaryPose.pose.orientation.y = q.y();
			primaryPose.pose.orientation.z = q.z();
			primaryPose.pose.orientation.w = q.w();
		
			primaryPosePub.publish(primaryPose);
		
			// publish vd	
				
			primaryTwist.linear.x = xdDot(0);
			primaryTwist.linear.y = xdDot(1);
			primaryTwist.linear.z = 0;
			primaryTwist.angular.x = 0;
			primaryTwist.angular.y = 0;
			primaryTwist.angular.z = xdDot(2);
			
			primaryTwistPub.publish(primaryTwist);
		}
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Primary_trajectory_node");
    
    Obstacle_trajectory obj;
    
	ros::spin();
    return 0;
}
