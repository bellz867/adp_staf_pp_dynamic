#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//differential q matrix
Eigen::MatrixXd getqDiff(Eigen::Vector4d q)
{
	Eigen::MatrixXd qDiff(4,3);
	qDiff << -q(1), -q(2), -q(3),
			  q(0), -q(3),  q(2),
			  q(3),  q(0), -q(1),
			 -q(2),  q(1),  q(0);
	return qDiff;
}

//q as matrix
Eigen::Matrix4d getqMat(Eigen::Vector4d q)
{
	Eigen::Matrix4d qMat;
	qMat << q(0), -q(1), -q(2), -q(3),
			q(1),  q(0), -q(3),  q(2),
			q(2),  q(3),  q(0), -q(1),
			q(3), -q(2),  q(1),  q(0);
	return qMat;
}

//q inverse
Eigen::Vector4d getqInv(Eigen::Vector4d q)
{
	Eigen::Vector4d qInv;
	qInv << q(0), -q(1), -q(2), -q(3);
	return qInv;
}


class Predictor
{
	// ros handles, etc.
	ros::NodeHandle nh;
    ros::Timer controlLoop;
    ros::Publisher predictorPosePub;
    ros::Subscriber mocapSub, desPoseSub, desTwistSub;
    ros::Time time_init,time_last;
    
    // ros msgs
    geometry_msgs::Twist desTwist;
	geometry_msgs::PoseStamped predictorPose,bebopPose;
    
	// global variables    
    std::string bebopName;//namespace for the bebop
    Eigen::Vector3d statesP,statesHatP,statesDesL,statesDesA;
    Eigen::Vector4d statesQ,statesHatQ;
    std::vector<double> k1_gain,k2_gain,d_max;
    bool mocap;

	public:
		Predictor()
		{
			// node handle
			ros::NodeHandle nhp("~");
			
			// parameters
			nhp.param<std::string>("bebopName",bebopName,"bebop4");
			
			// initialize states and controls
			statesHatP << 0,0,0;
			statesHatQ << 0,0,0,0;
			statesDesL << 0,0,0;
			statesDesA << 0,0,0;
			
			// set initial boolean
			mocap = false;

			// subscribers
			mocapSub = nh.subscribe(bebopName+"/mocapPose",1,&Predictor::mocapCB,this);
			//desTwistSub = nh.subscribe(bebopName+"/cmd_vel",1,&Predictor::desTwistCB,this);
			desTwistSub = nh.subscribe(bebopName+"/desTwist",1,&Predictor::desTwistCB,this);
			
			// publishers
			predictorPosePub = nh.advertise<geometry_msgs::PoseStamped>(bebopName+"/predictorPose",1);
			
			// set time
			time_init = ros::Time::now();
			time_last = time_init;
		}
		void mocapCB(const geometry_msgs::PoseStampedPtr& pose)
		{	
			// Set current frame time
			ros::Time time_now = ros::Time::now();
				//std::cout << (time_now-time_init).toSec() << std::endl;

			// Get dt
			double dt = (time_now - time_last).toSec();
			
			// Store time and states
			time_last = time_now;
			
			
			// Get true pose (not used in sim)
			bebopPose = *pose;
			statesP << bebopPose.pose.position.x, bebopPose.pose.position.y, bebopPose.pose.position.z;
			statesQ << bebopPose.pose.orientation.w, bebopPose.pose.orientation.x, bebopPose.pose.orientation.y, bebopPose.pose.orientation.z;
	
			// First iteration
			if(!mocap)
			{	
				statesHatP = statesP;
				statesHatQ = statesQ;
				std::cout << "setting statesHatQ to statesQ" << std::endl;
				mocap = true;
				return;
			}
			// Update position
			statesHatP += statesDesL*dt;
			
			// Update orientation
			tf::Quaternion hatQ(statesHatQ(1),statesHatQ(2),statesHatQ(3),statesHatQ(0));
			tf::Quaternion dQ;
			dQ.setRPY(statesDesA(0)*dt,statesDesA(1)*dt,statesDesA(2)*dt);
			tf::Quaternion newHatQ = dQ*hatQ;
			
			statesHatQ(0) = newHatQ.getW();
			statesHatQ(1) = newHatQ.getX();
			statesHatQ(2) = newHatQ.getY();
			statesHatQ(3) = newHatQ.getZ();
			statesHatQ.normalize();	
		
			// publish predictorPose	
				
			predictorPose.header.stamp = time_now;
			predictorPose.header.frame_id = "world";
			predictorPose.pose.position.x = statesHatP(0);
			predictorPose.pose.position.y = statesHatP(1);
			predictorPose.pose.position.z = statesHatP(2);
			predictorPose.pose.orientation.x = statesHatQ(1);
			predictorPose.pose.orientation.y = statesHatQ(2);
			predictorPose.pose.orientation.z = statesHatQ(3);
			predictorPose.pose.orientation.w = statesHatQ(0);
		
			predictorPosePub.publish(predictorPose);		
		}
		void desTwistCB(const geometry_msgs::TwistPtr& desired)
		{
			desTwist = *desired;
			statesDesL << desTwist.linear.x,desTwist.linear.y,desTwist.linear.z;
			statesDesA << desTwist.angular.x,desTwist.angular.y,desTwist.angular.z;
		}
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Predictor");
    
    Predictor obj;
    
	ros::spin();
    return 0;
}

