#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <cmath>
#include <deque>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>

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

/***************************Copied Classes******************************/

// trapizoidal rule integral estimator of a state vector wrt time
class IntegralEstimator
{
	double Deltat;                           // time window of data to hold
	std::deque<ros::Time> timeBuffer;        // time data
	std::deque<Eigen::Vector3d> stateBuffer; // state data
	int stateSize;                           // size of the state
	
public:
	IntegralEstimator()
	{}
	
	IntegralEstimator(double DeltatInit, int stateSizeInit)
	{
		Deltat = DeltatInit;
		stateSize = stateSizeInit;
	}
	
	IntegralEstimator(const IntegralEstimator& integralEstimatorNew)
	{
		Deltat = integralEstimatorNew.Deltat;
		stateSize = integralEstimatorNew.stateSize;
	}
	
	IntegralEstimator operator =(const IntegralEstimator& integralEstimatorNew)
	{
		Deltat = integralEstimatorNew.Deltat;
		stateSize = integralEstimatorNew.stateSize;
		return *this;
	}
	
	// update the buffers with new data
	Eigen::Vector3d update(Eigen::Vector3d stateNew, ros::Time timeNew)
	{
		timeBuffer.push_back(timeNew);   // save the time
		stateBuffer.push_back(stateNew); // save the state
		
		Eigen::Vector3d stateBufferIntegral;                    // integral of the buffer
		stateBufferIntegral = Eigen::Vector3d::Zero(stateSize); // initialize to 0
		
		// use greater than 3 because trapezoidal rule for 2 data points doesnt make sense
		if (timeBuffer.size() >= 3)
		{
			// while the buffer is too big pop off the oldest data as long as it wont make 
			// the time on the buffer too small. compare with the second oldest data to ensure
			// the buffer stays large enough
			while ((timeBuffer.at(timeBuffer.size()-1) - timeBuffer.at(1)).toSec() >= Deltat)
			{
				timeBuffer.pop_front();
				stateBuffer.pop_front();
			}
			
			// if the buffer has enough time worth of data on it then calculate the 
			// integral of Y, and calculate the new Dx
			if ((timeBuffer.at(timeBuffer.size()-1) - timeBuffer.at(0)).toSec() >= Deltat)
			{
				for (int i = 0; i < timeBuffer.size()-1; i++)
				{
					stateBufferIntegral += 0.5*(timeBuffer.at(i+1) - timeBuffer.at(i)).toSec()*(stateBuffer.at(i+1) + stateBuffer.at(i));
				}
			}
		}
		
		return stateBufferIntegral;
	}
};

// LS estimator for a first order approximatoion of the derivative of a state vector wrt time, thanks Anup
class DerivativeEstimator
{    
    int bufferSize; //Number of data points to store
    int stateSize; //Number of elements for the state
    bool bufferFull; //Estimation will start after buffer is full for first time
    Eigen::VectorXd timeBuff; //ring buffer for time data
    Eigen::VectorXd stateBuff; //ring buffer for position data
    int timeInd; //index of oldest time data. Data at this index gets replaced with new data
    int stateInd; //index of oldest position data Data at this index gets replaced with new data
    bool firstUpdate;//indicates first update has not happened
    
public:
	DerivativeEstimator()
	{}

    DerivativeEstimator(int bufferSizeInit, int stateSizeInit)
    {
        //Initialize buffers
        bufferSize = bufferSizeInit;
        stateSize = stateSizeInit;
        timeInd = 0;
        stateInd = 0;
        timeBuff = Eigen::VectorXd::Zero(bufferSize);
        stateBuff = Eigen::VectorXd::Zero(stateSize*bufferSize);
        bufferFull = false;
        firstUpdate = true;
    }
    
    DerivativeEstimator(const DerivativeEstimator& derivativeEstimatorNew)
    {
        //Initialize buffers
        bufferSize = derivativeEstimatorNew.bufferSize;
        stateSize = derivativeEstimatorNew.stateSize;
        timeInd = 0;
        stateInd = 0;
        timeBuff = Eigen::VectorXd::Zero(bufferSize);
        stateBuff = Eigen::VectorXd::Zero(stateSize*bufferSize);
        bufferFull = false;
        firstUpdate = true;
    }
    
    DerivativeEstimator operator=(const DerivativeEstimator& derivativeEstimatorNew)
    {
        //Initialize buffers
        bufferSize = derivativeEstimatorNew.bufferSize;
        stateSize = derivativeEstimatorNew.stateSize;
        timeInd = 0;
        stateInd = 0;
        timeBuff = Eigen::VectorXd::Zero(bufferSize);
        stateBuff = Eigen::VectorXd::Zero(stateSize*bufferSize);
        bufferFull = false;
        firstUpdate = true;
        return *this;
    }
    
    Eigen::VectorXd update(Eigen::VectorXd newMeasure, ros::Time newTime)
    {
		// Picture courtesy of Anup
        // Setting up least squares problem A*theta = P. theta is made up of the coefficients for the best fit line,
        // e.g., X = Mx*T + Bx, Y = My*t + By, Z = Mz*t + Bz. Velocity is estimated as the slope of the best fit line, i.e., Vx = Mx, Vy = My, Vz = Mz. 
        // Each block of data is arranged like this:
        // [Xi]     [1, Ti,  0,  0,  0,  0] * [Bx]
        // [Yi]  =  [0,  0,  1, Ti,  0,  0]   [Mx]
        // [Zi]     [0,  0,  0,  0,  1, Ti]   [By]
        //  \/      \_____________________/   [My]
        //  Pi                 \/             [Bz]
        //                     Ai             [Mz]
        //                                     \/
        //                                   theta
        //
        // and then data is all stacked like this, where n is the buffer size:
        // [P1]     [A1] * [Bx]
        // [P2]  =  [A2]   [Mx]
        //  :        :     [By]
        // [Pn]     [An]   [My]
        //                 [Bz]
        //                 [Mz]

		firstUpdate = false;

        //Fill buffers
        timeBuff(timeInd) = newTime.toSec();
        stateBuff.segment(stateInd,stateSize) = newMeasure;
        
        //Increment index, roll back over
        timeInd = (timeInd+1)%bufferSize;
        stateInd = (stateInd + stateSize)%(stateSize*bufferSize);

        //If the index has rolled over once, the buffer is full
        if (timeInd == 0)
        {
            bufferFull = true;
        }

        Eigen::VectorXd stateDerivative = Eigen::VectorXd::Zero(stateSize,1);//initialize state derivative
        if (bufferFull)
        {
            // normalize time for numerical stability/accuracy of subsequent matrix inversion
            double delT = timeBuff.maxCoeff() - timeBuff.minCoeff();
            Eigen::VectorXd timeNorm = (timeBuff.array() - timeBuff.minCoeff())/delT;
		
			clock_t startTime = clock();
            // Solve LLS for best fit line parameters
            Eigen::MatrixXd stateA(stateSize*bufferSize,2*stateSize);
            for (int ii = 0; ii < bufferSize; ii++)
            {
				Eigen::MatrixXd newA = Eigen::MatrixXd::Zero(stateSize,2*stateSize);
				for (int jj = 0; jj < stateSize; jj++)
				{
					int thisColStart = 2*jj;
					newA.block(jj,thisColStart,1,2) << 1,timeNorm(ii);
				}
				
				stateA.block(ii*stateSize,0,stateSize,2*stateSize) = newA;
            }
            
            startTime = clock();

			Eigen::MatrixXd ATA = stateA.transpose()*stateA;
			Eigen::MatrixXd ATB = stateA.transpose()*stateBuff;
			Eigen::VectorXd theta = ATA.ldlt().solve(ATB);
			
			// Get state derivatives
            for (int ii = 0; ii < stateSize; ii++)
            {
				int oddElement = ii*2+1;
				stateDerivative(ii) = theta(oddElement)/delT;// rescaled to account for time normalization
			}
        }
        
        return stateDerivative;//return state derivative
    }

	//return the current index in the state
	bool isfirstUpdate()
	{
		return firstUpdate;
	}

	//return if the buffer is full indicating the estimate is good
	bool isbufferFull()
	{
		return bufferFull;
	}
	
	//reset the estimator
	void reset()
	{
		timeInd = 0;
        stateInd = 0;
        bufferFull = false;
        firstUpdate = true;
	}
};

// PID controller for a state vector error
class PID
{
	double kP;
	double kD;
	double kI;
	int derivativeBufferSize;
	double integralBufferSize;
	DerivativeEstimator derivativeEstimator;
	IntegralEstimator integralEstimator;
	int stateSize;

public:
	PID()
	{}
	
	PID(double kPInit, double kDInit, double kIInit, int derivativeBufferSizeInit, double integralBufferSizeInit, int stateSizeInit)
	{
		kP = kPInit;
		kD = kDInit;
		kI = kIInit;
		stateSize = stateSizeInit;
		derivativeBufferSize = derivativeBufferSizeInit;
		integralBufferSize = integralBufferSizeInit;
		derivativeEstimator = DerivativeEstimator(derivativeBufferSize,stateSize);
		integralEstimator = IntegralEstimator(integralBufferSize,stateSize);
	}
	
	PID(const PID& pidNew)
	{
		kP = pidNew.kP;
		kD = pidNew.kD;
		kI = pidNew.kI;
		stateSize = pidNew.stateSize;
		derivativeBufferSize = pidNew.derivativeBufferSize;
		integralBufferSize = pidNew.integralBufferSize;
		derivativeEstimator = DerivativeEstimator(derivativeBufferSize,stateSize);
		integralEstimator = IntegralEstimator(integralBufferSize,stateSize);
	}
	
	PID operator =(const PID& pidNew)
	{
		kP = pidNew.kP;
		kD = pidNew.kD;
		kI = pidNew.kI;
		stateSize = pidNew.stateSize;
		derivativeBufferSize = pidNew.derivativeBufferSize;
		integralBufferSize = pidNew.integralBufferSize;
		derivativeEstimator = DerivativeEstimator(derivativeBufferSize,stateSize);
		integralEstimator = IntegralEstimator(integralBufferSize,stateSize);
		return *this;
	}
	
	Eigen::VectorXd update(Eigen::Vector3d errorNew, ros::Time timeNew)
	{
		Eigen::Matrix3d kScale;
		kScale << 1.0,  0,  0,
				    0,1.0,  0,
				    0,  0,0.5;
		Eigen::Vector3d kPu = kP*kScale*errorNew;
		Eigen::Vector3d kDu = kD*kScale*derivativeEstimator.update(errorNew, timeNew);
		Eigen::Vector3d kIu = kI*kScale*integralEstimator.update(errorNew, timeNew);
		return kPu+kDu+kIu;
	}
	
};

class VelocityCommand
{
	ros::NodeHandle nh;
	ros::Subscriber mocapSub;
	std::string bebopName;
	ros::Publisher velPub;
	std::vector<double> linVelGains,angVelGains;
	int errorDerivativeBufferSize;
	int errorIntegralBufferSize;
	PID linVelPID;
	PID angVelPID;
	DerivativeEstimator velocities;//LS velocities estimator
	Eigen::Vector4d lastorientation;
	Eigen::Vector3d lastv;
	Eigen::Vector3d lastw;
	bool firstMocap;
	
public:
	VelocityCommand()
	{}

	VelocityCommand(std::vector<double> linVelGainsInit, std::vector<double> angVelGainsInit, int errorDerivativeBufferSizeInit, int errorIntegralBufferSizeInit, std::string bebopNameInit)
	{
		bebopName = bebopNameInit;
		linVelGains = linVelGainsInit;
		angVelGains = angVelGainsInit;
		errorDerivativeBufferSize = errorDerivativeBufferSizeInit;
		errorIntegralBufferSize = errorIntegralBufferSizeInit;
		linVelPID = PID(linVelGains.at(0),linVelGains.at(1),linVelGains.at(2),errorDerivativeBufferSize,errorIntegralBufferSize,3);
		angVelPID = PID(angVelGains.at(0),angVelGains.at(1),angVelGains.at(2),errorDerivativeBufferSize,errorIntegralBufferSize,3);
		velocities = DerivativeEstimator(errorDerivativeBufferSize,7);
		lastorientation = Eigen::Vector4d(1.0,0.0,0.0,0.0);
		lastv = Eigen::Vector3d::Zero();
		lastw = Eigen::Vector3d::Zero();
		firstMocap = true;
		mocapSub = nh.subscribe(bebopName + "/mocapPose",1,&VelocityCommand::mocapCB,this);
		velPub = nh.advertise<geometry_msgs::TwistStamped>(bebopName + "/body_vel",1);
		
	}
	
	VelocityCommand(const VelocityCommand& VelocityCommandN)
	{
		bebopName = VelocityCommandN.bebopName;
		linVelGains = VelocityCommandN.linVelGains;
		angVelGains = VelocityCommandN.angVelGains;
		errorDerivativeBufferSize = VelocityCommandN.errorDerivativeBufferSize;
		errorIntegralBufferSize = VelocityCommandN.errorIntegralBufferSize;
		linVelPID = PID(linVelGains.at(0),linVelGains.at(1),linVelGains.at(2),errorDerivativeBufferSize,errorIntegralBufferSize,3);
		angVelPID = PID(angVelGains.at(0),angVelGains.at(1),angVelGains.at(2),errorDerivativeBufferSize,errorIntegralBufferSize,3);
		velocities = DerivativeEstimator(errorDerivativeBufferSize,7);
		lastorientation = Eigen::Vector4d(1.0,0.0,0.0,0.0);
		lastv = Eigen::Vector3d::Zero();
		lastw = Eigen::Vector3d::Zero();
		firstMocap = true;
		mocapSub = nh.subscribe(bebopName + "/mocapPose",1,&VelocityCommand::mocapCB,this);
		velPub = nh.advertise<geometry_msgs::TwistStamped>(bebopName + "/body_vel",1);
	}
	
	VelocityCommand operator=(const VelocityCommand& VelocityCommandN)
	{
		bebopName = VelocityCommandN.bebopName;
		linVelGains = VelocityCommandN.linVelGains;
		angVelGains = VelocityCommandN.angVelGains;
		errorDerivativeBufferSize = VelocityCommandN.errorDerivativeBufferSize;
		errorIntegralBufferSize = VelocityCommandN.errorIntegralBufferSize;
		linVelPID = PID(linVelGains.at(0),linVelGains.at(1),linVelGains.at(2),errorDerivativeBufferSize,errorIntegralBufferSize,3);
		angVelPID = PID(angVelGains.at(0),angVelGains.at(1),angVelGains.at(2),errorDerivativeBufferSize,errorIntegralBufferSize,3);
		velocities = DerivativeEstimator(errorDerivativeBufferSize,7);
		lastorientation = Eigen::Vector4d(1.0,0.0,0.0,0.0);
		lastv = Eigen::Vector3d::Zero();
		lastw = Eigen::Vector3d::Zero();
		firstMocap = true;
		mocapSub = nh.subscribe(bebopName + "/mocapPose",1,&VelocityCommand::mocapCB,this);
		velPub = nh.advertise<geometry_msgs::TwistStamped>(bebopName + "/body_vel",1);
		return *this;
	}
	
	geometry_msgs::Twist update(Eigen::Vector3d linVelDes, Eigen::Vector3d angVelDes)
	{
		geometry_msgs::Twist u;
		if (firstMocap)
		{
			return u;
		}
		
		Eigen::Vector3d linVelError = linVelDes - lastv;
		Eigen::Vector3d angVelError = angVelDes - lastw;
		Eigen::Vector3d linVelCmd = linVelPID.update(linVelError,ros::Time::now());
		Eigen::Vector3d angVelCmd = angVelPID.update(angVelError,ros::Time::now());
		
		u.linear.x = linVelCmd(0) + 0.15*linVelDes(0);
		u.linear.y = linVelCmd(1) + 0.15*linVelDes(1);
		u.linear.z = linVelCmd(2) + linVelDes(2);
		u.angular.z = angVelCmd(2) + angVelDes(2);
		return u;
	}
	
	void mocapCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		Eigen::Vector3d position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
		Eigen::Vector4d orientation(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
		orientation /= orientation.norm();
		
		if (firstMocap)
		{
			lastorientation = orientation;
		}
		
		if ((lastorientation - orientation).norm() > ((lastorientation + orientation).norm()))
		{
			orientation *= -1.0;
		}
		lastorientation = orientation;
		Eigen::VectorXd newPose = Eigen::VectorXd::Zero(7);
		newPose.segment(0,3) = position;
		newPose.segment(3,4) = orientation;
		
		Eigen::VectorXd lastVel = velocities.update(newPose,msg->header.stamp);//update velocity
		Eigen::Vector3d lastLinVel = lastVel.segment(0,3);
		Eigen::Vector4d lastqDot = lastVel.segment(3,4);
		Eigen::MatrixXd Bq = getqDiff(orientation);//get the differential matrix for new orientation
		Eigen::Vector3d lastAngVelBody = 2*(Bq.transpose())*lastqDot;//get the angular velocity in the body frame
		lastv = (getqMat(getqInv(orientation))*getqMat(Eigen::Vector4d(0.0,lastLinVel(0),lastLinVel(1),lastLinVel(2)))*orientation).block(1,0,3,1);
		lastw = lastAngVelBody;
		geometry_msgs::TwistStamped lastvel;
		lastvel.header.stamp = msg->header.stamp;
		lastvel.twist.linear.x = lastv(0);
		lastvel.twist.linear.y = lastv(1);
		lastvel.twist.linear.z = lastv(2);
		lastvel.twist.angular.x = lastw(0);
		lastvel.twist.angular.y = lastw(1);
		lastvel.twist.angular.z = lastw(2);
		velPub.publish(lastvel);
		if (firstMocap)
		{
			firstMocap = false;
		}
	}

	
};

class BebopControl
{
	ros::NodeHandle nh;
	ros::Subscriber udSub, uSub;
	ros::Publisher uPub;
	std::string bebopName;
	VelocityCommand velocityCommand;
	int errorDerivativeBufferSize;
	int errorIntegralBufferSize;
	std::vector<double> linVelGains,angVelGains;
	double usat;
		
public:
	
	BebopControl()
	{
		//initialize
		ros::NodeHandle nhp("~");
		nhp.param<std::string>("bebopName",bebopName,"NONE");
		nhp.param<std::vector<double>>("linVelGains",linVelGains,{0.01, 0.01, 0.01});
		nhp.param<std::vector<double>>("angVelGains",angVelGains,{0.9, 0.3, 0.8});
		nhp.param<int>("errorDerivativeBufferSize",errorDerivativeBufferSize,10);
		nhp.param<int>("errorIntegralBufferSize",errorIntegralBufferSize,15);
		nhp.param<double>("usat",usat,0.1);
		udSub = nh.subscribe(bebopName + "/desTwist",1,&BebopControl::udCB,this);
		uPub = nh.advertise<geometry_msgs::Twist>(bebopName + "/des_vel_cmd",1);  // previously turtlebot0/cmd_vel_mux/input/navi
		VelocityCommand velocityCommandNew(linVelGains, angVelGains, errorDerivativeBufferSize, errorIntegralBufferSize, bebopName);
		velocityCommand = velocityCommandNew;
		std::cout << "linVelGains: " << linVelGains.at(0) << " " << linVelGains.at(1) << " " << linVelGains.at(2) << std::endl;
		std::cout << "low level usat: " << usat << std::endl;
	}
	void udCB(const geometry_msgs::TwistPtr& udTwistPtr)
	{
		geometry_msgs::Twist udTwist = *udTwistPtr;
		Eigen::Vector3d linVelDes(udTwist.linear.x,udTwist.linear.y,udTwist.linear.z);
		Eigen::Vector3d angVelDes(0.0,0.0,udTwist.angular.z);
		geometry_msgs::Twist ucmd = velocityCommand.update(linVelDes,angVelDes);

		if(ucmd.linear.x > usat) ucmd.linear.x = usat;
		if(ucmd.linear.x < -usat) ucmd.linear.x = -usat;
		if(ucmd.linear.y > usat) ucmd.linear.y = usat;
		if(ucmd.linear.y < -usat) ucmd.linear.y = -usat;
		if(ucmd.linear.z > usat) ucmd.linear.z = usat;
		if(ucmd.linear.z < -usat) ucmd.linear.z = -usat;
		uPub.publish(ucmd);
	}

		
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "bebop_control_node");
	
	BebopControl bebop_practice;
	ros::spin();
	return 0;
	
}
