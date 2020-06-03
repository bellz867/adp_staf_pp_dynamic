#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <fstream>

class Obstacle
{
public:
	
	Eigen::Vector2d states;
	double ra;
	double rbar;
	double rd;
	double path_cx;
	double path_cy;
	double path_w;
	double path_r1;
	double path_r2;
	double path_phi;
	Obstacle(double a,double bar,double d, double x, double y, double w, double r1, double r2, double phi)
	{
		ra = a;
		rbar = bar;
		rd = d;
		path_cx = x;
		path_cy = y;
		path_w = w;
		path_r1 = r1;
		path_r2 = r2;
		path_phi = phi;		
		states << 0,0;
	}
	
	
};
class Obstacle_subscriber
{
	ros::NodeHandle nho;
	ros::Subscriber mocapPose;
	
	Eigen::Vector2d* pose;
	
	// World frame transformation
    double offset_x;
    double offset_y;
	
	public:
	Obstacle_subscriber(std::string bebopName, Eigen::Vector2d* vec, bool use_true_obstacles, double x, double y)
	{
		pose = vec;
		offset_x = x;
		offset_y = y;
		
		if(use_true_obstacles) mocapPose = nho.subscribe(bebopName+"/mocapPose",1,&Obstacle_subscriber::mocapCB,this);
		else mocapPose = nho.subscribe(bebopName+"/desPose",1,&Obstacle_subscriber::mocapCB,this);
	}
	void mocapCB(const geometry_msgs::PoseStampedConstPtr& msg)
	{
		*pose << msg->pose.position.x+offset_x, msg->pose.position.y+offset_y;
		
		return;
	}
};
class Agent_controller
{
    // ROS stuff
    ros::NodeHandle nh;
    ros::Publisher velCmdPub, desTwistPub;
    ros::Subscriber mocapSub_bbp, obstacleSub0, obstacleSub1,  obstacleSub2,syncSub;
    ros::Time time_now,time_last,time_start;
    
    
    // Topic prefix
    std::string bebopName;//namespace for the bebop
    
    
    // Parameters
    Eigen::VectorXd si;
    Eigen::VectorXd script_F;
    Eigen::VectorXd Rbar;
    Eigen::MatrixXd R;
	double usat;
    double repsilon;
    double l;
    double kc1,kc2,gamma1,beta,ka1,ka2,ku;
    double ra,rbar,rd;
    double N;
    double c_scale;
    double scale2;
    double dbar;
    double W_limit;
    double cost;
    double secs_per_rev;
    Eigen::VectorXd r;
	std::vector<Eigen::VectorXd> c;
    int n_of_obstacles;
	std::vector<double> kv;//lin command gain
	std::vector<double> kw;//ang command gain
	std::vector<double> weightQx;
	std::vector<double> weightQzSum;
	std::vector<double> init_desPose; //initial desired pose
	std::vector<Eigen::VectorXd> si_prime;
	std::vector<std::string> obstacleNames;
	bool start_exp, dataSaved, sync;
    	
    // States
    tf::Vector3 desPoseLin,ttbPoseLin;
    tf::Quaternion desPoseAng, lastDesPoseAng; 
    tf::Transform lastPose;
    geometry_msgs::TwistStamped desTwist;
    Eigen::VectorXd WcHat,WcHatDot;
    Eigen::VectorXd WaHat,WaHatDot;
    Eigen::MatrixXd	Gamma,Gammaa;
    Eigen::MatrixXd	GammaDot;
    Eigen::VectorXd GammaElements, GammaaElements;
    std::vector<Obstacle> obstacles;
    std::vector<Obstacle_subscriber *> obstacle_sub;
    tf::Vector3 mLin_last,mCmd;
    
    // World frame transformation
    double offset_x;
    double offset_y;

	// Save "time" << "agentStates" << "obstacleStates" << "WcHat" << "WaHat" << "uHat" << "vHat" << "thetaHat" << "cost" << "Yp*thetaHat" << "delta"
	std::vector<double> timeData,vHatData,costData,WcHatSigmaData;
	std::vector<Eigen::Vector2d> agentStatesData,obstacle1StatesData,uHatData,obstacle2StatesData,obstacle3StatesData;
	std::vector<Eigen::VectorXd> WcHatData,WaHatData, deltaData;

public:
    Agent_controller()
    {
        // Initialize parameters
        ros::NodeHandle nhp("~");
		nhp.param<std::vector<double>>("kv",kv,{0.3,0.05,0.05});
		nhp.param<std::vector<double>>("kw",kw,{0.2});
		nhp.param<std::vector<double>>("weightQx",weightQx,{5.0,1.0});
		nhp.param<std::vector<double>>("weightQzSum",weightQzSum,{2.0,2.0});
		nhp.param<std::vector<double>>("init_desPose",init_desPose,{1.0,1.0,1.0});
		nhp.param<std::string>("bebopName",bebopName,"bebop4");
		nhp.param<std::vector<std::string>>("obstacleNames",obstacleNames,{"bebop0","bebop2","bebop3"});
		nhp.param<int>("n_of_obstacles",n_of_obstacles,3);
		nhp.param<double>("ra",ra,0.3);
		nhp.param<double>("rbar",rbar,0.45);
		nhp.param<double>("rd",rd,1);
		nhp.param<double>("repsilon",repsilon,0.1);
		nhp.param<double>("secs_per_rev",secs_per_rev,40);
		nhp.param<double>("usat",usat,0.1);
		time_now = ros::Time::now();
		time_last = ros::Time::now();
		time_start = time_now;
		Obstacle_subscriber * obj;

		std::cout << "ra: " << ra << ", rbar: " << rbar << ", rd: " << rd << std::endl;
		
		/** TEMP - include in launch file?? **/
		
		std::random_device random_dev;
		std::mt19937 gen(random_dev());
		std::uniform_real_distribution<> dis;
		
		offset_x = -2.5;
		offset_y = 0;
		
		N = 1.0;
		kc1 = 0.05;
		kc2 = 0.75;
		gamma1 = 1;
		beta = 0.001;
		ka1 = 0.75;
		ka2 = 0.01;
		ku = 1;
		c_scale = 0.5;
		scale2 = 1;
		dbar = 0;
		c.push_back(c_scale * Eigen::Vector2d(0.0,-1.0));
		c.push_back(c_scale * Eigen::Vector2d(0.866,0.5));
		c.push_back(c_scale * Eigen::Vector2d(-0.866,-0.5));
		
		Rbar = Eigen::VectorXd(2);
		Rbar << 10.0,10.0;
		R = Rbar.asDiagonal();
		
		W_limit = 30;

		WaHat = Eigen::VectorXd(12);
		WaHat << 1,1,1,1,1,1,1,1,1,1,1,1;
		WaHat *= 1;
		
		WaHatDot = Eigen::VectorXd(12);
		WaHatDot << 0,0,0,0,0,0,0,0,0,0,0,0;
		
		dis = std::uniform_real_distribution<>(0.0,1.0);
				
		WcHat = Eigen::VectorXd(12);
		//WcHat << dis(gen),dis(gen),dis(gen),dis(gen),dis(gen),dis(gen),dis(gen),dis(gen),dis(gen),dis(gen),dis(gen),dis(gen);
		WcHat << 0.056458, 0.793683, 0.541452, 0.188974, 0.915815, 0.438305, 0.285895, 0.249507, 0.893903, 0.497887, 0.445384, 0.231759;
		//WcHat << 1,1,1,1,1,1,1,1,1,1,1,1;
		WcHat *= 4;
		
		WcHatDot = Eigen::VectorXd(12);
		WcHatDot << 0,0,0,0,0,0,0,0,0,0,0,0;
		
		si = Eigen::VectorXd(3);
		si(0) = 0.0;
		si(1) = 0.0;
		si(2) = 0.0;
		
		script_F = Eigen::VectorXd(3);
		script_F(0) = 0.0;
		script_F(1) = 0.0;
		script_F(2) = 0.0;
		
		Eigen::Vector2d s_prime;
		s_prime << 0.0,0.0;
		si_prime.push_back(s_prime);
		si_prime.push_back(s_prime);
		si_prime.push_back(s_prime);
		
		GammaElements = Eigen::VectorXd(12);
		GammaElements << 1,1,1,1,1,1,1,1,1,1,1,1;
		Gamma = GammaElements.asDiagonal();
		Gamma *= 1;
		
		GammaaElements = Eigen::VectorXd(12);
		GammaaElements << 1,1,1,1,1,1,1,1,1,1,1,1;
		Gammaa = GammaElements.asDiagonal();
		
		GammaDot.setZero(12,12);
		
		cost = 0;
		r = Eigen::VectorXd(1);
		r << 0;
		
		/** End TEMP **/
		
		
		std::cout << "Controlling: " << bebopName << std::endl;
		
		
		// Initialize obstacles	
		for(int i = 0; i < n_of_obstacles; i++)
		{
			Obstacle obstacle = Obstacle(ra,rbar,rd,0,0,0,0,0,0);
			obstacles.push_back(obstacle);
		}
		for(int i = 0; i < n_of_obstacles; i++)
		{
			obj = new Obstacle_subscriber(obstacleNames.at(i),&obstacles.at(i).states,false, offset_x, offset_y);
			obstacle_sub.push_back(obj);
		}
		obstacles.at(0).path_cx = 0;
		obstacles.at(0).path_cx = 0;
		obstacles.at(0).path_r1 = 1.5;
		obstacles.at(0).path_r2 = 1.5;
		obstacles.at(0).path_w = 1/secs_per_rev;
		obstacles.at(0).path_phi = 1.5;
		obstacles.at(1).path_cx = 0.5;
		obstacles.at(1).path_cx = 0;
		obstacles.at(1).path_r1 = 1.0;
		obstacles.at(1).path_r2 = 0.5;
		obstacles.at(1).path_w = -1/secs_per_rev;
		obstacles.at(1).path_phi = 3.14;
		obstacles.at(2).path_cx = -1.5;
		obstacles.at(2).path_cx = 0;
		obstacles.at(2).path_r1 = 1.0;
		obstacles.at(2).path_r2 = 0.5;
		obstacles.at(2).path_w = 1/secs_per_rev;
		obstacles.at(2).path_phi = 3.14;
				
	
		std::cout << "Avoiding: " << n_of_obstacles << " obstacles." << std::endl;
		
		start_exp = false;	
		dataSaved = false;	
		sync = false;
		
        // Initialize states
		desPoseLin.setX(init_desPose.at(0));
		desPoseLin.setY(init_desPose.at(1));
		desPoseLin.setZ(init_desPose.at(2));
		desPoseAng.setX(0);
		desPoseAng.setY(0);
		desPoseAng.setZ(0);
		desPoseAng.setW(1);
		ttbPoseLin = tf::Vector3(0,0,0);
		mLin_last = tf::Vector3(0,0,0);
		mCmd = tf::Vector3(0,0,0);
		lastDesPoseAng = desPoseAng;
            
        // Subscribers
        
        //mocapSub_bbp = nh.subscribe(bebopName+"/predictorPose",1,&Agent_controller::mocapCB,this);
        mocapSub_bbp = nh.subscribe(bebopName+"/mocapPose",1,&Agent_controller::mocapCB,this);
        syncSub = nh.subscribe("/sync",1,&Agent_controller::syncCB,this);
        
        // Publishers
        desTwistPub = nh.advertise<geometry_msgs::Twist>(bebopName+"/desTwist",1);
        
        
        std::cout << "Initial position: " << std::endl;
		std::cout << desPoseLin.getX() << " " << desPoseLin.getY() << " " << desPoseLin.getZ() << std::endl;
		
		std::cout << "WeightQx: " << weightQx.at(0) << " " << weightQx.at(1)  << std::endl; 
		std::cout << "WeightQzSum: " << weightQzSum.at(0) << " " << weightQzSum.at(1) << std::endl; 
		std::cout << "r_epsilon: " << repsilon << std::endl; 
  		std::cout << "usat: " << usat << std::endl;
        std::cout << "End of initialization." << std::endl;
        
    }
    void syncCB(const std_msgs::Bool msg)
    {
		start_exp = msg.data;
		if(!sync)
		{
			time_start = ros::Time::now();
			sync = true;
		}
	}
    void mocapCB(const geometry_msgs::PoseStampedConstPtr& pose)
    {	
		if(!sync){
		
		time_now = ros::Time::now();
		double dt = (time_now-time_last).toSec();
		
		Eigen::Vector2d states;
		states << pose->pose.position.x, pose->pose.position.y;
		
		double mx = (desPoseLin.getX()-pose->pose.position.x);
		double my = (desPoseLin.getY()-pose->pose.position.y);
		double mz = (desPoseLin.getZ()-pose->pose.position.z);
		
		tf::Vector3 mLin(mx,my,mz);
				
		if(mLin.length() < 0.1)
		{
			updateVel(tf::Vector3(0,0,0),tf::Quaternion(0,0,0,1)); // Pulish control commands in world frame
			char wait_char;
			std::cout << "Bebop ready! Go?" << std::endl;
			return;

		}		
				
		tf::Vector3 mDeriv = (mLin-mLin_last)/dt;
		
		mCmd.setX(kv.at(0)*mLin.getX()+kv.at(2)*mDeriv.getX());
		mCmd.setY(kv.at(0)*mLin.getY()+kv.at(2)*mDeriv.getY());
		mCmd.setZ(kv.at(0)*mLin.getZ()+kv.at(2)*mDeriv.getZ());
		
		
		
		/** Output control to body frame **/
		
		tf::Transform tfPose, tfRot;
		tfPose.setOrigin(tf::Vector3(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z));
		tfPose.setRotation(tf::Quaternion(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w));
		
		tfRot.setOrigin(tf::Vector3(0,0,0));
		tfRot.setRotation(tfPose.getRotation());
		
		tf::Vector3 vLin = tfRot.inverse()*mCmd;
		
		bool sat = 1;
		double vLinCap = 1.2*usat;
		
		if(sat)
		{
			if(vLin.getX()>vLinCap) vLin.setX(vLinCap);
			if(vLin.getY()>vLinCap) vLin.setY(vLinCap);
			if(vLin.getZ()>vLinCap) vLin.setZ(vLinCap);
			if(vLin.getX()<-vLinCap) vLin.setX(-vLinCap);
			if(vLin.getY()<-vLinCap) vLin.setY(-vLinCap);
			if(vLin.getZ()<-vLinCap) vLin.setZ(-vLinCap);
		}
		
		tf::Quaternion rot(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w);
		tf::Quaternion qTilde = desPoseAng.inverse()*rot;
		tf::Quaternion wd = tf::Quaternion(desTwist.twist.angular.x,desTwist.twist.angular.y,desTwist.twist.angular.z,0);
		tf::Quaternion vAng = qTilde*(-kw.at(0))+qTilde.inverse()*wd*qTilde;
		
		Eigen::Vector2d mCmdEigen;
		mCmdEigen << vLin.getX(), vLin.getY();
		
		//updateVel(mCmd,vAng); // Pulish control commands in world frame
		
		updateVel(vLin,vAng); // Publish control commands in body frame
		
		time_last = time_now;
		mLin_last = mLin;
		lastDesPoseAng = desPoseAng;
		lastPose = tfPose;
		
		}
		else{	
		time_now = ros::Time::now();
		double dt = (time_now-time_last).toSec();
		
		Eigen::Vector2d states;
		states << pose->pose.position.x, pose->pose.position.y;
		
		// Transform world frame;
		states(0) += offset_x;
		states(1) += offset_y;
		
		
		// Check complete and save data
		if(!start_exp && !dataSaved)
			{
			
				std::ofstream writeFile("/home/ncr/experiment_data/adp_staf_pp_dynamic/saved_data/auto/data.txt");
				if (writeFile.is_open())
				{
					//std::vector<double> timeData,vHatData,costData,YpHatData, deltaData;
					//std::vector<Eigen::Vector2d> agentStatesData,obstacleStatesData,uHatData;
					//std::vector<Eigen::VectorXd> WcHatData,WaHatData,thetaHatData
					writeFile << "time," << "agentStatesX," << "agentStatesY," << "obstacle1StatesX," << "obstacle1StatesY," << "obstacle2StatesX," << "obstacle2StatesY,"
							  << "obstacle3StatesX," << "obstacle3StatesY," << "uHatX," << "uHatY," 
							  << "WcHat1," << "WcHat2," << "WcHat3," << "WcHat4," << "WcHat5," << "WcHat6,"
							  << "WcHat7," << "WcHat8," << "WcHat9," << "WcHat10," << "WcHat11," << "WcHat12,"
							  << "WaHat1," << "WaHat2," << "WaHat3," << "WaHat4," << "WaHat5," << "WaHat6,"
							  << "WaHat7," << "WaHat8," << "WaHat9," << "WaHat10," << "WaHat11," << "WaHat12,"
							  << "vHat," << "WcHatSigma," << "cost," << "delta" << "\n";
					for (int i = 0; i < timeData.size(); i++)
					{
						writeFile << timeData.at(i) << "," << agentStatesData.at(i)(0) << "," << agentStatesData.at(i)(1) << "," 
								  << obstacle1StatesData.at(i)(0) << "," << obstacle1StatesData.at(i)(1) << "," 
								  << obstacle2StatesData.at(i)(0) << "," << obstacle2StatesData.at(i)(1) << ","
								  << obstacle3StatesData.at(i)(0) << "," << obstacle3StatesData.at(i)(1) << ","
								  << uHatData.at(i)(0) << "," << uHatData.at(i)(1) << ","
								  << WcHatData.at(i)(0) << "," << WcHatData.at(i)(1) << "," << WcHatData.at(i)(2) << "," << WcHatData.at(i)(3) << "," << WcHatData.at(i)(4) << "," << WcHatData.at(i)(5) << "," 
								  << WcHatData.at(i)(6) << "," << WcHatData.at(i)(7) << "," << WcHatData.at(i)(8) << "," << WcHatData.at(i)(9) << "," << WcHatData.at(i)(10) << "," << WcHatData.at(i)(11) << "," 
								  << WaHatData.at(i)(0) << "," << WaHatData.at(i)(1) << "," << WaHatData.at(i)(2) << "," << WaHatData.at(i)(3) << "," << WaHatData.at(i)(4) << "," << WaHatData.at(i)(5) << "," 
								  << WaHatData.at(i)(6) << "," << WaHatData.at(i)(7) << "," << WaHatData.at(i)(8) << "," << WaHatData.at(i)(9) << "," << WaHatData.at(i)(10) << "," << WaHatData.at(i)(11) << "," 
								  << vHatData.at(i) << "," << WcHatSigmaData.at(i) << "," << costData.at(i) << "," << deltaData.at(i) << "\n";
					}
				writeFile.close();
				std::cout << "Data recorded" << std::endl;
				dataSaved = true;
				return;
				}
			}
		
		
		// Check si
		checksi(states);
		script_FCal(states);
		
		// Update NN weights
		WaHat += WaHatDot*dt;
		WcHat += WcHatDot*dt;
		Gamma += GammaDot*dt;
		cost += r(0)*dt;
		
		for(int i = 0; i < WaHat.rows(); i++)
		{
			WaHat(i) = WaHat(i) > W_limit ? W_limit : WaHat(i);
			WaHat(i) = WaHat(i) < -W_limit ? -W_limit : WaHat(i);
			WcHat(i) = WcHat(i) > W_limit ? W_limit : WcHat(i);
			WcHat(i) = WcHat(i) < -W_limit ? -W_limit : WcHat(i);
		}
		
		
		// Find statesk
		std::random_device rd;
		std::mt19937 gen(rd());
		double nm = (states.transpose()*states+dbar)/(1+scale2*states.transpose()*states);
		std::uniform_real_distribution<> dis(-nm*0.1,nm*0.1);
		
		Eigen::Vector2d k;
		k << dis(gen), dis(gen);
		Eigen::Vector2d statesk = states+k;
		
		/** RANDOM K **/
		
		
		// Calculate controller uHat
		Eigen::Vector2d uHat = uHatCal(states);
		
		// Calculate controller uHatk
		Eigen::Vector2d uHatk = uHatCal(statesk);
		
		// Calculate auxiliary signals
		Eigen::VectorXd wk(12);
		wk = gradient_sigma(statesk)*(F(statesk)+G(statesk)*uHatCal(statesk));
		
		Eigen::VectorXd wpk(12);
		wpk = gradient_pa(statesk).transpose()*(F(statesk)+G(statesk)*uHatCal(statesk));

		Eigen::VectorXd rk(1);
		rk = Qx(statesk) + QzSum() + Phi(uHatk) + P(statesk);
	
		Eigen::VectorXd deltak = WcHat.transpose()*wk + wpk + rk;


		Eigen::VectorXd w(12);
		w = gradient_sigma(states)*(F(states)+G(states)*uHatCal(states));
		
		Eigen::VectorXd wp(12);
		wp = gradient_pa(states).transpose()*(F(states)+G(states)*uHatCal(states));

		r = Qx(states) + QzSum() + Phi(uHat) + P(states);
		
		
		Eigen::VectorXd delta = WcHat.transpose()*w + wp + r;
		
		
		double rho = 1+gamma1*dotX(w,w);
		double rhok = 1+gamma1*dotX(wk,wk);

		// Update law for WcHat
		Eigen::VectorXd sum(12);
		sum << 0,0,0,0,0,0,0,0,0,0,0,0;
		for(int i =0;i < N;i++)
		{
			sum += (1/rhok)*wk*deltak;
			
		}
		
		WcHatDot = -Gamma*(((kc1*w)/rho)*delta+(kc2/N)*sum);

		// Update law for WaHat
		sum << 0,0,0,0,0,0,0,0,0,0,0,0;
		for(int i =0;i < N;i++)
		{
			sum += Ga1(statesk)*(1/rhok)*wk.transpose()*WcHat;
			
		}		
		WaHatDot = -Gammaa*(ka1*(WaHat-WcHat)+ka2*WaHat+(kc1/rho)*Ga1(states)*wk.transpose()*WcHat+(kc2/N)*sum);
				

		// Update law for Gamma
		Eigen::MatrixXd sumMat;
		sumMat.setZero(12,12);
		for(int i =0;i < N;i++)
		{
			sumMat += (1/pow(rhok,2.0))*(wk*wk.transpose())*Gamma;
			
		}
		GammaDot = beta*Gamma - kc1*Gamma*(1/pow(rho,2.0))*(w*w.transpose())*Gamma - (kc2/N)*Gamma*sumMat;
	

		// Ouput uHat to control command
		mCmd.setX(uHat(0));
		mCmd.setY(uHat(1));
			
		
		/** Regulate Z **/

		double mz = (desPoseLin.getZ()-pose->pose.position.z);

		tf::Vector3 mLin(0,0,mz);
				
		tf::Vector3 mDeriv = (mLin-mLin_last)/dt;
		
		mCmd.setZ(kv.at(0)*mLin.getZ()+kv.at(2)*mDeriv.getZ());
		
		/** Output control to body frame **/
		
		tf::Transform tfPose, tfRot;
		tfPose.setOrigin(tf::Vector3(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z));
		tfPose.setRotation(tf::Quaternion(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w));
		
		tfRot.setOrigin(tf::Vector3(0,0,0));
		tfRot.setRotation(tfPose.getRotation());
		
		tf::Vector3 vLin = tfRot.inverse()*mCmd;
		
		bool sat = 1;
		double vLinCap = 1.2*usat;
		
		if(sat)
		{
			if(vLin.getX()>vLinCap) vLin.setX(vLinCap);
			if(vLin.getY()>vLinCap) vLin.setY(vLinCap);
			if(vLin.getZ()>vLinCap) vLin.setZ(vLinCap);
			if(vLin.getX()<-vLinCap) vLin.setX(-vLinCap);
			if(vLin.getY()<-vLinCap) vLin.setY(-vLinCap);
			if(vLin.getZ()<-vLinCap) vLin.setZ(-vLinCap);
		}
		
		tf::Quaternion rot(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w);
		tf::Quaternion qTilde = desPoseAng.inverse()*rot;
		tf::Quaternion wd = tf::Quaternion(desTwist.twist.angular.x,desTwist.twist.angular.y,desTwist.twist.angular.z,0);
		tf::Quaternion vAng = qTilde*(-kw.at(0))+qTilde.inverse()*wd*qTilde;
		
		Eigen::Vector2d mCmdEigen;
		mCmdEigen << vLin.getX(), vLin.getY();
		
		//updateVel(mCmd,vAng); // Pulish control commands in world frame
		
		updateVel(vLin,vAng); // Publish control commands in body frame
		
		bool debug = 0;
		if(debug)
		{
			std::cout << "Gradient_Pa: " << gradient_pa(states) << std::endl << std::endl;

		}
		time_last = time_now;
		mLin_last = mLin;
		lastDesPoseAng = desPoseAng;
		lastPose = tfPose;
		
		double vHat = (WcHat.transpose()*sigma(states))(0)+pa(states);
		double WcHatSigma = (WcHat.transpose()*sigma(states))(0);
		/** Output Text Block **/
		if(!dataSaved){
			std::cout << "==="
					  << std::endl
					  
					  << std::setw(30) << std::left << "Agent position:"
					  << std::setw(30) << std::left << "Obstacle 1 position:"
					  << std::setw(30) << std::left << "Obstacle 2 position:"
					  << std::setw(30) << std::left << "Obstacle 3 position:"
					  << std::endl;
			for(int i = 0 ; i < states.rows() ; i++){
				std::cout << std::setw(30) << std::left << states(i)
						  << std::setw(30) << std::left << obstacles.at(0).states(i)
						  << std::setw(30) << std::left << obstacles.at(1).states(i)
						  << std::setw(30) << std::left << obstacles.at(2).states(i)
						  << std::endl;
			}
			std::cout << std::endl
					  << std::setw(30) << std::left << "Controller uHat:"
					  << std::setw(30) << std::left << "uHat in body:"
					  << std::setw(30) << std::left << "Random offset k:"
					  << std::endl;
			for(int i = 0; i < uHat.rows(); i++){
				std::cout << std::setw(30) << std::left << uHat(i)
						  << std::setw(30) << std::left << mCmdEigen(i)
						  << std::setw(30) << std::left << k(i)
						  << std::endl;
			}
			std::cout << std::endl
					  << std::setw(30) << std::left << "WcHat:"
					  << std::setw(30) << std::left << "WaHat:"
					  << std::setw(30) << std::left << "WcHatDot:"
					  << std::setw(30) << std::left << "WaHatDot:"
					  << std::endl;
			for(int i = 0; i < WcHat.rows(); i++){
				std::cout << std::setw(30) << std::left << WcHat(i)
						  << std::setw(30) << std::left << WaHat(i)
						  << std::setw(30) << std::left << WcHatDot(i)
						  << std::setw(30) << std::left << WaHatDot(i)
						  << std::endl;
			}				  
			std::cout << std::endl
					  << std::setw(30) << std::left << "Si:"
					  << std::endl;
			for(int i = 0; i < si.rows(); i++){
				std::cout << std::setw(30) << std::left << si(i)
						  << std::endl;
			}				  
			std::cout << std::endl
					  << std::setw(30) << std::left << "Cost:"
					  << std::endl;
			for(int i = 0; i < r.rows(); i++){
				std::cout << std::setw(30) << std::left << cost
						  << std::endl;
			}				  
			std::cout << std::endl;						
			  
				  
		}
		/** End Ouput Text Block **/
		
		// Save data
		if(!dataSaved)
		{
			//std::vector<double> timeData,vHatData,costData,YpHatData, deltaData;
			//std::vector<Eigen::Vector2d> agentStatesData,obstacleStatesData,uHatData;
			//std::vector<Eigen::VectorXd> WcHatData,WaHatData;
			
			timeData.push_back((time_now-time_start).toSec());
			vHatData.push_back(vHat);
			WcHatSigmaData.push_back(WcHatSigma);
			costData.push_back(cost);
			deltaData.push_back(delta);
			agentStatesData.push_back(states);
			obstacle1StatesData.push_back(obstacles.at(0).states);
			obstacle2StatesData.push_back(obstacles.at(1).states);
			obstacle3StatesData.push_back(obstacles.at(2).states);
			uHatData.push_back(uHat);
			WcHatData.push_back(WcHat);
			WaHatData.push_back(WaHat);
		}
		
	}
	}
	
	void updateVel(tf::Vector3 vLin, tf::Quaternion w)// update this bebops velocity command
	{
		double vx = vLin.getX();
		double vy = vLin.getY();
		double vz = vLin.getZ();
		double wx = w.getX();
		double wy = w.getY();
		double wz = w.getZ();
		
		geometry_msgs::Twist velCmd;
		velCmd.linear.x = vx;
		velCmd.linear.y = vy;
		velCmd.linear.z = vz;
		velCmd.angular.x = wx;
		velCmd.angular.y = wy;
		velCmd.angular.z = wz;
		desTwistPub.publish(velCmd);
	}
	Eigen::VectorXd sigma(Eigen::VectorXd states)
	{
		Eigen::VectorXd a((1+n_of_obstacles)*3);

		a << sigma0(states),
			 si(0)*sigma0(obstacles.at(0).states),
			 si(1)*sigma0(obstacles.at(1).states),
			 si(2)*sigma0(obstacles.at(2).states);

		return a;
	}
	Eigen::VectorXd sigma0(Eigen::VectorXd states)
	{
		Eigen::VectorXd a(3);
		double nm = (states.transpose()*states+dbar)/(1+scale2*states.transpose()*states);
		a << dotX(states,states+nm*c.at(0)), dotX(states,states+nm*c.at(1)), dotX(states,states+nm*c.at(2));
		return a;
	}
	Eigen::MatrixXd sigma0_prime(Eigen::VectorXd states)
	{
		double nm = (states.transpose()*states+dbar)/(1+scale2*states.transpose()*states);
		Eigen::MatrixXd a(3,2);
		a << (2*states+nm*c.at(0)+2*(1-scale2*dbar)*states*c.at(0).transpose()*states/pow((1+scale2*states.transpose()*states),2.0)).transpose(), 
		     (2*states+nm*c.at(1)+2*(1-scale2*dbar)*states*c.at(1).transpose()*states/pow((1+scale2*states.transpose()*states),2.0)).transpose(), 
		     (2*states+nm*c.at(2)+2*(1-scale2*dbar)*states*c.at(2).transpose()*states/pow((1+scale2*states.transpose()*states),2.0)).transpose();
		     
		return a;
	}
	Eigen::MatrixXd gradient_sigma(Eigen::VectorXd states)
	{
		
				
		Eigen::MatrixXd a = Eigen::MatrixXd::Zero((1+n_of_obstacles)*3,(1+n_of_obstacles)*2);
		
		a.block(0,0,3,2) = sigma0_prime(states);
		a.block(3,0,3,2) = sigma0(obstacles.at(0).states)*si_prime.at(0).transpose();
		a.block(6,0,3,2) = sigma0(obstacles.at(1).states)*si_prime.at(1).transpose();
		a.block(9,0,3,2) = sigma0(obstacles.at(2).states)*si_prime.at(2).transpose();
		a.block(0,2,3,2) = Eigen::MatrixXd::Zero(3,2);
		a.block(3,2,3,2) = si(0)*sigma0_prime(obstacles.at(0).states)-sigma0(obstacles.at(0).states)*si_prime.at(0).transpose();
		a.block(6,2,3,2) = Eigen::MatrixXd::Zero(3,2);
		a.block(9,2,3,2) = Eigen::MatrixXd::Zero(3,2);
		a.block(0,4,3,2) = Eigen::MatrixXd::Zero(3,2);
		a.block(3,4,3,2) = Eigen::MatrixXd::Zero(3,2);
		a.block(6,4,3,2) = si(1)*sigma0_prime(obstacles.at(1).states)-sigma0(obstacles.at(1).states)*si_prime.at(1).transpose();
		a.block(9,4,3,2) = Eigen::MatrixXd::Zero(3,2);
		a.block(0,6,3,2) = Eigen::MatrixXd::Zero(3,2);
		a.block(3,6,3,2) = Eigen::MatrixXd::Zero(3,2);
		a.block(6,6,3,2) = Eigen::MatrixXd::Zero(3,2);
		a.block(9,6,3,2) = si(2)*sigma0_prime(obstacles.at(2).states)-sigma0(obstacles.at(2).states)*si_prime.at(2).transpose();


		return a;
	}
	double pa(Eigen::VectorXd statesVec)
	{
		double sum = 0;
		for(int i = 0; i < n_of_obstacles; i++)
		{
			double dist = (statesVec-obstacles.at(i).states).norm();
			double p = (pow(dist,2.0)-pow(rd,2.0))/(pow(pow(dist,2.0)-pow(ra,2.0),2.0)+repsilon);
			sum += pow((p > 0 ? 0 : p),2.0);
		}
		return sum;
	}
	Eigen::VectorXd gradient_pa(Eigen::VectorXd statesVec)
	{
		Eigen::VectorXd a(8);
		std::vector<Eigen::Vector2d> pa_prime;		
		for(int i = 0; i < n_of_obstacles; i++)
		{
			double dist = (statesVec-obstacles.at(i).states).norm();
			if(dist > rd) pa_prime.push_back(Eigen::Vector2d::Zero());
			else{
				Eigen::Vector2d pa_x_prime = (4*(pow(dist,2.0)-pow(rd,2.0))/pow((pow(pow(dist,2.0)-pow(ra,2.0),2.0)+repsilon),2.0)-8*pow((pow(dist,2.0)-pow(rd,2.0)),2.0)*(pow(dist,2.0)-pow(ra,2.0))/pow((pow(pow(dist,2.0)-pow(ra,2.0),2.0)+repsilon),3.0))*(statesVec-obstacles.at(i).states);
				pa_prime.push_back(pa_x_prime);
			}
		}
		Eigen::Vector2d sum = Eigen::Vector2d::Zero();
		for(int i = 0; i < n_of_obstacles;i++)
		{
			sum += pa_prime.at(i);
		}
		a << sum,-pa_prime.at(0),-pa_prime.at(1),-pa_prime.at(2);
		return a;
	}
	Eigen::MatrixXd G(Eigen::VectorXd statesVec)
	{
		Eigen::MatrixXd g(8,2);
		g << 1, 0,
			 0, 1,
			 0, 0,
			 0, 0,
			 0, 0,
			 0, 0,
			 0, 0,
			 0, 0;
		return g;
	}
	Eigen::VectorXd F(Eigen::VectorXd statesVec)
	{	
		Eigen::Vector2d h1;
		h1 << (-obstacles.at(0).path_r1/obstacles.at(0).path_r2)*(obstacles.at(0).states(1)-obstacles.at(0).path_cy),(obstacles.at(0).path_r2/obstacles.at(0).path_r1)*(obstacles.at(0).states(0)-obstacles.at(0).path_cx);
		Eigen::Vector2d h2;
		h2 << (-obstacles.at(1).path_r1/obstacles.at(1).path_r2)*(obstacles.at(1).states(1)-obstacles.at(1).path_cy),(obstacles.at(1).path_r2/obstacles.at(1).path_r1)*(obstacles.at(1).states(0)-obstacles.at(1).path_cx);
		Eigen::Vector2d h3;
		h3 << (-obstacles.at(2).path_r1/obstacles.at(2).path_r2)*(obstacles.at(2).states(1)-obstacles.at(2).path_cy),(obstacles.at(2).path_r2/obstacles.at(2).path_r1)*(obstacles.at(2).states(0)-obstacles.at(2).path_cx);
		
		Eigen::VectorXd a((1+n_of_obstacles)*2);
		a << 0,
			 0,
			 script_F(0)*h1(0),
			 script_F(0)*h1(1),
			 script_F(1)*h2(0),
			 script_F(1)*h2(1),
			 script_F(2)*h3(0),
			 script_F(2)*h3(1);

		return a;
	}
	Eigen::VectorXd transitionF(Eigen::VectorXd states, std::vector<Obstacle> obs)
	{
		Eigen::VectorXd a((1+n_of_obstacles)*2);
		a << 1,1,0,0,0,0,0,0;
		return a;
	}
	double dotX(Eigen::VectorXd v1,Eigen::VectorXd v2)
	{
		double a = 0;
		

		if(v1.rows() != v2.rows())
		{
			std::cout << "CrossX Failed! Different sizes of vectors! " << v1.rows() << " vs. " << v2.rows() << std::endl;
			return a;
		}
		
		for(int i = 0; i < v1.rows(); i++)
		{
			a += v1(i)*v2(i);
		}
		
		return a;
	}
	Eigen::VectorXd uHatCal(Eigen::VectorXd statesVec)
	{
		Eigen::VectorXd _u = (R.inverse()/(2*usat))*(G(statesVec).transpose()*(gradient_sigma(statesVec).transpose()*WaHat+gradient_pa(statesVec)));
		Eigen::VectorXd u(_u.rows());
		u = -usat*Tanh(_u);
		if(std::abs(u(0))>=usat) u(0) = (usat-0.001)*(sign(u(0))); 
		if(std::abs(u(1))>=usat) u(1) = (usat-0.001)*(sign(u(1))); 
		return u;
	}
    Eigen::VectorXd Ga1(Eigen::VectorXd statesVec)
    {
		Eigen::VectorXd a((1+n_of_obstacles)*3);
		a = usat*gradient_sigma(statesVec)*G(statesVec)*(Tanh((1/ku)*DbarHat(statesVec))-Tanh((R.inverse()/(2*usat))*DbarHat(statesVec)));
		return a;
	}
	Eigen::VectorXd DbarHat(Eigen::VectorXd statesVec)
	{
		Eigen::VectorXd a(2);
		a = G(statesVec).transpose()*(gradient_sigma(statesVec).transpose()*WaHat+gradient_pa(statesVec));
		return a;
	}
	Eigen::VectorXd Tanh(Eigen::VectorXd statesVec)
	{
		Eigen::VectorXd a(statesVec.rows());
		for(int i =0;i<statesVec.rows();i++)
		{
			a(i) = tanhf(statesVec(i));
		}
		
		return a;
	}
    void checksi(Eigen::VectorXd statesVec)
    {
		for(int i = 0; i < n_of_obstacles; i++)
		{
			double dist = (statesVec-obstacles.at(i).states).norm();
			if(dist > rd){
				 si(i) = 0;
				 si_prime.at(i) << 0,0;
			}
			else if(dist <= rd && dist > rbar){
				 si(i) = 0.5*(1+cos(M_PI*(dist-rbar)/(rd-rbar)));
				 Eigen::Vector2d s_p;
				 s_p = -(M_PI/2.0)*sin(M_PI*(dist-rbar)/(rd-rbar))*(1/(dist*(rd-rbar)))*(statesVec-obstacles.at(i).states).transpose();
				 si_prime.at(i) = s_p;
			}
			else if(dist <= rbar){
				 si(i) = 1;
				 si_prime.at(i) << 0,0;
			}
			else std::cout << "An unexpected case just happened!!!" << std::endl;
		}
		return;
	}
	void script_FCal(Eigen::VectorXd statesVec)
	{
		for(int i = 0; i < n_of_obstacles; i++)
		{
			double dist = (statesVec-obstacles.at(i).states).norm();
			if(dist > rd) script_F(i) = 0;
			else if(dist <= rd && dist > rbar) script_F(i) = 0.5*(1+cos(M_PI*(dist-rbar)/(rd-rbar)));
			else script_F(i) = 1;	
		}
		return;
	}
	Eigen::VectorXd Qx(Eigen::VectorXd statesVec)
	{
		Eigen::VectorXd weight(statesVec.rows());
		weight << weightQx.at(0), weightQx.at(1);
		
		Eigen::MatrixXd weightMat;
		weightMat = weight.asDiagonal();
		
		return statesVec.transpose()*weightMat*statesVec;
	}
	Eigen::VectorXd QzSum()
	{

		Eigen::VectorXd sum(1);
		sum << 0;
		
		Eigen::VectorXd weight(obstacles.at(0).states.rows());
		weight << weightQzSum.at(0), weightQzSum.at(1);	
		
		Eigen::MatrixXd weightMat;
		weightMat = weight.asDiagonal();
		
		for(int i = 0; i < n_of_obstacles; i++)
		{
			Eigen::VectorXd q(1);
			q(0) = obstacles.at(i).states.transpose()*weightMat*obstacles.at(i).states;
			sum(0) += si(i)*q(0);
		}

		return sum;
	}	
	
	Eigen::VectorXd Phi(Eigen::VectorXd u)
	{
		Eigen::VectorXd p(1);

		for(int i = 0; i < 2; i++)
		{
			p(0) += 2*usat*Rbar(i)*u(i)*atanhf(u(i)/usat)+pow(usat,2.0)*Rbar(i)*log(1-(pow(u(i),2.0)/pow(usat,2.0)));
		}
		return p;
	}
	Eigen::VectorXd P(Eigen::VectorXd statesVec)
	{
		double sum = 0;
		for(int i = 0; i < n_of_obstacles; i++)
		{
			double dist = (pow((statesVec-obstacles.at(i).states).norm(),2.0)-pow(rd,2.0))/(pow(pow((statesVec-obstacles.at(i).states).norm(),2.0)-pow(ra,2.0),2.0));
			sum += pow((dist < 0 ? dist : 0),2.0);
		}
		Eigen::VectorXd s(1);
		s(0) = sum;
		return s;
	}
	double sign(double a)
	{
		return a >= 0 ? 1 : -1;
	}
    
}; // end Agent_controller

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Agent_controller");
    
    
    Agent_controller obj;
    
    ros::spin();
    return 0;
}

