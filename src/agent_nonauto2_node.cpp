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
		
		if(!use_true_obstacles) mocapPose = nho.subscribe(bebopName+"/desPose",1,&Obstacle_subscriber::mocapCB,this);
		else mocapPose = nho.subscribe(bebopName+"/mocapPose",1,&Obstacle_subscriber::mocapCB,this);
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
    ros::Subscriber mocapSub_bbp,syncSub;
    ros::Time time_now,time_last,time_start;
    
    
    // Topic prefix
    std::string bebopName;//namespace for the bebop
    
    
    // Parameters
    Eigen::VectorXd si;
    double si2;
    Eigen::VectorXd si2_prime;
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
    double alpha;
    double eta1;
    Eigen::VectorXd thetaHat;
    Eigen::VectorXd thetaHatDot;
    Eigen::MatrixXd Gammap;
    double kp;
    Eigen::VectorXd Yi;
    std::vector<Eigen::VectorXd> Yi_stack;
    Eigen::VectorXd Yp_last;
    Eigen::MatrixXd V;
    double pa_last;
    std::vector<Eigen::VectorXd> delta_pa_stack;
	std::vector<Eigen::VectorXd> c;
	std::vector<double> cKappa;
    int n_of_obstacles;
	std::vector<double> kv;//lin command gain
	std::vector<double> kw;//ang command gain
	std::vector<double> weightQx;
	std::vector<double> weightQzSum;
	std::vector<double> init_desPose; //initial desired pose
	std::vector<Eigen::VectorXd> si_prime;
	std::vector<std::string> obstacleNames;
	bool sync,start_exp, dataSaved;
    
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
	int Np;
	
    // Save "time" << "agentStates" << "obstacleStates" << "WcHat" << "WaHat" << "uHat" << "vHat" << "thetaHat" << "cost" << "Yp*thetaHat" << "delta"
	std::vector<double> timeData,vHatData,costData,WcHatSigmaData;
	std::vector<Eigen::Vector2d> agentStatesData,obstacle1StatesData,uHatData,obstacle2StatesData,obstacle3StatesData;
	std::vector<Eigen::VectorXd> WcHatData,WaHatData,thetaHatData, YpHatData, deltaData;
    
public:
    Agent_controller()
    {
        // Initialize parameters
        ros::NodeHandle nhp("~");
		nhp.param<std::vector<double>>("kv",kv,{0.3,0.05,0.06});
		nhp.param<std::vector<double>>("kw",kw,{0.2});
		nhp.param<std::vector<double>>("weightQx",weightQx,{5.0,1.0});
		nhp.param<std::vector<double>>("weightQzSum",weightQzSum,{2.0,2.0});
		nhp.param<std::vector<double>>("kw",kw,{0.2});
		nhp.param<std::vector<double>>("init_desPose",init_desPose,{-4.0,1.5,1.0});
		nhp.param<std::string>("bebopName",bebopName,"bebop4");
		nhp.param<std::vector<std::string>>("obstacleNames",obstacleNames,{"bebop0","bebop2","bebop3"});
		nhp.param<int>("n_of_obstacles",n_of_obstacles,3);
		nhp.param<double>("ra",ra,0.1);
		nhp.param<double>("rbar",rbar,0.3);
		nhp.param<double>("rd",rd,0.5);
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
		//repsilon = 0.10;
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
		alpha = 0.25;
		eta1 = 0.01;
		c.push_back(c_scale * Eigen::Vector2d(0.0,-1.0));
		c.push_back(c_scale * Eigen::Vector2d(0.866,0.5));
		c.push_back(c_scale * Eigen::Vector2d(-0.866,0.5));
		
		cKappa.push_back(c_scale * 0.25);
		cKappa.push_back(c_scale * 0.05);
		
		Rbar = Eigen::VectorXd(2);
		Rbar << 10.0,10.0;
		R = Rbar.asDiagonal();
	
		W_limit = 30;
		

		WaHat = Eigen::VectorXd(5);
		WaHat << 1,1,1,1,1;
		WaHat *= 1;
		
		WaHatDot = Eigen::VectorXd(5);
		WaHatDot << 0,0,0,0,0;
		
		
		dis = std::uniform_real_distribution<>(0.0,1.0);
		
		WcHat = Eigen::VectorXd(5);
		//WcHat << dis(gen),dis(gen),dis(gen),dis(gen),dis(gen);
		//WcHat << 1,1,1,1,1;
		WcHat << 0.25629, 0.635961, 0.97647, 0.0332995, 0.733583;
		WcHat *= 4;
		
		WcHatDot = Eigen::VectorXd(5);
		WcHatDot << 0,0,0,0,0;
		
		si = Eigen::VectorXd(3);
		si(0) = 0.0;
		si(1) = 0.0;
		si(2) = 0.0;
		si2 = 1.0;
		
		si2_prime = Eigen::VectorXd(2);
		si2_prime << 0.0,0.0;
		
		script_F = Eigen::VectorXd(3);
		script_F(0) = 0.0;
		script_F(1) = 0.0;
		script_F(2) = 0.0;
		
		Eigen::Vector2d s_prime;
		s_prime << 0.0,0.0;
		si_prime.push_back(s_prime);
		si_prime.push_back(s_prime);
		si_prime.push_back(s_prime);
		
		GammaElements = Eigen::VectorXd(5);
		GammaElements << 1,1,1,300,300;
		Gamma = GammaElements.asDiagonal();
		Gamma *= 1;
		
		GammaaElements = Eigen::VectorXd(5);
		GammaaElements << 1,1,1,1,1;
		Gammaa = GammaElements.asDiagonal();
		
		GammaDot.setZero(5,5);
		
		cost = 0;
		r = Eigen::VectorXd(1);
		r << 0;
		Np = 25;
		
	    thetaHat = Eigen::VectorXd(10);
	    thetaHat << 1,1,1,1,1,1,1,1,1,1;
		thetaHatDot = Eigen::VectorXd::Zero(10);
		Eigen::VectorXd GammapElements(10);
		GammapElements << 1,1,1,1,1,1,1,1,1,1;
		
		Gammap = GammapElements.asDiagonal();
		kp = 10.0;
		Yi = Eigen::VectorXd::Zero(10);
		Yp_last = Eigen::VectorXd::Zero(10);
		pa_last = 0;
			
		V = Eigen::MatrixXd(3,10);
		V << 1,1,1,1,1,1,1,1,1,1,
			 1,1,1,1,1,1,1,1,1,1,
			 1,1,1,1,1,1,1,1,1,1;

		dis = std::uniform_real_distribution<>(-5.0,5.0);
		
		for(int i = 0; i < V.rows(); i++)
		{
			for(int j = 0; j< V.cols(); j++)
			{
				V(i,j) = dis(gen);
			}
		}
	
		/** End TEMP **/
		
		
		std::cout << "Controlling: " << bebopName << std::endl;
		
		
		// Initialize obstacles	
		for(int i = 0; i < n_of_obstacles; i++)
		{
			Obstacle obstacle = Obstacle(ra,rbar,rd,0,0,0,0,0,0);
			obstacles.push_back(obstacle);
		}

		obj = new Obstacle_subscriber(obstacleNames.at(0),&obstacles.at(0).states,false, offset_x, offset_y);
		obstacle_sub.push_back(obj);

		obj = new Obstacle_subscriber(obstacleNames.at(1),&obstacles.at(1).states,false, offset_x, offset_y);
		obstacle_sub.push_back(obj);
		
		obj = new Obstacle_subscriber(obstacleNames.at(2),&obstacles.at(2).states,false, offset_x, offset_y);
		obstacle_sub.push_back(obj);
		
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
		
		std::cout << "Initial position: " << std::endl;
		std::cout << desPoseLin.getX() << " " << desPoseLin.getY() << " " << desPoseLin.getZ() << std::endl;
		std::cout << "WeightQx: " << weightQx.at(0) << " " << weightQx.at(1)  << std::endl; 
		std::cout << "WeightQzSum: " << weightQzSum.at(0) << " " << weightQzSum.at(1) << std::endl; 
		std::cout << "r_epsilon: " << repsilon << std::endl; 
		std::cout << "usat: " << usat << std::endl; 
		
            
        // Subscribers
        
        //mocapSub_bbp = nh.subscribe(bebopName+"/predictorPose",1,&Agent_controller::mocapCB,this); //make sure to publish to world frame
        mocapSub_bbp = nh.subscribe(bebopName+"/mocapPose",1,&Agent_controller::mocapCB,this); //make sure to publish to body frame
        syncSub = nh.subscribe("/sync",1,&Agent_controller::syncCB,this);
        
        // Publishers
        desTwistPub = nh.advertise<geometry_msgs::Twist>(bebopName+"/desTwist",1);
        
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
			updateVel(tf::Vector3(0,0,0),tf::Quaternion(0,0,0,1));
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
			
				std::ofstream writeFile("/home/ncr/experiment_data/adp_staf_pp_dynamic/saved_data/nonauto_cl/data.txt");
				if (writeFile.is_open())
				{
					//std::vector<double> timeData,vHatData,costData,YpHatData, deltaData;
					//std::vector<Eigen::Vector2d> agentStatesData,obstacleStatesData,uHatData;
					//std::vector<Eigen::VectorXd> WcHatData,WaHatData,thetaHatData
					writeFile << "time," << "agentStatesX," << "agentStatesY," << "obstacle1StatesX," << "obstacle1StatesY," << "obstacle2StatesX," << "obstacle2StatesY,"
							  << "obstacle3StatesX," << "obstacle3StatesY," << "uHatX," << "uHatY," 
							  << "WcHat1," << "WcHat2," << "WcHat3," << "WcHat4," << "WcHat5,"
							  << "WaHat1," << "WaHat2," << "WaHat3," << "WaHat4," << "WaHat5,"
							  << "thetaHat1," << "thetaHat2," << "thetaHat3," << "thetaHat4," << "thetaHat5," << "thetaHat6," << "thetaHat7," << "thetaHat8," << "thetaHat9," << "thetaHat10," 
							  << "vHat," << "WcHatSigma," << "cost," << "YpthetaHat," << "delta" << "\n";
					for (int i = 0; i < timeData.size(); i++)
					{
						writeFile << timeData.at(i) << "," << agentStatesData.at(i)(0) << "," << agentStatesData.at(i)(1) << "," 
								  << obstacle1StatesData.at(i)(0) << "," << obstacle1StatesData.at(i)(1) << "," 
								  << obstacle2StatesData.at(i)(0) << "," << obstacle2StatesData.at(i)(1) << ","
								  << obstacle3StatesData.at(i)(0) << "," << obstacle3StatesData.at(i)(1) << ","
								  << uHatData.at(i)(0) << "," << uHatData.at(i)(1) << ","
								  << WcHatData.at(i)(0) << "," << WcHatData.at(i)(1) << "," << WcHatData.at(i)(2) << "," << WcHatData.at(i)(3) << "," << WcHatData.at(i)(4) << "," 
								  << WaHatData.at(i)(0) << "," << WaHatData.at(i)(1) << "," << WaHatData.at(i)(2) << "," << WaHatData.at(i)(3) << "," << WaHatData.at(i)(4) << "," 
								  << thetaHatData.at(i)(0) << "," << thetaHatData.at(i)(1) << "," << thetaHatData.at(i)(2) << "," << thetaHatData.at(i)(3) << "," << thetaHatData.at(i)(4) << "," 
								  << thetaHatData.at(i)(5) << "," << thetaHatData.at(i)(6) << "," << thetaHatData.at(i)(7) << "," << thetaHatData.at(i)(8) << "," << thetaHatData.at(i)(9) << "," 
								  << vHatData.at(i) << "," << WcHatSigmaData.at(i) << "," << costData.at(i) << "," << YpHatData.at(i) << "," << deltaData.at(i) << "\n";
					}
				writeFile.close();
				std::cout << "Data recorded" << std::endl;
				dataSaved = true;
				return;
				}
			}
		
		// Check si
		checksi(states);
		checksi2(states);
		script_FCal(states);
		
		
		// Update NN weights
		WaHat += WaHatDot*dt;
		WcHat += WcHatDot*dt;
		Gamma += GammaDot*dt;
		cost += r(0)*dt;
		thetaHat += thetaHatDot*dt;
		
		
		Yi = Yp_last*dt;
		Yi_stack.push_back(Yi);
		

		Eigen::VectorXd delta_pa(1);
		delta_pa << pa(states)-pa_last;
		pa_last = pa(states);
		delta_pa_stack.push_back(delta_pa);

		
		Eigen::VectorXd Yp(10);
		Yp = YpCal(states);
		Yp_last = Yp;

		Eigen::VectorXd sum_th(10);
		sum_th = Eigen::VectorXd::Zero(10);
		
		for(int i = 0; i < Yi_stack.size() ; i++)
		{			
			sum_th += Yi_stack.at(i)*(delta_pa_stack.at(i)-Yi_stack.at(i).transpose()*thetaHat);
		}
		
		if(si(0) != 0 ||si(1) != 0 ||si(2) != 0 ) thetaHatDot = Gammap*kp*sum_th;
		else thetaHatDot = Eigen::VectorXd::Zero(10);

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

		// Calculate kappa
		
		double kappa = alpha/(eta1*(time_now-time_start).toSec()+1);
		double kappaDot = -alpha*eta1/pow((eta1*(time_now-time_start).toSec()+1),2.0);
		
		double nm2 = (1-(time_now-time_start).toSec())/(1+(time_now-time_start).toSec());
		std::uniform_real_distribution<> dis2(-nm2*0.05,nm2*0.05);		
		
		//double kappak = alpha/(eta1*((time_now-time_start).toSec()+dis2(gen))+1);
		//double kappakDot = -alpha*eta1/pow((eta1*((time_now-time_start).toSec()+dis2(gen))+1),2.0);
		
		double kappak = kappa;
		double kappakDot = kappaDot;
		
		//double kappa = alpha*exp(-(time_now-time_start).toSec()*eta1);
		//double kappaDot = -alpha*eta1*exp(-(time_now-time_start).toSec()*eta1);

		// Calculate controller uHat
		Eigen::Vector2d uHat = uHatCal(states,kappa);
		
		// Calculate controller uHatk
		Eigen::Vector2d uHatk = uHatCal(statesk,kappa);
		
		// Calculate auxiliary signals

		Eigen::VectorXd wk(5);
		wk = gradient_sigmaKappa(statesk, kappak)*(F(statesk, kappakDot)+G(statesk)*uHatCal(statesk,kappak));

		Eigen::VectorXd wpk(1);
		wpk << wpCal(statesk,kappak);

		Eigen::VectorXd rk(1);
		rk = Qx(statesk) + QzSum() + Phi(uHatk) + P(statesk);

		Eigen::VectorXd deltak = WcHat.transpose()*wk + wpk + rk;

		Eigen::VectorXd w(5);
		w = gradient_sigmaKappa(states, kappa)*(F(states, kappaDot)+G(states)*uHatCal(states,kappa));
		
		Eigen::VectorXd wp(1);
		wp << wpCal(states,kappa);

		r = Qx(states) + QzSum() + Phi(uHat) + P(states);
		
		
		Eigen::VectorXd delta = WcHat.transpose()*w + wp + r;

		double rho = 1+gamma1*dotX(w,w);
		double rhok = 1+gamma1*dotX(wk,wk);

		// Update law for WcHat
		Eigen::VectorXd sum(5);
		sum << 0,0,0,0,0;
		for(int i =0;i < N;i++)
		{
			sum += (1/rhok)*wk*deltak;
			
		}		
		WcHatDot = -Gamma*(((kc1*w)/rho)*delta+(kc2/N)*sum);

		// Update law for WaHat


		sum << 0,0,0,0,0;
		for(int i =0;i < N;i++)
		{
			sum += Ga1(states, kappa)*(1/rhok)*wk.transpose()*WcHat;
			
		}				
		WaHatDot = -Gammaa*(ka1*(WaHat-WcHat)+ka2*WaHat+(kc1/rho)*Ga1(states, kappa)*wk.transpose()*WcHat+(kc2/N)*sum);


		// Update law for Gamma
		Eigen::MatrixXd sumMat;
		sumMat.setZero(5,5);
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
		
		time_last = time_now;
		mLin_last = mLin;
		lastDesPoseAng = desPoseAng;
		lastPose = tfPose;
		
		double vHat = (WcHat.transpose()*sigmaKappa(states, kappa))(0)+pa(states);
		double WcHatSigma = (WcHat.transpose()*sigmaKappa(states, kappa))(0);
		
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
					  << std::setw(30) << std::left << "Gradient of Pa:"
					  << std::endl;
			for(int i = 0; i < si.rows(); i++){
				std::cout << std::setw(30) << std::left << si(i)
						  << std::setw(30) << std::left << gradient_pa(states)(i)
						  << std::endl;
			}				  
			std::cout << std::endl
					  << std::setw(30) << std::left << "Cost:"
					  << std::setw(30) << std::left << "Pa:"
					  << std::setw(30) << std::left << "delta:"
					  << std::setw(30) << std::left << "vHat"
					  << std::setw(30) << std::left << "P"
					  << std::endl;
			for(int i = 0; i < r.rows(); i++){
				std::cout << std::setw(30) << std::left << cost
						  << std::setw(30) << std::left << pa(states)
						  << std::setw(30) << std::left << delta(0)
						  << std::setw(30) << std::left << vHat
						  << std::setw(30) << std::left << P(states)(0)
						  << std::endl;
			}				  
			std::cout << std::endl						
					  << std::setw(30) << std::left << "thetaHat:"
					  << std::setw(30) << std::left << "thetaHatDot:"
					  << std::endl;
			for(int i = 0; i < thetaHat.rows(); i++){
				std::cout << std::setw(30) << std::left << thetaHat(i)
						  << std::setw(30) << std::left << thetaHatDot(i)
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
			//std::vector<Eigen::VectorXd> WcHatData,WaHatData,thetaHatData
			
			timeData.push_back((time_now-time_start).toSec());
			vHatData.push_back(vHat);
			WcHatSigmaData.push_back(WcHatSigma);
			costData.push_back(cost);
			deltaData.push_back(delta);
			YpHatData.push_back(wp);
			agentStatesData.push_back(states);
			obstacle1StatesData.push_back(obstacles.at(0).states);
			obstacle2StatesData.push_back(obstacles.at(1).states);
			obstacle3StatesData.push_back(obstacles.at(2).states);
			uHatData.push_back(uHat);
			WcHatData.push_back(WcHat);
			WaHatData.push_back(WaHat);
			thetaHatData.push_back(thetaHat);
		}
	}
		
	}
	double wpCal(Eigen::VectorXd statesVec, double kappa)
	{
		/*
		Eigen::Vector2d h1;
		h1 << (-obstacles.at(0).path_r1/obstacles.at(0).path_r2)*(obstacles.at(0).states(1)-obstacles.at(0).path_cy),(obstacles.at(0).path_r2/obstacles.at(0).path_r1)*(obstacles.at(0).states(0)-obstacles.at(0).path_cx);
		Eigen::Vector2d h2;
		h2 << (-obstacles.at(1).path_r1/obstacles.at(1).path_r2)*(obstacles.at(1).states(1)-obstacles.at(1).path_cy),(obstacles.at(1).path_r2/obstacles.at(1).path_r1)*(obstacles.at(1).states(0)-obstacles.at(1).path_cx);
		Eigen::Vector2d h3;
		h3 << (-obstacles.at(2).path_r1/obstacles.at(2).path_r2)*(obstacles.at(2).states(1)-obstacles.at(2).path_cy),(obstacles.at(2).path_r2/obstacles.at(2).path_r1)*(obstacles.at(2).states(0)-obstacles.at(2).path_cx);
	
	    std::vector<Eigen::Vector2d> h;
	    h.push_back(h1);
	    h.push_back(h2);
	    h.push_back(h3);*/
		Eigen::VectorXd pa_prime(1);		

		pa_prime = YpCal(statesVec).transpose()*thetaHat;
	
		return pa_prime(0);
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
	Eigen::VectorXd sigmaKappa(Eigen::VectorXd states, double kappa)
	{
		Eigen::VectorXd a(5);

		a << sigma0(states),
			 si2*sigmaKappa0(kappa);
		return a;
	}
	Eigen::VectorXd sigmaKappa0(double kappa)
	{
		Eigen::VectorXd a(2);
		double nm = (kappa*kappa+dbar)/(1+scale2*kappa*kappa);
		a << kappa*(kappa+nm*cKappa.at(0)), kappa*(kappa+nm*cKappa.at(1));
		return a;
	}
	Eigen::MatrixXd sigmaKappa0_prime(double kappa)
	{
		double nm = (kappa*kappa+dbar)/(1+scale2*kappa*kappa);
		Eigen::MatrixXd a(2,1);
		a << 2*kappa+nm*cKappa.at(0)+2*(1-scale2*dbar)*kappa*cKappa.at(0)*kappa/pow((1+scale2*kappa*kappa),2.0), 
		     2*kappa+nm*cKappa.at(1)+2*(1-scale2*dbar)*kappa*cKappa.at(1)*kappa/pow((1+scale2*kappa*kappa),2.0);
		     
		return a;
	}
	Eigen::MatrixXd gradient_sigmaKappa(Eigen::VectorXd states, double kappa)
	{
		Eigen::MatrixXd a = Eigen::MatrixXd::Zero(5,3);
		a.block(0,0,3,2) = sigma0_prime(states);
		a.block(0,2,3,1) = Eigen::MatrixXd::Zero(3,1);
		a.block(3,0,2,2) = sigmaKappa0(kappa)*si2_prime.transpose();
		a.block(3,2,2,1) = si2*sigmaKappa0_prime(kappa);

		return a;
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
		Eigen::VectorXd a(3);
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
		a << sum,0.0;
		return a;
	}
	Eigen::MatrixXd G(Eigen::VectorXd statesVec)
	{
		/*
		Eigen::MatrixXd g(8,2);
		g << 1, 0,
			 0, 1,
			 0, 0,
			 0, 0,
			 0, 0,
			 0, 0,
			 0, 0,
			 0, 0;*/
			 
		Eigen::MatrixXd g(3,2);
		g << 1, 0,
			 0, 1,
			 0, 0;		 
			 
		return g;
	}
	Eigen::VectorXd F(Eigen::VectorXd statesVec, double kappaDot)
	{	
		Eigen::Vector2d h1;
		h1 << (-obstacles.at(0).path_r1/obstacles.at(0).path_r2)*(obstacles.at(0).states(1)-obstacles.at(0).path_cy),(obstacles.at(0).path_r2/obstacles.at(0).path_r1)*(obstacles.at(0).states(0)-obstacles.at(0).path_cx);
		Eigen::Vector2d h2;
		h2 << (-obstacles.at(1).path_r1/obstacles.at(1).path_r2)*(obstacles.at(1).states(1)-obstacles.at(1).path_cy),(obstacles.at(1).path_r2/obstacles.at(1).path_r1)*(obstacles.at(1).states(0)-obstacles.at(1).path_cx);
		Eigen::Vector2d h3;
		h3 << (-obstacles.at(2).path_r1/obstacles.at(2).path_r2)*(obstacles.at(2).states(1)-obstacles.at(2).path_cy),(obstacles.at(2).path_r2/obstacles.at(2).path_r1)*(obstacles.at(2).states(0)-obstacles.at(2).path_cx);
		
		Eigen::VectorXd a(3);
		/*a << 0,
			 0,
			 script_F(0)*h1(0),
			 script_F(0)*h1(1),
			 script_F(1)*h2(0),
			 script_F(1)*h2(1),
			 script_F(2)*h3(0),
			 script_F(2)*h3(1);*/
		a << 0,0,kappaDot;

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
	Eigen::VectorXd uHatCal(Eigen::VectorXd statesVec, double kappa)
	{
		Eigen::VectorXd _u = (R.inverse()/(2*usat))*(G(statesVec).transpose()*(gradient_sigmaKappa(statesVec, kappa).transpose()*WaHat+gradient_pa(statesVec)));
		Eigen::VectorXd u(_u.rows());
		u = -usat*Tanh(_u);
		if(std::abs(u(0))>=usat) u(0) = (usat-0.001)*(sign(u(0))); 
		if(std::abs(u(1))>=usat) u(1) = (usat-0.001)*(sign(u(1))); 
		return u;
	}
    Eigen::VectorXd Ga1(Eigen::VectorXd statesVec, double kappa)
    {
		Eigen::VectorXd a(5);
		a = usat*gradient_sigmaKappa(statesVec, kappa)*G(statesVec)*(Tanh((1/ku)*DbarHat(statesVec, kappa))-Tanh((R.inverse()/(2*usat))*DbarHat(statesVec, kappa)));
		return a;
	}
	Eigen::VectorXd DbarHat(Eigen::VectorXd statesVec, double kappa)
	{
		Eigen::VectorXd a(2);
		a = G(statesVec).transpose()*(gradient_sigmaKappa(statesVec, kappa).transpose()*WaHat+gradient_pa(statesVec));
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
    void checksi2(Eigen::VectorXd statesVec)
    {
		/*
		double dist = (statesVec).norm();
		if(dist > rd){
			 si2 = 1.0;
			 si2_prime << 0.0,0.0;
		}
		else if(dist <= rd && dist > ra){
			 si2 = 0.5*(1+cos(M_PI*(dist-rd)/(rd-ra)));
			 Eigen::Vector2d s_p;
			 s_p = -(M_PI/2.0)*sin(M_PI*(dist-rd)/(rd-ra))*(1/(dist*(rd-ra)))*(statesVec).transpose();
			 si2_prime = s_p;
		}
		else if(dist <= ra){
			 si2 = 0.0;
			 si2_prime << 0.0,0.0;
		}
		else std::cout << "An unexpected case just happened!!!" << std::endl;*/
		
		si2 = ((statesVec.transpose()*statesVec)/((statesVec.transpose()*statesVec)+1))(0);
		si2_prime = 2*(statesVec.transpose())/pow((statesVec.transpose()*statesVec)+1,2.0);
		
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
	Eigen::VectorXd YpCal(Eigen::VectorXd statesVec)
	{
		Eigen::VectorXd a(10);

		//a << activation_pa(pa(statesVec));
		a << activation_gpa(gradient_pa(statesVec));

		return a;
	}
	Eigen::VectorXd activation_pa(double pa)
	{
		Eigen::VectorXd a;
		return a;
	}
	Eigen::VectorXd activation_gpa(Eigen::VectorXd gpa)
	{
		Eigen::VectorXd a;
		
		a = Tanh(V.transpose()*gpa);
		
		return a;
	}
	Eigen::VectorXd elementWiseDot(Eigen::VectorXd a, Eigen::VectorXd b)
	{
		Eigen::VectorXd result = Eigen::VectorXd(a.rows());
		for(int i = 0 ; i < a.rows() ; i++)
		{
			result(i) = a(i)*b(i);
		}
		return result;
	}
    
}; // end Agent_controller

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Agent_controller");
    
    
    Agent_controller obj;
    
    ros::spin();
    return 0;
}

