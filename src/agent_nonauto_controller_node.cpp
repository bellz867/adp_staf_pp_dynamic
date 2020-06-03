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
    ros::Subscriber mocapSub_bbp, obstacleSub0, obstacleSub1,  obstacleSub2;
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
    Eigen::VectorXd r;
    double alpha;
    double eta1;
	std::vector<Eigen::VectorXd> c;
	std::vector<double> cKappa;
    int n_of_obstacles;
	std::vector<double> kv;//lin command gain
	std::vector<double> kw;//ang command gain
	std::vector<double> init_desPose; //initial desired pose
	std::vector<Eigen::VectorXd> si_prime;
	std::vector<std::string> obstacleNames;
    
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


public:
    Agent_controller()
    {
        // Initialize parameters
        ros::NodeHandle nhp("~");
		nhp.param<std::vector<double>>("kv",kv,{0.2,0.1,0.7});
		nhp.param<std::vector<double>>("kw",kw,{0.2});
		nhp.param<std::vector<double>>("init_desPose",init_desPose,{1.0,1.0,1.0});
		nhp.param<std::string>("bebopName",bebopName,"bebop4");
		nhp.param<std::vector<std::string>>("obstacleNames",obstacleNames,{"bebop0","bebop2","bebop3"});
		nhp.param<int>("n_of_obstacles",n_of_obstacles,3);
		nhp.param<double>("ra",ra,0.3);
		nhp.param<double>("rbar",rbar,0.45);
		nhp.param<double>("rd",rd,1);
		time_now = ros::Time::now();
		time_last = ros::Time::now();
		time_start = time_now;
		Obstacle_subscriber * obj;
		
		std::cout << kv.at(0) << " " << kv.at(1) << " " << kv.at(2) << std::endl;
		std::cout << ra << " " << rbar << " " << rd << std::endl;
		
		/** TEMP - include in launch file?? **/
		
		offset_x = -2;
		offset_y = 0;
		
		N = 1.0;
		repsilon = 0.1;
		kc1 = 0.001;
		kc2 = 1;
		gamma1 = 1;
		beta = 0.001;
		ka1 = 0.75;
		ka2 = 0.75;
		ku = 1;
		c_scale = 1;
		scale2 = 1;
		dbar = 0;
		alpha = 0.25;
		eta1 = 0.1;
		c.push_back(c_scale * Eigen::Vector2d(0.0,1.0));
		c.push_back(c_scale * Eigen::Vector2d(-0.866,-0.5));
		c.push_back(c_scale * Eigen::Vector2d(0.866,-0.5));
		
		cKappa.push_back(c_scale * 1.0);
		cKappa.push_back(c_scale * 0.0);
		
		Rbar = Eigen::VectorXd(2);
		Rbar << 10.0,10.0;
		R = Rbar.asDiagonal();
		
		usat = 0.5;
		W_limit = 30;
		
		WaHat = Eigen::VectorXd(5);
		WaHat << 1,1,1,1,1;
		WaHat *= 0.4;
		
		WaHatDot = Eigen::VectorXd(5);
		WaHatDot << 0,0,0,0,0;
		
		WcHat = Eigen::VectorXd(5);
		WcHat << 1,1,1,1,1;
		WcHat *= 0.4;
		
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
		GammaElements << 1,1,1,1,1;
		Gamma = GammaElements.asDiagonal();
		Gamma *= 1;
		
		GammaaElements = Eigen::VectorXd(5);
		GammaaElements << 1,1,1,1,1;
		Gammaa = GammaElements.asDiagonal();
		
		GammaDot.setZero(5,5);
		
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
		obstacles.at(0).path_w = 1/20;
		obstacles.at(0).path_phi = 0;
		obstacles.at(1).path_cx = 0.5;
		obstacles.at(1).path_cx = 0;
		obstacles.at(1).path_r1 = 1.0;
		obstacles.at(1).path_r2 = 0.5;
		obstacles.at(1).path_w = -1/20;
		obstacles.at(1).path_phi = 3.14;
		obstacles.at(2).path_cx = -1.5;
		obstacles.at(2).path_cx = 0;
		obstacles.at(2).path_r1 = 1.0;
		obstacles.at(2).path_r2 = 0.5;
		obstacles.at(2).path_w = 1/20;
		obstacles.at(2).path_phi = 3.14;
				
	
		std::cout << "Avoiding: " << n_of_obstacles << " obstacles." << std::endl;
		
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
        
        mocapSub_bbp = nh.subscribe(bebopName+"/predictorPose",1,&Agent_controller::mocapCB,this);
        //mocapSub_bbp = nh.subscribe(bebopName+"/mocapPose",1,&Agent_controller::mocapCB,this);
        
        // Publishers
        desTwistPub = nh.advertise<geometry_msgs::Twist>(bebopName+"/desTwist",1);
        
        std::cout << "End of initialization." << std::endl;
        
    }
    void mocapCB(const geometry_msgs::PoseStampedConstPtr& pose)
    {		
		time_now = ros::Time::now();
		double dt = (time_now-time_last).toSec();
		
		Eigen::Vector2d states;
		states << pose->pose.position.x, pose->pose.position.y;
		
		// Transform world frame;
		states(0) += offset_x;
		states(1) += offset_y;
		
		// Check si
		checksi(states);
		checksi2(states);
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
		std::uniform_real_distribution<> dis(-nm*0.25,nm*0.25);
		
		Eigen::Vector2d k;
		k << dis(gen), dis(gen);
		Eigen::Vector2d statesk = states+k;
		
		// Calculate kappa
		
		double kappa = alpha/(eta1*(time_now-time_start).toSec()+1);
		double kappaDot = -alpha*eta1/pow((eta1*(time_now-time_start).toSec()+1),2.0);

		// Calculate controller uHat
		Eigen::Vector2d uHat = uHatCal(states,kappa);
		
		// Calculate controller uHatk
		Eigen::Vector2d uHatk = uHatCal(statesk,kappa);
		
		// Calculate auxiliary signals
		

		Eigen::VectorXd wk(5);
		wk = gradient_sigmaKappa(statesk, kappa)*(F(statesk, kappaDot)+G(statesk)*uHatCal(statesk,kappa));

		Eigen::VectorXd wpk(1);
		wpk << wpCal(statesk,kappa);

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
		
		updateVel(mCmd,vAng); // Pulish control commands in world frame
		
		//updateVel(vLin,vAng); // Publish control commands in body frame
		
		time_last = time_now;
		mLin_last = mLin;
		lastDesPoseAng = desPoseAng;
		lastPose = tfPose;
		
		/** Output Text Block **/
		if(1){
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
					  << std::setw(30) << std::left << "wp:"
					  << std::endl;
			for(int i = 0; i < r.rows(); i++){
				std::cout << std::setw(30) << std::left << cost
						  << std::setw(30) << std::left << wp
						  << std::endl;
			}				  
			std::cout << std::endl;						
			  
				  
		}
		/** End Ouput Text Block **/
		
		
	}
	double wpCal(Eigen::VectorXd statesVec, double kappa)
	{
		
		Eigen::Vector2d h1;
		h1 << (-obstacles.at(0).path_r1/obstacles.at(0).path_r2)*(obstacles.at(0).states(1)-obstacles.at(0).path_cy),(obstacles.at(0).path_r2/obstacles.at(0).path_r1)*(obstacles.at(0).states(0)-obstacles.at(0).path_cx);
		Eigen::Vector2d h2;
		h2 << (-obstacles.at(1).path_r1/obstacles.at(1).path_r2)*(obstacles.at(1).states(1)-obstacles.at(1).path_cy),(obstacles.at(1).path_r2/obstacles.at(1).path_r1)*(obstacles.at(1).states(0)-obstacles.at(1).path_cx);
		Eigen::Vector2d h3;
		h3 << (-obstacles.at(2).path_r1/obstacles.at(2).path_r2)*(obstacles.at(2).states(1)-obstacles.at(2).path_cy),(obstacles.at(2).path_r2/obstacles.at(2).path_r1)*(obstacles.at(2).states(0)-obstacles.at(2).path_cx);
	
	    std::vector<Eigen::Vector2d> h;
	    h.push_back(h1);
	    h.push_back(h2);
	    h.push_back(h3);
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
		double sum = 0.0;
		for(int i = 0; i < n_of_obstacles;i++)
		{
			sum += pa_prime.at(i).transpose()*(Eigen::Vector2d::Zero()+uHatCal(statesVec,kappa)-script_F(i)*h.at(i));
		}
				
		return sum;
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
		double dist = (statesVec).norm();
		if(dist > rd){
			 si2 = 0;
			 si2_prime << 0.0,0.0;
		}
		else if(dist <= rd && dist > rbar){
			 si2 = 0.5*(1+cos(M_PI*(dist-rbar)/(rd-rbar)));
			 Eigen::Vector2d s_p;
			 s_p = -(M_PI/2.0)*sin(M_PI*(dist-rbar)/(rd-rbar))*(1/(dist*(rd-rbar)))*(statesVec).transpose();
			 si2_prime = s_p;
		}
		else if(dist <= rbar){
			 si2 = 1.0;
			 si2_prime << 0.0,0.0;
		}
		else std::cout << "An unexpected case just happened!!!" << std::endl;
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
		weight << 0.5,0.5;
		
		Eigen::MatrixXd weightMat;
		weightMat = weight.asDiagonal();
		
		return statesVec.transpose()*weightMat*statesVec;
	}
	Eigen::VectorXd QzSum()
	{

		Eigen::VectorXd sum(1);
		sum << 0;
		
		Eigen::VectorXd weight(obstacles.at(0).states.rows());
		weight << 2.0,2.0;	
		
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

