#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>

class AR_Path
{
	public:
	std::vector<tf::Vector3> path_in_world;
	std::vector<tf::Vector3> path_in_image;
	std::vector<cv::Point> path_in_pixel;
	std::vector<double> alpha;
	std::vector<cv::Scalar> color;
	
	std::vector<cv::Point> path_output;
	std::vector<double> alpha_output;
	std::vector<cv::Scalar> color_output;
	
};

class AR_Triangle
{
	
	public:
	cv::Point pts[3];
	cv::Scalar color;
	double alpha;
	
		AR_Triangle(cv::Point p0, cv::Point p1, cv::Point p2, cv::Scalar c, double a)
		{
			pts[0] = p0;
			pts[1] = p1;
			pts[2] = p2;
			color = c;
			alpha = a;
		}
};
class Cone
{
	public:
		double radius;
		double height;
		tf::Vector3 bottom_center;
		tf::Vector3 top_center;
		cv::Scalar color;
		cv::Scalar top_color;
		cv::Scalar bottom_color;
		double alpha;
		std::vector<tf::Vector3> bottom;
		std::vector<cv::Point> bottom_in_pix;
		std::vector<tf::Vector3> top;
		cv::Point bottom_center_pix;
		cv::Point top_center_pix;
		std::vector<cv::Point> top_in_pix;
		std::vector<AR_Triangle> triangles;
		std::vector<cv::Point> overlayPts;
		bool drawTop;
		bool drawBottom;
			
		Cone(double r, double h, tf::Vector3 c, cv::Scalar s, cv::Scalar ts, cv::Scalar bs, double a)
		{
			color = s;
			top_color = ts;
			bottom_color = bs;
			alpha = a;
			radius = r;
			height = h;
			bottom_center = c;
			top_center = tf::Vector3(c.getX(),c.getY(),c.getZ()+h);
			
			drawTop = 0;
			drawBottom = 0;
			
			for(double i = 0; i < 2*M_PI; i+=0.1)
			{
				bottom.push_back(tf::Vector3(radius*cos(i)+bottom_center.getX(), radius*sin(i)+bottom_center.getY(), bottom_center.getZ()));
				top.push_back(tf::Vector3(radius*cos(i)+top_center.getX(), radius*sin(i)+top_center.getY(), top_center.getZ()));
			}
		}
		void createTriangles()
		{
			for(int i = 0; i < bottom_in_pix.size()-1; i++)
			{

				triangles.push_back(AR_Triangle(bottom_in_pix.at(i),bottom_in_pix.at(i+1),top_center_pix,color,alpha));
			}
			triangles.push_back(AR_Triangle(bottom_in_pix.at(bottom_in_pix.size()-1),top_center_pix,bottom_in_pix.at(0),color,alpha));
			
			
			if(drawBottom)
			{
				for(int i = 0; i < bottom_in_pix.size()-1; i++)
				{
					triangles.push_back(AR_Triangle(bottom_in_pix.at(i),bottom_in_pix.at(i+1),bottom_center_pix,bottom_color,alpha));
				}
				triangles.push_back(AR_Triangle(bottom_in_pix.at(0),bottom_in_pix.at(bottom_in_pix.size()-1),bottom_center_pix,bottom_color,alpha));
			}
			
			if(drawTop)
			{
				for(int i = 0; i < bottom_in_pix.size()-1; i++)
				{
					triangles.push_back(AR_Triangle(top_in_pix.at(i),top_in_pix.at(i+1),top_center_pix,top_color,alpha));
				}
				triangles.push_back(AR_Triangle(top_in_pix.at(0),top_in_pix.at(top_in_pix.size()-1),top_center_pix,top_color,alpha));
			}
		}
};
class Cylinder
{
	public:
		double top_radius;
		double bottom_radius;
		double height;
		tf::Vector3 bottom_center;
		tf::Vector3 top_center;
		cv::Scalar color;
		cv::Scalar top_color;
		cv::Scalar bottom_color;
		double alpha;
		std::vector<tf::Vector3> bottom;
		std::vector<cv::Point> bottom_in_pix;
		std::vector<tf::Vector3> top;
		cv::Point bottom_center_pix;
		cv::Point top_center_pix;
		std::vector<cv::Point> top_in_pix;
		std::vector<AR_Triangle> triangles;
		std::vector<cv::Point> overlayPts;
		bool drawTop;
		bool drawBottom;
			
		Cylinder(double tr, double br, double h, tf::Vector3 c, cv::Scalar s, cv::Scalar ts, cv::Scalar bs, double a, bool dT, bool dB)
		{
			color = s;
			top_color = ts;
			bottom_color = bs;
			alpha = a;
			top_radius = tr;
			bottom_radius = br;
			height = h;
			bottom_center = c;
			top_center = tf::Vector3(c.getX(),c.getY(),c.getZ()+h);
			
			drawTop = dT;
			drawBottom = dB;
			
			for(double i = 0; i < 2*M_PI; i+=0.1)
			{
				bottom.push_back(tf::Vector3(bottom_radius*cos(i)+bottom_center.getX(), bottom_radius*sin(i)+bottom_center.getY(), bottom_center.getZ()));
				top.push_back(tf::Vector3(top_radius*cos(i)+top_center.getX(), top_radius*sin(i)+top_center.getY(), top_center.getZ()));
			}
		}
		void createTriangles()
		{
			for(int i = 0; i < bottom_in_pix.size()-1; i++)
			{

				triangles.push_back(AR_Triangle(bottom_in_pix.at(i),top_in_pix.at(i),top_in_pix.at(i+1),color,alpha));
				triangles.push_back(AR_Triangle(bottom_in_pix.at(i),bottom_in_pix.at(i+1),top_in_pix.at(i+1),color,alpha));
			}
			triangles.push_back(AR_Triangle(bottom_in_pix.at(bottom_in_pix.size()-1),top_in_pix.at(bottom_in_pix.size()-1),bottom_in_pix.at(0),color,alpha));
			triangles.push_back(AR_Triangle(bottom_in_pix.at(0),top_in_pix.at(bottom_in_pix.size()-1),top_in_pix.at(0),color,alpha));
			
			
			if(drawBottom)
			{
				for(int i = 0; i < bottom_in_pix.size()-1; i++)
				{
					triangles.push_back(AR_Triangle(bottom_in_pix.at(i),bottom_in_pix.at(i+1),bottom_center_pix,bottom_color,alpha));
				}
				triangles.push_back(AR_Triangle(bottom_in_pix.at(0),bottom_in_pix.at(bottom_in_pix.size()-1),bottom_center_pix,bottom_color,alpha));
			}
			
			if(drawTop)
			{
				for(int i = 0; i < bottom_in_pix.size()-1; i++)
				{
					triangles.push_back(AR_Triangle(top_in_pix.at(i),top_in_pix.at(i+1),top_center_pix,top_color,alpha));
				}
				triangles.push_back(AR_Triangle(top_in_pix.at(0),top_in_pix.at(top_in_pix.size()-1),top_center_pix,top_color,alpha));
			}
		}
};
class Obstacle
{
public:
	Eigen::Vector2d states;
	Obstacle()
	{	
		states << 0,0;
	}
	
	
};
class Obstacle_subscriber
{
	ros::NodeHandle nho;
	ros::Subscriber mocapPose;

	
public:	
	AR_Path ra,rbar,rd;
	Eigen::Vector3d pose;
	Obstacle_subscriber(std::string bebopName, bool use_true_obstacles)
	{
		pose << 0.0,0.0,0.0;
		
		for (double i = 0; i <= 2*M_PI+0.1; i+=0.1)	{
			ra.path_in_world.push_back(0.2*tf::Vector3(cos(i), sin(i),0));
			rbar.path_in_world.push_back(0.45*tf::Vector3(cos(i), sin(i),0));
			rd.path_in_world.push_back(0.75*tf::Vector3(cos(i), sin(i),0));
		}		
		
		ra.color = std::vector<cv::Scalar>(ra.path_in_world.size(), cv::Scalar(0,0,255));
		ra.alpha = std::vector<double>(ra.path_in_world.size(), 0.3);
		rbar.color = std::vector<cv::Scalar>(rbar.path_in_world.size(), cv::Scalar(0,255,255));
		rbar.alpha = std::vector<double>(rbar.path_in_world.size(), 0.3);
		rd.color = std::vector<cv::Scalar>(rd.path_in_world.size(), cv::Scalar(0,255,0));
		rd.alpha = std::vector<double>(rd.path_in_world.size(), 0.3);
		
		if(use_true_obstacles) mocapPose = nho.subscribe(bebopName+"/desPose",1,&Obstacle_subscriber::mocapCB,this);
		else mocapPose = nho.subscribe(bebopName+"/desPose",1,&Obstacle_subscriber::mocapCB,this);
	}
	void mocapCB(const geometry_msgs::PoseStampedConstPtr& msg)
	{
		pose << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
		return;
	}
};


class AR_Projector
{
	ros::NodeHandle nh;
	ros::Subscriber mocapSub,bebopSub,switchingSub;
	ros::Publisher arImgPub, imgPub;
	image_transport::ImageTransport it;
	image_transport::CameraSubscriber camSub;
	bool new_pose,get_bebop_pose, get_tripod_pose, get_image;
	
	// Draw Axes
	AR_Path xAxis;
	AR_Path yAxis;
	AR_Path zAxis;
	AR_Path bebop_path;

	geometry_msgs::PoseStamped tripodCamPose, last_tripodCamPose, bebopPose, switchingPose;
	
	tf::Quaternion qic;
	tf::Transform tic;
    std::vector<Obstacle_subscriber *> obstacle_sub;
	std::vector<std::string> obstacleNames;
	
	cv::Mat bg;
		
public:
	
	AR_Projector() : it(nh) 
	{
		//initialize
		ros::NodeHandle nhp("~");
		camSub = it.subscribeCamera("camera/image_raw",1,&AR_Projector::imageCB,this);
		mocapSub = nh.subscribe("tripodcam/mocapPose",1,&AR_Projector::mocapCB,this);
		bebopSub = nh.subscribe("bebop4/mocapPose",1,&AR_Projector::bebopCB,this);
		nhp.param<std::vector<std::string>>("obstacleNames",obstacleNames,{"bebop0","bebop2","bebop3"});
		imgPub = nh.advertise<sensor_msgs::Image>("outputImage",1);

		new_pose = false;
		get_bebop_pose = false;
		get_tripod_pose = false;
		get_image = false;
		
		tripodCamPose.pose.position.x = 0;
		tripodCamPose.pose.position.y = 0;
		tripodCamPose.pose.position.z = 0;
		
		qic = tf::Quaternion(-0.511255, 0.491203, -0.503668, 0.493576);
		tic = tf::Transform(qic,tf::Vector3(0.0960949, -0.0554466, -0.0207288));
		
		
		Obstacle_subscriber * obj;
		
		for(int i = 0; i < 3; i++)
		{
			obj = new Obstacle_subscriber(obstacleNames.at(i),false);
			obstacle_sub.push_back(obj);
		}
		
		// axes
		
		xAxis.path_in_world.push_back(tf::Vector3(0,0,0));
		xAxis.path_in_world.push_back(tf::Vector3(0.2,0,0));
		yAxis.path_in_world.push_back(tf::Vector3(0,0,0));
		yAxis.path_in_world.push_back(tf::Vector3(0,0.2,0));
		zAxis.path_in_world.push_back(tf::Vector3(0,0,0));
		zAxis.path_in_world.push_back(tf::Vector3(0,0,0.2));			
		
		xAxis.color = std::vector<cv::Scalar>(xAxis.path_in_world.size(),cv::Scalar(0,0,255));
		yAxis.color = std::vector<cv::Scalar>(yAxis.path_in_world.size(),cv::Scalar(0,255,0));			
		zAxis.color = std::vector<cv::Scalar>(zAxis.path_in_world.size(),cv::Scalar(255,0,0));
		
		xAxis.alpha = std::vector<double>(xAxis.path_in_world.size(),1);
		yAxis.alpha = std::vector<double>(yAxis.path_in_world.size(),1);			
		zAxis.alpha = std::vector<double>(zAxis.path_in_world.size(),1);	
	
	}
	void bebopCB(const geometry_msgs::PoseStampedPtr& bebopPosePtr)
	{
		bebopPose = *bebopPosePtr;
		get_bebop_pose = true;
		return;
	}
	void mocapCB(const geometry_msgs::PoseStampedPtr& tripodCamPosePtr)
	{
		if(!get_tripod_pose) {
			tripodCamPose = *tripodCamPosePtr;
			get_tripod_pose = true;
		}

		//std::cout << tripodCamPose << std::endl;
		return;
	}
	void imageCB(const sensor_msgs::ImageConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr& camInfoMsg)
	{			
		
		//if (!get_switching || !get_bebop_pose) return;
		
		sensor_msgs::CameraInfo cam_info = *camInfoMsg;

		cv_bridge::CvImagePtr cv_ptr; 
		cv_ptr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8); //convert sensor msg to opencv-compatible cvimage
		cv::Mat img = cv_ptr->image;

		bool skip = 0;
		if(skip? tripodCamPose.pose.position.x != last_tripodCamPose.pose.position.x : 1)
		{
			last_tripodCamPose = tripodCamPose;
			// Find new transformation
		
			tf::Quaternion qcw(tripodCamPose.pose.orientation.x,tripodCamPose.pose.orientation.y,tripodCamPose.pose.orientation.z,tripodCamPose.pose.orientation.w);
			tf::Transform tcw(qcw,tf::Vector3(tripodCamPose.pose.position.x,tripodCamPose.pose.position.y,tripodCamPose.pose.position.z));
		
			//tf::Transform tiw = tcw*tic; // Transforms point from image coordinate to world coordinate.
			tf::Transform twi = tic.inverse()*tcw.inverse();

			world2px(xAxis,twi,cam_info);
			world2px(yAxis,twi,cam_info);
			world2px(zAxis,twi,cam_info);
			
			// Draw goal
			
			for(double i = 0; i < 0.5; i+=0.01)
			{
				Cylinder cyl(0.2,0.2,0.01,tf::Vector3(2.5,0,i), cv::Scalar(0,255,255),cv::Scalar(0,255,255),cv::Scalar(0,255,255),0.7*(1-i/0.5),0,false);
				for(int i = 0; i < cyl.bottom.size(); i++)
					{
						cyl.bottom_in_pix.push_back(world2pix(cyl.bottom.at(i),twi,cam_info));
						cyl.top_in_pix.push_back(world2pix(cyl.top.at(i),twi,cam_info));
					}
					cyl.bottom_center_pix = world2pix(cyl.bottom_center,twi,cam_info);
					cyl.top_center_pix = world2pix(cyl.top_center,twi,cam_info);

					cyl.createTriangles();
					
					for(int i = 0; i < cyl.triangles.size(); i++)
					{
					
						img = drawTriangle(cyl.triangles.at(i),img,false);
					}	
			}
			
			
			// Draw obstacle
			AR_Path draw_circle;
			double radius = 0.2;
			double resolution = 8;
			
			for(int i = 0; i < 3; i++)
			{
				draw_circle.path_in_world = obstacle_sub.at(i)->ra.path_in_world;
				draw_circle.color = obstacle_sub.at(i)->ra.color;
				draw_circle.alpha = obstacle_sub.at(i)->ra.alpha;
				for(int j = 0; j < draw_circle.path_in_world.size(); j++)
				{
					draw_circle.path_in_world.at(j) += tf::Vector3(obstacle_sub.at(i)->pose(0),obstacle_sub.at(i)->pose(1),0.0);
				}
				world2px(draw_circle,twi,cam_info);
				img = overlayPath(draw_circle,img,1);
				resetProj(draw_circle);
				
				draw_circle.path_in_world = obstacle_sub.at(i)->rbar.path_in_world;
				draw_circle.color = obstacle_sub.at(i)->rbar.color;
				draw_circle.alpha = obstacle_sub.at(i)->rbar.alpha;
				for(int j = 0; j < draw_circle.path_in_world.size(); j++)
				{
					draw_circle.path_in_world.at(j) += tf::Vector3(obstacle_sub.at(i)->pose(0),obstacle_sub.at(i)->pose(1),0.0);
				}
				world2px(draw_circle,twi,cam_info);
				img = overlayPath(draw_circle,img,1);
				resetProj(draw_circle);
				
				draw_circle.path_in_world = obstacle_sub.at(i)->rd.path_in_world;
				draw_circle.color = obstacle_sub.at(i)->rd.color;
				draw_circle.alpha = obstacle_sub.at(i)->rd.alpha;
				for(int j = 0; j < draw_circle.path_in_world.size(); j++)
				{
					draw_circle.path_in_world.at(j) += tf::Vector3(obstacle_sub.at(i)->pose(0),obstacle_sub.at(i)->pose(1),0.0);
				}
				world2px(draw_circle,twi,cam_info);
				img = overlayPath(draw_circle,img,1);
				resetProj(draw_circle);
				
				for (double k = 0; k < M_PI/2; k+=M_PI/2/resolution)
				{	
					double height = radius*sin(k);
					double delta_height = radius*sin(k+M_PI/2/resolution)-radius*sin(k);
					
					Cylinder cyl((pow(radius,2.0)-pow(height+delta_height,2.0) > 0 ? sqrt(pow(radius,2.0)-pow(height+delta_height,2.0)): 0),sqrt(pow(radius,2.0)-pow(height,2.0)),delta_height,tf::Vector3(0,0,height)+tf::Vector3(obstacle_sub.at(i)->pose(0),obstacle_sub.at(i)->pose(1),obstacle_sub.at(i)->pose(2)), cv::Scalar(0,0,255),cv::Scalar(0,0,200),cv::Scalar(0,0,200),0.3,0,0);					
					for(int i = 0; i < cyl.bottom.size(); i++)
					{
						cyl.bottom_in_pix.push_back(world2pix(cyl.bottom.at(i),twi,cam_info));
						cyl.top_in_pix.push_back(world2pix(cyl.top.at(i),twi,cam_info));
					}
					cyl.bottom_center_pix = world2pix(cyl.bottom_center,twi,cam_info);
					cyl.top_center_pix = world2pix(cyl.top_center,twi,cam_info);

					cyl.createTriangles();
					
					for(int i = 0; i < cyl.triangles.size(); i++)
					{
					
						img = drawTriangle(cyl.triangles.at(i),img,true);
					}
				}
				for (double k = 0; k > -M_PI/2; k-=M_PI/2/resolution)
				{	
					double height = radius*sin(k);
					double delta_height = radius*sin(k)-radius*sin(k-M_PI/2/resolution);
					Cylinder cyl(sqrt(pow(radius,2.0)-pow(height,2.0)),(pow(radius,2.0)-pow(height-delta_height,2.0) > 0 ? sqrt(pow(radius,2.0)-pow(height-delta_height,2.0)): 0),delta_height,tf::Vector3(0,0,height-delta_height)+tf::Vector3(obstacle_sub.at(i)->pose(0),obstacle_sub.at(i)->pose(1),obstacle_sub.at(i)->pose(2)), cv::Scalar(0,0,255),cv::Scalar(0,0,200),cv::Scalar(0,0,200),0.3,0,0);					
					for(int i = 0; i < cyl.bottom.size(); i++)
					{
						cyl.bottom_in_pix.push_back(world2pix(cyl.bottom.at(i),twi,cam_info));
						cyl.top_in_pix.push_back(world2pix(cyl.top.at(i),twi,cam_info));
					}
					cyl.bottom_center_pix = world2pix(cyl.bottom_center,twi,cam_info);
					cyl.top_center_pix = world2pix(cyl.top_center,twi,cam_info);

					cyl.createTriangles();
					
					for(int i = 0; i < cyl.triangles.size(); i++)
					{
					
						img = drawTriangle(cyl.triangles.at(i),img,true);
					}
				}	
			
			}
		
			// Bebop path
			if(bebopPose.pose.position.x != 0)
			{
			bebop_path.path_in_world.push_back(tf::Vector3(bebopPose.pose.position.x,bebopPose.pose.position.y,0));
			bebop_path.color.push_back(cv::Scalar(255,0,0)); 
			bebop_path.alpha.push_back(1);
			
			while (bebop_path.path_in_world.size() > 20)
			{
				bebop_path.path_in_world.erase(bebop_path.path_in_world.begin()); // keep latest data buffer
				bebop_path.color.erase(bebop_path.color.begin()); // keep latest data buffer
				bebop_path.alpha.erase(bebop_path.alpha.begin());
			}
			for(double i = 0; i < bebop_path.alpha.size();i++)
			{
				bebop_path.alpha.at(i) = tanh(i/bebop_path.alpha.size());
			}
			world2px(bebop_path,twi,cam_info);
			img = overlayPath(bebop_path,img,3);
			resetProj(bebop_path);		
	
			AR_Path pointer;
			pointer.path_in_world.push_back(tf::Vector3(bebopPose.pose.position.x,bebopPose.pose.position.y,0));
			pointer.path_in_world.push_back(tf::Vector3(bebopPose.pose.position.x,bebopPose.pose.position.y,bebopPose.pose.position.z-0.05));
			pointer.color = std::vector<cv::Scalar>(zAxis.path_in_world.size(),cv::Scalar(0255,0,0));
			pointer.alpha = std::vector<double>(xAxis.path_in_world.size(),0.2);
			
			world2px(pointer,twi,cam_info);
			img = overlayPath(pointer,img,1);
			resetProj(pointer);
			}
			
			/*
			img = overlayPath(yAxis,img,3);
			img = overlayPath(xAxis,img,3);
			img = overlayPath(zAxis,img,3);
			
			*/
			resetProj(yAxis);
			resetProj(xAxis);
			resetProj(zAxis);

		}
		imgPub.publish(cv_ptr->toImageMsg()); //publish sensor msg
		return;
	}
	
	void world2px(AR_Path& path,tf::Transform& twi, sensor_msgs::CameraInfo cam_info)
	{
		for (int i = 0 ; i < path.path_in_world.size(); i++)
		{
			path.path_in_image.push_back(world2img(path.path_in_world.at(i),twi));
			path.path_in_image.at(i) = normalizeZ(path.path_in_image.at(i));
			path.path_in_pixel.push_back(proj2px(path.path_in_image.at(i),cam_info));
		} 
		
		drawLine(path);
		
		return;
	}
	cv::Point world2pix(tf::Vector3& pt,tf::Transform& twi, sensor_msgs::CameraInfo cam_info)
	{
			tf::Vector3 pt_in_image = world2img(pt,twi);
			pt_in_image = normalizeZ(pt_in_image);
			cv::Point pt_in_pixel = proj2px(pt_in_image,cam_info);
		return pt_in_pixel;
	}
	void drawLine(AR_Path& path) // Bresenham's line algorithm
	{	
		for (int i = 0; i < path.path_in_pixel.size()-1; i++)
		{
			int x0 = path.path_in_pixel.at(i).x; 
			int x1 = path.path_in_pixel.at(i+1).x; 
			int y0 = path.path_in_pixel.at(i).y; 
			int y1 = path.path_in_pixel.at(i+1).y; 
			bool steep = false;
			if(std::abs(x0-x1)<std::abs(y0-y1))
			{
				std::swap(x0,y0);
				std::swap(x1,y1);
				steep = true;
			}
			if(x0>x1)
			{
				std::swap(x0,x1);
				std::swap(y0,y1);
			}
			int dx = x1-x0;
			int dy = y1-y0;
			int derror2 = std::abs(dy)+std::abs(dy);
			int error2 = 0;
			int y = y0;
			for(int x = x0; x<=x1; x++)
			{
				path.color_output.push_back(path.color.at(i));
				path.alpha_output.push_back(path.alpha.at(i));
				if(steep)
				{
					path.path_output.push_back(cv::Point(y,x));
				}
				else
				{
					path.path_output.push_back(cv::Point(x,y));
				}
				error2 += derror2;
				if(error2 > dx)
				{
					y += (y1>y0?1:-1);
					error2 -= dx+dx;
				}
			}
		}
		return;
	}
	cv::Mat drawTriangle(AR_Triangle& tri, cv::Mat img, bool drawGrid) // Line-sweeping algorithm (old school method)
	{
		cv::Point t0 = tri.pts[0];
		cv::Point t1 = tri.pts[1];
		cv::Point t2 = tri.pts[2];
		if(t0.y == t1.y && t0.y == t2.y) return img;
		
		if(t0.y>t1.y) std::swap(t0,t1);
		if(t0.y>t2.y) std::swap(t0,t2);
		if(t1.y>t2.y) std::swap(t1,t2);
		
		int total_height = t2.y-t0.y;
		for(int i = 0 ; i < total_height; i++)
		{
			bool second_half = i > t1.y-t0.y || t1.y == t0.y;
			int segment_height = second_half?(t2.y-t1.y):(t1.y-t0.y);
			float gamma = (float)i/total_height;
			float beta = (float)(i-(second_half?(t1.y-t0.y):0))/segment_height;
			cv::Point A = t0 + (t2-t0)*gamma;
			cv::Point B = second_half? (t1+(t2-t1)*beta):(t0+(t1-t0)*beta);
			if(A.x > B.x) std::swap(A,B);
			if(drawGrid)
			{
				for(int j=A.x; j<=B.x;j++)
				{
					img = overlayPix(j,t0.y+i, img, tri.color,tri.alpha);
				}
			}
			else
			{
				for(int j=A.x; j<B.x;j++)
				{
					img = overlayPix(j,t0.y+i, img, tri.color,tri.alpha);
				}
			}
		}
		return img;
	}
	void resetProj(AR_Path& path)
	{
		path.path_in_image.clear();
		path.path_in_pixel.clear();
		path.path_output.clear();
		path.color_output.clear();
		path.alpha_output.clear();
	}
	cv::Mat overlayPath(AR_Path& vec, cv::Mat img, double thickness) 
	{
		for (int i = 0; i < vec.path_output.size(); i++)
		{
			for(int j = -1*thickness; j <= thickness; j++)
			{
				for(int k = -1*thickness; k <= thickness; k++)
				{	
					if(vec.path_output.at(i).y-j < 0 || vec.path_output.at(i).y+j > img.rows || vec.path_output.at(i).x-k < 0 || vec.path_output.at(i).x+k > img.cols) continue;		
					img.at<cv::Vec3b>(vec.path_output.at(i).y+j,vec.path_output.at(i).x+k)[0] = img.at<cv::Vec3b>(vec.path_output.at(i).y+j,vec.path_output.at(i).x+k)[0]*(1-vec.alpha_output.at(i))+vec.color_output.at(i).val[0]*vec.alpha_output.at(i); 
					img.at<cv::Vec3b>(vec.path_output.at(i).y+j,vec.path_output.at(i).x+k)[1] = img.at<cv::Vec3b>(vec.path_output.at(i).y+j,vec.path_output.at(i).x+k)[1]*(1-vec.alpha_output.at(i))+vec.color_output.at(i).val[1]*vec.alpha_output.at(i); 
					img.at<cv::Vec3b>(vec.path_output.at(i).y+j,vec.path_output.at(i).x+k)[2] = img.at<cv::Vec3b>(vec.path_output.at(i).y+j,vec.path_output.at(i).x+k)[2]*(1-vec.alpha_output.at(i))+vec.color_output.at(i).val[2]*vec.alpha_output.at(i); 		
				}
			}
		}
				
		return img;
	}
	
	cv::Mat overlayPix(int x,int y, cv::Mat img, cv::Scalar color, double alpha)
	{
		//std::cout << "overlayPix" << std::endl;
		if(y < 0 || y > img.rows || x < 0 || x > img.cols) return img;
		img.at<cv::Vec3b>(y,x)[0] = img.at<cv::Vec3b>(y,x)[0]*(1-alpha)+color.val[0]*alpha; 
		img.at<cv::Vec3b>(y,x)[1] = img.at<cv::Vec3b>(y,x)[1]*(1-alpha)+color.val[1]*alpha; 
		img.at<cv::Vec3b>(y,x)[2] = img.at<cv::Vec3b>(y,x)[2]*(1-alpha)+color.val[2]*alpha; 
		return img;
	}
	
	tf::Vector3 world2img(tf::Vector3& point, tf::Transform& transformation)
	{
		tf::Vector3 trans_pt(transformation*point); // Transform from world frame to image frame.
		return trans_pt;
	}
	
	tf::Vector3 normalizeZ(tf::Vector3& point)
	{
		point.setX(point.getX()/point.getZ());
		point.setY(point.getY()/point.getZ());
		point.setZ(point.getZ()/point.getZ());

		return point;
	}
	
	cv::Point proj2px(tf::Vector3& point, sensor_msgs::CameraInfo cam_info)
	{
		cv::Point proj_pt(cam_info.K[0]*point.getX()+cam_info.K[2],cam_info.K[4]*point.getY()+cam_info.K[5]); //Pin-hole model projection.
		return proj_pt;
	}
	

	
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "AR_Projector_Node");
	
	AR_Projector ar_project;
	ros::spin();
	return 0;
	
}
	
