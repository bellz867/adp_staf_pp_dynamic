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
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgcodecs.hpp>

class Object
{
public:
	std::string name;
	tf::Vector3 center_in_world;
	tf::Vector3 center_in_image; //used to determine object layer.
	cv::Point center_in_pixel; 
	sensor_msgs::CameraInfo* cam_info;
	tf::Transform* world2img_tf;
	cv::Mat parent_img;
	
	cv::Point world2px(tf::Vector3& pt)
	{
		// Transform to image frame
		tf::Vector3 pt2 = *world2img_tf*pt;
				
		// Normalize w.r.t. Z axis
		pt2.setX(pt2.getX()/pt2.getZ());
		pt2.setY(pt2.getY()/pt2.getZ());
		pt2.setZ(pt2.getZ()/pt2.getZ());
		
		// Apply the pin-hole model projection
		cv::Point pt3(cam_info->K[0]*pt2.getX()+cam_info->K[2],cam_info->K[4]*pt2.getY()+cam_info->K[5]);
		
		return pt3;
	}
	tf::Vector3 world2img(tf::Vector3& pt)
	{
		return *world2img_tf*pt;
	}
	void setName(std::string name)
	{
		this->name = name;
	}
	virtual void draw(cv::Mat){}
};
class Triangle
{
	
	public:
	cv::Point pts[3];
	cv::Scalar color;
	double alpha;
	
	Triangle(cv::Point p0, cv::Point p1, cv::Point p2, cv::Scalar c, double a)
	{
		pts[0] = p0;
		pts[1] = p1;
		pts[2] = p2;
		color = c;
		alpha = a;
	}
	void drawTriangle(cv::Mat img, bool& drawGrid) // Line-sweeping algorithm (old school method)
	{
		if(pts[0].y == pts[1].y && pts[0].y == pts[2].y) return;
		
		if(pts[0].y>pts[1].y) std::swap(pts[0],pts[1]);
		if(pts[0].y>pts[2].y) std::swap(pts[0],pts[2]);
		if(pts[1].y>pts[2].y) std::swap(pts[1],pts[2]);
		
		int total_height = pts[2].y-pts[0].y;
		for(int i = 0 ; i < total_height; i++)
		{
			bool second_half = i > pts[1].y-pts[0].y || pts[1].y == pts[0].y;
			int segment_height = second_half?(pts[2].y-pts[1].y):(pts[1].y-pts[0].y);
			float gamma = (float)i/total_height;
			float beta = (float)(i-(second_half?(pts[1].y-pts[0].y):0))/segment_height;
			cv::Point A = pts[0] + (pts[2]-pts[0])*gamma;
			cv::Point B = second_half? (pts[1]+(pts[2]-pts[1])*beta):(pts[0]+(pts[1]-pts[0])*beta);
			if(A.x > B.x) std::swap(A,B);
			if(drawGrid)
			{
				for(int j=A.x; j<=B.x;j++)
				{
					overlayPx(j,pts[0].y+i, img, color,alpha);
				}
			}
			else
			{
				for(int j=A.x; j<B.x;j++)
				{
					overlayPx(j,pts[0].y+i, img, color,alpha);
				}
			}
		}
		return;
	}
	void overlayPx(int x,int y, cv::Mat parent_img, cv::Scalar& color, double& alpha)
	{		
				if(y < 0 || y > parent_img.rows || x < 0 || x > parent_img.cols) return;		
				parent_img.at<cv::Vec3b>(y,x)[0] = parent_img.at<cv::Vec3b>(y,x)[0]*(1-alpha)+color.val[0]*alpha; 
				parent_img.at<cv::Vec3b>(y,x)[1] = parent_img.at<cv::Vec3b>(y,x)[1]*(1-alpha)+color.val[1]*alpha; 
				parent_img.at<cv::Vec3b>(y,x)[2] = parent_img.at<cv::Vec3b>(y,x)[2]*(1-alpha)+color.val[2]*alpha; 		

		return;		
	}
};
class Object3D : public Object
{
public:
	std::vector<tf::Vector3> pts_in_world;
	std::vector<double> alpha;
	std::vector<cv::Scalar> color;	
	std::vector<Triangle> triangles;
	bool drawGrid;
	void drawObject()
	{
		for(int i = 0; i < triangles.size(); i++)
		{
			triangles.at(i).drawTriangle(parent_img, drawGrid);
		}
		return;
	}	
};
class Object2D : public Object
{
	
public:
	std::vector<tf::Vector3> pts_in_world;
	std::vector<double> alpha;
	std::vector<cv::Scalar> color;	
	std::vector<Triangle> triangles;
	bool drawGrid;
	void drawObject()
	{
		for(int i = 0; i < triangles.size(); i++)
		{
			triangles.at(i).drawTriangle(parent_img, drawGrid);
		}
		return;
	}	
};
class Object1D : public Object
{	
public:
	std::vector<tf::Vector3> pts_in_world;
	std::vector<cv::Point> pts_in_pixel;
	std::vector<double> alpha;
	std::vector<cv::Scalar> color;
	double thickness;
	
	void drawObject() // Bresenham's line algorithm
	{	
		for (int i = 0; i < pts_in_pixel.size()-1; i++)
		{

			int x0 = pts_in_pixel.at(i).x; 
			int x1 = pts_in_pixel.at(i+1).x; 
			int y0 = pts_in_pixel.at(i).y; 
			int y1 = pts_in_pixel.at(i+1).y; 
			
			//std::cout << "drawing at [" <<  x0 << " " << x1 << "] [" << y0 << " " << y1 << "]" << std::endl;
			
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
				if(steep)
				{
					overlayPx(y,x,color.at(i),alpha.at(i),thickness);						
				}
				else
				{
					overlayPx(x,y,color.at(i),alpha.at(i),thickness);	
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
	void overlayPx(int& x,int& y, cv::Scalar& color, double& alpha, double& thickness)
	{		
		for(int j = -1*thickness; j <= thickness; j++)
		{
			for(int k = -1*thickness; k <= thickness; k++)
			{	
				if(y+j < 0 || y+j > parent_img.rows || x+k < 0 || x+k > parent_img.cols) continue;		
				parent_img.at<cv::Vec3b>(y+j,x+k)[0] = parent_img.at<cv::Vec3b>(y+j,x+k)[0]*(1-alpha)+color.val[0]*alpha; 
				parent_img.at<cv::Vec3b>(y+j,x+k)[1] = parent_img.at<cv::Vec3b>(y+j,x+k)[1]*(1-alpha)+color.val[1]*alpha; 
				parent_img.at<cv::Vec3b>(y+j,x+k)[2] = parent_img.at<cv::Vec3b>(y+j,x+k)[2]*(1-alpha)+color.val[2]*alpha; 		
			}
		}
		return;		
	}
};
class Generic1D : public Object1D 
{
		int buffer;	
		
public:	

		Generic1D(tf::Transform *world2img_tf, sensor_msgs::CameraInfo& cam_info, double thickness, int buffer)
		{
			this->world2img_tf = world2img_tf;
			this->buffer = buffer;
			this->thickness = thickness;
			this->cam_info = &cam_info;	
		}	
		void addPt(tf::Vector3& pt, double a, cv::Scalar c)
		{
			pts_in_world.push_back(pt);
			pts_in_pixel.push_back(world2px(pt));
			alpha.push_back(a);
			color.push_back(c);

			
			// Sliding buffer (to avoid accumulating too many points)
			while(pts_in_world.size()>buffer)
			{
				pts_in_world.erase(pts_in_world.begin());
				pts_in_pixel.erase(pts_in_pixel.begin());
				color.erase(color.begin()); 
				alpha.erase(alpha.begin());
			}

			// Make gradient alpha effect (optional)
			for(double i = 0; i < alpha.size();i++)
			{
				alpha.at(i) = tanh(i/alpha.size());
			}
			return;
		}
		void update(tf::Vector3 center)
		{
			pts_in_pixel.clear();
			for(int i = 0; i < pts_in_world.size(); i++)
			{
				pts_in_pixel.push_back(world2px(pts_in_world.at(i)));
			}
			center_in_image = world2img(center);
		}
		void draw(cv::Mat parent_img)
		{
			//std::cout << "updating..." << std::endl;
			this->parent_img = parent_img;
			drawObject();			
		}	
};
/*
 * 
class Generic2D : public Object2D
{
};
class Generic3D : public Object3D
{
};
*/
class Ellipse : public Object1D
{
	
public:
	Ellipse(tf::Transform *world2img_tf, sensor_msgs::CameraInfo& cam_info, double radius1, double radius2, double elevation, cv::Scalar color, double alpha, double thickness, double resolution)
	{
		this->world2img_tf = world2img_tf;
		this->cam_info = &cam_info;
		this->thickness = thickness;
			
		for(int i = 0; i <= resolution; i++)
		{
			pts_in_world.push_back(tf::Vector3(radius1*cos((i/resolution)*2*M_PI), radius2*sin((i/resolution)*2*M_PI), elevation));
			this->color.push_back(color);
			this->alpha.push_back(alpha);
		}
	}
	void update(tf::Vector3 center)
	{
		pts_in_pixel.clear();
		for(int i = 0; i < pts_in_world.size(); i++)
		{
			tf::Vector3 pt = pts_in_world.at(i) + center;
			pts_in_pixel.push_back(world2px(pt));
		}
		center_in_image = world2img(center);
	}
	void draw(cv::Mat parent_img)
	{
		//std::cout << "updating..." << std::endl;
		this->parent_img = parent_img;
		drawObject();			
	}	
};
class Rectangle : public Object1D
{
public:
	Rectangle(tf::Transform *world2img_tf, sensor_msgs::CameraInfo& cam_info, double width, double length, double elevation, cv::Scalar color, double alpha, double thickness)
	{
		this->world2img_tf = world2img_tf;
		this->cam_info = &cam_info;
		this->thickness = thickness;

		pts_in_world.push_back(tf::Vector3(-width/2, -length/2, elevation));
		this->color.push_back(color);
		this->alpha.push_back(alpha);
		pts_in_world.push_back(tf::Vector3(-width/2, length/2, elevation));
		this->color.push_back(color);
		this->alpha.push_back(alpha);	
		pts_in_world.push_back(tf::Vector3(width/2, length/2, elevation));
		this->color.push_back(color);
		this->alpha.push_back(alpha);	
		pts_in_world.push_back(tf::Vector3(width/2, -length/2, elevation));
		this->color.push_back(color);
		this->alpha.push_back(alpha);	
		pts_in_world.push_back(tf::Vector3(-width/2, -length/2, elevation));
		this->color.push_back(color);
		this->alpha.push_back(alpha);	
	}
	void update(tf::Vector3& center)
	{
		pts_in_pixel.clear();
		for(int i = 0; i < pts_in_world.size(); i++)
		{
			tf::Vector3 pt = pts_in_world.at(i) + center;
			pts_in_pixel.push_back(world2px(pt));
		}
		center_in_image = world2img(center);
	}
	void draw(cv::Mat parent_img)
	{
		//std::cout << "updating..." << std::endl;
		this->parent_img = parent_img;
		drawObject();			
	}	
};
class EllipseFilled : public Object2D
{
public:
	EllipseFilled(tf::Transform *world2img_tf, sensor_msgs::CameraInfo& cam_info, double radius1, double radius2, double elevation, cv::Scalar color, double alpha, double resolution, bool drawGrid)
	{
		this->world2img_tf = world2img_tf;
		this->cam_info = &cam_info;
		this->center_in_world = tf::Vector3(0,0,elevation);
		this->drawGrid = drawGrid;
			
		for(int i = 0; i <= resolution; i++)
		{
			pts_in_world.push_back(tf::Vector3(radius1*cos((i/resolution)*2*M_PI), radius2*sin((i/resolution)*2*M_PI), elevation));
			this->color.push_back(color);
			this->alpha.push_back(alpha);
		}
		for(int i = 0; i < pts_in_world.size()-1; i++)
		{
			triangles.push_back(Triangle(world2px(pts_in_world.at(i)),world2px(pts_in_world.at(i+1)),world2px(center_in_world),this->color.at(i),this->alpha.at(i)));
		}
	}
	void update(tf::Vector3 center)
	{
		for(int i = 0; i < pts_in_world.size()-1; i++)
		{
			tf::Vector3 pt1 = pts_in_world.at(i)+center;
			tf::Vector3 pt2 = pts_in_world.at(i+1)+center;
			tf::Vector3 pt3 = center_in_world+center;
			triangles.at(i) = Triangle(world2px(pt1),world2px(pt2),world2px(pt3),color.at(i),alpha.at(i));
		}
		center_in_image = world2img(center);
		return;
	}
	void draw(cv::Mat parent_img)
	{
		//std::cout << "updating..." << std::endl;
		this->parent_img = parent_img;
		drawObject();
		return;			
	}	
};
class RectangleFilled : public Object2D
{
public:
	RectangleFilled(tf::Transform *world2img_tf, sensor_msgs::CameraInfo& cam_info, double width, double length, double elevation, cv::Scalar color, double alpha, bool drawGrid)
	{
		this->world2img_tf = world2img_tf;
		this->cam_info = &cam_info;
		this->center_in_world = tf::Vector3(0,0,elevation);
		this->drawGrid = drawGrid;
			
		pts_in_world.push_back(tf::Vector3(-width/2, -length/2, elevation));
		this->color.push_back(color);
		this->alpha.push_back(alpha);
		pts_in_world.push_back(tf::Vector3(-width/2, length/2, elevation));
		this->color.push_back(color);
		this->alpha.push_back(alpha);	
		pts_in_world.push_back(tf::Vector3(width/2, length/2, elevation));
		this->color.push_back(color);
		this->alpha.push_back(alpha);	
		pts_in_world.push_back(tf::Vector3(width/2, -length/2, elevation));
		this->color.push_back(color);
		this->alpha.push_back(alpha);	
		pts_in_world.push_back(tf::Vector3(-width/2, -length/2, elevation));
		this->color.push_back(color);
		this->alpha.push_back(alpha);
		for(int i = 0; i < pts_in_world.size()-1; i++)
		{
			triangles.push_back(Triangle(world2px(pts_in_world.at(i)),world2px(pts_in_world.at(i+1)),world2px(center_in_world),this->color.at(i),this->alpha.at(i)));
		}
	}
	void update(tf::Vector3 center)
	{
		for(int i = 0; i < pts_in_world.size()-1; i++)
		{
			tf::Vector3 pt1 = pts_in_world.at(i)+center;
			tf::Vector3 pt2 = pts_in_world.at(i+1)+center;
			tf::Vector3 pt3 = center_in_world+center;
			triangles.at(i) = Triangle(world2px(pt1),world2px(pt2),world2px(pt3),color.at(i),alpha.at(i));
		}
		center_in_image = world2img(center);
		return;
	}
	void draw(cv::Mat parent_img)
	{
		//std::cout << "updating..." << std::endl;
		this->parent_img = parent_img;
		drawObject();
		return;			
	}	
};
class Backdrop : public Object2D
{
public:
	Backdrop(tf::Transform *world2img_tf, sensor_msgs::CameraInfo& cam_info, double width, double height, cv::Scalar color, double alpha, bool drawGrid)
	{
		this->world2img_tf = world2img_tf;
		this->cam_info = &cam_info;
		this->center_in_world = tf::Vector3(0,0,0);
		this->drawGrid = drawGrid;
			
		pts_in_world.push_back(tf::Vector3(-width/2, 0, -height/2));
		this->color.push_back(color);
		this->alpha.push_back(alpha);
		pts_in_world.push_back(tf::Vector3(-width/2, 0, height/2));
		this->color.push_back(color);
		this->alpha.push_back(alpha);	
		pts_in_world.push_back(tf::Vector3(width/2, 0, height/2));
		this->color.push_back(color);
		this->alpha.push_back(alpha);	
		pts_in_world.push_back(tf::Vector3(width/2, 0, -height/2));
		this->color.push_back(color);
		this->alpha.push_back(alpha);	
		pts_in_world.push_back(tf::Vector3(-width/2, 0, -height/2));
		this->color.push_back(color);
		this->alpha.push_back(alpha);
		for(int i = 0; i < pts_in_world.size()-1; i++)
		{
			triangles.push_back(Triangle(world2px(pts_in_world.at(i)),world2px(pts_in_world.at(i+1)),world2px(center_in_world),this->color.at(i),this->alpha.at(i)));
		}
	}
	void update(tf::Transform tf)
	{
		for(int i = 0; i < pts_in_world.size()-1; i++)
		{
			tf::Vector3 pt1 = tf*pts_in_world.at(i);
			tf::Vector3 pt2 = tf*pts_in_world.at(i+1);
			tf::Vector3 pt3 = tf*center_in_world;
			triangles.at(i) = Triangle(world2px(pt1),world2px(pt2),world2px(pt3),color.at(i),alpha.at(i));
			
		}
		center_in_image = world2img(tf.getOrigin());
		return;
	}
	void draw(cv::Mat parent_img)
	{
		//std::cout << "updating..." << std::endl;
		this->parent_img = parent_img;
		drawObject();
		return;			
	}	
};
class Sphere : public Object3D
{
		std::vector<std::vector<tf::Vector3>> layer_pts_in_world;
		double radius3;
		
public:
	Sphere(tf::Transform *world2img_tf, sensor_msgs::CameraInfo& cam_info, double radius1, double radius2,
				double radius3, cv::Scalar color, double alpha, double radial_res, double verticle_res, bool drawGrid)
	{
		this->world2img_tf = world2img_tf;
		this->cam_info = &cam_info;
		this->center_in_world = tf::Vector3(0,0,0);
		this->drawGrid = drawGrid;
		this->radius3 = radius3;
		
		for(int i = -verticle_res; i <= verticle_res; i++)
		{
			std::vector<tf::Vector3> temp;
			for(int j = 0; j <= radial_res; j++)
			{
				temp.push_back(tf::Vector3(radius1*cos((i/verticle_res)*M_PI/2)*cos((j/radial_res)*2*M_PI), radius2*cos((i/verticle_res)*M_PI/2)*sin((j/radial_res)*2*M_PI), radius3*sin((i/verticle_res)*M_PI/2)));
			}
			layer_pts_in_world.push_back(temp);
			this->color.push_back(color);
			this->alpha.push_back(alpha);

		}
	
		for(int i = 0; i < layer_pts_in_world.size()-1; i++)
		{
			for(int j = 0; j < layer_pts_in_world.at(i).size()-1; j++)
			{
				triangles.push_back(Triangle(world2px(layer_pts_in_world.at(i).at(j)),world2px(layer_pts_in_world.at(i+1).at(j)),world2px(layer_pts_in_world.at(i+1).at(j+1)),this->color.at(i),this->alpha.at(i)));
				triangles.push_back(Triangle(world2px(layer_pts_in_world.at(i+1).at(j+1)),world2px(layer_pts_in_world.at(i).at(j+1)),world2px(layer_pts_in_world.at(i).at(j)),this->color.at(i),this->alpha.at(i)));
			}
		}
	}
	void update(tf::Vector3 center)
	{
		triangles.clear();
		for(int i = 0; i < layer_pts_in_world.size()-1; i++)
		{
			for(int j = 0; j < layer_pts_in_world.at(i).size()-1; j++)
			{	
				tf::Vector3 pt1 = layer_pts_in_world.at(i).at(j)+center;
				tf::Vector3 pt2 = layer_pts_in_world.at(i+1).at(j)+center;
				tf::Vector3 pt3 = layer_pts_in_world.at(i+1).at(j+1)+center;
				tf::Vector3 pt4 = layer_pts_in_world.at(i).at(j+1)+center;
				triangles.push_back(Triangle(world2px(pt1),world2px(pt2),world2px(pt3),this->color.at(i),this->alpha.at(i)));
				triangles.push_back(Triangle(world2px(pt3),world2px(pt4),world2px(pt1),this->color.at(i),this->alpha.at(i)));
			}
		}
		center_in_image = world2img(center);
		return;
	}
	void draw(cv::Mat parent_img)
	{
		this->parent_img = parent_img;
		drawObject();
		
		return;			
	}	
};
class Rectangular : public Object3D
{
	std::vector<std::vector<tf::Vector3>> layer_pts_in_world;
	bool drawBot, drawTop;
	double height;
	RectangleFilled *bot_rectangle;
	RectangleFilled *top_rectangle;
public:
	Rectangular(tf::Transform *world2img_tf, sensor_msgs::CameraInfo& cam_info, double width, double length,
				double height, cv::Scalar color, double alpha, double verticle_res, bool drawGrid, bool drawBot, bool drawTop)
	{
		this->drawBot = drawBot;
		this->drawTop = drawTop;
		this->world2img_tf = world2img_tf;
		this->cam_info = &cam_info;
		this->center_in_world = tf::Vector3(0,0,0);
		this->drawGrid = drawGrid;
		this->height = height;
		
		for(int i = 0; i <= verticle_res; i++)
		{
			std::vector<tf::Vector3> temp;
			temp.push_back(tf::Vector3(width/2, length/2, height*(i/verticle_res)));
			temp.push_back(tf::Vector3(width/2, -length/2, height*(i/verticle_res)));
			temp.push_back(tf::Vector3(-width/2, -length/2, height*(i/verticle_res)));
			temp.push_back(tf::Vector3(-width/2, length/2, height*(i/verticle_res)));
			temp.push_back(tf::Vector3(width/2, length/2, height*(i/verticle_res)));
			
			layer_pts_in_world.push_back(temp);
			this->color.push_back(color);
			this->alpha.push_back((1-i/verticle_res)*alpha);

		}
	
		bot_rectangle = new RectangleFilled(world2img_tf,cam_info,width,length,0.0,color,alpha,drawGrid);
		top_rectangle = new RectangleFilled(world2img_tf,cam_info,width,length,height,color,alpha,drawGrid);
		
		for(int i = 0; i < layer_pts_in_world.size()-1; i++)
		{
			for(int j = 0; j < layer_pts_in_world.at(i).size()-1; j++)
			{
				triangles.push_back(Triangle(world2px(layer_pts_in_world.at(i).at(j)),world2px(layer_pts_in_world.at(i+1).at(j)),world2px(layer_pts_in_world.at(i+1).at(j+1)),this->color.at(i),this->alpha.at(i)));
				triangles.push_back(Triangle(world2px(layer_pts_in_world.at(i+1).at(j+1)),world2px(layer_pts_in_world.at(i).at(j+1)),world2px(layer_pts_in_world.at(i).at(j)),this->color.at(i),this->alpha.at(i)));
			}
		}
	}
	void update(tf::Vector3 center)
	{
		triangles.clear();
		for(int i = 0; i < layer_pts_in_world.size()-1; i++)
		{
			for(int j = 0; j < layer_pts_in_world.at(i).size()-1; j++)
			{	
				tf::Vector3 pt1 = layer_pts_in_world.at(i).at(j)+center;
				tf::Vector3 pt2 = layer_pts_in_world.at(i+1).at(j)+center;
				tf::Vector3 pt3 = layer_pts_in_world.at(i+1).at(j+1)+center;
				tf::Vector3 pt4 = layer_pts_in_world.at(i).at(j+1)+center;
				triangles.push_back(Triangle(world2px(pt1),world2px(pt2),world2px(pt3),this->color.at(i),this->alpha.at(i)));
				triangles.push_back(Triangle(world2px(pt3),world2px(pt4),world2px(pt1),this->color.at(i),this->alpha.at(i)));
			}
		}
		bot_rectangle->update(center);
		top_rectangle->update(center+tf::Vector3(0,0,height));
		center_in_image = world2img(center);
		return;
	}
	void draw(cv::Mat parent_img)
	{

		this->parent_img = parent_img;
		drawObject();
		
		if(drawBot) bot_rectangle->draw(parent_img);
		if(drawTop) top_rectangle->draw(parent_img);
		
		return;			
	}
};
class Cylinder : public Object3D
{
		std::vector<std::vector<tf::Vector3>> layer_pts_in_world;
		bool drawBot, drawTop;
		double height;
		EllipseFilled *bot_ellipse;
		EllipseFilled *top_ellipse;
public:
	Cylinder(tf::Transform *world2img_tf, sensor_msgs::CameraInfo& cam_info, double radius1, double radius2,
				double height, cv::Scalar color, double alpha, double radial_res, double verticle_res, bool drawGrid, bool drawBot, bool drawTop)
	{
		this->drawBot = drawBot;
		this->drawTop = drawTop;
		this->world2img_tf = world2img_tf;
		this->cam_info = &cam_info;
		this->center_in_world = tf::Vector3(0,0,0);
		this->drawGrid = drawGrid;
		this->height = height;
		
		for(int i = 0; i <= verticle_res; i++)
		{
			std::vector<tf::Vector3> temp;
			for(int j = 0; j <= radial_res; j++)
			{
				temp.push_back(tf::Vector3(radius1*cos((j/radial_res)*2*M_PI), radius2*sin((j/radial_res)*2*M_PI), height*(i/verticle_res)));
			}
			layer_pts_in_world.push_back(temp);
			this->color.push_back(color);
			this->alpha.push_back((1-i/verticle_res)*alpha);

		}
	
		bot_ellipse = new EllipseFilled(world2img_tf,cam_info,radius1,radius2,0.0,color,alpha,radial_res,drawGrid);
		top_ellipse = new EllipseFilled(world2img_tf,cam_info,radius1,radius2,height,color,alpha,radial_res,drawGrid);
		
		for(int i = 0; i < layer_pts_in_world.size()-1; i++)
		{
			for(int j = 0; j < layer_pts_in_world.at(i).size()-1; j++)
			{
				triangles.push_back(Triangle(world2px(layer_pts_in_world.at(i).at(j)),world2px(layer_pts_in_world.at(i+1).at(j)),world2px(layer_pts_in_world.at(i+1).at(j+1)),this->color.at(i),this->alpha.at(i)));
				triangles.push_back(Triangle(world2px(layer_pts_in_world.at(i+1).at(j+1)),world2px(layer_pts_in_world.at(i).at(j+1)),world2px(layer_pts_in_world.at(i).at(j)),this->color.at(i),this->alpha.at(i)));
			}
		}
	}
	void update(tf::Vector3 center)
	{
		triangles.clear();
		for(int i = 0; i < layer_pts_in_world.size()-1; i++)
		{
			for(int j = 0; j < layer_pts_in_world.at(i).size()-1; j++)
			{	
				tf::Vector3 pt1 = layer_pts_in_world.at(i).at(j)+center;
				tf::Vector3 pt2 = layer_pts_in_world.at(i+1).at(j)+center;
				tf::Vector3 pt3 = layer_pts_in_world.at(i+1).at(j+1)+center;
				tf::Vector3 pt4 = layer_pts_in_world.at(i).at(j+1)+center;
				triangles.push_back(Triangle(world2px(pt1),world2px(pt2),world2px(pt3),this->color.at(i),this->alpha.at(i)));
				triangles.push_back(Triangle(world2px(pt3),world2px(pt4),world2px(pt1),this->color.at(i),this->alpha.at(i)));
			}
		}
		bot_ellipse->update(center);
		top_ellipse->update(center+tf::Vector3(0,0,height));
		center_in_image = world2img(center);
		return;
	}
	void draw(cv::Mat parent_img)
	{
		this->parent_img = parent_img;
		drawObject();
		
		if(drawBot) bot_ellipse->draw(parent_img);
		if(drawTop) top_ellipse->draw(parent_img);
		
		return;			
	}		
};
class Obstacle_subscriber
{
	ros::NodeHandle nho;
	ros::Subscriber mocapPose;
	
public:	
	tf::Vector3 position3d;
	tf::Vector3 position2d;
	double ra;
	double rbar;
	double rd;
	
	
	Obstacle_subscriber(std::string bebopName, bool use_true_obstacles)
	{
		position3d = tf::Vector3(0,0,0);
		position2d = tf::Vector3(0,0,0);
		ra = 0.2;
		rbar = 0.45;
		rd = 0.7;
		
		if(!use_true_obstacles) mocapPose = nho.subscribe(bebopName+"/desPose",1,&Obstacle_subscriber::mocapCB,this);
		else mocapPose = nho.subscribe(bebopName+"/mocapPose",1,&Obstacle_subscriber::mocapCB,this);
	}
	void mocapCB(const geometry_msgs::PoseStampedConstPtr& msg)
	{
		position3d = tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
		position2d = tf::Vector3(msg->pose.position.x, msg->pose.position.y, 0);
		return;
	}
};
class Agent : public Object3D
{
public:
	std::vector<cv::Point> pts_in_pixel;
	cv::Mat roi;
	cv::Mat mask;
	cv::Rect rec;
	
	Agent(tf::Transform *world2img_tf, sensor_msgs::CameraInfo& cam_info, double width, double length, double height)
	{
		this->world2img_tf = world2img_tf;
		this->cam_info = &cam_info;
		pts_in_world.push_back(tf::Vector3(width/2,length/2-0.05,height/2));
		pts_in_world.push_back(tf::Vector3(width/2,length/2-0.05,-height/2));
		pts_in_world.push_back(tf::Vector3(width/2,-length/2-0.05,height/2));
		pts_in_world.push_back(tf::Vector3(width/2,-length/2-0.05,-height/2));
		pts_in_world.push_back(tf::Vector3(-width/2,length/2-0.05,height/2));
		pts_in_world.push_back(tf::Vector3(-width/2,length/2-0.05,-height/2));
		pts_in_world.push_back(tf::Vector3(-width/2,-length/2-0.05,height/2));
		pts_in_world.push_back(tf::Vector3(-width/2,-length/2-0.05,-height/2));
	}
	
	void update(tf::Vector3 center)
	{
		center_in_image = world2img(center); // to find depth from camera.
	}
	void find(cv::Mat img,cv::Mat source, tf::Transform bebop_tf)
	{
		cv::namedWindow("foreground",cv::WINDOW_AUTOSIZE);
		cv::namedWindow("foregroundrgb",cv::WINDOW_AUTOSIZE);

		double roi_x1 = source.cols;
		double roi_y1 = source.rows;
		double roi_x2 = 0;
		double roi_y2 = 0;
		for(int i = 0; i < pts_in_world.size(); i++)
		{
			tf::Vector3 newPose = bebop_tf*pts_in_world.at(i);
			if(world2px(newPose).x < roi_x1) roi_x1 = world2px(newPose).x;
			if(world2px(newPose).y < roi_y1) roi_y1 = world2px(newPose).y;
			if(world2px(newPose).x > roi_x2) roi_x2 = world2px(newPose).x;
			if(world2px(newPose).y > roi_y2) roi_y2 = world2px(newPose).y;
		}
		//	std::cout << std::endl << "size = " << roi_x1 << " " << roi_y1 << " " << roi_x2 << " " << roi_y2 << std::endl;	
		double roi_width = roi_x2-roi_x1;
		double roi_height = roi_y2-roi_y1;	
		
		if(roi_y1 < 0 || roi_y1+roi_height > source.rows || roi_x1< 0 || roi_x1+roi_width  > source.cols)
		{
			roi = roi*0.8 + cv::Scalar(0,0,255)*0.2;
			return;
		}
		rec = cv::Rect(roi_x1,roi_y1,roi_width,roi_height);

		roi = img(rec);
		//std::cout << "size = " << rec.size() << std::endl;
		mask = cv::Mat(roi.size(),CV_8UC1);
		mask = 0;
		
		cv::Mat img_hsv;
		cv::cvtColor(img,img_hsv,cv::COLOR_BGR2HSV);
		
		if(1){
			for(int i = roi_x1 ; i < roi_x1+roi_width; i++)
			{
				for(int j = roi_y1; j < roi_y1+roi_height; j++)
				{	
					if(img_hsv.at<cv::Vec3b>(j,i)[0] > 90 && img_hsv.at<cv::Vec3b>(j,i)[0] < 145 && img_hsv.at<cv::Vec3b>(j,i)[1] > 70 && img_hsv.at<cv::Vec3b>(j,i)[2] > 30 ) continue; 
					if(j < 0 || j > source.rows || i < 0 || i > source.cols) continue;
					double d0 = img.at<cv::Vec3b>(j,i)[0] - source.at<cv::Vec3b>(j,i)[0];
					double d1 = img.at<cv::Vec3b>(j,i)[1] - source.at<cv::Vec3b>(j,i)[1];
					double d2 = img.at<cv::Vec3b>(j,i)[2] - source.at<cv::Vec3b>(j,i)[2];
					if(pow(d0,2.0)+pow(d0,2.0)+pow(d0,2.0) > 300)
					{
						//std::cout << (int)img_hsv.at<cv::Vec3b>(j,i)[0] << " " << (int)img_hsv.at<cv::Vec3b>(j,i)[1] << " " << (int)img_hsv.at<cv::Vec3b>(j,i)[2] << std::endl;
						mask.at<uchar>(j-roi_y1,i-roi_x1) = 255;
					}
				}
			}	
		}
		cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
		cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2,2));
		cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element2);
		cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);
		cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);
		cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);
		//cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element);
		
		cv::imshow("forground",mask); 
		cv::imshow("forgroundrgb",roi);
		cv::waitKey(10); 

	}
	void draw(cv::Mat parent_img)
	{
		this->parent_img = parent_img;
		if(mask.empty()) return;
		roi.copyTo(parent_img(rec),mask);
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
	bool new_pose, get_bebop_pose, get_tripod_pose, get_image;

	std::vector<Object*> ar_objects;
	
	geometry_msgs::PoseStamped tripodCamPose, last_tripodCamPose, bebopPose, switchingPose;
	
	tf::Quaternion qic;
	tf::Transform tic;
	tf::Transform twi;
    std::vector<Obstacle_subscriber *> obstacle_sub;
	std::vector<std::string> obstacleNames;
	Generic1D *bebop_path;
	Ellipse *ra1,*ra2,*ra3;
	Ellipse *rbar1,*rbar2,*rbar3;
	Ellipse *rd1,*rd2,*rd3;
	Ellipse *bebop_r;
	
	Sphere *obstacle1;
	Sphere *obstacle2;
	Sphere *obstacle3;
	Cylinder *goal;
	Agent *bebop;
	Backdrop *backdrop;
	Backdrop *backdrop2;
	Backdrop *floor;
	
	/*
	Ellipse *test_ellipse;
	Rectangle *test_rectangle;
	EllipseFilled *test_ellipsefilled;
	RectangleFilled *test_rectanglefilled;
	Cylinder *test_cylinder;
	Rectangular *test_rectangular;
	Sphere *test_sphere;
	*/
	
	cv::Mat bg;
	cv::Mat base;
	cv::Mat raw;
	cv::Mat source;
	sensor_msgs::CameraInfo cam_info;
	cv_bridge::CvImagePtr cv_ptr;
	
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
		tripodCamPose.pose.orientation.x = 0;
		tripodCamPose.pose.orientation.y = 0;
		tripodCamPose.pose.orientation.z = 0;
		tripodCamPose.pose.orientation.w = 1;
		
		bebopPose.pose.position.x = 0;
		bebopPose.pose.position.y = 0;
		bebopPose.pose.position.z = 0;
		bebopPose.pose.orientation.x = 0;
		bebopPose.pose.orientation.y = 0;
		bebopPose.pose.orientation.z = 0;
		bebopPose.pose.orientation.w = 1;
		
		qic = tf::Quaternion(-0.511255, 0.491203, -0.503668, 0.493576);
		tic = tf::Transform(qic,tf::Vector3(0.0960949, -0.0554466, -0.0207288));
		
		
		Obstacle_subscriber * obj;
		
		obj = new Obstacle_subscriber(obstacleNames.at(0),false);
		obstacle_sub.push_back(obj);
		
		obj = new Obstacle_subscriber(obstacleNames.at(1),false);
		obstacle_sub.push_back(obj);
		
		obj = new Obstacle_subscriber(obstacleNames.at(2),false);
		obstacle_sub.push_back(obj);
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
		return;
	}
	void imageCB(const sensor_msgs::ImageConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr& camInfoMsg)
	{			
		
		cam_info = *camInfoMsg;
		cv_ptr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8); //convert sensor msg to opencv-compatible cvimage
		raw = cv_ptr->image;


		// Set world to camera transformation
		tf::Quaternion qcw(tripodCamPose.pose.orientation.x,tripodCamPose.pose.orientation.y,tripodCamPose.pose.orientation.z,tripodCamPose.pose.orientation.w);
		tf::Transform tcw(qcw,tf::Vector3(tripodCamPose.pose.position.x,tripodCamPose.pose.position.y,tripodCamPose.pose.position.z));
	
		//tf::Transform tiw = tcw*tic; // Transforms point from image coordinate to world coordinate.
		twi = tic.inverse()*tcw.inverse(); // Transforms point from world cooridnate to image coordinate.

		if(!get_image) {
						
			/***** Examples *****/
			/*
				bebop_path = new Generic1D(&twi,cam_info,2,50);
				ar_objects.push_back(bebop_path);
				
				test_ellipse = new Ellipse(&twi,cam_info,0.2,0.2,0.0,cv::Scalar(0,0,255),0.3,2,100);
				ar_objects.push_back(test_ellipse);

				test_rectangle = new Rectangle(&twi,cam_info,0.8,0.5,0.0,cv::Scalar(0,255,0),0.3,2);
				ar_objects.push_back(test_rectangle);
				
				test_ellipsefilled = new EllipseFilled(&twi,cam_info,0.8,1.3,0.0,cv::Scalar(255,0,0),0.3,100,0);
				ar_objects.push_back(test_ellipsefilled);

				test_rectanglefilled = new RectangleFilled(&twi,cam_info,2.0,1.0,0.0,cv::Scalar(255,255,0),0.3,0);
				ar_objects.push_back(test_rectanglefilled);
					
				test_cylinder = new Cylinder(&twi, cam_info, 0.8,0.8,1.2,cv::Scalar(0,255,255),0.3,100,25,1,1,0);
				ar_objects.push_back(test_cylinder);
					 
				test_rectangular = new Rectangular(&twi,cam_info,0.4,1.3,0.7,cv::Scalar(255,0,255),0.5,25,1,1,0);	
				ar_objects.push_back(test_rectangular);
				
				test_sphere = new Sphere(&twi,cam_info,0.5,0.5,0.5,cv::Scalar(255,100,100),0.3,30,8,1);
				ar_objects.push_back(test_sphere);
			*/
			/***** End of exmaples *****/
			
			// Initialize objects
			bebop_path = new Generic1D(&twi,cam_info,2,50);
			
			
			ra1 = new Ellipse(&twi,cam_info,obstacle_sub.at(0)->ra,obstacle_sub.at(0)->ra,0.0,cv::Scalar(0,0,255),0.3,2,50);
			ra2 = new Ellipse(&twi,cam_info,obstacle_sub.at(1)->ra,obstacle_sub.at(1)->ra,0.0,cv::Scalar(0,0,255),0.3,2,50);
			ra3 = new Ellipse(&twi,cam_info,obstacle_sub.at(2)->ra,obstacle_sub.at(2)->ra,0.0,cv::Scalar(0,0,255),0.3,2,50);
			rbar1 = new Ellipse(&twi,cam_info,obstacle_sub.at(0)->rbar,obstacle_sub.at(0)->rbar,0.0,cv::Scalar(0,255,255),0.3,2,50);
			rbar2 = new Ellipse(&twi,cam_info,obstacle_sub.at(1)->rbar,obstacle_sub.at(1)->rbar,0.0,cv::Scalar(0,255,255),0.3,2,50);
			rbar3 = new Ellipse(&twi,cam_info,obstacle_sub.at(2)->rbar,obstacle_sub.at(2)->rbar,0.0,cv::Scalar(0,255,255),0.3,2,50);
			rd1 = new Ellipse(&twi,cam_info,obstacle_sub.at(0)->rd,obstacle_sub.at(0)->rd,0.0,cv::Scalar(0,255,0),0.3,2,50);
			rd2 = new Ellipse(&twi,cam_info,obstacle_sub.at(1)->rd,obstacle_sub.at(1)->rd,0.0,cv::Scalar(0,255,0),0.3,2,50);
			rd3 = new Ellipse(&twi,cam_info,obstacle_sub.at(2)->rd,obstacle_sub.at(2)->rd,0.0,cv::Scalar(0,255,0),0.3,2,50);
			obstacle1 = new Sphere(&twi,cam_info,0.2,0.2,0.2,cv::Scalar(0,0,255),0.3,30,8,1);
			obstacle2 = new Sphere(&twi,cam_info,0.2,0.2,0.2,cv::Scalar(0,0,255),0.3,30,8,1);
			obstacle3 = new Sphere(&twi,cam_info,0.2,0.2,0.2,cv::Scalar(0,0,255),0.3,30,8,1);
			
			
			bebop_r = new Ellipse(&twi,cam_info,0.2,0.2,0.0,cv::Scalar(255,0,0),0.3,2,50);
			

			
			goal = new Cylinder(&twi,cam_info,0.2,0.2,0.5,cv::Scalar(0,255,255),0.7,30,25,0,0,0);

			bebop = new Agent(&twi,cam_info,0.4,0.4,0.2);

			ra1->setName("Obstacle 1 ra");
			ra2->setName("Obstacle 2 ra");
			ra3->setName("Obstacle 3 ra");
			rbar1->setName("Obstacle 1 rbar");
			rbar2->setName("Obstacle 2 rbar");
			rbar3->setName("Obstacle 3 rbar");
			rd1->setName("Obstacle 1 rd");
			rd2->setName("Obstacle 2 rd");
			rd3->setName("Obstacle 3 rd");
			obstacle1->setName("Obstacle1");
			obstacle2->setName("Obstacle2");
			obstacle3->setName("Obstacle3");
			
			bebop_r->setName("bebop r");
			bebop_path->setName("bebop path");
			goal->setName("Goal");
			bebop->setName("bebop");


			ar_objects.push_back(bebop_path);
			ar_objects.push_back(bebop_r);
			
			ar_objects.push_back(ra1);		
			ar_objects.push_back(ra2);		
			ar_objects.push_back(ra3);		
			ar_objects.push_back(rbar1);		
			ar_objects.push_back(rbar2);		
			ar_objects.push_back(rbar3);		
			ar_objects.push_back(rd1);		
			ar_objects.push_back(rd2);		
			ar_objects.push_back(rd3);		
			ar_objects.push_back(obstacle1);		
			ar_objects.push_back(obstacle2);		
			ar_objects.push_back(obstacle3);	
			
			ar_objects.push_back(goal);
			
			ar_objects.push_back(bebop);		

			goal->update(tf::Vector3(2.5,0,0));
		
			base = raw.clone(); // Copy first frame for base image.	
			source = raw.clone();
			double backdrop_height = 2.0;
			
			backdrop = new Backdrop(&twi,cam_info,10.0,backdrop_height,cv::Scalar(100,0,0),1,0);
			backdrop2 = new Backdrop(&twi,cam_info,6,backdrop_height,cv::Scalar(150,0,0),1,0);
			floor = new Backdrop(&twi,cam_info,8.6,backdrop_height,cv::Scalar(150,0,0),1,0);

			
			tf::Quaternion q;
			tf::Transform tf;
			
			q.setRotation(tf::Vector3(0,0,1),M_PI*0.5);
			tf.setOrigin(tf::Vector3(-4.3,0,backdrop_height/2));
			tf.setRotation(q);
		
			backdrop->update(tf);
			backdrop->draw(base);
			
			q.setRotation(tf::Vector3(0,0,1),M_PI*0.0);
			tf.setOrigin(tf::Vector3(-1.3,-2.2,backdrop_height/2));
			tf.setRotation(q);
		
			backdrop2->update(tf);
			backdrop2->draw(base);	
			
			
			q.setRotation(tf::Vector3(0,0,1),M_PI*0.0);
			tf.setOrigin(tf::Vector3(0.0,2.5,backdrop_height/2));
			tf.setRotation(q);
						
			floor->update(tf);
			floor->draw(base);
		
			
			get_image = true;	
			
			
			std::cout << "Background captured. Move drone into frame. Enter any key to continue: " << std::endl;
			char wait_char;			
			std::cin >> wait_char;
			return;
		}
		
		bg = base.clone(); // Copy a background image to draw on.
		
		// Update objects
		
		
		tf::Vector3 bebop_position3d(bebopPose.pose.position.x,bebopPose.pose.position.y,bebopPose.pose.position.z);
		tf::Quaternion bebop_quaternion(bebopPose.pose.orientation.x,bebopPose.pose.orientation.y,bebopPose.pose.orientation.z,bebopPose.pose.orientation.w);
		tf::Transform bebop_tf;
		bebop_tf.setOrigin(bebop_position3d);
		bebop_tf.setRotation(bebop_quaternion);
		tf::Vector3 bebop_position2d(bebopPose.pose.position.x,bebopPose.pose.position.y,0);
		
		


		bebop->update(bebop_position3d);
		bebop->find(raw,source,bebop_tf);
		
		
		bebop_path->addPt(bebop_position2d,0.8,cv::Scalar(255,0,0));
		bebop_path->update(bebop_position2d);
		bebop_r->update(bebop_position2d);		
		
			
		ra1->update(obstacle_sub.at(0)->position2d);
		ra2->update(obstacle_sub.at(1)->position2d);
		ra3->update(obstacle_sub.at(2)->position2d);
		rbar1->update(obstacle_sub.at(0)->position2d);
		rbar2->update(obstacle_sub.at(1)->position2d);
		rbar3->update(obstacle_sub.at(2)->position2d);
		rd1->update(obstacle_sub.at(0)->position2d);
		rd2->update(obstacle_sub.at(1)->position2d);
		rd3->update(obstacle_sub.at(2)->position2d);
		obstacle1->update(obstacle_sub.at(0)->position3d);
		obstacle2->update(obstacle_sub.at(1)->position3d);
		obstacle3->update(obstacle_sub.at(2)->position3d);

		
		/*
		tf::Vector3 test_center = tf::Vector3(0.5,0.3,0);
		test_rectangle->update(test_center);
		test_ellipse->update(test_center);
		test_ellipsefilled->update(test_center);
		test_rectanglefilled->update(test_center);
		test_cylinder->update(test_center);
		test_rectangular->update(test_center);
		test_sphere->update(test_center);
		*/ 
		

		// Sort objects based on distance to camera (to determine foreground).
		
		for(int i = 0; i < ar_objects.size(); i++)
		{
			for(int j = i; j < ar_objects.size(); j++)
			{
				if(ar_objects.at(i)->center_in_image.length() < ar_objects.at(j)->center_in_image.length())
				{
					std::swap(ar_objects[i],ar_objects[j]);
				}
			}
		}

		// Draw objects
		
		for(int i = 0 ; i < ar_objects.size() ; i++)
		{
			ar_objects.at(i)->draw(bg);

		}
	
		cv_ptr->image = bg;
		imgPub.publish(cv_ptr->toImageMsg()); //publish sensor msg
		return;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "AR_Projector_Node");
	
	AR_Projector ar_project;
	ros::spin();
	return 0;
	
}
	
