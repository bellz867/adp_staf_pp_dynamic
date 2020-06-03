#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>

class Synchronizer
{
	ros::NodeHandle nh;
	ros::Publisher syncPub;	
public:
	
	Synchronizer()
	{
		syncPub = nh.advertise<std_msgs::Bool>("/sync",1);
		std_msgs::Bool b;
		b.data = false;
		syncPub.publish(b);
		char wait_char;
		
		std::cout << "Enter any key to START experiement." << std::endl;
		std::cin >> wait_char;
		b.data = true;
		syncPub.publish(b);
		
		std::cout << "Enter any key to END experiement." << std::endl;
		std::cin >> wait_char;
		b.data = true;
		syncPub.publish(b);
	}		
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "bebop_control_node");
	
	Synchronizer synchronizer;
	ros::spin();
	return 0;
	
}
