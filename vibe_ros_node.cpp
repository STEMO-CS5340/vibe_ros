#include <vibe_ros.hpp>

#include <ros/ros.h>

int main(int argc, char * argv[])
{

    ros::init(argc, argv, "vibe_ros");
	ros::NodeHandle nh;
	ros::NodeHandle local_nh("~");

    std::cout  << "main" << std::endl;

    ViBeManager vm; // DO NOT INCLUDE PARENTHESIS

    std::cout  << "Vibe Starts" << std::endl;

    ros::spin();

    std::cout  << "Main() ends" << std::endl;
}