#include <process_data.h>
#include "ros/ros.h"

int main(int argc, char **argv)
{
    // Initialise the node
    ros::init(argc, argv, "match_timstamps");
    ros::NodeHandle pn("~");

    // Start the control loop
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle);
    SkeletonProcessing hithere(nh);
 
    ros::spin();

    // Shutdown
    nh->shutdown();
    ros::shutdown();
    std::cout << "ROS has shutdown" << std::endl;

    return 0;
}