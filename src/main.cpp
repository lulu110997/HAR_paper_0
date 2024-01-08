#include <publish_data.h>
#include "ros/ros.h"

int main(int argc, char **argv)
{
    // Initialise the node
    ros::init(argc, argv, "match_timstamps");
    ros::NodeHandle pn("~");

    // Start the control loop
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle);
    std::shared_ptr<SkeletonProcessing> skel_proc(new SkeletonProcessing(nh));
    std::shared_ptr<PublishSkeleton> pub_skel(new PublishSkeleton(nh, skel_proc));
 
    ros::spin();

    // Shutdown
    nh->shutdown();
    ros::shutdown();
    std::cout << "ROS has shutdown" << std::endl;

    return 0;
}