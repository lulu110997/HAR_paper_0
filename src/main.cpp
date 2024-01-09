#include <process_data.h>

int main(int argc, char **argv)
{
    // Initialise the node
    ros::init(argc, argv, "match_timstamps");
    ros::NodeHandle pn("~");

    // Start the control loop
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle);
    SkeletonProcessing skel_proc(nh);
    // - shared ptr: var exists even when one class exits.
    // - passing by reference is the same as passing as a normal pointer. If that pointer gets destroyed
    // in one class, the other class using that pointer will segfault
    // - Overhead of communicating between nodes is small if messages is small. Otherwise, use
    // nodelet for massive data types like images and laser scans
    // - https://en.cppreference.com/w/cpp/algorithm/copy std copy for copying containers
 
    ros::spin();

    // Shutdown
    nh->shutdown();
    ros::shutdown();
    std::cout << "ROS has shutdown" << std::endl;

    return 0;
}