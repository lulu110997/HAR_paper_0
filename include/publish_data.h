#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>
#include <atomic>
#include <deque>
#include <signal.h>
#include "process_data.h"

#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class PublishSkeleton
{
    public:
    /*! @brief PublishSkeleton constructor.
    *
    * Will take the node handle and initialise the callbacks and internal variables
    *
    */

   PublishSkeleton(const std::shared_ptr<ros::NodeHandle> &nh, const std::shared_ptr<SkeletonProcessing> &skel_proc);

    /*! @brief Detection destructor.
    *
    *  Will tear down the object
    */
   ~PublishSkeleton();

    private:
    /*! @brief
    Publish data at a consistent rate of 30hz
    */
   void publish_data();

    /*! @brief
    Look for matching data measurements. Ensures the measurements are within 30 seconds of the most recent measurement.
    If no matching measurement is found, an empty msg is returned
    @param ts: most recent ts
    @param data2: the data measurement where we will find a matching data
    @param matching_data: data that matches (or closest to matching) the given timestamp within 30ms
    */
   void find_match(const float &ts, std::deque<geometry_msgs::PoseArray> &data2, geometry_msgs::PoseArray &matching_data);

    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<SkeletonProcessing> skel_proc_;
    ros::Publisher hs_left_pub_, hs_right_pub_, body_pub_;
    std::thread *pub_thread_;
    std::deque<geometry_msgs::PoseArray> hs_left_, hs_right_, body_;
};