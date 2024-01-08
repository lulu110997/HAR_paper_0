#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>
#include <atomic>
#include <deque>
#include <signal.h>


#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class SkeletonProcessing
{
    public:
    /*! @brief SkeletonProcessing constructor.
    *
    * Will take the node handle and initialise the callbacks and internal variables
    * 
    * Wait until we receive a message. Once we receive a message, we wait 1/25 ms (corresponds to 30hz + some delay for processing) to receive another message.
    * If no message is received within this time period, we publish this message and an empty field for the message that did not come.
    */
   SkeletonProcessing(const std::shared_ptr<ros::NodeHandle> &nh);

    /*! @brief SkeletonProcessing destructor.
    *
    *  Will tear down the object
    */
   ~SkeletonProcessing();

    /*! @brief getter for hand skeleton (left).
    *
    *  @param geometry_msgs::PoseArrayConstPtr - The PoseArray message to store the data in
    */
   void get_hs_left(std::deque<geometry_msgs::PoseArray> &hs_left_q);

    /*! @brief getter for hand skeleton (right).
    *
    *  @param geometry_msgs::PoseArrayConstPtr - The PoseArray message to store the data in
    */
   void get_hs_right(std::deque<geometry_msgs::PoseArray> &hs_right_q);

    /*! @brief getter for body skeleton.
    *
    *  @param geometry_msgs::PoseArrayConstPtr - The PoseArray message to store the data in
    */
   void get_body(std::deque<geometry_msgs::PoseArray> &body_q);

    private:
    /*! @brief Callback for receiving left hand data
    *
    *  @param geometry_msgs::PoseArrayConstPtr - The PoseArray message
    */
   void hs_left_callback(const geometry_msgs::PoseArrayConstPtr& msg);

    /*! @brief Callback for receiving right hand data
    *
    *  @param geometry_msgs::PoseArrayConstPtr - The PoseArray message
    */
   void hs_right_callback(const geometry_msgs::PoseArrayConstPtr& msg);

    /*! @brief Callback for receiving body data
    *
    *  @param geometry_msgs::PoseArrayConstPtr - The PoseArray message
    */
   void body_callback(const geometry_msgs::PoseArrayConstPtr& msg);
   
    std::shared_ptr<ros::NodeHandle> nh_;
    ros::Subscriber hs_left_subs_, hs_right_subs_, body_subs_;
    
    std::deque<geometry_msgs::PoseArray> empty_q_;

    struct SkelData
    {
        std::deque<geometry_msgs::PoseArray> data_q;
        std::mutex mtx;
    } hs_left_, hs_right_, body_;


};