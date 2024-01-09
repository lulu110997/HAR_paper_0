#include <thread>
#include <mutex>
#include <deque>
#include <signal.h>

#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"

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

    /*! @brief
    Publish data at a rate of >=30hz. 30hz if there is consistent measurements. Can be longer otherwise
    */
   void publish_data();

    /*! @brief
    Look for matching data measurements. Ensures the measurements are less than or equal to 1/30 seconds of the most recent measurement.
    If no matching measurement is found, poses in matching data is cleared
    @param ts: most recent ts
    @param data2: the data measurement where we will find a matching data
    @param matching_data: data that matches (or closest to matching) the given timestamp within 30ms
    */
   void find_match(const double &ts, std::deque<geometry_msgs::PoseArray> &data2, geometry_msgs::PoseArray &matching_data);
   
    std::shared_ptr<ros::NodeHandle> nh_;  // Node handle
    ros::Subscriber hs_left_subs_, hs_right_subs_, body_subs_;  // Subscribers
    ros::Publisher hs_left_pub_, hs_right_pub_, body_pub_;  // Publishers

    std::thread *pub_thread_;  // Main thread of execution for publishing data    

    struct SkelData  // Thread safe data buffer
    {
        std::deque<geometry_msgs::PoseArray> data_q;
        std::mutex mtx;
    } hs_left_, hs_right_, body_;

};