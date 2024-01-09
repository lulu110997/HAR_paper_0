#include "process_data.h"
#include <ros/console.h>

SkeletonProcessing::SkeletonProcessing(const std::shared_ptr<ros::NodeHandle> &nh)
{
    nh_ = nh;

    // Create subscribers
    hs_left_subs_ = nh_->subscribe("/left_hand_skel_data", 30, &SkeletonProcessing::hs_left_callback, this);
    hs_right_subs_ = nh_->subscribe("/right_hand_skel_data", 30, &SkeletonProcessing::hs_right_callback, this);
    body_subs_ = nh_->subscribe("/nuitrack_skel_data", 30, &SkeletonProcessing::body_callback, this);

    // Create publishers
    hs_left_pub_ = nh_->advertise<geometry_msgs::PoseArray>("hs_left_matched", 15);
    hs_right_pub_ = nh_->advertise<geometry_msgs::PoseArray>("hs_right_matched", 15);
    body_pub_ = nh_->advertise<geometry_msgs::PoseArray>("body_matched", 15);
    pub_thread_ = new std::thread(&SkeletonProcessing::publish_data, this); // Creates and starts the thread

}

SkeletonProcessing::~SkeletonProcessing()
{
    pub_thread_->join();
}

void SkeletonProcessing::hs_left_callback(const geometry_msgs::PoseArrayConstPtr &msg)
{
    std::lock_guard<std::mutex> lck(hs_left_.mtx);
    hs_left_.data_q.push_back(*msg);
}

void SkeletonProcessing::hs_right_callback(const geometry_msgs::PoseArrayConstPtr &msg)
{    
    std::lock_guard<std::mutex> lck(hs_right_.mtx);
    hs_right_.data_q.push_back(*msg);
}

void SkeletonProcessing::body_callback(const geometry_msgs::PoseArrayConstPtr &msg)
{
    std::lock_guard<std::mutex> lck(body_.mtx);
    body_.data_q.push_back(*msg);
}

void SkeletonProcessing::publish_data()
{
    ros::Rate no_data_sleep(60.0);  // Sleep for when no data is received. Corresponds to 60fps of realsense
    ros::Rate rate(30.0);  // Normal rate for publishing. Corresponds to 30fps of Kinect

    // Create variables for storing deque and matching skeleteal measurements
    std::deque<geometry_msgs::PoseArray> hs_left_copy, hs_right_copy, body_copy;
    geometry_msgs::PoseArray hs_left_matching;
    geometry_msgs::PoseArray hs_right_matching;
    geometry_msgs::PoseArray body_matching;

    while (ros::ok())
    {
        {
            // Create locks
            std::lock_guard<std::mutex> hs_left_lck(hs_left_.mtx);
            std::lock_guard<std::mutex> hs_right_lck(hs_right_.mtx);
            std::lock_guard<std::mutex> body_lck(body_.mtx);
            // Obtain skeleton data for processing
            hs_left_.data_q.swap(hs_left_copy);
            hs_right_.data_q.swap(hs_right_copy);
            body_.data_q.swap(body_copy);
            // Clear deque
            hs_left_.data_q.clear();
            hs_right_.data_q.clear();
            body_.data_q.clear();
        }

        if (hs_left_copy.empty() and hs_right_copy.empty() and body_copy.empty())
        {
            // No new d at thiata. Might have new hand skeleton data in next 1/60.0 seconds
            // ROS_INFO("No new data");
            no_data_sleep.sleep();
        }
        else
        {
            // Check if any skeleton data measurement is empty. If not empty, store the latest timestamp
            double hs_left_time = 0.0;
            double hs_right_time = 0.0;
            double body_time = 0.0;
            if (!hs_left_copy.empty())  {hs_left_time = hs_left_copy.back().header.stamp.toSec();}
            if (!hs_right_copy.empty()) {hs_right_time = hs_right_copy.back().header.stamp.toSec();}
            if (!body_copy.empty())     {body_time = body_copy.back().header.stamp.toSec();}
            

            // Find max ts as we want to use the most recent data. Note you the left and right hand skeleton data do not always come together
            // ie; you can have a left skeleton data but no right skeleton data (eg due to occlusion confidence score on one hand is low.
            // Therefore, only one hand skeleton is extracted

            if (hs_left_time >= std::max(hs_right_time, body_time) and !hs_left_copy.empty())
            {
                // The left hand is more recent, find closest corresponding data for right/body within 30ms
                hs_left_matching = hs_left_copy.back();
                find_match(hs_left_time, hs_right_copy, hs_right_matching);
                find_match(hs_left_time, body_copy, body_matching);
                    
                
            }
            else if (hs_right_time >= std::max(hs_left_time, body_time) and !hs_right_copy.empty())
            {
                // The right hand is more recent, find corresponding data for left/body within 30ms
                hs_right_matching = hs_right_copy.back();
                find_match(hs_right_time, hs_left_copy, hs_left_matching);
                find_match(hs_right_time, body_copy, body_matching);


            }
            
            else if (body_time >= std::max(hs_left_time, hs_right_time) and !body_copy.empty())
            {
                // The body skeleton is more recent, find corresponding data for hands skeleton within 30ms
                body_matching = body_copy.back();
                find_match(body_time, hs_left_copy, hs_left_matching);
                find_match(body_time, hs_right_copy, hs_right_matching);
            }
            else
            {
                // ROS_ERROR(std::to_string(hs_left_time).c_str());
                // ROS_ERROR(std::to_string(hs_right_time).c_str());
                // ROS_ERROR(std::to_string(body_time).c_str());
                ROS_ERROR("Can't find most recent data");
                ros::shutdown();
            }

            // Replace the ts of the measurements to now so that they match and then publish
            ros::Time time_now = ros::Time().now();
            hs_left_matching.header.stamp = time_now;
            hs_right_matching.header.stamp = time_now;
            body_matching.header.stamp = time_now;
            hs_left_pub_.publish(hs_left_matching);
            hs_right_pub_.publish(hs_right_matching);
            body_pub_.publish(body_matching);

            // Clear the deque copies
            hs_left_copy.clear(); hs_right_copy.clear(); body_copy.clear();
            rate.sleep();
        }
        
    }
}

void SkeletonProcessing::find_match(const double &ts, std::deque<geometry_msgs::PoseArray> &data2, geometry_msgs::PoseArray &matching_data)
{
    if (!data2.empty())
    {
        double min_t_delta = 1000.0;
        while (!data2.empty())
        {
            double time_delta = fabs(ts - data2.back().header.stamp.toSec());
            // Closest match must be less than or equal to 
            if ((time_delta < min_t_delta) and (time_delta <= 1/30.0))
            {
                // It's a match. Save this msg for publishing
                min_t_delta = time_delta;
                matching_data = data2.back();
            }
            else
            {
                data2.pop_back();
            }
        }
    }
    else {matching_data.poses.clear();} // No new data. Clear corresponding matching variable to ensure any saved data is not published
    
}