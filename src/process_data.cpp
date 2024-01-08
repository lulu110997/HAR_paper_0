#include "process_data.h"

SkeletonProcessing::SkeletonProcessing(const std::shared_ptr<ros::NodeHandle> &nh)
{
    nh_ = nh;

    // Create subscribers
    hs_left_subs_ = nh_->subscribe("left_hand_skel_data", 15, &SkeletonProcessing::hs_left_callback, this);
    hs_right_subs_ = nh_->subscribe("right_hand_skel_data", 15, &SkeletonProcessing::hs_right_callback, this);
    body_subs_ = nh_->subscribe("nuitrack_skel_data", 15, &SkeletonProcessing::body_callback, this);

}

SkeletonProcessing::~SkeletonProcessing(){}

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

void SkeletonProcessing::get_hs_left(std::deque<geometry_msgs::PoseArray> &hs_left_q)
{
    std::lock_guard<std::mutex> lck(hs_left_.mtx);
    hs_left_q = hs_left_.data_q;
    hs_left_.data_q.swap(empty_q_);
}


void SkeletonProcessing::get_hs_right(std::deque<geometry_msgs::PoseArray> &hs_right_q)
{
    std::lock_guard<std::mutex> lck(hs_right_.mtx);
    hs_right_q = hs_right_.data_q;
    hs_right_.data_q.swap(empty_q_);
}


void SkeletonProcessing::get_body(std::deque<geometry_msgs::PoseArray> &body_q)
{
    std::lock_guard<std::mutex> lck(body_.mtx);
    body_q = body_.data_q;
    body_.data_q.swap(empty_q_);
}