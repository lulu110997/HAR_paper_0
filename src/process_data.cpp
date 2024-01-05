#include "process_data.h"

void CheckTwoRecs(std::queue<geometry_msgs::PoseArray> &data1_q,
                  std::queue<geometry_msgs::PoseArray> &data2_q)
{
}

void CheckThreeRecs(std::queue<geometry_msgs::PoseArray> &data1_q,
                    std::queue<geometry_msgs::PoseArray> &data2_q,
                    std::queue<geometry_msgs::PoseArray> &data3_q)
{
}

SkeletonProcessing::SkeletonProcessing(const std::shared_ptr<ros::NodeHandle> &nh)
{
    nh_ = nh;

    // Create subscribers
    hs_left_subs_ = nh_->subscribe("left_hand_skel_data", 30, &SkeletonProcessing::hs_left_callback, this);
    hs_right_subs_ = nh_->subscribe("right_hand_skel_data", 30, &SkeletonProcessing::hs_right_callback, this);
    body_subs_ = nh_->subscribe("nuitrack_skel_data", 30, &SkeletonProcessing::body_callback, this);

    // Create publishers and start thread for publishing
    hs_left_pub_ = nh_->advertise<geometry_msgs::PoseArray>("hs_left_matched", 30);
    hs_right_pub_ = nh_->advertise<geometry_msgs::PoseArray>("hs_right_matched", 30);
    body_pub_ = nh_->advertise<geometry_msgs::PoseArray>("body_matched", 30);
    pub_thread_ = new std::thread(&SkeletonProcessing::publish_data, this); // Creates and starts the thread
    pub_mtx_.lock();

}

SkeletonProcessing::~SkeletonProcessing()
{
    received_one_.store(true);
    pub_mtx_.unlock();
    cv_.notify_all();
    pub_thread_->join();
}

void SkeletonProcessing::hs_left_callback(const geometry_msgs::PoseArrayConstPtr &msg)
{
    {
        std::lock_guard<std::mutex> lck(hs_left_.mtx);
        hs_left_.data_q.push(*msg);
    }
    if (!received_one_.load()) {received_one_.store(true); pub_mtx_.unlock(); cv_.notify_all();} 
}

void SkeletonProcessing::hs_right_callback(const geometry_msgs::PoseArrayConstPtr &msg)
{    
    {
        std::lock_guard<std::mutex> lck(hs_right_.mtx);
        hs_right_.data_q.push(*msg);
    }
    if (!received_one_.load()) {received_one_.store(true); pub_mtx_.unlock(); cv_.notify_all();}
}

void SkeletonProcessing::body_callback(const geometry_msgs::PoseArrayConstPtr &msg)
{
    {
        std::lock_guard<std::mutex> lck(body_.mtx);
        body_.data_q.push(*msg);
    }
    if (!received_one_.load()) {received_one_.store(true); pub_mtx_.unlock(); cv_.notify_all();}

}

void SkeletonProcessing::publish_data()
{
    ros::Rate rate(30);
    bool hl_empty = false;
    bool hr_empty = false;
    bool body_empty = false;
    std::queue<geometry_msgs::PoseArray> empty_q;

    while (ros::ok()){
        std::unique_lock<std::mutex> lk(pub_mtx_);
        cv_.wait(lk, [&]{return received_one_.load();});

        {
            std::unique_lock<std::mutex> lck_l(hs_left_.mtx);
            std::unique_lock<std::mutex> lck_r(hs_right_.mtx);
            std::unique_lock<std::mutex> lck_b(body_.mtx);
            if (hs_left_.data_q.empty()) {lck_l.unlock(); hl_empty = true;}
            if (hs_right_.data_q.empty()) {lck_r.unlock(); hr_empty = true;}
            if (body_.data_q.empty()) {lck_b.unlock(); hr_empty = true;}
            
            float l_ts = hs_left_.data_q.front().header.stamp.toSec();
            float r_ts = hs_right_.data_q.front().header.stamp.toSec();
            float b_ts = body_.data_q.front().header.stamp.toSec();
            geometry_msgs::PoseArray asdf;
            geometry_msgs::Pose assss;
            assss.position.x = 0;
            assss.position.y = 1;
            assss.position.z = 0;
            asdf.poses.push_back(assss);
            hs_left_pub_.publish(asdf);

            if (hl_empty and hr_empty and body_empty) {} // Nothing have recordings

            else if (!(hl_empty and hr_empty and body_empty))
            {
                // Everything have recordings
                CheckThreeRecs(hs_left_.data_q, hs_right_.data_q, body_.data_q);
            }
            else if (!hl_empty and !hr_empty)
            {
                // Only the hand skeletons have recordings
                CheckTwoRecs(hs_right_.data_q, hs_left_.data_q);
            }
            
            else if (!hr_empty and !body_empty)
            {
                // Only the right hand and body have recordings
                CheckTwoRecs(hs_right_.data_q, body_.data_q);                    
            }
            else if (!hl_empty and !body_empty)
            {
                // Only the left hand and body have recordings
                CheckTwoRecs(hs_left_.data_q, body_.data_q);
            }
            else if (!hl_empty)
            {
                // Only the left hand have recordings, publish it
            }
            else if (!hl_empty)
            {
                // Only the left hand have recordings, publish it
            }
            else if (!hl_empty)
            {
                // Only the left hand have recordings, publish it
            }
            else
            {
                std::cout << "Unaccounted term" << std::endl;
                raise(0);
            }
            
            std::swap(hs_left_.data_q, empty_q);
            std::swap(hs_right_.data_q, empty_q);
            std::swap(body_.data_q, empty_q);


        }
        
        hl_empty = false;
        hr_empty = false;
        body_empty = false;
        received_one_.store(false);
        rate.sleep();
    }
}