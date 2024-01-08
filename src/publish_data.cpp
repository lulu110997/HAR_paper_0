#include "publish_data.h"

PublishSkeleton::PublishSkeleton(const std::shared_ptr<ros::NodeHandle> &nh,
const std::shared_ptr<SkeletonProcessing> &skel_proc)
{
    nh_ = nh;
    skel_proc_ = skel_proc;

    // Create publishers and start thread for publishing
    hs_left_pub_ = nh_->advertise<geometry_msgs::PoseArray>("hs_left_matched", 30);
    hs_right_pub_ = nh_->advertise<geometry_msgs::PoseArray>("hs_right_matched", 30);
    body_pub_ = nh_->advertise<geometry_msgs::PoseArray>("body_matched", 30);
    pub_thread_ = new std::thread(&PublishSkeleton::publish_data, this); // Creates and starts the thread
 
}

PublishSkeleton::~PublishSkeleton()
{
    pub_thread_->join();
}

void PublishSkeleton::publish_data()
{
    ros::Rate rate(30);

    while (ros::ok())
    {
        skel_proc_->get_hs_left(hs_left_);
        skel_proc_->get_hs_right(hs_right_);
        skel_proc_->get_body(body_);

        float hs_left_time = 1000.0;
        float hs_right_time = 1000.0;
        float body_time = 1000.0;

        if (!hs_left_.empty())
        {
            float hs_left_time = hs_left_.back().header.stamp.toSec();
        }
        if (!hs_right_.empty())
        {
            float hs_right_time = hs_right_.back().header.stamp.toSec();
        }
        if (!body_.empty())
        {
            float body_time = body_.back().header.stamp.toSec();
        }
        

        // std::vector<std::pair<std::deque<geometry_msgs::PoseArray>, unsigned int>> vp;
        // vp.push_back(std::make_pair(hs_left_, hs_left_time));
        // vp.push_back(std::make_pair(hs_right_, hs_right_time));
        // vp.push_back(std::make_pair(body_, body_time));
        // std::sort(vp.begin(), vp.end(), [](std::pair<std::deque<geometry_msgs::PoseArray>, unsigned int> a,
        //                                    std::pair<std::deque<geometry_msgs::PoseArray>, unsigned int> b)
        //                                     {
        //                                         return a.second > b.second;
        //                                     });

        // Find max ts as we want to use the most recent data. Note you the left and right hand skeleton data do not always come together
        // ie; you can have a left skeleton data but no right skeleton data (eg due to occlusion confidence score on one hand is low.
        // Therefore, only one hand skeleton is extracted
        geometry_msgs::PoseArray hs_left_matching;
        geometry_msgs::PoseArray hs_right_matching;
        geometry_msgs::PoseArray body_matching;

        if (hs_left_time > std::min(hs_right_time, body_time) and !hs_left_.empty())
        {
            // The hand skeleton is more recent, find closest corresponding data for body within 30ms
            hs_left_matching = hs_left_.back();
            find_match(hs_left_time, hs_right_, hs_right_matching);
            find_match(hs_left_time, body_, body_matching);
                
            
        }
        else if (hs_right_time > std::min(hs_left_time, body_time) and !hs_right_.empty())
        {
            // The hand skeleton is more recent, find corresponding data for body within 30ms
            hs_right_matching = hs_right_.back();
            find_match(hs_right_time, hs_left_, hs_left_matching);
            find_match(hs_right_time, body_, body_matching);


        }
        
        else if (body_time > std::min(hs_left_time, hs_right_time) and !body_.empty())
        {
            // The body skeleton is more recent, find corresponding data for hand skeleton within 30ms
            body_matching = body_.back();
            find_match(body_time, hs_left_, hs_left_matching);
            find_match(body_time, hs_right_, hs_right_matching);
        }

        ros::Time time_now = ros::Time().now();
        hs_left_matching.header.stamp = time_now;
        hs_right_matching.header.stamp = time_now;
        body_matching.header.stamp = time_now;
        hs_left_pub_.publish(hs_left_matching);
        hs_right_pub_.publish(hs_right_matching);
        body_pub_.publish(body_matching);
        rate.sleep();
    }
}

void PublishSkeleton::find_match(const float &ts, std::deque<geometry_msgs::PoseArray> &data2, geometry_msgs::PoseArray &matching_data)
{
    if (!data2.empty())
    {
        float min_t_delta = 1000.0;
        while (!data2.empty())
        {
            float time_delta = fabs(ts - data2.back().header.stamp.toSec());
            if ((time_delta < min_t_delta) and (time_delta < 1/45.0))
            {
                // It's a match
                min_t_delta = time_delta;
                matching_data = data2.back();
            }
            else
            {
                data2.pop_back();
            }
        }
    }
}