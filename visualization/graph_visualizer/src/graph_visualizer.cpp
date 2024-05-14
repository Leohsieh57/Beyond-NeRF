#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <graph_visualizer/graph_visualizer.h>
#include <glog/logging.h>
#include <bnerf_utils/bnerf_utils.h>
#include <pcl/common/transforms.h>

namespace bnerf
{
    GraphVisualizer::GraphVisualizer(ros::NodeHandle & nh)
        : scan_pub_(nh.advertise<CloudXYZ>("combined_map", 128))
    {
        stat_sub_ = nh.subscribe("graph_status", 16, &GraphVisualizer::GraphStatusCallBack, this);
        scan_sub_ = nh.subscribe("input_scan", 16, &GraphVisualizer::ScanCallBack, this);
    }


    void GraphVisualizer::GraphStatusCallBack(
        const bnerf_msgs::GraphIterationStatus & msg)
    {
        LOG_ASSERT(!msg.graph_stamps.empty());
        LOG_ASSERT(msg.graph_stamps.size() == msg.graph_states.size());

        auto min_stamp = *min_element(msg.graph_stamps.begin(), msg.graph_stamps.end());

        vector<sensor_msgs::PointCloud2::ConstPtr> clouds(msg.graph_stamps.size());
        {
            lock_guard<mutex> lock(scan_mutex_);
            auto mit = scan_msgs_.find(min_stamp);
            if (mit != scan_msgs_.end())
                scan_msgs_.erase(scan_msgs_.begin(), mit);

            for (size_t i = 0; i < clouds.size(); i++)
            {
                const auto &t = msg.graph_stamps.at(i);
                const auto mit = scan_msgs_.find(t);
                if (mit != scan_msgs_.end())
                    clouds[i] = mit->second;
            }
        }

        if (all_of(clouds.begin(), clouds.end(), [](auto s) {return !s; }))
            return;

        CloudXYZ combined;
        for (size_t i = 0; i < clouds.size(); i++)
        {
            if (!clouds[i])
                continue;

            auto trans = convert<SE3d>(msg.graph_states[i]).inverse();
            CloudXYZ submap;
            pcl::fromROSMsg(*clouds[i], submap);
            pcl::transformPointCloud(submap, submap, trans.matrix().cast<float>());
            combined += submap;
        }

        if (combined.empty())
            return;

        combined.header.frame_id = "world";
        combined.header.stamp = pcl_conversions::toPCL(ros::Time::now());
        scan_pub_.publish(combined);
    }


    void GraphVisualizer::ScanCallBack(const sensor_msgs::PointCloud2::ConstPtr & msg)
    {
        const auto &t = msg->header.stamp;
        lock_guard<mutex> lock(scan_mutex_);
        scan_msgs_.emplace_hint(scan_msgs_.end(), t, msg);
        while (scan_msgs_.size() > 5000)
            scan_msgs_.erase(scan_msgs_.begin());
    }
}