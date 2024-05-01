/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <bnerf_msgs/VoxelGridFilterInfo.h>
#include <voxel_grid_filter/voxel_grid_filter.h>
#include <glog/logging.h>
#include <chrono>


namespace bnerf
{
    bool VoxelGridFilter::IsPointInvalid(const pcl::PointXYZI & pt)
    {
        const float square_dist = pt.getVector3fMap().squaredNorm();
        return square_dist < square_min_range_ || square_dist > square_max_range_;
    }


    VoxelGridFilter::VoxelGridFilter(ros::NodeHandle & nh)
    {
        LOG_ASSERT(nh.getParam("leaf_size", leaf_size_));
        nh.param<float>("min_range", square_min_range_, 1e-3);
        nh.param<float>("max_range", square_max_range_, 200);
        
        square_min_range_ *= square_min_range_;
        square_max_range_ *= square_max_range_;

        scan_pub_ = nh.advertise<sensor_msgs::PointCloud2>("filtered_points", 100);
        info_pub_ = nh.advertise<bnerf_msgs::VoxelGridFilterInfo>("info", 1000);
        scan_sub_ = nh.subscribe("raw_scan", 100, &VoxelGridFilter::ScanCallBack, this);
    }


    void VoxelGridFilter::ScanCallBack(const sensor_msgs::PointCloud2 & raw_scan_msg)
    {
        const auto t1 = std::chrono::high_resolution_clock::now();

        pcl::PointCloud<pcl::PointXYZI>::Ptr scan(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(raw_scan_msg, *scan);

        using std::placeholders::_1;
        auto invalid = std::bind(&VoxelGridFilter::IsPointInvalid, this, _1);
        auto iend = remove_if(scan->begin(), scan->end(), invalid);
        scan->erase(iend, scan->end());

        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
        voxel_grid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        voxel_grid.setInputCloud(scan);
        voxel_grid.filter(*scan);

        sensor_msgs::PointCloud2 filtered_msg;
        pcl::toROSMsg(*scan, filtered_msg);
        filtered_msg.header = raw_scan_msg.header;

        scan_pub_.publish(filtered_msg);

        const auto t2 = std::chrono::high_resolution_clock::now();
        const auto nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count();

        bnerf_msgs::VoxelGridFilterInfo info_msg;
        info_msg.header = filtered_msg.header;
        info_msg.exec_time.fromNSec(nsecs);
        info_msg.num_filtered_points = filtered_msg.width;
        info_msg.num_raw_scan_points = raw_scan_msg.width;
        info_pub_.publish(info_msg);
    }
}