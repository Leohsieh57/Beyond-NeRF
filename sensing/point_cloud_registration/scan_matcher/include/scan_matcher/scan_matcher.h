#ifndef __SCAN_MATCHER_H__
#define __SCAN_MATCHER_H__


#include <voxelizer/voxelizer.h>
#include <scan_matcher/optimizer.h>
#include <bnerf_utils/bnerf_utils.h>
#include <transform_finder/transform_finder.h>

namespace bnerf
{
    class ScanMatcher
    {
        public: 
        ScanMatcher();
        void SourceScanCallBack(const sensor_msgs::PointCloud2::ConstPtr &);
        void TargetScanCallBack(const sensor_msgs::PointCloud2::ConstPtr &);

        private:
        Voxelizer::ConstPtr GetVoxelizer(sensor_msgs::PointCloud2::ConstPtr);
        
        //registrator stuffs
        private:
        ros::NodeHandle nh_;
        mutex vox_mutex_;
        vector<Voxelizer::ConstPtr> vox_win_;
        TransformFinder tf_finder_;

        //parameter & node handle stuffs
        private:
        bool verbose_;
        bool use_gicp_;
        double epsilon_;
        int max_iters_;
        ros::Duration vox_win_span_;

        //ros stuff
        ros::Publisher edge_pub_;
        ros::Publisher vox_pub_;
        ros::Publisher reg_pub_;
        ros::Subscriber src_sub_;
        ros::Subscriber tgt_sub_;

        //visualization stuff
        private:
        unique_ptr<ros::Publisher> viz_pub_;
    };
}

#endif