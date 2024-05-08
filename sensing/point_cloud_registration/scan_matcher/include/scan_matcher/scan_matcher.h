#ifndef __SCAN_MATCHER_H__
#define __SCAN_MATCHER_H__


#include <bnerf_utils/logging/logger.h>
#include <voxelizer/voxelizer.h>
#include <scan_matcher/optim_data.h>
#include <bnerf_utils/bnerf_utils.h>
#include <bnerf_msgs/GraphBinaryEdge.h>


namespace bnerf
{
    class ScanMatcher
    {
        public: 
        ScanMatcher();
        SE3d OptimizeAlignment(CloudXYZ::ConstPtr, const SE3d & init_guess = SE3d());

        void SourceScanCallBack(const CloudXYZ::ConstPtr &);
        void TargetScanCallBack(const CloudXYZ::ConstPtr &);

        private:
        Voxelizer::Ptr GetVoxelizer(CloudXYZ::ConstPtr);
        CloudXYZI GetCombinedScan(const OptimData &) const;
        bnerf_msgs::GraphBinaryEdge GetBinaryEdge(const OptimData &) const;
        

        //registrator stuffs
        private:
        ros::NodeHandle nh_;
        mutex vox_mutex_;
        vector<Voxelizer::Ptr> voxers_;

        //parameter & node handle stuffs
        private:
        bool use_gicp_;
        double epsilon_;
        int max_iters_;
        ros::Duration tgt_timeout_;

        //ros stuff
        ros::Publisher edge_pub_;
        ros::Subscriber src_sub_;
        ros::Subscriber tgt_sub_;

        //visualization stuff
        private:
        unique_ptr<ros::Publisher> viz_pub_;
    };
}

#endif