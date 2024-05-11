#ifndef __SCAN_MATCHER_H__
#define __SCAN_MATCHER_H__


#include <bnerf_utils/logging/logger.h>
#include <voxelizer/voxelizer.h>
#include <scan_matcher/optimizer.h>
#include <bnerf_utils/bnerf_utils.h>


namespace bnerf
{
    class ScanMatcher
    {
        public: 
        ScanMatcher();
        void SourceScanCallBack(const CloudXYZ::ConstPtr &);
        void TargetScanCallBack(const CloudXYZ::ConstPtr &);

        private:
        Voxelizer::ConstPtr GetVoxelizer(CloudXYZ::ConstPtr);

        //registrator stuffs
        private:
        ros::NodeHandle nh_;
        mutex vox_mutex_;
        vector<Voxelizer::ConstPtr> voxers_;

        //parameter & node handle stuffs
        private:
        bool verbose_;
        bool use_gicp_;
        double epsilon_;
        int max_iters_;
        ros::Duration tgt_timeout_;

        //ros stuff
        ros::Publisher edge_pub_;
        ros::Publisher info_pub_;
        ros::Subscriber src_sub_;
        ros::Subscriber tgt_sub_;

        //visualization stuff
        private:
        unique_ptr<ros::Publisher> viz_pub_;
    };
}

#endif