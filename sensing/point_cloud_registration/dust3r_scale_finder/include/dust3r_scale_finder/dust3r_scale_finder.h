#ifndef __BNERF_DUST3R_RESCALER_H__
#define __BNERF_DUST3R_RESCALER_H__


#include <bnerf_utils/typedef.h>
#include <voxelizer/voxelizer.h>
#include <bnerf_msgs/ScanMatchingInfo.h>
#include <bnerf_msgs/ScanMatchingFactor.h>


namespace bnerf
{
    class DUSt3RScaleFinder
    {
        public: 
        DUSt3RScaleFinder(ros::NodeHandle &);
        void SourceScanCallBack(const CloudXYZ::ConstPtr &);
        void TargetScanCallBack(const CloudXYZ::ConstPtr &);

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