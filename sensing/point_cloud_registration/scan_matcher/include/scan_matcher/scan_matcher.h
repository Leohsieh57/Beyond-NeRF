#ifndef __SCAN_MATCHER_H__
#define __SCAN_MATCHER_H__


#include <bnerf_utils/logging/logger.h>
#include <voxelizer/voxelizer.h>
#include <scan_matcher/data.h>


namespace bnerf
{
    class ScanMatcher
    {
        public: 
        ScanMatcher(ros::NodeHandle &);
        SE3d OptimizeAlignment(CloudXYZ::ConstPtr, const SE3d & init_guess = SE3d());

        void SourceScanCallBack(const CloudXYZ::ConstPtr &);
        void TargetScanCallBack(const CloudXYZ::ConstPtr &);

        private:
        Voxelizer::Ptr GetVoxelizer();
        
        //registrator stuffs
        private:
        mutex vox_mutex_;
        Voxelizer::Ptr voxer_;

        //parameter & node handle stuffs
        private:
        int threads_;
        double epsilon_;
        int max_iters_;
    };
}

#endif