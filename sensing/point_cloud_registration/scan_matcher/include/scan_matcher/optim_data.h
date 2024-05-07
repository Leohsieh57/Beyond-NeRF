#ifndef __BNERF_OPTIM_DATA_H__
#define __BNERF_OPTIM_DATA_H__


#include <bnerf_utils/typedef.h>
#include <scan_matcher/state.h>
#include <voxelizer/voxelizer.h>


namespace bnerf
{
    struct OptimData
    {
        OptimData(Voxelizer::Ptr, CloudXYZ::ConstPtr);

        State::Ptr best_, temp_;
        Voxelizer::Ptr voxer_;
        CloudXYZ::ConstPtr source_;
        int threads_;
        double penalty_;

        Mat66d H_;
        Vec6d  b_;
        Mat3Xd errors_;
        Mat6Xd hessis_;
        Mat6Xd jacobs_;
        Mat3Xd trans_pts_;

        void SetEstimation(const SE3d &);
        void AccumulateHessian(State::ConstPtr);
        int GetValidIds(State::Ptr) const;
    };
}

#endif