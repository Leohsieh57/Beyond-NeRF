#ifndef __BNERF_OPTIM_DATA_H__
#define __BNERF_OPTIM_DATA_H__


#include <bnerf_utils/typedef.h>
#include <voxelizer/voxelizer.h>


namespace bnerf
{
    struct Optimizer
    {
        Optimizer(Voxelizer::ConstPtr, CloudXYZ::ConstPtr);

        SE3d trans_;
        Voxelizer::ConstPtr voxer_;
        CloudXYZ::ConstPtr source_;
        vector<vector<Voxel::ConstPtr>> voxels_;

        int threads_;

        Mat66d H_;
        Vec6d  b_;
        double loss_;
        vector<double> chi2s_;
        Mat3Xd trans_pts_;
        vector<Mat3Xd> residuals_;

        vector<int> valid_ids_;
        void SetEstimation(const SE3d &);
        void AccumulateHessian();
        int GetValidIds();
    };
}

#endif