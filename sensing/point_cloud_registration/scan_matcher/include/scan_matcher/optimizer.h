#ifndef __BNERF_OPTIM_DATA_H__
#define __BNERF_OPTIM_DATA_H__


#include <bnerf_utils/typedef.h>
#include <voxelizer/voxelizer.h>
#include <bnerf_msgs/ScanMatchingInfo.h>
#include <bnerf_msgs/GraphBinaryEdge.h>


namespace bnerf
{
    struct Optimizer
    {
        Optimizer(Voxelizer::ConstPtr, sensor_msgs::PointCloud2::ConstPtr);
        void GetScanMatchingInfo(bnerf_msgs::ScanMatchingInfo &) const;
        void GetCombinedScan(CloudXYZI &) const;
        void GetGraphBinaryEdge(bnerf_msgs::GraphBinaryEdge &) const;

        SE3d trans_;
        ros::Time src_stamp_;
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