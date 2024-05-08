#ifndef __VOXELIZER_NDT_H__
#define __VOXELIZER_NDT_H__


#include <voxelizer/voxelizer.h>


namespace bnerf {
    class VoxelizerNDT : public Voxelizer {
        public: 
        VoxelizerNDT(ros::NodeHandle &);
        void GetVoxels(const Vec3d &, vector<Voxel::ConstPtr> &) const override;
        Voxel::ConstPtr GetVoxel(const Vec3d &) const;
        
        private: 
        int ComputeVolume() override;
        double GetPenalty() const override;
        void GetAccumIds(const int &, vector<int> &) const override;
        
        
        private: 
        Mat3Xd neighs_;
        Vec4i shifts_;
        Vec3d leaf_, ileaf_;
        Vec3d box_min_, box_max_; 
    };
}

#endif