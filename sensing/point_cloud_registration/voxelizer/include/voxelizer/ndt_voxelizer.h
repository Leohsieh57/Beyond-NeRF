#ifndef __NDT_VOXELIZER_H__
#define __NDT_VOXELIZER_H__


#include <voxelizer/voxelizer.h>


namespace bnerf {
    class NdtVoxelizer : public Voxelizer {
        public: 
        NdtVoxelizer(ros::NodeHandle &);
        Voxel::ConstPtr GetVoxel(const Vec3d &) const override;
        
        private: 
        int ComputeVolume() override;
        double GetPenalty() const override;
        void GetAccumIds(const int &, vector<int> &) const override;
        
        private: 
        Vec4i shifts_;
        Vec3d leaf_, ileaf_;
        Vec3d box_min_, box_max_; 
    };
}

#endif