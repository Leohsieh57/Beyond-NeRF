#ifndef __VOXELIZER_GICP_H__
#define __VOXELIZER_GICP_H__


#include <voxelizer/voxelizer.h>
#include <pcl/kdtree/kdtree_flann.h>


namespace bnerf {
    class VoxelizerGICP : public Voxelizer {
        public: 
        VoxelizerGICP(ros::NodeHandle &);
        Voxel::ConstPtr GetVoxel(const Vec3d &) const override;
        
        private: 
        int ComputeVolume() override;
        double GetPenalty() const override;
        void GetAccumIds(const int &, vector<int> &) const override;
        void SetInputCallBack() override;

        private: 
        int accum_knn_;
        double accum_radi_, match_radi_;
        vector<Voxel::ConstPtr> valids_;
        pcl::KdTreeFLANN<PointXYZ> tree_;
    };
}

#endif