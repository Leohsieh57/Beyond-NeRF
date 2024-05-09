#include <voxelizer/voxelizer_ndt.h>
#include <bnerf_utils/bnerf_utils.h>


namespace bnerf {
    VoxelizerNDT::VoxelizerNDT(ros::NodeHandle &nh)
        : Voxelizer(nh)
    {
        double leaf;
        GET_REQUIRED(nh, "leaf_size", leaf);
        
        leaf_.fill(leaf);
        LOG_ASSERT((leaf_.array() > 0).all());

        ileaf_ = leaf_.cwiseInverse();
        shifts_.setOnes();

        string search;
        GET_REQUIRED(nh, "neighbor_search", search);

        if (search == "direct1")
        {
            neighs_.resize(3, 1);
            neighs_.setZero();
        }
        else if (search == "direct7")
        {
            neighs_.resize(3, 7);
            neighs_.setZero();
            neighs_.leftCols<3>().diagonal() << -leaf_;
            neighs_.rightCols<3>().diagonal() << leaf_;
        }
        else if (search == "direct27")
        {
            int idx = 0;
            neighs_.resize(3, 27);
            for (int i = -1; i <= 1; i++)
                for (int j = -1; j <= 1; j++)
                    for (int k = -1; k <= 1; k++)
                        neighs_.col(idx++) = Vec3d(i, j, k).cwiseProduct(leaf_);
        }
        else
            LOG(FATAL) << "\nunknown search method '" 
                << search << "' specified, aborting.. " << endl;
        
    }


    void VoxelizerNDT::GetVoxels(const Vec3d & pt, 
        vector<Voxel::ConstPtr> & voxels) const
    {
        voxels.clear();
        for (int i = 0; i < neighs_.cols(); i++)
        {
            const auto voxel = GetVoxel(pt + neighs_.col(i));
            if (voxel)
                voxels.push_back(voxel);
        }
    }


    Voxel::ConstPtr VoxelizerNDT::GetVoxel(const Vec3d &pt) const 
    {
        if ((pt.array() >= box_max_.array()).any())
            return nullptr;

        if ((pt.array() <= box_min_.array()).any())
            return nullptr;

        auto ids = ileaf_.cwiseProduct(pt - box_min_).cast<int>();
        uint vid = shifts_.tail<3>().dot(ids);
        return voxels_.at(vid);
    }


    double VoxelizerNDT::GetPenalty() const 
    {
        return leaf_.squaredNorm();
    }


    int VoxelizerNDT::ComputeVolume() 
    {
        const auto pts = target_->getMatrixXfMap().topRows<3>();
        box_min_ = pts.rowwise().minCoeff().cast<double>(); 
        box_max_ = pts.rowwise().maxCoeff().cast<double>();

        Vec3d grids = box_max_ - box_min_;
        grids = ileaf_.cwiseProduct(grids).array().ceil();
        shifts_.head<3>() = grids.cast<int>();
        grids = grids.cwiseProduct(leaf_) / 2;

        box_max_ = box_min_ = 0.5 * (box_max_ + box_min_);
        box_min_ -= grids;
        box_max_ += grids;

        LOG_ASSERT((shifts_.array() > 0).all());
        for (int i = 1; i < 3; i++)
            shifts_.head(i) *= shifts_[i];

        return shifts_[0];
    }


    void VoxelizerNDT::GetAccumIds(
        const int &pid, vector<int> &accum_ids) const 
    {
        Vec3d ids = target_->at(pid).getVector3fMap().cast<double>();
        ids = ileaf_.cwiseProduct(ids -= box_min_);
        
        int aid = shifts_.tail<3>().dot(ids.cast<int>());
        accum_ids.assign(1, aid);
    }


    string VoxelizerNDT::GetSolverName() const
    {
        return "normal_distribution_transformation";
    }
}