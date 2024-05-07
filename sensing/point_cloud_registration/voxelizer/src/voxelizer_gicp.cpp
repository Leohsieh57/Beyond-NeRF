#include <voxelizer/voxelizer_gicp.h>
#include <bnerf_utils/bnerf_utils.h>
#include <omp.h>


namespace bnerf {
    VoxelizerGICP::VoxelizerGICP(ros::NodeHandle &nh)
        : Voxelizer(nh)
    {
        GET_REQUIRED(nh, "accum_knn", accum_knn_);
        GET_REQUIRED(nh, "accum_radius", accum_radi_);
        GET_REQUIRED(nh, "match_radius", match_radi_);
    }


    double VoxelizerGICP::GetPenalty() const {
        return match_radi_ * match_radi_;
    }


    Voxel::ConstPtr VoxelizerGICP::GetVoxel(const Vec3d &pt) const {
        vector<int> ids;
        vector<float> dists;

        const PointXYZ query(pt.x(), pt.y(), pt.z());
        tree_.radiusSearch(query, match_radi_, ids, dists, 1);

        return ids.empty()? nullptr : valids_[ids[0]];
    }


    void VoxelizerGICP::SetInputCallBack() {
        const size_t &vol = voxels_.size();

        size_t valids = 0;
        #pragma omp parallel for num_threads(threads_) reduction(+:valids)
        for (size_t i = 0; i < vol; i++)
            if (voxels_[i])
                valids++;

        vector<Voxel::ConstPtr> omp_voxels[threads_];
        #pragma omp parallel for num_threads(threads_)
        for (auto &voxels : omp_voxels)
            voxels.reserve(valids);

        #pragma omp parallel for num_threads(threads_)
        for (size_t i = 0; i < vol; i++) {
            const auto &vox = voxels_[i];
            if (!vox)
                continue;

            const int tid = omp_get_thread_num();
            omp_voxels[tid].push_back(vox);
        }

        for (auto &voxels : omp_voxels) 
            if (omp_voxels->size() < voxels.size())
                omp_voxels->swap(voxels);

        vector<size_t> shifts = {0};
        shifts.reserve(threads_ + 1);
        for (const auto &ids : omp_voxels)
            shifts.push_back(shifts.back() + ids.size());

        valids_.swap(*omp_voxels);
        valids_.resize(shifts.back());
        
        #pragma omp parallel for num_threads(threads_)
        for (int i = 1; i < threads_; i++) {
            auto &voxels = omp_voxels[i];
            auto ibegin = valids_.begin() + shifts[i];
            move(voxels.begin(), voxels.end(), ibegin);
        }

        CloudXYZ::Ptr centroids(new CloudXYZ);
        centroids->resize(valids);
        
        #pragma omp parallel for num_threads(threads_)
        for (size_t i = 0; i < valids; i++) {
            const auto &vox = valids_[i];
            LOG_ASSERT(vox);

            centroids->at(i).getVector3fMap()
                << vox->mean_.cast<float>();
        }

        tree_.setInputCloud(centroids);
    }


    int VoxelizerGICP::ComputeVolume() 
    {
        tree_.setInputCloud(target_);
        return target_->size();
    }


    void VoxelizerGICP::GetAccumIds(
        const int &pid, vector<int> &ids) const 
    {
        vector<float> dists;
        tree_.radiusSearch(pid, accum_radi_, ids, dists, accum_knn_);
    }
}