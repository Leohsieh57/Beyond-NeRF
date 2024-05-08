#include <voxelizer/voxelizer.h>
#include <bnerf_utils/bnerf_utils.h>
#include <omp.h>


namespace bnerf {
    Voxelizer::Voxelizer(ros::NodeHandle &nh) {
        GET_REQUIRED(nh, "min_points",  min_pts_);
        GET_REQUIRED(nh, "num_threads", threads_);
        strides_ = 12 * threads_;
    }


    void Voxelizer::SetInputTarget(CloudXYZ::ConstPtr target) {
        LOG_ASSERT(target_ = target);
        const auto vol = ComputeVolume();
        LOG_ASSERT(vol > 0);

        Array<int> counts;
        Array<double> accums;
        Array<vector<int>> accum_ids;

        voxels_.resize(vol);
        accums.resize(size_t(vol) * strides_);
        counts.assign(size_t(vol) * threads_, 0);

        accum_ids.resize(target->size());
        #pragma omp parallel for num_threads(threads_)
        for (size_t i = 0; i < target_->size(); i++) {
            auto &ids = accum_ids[i];
            GetAccumIds(i, ids);
            
            size_t shift = vol * omp_get_thread_num();
            for (const int &id : ids) 
                counts[id + shift]++;
        }

        #pragma omp parallel for num_threads(threads_)
        for (int vid = 0; vid < vol; vid++) {
            int *data = counts + vid;
            for (int i = 1; i < threads_; i++) 
                counts[vid] += *(data += vol);
        }

        auto invalid = [this, & counts](const size_t &vid) {
            return counts[vid] < min_pts_; };
            
        auto get_acc = [this, & accums](const size_t &vid) {
            return accums + vid * strides_; };
            
        #pragma omp parallel for num_threads(threads_)
        for (int vid = 0; vid < vol; vid++) 
            if (!invalid(vid))
                fill_n(get_acc(vid), strides_, 0);

        #pragma omp parallel for num_threads(threads_)
        for (size_t i = 0; i < target_->size(); i++) {
            const auto &ids = accum_ids[i];
            if (all_of(ids.begin(), ids.end(), invalid))
                continue; 

            const auto &pt = target_->at(i);
            const auto b = pt.getVector3fMap().cast<double>();

            Mat34d acc;
            acc << b * b.transpose(), b;

            const int shift = 12 * omp_get_thread_num();
            for (const int &vid : ids) {
                if (invalid(vid))
                    continue;
                
                double *data = get_acc(vid) + shift;
                Eigen::Map<Mat34d>(data) += acc;
            }
        }

        #pragma omp parallel for num_threads(threads_)
        for (int vid = 0; vid < vol; vid++) {
            if (invalid(vid)) {
                voxels_[vid] = nullptr;
                continue;
            }

            double *data = get_acc(vid);
            Eigen::Map<Mat34d> acc(data);
            for (int i = 1; i < threads_; i++)
                acc += Eigen::Map<const Mat34d>(data+=12);

            acc /= counts[vid];
            voxels_[vid] = CreateVoxel(acc);
        }

        SetInputCallBack();
    }


    Voxel::ConstPtr Voxelizer::CreateVoxel(Eigen::Map<Mat34d> &acc) {
        const auto b = acc.rightCols<1>();
        auto H = acc.leftCols<3>() -= b * b.transpose();

        Eigen::SelfAdjointEigenSolver<Mat33d> solver;
        solver.compute(H);

        const Vec3d &evals = solver.eigenvalues();
        if ((evals.array() <= 0).any()) 
            return nullptr;

        Voxel::ConstPtr vox(new Voxel(acc.data()));
        const Mat33d &evecs = solver.eigenvectors();

        Vec3d diags = evals.cwiseInverse().normalized();
        H = evecs * diags.asDiagonal() * evecs.transpose();
        return vox;
    }


    const int & Voxelizer::GetNumThreads() const 
    {
        return threads_;
    }


    CloudXYZ::ConstPtr Voxelizer::GetInputTarget() const
    {
        return target_;
    }
    

    ros::Time Voxelizer::GetStamp() const
    {
        ros::Time stamp;
        pcl_conversions::fromPCL(target_->header.stamp, stamp);
        return stamp;
    }
}