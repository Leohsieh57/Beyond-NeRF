#include <glog/logging.h>
#include <bnerf_utils/conversions.h>
#include <omp.h>
#include <scan_matcher/optimizer.h>


namespace bnerf
{

#pragma omp declare reduction(+:Mat66d:omp_out+=omp_in) \
initializer(omp_priv=Mat66d::Zero())
#pragma omp declare reduction(+:Vec6d: omp_out+=omp_in) \
initializer(omp_priv=Vec6d::Zero())

    Optimizer::Optimizer(Voxelizer::ConstPtr voxer, CloudXYZ::ConstPtr source)
        : voxer_(voxer)
        , source_(source)
        , voxels_(source->size())
        , threads_(voxer->GetNumThreads())
        , chi2s_(source->size())
        , trans_pts_(3, source->size())
        , residuals_(source->size())
    {

    }


    void Optimizer::AccumulateHessian()
    {
        b_.setZero();
        H_.setZero();

        #pragma omp parallel for reduction(+:H_) reduction(+:b_) num_threads(threads_)
        for (const int &pid : valid_ids_)
        {
            const auto &voxels = voxels_[pid];
            LOG_ASSERT(!voxels.empty());

            Mat36d jb;
            jb.leftCols<3>().setIdentity();
            jb.rightCols<3>() = SO3d::hat(-trans_pts_.col(pid));

            auto & res = residuals_[pid];
            for (int rid = 0; rid < res.cols(); rid++)
            {
                const auto & vox = voxels[rid];
                b_ += jb.transpose() * vox->info_ * res.col(rid);
                H_ += jb.transpose() * vox->info_ * jb;
            }
        }
    }


    void Optimizer::SetEstimation(const SE3d & trans)
    {
        loss_ = 0;
        trans_ = trans;
        #pragma omp parallel for num_threads(threads_) reduction(+:loss_)
        for (size_t pid = 0; pid < source_->size(); pid++)
        {
            auto pt = trans_pts_.col(pid);
            pt = source_->at(pid).getVector3fMap().cast<double>();

            auto & voxels = voxels_[pid];
            voxer_->GetVoxels(pt = trans * pt, voxels);

            auto & res = residuals_[pid];
            res.conservativeResize(3, voxels.size());

            double &chi2 = chi2s_[pid] = 0;
            for (int rid = 0; rid < res.cols(); rid++)
            {
                const auto & vox = voxels[rid];
                const auto e = res.col(rid) = pt - vox->mean_;
                chi2 += e.transpose() * vox->info_ * e;
            }

            loss_ += chi2;
        }

        GetValidIds();
    }


    int Optimizer::GetValidIds() 
    {
        int valids = 0;
        #pragma omp parallel for num_threads(threads_) reduction(+:valids)
        for (const auto &voxels : voxels_)
            if (!voxels.empty())
                valids++;

        vector<int> omp_ids[threads_];
        #pragma omp parallel for num_threads(threads_)
        for (auto &ids : omp_ids)
            ids.reserve(valids);

        #pragma omp parallel for num_threads(threads_)
        for (size_t i = 0; i < voxels_.size(); i++) 
            if (!voxels_[i].empty())
                omp_ids[omp_get_thread_num()].push_back(i);

        for (auto &ids : omp_ids) 
            if (omp_ids->size() < ids.size())
                omp_ids->swap(ids);

        vector<int> shifts = {0};
        for (const auto &ids : omp_ids)
            shifts.push_back(shifts.back() + ids.size());

        //reconstruct valid_ids
        omp_ids->resize(valids);
        omp_ids->swap(valid_ids_);
        
        #pragma omp parallel for num_threads(threads_)
        for (int i = 1; i < threads_; i++)
        {
            auto &ids = omp_ids[i];
            auto ibegin = valid_ids_.begin();
            copy(ids.begin(), ids.end(), ibegin + shifts[i]);
        }

        return valids;
    }
}