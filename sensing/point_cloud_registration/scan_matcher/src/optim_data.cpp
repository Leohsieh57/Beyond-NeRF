#include <glog/logging.h>
#include <bnerf_utils/conversions.h>
#include <omp.h>
#include <scan_matcher/optim_data.h>


namespace bnerf
{

#pragma omp declare reduction(+:Mat66d:omp_out+=omp_in) \
initializer(omp_priv=Mat66d::Zero())
#pragma omp declare reduction(+:Vec6d: omp_out+=omp_in) \
initializer(omp_priv=Vec6d::Zero())

    OptimData::OptimData(Voxelizer::ConstPtr voxer, CloudXYZ::ConstPtr source)
        : best_(new State)
        , temp_(new State)
        , voxer_(voxer)
        , source_(source)
        , threads_(voxer->GetNumThreads())
        , penalty_(voxer->GetPenalty())
    {
        LOG_ASSERT(source);
        const int w = source_->size();

        for (const auto st : {best_, temp_})
        {
            st->chi2s_.resize(w);
            st->voxels_.resize(w);
        }
        
        errors_.resize(w);
        trans_pts_.conservativeResize(3, w);
    }


    void OptimData::AccumulateHessian(State::ConstPtr st)
    {
        b_.setZero();
        H_.setZero();

        #pragma omp parallel for reduction(+:H_) reduction(+:b_) num_threads(threads_)
        for (const int &pid : st->valid_ids_)
        {
            const auto &voxels = st->voxels_[pid];
            LOG_ASSERT(!voxels.empty());

            Mat63d jb;
            jb.topRows<3>().setIdentity();
            jb.bottomRows<3>() = SO3d::hat(trans_pts_.col(pid));

            auto & res = errors_.at(pid);
            for (int rid = 0; rid < res.cols(); rid++)
            {
                const auto & vox = voxels.at(rid);
                b_ += jb * vox->info_ * res.col(rid);
                H_ += jb * vox->info_ * jb.transpose();
            }
        }
    }


    void OptimData::SetEstimation(const SE3d & trans)
    {
        temp_->trans_ = trans;
        double &loss = temp_->loss_ = 0;

        #pragma omp parallel for num_threads(threads_) reduction(+:loss)
        for (size_t pid = 0; pid < source_->size(); pid++)
        {
            auto pt = trans_pts_.col(pid);
            pt = source_->at(pid).getVector3fMap().cast<double>();

            auto & voxels = temp_->voxels_.at(pid);
            voxer_->GetVoxels(pt = trans * pt, voxels);

            if (voxels.empty()) 
                continue;

            auto & res = errors_.at(pid);
            res.conservativeResize(3, voxels.size());

            double & chi2 = temp_->chi2s_[pid] = 0;
            for (int rid = 0; rid < res.cols(); rid++)
            {
                const auto & vox = voxels.at(rid);
                const auto e = res.col(rid) = pt - vox->mean_;
                chi2 += e.transpose() * vox->info_ * e;
            }

            loss += chi2;
        }

        int invalids = source_->size() - GetValidIds(temp_);
        loss += invalids * penalty_;
    }


    int OptimData::GetValidIds(State::Ptr st) const 
    {
        int valids = 0;
        #pragma omp parallel for num_threads(threads_) reduction(+:valids)
        for (const auto &voxels : st->voxels_)
            if (!voxels.empty())
                valids++;

        vector<int> omp_ids[threads_];
        #pragma omp parallel for num_threads(threads_)
        for (auto &ids : omp_ids)
            ids.reserve(valids);

        #pragma omp parallel for num_threads(threads_)
        for (size_t i = 0; i < st->voxels_.size(); i++) 
            if (!st->voxels_[i].empty())
                omp_ids[omp_get_thread_num()].push_back(i);

        for (auto &ids : omp_ids) 
            if (omp_ids->size() < ids.size())
                omp_ids->swap(ids);

        vector<int> shifts = {0};
        shifts.reserve(threads_ + 1);
        for (const auto &ids : omp_ids)
            shifts.push_back(shifts.back() + ids.size());

        //reconstruct valid_ids
        omp_ids->resize(valids);
        omp_ids->swap(st->valid_ids_);
        
        #pragma omp parallel for num_threads(threads_)
        for (int i = 1; i < threads_; i++)
        {
            auto &ids = omp_ids[i];
            auto ibegin = st->valid_ids_.begin();
            copy(ids.begin(), ids.end(), ibegin + shifts[i]);
        }

        return valids;
    }
}