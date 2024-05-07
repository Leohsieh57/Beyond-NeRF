#include <glog/logging.h>
#include <bnerf_utils/conversions.h>
#include <omp.h>
#include <scan_matcher/scan_matcher.h>


namespace bnerf
{

#pragma omp declare reduction(+:Mat66d:omp_out+=omp_in) \
initializer(omp_priv=Mat66d::Zero())
#pragma omp declare reduction(+:Vec6d: omp_out+=omp_in) \
initializer(omp_priv=Vec6d::Zero())

    ScanMatcher::ScanMatcher(ros::NodeHandle & nh)
    {
        GET_REQUIRED(nh, "max_iters", max_iters_);
        GET_REQUIRED(nh, "num_threads", threads_);

        // GET_OPTIONAL(nh, "verbose", verbose_, false);
        GET_OPTIONAL(nh, "epsilon", epsilon_, 1e-3);
        epsilon_ *= epsilon_;
    }


    void ScanMatcher::SourceScanCallBack(const CloudXYZ::ConstPtr & source)
    {
        
    }


    void ScanMatcher::TargetScanCallBack(const CloudXYZ::ConstPtr & target)
    {
        LOG_ASSERT(target);

        const auto voxer = Voxelizer::CreatePtr();
        voxer->SetInputTarget(target);

        lock_guard<mutex> lock(vox_mutex_);
        voxer_ = voxer;
    }


    SE3d ScanMatcher::OptimizeAlignment(
        CloudXYZ::ConstPtr source, const SE3d & init_guess)
    {
        Data data(GetVoxelizer(), source, threads_);
        data.SetEstimation(init_guess);

        for (int i = 0; ros::ok() && i < max_iters_; i++)
        {
            data.best_.swap(data.temp_); //accept temp state
            data.AccumulateHessian(data.best_);

            Vec6d inc = data.H_.ldlt().solve(-data.b_);
            LOG_ASSERT(inc.allFinite());
            if (inc.squaredNorm() < epsilon_)
                break;

            SE3d eps = SE3d::exp(inc);
            data.SetEstimation(eps * data.best_->trans_);
        }

        return data.best_->trans_;
    }


    Voxelizer::Ptr ScanMatcher::GetVoxelizer()
    {
        lock_guard<mutex> lock(vox_mutex_);
        return voxer_;
    }
}