#include <glog/logging.h>
#include <bnerf_utils/conversions.h>
#include <omp.h>
#include <optimizer/optimizer.h>


namespace bnerf {

#pragma omp declare reduction(+:Mat66d:omp_out+=omp_in) \
initializer(omp_priv=Mat66d::Zero())
#pragma omp declare reduction(+:Vec6d: omp_out+=omp_in) \
initializer(omp_priv=Vec6d::Zero())

    Optimizer::Optimizer(const string &ns)
        : voxer_(Voxelizer::CreatePtr())
        , best_(new State)
        , temp_(new State)
        , k_(kmeans_.cols())
        , preds_(k_)
    {
        ros::NodeHandle nh(ns);
        GET_REQUIRED(nh, "max_iters", max_iters_);
        GET_REQUIRED(nh, "num_threads", threads_);

        GET_OPTIONAL(nh, "verbose", verbose_, false);
        GET_OPTIONAL(nh, "epsilon", epsilon_, 1e-3);
        epsilon_ *= epsilon_;
    }


    SE3d Optimizer::OptimizeAlignment(const SE3d &guess, function<void()> callback) {
        penalty_ = voxer_->GetPenalty();
        SetEstimation(guess);

        if (verbose_)
            LOG(WARNING) << "scan size: " << source_->size();

        for (int i = 0; ros::ok() && i < max_iters_; i++) {
            best_.swap(temp_); //accept temp state
            AccumulateHessian(best_);
            if (callback != NULL)
                callback();

            Vec6d inc = H_.ldlt().solve(-b_);
            LOG_ASSERT(inc.allFinite());
            if (inc.squaredNorm() < epsilon_)
                break;

            SE3d eps = SE3d::exp(inc);
            SetEstimation(eps * best_->trans_);
            
            if (verbose_) 
                LOG(INFO) << setprecision(12)
                    << "iteration: " << i
                    << "\tvalids-points: " << temp_->valid_ids_.size()
                    << "\ttotal-loss: " << temp_->loss_;
        }

        return best_->trans_;
    }


    void Optimizer::ConvexClustering() {
        auto kmeans = move(kmeans_);
        Eigen::SelfAdjointEigenSolver<Mat66d> sol;
        sol.compute(H_);

        Vec6d diags = sol.eigenvalues();
        diags = diags.cwiseMax(1e-6);
        diags = diags.cwiseInverse().cwiseSqrt();

        kmeans_.setZero();
        auto radis = kmeans_.rightCols<6>();
        radis = sol.eigenvectors();
        radis *= sqrt(6) * diags.asDiagonal();
        kmeans_.leftCols<6>() = -radis;

        const int area = k_ * threads_;
        Mat67d accums[area];
        for (int iteration = 0; ros::ok() && iteration < max_iters_; iteration++) {
            #pragma omp parallel for num_threads(threads_)
            for (auto &acc : accums)
                acc.setZero();

            #pragma omp parallel for num_threads(threads_)
            for (const int &pid : best_->valid_ids_) {
                const auto b = jacobs_.col(pid);
                const auto H = hessis_.middleCols<6>(6 * pid);

                auto &label = labels_[pid];
                VecXd dists = 2 * kmeans_.transpose() * b;
                dists += (kmeans_.transpose() * H * kmeans_).diagonal();
                dists.minCoeff(&label);

                const int i = label + k_ * omp_get_thread_num();
                auto &acc = accums[i];
                acc.leftCols<6>()  += H;
                acc.rightCols<1>() += b;
            }

            ClusteringCallBack();

            #pragma omp parallel for num_threads(threads_)
            for (int i = 0; i < k_; i++) {
                auto &acc = accums[i];
                for (int j = i + k_; j < area; j+=k_) 
                    acc += accums[j];

                const auto H = acc.leftCols<6>();
                const auto b = acc.rightCols<1>();
                kmeans.col(i) = H.ldlt().solve(-b);
            }

            kmeans.swap(kmeans_);
            kmeans -= kmeans_;

            bool converge = true;
            for (int i = 0; converge && i < k_; i++) 
                converge &= kmeans.col(i).squaredNorm() < epsilon_;

            if (converge)
                break;
        }
    }


    void Optimizer::AccumulateHessian(State::ConstPtr st) {
        b_.setZero();
        H_.setZero();

        #pragma omp parallel for reduction(+:H_) reduction(+:b_) num_threads(threads_)
        for (const int &pid : st->valid_ids_) {
            const auto &vox = st->voxels_[pid];
            LOG_ASSERT(vox);

            Mat63d jb;
            jb.topRows<3>().setIdentity();
            jb.bottomRows<3>() = SO3d::hat(trans_pts_.col(pid));

            auto bi = jacobs_.col(pid);
            auto Hi = hessis_.middleCols<6>(6 * pid);

            b_ += bi = jb * vox->info_ * errors_.col(pid);
            H_ += Hi = jb * vox->info_ * jb.transpose();
        }
    }


    void Optimizer::SetEstimation(const SE3d &trans) {
        temp_->trans_ = trans;
        double &loss = temp_->loss_ = 0;

        #pragma omp parallel for num_threads(threads_) reduction(+:loss)
        for (size_t pid = 0; pid < source_->size(); pid++) {
            auto pt = trans_pts_.col(pid);
            pt = source_->at(pid).getVector3fMap().cast<double>();

            auto &vox = temp_->voxels_[pid];
            vox = voxer_->GetVoxel(pt = trans * pt);
            if (!vox) 
                continue;
            
            const auto res = errors_.col(pid) = pt - vox->mean_;
            double &chi2 = temp_->chi2s_[pid];
            loss += chi2 = res.dot(vox->info_ * res);
        }

        int valids = temp_->GetValidIds(threads_);
        loss += penalty_ * (source_->size() - valids);
    }


    void Optimizer::SetInputSource(CloudXYZ::ConstPtr source) {
        LOG_ASSERT(source_ = source);
        const int w = source_->size();

        for (const auto st : {best_, temp_}) {
            st->chi2s_.resize(w);
            st->voxels_.resize(w);
        }
        
        labels_.resize(w);
        errors_.conservativeResize(3, w);
        jacobs_.conservativeResize(6, w);
        hessis_.conservativeResize(6, w * 6);
        trans_pts_.conservativeResize(3, w);
    }


    CloudXYZ::ConstPtr Optimizer::GetInputSource() const {
        return source_;
    }


    CloudXYZ::ConstPtr Optimizer::GetInputTarget() const {
        return target_;
    }


    void Optimizer::SetInputTarget(CloudXYZ::ConstPtr target) {
        voxer_->SetInputTarget(target_ = target);
    }
}