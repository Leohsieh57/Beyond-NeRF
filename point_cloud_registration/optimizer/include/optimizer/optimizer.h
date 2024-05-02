#ifndef __OPTIMIZER_H__
#define __OPTIMIZER_H__


#include <bnerf_utils/logging/logger.h>
#include <voxelizer/voxelizer.h>
#include "state.h"


namespace bnerf {
    class Optimizer {
        public: 
        Optimizer(const string &ns = "optimizer");
        Sim3d OptimizeAlignment(const Sim3d &, 
            function<void()> callback = NULL);

        void SetInputSource(CloudXYZ::ConstPtr);
        void SetInputTarget(CloudXYZ::ConstPtr);

        CloudXYZ::ConstPtr GetInputSource() const;
        CloudXYZ::ConstPtr GetInputTarget() const;

        
        //registrator stuffs
        protected:
        Voxelizer::Ptr voxer_;
        State::Ptr best_, temp_;
        CloudXYZ::ConstPtr source_, target_;

        //parameter & node handle stuffs
        protected: 
        bool verbose_;
        double epsilon_;
        int threads_;
        int max_iters_;
        double penalty_;
        

        //optimization internal
        protected: 
        Mat77d H_;
        Vec7d  b_;
        Mat3Xd errors_;
        Mat7Xd hessis_;
        Mat7Xd jacobs_;
        Mat3Xd trans_pts_;

        void SetEstimation(const Sim3d &);
        void AccumulateHessian(State::ConstPtr);
    };
}

#endif