#ifndef __VOXELIZER_H__
#define __VOXELIZER_H__


#include <bnerf_utils/bnerf_utils.h>
#include <bnerf_utils/conversions.h>
#include <voxelizer/voxel.h>


namespace bnerf {
    class Voxelizer {
        public: 
        Voxelizer(ros::NodeHandle &);
        void SetInputTarget(CloudXYZ::ConstPtr);
        CloudXYZ::ConstPtr GetInputTarget() const;
        ros::Time GetStamp() const;
        
        public:
        const int & GetNumThreads() const;
        virtual string GetSolverName() const = 0;
        virtual double GetPenalty() const = 0;
        virtual void GetVoxels(const Vec3d &, vector<Voxel::ConstPtr> &) const = 0;
        
        protected:
        virtual int ComputeVolume() = 0;
        virtual void SetInputCallBack();
        virtual void GetAccumIds(const int &, vector<int> &) const = 0;

        protected:
        int threads_;
        CloudXYZ::ConstPtr target_;
        vector<Voxel::ConstPtr> voxels_;

        private:
        int min_pts_, strides_;
        Voxel::ConstPtr CreateVoxel(Eigen::Map<Mat34d> &);
        ros::Publisher info_pub_;

        public:
        typedef shared_ptr<Voxelizer> Ptr;
        typedef shared_ptr<const Voxelizer> ConstPtr;
    };
}

#endif