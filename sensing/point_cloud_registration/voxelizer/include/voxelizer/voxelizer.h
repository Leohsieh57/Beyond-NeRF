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
        virtual double GetPenalty() const = 0;
        virtual Voxel::ConstPtr GetVoxel(const Vec3d &) const = 0;

        protected:
        virtual int ComputeVolume() = 0;
        virtual void GetAccumIds(const int &, vector<int> &) const = 0;
        virtual void SetInputCallBack() {};

        protected:
        int threads_;
        CloudXYZ::ConstPtr target_;
        vector<Voxel::ConstPtr> voxels_;

        private:
        int min_pts_, strides_;
        Voxel::ConstPtr CreateVoxel(Eigen::Map<Mat34d> &);

        public:
        typedef shared_ptr<Voxelizer> Ptr;
        typedef shared_ptr<const Voxelizer> ConstPtr;
    };
}

#endif