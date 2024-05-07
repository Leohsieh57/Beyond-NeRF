#ifndef __VOXEL_H__
#define __VOXEL_H__


#include <bnerf_utils/typedef.h>


namespace bnerf {
    struct Voxel {
        Voxel (const double * data)
            : mean_(data + 9)
            , info_(data) {}

        Vec3d mean_;
        Mat33d info_;

        typedef shared_ptr<Voxel> Ptr;
        typedef shared_ptr<const Voxel> ConstPtr;
    };
}

#endif