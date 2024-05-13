#ifndef __BNERF_FLOOR_H__
#define __BNERF_FLOOR_H__


#include <sensor_msgs/PointCloud2.h>
#include <bnerf_utils/bnerf_utils.h>


namespace bnerf
{
    struct Floor
    {
        Vec4d coeffs_;
        CloudXYZ::ConstPtr cloud_;
    };
}

#endif // __BNERF_FLOOR_H__
