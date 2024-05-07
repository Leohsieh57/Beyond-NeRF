#ifndef __DRC_STATE_H__
#define __DRC_STATE_H__


#include <bnerf_utils/typedef.h>
#include <voxelizer/voxel.h>


namespace bnerf
{
    struct State
    {
        SE3d trans_;
        double loss_;

        vector<double> chi2s_;
        vector<int> valid_ids_;
        vector<Voxel::ConstPtr> voxels_;

        int GetValidIds(const int &);
        
        typedef shared_ptr<State> Ptr;
        typedef shared_ptr<const State> ConstPtr;
    };
}

#endif