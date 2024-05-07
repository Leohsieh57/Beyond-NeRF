#include <scan_matcher/state.h>
#include <omp.h>


namespace bnerf
{
    int State::GetValidIds(const int &threads)
    {
        int valids = 0;
        #pragma omp parallel for num_threads(threads) reduction(+:valids)
        for (const auto &vox : voxels_)
            valids += bool(vox);

        vector<int> omp_ids[threads];
        #pragma omp parallel for num_threads(threads)
        for (auto &ids : omp_ids)
            ids.reserve(valids);

        #pragma omp parallel for num_threads(threads)
        for (size_t i = 0; i < voxels_.size(); i++) 
            if (voxels_[i])
                omp_ids[omp_get_thread_num()].push_back(i);

        for (auto &ids : omp_ids) 
            if (omp_ids->size() < ids.size())
                omp_ids->swap(ids);

        vector<int> shifts = {0};
        shifts.reserve(threads + 1);
        for (const auto &ids : omp_ids)
            shifts.push_back(shifts.back() + ids.size());

        //reconstruct valid_ids
        omp_ids->resize(valids);
        omp_ids->swap(valid_ids_);
        
        #pragma omp parallel for num_threads(threads)
        for (int i = 1; i < threads; i++) {
            auto &ids = omp_ids[i];
            auto ibegin = valid_ids_.begin();
            copy(ids.begin(), ids.end(), ibegin + shifts[i]);
        }

        return valids;
    }
}