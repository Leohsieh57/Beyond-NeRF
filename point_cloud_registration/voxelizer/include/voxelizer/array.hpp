#ifndef __SCAN_MATCHER_ARRAY_H__
#define __SCAN_MATCHER_ARRAY_H__


#include <bnerf_utils/typedef.h>
#include <bnerf_utils/logging/logger.h>


namespace bnerf {
    template<typename T> class Array : public unique_ptr<T[]> {
        public: 
        Array()
            : unique_ptr<T[]>(nullptr)
            , size_(0)
            , capacity_(0) {}

        void resize(const size_t &len) {
            size_ = len;
            if (capacity_ >= size_)
                return;

            capacity_ = size_;
            unique_ptr<T[]>::reset(new T[capacity_]);
        }

        const size_t & size() const {
            return size_;
        }
        
        void assign(const size_t &len, const T &val) {
            resize(len);
            fill_n(unique_ptr<T[]>::get(), size(), val);
        }

        T* operator+(const size_t &shift) {
            return unique_ptr<T[]>::get() + shift;
        }

        private:
        size_t size_, capacity_;
    };
}

#endif