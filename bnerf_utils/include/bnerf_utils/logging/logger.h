#ifndef __UTILS_CQDM_LOGGER_H__
#define __UTILS_CQDM_LOGGER_H__


#include <bnerf_utils/ros/publisher.hpp>
#include <bnerf_msgs/Log.h>
#include "dstream.h"
#include <glog/logging.h>


namespace bnerf {
    class Logger: public Publisher<bnerf_msgs::Log> {
        public: 
        Logger(ros::NodeHandle &, const string &);

        public: 
        void Log(const string &, const bool & ls = true);
        void SetLength(const uint &);
        dstream ss(const bool &);
        template<bool ls> dstream ss() {return ss(ls);};

        private: 
        ros::Time begin_;
    };

}

#endif
