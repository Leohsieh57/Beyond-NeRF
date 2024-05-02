#ifndef __UTILS_PUBLISHER_HPP__
#define __UTILS_PUBLISHER_HPP__


#include <ros/ros.h>
#include <glog/logging.h>
#include <bnerf_utils/typedef.h>


namespace bnerf {
    template<typename T> 
    struct Publisher: public T, public ros::Publisher {
        Publisher(ros::NodeHandle &nh, const string &topic, const int &queue=10)
            : T()
            , ros::Publisher(nh.advertise<T>(topic, queue)) {}
        
        void publish() const {
            ros::Publisher::publish(static_cast<T>(*this));
        }

        void waitForSubscribers(
            const ros::Duration &dur = ros::Duration(5)) const 
        {
            const auto timeout = ros::Time::now() + dur;
            for (ros::Rate r(100); ros::ok(); r.sleep()) {
                if (getNumSubscribers())
                    return;
                else if (timeout < ros::Time::now()) 
                    break;
            }

            LOG(ERROR) << "failed to find " 
                << "subscriber to '" << getTopic();
        }

        
        T & msg() {
            return *this;
        }

        typedef shared_ptr<Publisher<T>> Ptr;
    };
}

#endif