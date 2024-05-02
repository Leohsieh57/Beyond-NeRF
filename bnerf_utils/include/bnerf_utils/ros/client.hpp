#ifndef __UTILS_CLIENT_HPP__
#define __UTILS_CLIENT_HPP__


#include <ros/ros.h>
#include <glog/logging.h>
#include <bnerf_utils/typedef.h>


namespace bnerf {
    template<typename T>
    class Client: public T::Request, public ros::ServiceClient {
        public: 
        Client(ros::NodeHandle &nh, const string &topic)
            : T::Request()
            , ros::ServiceClient(nh.serviceClient<T>(topic))
        {
            LOG_ASSERT(waitForExistence(ros::Duration(5)))
                << endl << "failed to get response from '" 
                << getService() << "', aborting.. ";
        }
        
        typename T::Response &call() {
            LOG_ASSERT(ros::ServiceClient::call(req(), res_));
            return res_;
        }

        typename T::Response &res() {
            return res_;
        }

        typename T::Request &req() {
            return *this;
        }

        private:
        typename T::Response res_;
    };
}

#endif