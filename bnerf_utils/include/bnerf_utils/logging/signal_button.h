#ifndef __SIGNAL_BUTTON_H__
#define __SIGNAL_BUTTON_H__


#include <ros/ros.h>
#include <bnerf_utils/typedef.h>
#include <std_msgs/Empty.h>


namespace bnerf {
    class SignalButton {
        public: 
        SignalButton(ros::NodeHandle &, const string &);
        void SignalPending() const;
        typedef shared_ptr<const SignalButton> ConstPtr;

        private:
        ros::Time stamp_;
        ros::Subscriber sub_;
        void SignalCallBack(const std_msgs::Empty &);
    };
}

#endif