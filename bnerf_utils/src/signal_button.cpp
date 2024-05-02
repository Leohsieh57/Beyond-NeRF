#include <bnerf_utils/logging/signal_button.h>


namespace bnerf {
    SignalButton::SignalButton(ros::NodeHandle &nh, const string &topic) 
        : stamp_(0)
        , sub_(nh.subscribe(topic, 1, &SignalButton::SignalCallBack, this))
    {
        
    }


    void SignalButton::SignalPending() const {
        const auto stamp = stamp_;
        for (ros::Rate r(30); ros::ok(); r.sleep())
            if (stamp != stamp_)
                break;
    }


    void SignalButton::SignalCallBack(const std_msgs::Empty &) {
        stamp_ = ros::Time::now();
    }
}