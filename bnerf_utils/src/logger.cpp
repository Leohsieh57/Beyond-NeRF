#include <bnerf_utils/logging/logger.h>


namespace bnerf {
    Logger::Logger(ros::NodeHandle &nh, const string &topic)
        : Publisher<bnerf_msgs::Log>(nh, topic, 1) {}
        

    void Logger::Log(const string & log, const bool & proceed) {
        if (length == 0)
            LOG(ERROR) << "empty progress published, "
                << "please call Logger::SetLength() first";

        const auto stamp = ros::Time::now();
        if (!progress)
            begin_ = stamp;

        if (proceed == PROCEED) {
            duration = stamp - begin_;
            progress++;
            progress %= length;
        }
        
        if (!log.empty())
            this->log = log;

        publish();
    }


    void Logger::SetLength(const uint &len) {
        progress = 0;
        length = len;
    }


    dstream Logger::ss(const bool & ls) {
        const auto lambda = bind(&bnerf::Logger::Log, this, bnerf::_1, ls);
        return dstream(lambda);
    }

}