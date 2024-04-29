#include <dust3r/dust3r_tester.h>
#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>


namespace bnerf
{
    Dust3rTester::Dust3rTester(
        ros::NodeHandle & nh, const std::string & topic)
    {
        cli_ = nh.serviceClient<bnerf_msgs::PredictDUSt3R>("predict_dust3r");
        LOG_ASSERT(cli_.waitForExistence(ros::Duration(5)));

        pub_ = nh.advertise<sensor_msgs::PointCloud2>(topic + "_cloud", 10);
        sub_ = nh.subscribe(topic, 10, &Dust3rTester::ImageCallBack, this);
    }


    void Dust3rTester::ImageCallBack(const sensor_msgs::Image & msg)
    {
        req_.img1 = move(req_.img2);
        req_.img2 = msg;
        if (req_.img1.data.empty())
            return;

        bnerf_msgs::PredictDUSt3R::Response res;
        LOG_ASSERT(cli_.call(req_, res));

        if (res.cloud1.data.empty() || res.cloud2.data.empty())
        {
            LOG(WARNING) << "returned empty cloud, omitting.." << std::endl;
            return;
        }

        res.cloud1.width += res.cloud2.width;
        res.cloud1.data.insert(res.cloud1.data.end(), 
            res.cloud2.data.begin(), res.cloud2.data.end());

        pub_.publish(res.cloud1);
    }
}
