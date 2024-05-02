#include <optimizer/optimizer.h>
#include <bnerf_utils/bnerf_utils.h>


using namespace bnerf;
unique_ptr<Optimizer> optimizer;

void ScanCallBack(const CloudXYZ::ConstPtr & cloud)
{
    const auto last_target = optimizer->GetInputTarget();
    optimizer->SetInputTarget(cloud);

    if (!last_target)
        return;

    optimizer->SetInputSource(last_target);
    optimizer->OptimizeAlignment(SE3d());
}


int main(int argc, char **argv) {
    FLAGS_colorlogtostderr = true;
    google::InstallFailureSignalHandler();

    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh("~");

    optimizer.reset(new Optimizer);
    auto sub = nh.subscribe("input_scan", 100, ScanCallBack);
    
    ros::spin();
    return 0;
}