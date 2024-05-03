#include <optimizer/optimizer.h>
#include <bnerf_utils/bnerf_utils.h>


using namespace bnerf;
unique_ptr<Optimizer> optimizer;
unique_ptr<TransformCaster> caster;
CloudPublisher::Ptr target_pub, source_pub;

void ScanCallBack(const CloudXYZ::ConstPtr & cloud)
{
    const auto last_target = optimizer->GetInputTarget();
    optimizer->SetInputTarget(cloud);

    if (!last_target)
        return;

    optimizer->SetInputSource(last_target);
    SE3d trans = optimizer->OptimizeAlignment(SE3d());
    const auto stamp = ros::Time::now();
    
    target_pub->SetInput(optimizer->GetInputTarget(), stamp);
    source_pub->SetInput(optimizer->GetInputSource(), stamp);

    caster->cast("scan", trans, stamp);
    target_pub->TimerCallBack();
    source_pub->TimerCallBack();
}


int main(int argc, char **argv) {
    FLAGS_colorlogtostderr = true;
    google::InstallFailureSignalHandler();

    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh("~");

    optimizer.reset(new Optimizer);
    caster.reset(new TransformCaster);
    target_pub.reset(new CloudPublisher(nh, "target", "map"));
    source_pub.reset(new CloudPublisher(nh, "source", "scan"));

    auto sub = nh.subscribe("input_scan", 1, ScanCallBack);

    ros::spin();
    return 0;
}