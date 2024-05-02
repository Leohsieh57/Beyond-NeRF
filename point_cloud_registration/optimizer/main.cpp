#include <optimizer/optimizer.h>
#include <bnerf_utils/bnerf_utils.h>


using namespace bnerf;


int main(int argc, char **argv) {
    FLAGS_colorlogtostderr = true;
    google::InstallFailureSignalHandler();

    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh(ros::this_node::getName());

    Optimizer solver;
    auto loader = ScanLoader::CreatePtr();
    
    vector<ScanLoader::Regist> regs;
    for (const auto &seq : loader->GetSeqs()) {
        loader->LookUpRegists(seq, regs);
        
        for (const auto &[map_id, scan_ids] : regs) {
            auto cloud = loader->Load<Scan::RAW_SCAN>(seq, map_id);
            solver.SetInputTarget(cloud);
            for (const int &scan_id : scan_ids) {
                if (!ros::ok())
                    return 0;

                auto cloud = loader->Load<Scan::FILTERED>(seq, scan_id);
                solver.SetInputSource(cloud);
                solver.OptimizeAlignment(SE3d());
            }
        } 
    }

    ros::spin();
    return 0;
}