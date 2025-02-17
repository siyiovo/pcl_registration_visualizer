#include <pcl/registration/icp.h>

#include "pcl_registration_visualizer/pointcloud_matching_visualizer.hpp"

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZ;
PointCloudXYZ::Ptr icp_reg(const PointCloudXYZ::Ptr &pcl_tgt, const PointCloudXYZ::Ptr &pcl_src)
{
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-5);
    icp.setInputSource(pcl_src);
    icp.setInputTarget(pcl_tgt);
    PointCloudXYZ::Ptr pcl_result(new PointCloudXYZ);
    icp.align(*pcl_result);
    return pcl_result;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "icp_pcl_node");
    pcl_match::Computer<PointType> computer;
    computer.compute(icp_reg);
    pcl_match::Visualizer<PointType> visualizer(computer);
    visualizer.visualize();
    return 0;
}
