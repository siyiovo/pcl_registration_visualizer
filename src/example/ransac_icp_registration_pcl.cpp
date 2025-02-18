#include <pcl/point_types.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "pcl_registration_visualizer/pointcloud_matching_visualizer.hpp"

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZ;

pcl::PointCloud<pcl::Normal>::Ptr get_normal(const PointCloudXYZ::Ptr &pcl_in,
                                             pcl::NormalEstimationOMP<PointType, pcl::Normal>::Ptr &norm_est,
                                             pcl::search::KdTree<PointType>::Ptr &tree)
{
    norm_est->setInputCloud(pcl_in);
    norm_est->setKSearch(30);
    norm_est->setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr pcl_out(new pcl::PointCloud<pcl::Normal>);
    norm_est->compute(*pcl_out);
    return pcl_out;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr get_feature(pcl::FPFHEstimationOMP<PointType, pcl::Normal, pcl::FPFHSignature33> &fpfh_est,
                                                       const PointCloudXYZ::Ptr &pcl_in,
                                                       pcl::PointCloud<pcl::Normal>::Ptr &normal,
                                                       pcl::search::KdTree<PointType>::Ptr &tree)
{
    fpfh_est.setInputCloud(pcl_in);
    fpfh_est.setInputNormals(normal);
    fpfh_est.setSearchMethod(tree);
    fpfh_est.setRadiusSearch(0.05);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr pcl_out(new pcl::PointCloud<pcl::FPFHSignature33>);
    fpfh_est.compute(*pcl_out);
    return pcl_out;
}

//---------------just for DEBUG----------------
std::vector<std::pair<int, int>> feature_match(pcl::PointCloud<pcl::FPFHSignature33>::Ptr &pcl_tgt,
                                               pcl::KdTreeFLANN<pcl::FPFHSignature33> &tgt_feature_tree,
                                               pcl::PointCloud<pcl::FPFHSignature33>::Ptr &pcl_src,
                                               float max_match_distance = 0.25)
{
    tgt_feature_tree.setInputCloud(pcl_tgt);
    std::vector<std::pair<int, int>> correspondences;
    correspondences.reserve(pcl_src->size());
#pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < pcl_src->size(); ++i)
    {
        std::vector<int> indices(1);
        std::vector<float> distances(1);

        if (tgt_feature_tree.nearestKSearch(*pcl_src, i, 1, indices, distances) > 0)
        {
            if (distances[0] <= max_match_distance)
            {
                correspondences.emplace_back(i, indices[0]);
            }
        }
    }
    if (correspondences.empty())
    {
        ROS_ERROR("no correspondence found!");
    }
    ROS_INFO("the number of correspondences is %zu", correspondences.size());
    return correspondences;
}
//---------------just for DEBUG----------------

PointCloudXYZ::Ptr ransac_icp_reg(const PointCloudXYZ::Ptr &pcl_tgt, const PointCloudXYZ::Ptr &pcl_src)
{
    // get normal
    pcl::NormalEstimationOMP<PointType, pcl::Normal>::Ptr norm_est(new pcl::NormalEstimationOMP<PointType, pcl::Normal>);
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    auto pcl_tgt_normal = get_normal(pcl_tgt, norm_est, tree);
    auto pcl_src_normal = get_normal(pcl_src, norm_est, tree);

    // get feature via fpfh
    pcl::FPFHEstimationOMP<PointType, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    auto pcl_tgt_featured = get_feature(fpfh_est, pcl_tgt, pcl_tgt_normal, tree);
    auto pcl_src_featured = get_feature(fpfh_est, pcl_src, pcl_src_normal, tree);

    // match and get correspondences of feature points
    // pcl::KdTreeFLANN<pcl::FPFHSignature33> tgt_feature_tree;
    // auto correspondences = feature_match(pcl_tgt_featured, tgt_feature_tree, pcl_src_featured);

    // ransac
    PointCloudXYZ::Ptr pcl_result_ransac(new PointCloudXYZ);

    pcl::SampleConsensusPrerejective<PointType, PointType, pcl::FPFHSignature33> ransac;
    ransac.setInputSource(pcl_src);
    ransac.setSourceFeatures(pcl_src_featured);
    ransac.setInputTarget(pcl_tgt);
    ransac.setTargetFeatures(pcl_tgt_featured);
    ransac.setMaximumIterations(1000);
    ransac.setNumberOfSamples(3);
    ransac.setCorrespondenceRandomness(5);
    ransac.setSimilarityThreshold(0.9);
    ransac.setInlierFraction(0.3);
    ransac.align(*pcl_result_ransac);

    if (!ransac.hasConverged())
    {
        ROS_ERROR("RANSAC failed to converge");
        return nullptr;
    }

    // icp
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setInputSource(pcl_result_ransac);
    icp.setInputTarget(pcl_tgt);
    icp.setMaximumIterations(20); // 减少迭代次数
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(0.01);

    PointCloudXYZ::Ptr pcl_result(new PointCloudXYZ);
    icp.align(*pcl_result);
    return pcl_result;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ransac_icp_pcl_node");
    pcl_match::Computer<PointType> computer;
    computer.compute(ransac_icp_reg);
    pcl_match::Visualizer<PointType> visualizer(computer);
    visualizer.visualize();
    return 0;
}