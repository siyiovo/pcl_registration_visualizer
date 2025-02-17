#include <omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <random>

#include "pcl_registration_visualizer/pointcloud_matching_visualizer.hpp"

/***
 * @brief init the custom registration algorithm handler
 * @param handler Compute Handler
 * @result result PointCloudPtr
 */
template <typename PointT>
void pcl_match::Computer<PointT>::setComputeHandler(const pcl_match::Computer<PointT>::ComputeHandler &handler)
{
    config_.comp_handler = handler;
}

/***
 * @brief get keypoint from input pointcloud
 * @param pcl_in input PointCloudPtr
 * @return keypoint PointCloudPtr
 * @note method refers to LOAM, which used curvature to get corner points and surface points. Here we need corner points.
 */
template <typename PointT>
typename pcl_match::Computer<PointT>::PointCloudPtr pcl_match::Computer<PointT>::getKeypoint(const PointCloudPtr &pcl_in) const
{
    /*
        declare parameters:
        corner points: points that have high curvature
        surface points: points that have low curvature
        nr_num: number of neighbor points, here is 10

        specified method:
        construct convariance matrix, get eigenvalue
        if proportion of maximum eigenvalue in sum of eigenvalue > threshold of curvature , then means the curvature is high

    */
    int nr_num = config_.neighbor_num;                        // the number of neighbor points
    int step_size = config_.step_size;                        // step size of searching neighbor points
    double curvature_threshold = config_.curvature_threshold; // threshold of curvature
    int keypoint_num;

    pcl::KdTreeFLANN<PointT> kd_tree;
    PointCloudPtr pcl_corner(new PointCloudT);
    pcl_corner->reserve(pcl_in->size() / 20); // assume the number of corner points is 1/20 of the total points
    kd_tree.setInputCloud(pcl_in);
#pragma omp parallel for shared(pcl_corner) schedule(dynamic)
    for (size_t i = 0; i < pcl_in->size(); i += step_size)
    {
        std::vector<int> indices(nr_num);
        std::vector<float> distances(nr_num);
        // find 10 nearest points
        if (kd_tree.nearestKSearch(pcl_in->points[i], nr_num, indices, distances) < nr_num)
            continue;
        // get convariance matrix
        Eigen::Matrix3d cov;
        Eigen::Vector4d centroid;
        pcl::compute3DCentroid(*pcl_in, centroid);
        pcl::computeCovarianceMatrixNormalized(*pcl_in, indices, centroid, cov);
        // compute eigenvalue
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(cov, Eigen::ComputeEigenvectors);
        Eigen::Vector3d eigen_val = eigen_solver.eigenvalues();
        if (eigen_val.sum() == 0)
        {
            ROS_ERROR("Eigen value is zero!");
            continue;
        }
        float curvature = eigen_val.maxCoeff() / eigen_val.sum();

        if (curvature > curvature_threshold)
        {
            pcl_corner->push_back(pcl_in->points[i]);
            keypoint_num++;
        }
    }
    if (pcl_corner->points.size() < 100)
    {
        ROS_WARN("keypoint size %d is too small!", keypoint_num);
    }
    return pcl_corner;
}

/***
 * @brief judge whether the matching has converged via convariance matrix cov(pcl_result,pcl_tgt)
 * @note decompose the convariance matrix via SVD, another method is like ISS
 */
template <typename PointT>
bool pcl_match::Computer<PointT>::hasConverged() const
{
    if (!pcl_result || !pcl_tgt || pcl_result->empty() || pcl_tgt->empty())
    {
        throw std::runtime_error("pcl_result or pcl_tgt is empty!");
    }
    if (pcl_result->size() > 30000 || pcl_tgt->size() > 30000)
    {
        ROS_INFO("pcl_result or pcl_tgt size is too large!");
        if (config_.keypoint_mode)
        {
            ROS_INFO("start getting keypoint ...");
            auto pcl_result_keypoint = getKeypoint(pcl_result);
            auto pcl_tgt_keypoint = getKeypoint(pcl_tgt);
            pcl::copyPointCloud(*pcl_result_keypoint, *pcl_result);
            ROS_INFO("keypoint of pcl_result: %zu", pcl_result_keypoint->size());
            pcl::copyPointCloud(*pcl_tgt_keypoint, *pcl_tgt);
            ROS_INFO("keypoint of pcl_result: %zu", pcl_tgt_keypoint->size());
            ROS_INFO("now pcl_result and pcl_tgt have been replaced with respective keypoint pointcloud!");
        }
    }
    if (pcl_result->size() != pcl_tgt->size())
    {
        ROS_WARN("pcl_result and pcl_tgt size are not equal!");
        int bigger = std::max(pcl_result->size(), pcl_tgt->size());
        if (pcl_result->size() == bigger)
            pcl_result->resize(pcl_tgt->size());
        else
            pcl_tgt->resize(pcl_result->size());
    }
    /*
        NOTE that a point include x,y,z, so convariance matrix is a 3x3 matrix
        pcl_result[i] abbr. pr[i], pcl_tgt[i] abbr. pt[i]. define X[i] = pr[i]-result_mean, Y[i] = pt[i]-tgt_mean
        cov(pcl_result, pcl_tgt) = cov(X,Y) = X_transpose * Y
    */
    Eigen::Matrix3d result_cov = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d tgt_cov = Eigen::Matrix3d::Zero();
    Eigen::Vector4d result_centroid = Eigen::Vector4d::Zero();
    Eigen::Vector4d tgt_centroid = Eigen::Vector4d::Zero();
    // get centroid
    pcl::compute3DCentroid(*pcl_result, result_centroid);
    pcl::compute3DCentroid(*pcl_tgt, tgt_centroid);
    //  DON'T forget normalization
    pcl::computeCovarianceMatrixNormalized(*pcl_result, result_centroid, result_cov);
    pcl::computeCovarianceMatrixNormalized(*pcl_tgt, tgt_centroid, tgt_cov);
    Eigen::Matrix3d cov = result_cov.transpose() * tgt_cov;

    // decompose the convariance matrix via SVD and get singular matrix
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // the vector has been sorted in decreasing order
    Eigen::Vector3d sigular_vec = svd.singularValues();

    double min_sigular = sigular_vec.minCoeff();
    if (min_sigular > config_.min_singular_threshold)
    {
        ROS_DEBUG_STREAM("min sigular value: " << min_sigular);
        return true;
    }
    else
    {
        ROS_WARN("has not converged!");
        return false;
    }
}

/***
 * @brief get the fitness score of the matching result via KNN
 * @note method refers to pcl::Registration<PointType, PointType, float>::getFitnessScore
 */
template <typename PointT>
float pcl_match::Computer<PointT>::fitnessScore() const
{
    // NOTE that we regard pcl_tgt as target pointcloud , and pcl_result is as query pointcloud(source pointcloud here)
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(pcl_tgt);

    const double max_range = 10.0;
    float total_distance = 0.0;
    int nr_point_num = 0;

    if (!pcl_result || pcl_result->empty())
    {
        ROS_ERROR("pcl_result is empty!");
        return -1.0;
    }

    std::vector<int> indices(1);
    std::vector<float> distances(1);

    for (const auto &pt : pcl_result->points)
    {
        // find the nearest point
        kdtree.nearestKSearch(pt, 1, indices, distances);
        if (distances[0] <= max_range)
        {
            total_distance += std::sqrt(distances[0]);
            nr_point_num++;
        }
    }
    if (nr_point_num == 0)
    {
        ROS_ERROR("no neighbor point!");
        return -1.0;
    }
    float score = total_distance / static_cast<float>(nr_point_num);
    return score;
}

/***
 * @brief compute registration result via custom algorithm
 */
template <typename PointT>
void pcl_match::Computer<PointT>::compute(const pcl_match::Computer<PointT>::ComputeHandler &handler)
{
    if (!handler)
        ROS_ERROR("compute handler is not set!");
    if (current_iter_time_ == 0)
    {
        setComputeHandler(handler);
    }
    try
    {
        tictoc.tic();
        pcl_result = config_.comp_handler(pcl_tgt, pcl_src);
        current_iter_time_++;
        current_registration_time_ = tictoc.toc();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

/***
 * @brief get param from config/xxx.yaml
 */
template <typename PointT>
void pcl_match::Computer<PointT>::getConfig(ros::NodeHandle &nh)
{
    nh.param<std::string>("tgt_pcd_path", config_.tgt_pcd_path, std::string(""));
    nh.param<std::string>("src_pcd_path", config_.src_pcd_path, std::string(""));
    nh.param<bool>("no_priori_pcd", config_.no_priori_pcd, true);
    nh.param<bool>("keypoint_mode", config_.keypoint_mode, false);
    nh.param<int>("max_iter_time", config_.max_iter_time, 0);
    nh.param<int>("generate_pcd/point_num", config_.point_num, 3000);
    nh.param<double>("generate_pcd/rotation/deg", config_.rotation.deg, 0.0);
    nh.param<std::string>("generate_pcd/rotation/axis", config_.rotation.axis, std::string("z"));
    nh.param<double>("generate_pcd/translation/x", config_.translation.x, 10.0);
    nh.param<double>("generate_pcd/translation/y", config_.translation.y, 10.0);
    nh.param<double>("generate_pcd/translation/z", config_.translation.z, 10.0);
    nh.param<int>("window_width", config_.window_width, 1280);
    nh.param<int>("window_height", config_.window_height, 720);
    nh.param<int>("neighbor_num", config_.neighbor_num, 10);
    nh.param<int>("step_size", config_.step_size, 10);
    nh.param<double>("curvature_threshold", config_.curvature_threshold, 0.10);
    nh.param<double>("min_singular_threshold", config_.min_singular_threshold, 1e-5);
}

/***
 * @brief load pointcloud from .pcd
 * @param path .pcd path
 * @result loaded PointCloudPtr
 */
template <typename PointT>
typename pcl_match::Computer<PointT>::PointCloudPtr pcl_match::Computer<PointT>::loadPCD(std::string &path)
{
    PointCloudPtr pcl_in(new PointCloudT);
    if (pcl::io::loadPCDFile<PointT>(path, *pcl_in) == -1)
    {
        throw std::runtime_error("Failed to load PCD: " + path);
    }
    return pcl_in;
}

/***
 * @brief generate random pointcloud for test custom registration algorithm
 * @note it works ONLY when no_priori_pcd is true
 */
template <typename PointT>
void pcl_match::Computer<PointT>::generateRandomPCD()
{
    // init random generator
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<double> dist(0.0, 1.0);

    pcl_tgt->width = config_.point_num;
    pcl_tgt->height = 1;
    pcl_tgt->is_dense = false;
    pcl_tgt->resize(config_.point_num);
    // set random point into target pointcloud via <random>
    for (auto &pt : pcl_tgt->points)
    {
        pt.x = dist(gen);
        pt.y = dist(gen);
        pt.z = dist(gen);
        ROS_DEBUG_STREAM(pt.x << " " << pt.y << " " << pt.z);
    }
    *pcl_src = *pcl_tgt;
    // set a simple RIGID transformation to source pointcloud
    Eigen::Isometry3d trans = Eigen::Isometry3d::Identity();
    double radian = config_.rotation.deg * M_PI / static_cast<double>(180.0);
    trans.rotate(Eigen::AngleAxisd(radian, [&]() -> Eigen::Vector3d
                                   {
                                       if (config_.rotation.axis == "x")
                                           return Eigen::Vector3d::UnitX();
                                       else if (config_.rotation.axis == "y")
                                           return Eigen::Vector3d::UnitY();
                                       else
                                           return Eigen::Vector3d::UnitZ(); }()));
    trans.translate(Eigen::Vector3d(static_cast<double>(config_.translation.x),
                                    static_cast<double>(config_.translation.y),
                                    static_cast<double>(config_.translation.z)));
    pcl::transformPointCloud(*pcl_src, *pcl_src, trans.matrix());
    ROS_INFO("pcl_src point num: %zu", pcl_src->size());
}

/***
 * @brief visualize the result of registration
 */
template <typename PointT>
void pcl_match::Visualizer<PointT>::visualize()
{
    if (!computer_.pcl_result || !computer_.pcl_tgt || !computer_.pcl_src || computer_.pcl_result->empty() || computer_.pcl_tgt->empty() || computer_.pcl_src->empty())
    {
        PCL_ERROR("no pointcloud!");
        return;
    }
    /*---------init the visualizer---------*/
    viewer_->setBackgroundColor(0, 0, 0);
    // showed 2 windows
    int win_1(0), win_2(1);
    viewer_->createViewPort(0.0, 0.0, 0.5, 1.0, win_1);
    viewer_->createViewPort(0.5, 0.0, 1.0, 1.0, win_2);
    // set window 1
    // set target pointcloud white
    pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_color_handler(computer_.pcl_tgt, 255, 255, 255);
    viewer_->addPointCloud(computer_.pcl_tgt, tgt_color_handler, "pcl_tgt_1", win_1);
    // set source pointcloud red
    pcl::visualization::PointCloudColorHandlerCustom<PointT> src_color_handler(computer_.pcl_src, 255, 0, 0);
    viewer_->addPointCloud(computer_.pcl_src, src_color_handler, "pcl_src", win_1);

    // add help
    viewer_->addText("White: target pointcloud\nRed: source pointcloud\nGreen: computed pointcloud", 6, 15, 12, 1, 1, 1, "help", win_1);

    // set window 2
    // set result pointcloud green
    pcl::visualization::PointCloudColorHandlerCustom<PointT> result_color_handler(computer_.pcl_result, 0, 255, 0);
    viewer_->addPointCloud(computer_.pcl_tgt, tgt_color_handler, "pcl_tgt_2", win_2);
    viewer_->addPointCloud(computer_.pcl_result, result_color_handler, "pcl_result", win_2);

    // optimize the visualization of pointcloud
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pcl_tgt_1", win_1);
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pcl_src", win_1);
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pcl_tgt_2", win_2);
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pcl_result", win_2);

    // add registration info
    std::stringstream ss;
    ss << "Current Iteration: " << computer_.current_iter_time_ << " / " << computer_.config_.max_iter_time << "\n"
       << "Has Converged: " << (computer_.hasConverged() ? "Yes" : "No") << "\n"
       << "Fitness Score: " << computer_.fitnessScore() << "\n"
       << "Registration Time: " << computer_.current_registration_time_;
    viewer_->addText(ss.str(), 15, 21, 12, 1, 1, 1, "iteration_info", win_2);

    // get from param
    viewer_->setSize(computer_.config_.window_width, computer_.config_.window_height);

    // add update registration callback
    viewer_->registerKeyboardCallback(
        [&](const pcl::visualization::KeyboardEvent &event)
        {
            if (event.getKeySym() == "n" && event.keyDown())
            {
                if (computer_.current_iter_time_ < computer_.config_.max_iter_time &&
                    !computer_.is_iter_finished_)
                {
                    // show last pcl_src
                    viewer_->updatePointCloud(computer_.pcl_src, src_color_handler, "pcl_src");
                    // update pcl_src
                    computer_.tictoc.tic();
                    pcl::copyPointCloud(*(computer_.config_.comp_handler(computer_.pcl_tgt, computer_.pcl_src)), *computer_.pcl_result);
                    computer_.current_registration_time_ = computer_.tictoc.toc();
                    // update pcl_result
                    pcl::copyPointCloud(*computer_.pcl_result, *computer_.pcl_src);
                    viewer_->updatePointCloud(computer_.pcl_result, result_color_handler, "pcl_result");
                    computer_.current_iter_time_++;
                    std::stringstream ss_nxt;
                    ss_nxt << "Current Iteration: " << computer_.current_iter_time_ << " / " << computer_.config_.max_iter_time << "\n"
                           << "Has Converged: " << (computer_.hasConverged() ? "Yes" : "No") << "\n"
                           << "Fitness Score: " << computer_.fitnessScore() << "\n"
                           << "Registration Time: " << computer_.current_registration_time_;
                    viewer_->updateText(ss_nxt.str(), 15, 21, 12, 1, 1, 1, "iteration_info");
                }
                else
                {
                    computer_.is_iter_finished_ = true;
                    ROS_WARN("up to max iteration time!");
                }
            }
        });

    while (!viewer_->wasStopped())
    {
        viewer_->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

template class pcl_match::Computer<pcl::PointXYZ>;
template class pcl_match::Visualizer<pcl::PointXYZ>;
template class pcl_match::Computer<pcl::PointXYZINormal>;
template class pcl_match::Visualizer<pcl::PointXYZINormal>;