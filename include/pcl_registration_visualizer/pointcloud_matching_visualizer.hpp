#ifndef POINTCLOUD_MATCHING_VISUALIZER_HPP
#define POINTCLOUD_MATCHING_VISUALIZER_HPP

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/console/time.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace pcl_match
{
    template <typename PointT>
    class Visualizer;

    /***
     * @brief Computer Class, as a simple benchmark via custom algorithm
     * @tparam PointT point type. e.g. Computer<PointXYZ>
     */
    template <typename PointT>
    class Computer
    {
    public:
        using PointCloudT = pcl::PointCloud<PointT>;
        using PointCloudPtr = pcl::shared_ptr<PointCloudT>;
        using ComputeHandler = std::function<PointCloudPtr(const PointCloudPtr &, const PointCloudPtr &)>;

        struct UserConfig
        {
            std::string tgt_pcd_path;
            std::string src_pcd_path;
            bool no_priori_pcd; // do not use priori .pcd in the pcd folder, default: true
            bool keypoint_mode;
            int max_iter_time; // you MUST set this value if you want to visualize
            int point_num;
            struct Rotation
            {
                double deg;
                std::string axis;
            };
            struct Translation
            {
                double x;
                double y;
                double z;
            };
            Rotation rotation;
            Translation translation;
            int window_width;
            int window_height;
            int neighbor_num;
            int step_size;
            double curvature_threshold;
            double min_singular_threshold;
            ComputeHandler comp_handler;
        };

        Computer() : nh_("~"),
                     pcl_tgt(new PointCloudT),
                     pcl_src(new PointCloudT),
                     pcl_result(new PointCloudT)
        {
            // please do not use pcl::make_shared or std::make_shared to construct
            getConfig(nh_);
            // if use priori .pcd, then will execute code below
            if (!config_.no_priori_pcd)
            {
                ROS_INFO("use priori pcd");
                pcl_tgt = loadPCD(config_.tgt_pcd_path);
                pcl_src = loadPCD(config_.src_pcd_path);
            }
            else
            {
                ROS_INFO("No priori pcd, generate random pcd");
                generateRandomPCD();
            }
        }
        ~Computer() = default;

        void setComputeHandler(const ComputeHandler &handler);
        PointCloudPtr getKeypoint(const PointCloudPtr &pcl_in) const;
        bool hasConverged() const;
        float fitnessScore() const;
        void compute(const ComputeHandler &handler);

    private:
        ros::NodeHandle nh_;
        UserConfig config_;
        PointCloudPtr pcl_tgt;
        PointCloudPtr pcl_src;
        PointCloudPtr pcl_result;
        pcl::console::TicToc tictoc;
        int current_iter_time_ = 0;
        bool is_iter_finished_ = false;
        double current_registration_time_ = 0.0;

        void getConfig(ros::NodeHandle &nh);
        PointCloudPtr loadPCD(std::string &path);
        void generateRandomPCD();

        friend class Visualizer<PointT>;
    };

    /***
     * @brief Visualizer Class, visualize the comparation of two pointclouds and update the iteration result
     */
    template <typename PointT>
    class Visualizer
    {
    public:
        explicit Visualizer(Computer<PointT> &computer) : computer_(computer),
                                                          viewer_(new pcl::visualization::PCLVisualizer("Matching Visualizer"))
        {
        }
        ~Visualizer() = default;

        void visualize();

    private:
        Computer<PointT> &computer_;
        pcl::visualization::PCLVisualizer::Ptr viewer_;
    };

} // namespace pcl_match

#endif //! POINTCLOUD_MATCHING_VISUALIZER_HPP