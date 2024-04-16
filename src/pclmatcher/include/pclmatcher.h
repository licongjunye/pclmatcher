#ifndef PCLMATCHER_H
#define PCLMATCHER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sstream>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <thread>
#include <iostream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>
#include <geometry_msgs/PointStamped.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/segmentation/segment_differences.h>



class PCLMatcher {
public:
    PCLMatcher(ros::NodeHandle& nh);
    ~PCLMatcher();

    void loadPCD(const std::string& file_path);
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud);
    void fieldCloudPublisher();
    void icp_run();
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void computeInitialAlignment();
    void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void removeOverlappingPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& live_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& field_cloud);



private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub;
    ros::Publisher field_pub;  // 发布加载的 PCD 点云
    ros::Publisher final_pub;   // 发布匹配后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr field_cloud;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    sensor_msgs::PointCloud2 ros_field_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;  // 过滤后的点云
    Eigen::Affine3f transform; // 用于将点云调整到水平位置的变换矩阵
    ros::Publisher adjusted_pub;  // 发布调整后的点云
    ros::Publisher icpadjusted_pub;
    std::thread field_pub_thread;
    pcl::PointCloud<pcl::PointXYZ>::Ptr readyICP_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud;;

    
    Eigen::Matrix4f cumulative_transform; // 初始化累积变换矩阵
    std::thread icp_thread;
    bool isreadyICP = false;
    bool isICPFinish = false;
    ros::Subscriber initial_pose_sub;
    Eigen::Affine3f initial_pose; // 存储初始位姿的变换

    const int max_icp_iterations = 5; // 设置最大的ICP迭代次数
    const double fitness_score_threshold = 0.01; // 设置适应度得分阈值

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_buffer; // 用于存储点云的缓存
    void mergePointClouds(); // 合并点云的函数

    std::vector<Eigen::Vector3f> map_feature_points; // 先验地图的特征点
    std::vector<Eigen::Vector3f> sensor_feature_points; // 传感器的特征点
    Eigen::Matrix4f initial_alignment_transform;
    ros::Subscriber clickpoint_sub;
    bool isINITFinish = false;

    ros::Publisher obstaclecloud_pub;

};

#endif // PCLMATCHER_H
