#include "pclmatcher.h"

PCLMatcher::PCLMatcher(ros::NodeHandle& nh) : nh_(nh) {
    field_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_sub = nh.subscribe("/livox/lidar", 10, &PCLMatcher::cloudCallback, this);
    field_pub = nh.advertise<sensor_msgs::PointCloud2>("field_cloud", 1);
    field_pub_thread = std::thread(&PCLMatcher::fieldCloudPublisher, this);
    final_pub = nh.advertise<sensor_msgs::PointCloud2>("final_cloud", 1);
    initial_pose_sub = nh.subscribe("/initialpose", 1, &PCLMatcher::initialPoseCallback, this);
    clickpoint_sub = nh_.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &PCLMatcher::clickedPointCallback, this);

    filtered_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // 定义旋转角度和旋转轴
    transform = Eigen::Affine3f::Identity();
    float theta = - M_PI / 12; // 假设传感器倾斜的角度
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));  // Y轴旋转

    // 添加Z轴的平移
    transform.translation() << 0.0, 0.0, -4;  // Z轴向上平移4单位


    adjusted_pub = nh.advertise<sensor_msgs::PointCloud2>("adjusted_cloud", 10);
    icpadjusted_pub = nh.advertise<sensor_msgs::PointCloud2>("icpadjusted_cloud", 10);
    obstaclecloud_pub = nh.advertise<sensor_msgs::PointCloud2>("obstacle_cloud",10);

    cumulative_transform = Eigen::Matrix4f::Identity();
    icp_thread = std::thread(&PCLMatcher::icp_run,this);
}

PCLMatcher::~PCLMatcher() {
    if (field_pub_thread.joinable() || icp_thread.joinable()) {
        field_pub_thread.join();
        icp_thread.join();
    }
}


void PCLMatcher::clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    std::cout<<"x:"<<msg->point.x<<"y:"<<msg->point.y<<"z:"<<msg->point.z<<std::endl;
    Eigen::Vector3f point(msg->point.x, msg->point.y, msg->point.z);
    if(map_feature_points.size() < 4) {
        map_feature_points.push_back(point);
        ROS_INFO("Map feature point %ld added: [%f, %f, %f]", 
                 map_feature_points.size(), point.x(), point.y(), point.z());
    } else if(sensor_feature_points.size() < 4) {
        sensor_feature_points.push_back(point);
        ROS_INFO("Sensor feature point %ld added: [%f, %f, %f]", 
                 sensor_feature_points.size(), point.x(), point.y(), point.z());
    }

    if (map_feature_points.size() == 4 && sensor_feature_points.size() == 4) {
        computeInitialAlignment();
    }
}

void PCLMatcher::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) 
{
    Eigen::Vector3f point(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    if(map_feature_points.size() < 4) {
        map_feature_points.push_back(point);
        ROS_INFO("Map feature point %ld added: [%f, %f, %f]", 
                 map_feature_points.size(), point.x(), point.y(), point.z());
    } else if(sensor_feature_points.size() < 4) {
        sensor_feature_points.push_back(point);
        ROS_INFO("Sensor feature point %ld added: [%f, %f, %f]", 
                 sensor_feature_points.size(), point.x(), point.y(), point.z());
    }

    if (map_feature_points.size() == 4 && sensor_feature_points.size() == 4) {
        computeInitialAlignment();
    }
}

void PCLMatcher::computeInitialAlignment() {
    pcl::PointCloud<pcl::PointXYZ> map_points;
    pcl::PointCloud<pcl::PointXYZ> sensor_points;

    for (size_t i = 0; i < map_feature_points.size(); ++i) {
        map_points.push_back(pcl::PointXYZ(map_feature_points[i].x(), map_feature_points[i].y(), map_feature_points[i].z()));
        sensor_points.push_back(pcl::PointXYZ(sensor_feature_points[i].x(), sensor_feature_points[i].y(), sensor_feature_points[i].z()));
    }

    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
    svd.estimateRigidTransformation(sensor_points, map_points, initial_alignment_transform);

    ROS_INFO_STREAM("Initial alignment transform estimated:\n" << initial_alignment_transform);
  
    isINITFinish = true;

}

void PCLMatcher::loadPCD(const std::string& file_path) {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *field_cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", file_path.c_str());
        return;
    }
    ROS_INFO("Loaded %zu data points from %s", field_cloud->points.size(), file_path.c_str());
    // 应用体素滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(field_cloud);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.filter(*temp_cloud);

    field_cloud.swap(temp_cloud);
    
    pcl::toROSMsg(*field_cloud, ros_field_cloud);
}

void PCLMatcher::fieldCloudPublisher() {
    ros::Rate rate(10); // 设置发布频率，例如每秒10次
    while (ros::ok()) 
    {
        ros_field_cloud.header.frame_id = "livox_frame";
        ros_field_cloud.header.stamp = ros::Time::now();
        field_pub.publish(ros_field_cloud);
        rate.sleep();
    }
}

void PCLMatcher::icp_run()
{
    ros::Rate rate(10); // 设置发布频率，例如每秒10次
    while(ros::ok())
    {
        if(isreadyICP)
        {
            icp_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>(*readyICP_cloud));
            for (int i = 0; i < max_icp_iterations; ++i) 
            {
                // 更新源点云
                icp.setInputSource(icp_cloud);
                icp.setInputTarget(field_cloud);
                
                pcl::PointCloud<pcl::PointXYZ> TempCloud;
                icp.align(TempCloud);

                if (icp.hasConverged()) {
                    // 累积变换
                    Eigen::Matrix4f step_transform = icp.getFinalTransformation();
                    cumulative_transform = step_transform * cumulative_transform;
                    
                    // 使用新的变换更新current_source
                    pcl::transformPointCloud(*icp_cloud, *icp_cloud, cumulative_transform);

                    ROS_INFO("ICP iteration %d, score is %f", i, icp.getFitnessScore());

                    if (icp.getFitnessScore() < fitness_score_threshold) {
                        ROS_INFO("ICP fitness score is low enough, finishing ICP.");
                        break; // 如果适应度得分低于阈值，结束迭代
                    }
                } else {
                    ROS_INFO("ICP did not converge.");
                    break; // 如果ICP没有收敛，结束迭代
                }
                // 使用累积变换生成最终的点云
                sensor_msgs::PointCloud2 ros_adjusted_cloud;
                pcl::toROSMsg(*icp_cloud, ros_adjusted_cloud);
                ros_adjusted_cloud.header.frame_id = "livox_frame";
                ros_adjusted_cloud.header.stamp = ros::Time::now();
                icpadjusted_pub.publish(ros_adjusted_cloud);
                std::cout<<"icpadjusted_pub had pub!!!!!!!!!"<<std::endl;
            }
            isICPFinish = true;
        }
        if(isICPFinish)
        {
            isreadyICP = false;
            sensor_msgs::PointCloud2 ros_adjusted_cloud;
            pcl::toROSMsg(*icp_cloud, ros_adjusted_cloud);
            ros_adjusted_cloud.header.frame_id = "livox_frame";
            ros_adjusted_cloud.header.stamp = ros::Time::now();
            icpadjusted_pub.publish(ros_adjusted_cloud);
            std::cout<<"icpadjusted_pub had pub!!!!!!!!!"<<std::endl;
        } 
        rate.sleep();
    }
}

void PCLMatcher::mergePointClouds() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (auto &cloud : cloud_buffer) {
        *merged_cloud += *cloud; // 合并所有点云
    }
    // 更新current_source为合并后的点云
    readyICP_cloud = merged_cloud;
    isreadyICP = true;
}

void PCLMatcher::removeOverlappingPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& live_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& field_cloud)
{
   // 对两个点云进行体素滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr vox_source(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr vox_field(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.1, 0.1, 0.1);

    approximate_voxel_filter.setInputCloud(live_cloud);
    approximate_voxel_filter.filter(*vox_source);

    approximate_voxel_filter.setInputCloud(field_cloud);
    approximate_voxel_filter.filter(*vox_field);

    // 计算两个点云之间的差异
    pcl::SegmentDifferences<pcl::PointXYZ> segment;
    segment.setInputCloud(vox_source);
    segment.setTargetCloud(vox_field);
    segment.setDistanceThreshold(0.1);  // 可根据实际情况调整

    pcl::PointCloud<pcl::PointXYZ>::Ptr dynamic_obstacles(new pcl::PointCloud<pcl::PointXYZ>());
    segment.segment(*dynamic_obstacles);

    // dynamic_obstacles 现在包含所有的动态障碍物点
    // 接下来你可以将这个点云发布出去
    sensor_msgs::PointCloud2 ros_dynamic_obstacles;
    pcl::toROSMsg(*dynamic_obstacles, ros_dynamic_obstacles);
    ros_dynamic_obstacles.header.frame_id = "livox_frame";
    ros_dynamic_obstacles.header.stamp = ros::Time::now();
    obstaclecloud_pub.publish(ros_dynamic_obstacles);
}

void PCLMatcher::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_cloud, *source_cloud);
    pcl::transformPointCloud(*source_cloud, *source_cloud, transform.inverse());

    // 在x轴上设置滤波范围
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(source_cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(5.0, 32.0);
    pass_x.filter(*source_cloud);

    // 在y轴上设置滤波范围
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(source_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-8.0, 8.0);
    pass_y.filter(*source_cloud);

    //在z轴上设置过滤范围
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(source_cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(-10, 3);
    pass_z.filter(*source_cloud);
  

    if(!isINITFinish)
    {
        // 发布调整后的点云
        sensor_msgs::PointCloud2 ros_adjusted_cloud;
        pcl::toROSMsg(*source_cloud, ros_adjusted_cloud);
        ros_adjusted_cloud.header.frame_id = "livox_frame";
        ros_adjusted_cloud.header.stamp = ros::Time::now();
        adjusted_pub.publish(ros_adjusted_cloud);
    }else{
        pcl::transformPointCloud(*source_cloud, *source_cloud, initial_alignment_transform);
        sensor_msgs::PointCloud2 ros_adjusted_cloud;
        pcl::toROSMsg(*source_cloud, ros_adjusted_cloud);
        ros_adjusted_cloud.header.frame_id = "livox_frame";
        ros_adjusted_cloud.header.stamp = ros::Time::now();
        adjusted_pub.publish(ros_adjusted_cloud);

        if (cloud_buffer.size() < 5) 
        {
            std::cout<<"cloud_buffer.size()"<<cloud_buffer.size()<<std::endl;
            cloud_buffer.push_back(source_cloud);
            if(cloud_buffer.size() == 4)
            {
                mergePointClouds(); // 合并点云
            }
        }
    }

    if(isICPFinish)
    {
        removeOverlappingPoints(source_cloud, field_cloud);
    }
}

// 主函数在这里设置ROS环境和节点处理
int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_matcher_node");
    ros::NodeHandle nh;
    PCLMatcher matcher(nh);
    std::string pcd_file_path = "/home/hlc/code/RM_Radar2024/pclmatcher/file/pcd/HIT/RMUC2024.pcd";
    matcher.loadPCD(pcd_file_path);
    ros::spin();
    return 0;
}
