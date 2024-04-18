#include "pclmatcher.h"

PCLMatcher::PCLMatcher(ros::NodeHandle& nh) : nh_(nh) {
    field_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_sub = nh.subscribe("/livox/lidar", 10, &PCLMatcher::cloudCallback, this);
    field_pub = nh.advertise<sensor_msgs::PointCloud2>("field_cloud", 1);
    field_pub_thread = std::thread(&PCLMatcher::fieldCloudPublisher, this);
    initial_pose_sub = nh.subscribe("/initialpose", 1, &PCLMatcher::initialPoseCallback, this);
    clickpoint_sub = nh_.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &PCLMatcher::clickedPointCallback, this);

    filtered_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // 定义旋转角度和旋转轴
    level_transform = Eigen::Affine3f::Identity();
    float theta = - M_PI / 12; // 假设传感器倾斜的角度
    level_transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));  // Y轴旋转

    // 添加Z轴的平移
    level_transform.translation() << 0.0, 0.0, -4.1;  // Z轴向上平移4单位


    adjusted_pub = nh.advertise<sensor_msgs::PointCloud2>("adjusted_cloud", 10);
    icpadjusted_pub = nh.advertise<sensor_msgs::PointCloud2>("icpadjusted_cloud", 10);
    obstaclecloud_pub = nh.advertise<sensor_msgs::PointCloud2>("obstacle_cloud",10);

    cumulative_transform = Eigen::Matrix4f::Identity();
    icp_thread = std::thread(&PCLMatcher::icp_run,this);

    ground_readyICP.reset(new pcl::PointCloud<pcl::PointXYZ>);
    ground_field.reset(new pcl::PointCloud<pcl::PointXYZ>);
    icp_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
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
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
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

void PCLMatcher::icpFunction(pcl::PointCloud<pcl::PointXYZ>::Ptr& sourcecloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& targetcloud, float transformationEpsilon, float distance, float euclideanFitnessEpsilon, int maximumIterations, bool useReciprocalCorrespondences, pcl::PointCloud<pcl::PointXYZ>::Ptr& aligncloud, Eigen::Matrix4f& final_transform)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(sourcecloud);
    icp.setInputTarget(targetcloud);
    icp.setTransformationEpsilon(transformationEpsilon);
    icp.setMaxCorrespondenceDistance(distance);
    icp.setEuclideanFitnessEpsilon(euclideanFitnessEpsilon);
    icp.setMaximumIterations(maximumIterations);
    icp.setUseReciprocalCorrespondences(useReciprocalCorrespondences);
    icp.align(*aligncloud);
    pcl::transformPointCloud(*sourcecloud, *sourcecloud, icp.getFinalTransformation());

    // 累加变换矩阵
    final_transform = icp.getFinalTransformation();

    auto end_time = std::chrono::high_resolution_clock::now();
    // 计算并打印 ICP 算法耗时（微秒）
    auto duration_milliseconds = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000;
    std::cout << "Duration in milliseconds: " << duration_milliseconds << " ms" << std::endl;
}


void PCLMatcher::icp_run()
{
    ros::Rate rate(10); // 设置发布频率，例如每秒10次
    while(ros::ok())
    {
        if(isreadyICP)
        {
            // 累加变换矩阵
            Eigen::Matrix4f final_transform;
            icpFunction(readyICP_cloud, field_cloud, 1e-10, 1, 0.001, 50, true, icp_cloud, final_transform);
            initial_alignment_transform = final_transform * initial_alignment_transform;

            std::cout << "icp_run Initial alignment transform matrix:" << std::endl;
            std::cout << initial_alignment_transform.matrix() << std::endl;
            
            isICPFinish = true;            
        }
        if(isICPFinish)
        {
            isreadyICP = false;
            sensor_msgs::PointCloud2 ros_adjusted_cloud;
            pcl::toROSMsg(*readyICP_cloud, ros_adjusted_cloud);
            ros_adjusted_cloud.header.frame_id = "livox_frame";
            ros_adjusted_cloud.header.stamp = ros::Time::now();
            icpadjusted_pub.publish(ros_adjusted_cloud);
            // std::cout<<"icpadjusted_pub had pub!!!!!!!!!"<<std::endl;
        } 
        rate.sleep();
    }
}

void PCLMatcher::upsampleVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float leaf_size, int points_per_voxel) 
{
    auto start_time = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*downsampled);

    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(-leaf_size / 2, leaf_size / 2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr upsampled(new pcl::PointCloud<pcl::PointXYZ>());

    // 对每个体素的中心点进行多次复制和随机偏移，但只有当点的 x 坐标大于 15m 时
    for (const auto& point : downsampled->points) {
        if (point.x > 15.0) { // 只有当 x 坐标大于 15 米时，才进行上采样
            for (int i = 0; i < points_per_voxel; ++i) {
                pcl::PointXYZ new_point;
                new_point.x = point.x + distribution(generator);
                new_point.y = point.y + distribution(generator);
                new_point.z = point.z + distribution(generator);
                upsampled->push_back(new_point);
            }
        } else {
            for (int i = 0; i < points_per_voxel / 2; ++i) {
                pcl::PointXYZ new_point;
                new_point.x = point.x + distribution(generator);
                new_point.y = point.y + distribution(generator);
                new_point.z = point.z + distribution(generator);
                upsampled->push_back(new_point);
            }
        }
    }

    cloud.swap(upsampled); // 更新原始点云
    auto end_time = std::chrono::high_resolution_clock::now();
    // 计算并打印上采样耗时（毫秒）
    auto duration_ms = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000;
    std::cout << "upsampleVoxelGrid: " << duration_ms << " ms" << std::endl;
}


void PCLMatcher::mergePointClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& plb, pcl::PointCloud<pcl::PointXYZ>::Ptr& pl) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (auto &cloud : plb) {
        *merged_cloud += *cloud; // 合并所有点云
    }
    // 更新current_source为合并后的点云
    pl = merged_cloud;
}

void PCLMatcher::removeOverlappingPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& live_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& field_cloud)
{
    // 计算两个点云之间的差异
    pcl::SegmentDifferences<pcl::PointXYZ> segment;
    segment.setInputCloud(live_cloud);
    segment.setTargetCloud(field_cloud);
    segment.setDistanceThreshold(0.1);  // 可根据实际情况调整

    pcl::PointCloud<pcl::PointXYZ>::Ptr dynamic_obstacles(new pcl::PointCloud<pcl::PointXYZ>());
    segment.segment(*dynamic_obstacles);

    // 在y轴上设置滤波范围
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(dynamic_obstacles);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-15, -0.6);
    pass_y.filter(*dynamic_obstacles);

    // 在x轴上设置滤波范围
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(dynamic_obstacles);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(0.0, 27.5);
    pass_x.filter(*dynamic_obstacles);

    // 在z轴上设置滤波范围
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(dynamic_obstacles);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.0, 1.4);
    pass_z.filter(*dynamic_obstacles);

    // 累加点云
    // upsampleVoxelGrid(dynamic_obstacles, 0.1, 1000); // 每个体素中生成10个点
    
    sensor_msgs::PointCloud2 ros_dynamic_obstacles;
    pcl::toROSMsg(*dynamic_obstacles, ros_dynamic_obstacles);
    ros_dynamic_obstacles.header.frame_id = "livox_frame";
    ros_dynamic_obstacles.header.stamp = ros::Time::now();
    obstaclecloud_pub.publish(ros_dynamic_obstacles);
}

void PCLMatcher::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_cloud, *source_cloud);

    if (!isLevel) {
    // tode 计算初始的水平角度
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_level_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    //     for (int i = 0; i < 10; ++i) {
    //         *accumulated_level_cloud += *source_cloud;
    //     }
    //    // RANSAC平面拟合来找到地面平面
    //     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    //     pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    //     pcl::SACSegmentation<pcl::PointXYZ> seg;
    //     seg.setOptimizeCoefficients(true);
    //     seg.setModelType(pcl::SACMODEL_PLANE);
    //     seg.setMethodType(pcl::SAC_RANSAC);
    //     seg.setDistanceThreshold(0.01);
    //     seg.setInputCloud(accumulated_level_cloud);
    //     seg.segment(*inliers, *coefficients);

    //     if (inliers->indices.size() == 0) {
    //         std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    //         // 这里应该有一个返回或者异常处理
    //     }

    //     // 提取平面法线(n_x, n_y, n_z)
    //     float nx = coefficients->values[0];
    //     float ny = coefficients->values[1];
    //     float nz = coefficients->values[2];

    //     // 计算地面平面法线与水平面之间的夹角
    //     float angle_to_horizontal = atan2(nz, ny);

    //     // 围绕y轴旋转
    //     level_transform.rotate(Eigen::AngleAxisf(-angle_to_horizontal, Eigen::Vector3f::UnitY()));

        isLevel = true;
    } else {
        // 应用旋转到原始点云
        pcl::transformPointCloud(*source_cloud, *source_cloud, level_transform.inverse());
    }

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


    // 累加点云，仅对x坐标大于特定值（例如15米）的点进行
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (auto& point : *source_cloud) {
        if (point.x > 15.0) { // 仅对远处的点进行累加
            for (int i = 0; i < 10; ++i) { // 已有一次，所以循环19次
                accumulated_cloud->push_back(point);
            }
        } else {
            accumulated_cloud->push_back(point); // 近处的点保持不变
        }
    }

    // 使用累加后的点云进行后续处理
    source_cloud.swap(accumulated_cloud);
  

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

        // std::cout << "cloudCallback Initial alignment transform matrix:" << std::endl;
        // std::cout << initial_alignment_transform.matrix() << std::endl;

        if (cloud_buffer.size() < 20) 
        {
            std::cout<<"cloud_buffer.size()"<<cloud_buffer.size()<<std::endl;
            cloud_buffer.push_back(source_cloud);
            if(cloud_buffer.size() == 19)
            {
                mergePointClouds(cloud_buffer, readyICP_cloud); // 合并点云
                isreadyICP = true;
            }
        }
    }

    if(isICPFinish)
    {
        //单帧
        removeOverlappingPoints(source_cloud, field_cloud);
    }
}

// 主函数在这里设置ROS环境和节点处理
int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_matcher_node");
    ros::NodeHandle nh;
    PCLMatcher matcher(nh);
    std::string pcd_file_path = "/home/hlc/code/RM_Radar2024/pclmatcher/file/pcd/HIT/RM2024v6.pcd";
    matcher.loadPCD(pcd_file_path);
    ros::spin();
    return 0;
}
