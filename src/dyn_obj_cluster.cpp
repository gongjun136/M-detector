/*
 * @Author: gongjun136 gongjun136@gmail.com
 * @Date: 2024-03-12 10:14:37
 * @LastEditors: gongjun136 gongjun136@gmail.com
 * @LastEditTime: 2024-03-21 16:33:21
 * @FilePath: /catkin_ws_M-detector/src/M-detector-noted/src/dyn_obj_cluster.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "m-detector-noted/dyn_obj_cluster.h"
#include "cluster_predict/EA_disk.h"

#include <algorithm>
#include <chrono>
#include <execution>

void DynObjCluster::Init()
{
    // pub_pcl_dyn_extend = pub_pcl_dyn_extend_in;
    // cluster_vis_high = cluster_vis_high_in;
    // pub_ground_points = pub_ground_points_in;
    // xyz_origin << -20., -20., -20.;
    // maprange << 40., 40., 40.;

    // 设置xyz坐标的起始点，这里从 -100 开始，意味着考虑的空间范围相对于某个中心点
    xyz_origin << -100., -100., -20.;
    // 设置xyz坐标的范围，这里设置为 200x200x40，这是考虑的空间体积
    maprange << 200., 200., 40.;
    // 根据体素的分辨率计算出x，y方向上的网格数
    GridMapedgesize_xy = ceil(maprange(0) / Voxel_revolusion);
    // 计算z方向上的网格数
    GridMapedgesize_z = ceil(maprange(2) / Voxel_revolusion);
    // 计算整个网格的大小，即网格的总数
    GridMapsize = GridMapedgesize_xy * GridMapedgesize_xy * GridMapedgesize_z;
    // 输出初始化的一些信息，包括网格的大小
    std::cout << "clustering init begin, please wait------------" << GridMapsize << std::endl;
    // 预留足够的空间以提高后续操作的效率
    umap.reserve(GridMapsize);
    // 调整大小以匹配网格大小
    umap.resize(GridMapsize);
    umap_ground.reserve(GridMapsize);
    umap_ground.resize(GridMapsize);
    umap_insidebox.reserve(GridMapsize);
    umap_insidebox.resize(GridMapsize);
    // 完成初始化后的提示信息
    std::cout << "clustering init finish------------" << std::endl;
    // 如果指定了输出文件，准备将结果输出到文件
    if (out_file != "")
        out.open(out_file, std::ios::out | std::ios::binary);
}

/**
 * @brief 对给定的点云执行聚类和跟踪处理，更新动态标签。
 *
 * @param dyn_tag 动态点标签的向量，标记每个点是否为动态。
 * @param event_point 输入的事件点云，经过预处理后判断动态点
 * @param raw_point 原始点云数据，用于参考和进一步处理。
 * @param header_in 点云的消息头，包含时间戳和帧ID，用于保持时间一致性。
 * @param odom_rot_in 当前帧相对于起始帧的旋转矩阵，用于坐标变换。
 * @param odom_pos_in 当前帧相对于起始帧的位置向量，用于坐标变换。
 */
void DynObjCluster::Clusterprocess(std::vector<int> &dyn_tag, pcl::PointCloud<PointType> event_point, const pcl::PointCloud<PointType> &raw_point, const std_msgs::Header &header_in, const Eigen::Matrix3d odom_rot_in, const Eigen::Vector3d odom_pos_in)
{
    // 记录聚类开始的时间
    cluster_begin = ros::Time::now();

    // 初始化本次处理的消息头和位姿信息
    header = header_in;
    odom_rot = odom_rot_in;
    odom_pos = odom_pos_in;

    // 开始计时
    ros::Time t0 = ros::Time::now();
    // 设置时间间隔，用于聚类和跟踪算法中
    float delta_t = 0.1;
    // 初始化扩展点云
    pcl::PointCloud<PointType> extend_points;
    // 创建清洗后的点云指针
    pcl::PointCloud<PointType>::Ptr cloud_clean_ptr(new pcl::PointCloud<PointType>);
    cloud_clean_ptr = event_point.makeShared(); // 从事件点云创建共享指针

    // 初始化高置信度边界框
    bbox_t bbox_high;
    // 调用ClusterAndTrack函数处理点云，进行聚类和跟踪
    ClusterAndTrack(dyn_tag, cloud_clean_ptr, pub_pcl_before_high, header, pub_pcl_after_high, cluster_vis_high, predict_path_high, bbox_high, delta_t, raw_point);
    // 计算并更新本次处理的总时间
    time_total = (ros::Time::now() - t0).toSec();
    time_ind++; // 更新时间统计的计数
    // 更新平均处理时间
    time_total_average = time_total_average * (time_ind - 1) / time_ind + time_total / time_ind;
    cur_frame += 1;
}

/**
 * @brief 对点云执行聚类和跟踪处理，并发布处理结果。
 *
 * @param dyn_tag 动态点标签的向量，标记每个点是否为动态。
 * @param points_in 输入的点云，通常是经过预处理的，包含可能的动态点
 * @param points_in_msg 用于发布输入点云的ROS发布器。
 * @param header_in 输入点云的消息头，包含时间戳和帧ID。
 * @param points_out_msg 用于发布处理后点云的ROS发布器。
 * @param cluster_vis 用于发布聚类可视化结果的ROS发布器。
 * @param predict_path 用于发布预测路径的ROS发布器。
 * @param bbox 存储边界框信息的变量，用于聚类结果的可视化等。
 * @param delta 时间间隔，用于跟踪算法中。
 * @param raw_point 原始点云数据，用于参考和进一步处理。
 */
void DynObjCluster::ClusterAndTrack(std::vector<int> &dyn_tag,
                                    pcl::PointCloud<PointType>::Ptr &points_in,
                                    ros::Publisher points_in_msg,
                                    std_msgs::Header header_in,
                                    ros::Publisher points_out_msg,
                                    ros::Publisher cluster_vis,
                                    ros::Publisher predict_path,
                                    bbox_t &bbox, double delta,
                                    const pcl::PointCloud<PointType> &raw_point)
{
    // 将点云转换为ROS消息格式，并设置消息头信息
    sensor_msgs::PointCloud2 pcl4_ros_msg;
    pcl::toROSMsg(*points_in, pcl4_ros_msg);
    pcl4_ros_msg.header.stamp = header_in.stamp;
    pcl4_ros_msg.header.frame_id = header_in.frame_id;

    // 初始化聚类结果变量
    std::vector<pcl::PointIndices> cluster_indices; // 存储每个聚类的索引
    std::vector<std::vector<int>> voxel_clusters;   // 存储体素聚类的结果

    // 记录聚类开始的时间
    ros::Time t0 = ros::Time::now();

    // 用于记录已使用的体素映射
    std::unordered_set<int> used_map_set;

    // 调用GetClusterResult_voxel获取聚类结果
    GetClusterResult_voxel(points_in, umap, voxel_clusters, used_map_set);
    // 调用PubClusterResult_voxel发布聚类结果
    PubClusterResult_voxel(dyn_tag, header_in, bbox, delta, voxel_clusters, raw_point, used_map_set);
}

// void DynObjCluster::GetClusterResult(pcl::PointCloud<PointType>::Ptr points_in, std::vector<pcl::PointIndices> &cluster_indices)
// {
//     if (points_in->size() < 2)
//     {
//         return;
//     }
//     pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
//     tree->setInputCloud(points_in);
//     DBSCANKdtreeCluster<PointType> ec;
//     ec.setCorePointMinPts(nn_points_size);
//     ec.setClusterTolerance(nn_points_radius);
//     ec.setMinClusterSize(min_cluster_size);
//     ec.setMaxClusterSize(max_cluster_size);
//     ec.setSearchMethod(tree);
//     ec.setInputCloud(points_in);
//     ros::Time t0 = ros::Time::now();
//     ec.extract(cluster_indices);
// }

/**
 * @brief 对输入的点云进行体素化聚类处理。
 *
 * @param points_in 输入的点云，通常是经过预处理的，包含可能的动态点。
 * @param umap_in 体素映射的容器，用于存储每个体素内的点云数据。
 * @param voxel_clusters 聚类结果，每个元素是一个包含体素索引的向量，代表一个聚类。
 * @param used_map_set 用于记录已使用的体素索引，避免重复处理。
 */
void DynObjCluster::GetClusterResult_voxel(pcl::PointCloud<PointType>::Ptr points_in,
                                           std::vector<Point_Cloud> &umap_in,
                                           std::vector<std::vector<int>> &voxel_clusters,
                                           std::unordered_set<int> &used_map_set)
{
    // 记录处理开始的时间
    ros::Time t0 = ros::Time::now();

    // 如果设置了输出文件且输入点云大小小于2，则记录时间并返回
    if ((out_file != "") && points_in->size() < 2)
    {
        out << (ros::Time::now() - t0).toSec() << " ";
        return;
    }

    // 初始化体素聚类对象
    VOXEL_CLUSTER cluster;
    // 设置输入的点云
    cluster.setInputCloud(*points_in);
    // 设置体素化参数，包括体素分辨率和网格大小
    cluster.setVoxelResolution(Voxel_revolusion, GridMapedgesize_xy, GridMapedgesize_z, xyz_origin);
    // 设置聚类扩展范围
    cluster.setExtendRange(cluster_extend_pixel);
    // 设置最小聚类大小
    cluster.setMinClusterSize(cluster_min_pixel_number);
    // 创建体素地图
    cluster.createVoxelMap(umap_in, used_map_set);
    // 提取聚类结果
    cluster.extract(voxel_clusters);

    // 如果设置了输出文件，则记录处理时间
    if (out_file != "")
        out << (ros::Time::now() - t0).toSec() << " ";
}

/**
 * @brief 发布体素聚类结果，包括计算聚类的边界框和对聚类后的点云进行后处理。
 *
 * @param dyn_tag 每个点的动态标签向量，用于标记点是静态还是动态。
 * @param current_header 当前处理帧的头信息，包含时间戳和帧ID。
 * @param bbox 存储计算得到的边界框信息。
 * @param delta 时间间隔参数，用于一些时间相关的计算。
 * @param voxel_clusters 体素聚类的结果，每个聚类包含一组体素索引。
 * @param raw_point 原始的点云数据。
 * @param used_map_set 用于记录已使用的体素索引，帮助在后处理中识别哪些体素已经被处理。
 */
void DynObjCluster::PubClusterResult_voxel(std::vector<int> &dyn_tag, std_msgs::Header current_header, bbox_t &bbox, double delta,
                                           std::vector<std::vector<int>> &voxel_clusters, const pcl::PointCloud<PointType> &raw_point, std::unordered_set<int> &used_map_set)
{
    // 初始化部分变量
    pcl::PointCloud<PointType> cluster_points; // 存储聚类的点云
    pcl::PointCloud<PointType> true_ground;    // 存储识别为地面的点云

    // 定义一个用于可视化的MarkerArray
    visualization_msgs::MarkerArray numbers;
    numbers.markers.reserve(200);
    cluster_points.reserve(raw_point.size());
    true_ground.reserve(raw_point.size());

    // 将odom旋转矩阵转换为float类型
    Eigen::Matrix3f R = odom_rot.cast<float>();
    Eigen::Vector3f world_z = R.col(2); // 获取世界坐标系下的Z轴基向量

    int Grid_size_1d = 3;
    int Grid_size = pow(Grid_size_1d, 3);

    // 遍历所有聚类
    ros::Time t0 = ros::Time::now();
    int j = 0;
    for (auto it = voxel_clusters.begin(); it != voxel_clusters.end(); it++, j++)
    {
        Eigen::Vector3f xyz;             // 用于存储体素中点的坐标
        XYZExtract(*(it->begin()), xyz); // 提取第一个体素的中心坐标作为聚类的初始坐标

        // 初始化边界框的最小和最大坐标
        float x_min = xyz(0), x_max = xyz(0);
        float y_min = xyz(1), y_max = xyz(1);
        float z_min = xyz(2), z_max = xyz(2);
        int n = 0;

        // 遍历聚类中的所有体素，更新边界框的坐标
        for (auto pit = it->begin(); pit != it->end(); ++pit)
        {
            int voxel = *pit; // 当前体素索引
            umap[voxel].bbox_index = j;
            n = n + umap[voxel].points_num;
            XYZExtract(voxel, xyz); // 提取体素的中心坐标

            // 更新边界框的坐标
            if (xyz(0) < x_min)
                x_min = xyz(0);
            if (xyz(1) < y_min)
                y_min = xyz(1);
            if (xyz(2) < z_min)
                z_min = xyz(2);
            if ((xyz(0) + Voxel_revolusion) > x_max)
                x_max = xyz(0) + Voxel_revolusion;
            if ((xyz(1) + Voxel_revolusion) > y_max)
                y_max = xyz(1) + Voxel_revolusion;
            if ((xyz(2) + Voxel_revolusion) > z_max)
                z_max = xyz(2) + Voxel_revolusion;
        }

        // 根据边界框的大小判断聚类是否有效，并进行相应的处理
        float x_size = x_max - x_min;
        float y_size = y_max - y_min;
        float z_size = z_max - z_min;

        // 判断条件，考虑了聚类的最小体素数量和边界框的大小
        if (cluster_min_pixel_number == 1 || (x_size > Voxel_revolusion + 0.001f && y_size > Voxel_revolusion + 0.001f) || (x_size > Voxel_revolusion + 0.001f && z_size > Voxel_revolusion + 0.001f) || (y_size > Voxel_revolusion + 0.001f && z_size > Voxel_revolusion + 0.001f))
        {
            // 如果聚类满足条件，对其进行进一步处理
            pcl::PointCloud<PointType> clus_pcl;
            bbox.Point_cloud.push_back(clus_pcl); // 存储聚类的点云
            std::vector<int> new_point_indices;
            bbox.Point_indices.push_back(new_point_indices); // 存储点的索引

            // 计算聚类中心并存储相关信息
            geometry_msgs::PoseWithCovarianceStamped center;
            center.header = current_header;
            center.pose.pose.position.x = (x_max + x_min) / 2; // 聚类中心x坐标
            center.pose.pose.position.y = (y_max + y_min) / 2; // 聚类中心y坐标
            center.pose.pose.position.z = (z_max + z_min) / 2; // 聚类中心z坐标
            // 设定默认方向（无旋转）
            center.pose.pose.orientation.x = 0;
            center.pose.pose.orientation.y = 0;
            center.pose.pose.orientation.z = 0;
            center.pose.pose.orientation.w = 1;
            // 存储边界框大小信息
            center.pose.covariance[0 * 6 + 0] = x_size / 2;
            center.pose.covariance[1 * 6 + 1] = y_size / 2;
            center.pose.covariance[2 * 6 + 2] = z_size / 2;
            center.pose.covariance[3 * 6 + 3] = x_size;
            center.pose.covariance[4 * 6 + 4] = y_size;
            center.pose.covariance[5 * 6 + 5] = z_size;
            // 存储边界框最大坐标信息
            center.pose.covariance[2 * 6 + 3] = x_max;
            center.pose.covariance[3 * 6 + 4] = y_max;
            center.pose.covariance[4 * 6 + 5] = z_max;
            // 存储边界框最小坐标信息
            center.pose.covariance[3 * 6 + 2] = x_min;
            center.pose.covariance[4 * 6 + 3] = y_min;
            center.pose.covariance[5 * 6 + 4] = z_min;
            // 添加聚类中心到bbox容器
            bbox.Center.push_back(center);

            // 为每个聚类创建地面点云和地面体素集合
            pcl::PointCloud<PointType> new_pcl;
            bbox.Ground_points.push_back(new_pcl); // 存储地面点云
            bbox.true_ground.push_back(new_pcl);   // 存储真实地面点云
            std::unordered_set<int> new_set;
            bbox.Ground_voxels_set.push_back(new_set); // 存储地面体素集
            std::vector<int> new_vec;
            bbox.Ground_voxels_vec.push_back(new_vec); // 存储地面体素向量
            bbox.umap_points_num.push_back(n);         // 存储聚类内点数
        }
        else
        {
            // 如果聚类不满足条件，将其视为无效聚类，并进行清理
            j--;
            for (auto v = it->begin(); v != it->end(); ++v)
            {
                umap[*v].reset(); // 重置对应体素的信息
            }
        }
    }

    ros::Time t1 = ros::Time::now();
    double hash_newtime = 0.0;
    // 首先，为每个聚类的中心点、边界框的大小和位置初始化相关变量
    std::vector<int> index_bbox(bbox.Center.size()); // 存储聚类索引
    for (int i = 0; i < bbox.Center.size(); i++)
    {
        index_bbox[i] = i; // 初始化聚类索引
    }
    std::vector<std::unordered_set<int>> used_map_set_vec(bbox.Center.size()); // 为每个聚类创建一个用于存储已使用体素的集合

    // 使用并行计算来处理每个边界框，以加速地面估计和后续处理
    std::for_each(std::execution::par, index_bbox.begin(), index_bbox.end(), [&](const int &bbox_i)
                  {
        // 获取聚类中心点和边界框的大小
        PointType center;
        float x_size = bbox.Center[bbox_i].pose.covariance[3*6+3];  // 聚类x轴大小
        float y_size = bbox.Center[bbox_i].pose.covariance[4*6+4];  // 聚类y轴大小
        float z_size = bbox.Center[bbox_i].pose.covariance[5*6+5];  // 聚类z轴大小
        
        center.x = bbox.Center[bbox_i].pose.pose.position.x;
        center.y = bbox.Center[bbox_i].pose.pose.position.y;
        center.z = bbox.Center[bbox_i].pose.pose.position.z;
        PointType max;
        max.x = bbox.Center[bbox_i].pose.covariance[2*6+3];
        max.y = bbox.Center[bbox_i].pose.covariance[3*6+4];
        max.z = bbox.Center[bbox_i].pose.covariance[4*6+5];
        PointType min;
        min.x = bbox.Center[bbox_i].pose.covariance[3*6+2];
        min.y = bbox.Center[bbox_i].pose.covariance[4*6+3];
        min.z = bbox.Center[bbox_i].pose.covariance[5*6+4];
        
        // 计算边界框周围的体素索引，并根据体素状态更新相关集合
        int n_x = std::max(1.0f, 1.0f * x_size) / Voxel_revolusion;
        int n_y = std::max(1.0f, 1.0f * y_size) / Voxel_revolusion;
        int n_z = std::max(1.0f, 1.0f * z_size) / Voxel_revolusion;
        int voxel_center = floor((center.x - xyz_origin(0))/Voxel_revolusion) * GridMapedgesize_xy * GridMapedgesize_z + 
        floor((center.y - xyz_origin(1))/Voxel_revolusion) * GridMapedgesize_z + 
        floor((center.z - xyz_origin(2))/Voxel_revolusion);
        int ii = 0;
        Eigen::Vector3f xyz;
        // 遍历体素，根据体素位置与聚类边界框的关系，更新用于存储已使用体素的集合
        for (int i = 0; i <= 2 * n_x +1; i++)
        {   
            ii += (i%2 ? 1:-1) * i;
            int jj = 0;
            for (int j = 0; j <= 2 * n_y +1; j++)
            {   
                jj += (j%2 ? 1:-1) * j;
                int kk = 0;
                for (int k = 0; k <= 2 * n_z +1; k++)
                {    
                    kk += (k%2 ? 1:-1) * k;
                    int voxel = voxel_center + ii * GridMapedgesize_xy * GridMapedgesize_z + jj * GridMapedgesize_z + kk;
                    if(voxel < 0 || voxel > GridMapsize) continue;
                    XYZExtract(voxel, xyz);
                    // 判断体素是否位于聚类边界框内部或外部，并进行相应处理
                    Eigen::Vector3f voxel_loc(xyz(0) + 0.5f * Voxel_revolusion, xyz(1) + 0.5f * Voxel_revolusion, xyz(2) + 0.5f * Voxel_revolusion);
                    if (umap[voxel].points_num == 0 && !((voxel_loc(0) > min.x && voxel_loc(0) < max.x) && (voxel_loc(1) > min.y && voxel_loc(1) < max.y) && (voxel_loc(2) > min.z && voxel_loc(2) < max.z)))
                    {
                        // 如果体素在边界框外且未被标记，则将其标记为地面体素，并更新地面体素集合
                        umap_ground[voxel].bbox_index = bbox_i;
                        used_map_set_vec[bbox_i].insert(voxel);
                        bbox.Ground_voxels_set[bbox_i].emplace(voxel);
                        bbox.Ground_voxels_vec[bbox_i].push_back(voxel);
                    }
                    else if (umap[voxel].points_num == 0 && (voxel_loc(0) > min.x && voxel_loc(0) < max.x) && (voxel_loc(1) > min.y && voxel_loc(1) < max.y) && (voxel_loc(2) > min.z && voxel_loc(2) < max.z))
                    {
                        // 如果体素在边界框内部且未被标记，则将其标记为内部体素，并更新内部体素集合
                        umap_insidebox[voxel].bbox_index = bbox_i;
                        used_map_set_vec[bbox_i].insert(voxel);
                    }
                }
            }
        } });

    // 合并每个聚类的已使用体素集合到全局已使用体素集合中
    for (int bbox_i = 0; bbox_i < bbox.Center.size(); bbox_i++)
    {
        used_map_set.merge(used_map_set_vec[bbox_i]);
    }

    ros::Time t2 = ros::Time::now();
    // 对原始点云中的每个点进行处理，根据其所属的体素类型（地面、聚类内部等）进行分类和存储
    for (int ite = 0; ite < raw_point.size(); ite++)
    {
        // 跳过未分类的点
        if (dyn_tag[ite] == -1)
            continue;
        int voxel = floor((raw_point[ite].x - xyz_origin(0)) / Voxel_revolusion) * GridMapedgesize_xy * GridMapedgesize_z +
                    floor((raw_point[ite].y - xyz_origin(1)) / Voxel_revolusion) * GridMapedgesize_z +
                    floor((raw_point[ite].z - xyz_origin(2)) / Voxel_revolusion);

        // 跳过无效的体素索引
        if (voxel < 0 || voxel > GridMapsize)
        {
            continue;
        }
        if (umap_ground[voxel].bbox_index > -1)
        {
            // 处理识别为地面的点
            bbox.Ground_points[umap_ground[voxel].bbox_index].push_back(raw_point[ite]);
            if (umap_ground[voxel].points_num == 0)
            {
                umap_ground[voxel].cloud.reset(new pcl::PointCloud<PointType>());
                umap_ground[voxel].cloud->reserve(5);
                umap_ground[voxel].cloud_index.reset(new std::vector<int>());
                umap_ground[voxel].cloud_index->reserve(5);
                umap_ground[voxel].cloud->push_back(raw_point[ite]);
                umap_ground[voxel].cloud_index->push_back(ite);
            }
            else
            {
                umap_ground[voxel].cloud->push_back(raw_point[ite]);
                umap_ground[voxel].cloud_index->push_back(ite);
            }
            umap_ground[voxel].points_num++;
            dyn_tag[ite] = 0; // 标记为静态点
        }
        else if (umap[voxel].points_num > 0 && umap[voxel].bbox_index > -1)
        {
            // 处理聚类中的点
            auto tmp = raw_point[ite];
            tmp.curvature = ite; // 使用曲率字段存储原始索引
            bbox.Point_cloud[umap[voxel].bbox_index].push_back(tmp);
            bbox.Point_indices[umap[voxel].bbox_index].push_back(ite);
            umap[voxel].cloud->push_back(tmp);
            dyn_tag[ite] = 1; // 标记为动态点
        }
        else if (umap_insidebox[voxel].bbox_index > -1)
        {
            // 处理在边界框内部但不在地面上的点
            auto tmp = raw_point[ite];
            tmp.curvature = ite; // 使用曲率字段存储原始索引
            bbox.Point_cloud[umap_insidebox[voxel].bbox_index].push_back(tmp);
            bbox.Point_indices[umap_insidebox[voxel].bbox_index].push_back(ite);
            if (umap_insidebox[voxel].points_num == 0)
            {
                umap_insidebox[voxel].cloud.reset(new pcl::PointCloud<PointType>());
                umap_insidebox[voxel].cloud->reserve(5);
                umap_insidebox[voxel].cloud->push_back(tmp);
            }
            else
            {
                umap_insidebox[voxel].cloud->push_back(tmp);
            }
            umap_insidebox[voxel].points_num++;
            dyn_tag[ite] = 1; // 标记为动态点
        }
        else
        {
            // 其他情况，默认标记为静态点
            dyn_tag[ite] = 0;
        }
    }
    int k = 0;
    ros::Time t3 = ros::Time::now();
    // 对每个聚类执行地面估计和地面移除操作
    // 初始化两个向量来存储地面估计和区域生长的总执行时间，初始化为0
    std::vector<double> ground_estimate_total_time(index_bbox.size(), 0.0);
    std::vector<double> region_growth_time(index_bbox.size(), 0.0);
    // 并行遍历每个聚类的边界框
    std::for_each(std::execution::par, index_bbox.begin(), index_bbox.end(), [&](const int &k)
                  {   
        // 提取聚类的中心点和边界框信息
        geometry_msgs::PoseWithCovarianceStamped center = bbox.Center[k];
        float x_size = center.pose.covariance[3*6+3];
        float y_size = center.pose.covariance[4*6+4];
        float z_size = center.pose.covariance[5*6+5];
        float x_min = center.pose.covariance[3*6+2];
        float y_min = center.pose.covariance[4*6+3];
        float z_min = center.pose.covariance[5*6+4];
        float x_max = center.pose.covariance[2*6+3];
        float y_max = center.pose.covariance[3*6+4];
        float z_max = center.pose.covariance[4*6+5];

        // 初始化地面法向量和地面平面方程
        Eigen::Vector3f ground_norm(0.0, 0.0, 0.0);
        Eigen::Vector4f ground_plane;
        // 开始地面估计的计时
        ros::Time t_ge = ros::Time::now();
        // 执行地面估计
        bool ground_detect = ground_estimate(bbox.Ground_points[k], world_z, ground_norm, ground_plane, bbox.true_ground[k], bbox.Ground_voxels_set[k]);
        // 计算地面估计的执行时间并存储
        ground_estimate_total_time[k] = (ros::Time::now() - t_ge).toSec();
        Eigen::Matrix3f R;
        R.col(0) = ground_norm;
        // 如果检测到地面
        if(ground_detect)
        {   
            // 开始区域生长的计时
            ros::Time t_rg = ros::Time::now();
            // 执行区域生长算法，扩展事件点
            event_extend(R, ground_detect, bbox, dyn_tag, k);
            // 计算区域生长的执行时间并存储
            region_growth_time[k] = (ros::Time::now() - t_rg).toSec();
            // 移除地面体素
            ground_remove(ground_plane, bbox.Point_cloud[k], bbox.Point_indices[k], dyn_tag, bbox.true_ground[k], umap);
        }
        // 移除孤立体素
        isolate_remove(bbox.Point_cloud[k], bbox.Point_indices[k], dyn_tag);
       
       // 如果聚类的点数/聚类大小小于可信阈值，则不认为是可信的
        if ((float)bbox.umap_points_num[k] / (float)bbox.Point_cloud[k].size() < thrustable_thresold) // not trustable
        {   
             // 将聚类中的点标记为静态点
            for (int i = 0; i < bbox.Point_indices[k].size(); i++)
            {
                dyn_tag[bbox.Point_indices[k][i]] = 0;
            }
        }
        else
        {
            // 否则，将聚类点云和真实地面点云累加到总的点云中
            cluster_points += bbox.Point_cloud[k];
            true_ground += bbox.true_ground[k];
        } });
    // 计算地面估计和区域生长的总执行时间
    double total_ground_estimate_total_time = 0.0;
    double total_region_growth_time = 0.0;
    for (int i = 0; i < index_bbox.size(); i++)
    {
        total_ground_estimate_total_time += ground_estimate_total_time[i];
        total_region_growth_time += region_growth_time[i];
    }
    // 如果指定了输出文件，则将总执行时间输出到文件
    if (out_file != "")
        out << total_ground_estimate_total_time << " ";
    if (out_file != "")
        out << total_region_growth_time << " ";

    // cluster_vis_high.publish(numbers);

    // 记录开始清理前的时间
    ros::Time t5 = ros::Time::now();
    // 遍历所有已使用的体素索引
    for (auto ite = used_map_set.begin(); ite != used_map_set.end(); ite++)
    {
        // 重置每个已使用体素的状态，准备下一次聚类使用
        umap[*ite].reset();           // 重置主体素信息
        umap_ground[*ite].reset();    // 重置地面体素信息
        umap_insidebox[*ite].reset(); // 重置内部框体素信息
    }

    // 计算聚类总时间，为从聚类开始到现在的时间减去区域生长算法所花费的总时间
    // 这样做是为了从总时间中剔除区域生长的时间，得到其他处理过程的时间
    double cluster_time = (ros::Time::now() - cluster_begin).toSec() - total_region_growth_time;
    // 如果指定了输出文件路径，则将聚类时间写入文件
    if (out_file != "")
        out << cluster_time << std::endl;
}

/**
 * @brief 对地面候选点云进行地面估计。
 *
 * 此函数旨在通过输入的地面候选点云估计出地面的平面方程，并筛选出真实的地面点。
 * 它首先尝试通过分割地面候选点云，并对每个分割进行平面估计，然后选择最佳匹配的平面作为地面。
 * 最后，基于估计的地面平面，从原始的地面候选点云中筛选出符合条件的地面点。
 *
 * @param ground_pcl 地面候选点云，作为输入。
 * @param world_z 世界坐标系下的Z轴方向，用于判断地面法向量的方向。
 * @param ground_norm 输出参数，估计出的地面法向量。
 * @param ground_plane 输出参数，估计出的地面平面方程，形式为(ax + by + cz + d = 0)。
 * @param true_ground 输出参数，从地面候选点云中筛选出的真实地面点云。
 * @param extend_pixels 用于记录需要从地面估计中排除的体素索引。
 * @return 返回一个布尔值，表示是否成功估计出地面平面。
 */
bool DynObjCluster::ground_estimate(const pcl::PointCloud<PointType> &ground_pcl, const Eigen::Vector3f &world_z, Eigen::Vector3f &ground_norm, Eigen::Vector4f &ground_plane, pcl::PointCloud<PointType> &true_ground, std::unordered_set<int> &extend_pixels)
{
    // 如果输入的地面候选点云为空，则返回false，表示地面估计失败
    if (!ground_pcl.size() > 0)
        return false;

    // 计算分割的批次大小，最少为4，或者是候选点云大小的1%
    int BNUM = std::max(4, (int)ground_pcl.size() / 100);
    // 设置距离阈值和最大与地面夹角
    const float thershold = 0.10f;
    const float max_angle_from_body = 30.0f / 57.3f;
    pcl::PointCloud<PointType> split_pcl;  // 用于存储分割后的点云
    int max_count = 0;                     // 记录最大符合条件的点的数量
    Eigen::Vector3f max_normvec(0, 0, 0);  // 记录最优地面法向量
    pcl::PointCloud<PointType> max_points; // 记录最优地面点云

    // 遍历地面候选点云，分批处理
    for (int i = 0; i < ground_pcl.size(); i++)
    {
        split_pcl.push_back(ground_pcl[i]);
        if (split_pcl.size() == BNUM) // 当分割后的点云大小达到BNUM时进行处理
        {
            Eigen::Vector4f plane;
            // 估计当前分割点云的平面方程
            if (esti_plane(plane, split_pcl) && plane[3] < thershold) // 如果成功估计且距离小于阈值
            {
                Eigen::Vector3f normvec = plane.head(3).normalized(); // 计算法向量
                // 判断法向量与世界Z轴的夹角是否小于最大角度
                if (normvec.cross(world_z).norm() < sin(max_angle_from_body))
                {
                    int count = 0;                         // 记录符合条件的点的数量
                    pcl::PointCloud<PointType> tmp_points; // 临时存储符合条件的点
                    // 遍历候选点云，计算每个点到平面的距离，筛选符合条件的点
                    for (int j = 0; j < ground_pcl.size(); j++)
                    {
                        Eigen::Vector3f point;
                        point[0] = ground_pcl[j].x;
                        point[1] = ground_pcl[j].y;
                        point[2] = ground_pcl[j].z;
                        // 计算点到平面的距离
                        float dis = fabs(point.dot(plane.head(3)) + 1.0f) / plane.head(3).norm();
                        // 如果距离小于阈值，则认为是地面点
                        if (dis < thershold)
                        {
                            tmp_points.push_back(ground_pcl[j]);
                            count++;
                        }
                    }

                    // 更新最大符合条件的点的数量和对应的地面法向量和点云
                    if (count > max_count)
                    {
                        max_count = count;
                        max_normvec = normvec;
                        ground_plane = plane;
                        max_points = tmp_points;
                        // 如果符合条件的点数量超过地面候选点云的60%，则结束循环
                        if (max_count > 0.6f * ground_pcl.size())
                            break;
                    }
                }
            }
            // 清空分割后的点云，准备下一轮分割
            split_pcl.clear();
        }
    }
    if (ground_pcl.size() > 0 && (max_count > 0.2f * ground_pcl.size() || max_count > 500))
    {
        Eigen::Vector4f plane;
        if (esti_plane(plane, max_points) && plane[3] < thershold)
        {
            Eigen::Vector3f normvec = plane.head(3).normalized();
            if (normvec.cross(world_z).norm() < sin(max_angle_from_body))
            {
                max_normvec = normvec;
                ground_plane = plane;
            }
        }
        for (int j = 0; j < ground_pcl.size(); j++)
        {
            Eigen::Vector3f point;
            point[0] = ground_pcl[j].x;
            point[1] = ground_pcl[j].y;
            point[2] = ground_pcl[j].z;
            float dis = fabs(point.dot(ground_plane.head(3)) + 1.0f) / ground_plane.head(3).norm();
            if (dis < thershold)
            {
                true_ground.push_back(ground_pcl[j]);
                int voxel = floor((ground_pcl[j].x - xyz_origin(0)) / Voxel_revolusion) * GridMapedgesize_xy * GridMapedgesize_z + floor((ground_pcl[j].y - xyz_origin(1)) / Voxel_revolusion) * GridMapedgesize_z + floor((ground_pcl[j].z - xyz_origin(2)) / Voxel_revolusion);
                extend_pixels.erase(voxel);
            }
        }
        if (max_normvec[2] < 0)
            max_normvec *= -1;
        ground_norm = max_normvec;
    }
    if (abs(ground_norm.norm() - 1.0f) < 0.1f)
        return true;
    else
        return false;
}

/**
 * @brief 从聚类中移除地面点，更新聚类点云和动态标签。
 *
 * 该函数基于提供的地面平面方程，从给定聚类的点云中移除被识别为地面的点。它计算每个点到地面平面的距离，
 * 如果距离小于设定阈值，则认为该点是地面点，将其移除出聚类点云，并将对应的动态标签设为静态。否则，该点保留在聚类点云中。
 * 此过程有助于进一步净化聚类结果，确保聚类中主要包含动态对象的点云，同时将识别出的地面点添加到地面点云集合中。
 *
 * @param ground_plane 地面平面方程，形式为(ax + by + cz + d = 0)。
 * @param cluster_pcl 聚类的点云，函数将直接修改此点云，移除地面点。
 * @param cluster_pcl_ind 聚类点云中点的索引，用于更新动态标签。
 * @param dyn_tag 动态标签向量，标记每个点是否为动态对象的一部分。地面点将被标记为0。
 * @param true_ground 用于存储被识别为地面的点的点云。
 * @param umap 未在函数签名中直接使用，可能是早期版本的遗留参数，或者预留用于未来扩展。
 */
void DynObjCluster::ground_remove(const Eigen::Vector4f &ground_plane, pcl::PointCloud<PointType> &cluster_pcl, std::vector<int> &cluster_pcl_ind, std::vector<int> &dyn_tag, pcl::PointCloud<PointType> &true_ground, std::vector<Point_Cloud> &umap)
{
    // 设定与地面平面距离的阈值
    const float thershold = 0.10f;
    // 初始化新的聚类点云和对应的索引向量
    pcl::PointCloud<PointType> new_clustet_pcl;
    std::vector<int> new_cluster_pcl_ind;
    // 遍历聚类点云中的每个点
    for (int i = 0; i < cluster_pcl.size(); i++)
    {
        // 提取点坐标
        Eigen::Vector3f point(cluster_pcl[i].x, cluster_pcl[i].y, cluster_pcl[i].z);

        // 计算点到地面平面的距离
        float dis = fabs(point.dot(ground_plane.head(3)) + 1.0f) / ground_plane.head(3).norm();
        // 如果距离大于阈值，保留该点
        if (dis > thershold)
        {
            new_clustet_pcl.push_back(cluster_pcl[i]);
            new_cluster_pcl_ind.push_back(cluster_pcl_ind[i]);
        }
        else
        {
            // 如果距离小于阈值，将对应的动态标签设为静态，并将该点添加到地面点云中
            dyn_tag[cluster_pcl_ind[i]] = 0;
            true_ground.push_back(cluster_pcl[i]);
        }
    }
    // 更新聚类点云和对应的索引向量
    cluster_pcl = new_clustet_pcl;
    cluster_pcl_ind = new_cluster_pcl_ind;
}

/**
 * @brief 从聚类中移除孤立的点，只保留最大的连续体素块中的点。
 * 
 * 此函数针对聚类后的点云进行后处理，通过体素化的方式重新分析聚类中的点，移除那些孤立的点或小块，
 * 只保留最大的连续体素块对应的点云，以提高聚类的准确性和稳定性。对于被移除的点，将其对应的动态标签设置为静态。
 *
 * @param cluster_pcl 聚类的点云，函数将直接修改此点云，移除孤立的点。
 * @param cluster_pcl_ind 聚类点云中点的索引，用于更新动态标签。
 * @param dyn_tag 动态标签向量，标记每个点是否为动态对象的一部分。孤立的点将被标记为0。
 */
void DynObjCluster::isolate_remove(pcl::PointCloud<PointType> &cluster_pcl, std::vector<int> &cluster_pcl_ind, std::vector<int> &dyn_tag)
{
    // 如果聚类的点云大小小于2，直接返回，不进行处理
    if (cluster_pcl.size() < 2)
    {
        return;
    }

    // 初始化新的聚类点云和对应的索引向量
    pcl::PointCloud<PointType> new_cluster_pcl;
    std::vector<int> new_cluster_pcl_ind;

    // 初始化体素聚类对象和相关的容器
    VOXEL_CLUSTER cluster;
    std::unordered_map<int, Point_Cloud::Ptr> umap_cluster; // 存储体素映射的点云
    std::vector<std::vector<int>> voxel_cluster;            // 存储体素聚类结果
    
    // 设置体素聚类的参数，并执行体素化和聚类提取
    cluster.setInputCloud(cluster_pcl);
    cluster.setVoxelResolution(Voxel_revolusion, GridMapedgesize_xy, GridMapedgesize_z, xyz_origin);
    cluster.setExtendRange(cluster_extend_pixel);
    cluster.setMinClusterSize(cluster_min_pixel_number);
    cluster.createVoxelMap(umap_cluster);
    cluster.extract(voxel_cluster);

    // 寻找最大的体素块
    int max_cluster_ind = 0;
    int max_voxel_num = 0;
    for (int i = 0; i < voxel_cluster.size(); i++)
    {
        if (voxel_cluster[i].size() > max_voxel_num)
        {
            max_cluster_ind = i;
            max_voxel_num = voxel_cluster[i].size();
        }
    }

    // 提取最大体素块中的点，更新新的聚类点云和索引
    std::unordered_set<int> dyn_index;  // 用于记录动态点的索引
    for (int i = 0; i < max_voxel_num; i++)
    {
        int voxel = voxel_cluster[max_cluster_ind][i];
        for (int j = 0; j < umap_cluster[voxel]->cloud->size(); j++)
        {
            new_cluster_pcl.push_back(umap_cluster[voxel]->cloud->points[j]);
            new_cluster_pcl_ind.push_back(cluster_pcl_ind[umap_cluster[voxel]->cloud_index->at(j)]);
            dyn_index.insert(cluster_pcl_ind[umap_cluster[voxel]->cloud_index->at(j)]);
        }
    }

    // 对于不在最大体素块中的点，将其动态标签设置为静态
    for (int i = 0; i < cluster_pcl_ind.size(); i++)
    {
        if (!dyn_index.count(cluster_pcl_ind[i]))
        {
            dyn_tag[cluster_pcl_ind[i]] = 0;
        }
    }
    // 清理临时使用的容器和映射
    std::unordered_map<int, Point_Cloud::Ptr>().swap(umap_cluster);
    // 更新聚类点云和对应的索引向量
    cluster_pcl = new_cluster_pcl;
    cluster_pcl_ind = new_cluster_pcl_ind;
}

void DynObjCluster::oobb_estimate(const VoxelMap &vmap, const pcl::PointCloud<PointType> &points, Eigen::Vector3f &min_point_obj,
                                  Eigen::Vector3f &max_point_obj, Eigen::Matrix3f &R, const Eigen::Vector3f ground_norm)
{
    int NMATCH = 5;
    int n = 3; // number of rings
    EA_disk disk(n);
    std::vector<std::vector<Eigen::Vector4f>> NormVectorMap(disk.size);
    std::vector<std::vector<int>> PointSizeList(disk.size);
    for (int i = 0; i < vmap.size(); i++)
    {
        if (!vmap[i].empty() && vmap[i].points.size() >= NMATCH)
        {
            Eigen::Vector4f plane;

            if (esti_plane(plane, vmap[i]))
            {
                plane.head(3) = plane.head(3).normalized();
                if (plane[2] < 0)
                {
                    plane.head(3) *= -1;
                }
                Eigen::Vector2f sphere_coor;
                disk.CatesianToSphere(plane.head(3), sphere_coor);
                Eigen::Vector2f disk_coor;
                disk.SphereToDisk(sphere_coor, disk_coor);
                int index = disk.index_find(disk_coor);
                if (index > pow(2 * (n - 1) + 1, 2) + 4 * n)
                {
                    index = index - 4 * n;
                    plane.head(3) *= -1;
                }
                NormVectorMap[index].push_back(plane);
                PointSizeList[index].push_back(vmap[i].size());
            }
        }
    }
    int max_ind = 0, sec_ind = 0;
    float max_award = 0.0, sec_award = 0.0;
    for (int i = 0; i < NormVectorMap.size(); i++)
    {
        if (!NormVectorMap[i].empty())
        {
            float award = 0.0;
            for (int ite = 0; ite < NormVectorMap[i].size(); ite++)
            {
                award += std::sqrt(PointSizeList[i][ite]) / NormVectorMap[i][ite](3);
            }
            if (award > max_award)
            {
                sec_award = max_award;
                sec_ind = max_ind;
                max_award = award;
                max_ind = i;
            }
            else if (award > sec_award)
            {
                sec_award = award;
                sec_ind = i;
            }
        }
    }
    Eigen::Vector3f direction_main(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f direction_aux(0.0f, 0.0f, 0.0f);
    if (max_award > 0)
    {
        for (int ite = 0; ite < NormVectorMap[max_ind].size(); ite++)
        {
            direction_main = direction_main + NormVectorMap[max_ind][ite].head(3) * PointSizeList[max_ind][ite] / NormVectorMap[max_ind][ite](3);
        }
        direction_main.normalize();
    }
    else
        direction_main << 0.0, 0.0, 1.0;
    if (sec_award > 0)
    {
        for (int ite = 0; ite < NormVectorMap[sec_ind].size(); ite++)
        {
            direction_aux = direction_aux + NormVectorMap[sec_ind][ite].head(3) * PointSizeList[sec_ind][ite] / NormVectorMap[sec_ind][ite](3);
        }
        direction_aux.normalize();
    }
    else
        direction_aux << 1.0, 0.0, 0.0;

    if (ground_norm.norm() < 0.1)
    {
        R.col(0) = direction_main;
        R.col(1) = (direction_aux - direction_aux.dot(R.col(0)) * R.col(0)).normalized();
        Eigen::Vector3f world_z(0.0, 0.0, 1.0);
        if (abs(R.col(1).dot(world_z)) > 0.866f)
        {
            R.col(2) = R.col(1);
            R.col(1) = -(R.col(0).cross(R.col(2))).normalized();
        }
        else if (abs(R.col(0).dot(world_z)) > 0.866f)
        {
            R.col(1).swap(R.col(0));
            R.col(2) = R.col(1);
            R.col(1) = -(R.col(0).cross(R.col(2))).normalized();
        }
        else
        {
            R.col(2) = (R.col(0).cross(R.col(1))).normalized();
        }
    }
    else
    {
        R.col(0) = ground_norm;
        if (ground_norm.dot(direction_main) > 0.95)
        {
            direction_main = direction_aux;
        }
        R.col(1) = (direction_main - direction_main.dot(R.col(0)) * R.col(0)).normalized();
        R.col(2) = (R.col(0).cross(R.col(1))).normalized();
    }

    Eigen::Vector3f point_vec(points[0].x, points[0].y, points[0].z);
    Eigen::Vector3f project = (point_vec.transpose() * R).transpose();
    float x_min = project[0], x_max = project[0];
    float y_min = project[1], y_max = project[1];
    float z_min = project[2], z_max = project[2];
    for (int pit = 0; pit < points.size(); pit++)
    {
        point_vec << points[pit].x, points[pit].y, points[pit].z;
        project = (point_vec.transpose() * R).transpose();
        if (project[0] < x_min)
            x_min = project[0];
        if (project[1] < y_min)
            y_min = project[1];
        if (project[2] < z_min)
            z_min = project[2];
        if (project[0] > x_max)
            x_max = project[0];
        if (project[1] > y_max)
            y_max = project[1];
        if (project[2] > z_max)
            z_max = project[2];
    }
    max_point_obj << x_max, y_max, z_max;
    min_point_obj << x_min, y_min, z_min;
}

/**
 * @brief 扩展地面体素的邻居体素，用于合并相邻的动态对象体素。
 *
 * 此函数通过分析地面体素的邻居体素，尝试将其合并到相应的聚类中。对于每个地面体素，检查其六个方向的邻居体素，
 * 并根据邻居体素的属性（是否属于同一聚类、是否包含足够数量的点）决定是否将邻居体素合并到当前聚类。
 * 若邻居体素满足合并条件，即它们属于同一聚类且包含足够的点，此函数会通过估计邻居体素点云的平面，
 * 并与已检测的地面平面进行比较，来决定是否将这些邻居体素视为动态对象的一部分。
 *
 * @param R 地面平面的旋转矩阵，用于与邻居体素点云的平面法向量进行比较。
 * @param ground_detect 布尔值，指示是否已检测到地面。
 * @param bbox 存储聚类和地面体素信息的结构体。
 * @param dyn_tag 动态标签向量，用于标记每个点是否为动态对象的一部分。
 * @param bbox_index 当前处理的聚类索引。
 */
void DynObjCluster::event_extend(const Eigen::Matrix3f &R, bool ground_detect,
                                 bbox_t &bbox, std::vector<int> &dyn_tag, const int &bbox_index)
{
    // 遍历当前聚类的地面体素向量
    for (int i = 0; i < bbox.Ground_voxels_vec[bbox_index].size(); i++)
    {
        int voxel_cur = bbox.Ground_voxels_vec[bbox_index][i];
        // 检查当前体素是否在地面体素集合中，并且包含超过两个点
        if (bbox.Ground_voxels_set[bbox_index].count(voxel_cur) && umap_ground[voxel_cur].points_num > 2)
        {
            // 定义六个方向的邻居体素索引偏移
            int x_ind[6] = {1, -1, 0, 0, 0, 0};
            int y_ind[6] = {0, 0, 1, -1, 0, 0};
            int z_ind[6] = {0, 0, 0, 0, 1, -1};
            // 检查六个方向的邻居体素
            for (int ind = 0; ind < 6; ind++)
            {
                int voxel_neighbor = voxel_cur + x_ind[ind] * GridMapedgesize_xy * GridMapedgesize_z + y_ind[ind] * GridMapedgesize_z + z_ind[ind];
                // 确保邻居体素索引有效
                if (voxel_neighbor < 0 || voxel_neighbor > GridMapsize)
                    continue;
                // 检查邻居体素是否属于当前聚类，并且包含点
                if ((umap_insidebox[voxel_neighbor].bbox_index > -1 && umap_insidebox[voxel_neighbor].points_num > 0 && umap_insidebox[voxel_neighbor].bbox_index == bbox_index) || (umap[voxel_neighbor].points_num > 0 && umap[voxel_neighbor].cloud->size() > 0 && umap[voxel_neighbor].bbox_index == bbox_index))
                {
                    pcl::PointCloud<PointType> points = *(umap_ground[voxel_cur].cloud);
                    Eigen::Vector4f plane;
                    // 如果检测到地面，尝试对邻居体素点云进行平面估计
                    if (ground_detect)
                    {
                        // 如果邻居体素点云的平面法向量与地面平面法向量的夹角小于0.8（大约45度）
                        if (esti_plane(plane, points) && abs(R.col(0).dot(plane.head(3).normalized())) < 0.8f)
                        {
                            // 则认为邻居体素也属于当前聚类
                            umap[voxel_cur].bbox_index = bbox_index;
                            umap[voxel_cur].cloud = umap_ground[voxel_cur].cloud;
                            // 将邻居体素的点云合并到当前聚类的点云中
                            bbox.Point_cloud[bbox_index] += points;
                            // 更新动态标签，将这些点标记为动态对象的一部分
                            for (int j = 0; j < umap_ground[voxel_cur].cloud_index->size(); j++)
                            {
                                dyn_tag[umap_ground[voxel_cur].cloud_index->at(j)] = 1;
                                // 同时记录这些点的索引，用于后续处理c
                                bbox.Point_indices[bbox_index].push_back(umap_ground[voxel_cur].cloud_index->at(j));
                            }
                            // 同时记录这些点的索引，用于后续处理
                            break;
                        }
                    }
                }
            }
        }
    }
}

/**
 * @brief 使用主成分分析(PCA)方法估计点云平面。
 *
 * 此函数通过输入的点云计算一个最佳拟合平面。它首先构造了一个设计矩阵A和向量b，
 * 其中A的每一行包含了点云中一个点的x, y, z坐标，向量b初始化为-1。然后，利用QR分解解方程A * normvec = b，
 * 求解出最佳拟合平面的法向量normvec。最后，计算点云到该平面的平均距离，作为平面方程的最后一个参数。
 *
 * @param pca_result 输出参数，包含估计出的平面方程参数：法向量(normvec)和点到平面的平均距离。
 * @param point 输入参数，需要进行平面估计的点云。
 * @return 如果所有点到平面的距离都小于预设阈值，则返回true，表示成功估计出平面；否则返回false。
 */
bool DynObjCluster::esti_plane(Eigen::Vector4f &pca_result, const pcl::PointCloud<PointType> &point)
{
    // 预设的距离阈值，用于判断点是否符合平面
    const float threshold = 0.1;
    // 点云的大小
    int point_size = point.size();

    // 初始化设计矩阵A和向量b
    Eigen::Matrix<float, Eigen::Dynamic, 3> A;
    Eigen::Matrix<float, Eigen::Dynamic, 1> b;
    A.resize(point_size, 3);
    b.resize(point_size, 1);
    b.setOnes();
    b *= -1.0f; // 将b初始化为全-1

    // 填充设计矩阵A
    for (int j = 0; j < point_size; j++)
    {
        A(j, 0) = point[j].x;
        A(j, 1) = point[j].y;
        A(j, 2) = point[j].z;
    }

    // 使用QR分解求解方程A * normvec = b，得到平面的法向量normvec
    Eigen::Vector3f normvec = A.colPivHouseholderQr().solve(b);
    float norm = normvec.norm(); // 计算法向量的模

    // 计算点云中每个点到估计平面的平均距离
    float average_dis = 0.0;
    for (int j = 0; j < point_size; j++)
    {
        // 计算点到平面的距离
        float tmp = fabs(normvec.dot(A.row(j)) + 1.0);
        average_dis += tmp;
        // 如果任一点到平面的距离大于阈值，认为估计失败
        if (tmp > threshold)
        {
            return false;
        }
    }

    // 计算平均距离，并调整为非负值
    average_dis = std::max(0.01f, average_dis / point_size / norm);
    // 填充输出的平面方程参数
    pca_result(0) = normvec(0);
    pca_result(1) = normvec(1);
    pca_result(2) = normvec(2);
    pca_result(3) = average_dis;
    // 如果执行到此处，表示成功估计出平面
    return true;
}

/**
 * @brief 根据体素索引计算体素在三维空间中的坐标。
 *
 * 这个方法通过给定的体素索引，使用体素的原点坐标(xyz_origin)、体素分辨率(Voxel_revolusion)，
 * 以及网格的尺寸(GridMapedgesize_xy * GridMapedgesize_z)来计算对应体素的空间坐标。
 *
 * @param position 体素的索引，是一个整数，表示体素在一维数组中的位置。
 * @param xyz 输出参数，用于存储计算得到的体素的空间坐标。
 */
void DynObjCluster::XYZExtract(const int &position, Eigen::Vector3f &xyz)
{
    int left = position; // 用于存储剩余部分的索引，初始值为体素索引

    // 计算X坐标
    // 首先计算体素索引对应于X轴方向上的体素数量，乘以体素分辨率后加上原点的X坐标
    xyz(0) = xyz_origin(0) + (float)floor(position / (GridMapedgesize_xy * GridMapedgesize_z)) * Voxel_revolusion;

    // 更新剩余部分的索引，移除已经计算过的X轴部分
    left = left - (float)floor(position / (GridMapedgesize_xy * GridMapedgesize_z)) * (GridMapedgesize_xy * GridMapedgesize_z);

    // 计算Y坐标
    // 根据剩余的索引计算Y轴方向上的体素数量，同样乘以体素分辨率后加上原点的Y坐标
    xyz(1) = xyz_origin(1) + (float)floor(left / GridMapedgesize_z) * Voxel_revolusion;

    // 更新剩余部分的索引，移除已经计算过的Y轴部分
    left = left - (float)floor(left / GridMapedgesize_z) * GridMapedgesize_z;

    // 计算Z坐标
    // 剩余的索引即为Z轴方向上的体素数量，乘以体素分辨率后加上原点的Z坐标得到Z坐标
    xyz(2) = xyz_origin(2) + left * Voxel_revolusion;
}
