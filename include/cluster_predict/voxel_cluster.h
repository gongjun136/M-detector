/*
 * @Author: gongjun136 gongjun136@gmail.com
 * @Date: 2024-03-13 15:55:08
 * @LastEditors: gongjun136 gongjun136@gmail.com
 * @LastEditTime: 2024-03-21 13:47:20
 * @FilePath: /catkin_ws_M-detector/src/M-detector-noted/include/cluster_predict/voxel_cluster.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef VOXEL_CLUSTER_H
#define VOXEL_CLUSTER_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <unordered_map>
#include <unordered_set>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZINormal PointType;

struct Point_Cloud
{
    typedef boost::shared_ptr<Point_Cloud> Ptr;
    int bbox_index{-1};
    int points_num{0};
    pcl::PointCloud<PointType>::Ptr cloud;           // 点云
    boost::shared_ptr<std::vector<int>> cloud_index; // 点在点云中的索引
    Point_Cloud(PointType point, int index)
    {
        this->cloud->points.push_back(point);
        this->bbox_index = index;
    }

    Point_Cloud(int index)
    {
        this->bbox_index = index;
    }

    Point_Cloud()
    {
    }

    Point_Cloud(PointType point)
    {
        this->cloud.reset(new pcl::PointCloud<PointType>());
        this->cloud->reserve(5);
        this->cloud->points.push_back(point);
    }

    ~Point_Cloud(){};

    void reset()
    {
        this->points_num = 0;
        this->bbox_index = -1;
    }
};

/**
 * @brief 类用于对输入的点云进行体素化处理并进行聚类。
 * 它将点云转换成体素格子，然后基于体素格子的邻接性进行聚类。
 *
 */
class VOXEL_CLUSTER
{
public:
    typedef pcl::PointXYZINormal PointType; // 定义使用的点类型
    std::vector<int> voxel_list;            // 体素列表，存储每个体素的索引
    std::unordered_set<int> voxel_set;      // 体素集合，用于快速检查某个体素是否已经被处理

    // 构造函数和析构函数
    VOXEL_CLUSTER(){};
    ~VOXEL_CLUSTER(){};

    // 设置输入点云
    void setInputCloud(const pcl::PointCloud<PointType> &points_in)
    {
        points_ = points_in;
    }

    // 设置体素分辨率和边界大小
    void setVoxelResolution(float voxel_length, float edge_size_xy, float edge_size_z, const Eigen::Vector3f &xyz_origin_in)
    {
        Voxel_revolusion = voxel_length;
        Grid_edge_size_xy = edge_size_xy;
        Grid_edge_size_z = edge_size_z;
        xyz_origin = xyz_origin_in;
    }

    // 设置聚类扩展的范围
    void setExtendRange(int range)
    {
        max_range = range;
    }

    // 设置聚类的最小体素数量
    void setMinClusterSize(int min_cluster_voxels)
    {
        min_cluster_voxels_ = min_cluster_voxels;
    }

    /**
     * @brief 将输入的点云数据转换成体素映射。
     *
     * 此函数遍历输入点云中的每个点，根据点的坐标和设定的体素分辨率，将其分配到相应的体素中。
     * 每个体素代表空间中的一个固定大小的立方体区域。此过程是为了将连续的空间转换为离散的体素空间，
     * 以便进行高效的聚类处理。
     *
     * @param umap_in 用于存储每个体素内的点云数据的容器。
     * @param used_map_set 用于记录已经被使用（即至少包含一个点）的体素的索引，避免重复处理。
     */
    void createVoxelMap(std::vector<Point_Cloud> &umap_in, std::unordered_set<int> &used_map_set)
    {
        // 遍历输入点云中的所有点
        for (int i = 0; i < points_.size(); i++)
        {
            PointType tmp;
            // 读取当前点的坐标
            tmp.x = points_.points[i].x;
            tmp.y = points_.points[i].y;
            tmp.z = points_.points[i].z;
            tmp.intensity = 0; // 这里将强度设置为0，可能是因为在此上下文中不需要使用强度信息

            // VOXEL voxel(floor(tmp.x/Voxel_revolusion), floor(tmp.y/Voxel_revolusion), floor(tmp.z/Voxel_revolusion));
            // 计算当前点所在的体素的索引
            // 这是通过将点的坐标减去原点坐标后，除以体素分辨率并向下取整，来实现的
            int position = floor((tmp.x - xyz_origin(0)) / Voxel_revolusion) * Grid_edge_size_xy * Grid_edge_size_z + floor((tmp.y - xyz_origin(1)) / Voxel_revolusion) * Grid_edge_size_z + floor((tmp.z - xyz_origin(2)) / Voxel_revolusion);
            // 检查计算出的体素索引是否在有效范围内
            if (position < 0 || position > Grid_edge_size_xy * Grid_edge_size_xy * Grid_edge_size_z)
                continue;

            // 如果当前体素已经包含点，则增加该体素中点的数量
            if (umap_in[position].points_num > 0)
            {
                // umap[voxel].cloud.push_back(tmp);
                umap_in[position].points_num = umap_in[position].points_num + 1;
                continue;
            }
            else
            {
                // 如果当前体素是第一次被发现包含点，则初始化该体素并记录它
                used_map_set.insert(position);  // 标记为已使用
                voxel_list.push_back(position); // 加入到体素列表
                voxel_set.insert(position);     // 加入到体素集合

                // umap_in[voxel] = new Point_Cloud(-1);
                // 初始化体素内的点云数据和索引
                umap_in[position].cloud.reset(new pcl::PointCloud<PointType>());
                umap_in[position].cloud_index.reset(new std::vector<int>());
                umap_in[position].points_num = 1;
            }
        }
    }

    /**
     * @brief 将输入的点云数据转换成体素映射，并根据index_en参数决定是否存储每个点的索引。
     *
     * 与第一个createVoxelMap函数相比，这个版本在存储每个体素包含的点数的基础上，
     * 还可以选择性地存储每个点的具体信息（包括坐标和在原始点云中的索引）。
     * 这使得函数更加灵活，能够根据需要提供更多的信息，适用于后续处理需要使用点的具体位置或索引的场景。
     *
     * @param umap_in 用于存储每个体素内的点云数据和索引的容器。
     * @param index_en 布尔标志，指示是否需要存储每个点的索引信息。
     */
    void createVoxelMap(std::vector<Point_Cloud> &umap_in, bool index_en)
    {
        // 这里省略了遍历输入点云并将每个点分配到对应体素的具体实现。
        // 如果index_en为true，则每个体素内除了记录点数，还会具体存储每个点的信息和它们在原始点云中的索引。
        // 这提供了在后续处理中使用点的位置或索引进行更精细操作的可能性。
        // 遍历输入点云中的所有点
        for (int i = 0; i < points_.size(); i++)
        {
            PointType tmp;
            // 读取当前点的坐标
            tmp.x = points_.points[i].x;
            tmp.y = points_.points[i].y;
            tmp.z = points_.points[i].z;
            tmp.intensity = 0; // 这里将强度设置为0，可能是因为在此上下文中不需要使用强度信息

            // 计算当前点所在的体素的索引
            // 这是通过将点的坐标减去原点坐标后，除以体素分辨率并向下取整，来实现的
            int position = floor((tmp.x - xyz_origin(0)) / Voxel_revolusion) * Grid_edge_size_xy * Grid_edge_size_z + floor((tmp.y - xyz_origin(1)) / Voxel_revolusion) * Grid_edge_size_z + floor((tmp.z - xyz_origin(2)) / Voxel_revolusion);
            // 检查计算出的体素索引是否在有效范围内
            if (position < 0 || position > Grid_edge_size_xy * Grid_edge_size_xy * Grid_edge_size_z)
                continue;

            // 如果当前体素已经包含点，则增加该体素中点的数量
            if (umap_in[position].points_num > 0)
            {
                umap_in[position].cloud->push_back(tmp);
                umap_in[position].points_num = umap_in[position].points_num + 1;
                umap_in[position].cloud_index->push_back(i);
                continue;
            }
            else
            {
                // 如果当前体素是第一次被发现包含点，则初始化该体素并记录它
                voxel_list.push_back(position); // 加入到体素列表
                voxel_set.insert(position);     // 加入到体素集合

                // 初始化该体素的Point_Cloud对象，并设置初始点数和索引
                umap_in[position].cloud.reset(new pcl::PointCloud<PointType>());
                umap[position].cloud->reserve(5);     // 预分配空间以提高效率
                umap[position].cloud->push_back(tmp); // 添加当前点到体素的点云中
                umap[position].points_num = 1;        // 设置点数为1c
                // 初始化并设置索引数组
                umap_in[position].cloud_index.reset(new std::vector<int>());
                umap_in[position].cloud_index->reserve(5);
                umap_in[position].cloud_index->push_back(i);
            }
        }
    }

    /**
     * @brief 将输入的点云数据转换成体素映射，使用unordered_map作为存储容器。
     *
     * 与前两个createVoxelMap函数不同，这个版本使用std::unordered_map来存储体素映射，
     * 其中每个体素的索引作为键，指向Point_Cloud对象的智能指针作为值。
     * 这种方法提供了对每个体素数据的直接访问能力，使得更新和查询体素信息变得更加高效。
     * 特别适用于动态环境下需要频繁更新体素内点云数据的场景。
     *
     * @param umap 用于存储体素映射的容器，每个体素对应一个Point_Cloud对象的智能指针。
     */
    void createVoxelMap(std::unordered_map<int, Point_Cloud::Ptr> &umap)
    {
        // 这里省略了遍历输入点云并将每个点分配到对应体素的具体实现。
        // 使用unordered_map存储体素映射，允许直接通过体素索引快速访问或更新体素内的点云数据。
        // 这种数据结构的选择，使得处理大规模点云数据时能够获得更好的性能。
        for (int i = 0; i < points_.size(); i++)
        {
            PointType tmp;
            tmp.x = points_.points[i].x;
            tmp.y = points_.points[i].y;
            tmp.z = points_.points[i].z;
            int position = floor((tmp.x - xyz_origin(0)) / Voxel_revolusion) * Grid_edge_size_xy * Grid_edge_size_z + floor((tmp.y - xyz_origin(1)) / Voxel_revolusion) * Grid_edge_size_z + floor((tmp.z - xyz_origin(2)) / Voxel_revolusion);
            if (position < 0 || position > Grid_edge_size_xy * Grid_edge_size_xy * Grid_edge_size_z)
                continue;
            if (umap.count(position))
            {
                umap[position]->cloud->push_back(tmp);
                umap[position]->points_num = umap[position]->points_num + 1;
                umap[position]->cloud_index->push_back(i);
                continue;
            }
            else
            {
                voxel_list.push_back(position);
                voxel_set.insert(position);
                umap[position].reset(new Point_Cloud(tmp));
                // umap_in[voxel] = new Point_Cloud(tmp);
                umap[position]->points_num = 1;
                umap[position]->cloud_index.reset(new std::vector<int>());
                umap[position]->cloud_index->reserve(5);
                umap[position]->cloud_index->push_back(i);
            }
        }
    }

    /**
     * @brief 递归地扩展聚类到邻近的体素。
     *
     * 这个方法基于给定的体素索引，递归地寻找所有空间邻接的体素并将它们加入到当前聚类中。
     * 这一过程通过计算邻近体素的索引并检查这些体素是否已经被加入到聚类中来实现。
     * 如果邻近体素未被加入，则将其加入并继续递归扩展。
     *
     * @param voxel 当前处理的体素索引。
     * @param voxel_added 用于记录已经被加入到当前聚类的体素索引的集合。
     */
    void extendVoxelNeighbor(int voxel, std::unordered_set<int> &voxel_added)
    {
        // 遍历当前体素的所有可能的空间邻居
        for (int x_neighbor = -max_range; x_neighbor <= max_range; x_neighbor++)
        {
            for (int y_neighbor = -max_range; y_neighbor <= max_range; y_neighbor++)
            {
                for (int z_neighbor = -max_range; z_neighbor <= max_range; z_neighbor++)
                {
                    // 计算邻居体素与当前体素的距离
                    float neighbor_dis = sqrt(x_neighbor * x_neighbor + y_neighbor * y_neighbor + z_neighbor * z_neighbor);
                    // 如果距离在设定的最大范围内，则考虑此体素为邻居
                    if (neighbor_dis - (float)max_range < 0.001f)
                    {
                        // 计算邻居体素的索引
                        int voxel_neighbor = voxel + x_neighbor * Grid_edge_size_xy * Grid_edge_size_z + y_neighbor * Grid_edge_size_z + z_neighbor;
                        // 检查邻居体素索引是否有效
                        if (voxel_neighbor < 0 || voxel_neighbor > Grid_edge_size_xy * Grid_edge_size_xy * Grid_edge_size_z)
                            continue;

                        // 如果邻居体素是未处理的且未被加入到当前聚类c
                        if (voxel_set.count(voxel_neighbor) && !voxel_added.count(voxel_neighbor))
                        {
                            // 如果邻居体素是未处理的且未被加入到当前聚类
                            voxel_added.insert(voxel_neighbor);
                            // 递归地继续扩展这个邻居体素
                            extendVoxelNeighbor(voxel_neighbor, voxel_added);
                        }
                    }
                }
            }
        }
        // for (int neighbor_ind = 0; neighbor_ind < 6; neighbor_ind++)
        // {
        //     VOXEL voxel_neighbor(voxel.x + x_neighbor[neighbor_ind], voxel.y + y_neighbor[neighbor_ind], voxel.z + z_neighbor[neighbor_ind]);
        //     if (umap.count(voxel_neighbor) && !voxel_added.count(voxel_neighbor))
        //     {
        //         voxel_added.insert(voxel_neighbor);
        //         extendVoxelNeighbor(voxel_neighbor, voxel_added);
        //     }
        // }
    }

    /**
     * @brief 从体素映射中提取聚类。
     *
     * 此函数遍历所有体素，使用体素的空间邻接性来确定聚类。对于每个已识别的体素，
     * 它会递归地查找所有空间邻接的体素，形成一个聚类。每个聚类由一组体素索引组成，
     * 这些索引对应于输入点云中彼此邻近的点集。
     *
     * @param voxel_clusters 聚类结果的输出参数，每个元素是一个包含体素索引的向量，代表一个聚类。
     */
    void extract(std::vector<std::vector<int>> &voxel_clusters)
    {
        // 遍历所有体素，对每个体素进行聚类处理
        for (int voxel_ind = 0; voxel_ind < voxel_list.size(); voxel_ind++)
        {
            int voxel_cur = voxel_list[voxel_ind];
            // 如果当前体素已经被标记为某个聚类的一部分，则跳过
            if (!voxel_set.count(voxel_cur))
            {
                continue;
            }

            // 使用unordered_set来记录已经添加到当前聚类的体素
            std::unordered_set<int> voxel_added;
            // 将当前体素加入到聚类中
            voxel_added.emplace(voxel_cur);

            // 递归地扩展当前聚类，添加所有空间邻接的体素
            extendVoxelNeighbor(voxel_cur, voxel_added);
            int size = 0;

            // 将当前聚类中的体素索引存储到一个vector中
            std::vector<int> voxel_candidate_vec;
            for (auto iter = voxel_added.begin(); iter != voxel_added.end(); ++iter)
            {
                // 从体素集合中移除已经加入聚类的体素
                voxel_set.erase(*iter);
                // 添加体素索引到聚类
                voxel_candidate_vec.push_back(*iter);
                size++;
            }

            // 如果当前聚类包含的体素数目满足最小聚类体素数要求，则认为是有效聚类
            if (size >= min_cluster_voxels_)
            {
                /// 添加当前聚类到聚类结果中
                voxel_clusters.push_back(voxel_candidate_vec);
            }
        }
        // std::vector<Point_Cloud>().swap(umap_copy);
        // std::vector<Point_Cloud>().swap(umap);
    }

protected:
    pcl::PointCloud<PointType> points_; // 输入的点云数据
    float Voxel_revolusion;             // 体素的长度
    float Grid_edge_size_xy;            // XY平面的网格边界大小
    float Grid_edge_size_z;             // Z轴的网格边界大小
    Eigen::Vector3f xyz_origin;         // 网格的原点
    std::vector<Point_Cloud> umap;      // 体素映射
    int max_range;                      // 最大扩展范围
    // int x_neighbor[6] = {1, -1, 0, 0,  0,  0};
    // int y_neighbor[6] = {0, 0, 1, -1,  0,  0};
    // int z_neighbor[6] = {0, 0, 0,  0, -1, -1};
    int min_cluster_voxels_; // 最小聚类体素数
};

#endif