/*
 * @Author: gongjun136 gongjun136@gmail.com
 * @Date: 2024-03-12 09:39:30
 * @LastEditors: gongjun136 gongjun136@gmail.com
 * @LastEditTime: 2024-03-22 09:43:28
 * @FilePath: /catkin_ws_M-detector/src/M-detector-noted/include/M-detector-noted/DynObjFilter.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef DYN_OBJ_FLT_H
#define DYN_OBJ_FLT_H

#include <omp.h>
#include <mutex>
#include <math.h>
#include <ros/ros.h>
// #include <so3_math.h>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CompressedImage.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/LU>

#include <algorithm>
#include <chrono>
#include <execution>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <glog/logging.h>

#include "m-detector-noted/dyn_obj_cluster.h"
#include "types.h"
#include "parallel_q.h"

using namespace std;
using namespace Eigen;
using namespace cv;

// 常量定义
#define PI_MATH (M_PI)
constexpr double kDEG2RAD = M_PI / 180.0; // deg->rad
constexpr double kRAD2DEG = 180.0 / M_PI; // rad -> deg
#define HASH_P 116101
#define MAX_N 100000
#define MAX_2D_N (564393) // 1317755(1317755) //(50086) 39489912 // MAX_1D * MAX_1D_HALF

// 水平索引阈值: ???
#define MAX_1D (1257) //(2095) //(317)  // 2*pi/ hor_resolution
// 俯仰索引阈值: 水平像素数量的一半=3.14/0.007???
#define MAX_1D_HALF (449) // 3142() //158 31416 pi / ver_resolution
#define DEPTH_WIDTH (80 * 10)
#define COE_SMALL 1000
#define MAP_NUM 17   // 30
#define HASH_PRIM 19 // 37          // 一个质数

// 定义一个枚举类型，表示动态对象的不同状态
enum dyn_obj_flg
{
    STATIC,    // 静态点
    CASE1,     // 情况1的动态点
    CASE2,     // 情况2的动态点
    CASE3,     // 情况3的动态点
    SELF,      //
    UNCERTAIN, //
    INVALID    // 无效点
};

// 点结构体，存储点的详细信息
struct point_soph
{
    int hor_ind;    // 深度图水平索引
    V3F vec;        // 0-水平角；1-俯仰角；2-深度
    int ver_ind;    // 深度图俯仰索引
    int position;   // 深度图的位置标识/像素索引，水平索引乘以某个固定的值（MAX_1D_HALF），然后加上俯仰索引
    int occu_times; // 当前点被遮挡的次数
    int is_occu_times;
    Vector3i occu_index; // 当前点被遮挡的点信息：0-深度图索引，1-深度图的位置标识，2-像素处点集合的索引
    V3F occ_vec;         // 当前点被遮挡的点的vec
    double time;         // 时间戳
    // ---------情况3-------------
    Vector3i is_occu_index;
    V3F is_occ_vec;
    // ---------情况3-------------
    M3D rot;         // 旋转
    V3D transl;      // 平移
    dyn_obj_flg dyn; // 动态标识符
    V3D glob;        // 世界系坐标
    V3D local;       // 载体系坐标
    V3F cur_vec;
    float intensity; // 强度
    bool is_distort; // 是否失真
    V3D last_closest;
    array<float, MAP_NUM> last_depth_interps = {};
    // 索引(帧id%HASH_PRIM),V3F当前点在索引的深度图的vec
    // ？？？？如果帧数大于HASH_PRIM，会出现索引重复的情况
    array<V3F, HASH_PRIM> last_vecs = {};
    // 索引(帧id%HASH_PRIM),Vector3i(0-hor_ind;1-ver_ind;position)
    // ？？？？如果帧数大于HASH_PRIM，会出现索引重复的情况
    array<Vector3i, HASH_PRIM> last_positions = {};
    typedef boost::shared_ptr<point_soph> Ptr;
    point_soph(V3D &point, float &hor_resolution_max, float &ver_resolution_max)
    {
        vec(2) = float(point.norm());
        vec(0) = atan2f(float(point(1)), float(point(0)));
        vec(1) = atan2f(float(point(2)), sqrt(pow(float(point(0)), 2) + pow(float(point(1)), 2)));
        hor_ind = floor((vec(0) + PI_MATH) / hor_resolution_max);
        ver_ind = floor((vec(1) + 0.5 * PI_MATH) / ver_resolution_max);
        position = hor_ind * MAX_1D_HALF + ver_ind;
        time = -1;
        occu_times = is_occu_times = 0;
        occu_index = -1 * Vector3i::Ones();
        is_occu_index = -1 * Vector3i::Ones();
        occ_vec.setZero();
        is_occ_vec.setZero();
        transl.setZero();
        glob.setZero();
        rot.setOnes();
        last_depth_interps.fill(0.0);
        last_vecs.fill(V3F::Zero());
        last_positions.fill(Vector3i::Zero());
        is_distort = false;
        cur_vec.setZero();
        local.setZero();
        last_closest.setZero();
    };
    point_soph()
    {
        vec.setZero();
        hor_ind = ver_ind = position = occu_times = is_occu_times = 0;
        time = -1;
        occu_index = -1 * Vector3i::Ones();
        is_occu_index = -1 * Vector3i::Ones();
        occ_vec.setZero();
        is_occ_vec.setZero();
        transl.setZero();
        glob.setZero();
        rot.setOnes();
        last_depth_interps.fill(0.0);
        last_vecs.fill(V3F::Zero());
        last_positions.fill(Vector3i::Zero());
        is_distort = false;
        cur_vec.setZero();
        local.setZero();
        last_closest.setZero();
    };
    point_soph(V3F s, int ind1, int ind2, int pos)
    {
        vec = s;
        hor_ind = ind1;
        ver_ind = ind2;
        position = pos;
        occu_times = is_occu_times = 0;
        time = -1;
        occu_index = -1 * Vector3i::Ones();
        is_occu_index = -1 * Vector3i::Ones();
        occ_vec.setZero();
        is_occ_vec.setZero();
        transl.setZero();
        glob.setZero();
        rot.setOnes();
        last_depth_interps.fill(0.0);
        last_vecs.fill(V3F::Zero());
        last_positions.fill(Vector3i::Zero());
        is_distort = false;
        cur_vec.setZero();
        local.setZero();
        last_closest.setZero();
    };
    point_soph(const point_soph &cur)
    {
        vec = cur.vec;
        hor_ind = cur.hor_ind;
        ver_ind = cur.ver_ind;
        position = cur.position;
        time = cur.time;
        occu_times = cur.occu_times;
        is_occu_times = cur.is_occu_times;
        occu_index = cur.occu_index;
        is_occu_index = cur.is_occu_index;
        occ_vec = cur.occ_vec;
        is_occ_vec = cur.is_occ_vec;
        transl = cur.transl;
        glob = cur.glob;
        rot = cur.rot;
        dyn = cur.dyn;
        last_depth_interps = cur.last_depth_interps;
        last_vecs = cur.last_vecs;
        last_positions = cur.last_positions;
        local = cur.local;
        is_distort = cur.is_distort;
        cur_vec = cur.cur_vec;
        last_closest = cur.last_closest;
    };

    ~point_soph(){};

    /**
     * @brief 计算球面坐标
     *
     * @param point 载体系坐标
     * @param hor_resolution_max 水平分辨率(弧度制)
     * @param ver_resolution_max 垂直分辨率(弧度制)
     */
    void GetVec(V3D &point, float &hor_resolution_max, float &ver_resolution_max)
    {
        // 深度
        vec(2) = float(point.norm());
        // 水平角
        // atan2f(y, x) 计算点 (x,y) 相对于正X轴的角度，范围为 (-π, π]
        vec(0) = atan2f(float(point(1)), float(point(0)));
        // 俯仰角
        // 点到Z轴的投影长度是 sqrt(x^2 + y^2)，使用 atan2f(z, 投影长度) 计算俯仰角
        vec(1) = atan2f(float(point(2)), sqrt(pow(float(point(0)), 2) + pow(float(point(1)), 2)));
        // 根据水平分辨率计算水平索引
        // 水平角转换到 [0, 2π) 范围内，然后除以水平分辨率
        hor_ind = floor((vec(0) + PI_MATH) / hor_resolution_max);
        // 根据垂直分辨率计算俯仰索引
        // 俯仰角转换到 [0, π) 范围内，然后除以垂直分辨率
        ver_ind = floor((vec(1) + 0.5 * PI_MATH) / ver_resolution_max);
        // 计算该点的唯一位置标识，通常用于将点映射到一个一维数组或其他线性存储结构
        // 这里使用水平索引乘以某个固定的值（MAX_1D_HALF），然后加上俯仰索引
        position = hor_ind * MAX_1D_HALF + ver_ind;
    };

    void reset()
    {
        occu_times = is_occu_times = 0;
        occu_index = -1 * Vector3i::Ones();
        is_occu_index = -1 * Vector3i::Ones();
        occ_vec.setZero();
        is_occ_vec.setZero();
        last_closest.setZero();
        last_depth_interps.fill(0.0);
        last_vecs.fill(V3F::Zero());
        last_positions.fill(Vector3i::Zero());
        is_distort = false;
    };
};

// 深度图结构体
typedef std::vector<std::vector<point_soph *>> DepthMap2D;
class DepthMap
{
public:
    DepthMap2D depth_map; // 记录在像素索引处的点集合
    double time;
    int map_index; // 帧/深度图索引
    M3D project_R; // 旋转
    V3D project_T; // 平移
    std::vector<point_soph::Ptr> point_sopth_pointer;
    int point_sopth_pointer_count = 0;
    float *min_depth_all = nullptr;
    float *max_depth_all = nullptr;

    // 所有像素的静态深度范围
    float *max_depth_static = nullptr; // 静态点的最大深度，索引(水平索引*MAX_1D_HALF+俯仰索引)
    float *min_depth_static = nullptr; // 静态点的最小深度，索引(水平索引*MAX_1D_HALF+俯仰索引)
    int *max_depth_index_all = nullptr;
    int *min_depth_index_all = nullptr;
    std::vector<int> index_vector;
    typedef boost::shared_ptr<DepthMap> Ptr;

    DepthMap()
    {
        printf("build depth map2\n");
        depth_map.assign(MAX_2D_N, std::vector<point_soph *>());

        time = 0.;
        project_R.setIdentity(3, 3);
        project_T.setZero(3, 1);

        min_depth_static = new float[MAX_2D_N];
        min_depth_all = new float[MAX_2D_N];
        max_depth_all = new float[MAX_2D_N];
        max_depth_static = new float[MAX_2D_N];
        fill_n(min_depth_static, MAX_2D_N, 0.0);
        fill_n(min_depth_all, MAX_2D_N, 0.0);
        fill_n(max_depth_all, MAX_2D_N, 0.0);
        fill_n(max_depth_static, MAX_2D_N, 0.0);
        max_depth_index_all = new int[MAX_2D_N];
        min_depth_index_all = new int[MAX_2D_N];
        fill_n(min_depth_index_all, MAX_2D_N, -1);
        fill_n(max_depth_index_all, MAX_2D_N, -1);
        map_index = -1;
        index_vector.assign(MAX_2D_N, 0);
        for (int i = 0; i < MAX_2D_N; i++)
        {
            index_vector[i] = i;
        }
    }

    DepthMap(M3D rot, V3D transl, double cur_time, int frame)
    {
        depth_map.assign(MAX_2D_N, std::vector<point_soph *>());
        time = cur_time;
        project_R = rot;
        project_T = transl;
        min_depth_static = new float[MAX_2D_N];
        min_depth_all = new float[MAX_2D_N];
        max_depth_all = new float[MAX_2D_N];
        max_depth_static = new float[MAX_2D_N];
        fill_n(min_depth_static, MAX_2D_N, 0.0);
        fill_n(min_depth_all, MAX_2D_N, 0.0);
        fill_n(max_depth_all, MAX_2D_N, 0.0);
        fill_n(max_depth_static, MAX_2D_N, 0.0);
        max_depth_index_all = new int[MAX_2D_N];
        min_depth_index_all = new int[MAX_2D_N];
        fill_n(min_depth_index_all, MAX_2D_N, -1);
        fill_n(max_depth_index_all, MAX_2D_N, -1);
        map_index = frame;
        index_vector.assign(MAX_2D_N, 0);
        for (int i = 0; i < MAX_2D_N; i++)
        {
            index_vector[i] = i;
        }
    }

    DepthMap(const DepthMap &cur)
    {
        depth_map = cur.depth_map;
        time = cur.time;
        project_R = cur.project_R;
        project_T = cur.project_T;
        point_sopth_pointer = cur.point_sopth_pointer;
        min_depth_static = new float[MAX_2D_N];
        min_depth_all = new float[MAX_2D_N];
        max_depth_all = new float[MAX_2D_N];
        max_depth_static = new float[MAX_2D_N];
        fill_n(min_depth_static, MAX_2D_N, 0.0);
        fill_n(min_depth_all, MAX_2D_N, 0.0);
        fill_n(max_depth_all, MAX_2D_N, 0.0);
        fill_n(max_depth_static, MAX_2D_N, 0.0);
        max_depth_index_all = new int[MAX_2D_N];
        min_depth_index_all = new int[MAX_2D_N];
        map_index = cur.map_index;
        for (int i = 0; i < MAX_2D_N; i++)
        {
            min_depth_static[i] = cur.min_depth_static[i];
            max_depth_static[i] = cur.max_depth_static[i];
            min_depth_all[i] = cur.min_depth_all[i];
            max_depth_all[i] = cur.max_depth_all[i];
            max_depth_index_all[i] = cur.max_depth_index_all[i];
            min_depth_index_all[i] = cur.min_depth_index_all[i];
        }
        index_vector.assign(MAX_2D_N, 0);
        for (int i = 0; i < MAX_2D_N; i++)
        {
            index_vector[i] = i;
        }
    }
    ~DepthMap()
    {
        if (min_depth_static != nullptr)
            delete[] min_depth_static;
        if (min_depth_all != nullptr)
            delete[] min_depth_all;
        if (max_depth_all != nullptr)
            delete[] max_depth_all;
        if (max_depth_static != nullptr)
            delete[] max_depth_static;
        if (max_depth_index_all != nullptr)
            delete[] max_depth_index_all;
        if (min_depth_index_all != nullptr)
            delete[] min_depth_index_all;
    }

    void Reset(M3D rot, V3D transl, double cur_time, int frame)
    {
        time = cur_time;
        project_R = rot;
        project_T = transl;
        map_index = frame;
        double t = omp_get_wtime();
        std::for_each(std::execution::par, index_vector.begin(), index_vector.end(), [&](const int &i)
                      { depth_map[i].clear(); });
        fill_n(min_depth_static, MAX_2D_N, 0.0);
        fill_n(min_depth_all, MAX_2D_N, 0.0);
        fill_n(max_depth_all, MAX_2D_N, 0.0);
        fill_n(max_depth_static, MAX_2D_N, 0.0);
        fill_n(max_depth_index_all, MAX_2D_N, -1);
        fill_n(min_depth_index_all, MAX_2D_N, -1);
    }
};

class DynObjFilter
{
public:
    // -----------------------------通用参数-----------------------------------
    int case1_num = 0, case2_num = 0, case3_num = 0; // 三种情况的动态点计数
    // 车辆自身xy范围，这个范围不进行动态检测
    float self_x_f = 2.5, self_x_b = -1.5, self_y_l = 1.6, self_y_r = -1.6;
    float hor_resolution_max = 0.02f, ver_resolution_max = 0.02f; // 水平和垂直方向的分辨率(弧度制)
    // cutoff_value：截止值，确保计算出的深度差异阈值不会低于某个最小值
    // k_depth_min_thr1 和 k_depth_max_thr1：这两个参数是线性缩放因子，用于根据点的深度(p.vec(2))调整深度差异的最小和最大阈值。这允许系统根据点与传感器的距离动态调整阈值，以应对不同的观测条件。
    // d_depth_min_thr1 和 d_depth_max_thr1：这两个参数定义了深度阈值调整的基线，即在没有其他调整的情况下，深度差异阈值将基于这两个值进行计算。
    float k_depth_min_thr1 = 0.0, d_depth_min_thr1 = 50, cutoff_value = 0;
    float k_depth_max_thr1 = 0.0, d_depth_max_thr1 = 50, k_depth_max_thr2 = 0.0,
          d_depth_max_thr2 = 50, k_depth_max_thr3 = 0.0, d_depth_max_thr3 = 50;

    // ---------------------------情况1动态检测参数-----------------------------

    int depth_cons_hor_num1 = 0, depth_cons_ver_num1 = 0;

    // ---------------------------情况2动态检测参数-----------------------------
    int occluded_times_thr2 = 3; // 被过去点遮挡的次数阈值
    // 地图一致性检查的领域阈值:深度，水平角，垂直角
    float map_cons_depth_thr2 = 0.15f, map_cons_hor_thr2 = 0.02f, map_cons_ver_thr2 = 0.01f;
    // 地图一致性检查的领域水平角阈值和垂直角阈值/分辨率得到的领域点数
    int map_cons_hor_num2 = 0, map_cons_ver_num2 = 0;
    // 两点是否遮挡的邻域阈值:深度，水平角，垂直角
    float occ_depth_thr2 = 0.15f, occ_hor_thr2 = 0.02f, occ_ver_thr2 = 0.01f;
    // 两点是否遮挡的邻域水平角阈值和垂直角阈值/分辨率得到的领域点数
    float occ_hor_num2 = 0, occ_ver_num2 = 0;
    // 深度一致性检查的邻域阈值:水平角，垂直角
    float depth_cons_hor_thr2 = 0.02f, depth_cons_ver_thr2 = 0.01f;
    // 深度一致性检查的邻域水平角阈值和垂直角阈值/分辨率得到的领域点数
    int depth_cons_hor_num2 = 0, depth_cons_ver_num2 = 0;
    // 速度最小阈值，加速度最大阈值
    float v_min_thr2 = 0.5, acc_thr2 = 1.0;

    float depth_cons_depth_thr2 = 0.15f, depth_cons_depth_max_thr2 = 0.15f;

    // ---------------------------情况3动态检测参数-----------------------------

    // ---------------------------其他-----------------------------
    // 是否进行聚类操作
    bool cluster_coupled = false, cluster_future = false;
    std::deque<DepthMap::Ptr> depth_map_list;
    PARALLEL_Q<point_soph *> buffer;
    std::vector<point_soph *> point_soph_pointers; // 点云的每个点的详细信息数组
    int points_num_perframe = 200000;
    int cur_point_soph_pointers = 0;
    int max_pointers_num = 0;
    int frame_num_for_rec = 0;
    std::deque<PointCloudXYZI::Ptr> pcl_his_list;
    PointCloudXYZI::Ptr laserCloudSteadObj;         // 静态点云(世界系)
    PointCloudXYZI::Ptr laserCloudSteadObj_hist;
    PointCloudXYZI::Ptr laserCloudDynObj;           // 动态点云(载体系)
    PointCloudXYZI::Ptr laserCloudDynObj_world;     // 动态点云(世界系)
    PointCloudXYZI::Ptr laserCloudDynObj_clus;
    PointCloudXYZI::Ptr laserCloudSteadObj_clus;
    std::deque<PointCloudXYZI::Ptr> laserCloudSteadObj_accu;
    int laserCloudSteadObj_accu_times = 0;
    int laserCloudSteadObj_accu_limit = 5;
    float voxel_filter_size = 0.1;
    
    DynObjCluster Cluster;

        // 点云(非聚类)每个点的动态标签数组，0-静态，1-动态，
    std::vector<int> dyn_tag_origin;     
    // 点云(聚类后)每个点的动态标签数组，0-静态，1-动态，
    std::vector<int> dyn_tag_cluster;               
          
    bool draw_depth = false;

    float depth_thr1 = 0.15f, map_cons_depth_thr1 = 0.5f, map_cons_hor_thr1 = 0.02f, map_cons_ver_thr1 = 0.01f;
    float enter_min_thr1 = 2.0, enter_max_thr1 = 0.5;
    float map_cons_hor_dis1 = 0.2, map_cons_ver_dis1 = 0.2;
    // occluded_map_thr1：情况1被遮挡深度图数量阈值
    int map_cons_hor_num1 = 0, map_cons_ver_num1 = 0, occluded_map_thr1 = 3;
    bool case1_interp_en = false, case2_interp_en = false, case3_interp_en = false;
    float interp_hor_thr = 0.01f, interp_ver_thr = 0.01f, interp_thr1 = 1.0f, interp_thr2 = 1.0f, interp_thr3 = 1.0f;
    float interp_static_max = 10.0, interp_start_depth1 = 30, interp_kp1 = 0.1, interp_kd1 = 1.0;
    float interp_all_max = 100.0, interp_start_depth2 = 30, interp_kp2 = 0.1, interp_kd2 = 1.0;
    int interp_hor_num = 0, interp_ver_num = 0;
    float v_min_thr3 = 0.5, acc_thr3 = 1.0;

    float depth_cons_depth_thr1 = 0.15f, depth_cons_depth_max_thr1 = 1.0f, depth_cons_hor_thr1 = 0.02f, depth_cons_ver_thr1 = 0.01f;
    int occluding_times_thr3 = 3;
    float occ_depth_thr3 = 0.15f, occ_hor_thr3 = 0.02f, occ_ver_thr3 = 0.01f;
    float map_cons_depth_thr3 = 0.15f, map_cons_hor_thr3 = 0.02f, map_cons_ver_thr3 = 0.01f;
    float depth_cons_depth_thr3 = 0.15f, depth_cons_depth_max_thr3 = 0.15f, depth_cons_hor_thr3 = 0.02f, depth_cons_ver_thr3 = 0.01f, k_depth2 = 0.005, k_depth3 = 0.005;
    int map_cons_hor_num3 = 0, map_cons_ver_num3 = 0, occ_hor_num3 = 0, occ_ver_num3 = 0, depth_cons_hor_num3 = 0, depth_cons_ver_num3 = 0;
    // enlarge_depth远距离的深度差异阈值系数
    float enlarge_z_thr1 = 0.05, enlarge_angle = 2, enlarge_depth = 3;
    int enlarge_distort = 4; // 去畸变的深度差异阈值系数
    int checkneighbor_range = 1;

    double frame_dur = 0.1, buffer_delay = 0.1, depth_map_dur = 0.2f;
    int buffer_size = 300000, max_depth_map_num = 5;
    int hor_num = MAX_1D, ver_num = MAX_1D_HALF;

    int occu_time_th = 3, is_occu_time_th = 3, map_index = 0;

    double time_interp1 = 0.0, time_interp2 = 0.0;
    double time_search = 0.0, time_search_0 = 0.0, time_research = 0.0, time_build = 0.0, time_other0 = 0.0, time_total = 0.0, time_total_avr = 0.0;
    float buffer_time = 0.0f, buffer_dur = 0.1f;
    int point_index = 0, time_ind = 0, max_ind = 1257, occlude_windows = 3;
    bool debug_en = false; // 是否开启调试模式
    int roll_num = 700, pitch_num = 350;
    int dataset = 0; // 数据集标识符

    float fov_up = 2.0, fov_down = -23, fov_cut = -20, fov_left = 180, fov_right = -180;
    float blind_dis = 0.3; // 扫描盲区距离
    int pixel_fov_up, pixel_fov_down, pixel_fov_cut, pixel_fov_left, pixel_fov_right;
    int max_pixel_points = 50;
    bool stop_object_detect = false; // 是否进行物体检测
    // point_soph::Ptr last_point_pointer = nullptr;
    string frame_id = "camera_init";
    double invalid_total = 0.0;
    double case1_total = 0.0;
    double case2_total = 0.0;
    double case3_total = 0.0;
    bool dyn_filter_en = true;
    // mutex mtx_case2, mtx_case3;
    std::vector<int> pos_offset;
    // ros::Publisher demo_pcl_display;
    string time_file;  // 时间日志文件
    ofstream time_out; // 时间日志文件流

    // 性能分析的时间容器
    std::vector<double> time_test1, time_test2, time_test3, time_occ_check, time_map_cons, time_proj;
    string time_breakdown_file;
    ofstream time_breakdown_out;

    // 默认构造
    DynObjFilter(){};
    // 有参构造
    DynObjFilter(float windows_dur, float hor_resolution, float ver_resolution)
        : depth_map_dur(windows_dur), hor_resolution_max(hor_resolution), ver_resolution_max(ver_resolution){};
    // 默认析构
    ~DynObjFilter(){};

    // 初始化函数
    void init(ros::NodeHandle &nh);
    // 主函数
    void filter(PointCloudXYZI::Ptr feats_undistort, const M3D &rot_end, const V3D &pos_end, const double &scan_end_time);


    // -------------------------------通用函数-----------------------------------
    void SphericalProjection(point_soph &p, int depth_index, const M3D &rot, const V3D &transl, point_soph &p_spherical);
    float DepthInterpolationAll(point_soph &p, int map_index, const DepthMap2D &depth_map);

    // -------------------------------情况1动态点检测---------------------------
    bool Case1(point_soph &p);
    bool Case1Enter(const point_soph &p, const DepthMap &map_info);
    bool CheckVerFoV(const point_soph &p, const DepthMap &map_info);
    void CheckNeighbor(const point_soph &p, const DepthMap &map_info, float &max_depth, float &min_depth);
    bool Case1FalseRejection(point_soph &p, const DepthMap &map_info);
    bool Case1MapConsistencyCheck(point_soph &p, const DepthMap &map_info, bool interp);
    float DepthInterpolationStatic(point_soph &p, int map_index, const DepthMap2D &depth_map);

    // -------------------------------情况2动态点检测---------------------------
    bool Case2(point_soph &p);
    bool Case2Enter(point_soph &p, const DepthMap &map_info);
    bool Case2MapConsistencyCheck(point_soph &p, const DepthMap &map_info, bool interp);
    bool Case2DepthConsistencyCheck(const point_soph &p, const DepthMap &map_info);
    bool Case2SearchPointOccludingP(point_soph &p, const DepthMap &map_info);
    bool Case2IsOccluded(const point_soph &p, const point_soph &p_occ);
    bool Case2VelCheck(float v1, float v2, double delta_t);
    bool InvalidPointCheck(const V3D &body, const int intensity);
    bool SelfPointCheck(const V3D &body);
    // -------------------------------情况3动态点检测---------------------------
    bool Case3(point_soph &p);
    bool Case3Enter(point_soph &p, const DepthMap &map_info);
    bool Case3MapConsistencyCheck(point_soph &p, const DepthMap &map_info, bool interp);
    bool Case3SearchPointOccludedbyP(point_soph &p, const DepthMap &map_info);
    bool Case3IsOccluding(const point_soph &p, const point_soph &p_occ);
    bool Case3DepthConsistencyCheck(const point_soph &p, const DepthMap &map_info);
    bool Case3VelCheck(float v1, float v2, double delta_t);
    // -------------------------------其他------------------------------------
      void set_path(string file_path, string file_path_origin);
    void  Points2Buffer(vector<point_soph*> &points, std::vector<int> &index_vector);
    void  Buffer2DepthMap(double cur_time);
        void publish_dyn(const ros::Publisher & pub_point_out, const ros::Publisher & pub_frame_out, const ros::Publisher & pub_steady_points, const double & scan_end_time);
  

private:
    // 是否设置输出路径
    bool is_set_path = false;
    // 输出路径
    string out_file;            // 聚类后的动态标签文件保存路径
    string out_file_origin;     // 聚类前的动态标签文件保存路径
};

#endif