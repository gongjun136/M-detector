/*
 * @Author: gongjun136 gongjun136@gmail.com
 * @Date: 2024-03-12 09:49:25
 * @LastEditors: gongjun136 gongjun136@gmail.com
 * @LastEditTime: 2024-03-22 10:52:39
 * @FilePath: /catkin_ws_M-detector/src/M-detector-noted/src/dyn_obj_filter.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <vector>
#include <random>
#include "m-detector-noted/dyn_obj_filter.h"

void DynObjFilter::init(ros::NodeHandle &nh)
{
    // 1. 从 ROS 参数服务器加载动态对象过滤相关的配置参数
    // 这些参数包括缓冲延迟、缓冲区大小、每帧点数、深度图持续时间等
    nh.param<double>("dyn_obj/buffer_delay", buffer_delay, 0.1);
    nh.param<int>("dyn_obj/buffer_size", buffer_size, 300000);
    nh.param<int>("dyn_obj/points_num_perframe", points_num_perframe, 150000);
    nh.param<double>("dyn_obj/depth_map_dur", depth_map_dur, 0.2);
    nh.param<int>("dyn_obj/max_depth_map_num", max_depth_map_num, 5);
    nh.param<int>("dyn_obj/max_pixel_points", max_pixel_points, 50);
    nh.param<double>("dyn_obj/frame_dur", frame_dur, 0.1);
    nh.param<int>("dyn_obj/dataset", dataset, 0);
    nh.param<float>("dyn_obj/self_x_f", self_x_f, 0.15f);
    nh.param<float>("dyn_obj/self_x_b", self_x_b, 0.15f);
    nh.param<float>("dyn_obj/self_y_l", self_y_l, 0.15f);
    nh.param<float>("dyn_obj/self_y_r", self_y_r, 0.5f);
    nh.param<float>("dyn_obj/blind_dis", blind_dis, 0.15f);
    // -----------------------视场相关的参数--------------------------
    nh.param<float>("dyn_obj/fov_up", fov_up, 0.15f);         // 视场的上界（度）
    nh.param<float>("dyn_obj/fov_down", fov_down, 0.15f);     // 视场的下界（度）
    nh.param<float>("dyn_obj/fov_left", fov_left, 180.0f);    // 视场的左界（度）
    nh.param<float>("dyn_obj/fov_right", fov_right, -180.0f); // 视场的右界（度）
    nh.param<float>("dyn_obj/fov_cut", fov_cut, 0.15f);       // 用于某些特定裁剪操作的视场参数

    nh.param<int>("dyn_obj/checkneighbor_range", checkneighbor_range, 1);
    nh.param<bool>("dyn_obj/stop_object_detect", stop_object_detect, false);
    nh.param<float>("dyn_obj/depth_thr1", depth_thr1, 0.15f);
    nh.param<float>("dyn_obj/enter_min_thr1", enter_min_thr1, 0.15f);
    nh.param<float>("dyn_obj/enter_max_thr1", enter_max_thr1, 0.15f);
    nh.param<float>("dyn_obj/map_cons_depth_thr1", map_cons_depth_thr1, 0.5f);
    nh.param<float>("dyn_obj/map_cons_hor_thr1", map_cons_hor_thr1, 0.01f);
    nh.param<float>("dyn_obj/map_cons_ver_thr1", map_cons_ver_thr1, 0.01f);
    nh.param<float>("dyn_obj/map_cons_hor_dis1", map_cons_hor_dis1, 0.2f);
    nh.param<float>("dyn_obj/map_cons_ver_dis1", map_cons_ver_dis1, 0.1f);
    nh.param<float>("dyn_obj/depth_cons_depth_thr1", depth_cons_depth_thr1, 0.5f);
    nh.param<float>("dyn_obj/depth_cons_depth_max_thr1", depth_cons_depth_max_thr1, 0.5f);
    nh.param<float>("dyn_obj/depth_cons_hor_thr1", depth_cons_hor_thr1, 0.02f);
    nh.param<float>("dyn_obj/depth_cons_ver_thr1", depth_cons_ver_thr1, 0.01f);
    nh.param<float>("dyn_obj/enlarge_z_thr1", enlarge_z_thr1, 0.05f);
    nh.param<float>("dyn_obj/enlarge_angle", enlarge_angle, 2.0f);
    nh.param<float>("dyn_obj/enlarge_depth", enlarge_depth, 3.0f);
    nh.param<int>("dyn_obj/occluded_map_thr1", occluded_map_thr1, 3);
    nh.param<bool>("dyn_obj/case1_interp_en", case1_interp_en, false);
    nh.param<float>("dyn_obj/k_depth_min_thr1", k_depth_min_thr1, 0.0f);
    nh.param<float>("dyn_obj/d_depth_min_thr1", d_depth_min_thr1, 0.15f);
    nh.param<float>("dyn_obj/k_depth_max_thr1", k_depth_max_thr1, 0.0f);
    nh.param<float>("dyn_obj/d_depth_max_thr1", d_depth_max_thr1, 0.15f);
    nh.param<float>("dyn_obj/v_min_thr2", v_min_thr2, 0.5f);
    nh.param<float>("dyn_obj/acc_thr2", acc_thr2, 1.0f);
    nh.param<float>("dyn_obj/map_cons_depth_thr2", map_cons_depth_thr2, 0.15f);
    nh.param<float>("dyn_obj/map_cons_hor_thr2", map_cons_hor_thr2, 0.02f);
    nh.param<float>("dyn_obj/map_cons_ver_thr2", map_cons_ver_thr2, 0.01f);
    nh.param<float>("dyn_obj/occ_depth_thr2", occ_depth_thr2, 0.15f);
    nh.param<float>("dyn_obj/occ_hor_thr2", occ_hor_thr2, 0.02f);
    nh.param<float>("dyn_obj/occ_ver_thr2", occ_ver_thr2, 0.01f);
    nh.param<float>("dyn_obj/depth_cons_depth_thr2", depth_cons_depth_thr2, 0.5f);
    nh.param<float>("dyn_obj/depth_cons_depth_max_thr2", depth_cons_depth_max_thr2, 0.5f);
    nh.param<float>("dyn_obj/depth_cons_hor_thr2", depth_cons_hor_thr2, 0.02f);
    nh.param<float>("dyn_obj/depth_cons_ver_thr2", depth_cons_ver_thr2, 0.01f);
    nh.param<float>("dyn_obj/k_depth2", k_depth2, 0.005f);
    nh.param<int>("dyn_obj/occluded_times_thr2", occluded_times_thr2, 3);
    nh.param<bool>("dyn_obj/case2_interp_en", case2_interp_en, false);
    nh.param<float>("dyn_obj/k_depth_max_thr2", k_depth_max_thr2, 0.0f);
    nh.param<float>("dyn_obj/d_depth_max_thr2", d_depth_max_thr2, 0.15f);
    nh.param<float>("dyn_obj/v_min_thr3", v_min_thr3, 0.5f);
    nh.param<float>("dyn_obj/acc_thr3", acc_thr3, 1.0f);
    nh.param<float>("dyn_obj/map_cons_depth_thr3", map_cons_depth_thr3, 0.15f);
    nh.param<float>("dyn_obj/map_cons_hor_thr3", map_cons_hor_thr3, 0.02f);
    nh.param<float>("dyn_obj/map_cons_ver_thr3", map_cons_ver_thr3, 0.01f);
    nh.param<float>("dyn_obj/occ_depth_thr3", occ_depth_thr3, 0.15f);
    nh.param<float>("dyn_obj/occ_hor_thr3", occ_hor_thr3, 0.02f);
    nh.param<float>("dyn_obj/occ_ver_thr3", occ_ver_thr3, 0.01f);
    nh.param<float>("dyn_obj/depth_cons_depth_thr3", depth_cons_depth_thr3, 0.5f);
    nh.param<float>("dyn_obj/depth_cons_depth_max_thr3", depth_cons_depth_max_thr3, 0.5f);
    nh.param<float>("dyn_obj/depth_cons_hor_thr3", depth_cons_hor_thr3, 0.02f);
    nh.param<float>("dyn_obj/depth_cons_ver_thr3", depth_cons_ver_thr3, 0.01f);
    nh.param<float>("dyn_obj/k_depth3", k_depth3, 0.005f);
    nh.param<int>("dyn_obj/occluding_times_thr3", occluding_times_thr3, 3);
    nh.param<bool>("dyn_obj/case3_interp_en", case3_interp_en, false);
    nh.param<float>("dyn_obj/k_depth_max_thr3", k_depth_max_thr3, 0.0f);
    nh.param<float>("dyn_obj/d_depth_max_thr3", d_depth_max_thr3, 0.15f);
    nh.param<float>("dyn_obj/interp_hor_thr", interp_hor_thr, 0.01f);
    nh.param<float>("dyn_obj/interp_ver_thr", interp_ver_thr, 0.01f);
    nh.param<float>("dyn_obj/interp_thr1", interp_thr1, 1.0f);
    nh.param<float>("dyn_obj/interp_static_max", interp_static_max, 10.0f);
    nh.param<float>("dyn_obj/interp_start_depth1", interp_start_depth1, 20.0f);
    nh.param<float>("dyn_obj/interp_kp1", interp_kp1, 0.1f);
    nh.param<float>("dyn_obj/interp_kd1", interp_kd1, 1.0f);
    nh.param<float>("dyn_obj/interp_thr2", interp_thr2, 0.15f);
    nh.param<float>("dyn_obj/interp_thr3", interp_thr3, 0.15f);
    nh.param<bool>("dyn_obj/dyn_filter_en", dyn_filter_en, true);
    nh.param<bool>("dyn_obj/debug_publish", debug_en, true);
    nh.param<int>("dyn_obj/laserCloudSteadObj_accu_limit", laserCloudSteadObj_accu_limit, 5);
    nh.param<float>("dyn_obj/voxel_filter_size", voxel_filter_size, 0.1f);
    nh.param<bool>("dyn_obj/cluster_coupled", cluster_coupled, false);
    nh.param<bool>("dyn_obj/cluster_future", cluster_future, false);
    nh.param<float>("dyn_obj/ver_resolution_max", hor_resolution_max, 0.0025f);
    nh.param<float>("dyn_obj/hor_resolution_max", ver_resolution_max, 0.0025f);
    nh.param<float>("dyn_obj/buffer_dur", buffer_dur, 0.1f);
    nh.param<int>("dyn_obj/point_index", point_index, 0);
    nh.param<string>("dyn_obj/frame_id", frame_id, "camera_init");
    nh.param<string>("dyn_obj/time_file", time_file, "");
    nh.param<string>("dyn_obj/time_breakdown_file", time_breakdown_file, "");
    max_ind = floor(3.1415926 * 2 / hor_resolution_max);
    // LOG(INFO) << "hor_resolution_max: " << hor_resolution_max << ",ver_resolution_max: " << ver_resolution_max;

    // 加载配置参数给DynObjFilter的Cluster类
    nh.param<int>("dyn_obj/cluster_extend_pixel", Cluster.cluster_extend_pixel, 2);
    nh.param<int>("dyn_obj/cluster_min_pixel_number", Cluster.cluster_min_pixel_number, 4);
    nh.param<float>("dyn_obj/cluster_thrustable_thresold", Cluster.thrustable_thresold, 0.3f);
    nh.param<float>("dyn_obj/cluster_Voxel_revolusion", Cluster.Voxel_revolusion, 0.3f);
    nh.param<bool>("dyn_obj/cluster_debug_en", Cluster.debug_en, false);
    nh.param<string>("dyn_obj/cluster_out_file", Cluster.out_file, "");

    // 2. 初始化点云历史列表，用于存储过去收集的点云数据
    if (pcl_his_list.size() == 0)
    {
        PointCloudXYZI::Ptr first_frame(new PointCloudXYZI());
        first_frame->reserve(400000);
        pcl_his_list.push_back(first_frame);
        laserCloudSteadObj_hist = PointCloudXYZI::Ptr(new PointCloudXYZI());
        laserCloudSteadObj = PointCloudXYZI::Ptr(new PointCloudXYZI());
        laserCloudDynObj = PointCloudXYZI::Ptr(new PointCloudXYZI());
        laserCloudDynObj_world = PointCloudXYZI::Ptr(new PointCloudXYZI());
        // 将点云数据转换为深度图像的偏移量序列
        // pos_offset输出为[0,1,-1,2,-2,3,-3,...]
        int xy_ind[3] = {-1, 1};
        for (int ind_hor = 0; ind_hor < 2 * hor_num + 1; ind_hor++)
        {
            for (int ind_ver = 0; ind_ver < 2 * ver_num + 1; ind_ver++)
            {
                int offset = ((ind_hor) / 2 + ind_hor % 2) * xy_ind[ind_hor % 2] * MAX_1D_HALF + ((ind_ver) / 2 + ind_ver % 2) * xy_ind[ind_ver % 2];
                pos_offset.push_back(offset);
            }
        }
        // LOG(INFO)<<"pos_offset size: "<<pos_offset.size();
    }

    // 3. 根据加载的参数初始化成员变量，例如处理深度图的各种阈值和分辨率
    map_cons_hor_num1 = ceil(map_cons_hor_thr1 / hor_resolution_max);
    map_cons_ver_num1 = ceil(map_cons_ver_thr1 / ver_resolution_max);
    interp_hor_num = ceil(interp_hor_thr / hor_resolution_max);
    interp_ver_num = ceil(interp_ver_thr / ver_resolution_max);
    map_cons_hor_num2 = ceil(map_cons_hor_thr2 / hor_resolution_max);
    map_cons_ver_num2 = ceil(map_cons_ver_thr2 / ver_resolution_max);
    occ_hor_num2 = ceil(occ_hor_thr2 / hor_resolution_max);
    occ_ver_num2 = ceil(occ_ver_thr2 / ver_resolution_max);
    depth_cons_hor_num2 = ceil(depth_cons_hor_thr2 / hor_resolution_max);
    depth_cons_ver_num2 = ceil(depth_cons_ver_thr2 / ver_resolution_max);
    map_cons_hor_num3 = ceil(map_cons_hor_thr3 / hor_resolution_max);
    map_cons_ver_num3 = ceil(map_cons_ver_thr3 / ver_resolution_max);
    occ_hor_num3 = ceil(occ_hor_thr3 / hor_resolution_max);
    occ_ver_num3 = ceil(occ_ver_thr3 / ver_resolution_max);
    depth_cons_hor_num3 = ceil(depth_cons_hor_thr3 / hor_resolution_max);
    depth_cons_ver_num3 = ceil(depth_cons_ver_thr3 / ver_resolution_max);
    buffer.init(buffer_size);

    // -------------------------视场参数计算像素级的视场范围索引--------------------------
    // 这些索引将用于后续处理中确定点是否位于有效的视场内
    pixel_fov_up = floor((fov_up * kDEG2RAD + 0.5 * PI_MATH) / ver_resolution_max);
    pixel_fov_down = floor((fov_down * kDEG2RAD + 0.5 * PI_MATH) / ver_resolution_max);
    pixel_fov_cut = floor((fov_cut * kDEG2RAD + 0.5 * PI_MATH) / ver_resolution_max);
    pixel_fov_left = floor((fov_left * kDEG2RAD + PI_MATH) / hor_resolution_max);
    pixel_fov_right = floor((fov_right * kDEG2RAD + PI_MATH) / hor_resolution_max);
    // LOG(INFO) << "pixel_fov_up: " << pixel_fov_up << ", pixel_fov_down: " << pixel_fov_down
    //           << ", pixel_fov_left: " << pixel_fov_left << ", pixel_fov_right: " << pixel_fov_right;

    max_pointers_num = round((max_depth_map_num * depth_map_dur + buffer_delay) / frame_dur) + 1;
    // 4. 分配和初始化处理点云所需的内存和数据结构，例如点云的每个点的详细信息数组
    point_soph_pointers.reserve(max_pointers_num);
    for (int i = 0; i < max_pointers_num; i++)
    {
        point_soph *p = new point_soph[points_num_perframe];
        point_soph_pointers.push_back(p);
    }

    // 5. 如果指定了时间文件，准备时间日志的文件流
    if (time_file != "")
    {
        time_out.open(time_file, ios::out);
    }

    if (time_breakdown_file != "")
    {
        time_breakdown_out.open(time_breakdown_file, ios::out);
    }
    // 6. 调用 DynObjCluster 的 Init 函数，初始化点云聚类相关的参数和数据结构
    Cluster.Init();
    return;
}

void DynObjFilter::set_path(string file_path, string file_path_origin)
{
    is_set_path = true;
    out_file = file_path;
    out_file_origin = file_path_origin;
}

/**
 * @brief 动态物体检测
 *
 * @param feats_undistort 当前点云
 * @param rot_end 旋转
 * @param pos_end 平移
 * @param scan_end_time 时间错
 */
void DynObjFilter::filter(PointCloudXYZI::Ptr feats_undistort, const M3D &rot_end, const V3D &pos_end, const double &scan_end_time)
{
    double t00 = omp_get_wtime(); // 记录开始时间
    // 初始化时间和统计变量
    time_search = time_research = time_search_0 = time_build = time_total = time_other0 = time_interp1 = time_interp2 = 0.0;
    int num_build = 0, num_search_0 = 0, num_research = 0;
    // 如果输入点云为空，则直接返回
    if (feats_undistort == NULL)
        return;
    int size = feats_undistort->points.size(); // 点云大小

    // 如果开启调试模式，则初始化历史点云存储空间
    if (debug_en)
    {
        laserCloudSteadObj_hist.reset(new PointCloudXYZI());
        laserCloudSteadObj_hist->reserve(20 * size);
    }

    // 初始化用于存储点云动态性分类的容器
    dyn_tag_origin.clear();
    dyn_tag_origin.reserve(size);
    dyn_tag_origin.resize(size);
    dyn_tag_cluster.clear();
    dyn_tag_cluster.reserve(size);
    dyn_tag_cluster.resize(size);
    laserCloudDynObj.reset(new PointCloudXYZI());

    // 初始化动态对象和静态对象的点云存储空间
    laserCloudDynObj->reserve(size);
    laserCloudDynObj_world.reset(new PointCloudXYZI());
    laserCloudDynObj_world->reserve(size);
    laserCloudSteadObj.reset(new PointCloudXYZI());
    laserCloudSteadObj->reserve(size);
    laserCloudDynObj_clus.reset(new PointCloudXYZI());
    laserCloudDynObj_clus->reserve(size);
    laserCloudSteadObj_clus.reset(new PointCloudXYZI());
    laserCloudSteadObj_clus->reserve(size);

    // 准备输出文件
    ofstream out;
    ofstream out_origin;
    bool is_rec = false;
    bool is_rec_origin = false;
    if (is_set_path)
    {
        out.open(out_file, ios::out | ios::binary);
        out_origin.open(out_file_origin, ios::out | ios::binary);
        if (out.is_open())
        {
            is_rec = true;
        }
        if (out_origin.is_open())
        {
            is_rec_origin = true;
        }
    }
    // 为性能分析准备时间统计容器
    time_test1.reserve(size);
    time_test1.resize(size);
    time_test2.reserve(size);
    time_test2.resize(size);
    time_test3.reserve(size);
    time_test3.resize(size);
    time_occ_check.reserve(size);
    time_occ_check.resize(size);
    time_map_cons.reserve(size);
    time_map_cons.resize(size);
    time_proj.reserve(size);
    time_proj.resize(size);
    for (int i = 0; i < size; i++)
    {
        time_test1[i] = 0.0;
        time_test2[i] = 0.0;
        time_test3[i] = 0.0;
        time_occ_check[i] = 0.0;
        time_map_cons[i] = 0.0;
        time_proj[i] = 0.0;
    }
    int case2_num = 0;
    // 记录开始时间
    double t0 = omp_get_wtime();
    double time_case1 = 0, time_case2 = 0, time_case3 = 0;
    // 载体系的原始点云
    pcl::PointCloud<PointType> raw_points_world;
    raw_points_world.reserve(size);
    raw_points_world.resize(size);
    std::vector<int> index(size); // 点云每个点的索引
    for (int i = 0; i < size; i++)
    {
        index[i] = i;
    }
    vector<point_soph *> points;
    points.reserve(size);
    points.resize(size);
    point_soph *p = point_soph_pointers[cur_point_soph_pointers];

    // 在时间日志中记录当前帧点数
    if (time_file != "")
        time_out << size << " "; // rec computation time

    // 并行遍历点云，对每个点进行动态性分类
    std::for_each(std::execution::par, index.begin(), index.end(), [&](const int &i)
                  // std::for_each(std::execution::seq, index.begin(), index.end(), [&](const int &i)
                  {
        // 获取当前点，并计算世界系坐标
        p[i].reset();
        V3D p_body(feats_undistort->points[i].x, feats_undistort->points[i].y, feats_undistort->points[i].z);
        int intensity = feats_undistort->points[i].curvature;
        V3D p_glob(rot_end * (p_body) + pos_end);
        p[i].glob = p_glob;
        p[i].dyn = STATIC;      // 默认为静态c
        p[i].rot = rot_end.transpose();
        p[i].transl = pos_end;
        p[i].time = scan_end_time;
        p[i].local = p_body;
        p[i].intensity = feats_undistort->points[i].intensity;

        // kitti特殊处理，判断是否失真
        // 0 for kitti, 1 for nuscenes, 2 for waymo, 3 for avia
        if(dataset == 0 && fabs(intensity-666) < 10E-4)
        {
            // 对于KITTI数据集作者可能做了特别处理,把失真的点intensity设置为666
            p[i].is_distort = true;
        }
        // 检查点是否为无效点
        if (InvalidPointCheck(p_body, intensity))
        {
            p[i].dyn = INVALID;
            dyn_tag_origin[i] = 0;
            dyn_tag_cluster[i] = -1;
        }
        else if(SelfPointCheck(p_body))
        {
            // 检查点是否为机器人自身的一部分，如果是，则更新状态
            p[i].dyn = INVALID;
            dyn_tag_origin[i] = 0;
        }
        else if (Case1(p[i]))
        {
            // 第一种情况：
            p[i].dyn = CASE1;
            dyn_tag_origin[i] = 1;
        }
        else if (Case2(p[i]))
        {
            p[i].dyn = CASE2;
            dyn_tag_origin[i] = 1;
        }
        else if(Case3(p[i]))
        {
            p[i].dyn = CASE3;
            dyn_tag_origin[i] = 1;
        }
        else
        {
            // 如果以上情况都不满足，则认为是静态点
            dyn_tag_origin[i] = 0;
        }
        // 保存处理后的点，供后续步骤使用
        points[i] = &p[i]; });

    // 在时间日志中记录三种情况动态点检测花费时间
    if (time_file != "")
        time_out << omp_get_wtime() - t0 << " "; // rec computation time
    // 遍历处理后的点，根据其动态状态分类存储
    for (int i = 0; i < size; i++)
    {
        // 创建点类型变量，用于存储转换后的点云数据
        // 创建载体坐标系下的点
        PointType po;
        po.x = points[i]->local[0];
        po.y = points[i]->local[1];
        po.z = points[i]->local[2];
        po.intensity = points[i]->intensity;
        // 创建世界坐标系下的点
        PointType po_w;
        po_w.x = points[i]->glob[0];
        po_w.y = points[i]->glob[1];
        po_w.z = points[i]->glob[2];
        // 将转换后的点添加到原始点云中
        raw_points_world[i] = po;
        // 根据点的动态状态分类存储
        switch (points[i]->dyn)
        {
        case CASE1:          // 情况1动态点
            po.normal_x = 1; // 使用normal_x字段标记CASE1
            laserCloudDynObj->push_back(po);
            laserCloudDynObj_world->push_back(po_w);
            break;
        case CASE2:                              // 情况2动态点
            po.normal_y = points[i]->occu_times; // 使用normal_y字段记录遮挡次数
            laserCloudDynObj->push_back(po);
            laserCloudDynObj_world->push_back(po_w);
            break;
        case CASE3:                                 // 情况3动态点
            po.normal_z = points[i]->is_occu_times; // 使用normal_z字段记录遮挡次数
            laserCloudDynObj->push_back(po);
            laserCloudDynObj_world->push_back(po_w);
            break;
        default:                                 // 静态点
            laserCloudSteadObj->push_back(po_w); // 只将全局坐标系下的点添加到稳定点云中
        }
    }
    int num_1 = 0, num_2 = 0, num_3 = 0, num_inval = 0, num_neag = 0;
    // 记录聚类前的时间，用于计算聚类过程的时间开销
    double clus_before = omp_get_wtime(); // rec computation time
    // 准备消息头，用于后续的点云发布
    std_msgs::Header header_clus;
    header_clus.stamp = ros::Time().fromSec(scan_end_time); // 设置时间戳
    header_clus.frame_id = frame_id;                        // 设置坐标系ID

    // 如果启用了联合聚类或者未来帧聚类
    if (cluster_coupled || cluster_future)
    {
        // 聚类与区域生长
        Cluster.Clusterprocess(dyn_tag_cluster, *laserCloudDynObj, raw_points_world, header_clus, rot_end, pos_end);

        // 遍历所有点，根据聚类结果更新点的动态状态和分类
        for (int i = 0; i < size; i++)
        {
            PointType po;
            po.x = points[i]->glob(0); // 使用点的全局坐标
            po.y = points[i]->glob(1);
            po.z = points[i]->glob(2);
            po.curvature = i; // 使用curvature字段暂存点的索引
            switch (points[i]->dyn)
            {
            case CASE1:                      // 情况1动态点
                                             // 根据聚类结果判断点是否仍然被认为是动态的
                if (dyn_tag_cluster[i] == 0) // 情况1动态点
                {
                    // 更新点的状态为静态
                    points[i]->dyn = STATIC;
                    points[i]->occu_times = -1;
                    points[i]->is_occu_times = -1;
                    // 将点添加到静态对象的点云中
                    po.normal_x = 0;
                    po.normal_y = points[i]->is_occu_times;
                    po.normal_z = points[i]->occu_times;
                    po.intensity = (int)(points[i]->local.norm() * 10) + 10;
                    laserCloudSteadObj_clus->push_back(po);
                    num_neag += 1;
                }    // 如果点仍然被认为是动态的
                else // case1
                {
                    // 将点添加到动态对象的点云中
                    po.normal_x = 1;
                    po.normal_y = points[i]->is_occu_times;
                    po.normal_z = points[i]->occu_times;
                    laserCloudDynObj_clus->push_back(po);
                    if (!dyn_filter_en)
                    {
                        po.intensity = (int)(points[i]->local.norm() * 10) + 10;
                    }
                    num_1 += 1;
                }
                break;
            case CASE2: // 情况2动态点
                if (dyn_tag_cluster[i] == 0)
                {
                    points[i]->dyn = STATIC;
                    points[i]->occu_times = -1;
                    points[i]->is_occu_times = -1;
                    po.normal_x = 0;
                    po.normal_y = points[i]->is_occu_times;
                    po.normal_z = points[i]->occu_times;
                    po.intensity = (int)(points[i]->local.norm() * 10) + 10;
                    laserCloudSteadObj_clus->push_back(po);
                    num_neag += 1;
                }
                else
                {
                    po.normal_x = 0;
                    po.normal_y = points[i]->is_occu_times;
                    po.normal_z = points[i]->occu_times;
                    laserCloudDynObj_clus->push_back(po);
                    if (!dyn_filter_en)
                    {
                        po.intensity = (int)(points[i]->local.norm() * 10) + 10;
                    }
                    num_2 += 1;
                }
                break;
            case CASE3: // 情况3动态点
                if (dyn_tag_cluster[i] == 0)
                {
                    points[i]->dyn = STATIC;
                    points[i]->occu_times = -1;
                    points[i]->is_occu_times = -1;
                    po.normal_x = 0;
                    po.normal_y = points[i]->is_occu_times;
                    po.normal_z = points[i]->occu_times;
                    po.intensity = (int)(points[i]->local.norm() * 10) + 10;
                    laserCloudSteadObj_clus->push_back(po);
                    num_neag += 1;
                }
                else
                {
                    po.normal_x = 0;
                    po.normal_y = points[i]->is_occu_times;
                    po.normal_z = points[i]->occu_times;
                    laserCloudDynObj_clus->push_back(po);
                    if (!dyn_filter_en)
                    {
                        po.intensity = (int)(points[i]->local.norm() * 10) + 10;
                    }
                    num_3 += 1;
                }
                break;
            case STATIC: // 静态点
                         // 如果聚类结果将静态点重新分类为动态
                if (dyn_tag_cluster[i] == 1)
                {
                    // 以CASE1处理
                    points[i]->dyn = CASE1;
                    points[i]->occu_times = -1;
                    points[i]->is_occu_times = -1;
                    po.normal_x = 0;
                    po.normal_y = points[i]->is_occu_times;
                    po.normal_z = points[i]->occu_times;
                    laserCloudDynObj_clus->push_back(po);
                    if (!dyn_filter_en)
                    {
                        po.intensity = (int)(points[i]->local.norm() * 10) + 10;
                    }
                    num_1 += 1;
                }
                else
                {
                    // 将静态点添加到静态对象的点云中
                    po.normal_x = 0;
                    po.normal_y = points[i]->is_occu_times;
                    po.normal_z = points[i]->occu_times;
                    po.intensity = (int)(points[i]->local.norm() * 10) + 10;
                    laserCloudSteadObj_clus->push_back(po);
                    num_neag += 1;
                }
                break;
            default: // 无效点
                num_inval += 1;
                break;
            }
        }
    }
    // 如果指定了时间记录文件，记录聚类处理结束后到当前时刻的时间差
    if (time_file != "")
        time_out << omp_get_wtime() - clus_before << " "; // 记录聚类处理的计算时间
    double t3 = omp_get_wtime();
    // 将处理后的点云数据转换为某种缓冲区格式，为后续步骤做准备
    Points2Buffer(points, index);
    double t4 = omp_get_wtime();
    // 如果指定了时间记录文件，记录点云数据转换为缓冲区格式的处理时间
    if (time_file != "")
        time_out << omp_get_wtime() - t3 << " "; // 记录转换缓冲区的计算时间

    // 将缓冲区数据转换为深度图
    Buffer2DepthMap(scan_end_time);
    // 如果指定了时间记录文件，记录转换深度图的计算时间
    if (time_file != "")
        time_out << omp_get_wtime() - t3 << endl; // 记录转换深度图的计算时间

    // 根据是否启用联合聚类处理动态标签，并将结果写入文件
    if (cluster_coupled)
    {
        for (int i = 0; i < size; i++)
        {
            // 根据动态标签决定写入的值，1为动态点，否则为静态点
            int tmp = dyn_tag_cluster[i] == 1 ? 251 : 9;
            if (is_rec)
            {
                out.write((char *)&tmp, sizeof(int)); // 将结果写入输出文件
            }

            // 根据原始动态标签决定写入的值，1或2为动态点，否则为静态点
            tmp = (dyn_tag_origin[i] == 1 || dyn_tag_origin[i] == 2) ? 251 : 9;
            if (is_rec_origin)
            {
                out_origin.write((char *)&tmp, sizeof(int)); // 将结果写入原始动态标签的输出文件
            }
        }
    }
    else // 如果没有启用联合聚类
    {
        for (int i = 0; i < size; i++)
        {
            // 只根据原始动态标签写入结果
            int tmp = (dyn_tag_origin[i] == 1 || dyn_tag_origin[i] == 2) ? 251 : 9;
            if (is_rec)
            {
                out.write((char *)&tmp, sizeof(int));
            }
        }
    }

    // 计算不同测试阶段的总时间
    double total_test1 = 0, total_test2 = 0, total_test3 = 0, total_proj = 0, total_occ = 0, total_map = 0;
    for (int i = 0; i < size; i++)
    {
        total_test1 += time_test1[i];
        total_test2 += time_test2[i];
        total_test3 += time_test3[i];
        total_proj += time_proj[i];
        total_occ += time_occ_check[i];
        total_map += time_map_cons[i];
    }

    // 如果指定了性能分解文件，记录各个阶段的总时间
    if (time_breakdown_file != "")
    {
        time_breakdown_out << total_test1 << " " << total_test2 << " " << total_test3 << " " << total_proj << " " << total_occ << " " << total_map << endl;
    }

    frame_num_for_rec++; // 更新帧数记录
    // 更新点指针索引，准备下一帧数据处理
    cur_point_soph_pointers = (cur_point_soph_pointers + 1) % max_pointers_num;
    // 如果启用了记录，关闭文件
    if (is_rec)
        out.close();
    // 计算总处理时间，并更新平均处理时间统计
    time_total = omp_get_wtime() - t00;
    time_ind++;
    time_total_avr = time_total_avr * (time_ind - 1) / time_ind + time_total / time_ind;
}

/**
 * @brief 根据与载体原点的距离过近，判断一个点是否是无效的
 *
 * @param body 载体坐标
 * @param intensity 强度
 * @return true
 * @return false
 */
bool DynObjFilter::InvalidPointCheck(const V3D &body, const int intensity)
{
    // 计算点相对于机器人的距离平方
    if ((pow(body(0), 2) + pow(body(1), 2) + pow(body(2), 2)) < blind_dis * blind_dis ||
        // nuscenes数据集的条件，如果点过于接近机器人的中心（考虑X和Y轴的一个小区域，以及Z轴的一个小区域）
        (dataset == 1 && fabs(body(0)) < 0.1 && fabs(body(1)) < 1.0) && fabs(body(2)) < 0.1)
    {
        return true; // 如果满足以上任一条件，则认为点无效
    }
    else
    {
        return false; // 否则，点是有效的
    }
}

/**
 * @brief 检查一个点是否属于机器人自身
 *
 * @param body 载体坐标
 * @param dyn 是否
 * @return true
 * @return false
 */
bool DynObjFilter::SelfPointCheck(const V3D &body)
{
    // 针对kitti数据集（dataset == 0），根据点在机器人体坐标系中的位置进行判断
    if (dataset == 0)
    {
        // 检查点是否在机器人特定部分的坐标范围内
        // 这里定义了多个区域，每个区域对应机器人某一部分的空间范围
        // 如果点落在任何一个区域内，则认为该点属于机器人自身，函数返回 true
        if ((body(0) > -1.2 && body(0) < -0.4 && body(1) > -1.7 && body(1) < -1.0 && body(2) > -0.65 && body(2) < -0.4) ||
            (body(0) > -1.75 && body(0) < -0.85 && body(1) > 1.0 && body(1) < 1.6 && body(2) > -0.75 && body(2) < -0.40) ||
            (body(0) > 1.4 && body(0) < 1.7 && body(1) > -1.3 && body(1) < -0.9 && body(2) > -0.8 && body(2) < -0.6) ||
            (body(0) > 2.45 && body(0) < 2.6 && body(1) > -0.6 && body(1) < -0.45 && body(2) > -1.0 && body(2) < -0.9) ||
            (body(0) > 2.45 && body(0) < 2.6 && body(1) > 0.45 && body(1) < 0.6 && body(2) > -1.0 && body(2) < -0.9))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

/**
 * @brief 对点进行球面投影
 *
 * @param p 点结构体
 * @param depth_index 帧(深度图)索引
 * @param rot 旋转
 * @param transl 平移
 * @param p_spherical 输出的点结构体
 */
void DynObjFilter::SphericalProjection(point_soph &p, int depth_index, const M3D &rot, const V3D &transl, point_soph &p_spherical)
{
    // 检查是否之前已经计算过这个点在当前深度图上的投影(并行域内，导致可能计算过)
    // 通过z分量(半径/深度)大于一个小值，确定该向量是否包含有效的投影数据
    if (fabs(p.last_vecs.at(depth_index % HASH_PRIM)[2]) > 10E-5)
    {
        // 如果已计算过，直接使用之前的结果，避免重复计算
        // 这里使用了模运算来访问缓存的投影结果
        p_spherical.vec = p.last_vecs.at(depth_index % HASH_PRIM);
        p_spherical.hor_ind = p.last_positions.at(depth_index % HASH_PRIM)[0];
        p_spherical.ver_ind = p.last_positions.at(depth_index % HASH_PRIM)[1];
        p_spherical.position = p.last_positions.at(depth_index % HASH_PRIM)[2];
    }
    else
    {
        // 如果之前未计算过，执行投影计算
        // 首先，根据旋转和平移参数，将点从全局坐标系转换到深度图坐标系
        V3D p_proj(rot * (p.glob - transl));
        // 调用 GetVec 方法计算球面坐标，该方法考虑了水平和垂直分辨率
        p_spherical.GetVec(p_proj, hor_resolution_max, ver_resolution_max);
        // 标记当前深度图已经计算过，保存vec到last_vecs中
        p.last_vecs.at(depth_index % HASH_PRIM) = p_spherical.vec;
        // 标记当前深度图已经计算过，保存水平、垂直索引和位置标识给last_positions
        p.last_positions.at(depth_index % HASH_PRIM)[0] = p_spherical.hor_ind;
        p.last_positions.at(depth_index % HASH_PRIM)[1] = p_spherical.ver_ind;
        p.last_positions.at(depth_index % HASH_PRIM)[2] = p_spherical.position;
    }
}

float DynObjFilter::DepthInterpolationAll(point_soph &p, int map_index, const DepthMap2D &depth_map)
{

    V3F p_1 = V3F::Zero();
    V3F p_2 = V3F::Zero();
    V3F p_3 = V3F::Zero();
    vector<V3F> p_neighbors;
    int all_num = 0;
    for (int ind_hor = -interp_hor_num; ind_hor <= interp_hor_num; ind_hor++)
    {
        for (int ind_ver = -interp_ver_num; ind_ver <= interp_ver_num; ind_ver++)
        {
            int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF + ((p.ver_ind + ind_ver) % MAX_1D_HALF);
            if (pos_new < 0 || pos_new >= MAX_2D_N)
                continue;
            const vector<point_soph *> &points_in_pixel = depth_map[pos_new];
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph *point = points_in_pixel[j];
                if (fabs(point->time - p.time) < frame_dur)
                {
                    continue;
                }
                float hor_minus = point->vec(0) - p.vec(0);
                float ver_minus = point->vec(1) - p.vec(1);
                if (fabs(hor_minus) < interp_hor_thr && fabs(ver_minus) < interp_ver_thr)
                {
                    all_num++;
                    p_neighbors.push_back(point->vec);
                    if (p_1(2) < 0.000001 || fabs(hor_minus) + fabs(ver_minus) < fabs(p_1(0) - p.vec(0)) + fabs(p_1(1) - p.vec(1)))
                    {
                        p_1 = point->vec;
                    }
                }
            }
        }
    }
    int cur_size = p_neighbors.size();
    if (p_1(2) < 10E-5 || cur_size < 3)
    {
        return -1;
    }
    for (int t_i = 0; t_i < cur_size - 2; t_i++)
    {
        p_1 = p_neighbors[t_i];
        p_2 = V3F::Zero();
        p_3 = V3F::Zero();
        float min_fabs = 2 * (interp_hor_thr + interp_ver_thr);
        float x = p.vec(0) - p_1(0);
        float y = p.vec(1) - p_1(1);
        float alpha = 0, beta = 0;
        for (int i = t_i + 1; i < cur_size - 1; i++)
        {
            if (fabs(p_neighbors[i](0) - p.vec(0)) + fabs(p_neighbors[i](1) - p.vec(1)) < min_fabs)
            {
                p_2 = p_neighbors[i];
                float single_fabs = fabs(p_neighbors[i](0) - p.vec(0)) + fabs(p_neighbors[i](1) - p.vec(1));
                if (single_fabs >= min_fabs)
                    continue;
                for (int ii = i + 1; ii < cur_size; ii++)
                {
                    float cur_fabs = fabs(p_neighbors[i](0) - p.vec(0)) + fabs(p_neighbors[i](1) - p.vec(1)) +
                                     fabs(p_neighbors[ii](0) - p.vec(0)) + fabs(p_neighbors[ii](1) - p.vec(1));
                    if (cur_fabs < min_fabs)
                    {
                        float x1 = p_neighbors[i](0) - p_1(0);
                        float x2 = p_neighbors[ii](0) - p_1(0);
                        float y1 = p_neighbors[i](1) - p_1(1);
                        float y2 = p_neighbors[ii](1) - p_1(1);
                        float lower = x1 * y2 - x2 * y1;
                        if (fabs(lower) > 10E-5)
                        {
                            alpha = (x * y2 - y * x2) / lower;
                            beta = -(x * y1 - y * x1) / lower;
                            if (alpha > 0 && alpha < 1 && beta > 0 && beta < 1 && (alpha + beta) > 0 && (alpha + beta) < 1)
                            {
                                p_3 = p_neighbors[ii];
                                min_fabs = cur_fabs;
                            }
                        }
                    }
                }
            }
        }
        if (p_2(2) < 10E-5 || p_3(2) < 10E-5)
        {
            continue;
        }
        float depth_cal = (1 - alpha - beta) * p_1(2) + alpha * p_2(2) + beta * p_3(2);
        return depth_cal;
    }
    return -2;
} // -1 denotes no points, -2 denotes no triangular > 1000 denotes gauss interpolation

/**
 * @brief 论文中情况1的动态点。
 *
 * 该函数遍历所有最近深度图，使用球面投影检查点在每个深度图中的位置，然后应用
 * Case1Enter 和 Case1MapConsistencyCheck 函数来评估点的动态性。
 * 如果足够多的深度图表明点被遮挡，它可能被认为是动态的。
 *
 * @param p 点信息结构体，包含点的全局坐标和其他属性。
 * @return true 如果该点在足够多的深度图中被认为可能是动态的。
 * @return false 如果该点在大多数深度图中不被认为是动态的。
 */
bool DynObjFilter::Case1(point_soph &p)
{
    int depth_map_num = depth_map_list.size(); // 当前深度图数量
    int occluded_map = depth_map_num;          // 被遮挡深度图数量

    // 从最新的深度图开始向前遍历所有深度图
    for (int i = depth_map_num - 1; i >= 0; i--)
    {
        // 对点进行球面投影
        SphericalProjection(p, depth_map_list[i]->map_index, depth_map_list[i]->project_R, depth_map_list[i]->project_T, p);

        // 如果点的位置超出了预设的边界或者深度为负，则认为点无效，继续检查下一个深度图
        if (fabs(p.hor_ind) > MAX_1D || fabs(p.ver_ind) > MAX_1D_HALF || p.vec(2) < 0.0f || p.position < 0 || p.position >= MAX_2D_N)
        {
            p.dyn = INVALID;
            continue;
        }

        // 检查点是否满足情况1的动态点
        if (Case1Enter(p, *depth_map_list[i]))
        {
            // 进行地图一致性检测，防止误杀
            if (Case1FalseRejection(p, *depth_map_list[i]))
            {
                occluded_map -= 1; // 减少被遮挡的深度图计数
            }
        }
        else
        {
            // 点不满足情况1的动态点，减少被遮挡的深度图计数
            occluded_map -= 1;
        }
        // 如果被遮挡的深度图数量少于阈值，则认为点不是动态的
        if (occluded_map < occluded_map_thr1)
        {
            return false;
        }
        // // 如果被遮挡的深度图数量已经满足阈值，提前返回true
        if (occluded_map - (i) >= occluded_map_thr1)
        {
            return true;
        }
    }
    // 遍历结束后，如果被遮挡的深度图数量满足阈值，认为点可能是动态的
    if (occluded_map >= occluded_map_thr1)
    {
        return true;
    }
    return false;
}

/**
 * @brief 判断点是否满足作为动态点的第一种情况的入口条件。
 *
 * 这个判断基于点相对于周围静态环境的深度差异。具体来说，如果一个点的深度与它在
 * 深度图中相应位置的静态深度差异超过了预设的阈值，那么这个点可能是动态的。
 *
 * @param p 点的信息，包含它的位置和深度等数据。
 * @param map_info 包含深度图信息的结构体，提供了深度图中每个像素点的深度数据。
 * @return true 如果该点可能是动态的，否则返回false。
 */
bool DynObjFilter::Case1Enter(const point_soph &p, const DepthMap &map_info)
{
    float max_depth = 0, min_depth = 0; // 初始化最大和最小深度变量

    // 检查点所在像素位置是否有对应的深度数据
    if (map_info.depth_map[p.position].size() > 0)
    {
        // 如果有，直接使用该位置的最大和最小静态深度
        max_depth = map_info.max_depth_static[p.position];
        min_depth = map_info.min_depth_static[p.position];
    }
    else
    {
        // 如果没有，检查该点是否在预设的视场范围内，并且视场验证通过
        if (p.ver_ind <= pixel_fov_up && p.ver_ind > pixel_fov_down &&
            p.hor_ind <= pixel_fov_left && p.hor_ind >= pixel_fov_right &&
            CheckVerFoV(p, map_info))
        {
            // 如果是，检查该点周围的邻居点，更新最大和最小深度
            CheckNeighbor(p, map_info, max_depth, min_depth);
        }
    }

    // 计算深度差异的阈值
    float cur_min = max(cutoff_value, k_depth_min_thr1 * (p.vec(2) - d_depth_min_thr1)) + enter_min_thr1;
    float cur_max = max(cutoff_value, k_depth_max_thr1 * (p.vec(2) - d_depth_max_thr1)) + enter_max_thr1;
    // float cur_depth = depth_thr1;

    // 如果KITTI的点失真,根据失真程度调整深度差异阈值
    if (dataset == 0 && p.is_distort)
    {
        cur_min = enlarge_distort * cur_min;
        cur_max = enlarge_distort * cur_max;
        // cur_depth = enlarge_distort * cur_depth;
    }

    // 如果点的深度与静态环境的深度差异超过阈值，认为可能是动态点
    //                                    min_depth                             max_depth
    //                                       ^                                      ^
    //                                       |                                      |
    // 0--------------------------|----------|---------|-------------------|--------|------->
    // <------------------------->     |          |    <------------------->   |
    //         valid range             v          v           valid range      v
    //                              cur_max     cur_min                      cur_max
    float max_depth_all = 0, min_depth_all = 0;
    if (p.vec(2) < min_depth - cur_max ||
        (min_depth < p.vec(2) - cur_min && max_depth > p.vec(2) + cur_max) ||
        (stop_object_detect && min_depth < 10E-5 && max_depth < 10E-5 && map_info.depth_map[p.position].size() > 0 && p.vec(2) < map_info.max_depth_all[p.position] + 1.0))
    {
        case1_num++; // 动态点计数增加
        return true; // 该点可能是动态的
    }
    return false; // 该点不满足动态点的条件
}

/**
 * @brief 在垂直视场（Vertical FoV）的有效范围内，检查一个点是否位于视场边缘或被遮挡的静态点。
 *
 * 这个函数通过检查一个点的垂直位置（上方和下方）在深度图中是否有有效数据来实现。
 * 如果该点的上方或下方缺少深度数据，可能表示该点处于视场的边界或在视场内部但被遮挡，
 * 这有助于提高动态点检测的准确性，避免错误地将视场边缘或被遮挡的静态物体识别为动态对象。
 * 只有当一个点位于传感器垂直视场的有效范围内，并且该范围内有充足的深度数据时，它才被进一步考虑作为动态点的候选。
 *
 * @param p 点的信息，包括其在深度图中的位置（垂直索引和水平索引）。
 * @param map_info 深度图信息，包含深度图中每个位置的深度数据。
 * @return true 如果该点在垂直视场范围内缺少有效深度数据，表示它可能位于视场边缘或被遮挡，为静态点。
 * @return false 如果该点的上方和下方都有有效深度数据，表示它位于有效的视场范围内，可能为动态点。
 */
bool DynObjFilter::CheckVerFoV(const point_soph &p, const DepthMap &map_info)
{
    // 初始化表示点上方和下方是否有有效深度数据的标志
    bool ver_up = false, ver_down = false;

    // 从点的垂直索引向下检查，直到达到视场的下界
    for (int i = p.ver_ind; i >= pixel_fov_down; i--)
    {
        int cur_pos = p.hor_ind * MAX_1D_HALF + i;  // 计算当前深度图位置索引
        if (map_info.depth_map[cur_pos].size() > 0) // 如果该位置有深度数据
        {
            ver_down = true; // 标记为在下方有有效数据
            break;           // 不需要进一步检查更低的位置
        }
    }
    // 从点的垂直索引向上检查，直到达到视场的上界
    for (int i = p.ver_ind; i <= pixel_fov_up; i++)
    {
        int cur_pos = p.hor_ind * MAX_1D_HALF + i;  // 计算当前深度图位置索引
        if (map_info.depth_map[cur_pos].size() > 0) // 如果该位置有深度数据
        {
            ver_up = true; // 标记为在上方有有效数据
            break;         // 不需要进一步检查更高的位置
        }
    }
    // 如果点的上方和下方都有效深度数据，则它可能是动态点，返回false
    // 如果任意一方缺少有效深度数据，则认为它可能位于视场边缘或被遮挡，为静态点，返回true
    return !(ver_up && ver_down);
}

/**
 * @brief 检查给定点周围邻居点的深度范围
 *
 * 该函数通过遍历给定点周围的邻居点，来更新深度图对应像素的最大深度和最小深度值。
 *
 * @param p 点结构体，包含该点的水平和垂直索引。
 * @param map_info 深度图信息，包含每个像素点的深度数据。
 * @param max_depth 引用传递，用于更新周围邻居点的最大深度。
 * @param min_depth 引用传递，用于更新周围邻居点的最小深度。
 */
void DynObjFilter::CheckNeighbor(const point_soph &p, const DepthMap &map_info, float &max_depth, float &min_depth)
{
    int n = checkneighbor_range; // 邻居检查范围，决定了要检查多远的邻居点

    // 遍历给定点周围的邻居点
    for (int i = -n; i <= n; i++)
    {
        for (int j = -n; j <= n; j++)
        {
            // 计算当前邻居点在深度图中的索引位置
            int cur_pos = (p.hor_ind + i) * MAX_1D_HALF + p.ver_ind + j;
            // 判断当前索引在有效范围内，并且该位置有深度数据
            if (cur_pos < MAX_2D_N && cur_pos >= 0 && map_info.depth_map[cur_pos].size() > 0)
            {
                // 获取当前邻居点的最大和最小深度值
                float cur_max_depth = map_info.max_depth_static[cur_pos];
                float cur_min_depth = map_info.min_depth_static[cur_pos];

                // 更新给定点周围的最大和最小深度值
                // 如果已有的最小深度值有效，取当前最小深度和已有最小深度的较小值
                if (min_depth > 10E-5)
                    min_depth = std::min(cur_min_depth, min_depth);
                else
                    min_depth = cur_min_depth;

                // 如果已有的最大深度值有效，取当前最大深度和已有最大深度的较大值
                if (max_depth > 10E-5)
                    max_depth = std::max(cur_max_depth, max_depth);
                else
                    max_depth = cur_max_depth;
            }
        }
    }
}

/**
 * @brief 情况1动态点的误杀（假阳性剔除）的静态点
 *
 * @param p 点信息结构体。
 * @param map_info 深度图信息。
 * @return true 如果深度图一致性检查通过，即该点不应被误判为动态点。
 * @return false 否则。
 */
bool DynObjFilter::Case1FalseRejection(point_soph &p, const DepthMap &map_info)
{
    return Case1MapConsistencyCheck(p, map_info, case1_interp_en);
}

/**
 * @brief 检查点的深度信息在不同深度图中是否一致，用以评估该点是否动态。
 *
 * 函数通过计算点在其所在像素及周围像素中的深度一致性来执行检查。如果点的深度与
 * 周围静态环境深度的差异超过了一定阈值，那么它可能被认为是动态的。此外，还可以
 * 进行深度插值来增加检查的准确性。
 *
 * @param p 点信息，包括深度等。
 * @param map_info 包含深度图信息的结构体。
 * @param interp 是否进行深度插值以进一步验证点的一致性。
 * @return true 如果点的深度与静态环境在多数深度图中保持一致，认为它不是动态的。
 * @return false 如果点的深度在足够多的深度图中与静态环境存在显著差异，可能是动态的。
 */
bool DynObjFilter::Case1MapConsistencyCheck(point_soph &p, const DepthMap &map_info, bool interp)
{
    // 计算水平和垂直方向的检查半径
    float hor_half = max(map_cons_hor_dis1 / (max(p.vec(2), blind_dis)), map_cons_hor_thr1);
    float ver_half = max(map_cons_ver_dis1 / (max(p.vec(2), blind_dis)), map_cons_ver_thr1);

    // 计算用于一致性检查的深度差异阈值
    float cur_map_cons_depth_thr1 = max(cutoff_value, k_depth_max_thr1 * (p.vec(2) - d_depth_max_thr1)) + map_cons_depth_thr1;
    float cur_map_cons_min_thr1 = max(cutoff_value, k_depth_min_thr1 * (p.vec(2) - d_depth_min_thr1)) + enter_min_thr1;
    float cur_map_cons_max_thr1 = max(cutoff_value, k_depth_max_thr1 * (p.vec(2) - d_depth_max_thr1)) + enter_max_thr1;

    // 如果KITTI数据的点失真,则调整阈值
    if (dataset == 0 && p.is_distort)
    {
        cur_map_cons_depth_thr1 = enlarge_distort * cur_map_cons_depth_thr1;
        cur_map_cons_min_thr1 = enlarge_distort * cur_map_cons_min_thr1;
        cur_map_cons_max_thr1 = enlarge_distort * cur_map_cons_max_thr1;
    }

    // 如果点位于特定的Z轴区域内，调整检查半径和深度差异阈值
    if (fabs(p.vec(1)) < enlarge_z_thr1 / 57.3)
    {
        hor_half = enlarge_angle * hor_half;
        ver_half = enlarge_angle * ver_half;
        cur_map_cons_depth_thr1 = enlarge_depth * cur_map_cons_depth_thr1;
    }
    // 计算水平和垂直方向上的检查像素数
    int cur_map_cons_hor_num1 = ceil(hor_half / hor_resolution_max);
    int cur_map_cons_ver_num1 = ceil(ver_half / ver_resolution_max);
    int num = 0;
    point_soph closest_point;
    float closest_dis = 100;
    // 遍历点周围的像素，检查深度一致性
    for (int ind_hor = -cur_map_cons_hor_num1; ind_hor <= cur_map_cons_hor_num1; ind_hor++)
    {
        for (int ind_ver = -cur_map_cons_ver_num1; ind_ver <= cur_map_cons_ver_num1; ind_ver++)
        {
            // 计算新的像素位置
            int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF + ((p.ver_ind + ind_ver) % MAX_1D_HALF);
            if (pos_new < 0 || pos_new >= MAX_2D_N)
                continue;

            // 获取该像素位置的点集
            const vector<point_soph *> &points_in_pixel = map_info.depth_map[pos_new];
            // 如果该像素的静态深度在点深度差异阈值范围内，视为一致
            if (map_info.max_depth_static[pos_new] < p.vec(2) - cur_map_cons_min_thr1 ||
                map_info.min_depth_static[pos_new] > p.vec(2) + cur_map_cons_max_thr1)
            {
                continue;
            }
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph *point = points_in_pixel[j];
                if (point->dyn == STATIC &&
                    (fabs(p.vec(2) - point->vec(2)) < cur_map_cons_depth_thr1 ||
                     ((p.vec(2) - point->vec(2)) > cur_map_cons_depth_thr1 && (p.vec(2) - point->vec(2)) < cur_map_cons_min_thr1)) &&
                    fabs(p.vec(0) - point->vec(0)) < hor_half &&
                    fabs(p.vec(1) - point->vec(1)) < ver_half)
                {
                    return true; // 如果找到与点深度一致的静态点，认为该点不是动态的
                }
            }
        }
    }
    // 可选的深度插值验证
    if (interp && (p.local(0) < self_x_b || p.local(0) > self_x_f || p.local(1) > self_y_l || p.local(1) < self_y_r))
    {
        float depth_static = DepthInterpolationStatic(p, map_info.map_index, map_info.depth_map);
        float cur_interp = interp_thr1;
        if (p.vec(2) > interp_start_depth1)
            cur_interp += ((p.vec(2) - interp_start_depth1) * interp_kp1 + interp_kd1);
        if (dataset == 0)
        {
            if (fabs(depth_static + 1) < 10E-5 || fabs(depth_static + 2) < 10E-5)
            {
                return false;
            }
            else
            {
                if (fabs(depth_static - p.vec(2)) < cur_interp)
                {
                    return true;
                }
            }
        }
        else
        {
            if (fabs(depth_static + 1) < 10E-5 || fabs(depth_static + 2) < 10E-5)
            {
                return false;
            }
            else
            {
                if (fabs(depth_static - p.vec(2)) < cur_interp)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

float DynObjFilter::DepthInterpolationStatic(point_soph &p, int map_index, const DepthMap2D &depth_map)
{
    if (fabs(p.last_depth_interps.at(map_index - depth_map_list.front()->map_index)) > 10E-4)
    {
        float depth_cal = p.last_depth_interps.at(map_index - depth_map_list.front()->map_index);
        return depth_cal;
    }
    V3F p_1 = V3F::Zero();
    V3F p_2 = V3F::Zero();
    V3F p_3 = V3F::Zero();
    vector<V3F> p_neighbors;
    int all_num = 0, static_num = 0, no_bg_num = 0;
    for (int ind_hor = -interp_hor_num; ind_hor <= interp_hor_num; ind_hor++)
    {
        for (int ind_ver = -interp_ver_num; ind_ver <= interp_ver_num; ind_ver++)
        {
            int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF + ((p.ver_ind + ind_ver) % MAX_1D_HALF);
            if (pos_new < 0 || pos_new >= MAX_2D_N)
                continue;
            const vector<point_soph *> &points_in_pixel = depth_map[pos_new];
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph *point = points_in_pixel[j];
                if (fabs(point->time - p.time) < frame_dur)
                {
                    continue;
                }
                float hor_minus = point->vec(0) - p.vec(0);
                float ver_minus = point->vec(1) - p.vec(1);
                if (fabs(hor_minus) < interp_hor_thr && fabs(ver_minus) < interp_ver_thr)
                {
                    all_num++;
                    if (point->dyn == STATIC)
                    {
                        static_num++;
                    }
                    if ((point->vec(2) - p.vec(2)) <= interp_static_max && (p.vec(2) - point->vec(2)) < 5.0)
                    {
                        no_bg_num++;
                    }
                    if (point->dyn == STATIC)
                    {
                        p_neighbors.push_back(point->vec);
                        if (p_1(2) < 0.000001 || fabs(hor_minus) + fabs(ver_minus) < fabs(p_1(0) - p.vec(0)) + fabs(p_1(1) - p.vec(1)))
                        {
                            p_1 = point->vec;
                        }
                    }
                }
            }
        }
    }
    if (p_1(2) < 10E-5)
    {
        p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = -1;
        return -1;
    }
    int cur_size = p_neighbors.size();
    for (int t_i = 0; t_i < cur_size - 2; t_i++)
    {
        p_1 = p_neighbors[t_i];
        p_2 = V3F::Zero();
        p_3 = V3F::Zero();
        float min_fabs = 2 * (interp_hor_thr + interp_ver_thr);
        float x = p.vec(0) - p_1(0);
        float y = p.vec(1) - p_1(1);
        float alpha = 0, beta = 0;
        for (int i = t_i + 1; i < cur_size - 1; i++)
        {
            if (fabs(p_neighbors[i](0) - p.vec(0)) + fabs(p_neighbors[i](1) - p.vec(1)) < min_fabs)
            {
                p_2 = p_neighbors[i];
                float single_fabs = fabs(p_neighbors[i](0) - p.vec(0)) + fabs(p_neighbors[i](1) - p.vec(1));
                if (single_fabs >= min_fabs)
                    continue;
                for (int ii = i + 1; ii < cur_size; ii++)
                {
                    float cur_fabs = fabs(p_neighbors[i](0) - p.vec(0)) + fabs(p_neighbors[i](1) - p.vec(1)) +
                                     fabs(p_neighbors[ii](0) - p.vec(0)) + fabs(p_neighbors[ii](1) - p.vec(1));
                    if (cur_fabs < min_fabs)
                    {
                        float x1 = p_neighbors[i](0) - p_1(0);
                        float x2 = p_neighbors[ii](0) - p_1(0);
                        float y1 = p_neighbors[i](1) - p_1(1);
                        float y2 = p_neighbors[ii](1) - p_1(1);
                        float lower = x1 * y2 - x2 * y1;
                        if (fabs(lower) > 10E-5)
                        {
                            alpha = (x * y2 - y * x2) / lower;
                            beta = -(x * y1 - y * x1) / lower;
                            if (alpha > 0 && alpha < 1 && beta > 0 && beta < 1 && (alpha + beta) > 0 && (alpha + beta) < 1)
                            {
                                p_3 = p_neighbors[ii];
                                min_fabs = cur_fabs;
                            }
                        }
                    }
                }
            }
        }
        if (p_2(2) < 10E-5 || p_3(2) < 10E-5)
        {
            continue;
        }
        float depth_cal = (1 - alpha - beta) * p_1(2) + alpha * p_2(2) + beta * p_3(2);
        p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = depth_cal;
        return depth_cal;
    }
    if (static_num > 0 && cur_size < all_num / 2)
    {
        p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = -2;
        return -2;
    }
    else
    {
        p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = -2;
        return -2;
    }
} // return -1 denotes no point, -2 denotes no trianguolar but with points

/**
 * @brief 论文中情况2的动态点。
 *
 * 此函数首先检查输入点是否因为失真（比如由于传感器特性）而应当被忽略，然后对点进行球面投影，
 * 并在最新的深度图中检查其深度一致性和是否被遮挡。如果一个点在连续的深度图中被相同的物体遮挡，
 * 并且其深度变化满足一定条件，则认为该点是动态的。
 *
 * @param p 点信息结构体，包括位置、深度等信息。
 * @return true 如果该点在足够数量的深度图中表现出动态特性。
 * @return false 如果该点不满足动态点的条件。
 */
bool DynObjFilter::Case2(point_soph &p)
{
    // 忽略KITTI的失真点
    if (dataset == 0 && p.is_distort)
        return false;

    // 获取最新深度图索引
    int first_i = depth_map_list.size() - 1;
    if (first_i < 0) // 如果没有深度图，返回 false
        return false;

    // 对点进行球面投影
    point_soph p_spherical = p;
    SphericalProjection(p, depth_map_list[first_i]->map_index, depth_map_list[first_i]->project_R, depth_map_list[first_i]->project_T, p_spherical);

    // 如果点的位置超出了预设的边界或者深度为负，则认为点无效，继续检查下一个深度图
    if (fabs(p_spherical.hor_ind) >= MAX_1D || fabs(p_spherical.ver_ind) >= MAX_1D_HALF || p_spherical.vec(2) < 0.0f ||
        p_spherical.position < 0 || p_spherical.position >= MAX_2D_N)
    {
        p.dyn = INVALID;
        return false;
    }

    // 当前点被遮挡次数
    int cur_occ_times = 0;

    // 如果点通过初步进入检查（即，它在深度上大于所有深度图的点）
    if (Case2Enter(p_spherical, *depth_map_list[first_i]))
    {
        // 如果不满足地图一致性
        if (!Case2MapConsistencyCheck(p_spherical, *depth_map_list[first_i], case2_interp_en))
        {
            // 初始化变量，用于记录遮挡检测和速度检测
            double ti = 0; // 时间插值
            float vi = 0;  // 速度插值
            bool map_cons = true;
            // 遍历周围像素，检查当前点是否被遮挡且深度一致
            for (int ind_hor = -occ_hor_num2; ind_hor <= occ_hor_num2; ind_hor++)
            {
                for (int ind_ver = -occ_ver_num2; ind_ver <= occ_ver_num2; ind_ver++)
                {
                    // 计算新位置索引
                    int pos_new = ((p_spherical.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF + ((p_spherical.ver_ind + ind_ver) % MAX_1D_HALF);
                    if (pos_new < 0 || pos_new >= MAX_2D_N) // 跳过无效索引
                        continue;
                    // 获取像素位置的点集合
                    const vector<point_soph *> &points_in_pixel = depth_map_list[first_i]->depth_map[pos_new];
                    // 如果最小深度大于当前点的深度，则没有遮挡当前点，则继续检查下一个点
                    if (depth_map_list[first_i]->min_depth_all[pos_new] > p_spherical.vec(2))
                    {
                        continue;
                    }

                    // 检查每个点是否遮挡当前点
                    for (int k = 0; k < points_in_pixel.size() && map_cons; k++)
                    {
                        const point_soph *p_occ = points_in_pixel[k];
                        // 如果找到遮挡点，并且满足深度一致性
                        if (Case2IsOccluded(p_spherical, *p_occ) && Case2DepthConsistencyCheck(*p_occ, *depth_map_list[first_i]))
                        {
                            cur_occ_times = 1; // 记录遮挡次数

                            // 如果遮挡次数达到阈值，判定为动态
                            if (cur_occ_times >= occluded_times_thr2)
                                break;

                            // 计算被遮挡点和当前点的平均时间和速度
                            ti = (p_occ->time + p.time) / 2;
                            vi = (p_spherical.vec(2) - p_occ->vec(2)) / (p.time - p_occ->time);

                            // 记录遮挡信息
                            p.occu_index[0] = depth_map_list[first_i]->map_index;
                            p.occu_index[1] = pos_new;
                            p.occu_index[2] = k;
                            p.occ_vec = p_spherical.vec;
                            p.occu_times = cur_occ_times;

                            // 在已有深度图列表中逆向遍历，寻找p1是否在更早的帧中也被遮挡
                            int i = depth_map_list.size() - 2;
                            point_soph p1 = *points_in_pixel[k];
                            while (i >= 0)
                            {
                                // 如果p1在当前帧没有被遮挡或者遮挡信息指向的帧在深度图列表范围之外
                                if (p1.occu_index[0] == -1 || p1.occu_index[0] < depth_map_list.front()->map_index)
                                {
                                    // 对p1进行球面投影到当前深度图i
                                    SphericalProjection(p1, depth_map_list[i]->map_index, depth_map_list[i]->project_R, depth_map_list[i]->project_T, p1);
                                    // 检查在深度图i中是否存在遮挡p1的点
                                    if (Case2SearchPointOccludingP(p1, *depth_map_list[i]))
                                    {
                                        // 如果存在，更新p1的遮挡信息
                                        p1.occ_vec = p1.vec;
                                    }
                                    else
                                    {
                                        // 如果不存在遮挡点，跳出循环
                                        break;
                                    }
                                }
                                // 更新遍历的深度图索引为p1遮挡信息指向的深度图索引
                                i = p1.occu_index[0] - depth_map_list.front()->map_index;
                                // 获取遮挡p1的点p2
                                point_soph *p2 = depth_map_list[i]->depth_map[p1.occu_index[1]][p1.occu_index[2]];
                                // 对当前点p进行球面投影到深度图i
                                SphericalProjection(p, depth_map_list[i]->map_index, depth_map_list[i]->project_R, depth_map_list[i]->project_T, p);
                                // 检查当前点p在深度图i中的地图一致性
                                if (Case2MapConsistencyCheck(p, *depth_map_list[i], case2_interp_en))
                                {
                                    // 如果一致，则p点为静态点，更新地图一致性标志为false并跳出循环
                                    map_cons = false;
                                    break;
                                }
                                // 计算遮挡点p2与p1之间的速度、时间的平均值
                                float vc = (p1.occ_vec(2) - p2->vec(2)) / (p1.time - p2->time);
                                double tc = (p2->time + p1.time) / 2;
                                // 检查当前点p是否被p2遮挡，并且深度一致性及速度检查通过
                                if (Case2IsOccluded(p, *p2) &&
                                    Case2DepthConsistencyCheck(*p2, *depth_map_list[i]) && Case2VelCheck(vi, vc, ti - tc))
                                {
                                    // 增加遮挡次数
                                    cur_occ_times += 1;
                                    // 如果遮挡次数达到阈值，标记为动态点并返回true
                                    if (cur_occ_times >= occluded_times_thr2)
                                    {
                                        p.occu_times = cur_occ_times;
                                        return true;
                                    }
                                    // 更新遮挡点信息以便下一轮检查
                                    p1 = *p2;
                                    vi = vc;
                                    ti = tc;
                                }
                                else
                                {
                                    // 如果不满足遮挡条件，跳出循环
                                    break;
                                }
                                // 移至更早的深度图
                                i--;
                            }
                        }
                        if (cur_occ_times >= occluded_times_thr2)
                            break;
                    }
                    if (cur_occ_times >= occluded_times_thr2)
                        break;
                }
                if (cur_occ_times >= occluded_times_thr2)
                    break;
            }
        }
    }

    // 如果最终被遮挡次数超过阈值，则认为点是动态的
    if (cur_occ_times >= occluded_times_thr2)
    {
        p.occu_times = cur_occ_times;
        return true;
    }
    return false;
}

/**
 * @brief 判断一个点是否满足进入第二种动态点检测情况的条件。
 *
 * 此函数基于点的深度与其所在位置最远点的深度差，以及点的静态性质，来判断该点是否可能被视为动态。
 * 如果一个点的深度明显大于该位置的最远深度阈值，那么它可能被前面的物体遮挡，因此认为它进入了动态点的判断流程。
 *
 * @param p 点信息结构体，包括位置、深度等信息。
 * @param map_info 当前深度图的信息，包含所有点的深度信息等。
 * @return true 如果该点满足动态点的初步判断条件。
 * @return false 如果该点不满足进入动态点判断流程的条件。
 */
bool DynObjFilter::Case2Enter(point_soph &p, const DepthMap &map_info)
{
    // 如果点已被标记为非静态，则直接排除
    if (p.dyn != STATIC)
    {
        return false;
    }

    // 计算当前点与其所在位置最大深度点的深度差异阈值
    float depth_thr2_final = max(cutoff_value, k_depth_max_thr2 * (p.vec(2) - d_depth_max_thr2)) + occ_depth_thr2;

    // 如果当前位置有点，则获取该位置最大深度点，并重新计算深度差异阈值
    float max_depth = 0;
    if (map_info.depth_map[p.position].size() > 0)
    {
        const point_soph *max_point = map_info.depth_map[p.position][map_info.max_depth_index_all[p.position]];
        max_depth = max_point->vec(2);                                  // 更新最大深度值
        float delta_t = (p.time - max_point->time);                     // 计算时间差
        depth_thr2_final = min(depth_thr2_final, v_min_thr2 * delta_t); // 根据时间差调整深度差异阈值
    }

    // 判断当前点的深度是否大于最大深度加上深度差异阈值，如果是，则认为该点可能被遮挡，满足动态点的初步条件
    if (p.vec(2) > max_depth + depth_thr2_final)
    {
        case2_num++; // 动态点计数器增加
        return true; // 满足条件，进入动态点判断流程
    }
    else
    {
        return false;
    }
}

/**
 * @brief 检查一个点在深度图中的一致性。
 *
 * 这个函数检查一个点与周围点在深度上的一致性。如果该点与周围静态点的深度差异在一定范围内，
 * 则认为该点与深度图保持一致，可能不是动态的。
 *
 * @param p 待检测的点。
 * @param map_info 当前处理的深度图信息。
 * @param interp 是否使用插值来判断深度一致性。
 * @return true 如果点与深度图保持一致，否则返回 false。
 */
bool DynObjFilter::Case2MapConsistencyCheck(point_soph &p, const DepthMap &map_info, bool interp)
{
    // 设置水平和垂直方向的深度一致性检查阈值
    float cur_hor = map_cons_hor_thr2;
    float cur_ver = map_cons_ver_thr2;
    // 计算当前点深度的一致性检查阈值
    float cur_depth = max(cutoff_value, k_depth_max_thr2 * (p.vec(2) - d_depth_max_thr2)) + map_cons_depth_thr2;

    // 遍历该点周围的像素，检查深度一致性
    for (int ind_hor = -map_cons_hor_num2; ind_hor <= map_cons_hor_num2; ind_hor++)
    {
        for (int ind_ver = -map_cons_ver_num2; ind_ver <= map_cons_ver_num2; ind_ver++)
        {
            // 计算新位置的索引
            int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF + ((p.ver_ind + ind_ver) % MAX_1D_HALF);
            if (pos_new < 0 || pos_new >= MAX_2D_N)
                continue; // 跳过无效索引

            // 获取新位置上的点
            const vector<point_soph *> &points_in_pixel = map_info.depth_map[pos_new];

            // 如果当前位置的最大和最小深度与点深度的差异超出阈值，则跳过
            if (map_info.max_depth_all[pos_new] > p.vec(2) + cur_depth &&
                map_info.min_depth_all[pos_new] < p.vec(2) - cur_depth)
            {
                continue;
            }

            // 检查该位置上的每个点
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph *point = points_in_pixel[j];
                // 如果点是静态的，并且与待检测点在时间和深度上都足够接近，则认为是一致的
                if (point->dyn == STATIC &&
                    fabs(p.time - point->time) > frame_dur &&
                    fabs(p.vec(2) - point->vec(2)) < cur_depth &&
                    fabs(p.vec(0) - point->vec(0)) < map_cons_hor_thr2 &&
                    fabs(p.vec(1) - point->vec(1)) < map_cons_ver_thr2)
                {
                    return true;
                }
            }
        }
    }

    // 如果启用了插值，并且点位于车辆自身范围外，则进行插值判断
    if (interp && (p.local(0) < self_x_b || p.local(0) > self_x_f || p.local(1) > self_y_l || p.local(1) < self_y_r))
    {
        // 计算插值深度一致性阈值
        float cur_interp = interp_thr2 * (depth_map_list.back()->map_index - map_info.map_index + 1);
        // 进行深度插值
        float depth_all = DepthInterpolationAll(p, map_info.map_index, map_info.depth_map);
        // 如果深度插值结果与点深度足够接近，则认为是一致的
        if (fabs(p.vec(2) - depth_all) < cur_interp)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    return false; // 如果所有检查都未通过，则认为点与深度图不一致，则仍然可能为动态点
}

/**
 * @brief 检查深度图是否有点遮挡了点p。
 *
 * @param p 要检查的点，其信息可能会被更新以反映遮挡情况。
 * @param map_info 当前考虑的深度图信息，包含深度图中所有点的信息。
 * @return true 如果找到遮挡点p的点；false 如果没有找到。
 */
bool DynObjFilter::Case2SearchPointOccludingP(point_soph &p, const DepthMap &map_info)
{
    // 遍历p周围的点，范围由occ_hor_num2和occ_ver_num2定义
    for (int ind_hor = -occ_hor_num2; ind_hor <= occ_hor_num2; ind_hor++)
    {
        for (int ind_ver = -occ_ver_num2; ind_ver <= occ_ver_num2; ind_ver++)
        {
            // 计算新的位置索引
            int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF + ((p.ver_ind + ind_ver) % MAX_1D_HALF);
            // 检查新位置是否有效
            if (pos_new < 0 || pos_new >= MAX_2D_N)
                continue;
            // 获取当前位置的所有点
            const vector<point_soph *> &points_in_pixel = map_info.depth_map[pos_new];
            // 如果当前位置的最小深度大于p的深度，跳过
            if (map_info.min_depth_all[pos_new] > p.vec(2))
            {
                continue;
            }

            // 遍历当前位置的所有点
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph *p_cond = points_in_pixel[j];
                // 检查点p_cond是否遮挡了点p，并且在深度上一致
                if (Case2IsOccluded(p, *p_cond) && Case2DepthConsistencyCheck(*p_cond, map_info))
                {
                    // 更新p的遮挡信息
                    p.occu_index[0] = map_info.map_index;
                    p.occu_index[1] = pos_new;
                    p.occu_index[2] = j;
                    p.occ_vec = p.vec;
                    return true;
                }
            }
        }
    }
    return false;
}

/**
 * @brief 判断点p是否被点p_occ遮挡。
 *
 * 该函数用于在处理第二种动态点检测情况时，判断一个点是否被另一个点遮挡。
 *
 * @param p 当前考虑的点。
 * @param p_occ 可能遮挡p点的点。
 * @return true 如果p被p_occ遮挡，否则返回false。
 */
bool DynObjFilter::Case2IsOccluded(const point_soph &p, const point_soph &p_occ)
{
    // 排除失真点或无效点
    if ((dataset == 0 && p_occ.is_distort) || (dataset == 0 && p.is_distort) || p_occ.dyn == INVALID)
        return false;

    // 排除车辆自身区域内的点，自身区域内的点不考虑遮挡
    if ((p.local(0) > self_x_b && p.local(0) < self_x_f && p.local(1) < self_y_l && p.local(1) > self_y_r) ||
        (p_occ.local(0) > self_x_b && p_occ.local(0) < self_x_f && p_occ.local(1) < self_y_l && p_occ.local(1) > self_y_r))
    {
        return false;
    }

    // 计算时间差
    float delta_t = p.time - p_occ.time;
    // 初始化水平和垂直遮挡阈值
    float cur_occ_hor = occ_hor_thr2;
    float cur_occ_ver = occ_ver_thr2;

    // 当时间差大于0时（即p点发生在p_occ之后），计算深度差异阈值
    if (delta_t > 0)
    {
        // p是否被点p_occ遮挡时应该使用的深度差异阈值
        float depth_thr2_final = min(max(cutoff_value, k_depth_max_thr2 * (p.vec(2) - d_depth_max_thr2)) + occ_depth_thr2, v_min_thr2 * delta_t);
        // 判断p点是否在p_occ点的遮挡范围内
        if (p.vec(2) > p_occ.vec(2) + depth_thr2_final &&
            fabs(p.vec(0) - p_occ.vec(0)) < cur_occ_hor &&
            fabs(p.vec(1) - p_occ.vec(1)) < cur_occ_ver)
        {
            return true; // 如果满足条件，则p被p_occ遮挡
        }
    }
    return false; // 如果不满足条件，则p不被p_occ遮挡
}

/**
 * 检查点p在深度上与周围点的一致性，用于判断点p是否可能被遮挡或是动态对象的一部分。
 *
 * @param p 需要检查的点。
 * @param map_info 深度图信息，包含了深度图中所有点的信息。
 * @return 如果点p在深度上与周围点一致，返回true；否则返回false。
 */
bool DynObjFilter::Case2DepthConsistencyCheck(const point_soph &p, const DepthMap &map_info)
{
    // 初始化累加器和计数器
    float all_minus = 0; // 累加周围点与p的深度差的绝对值
    int num = 0;         // 计数与p深度差异小于阈值的静态点数量
    int smaller_num = 0; // 计数深度小于p的点数量
    int greater_num = 0; // 计数深度大于p的点数量
    int all_num = 0;     // 总考虑的点数

    // 遍历p周围的点
    for (int ind_hor = -depth_cons_hor_num2; ind_hor <= depth_cons_hor_num2; ind_hor++)
    {
        for (int ind_ver = -depth_cons_ver_num2; ind_ver <= depth_cons_ver_num2; ind_ver++)
        {
            // 计算新的位置索引
            int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF + ((p.ver_ind + ind_ver) % MAX_1D_HALF);
            if (pos_new < 0 || pos_new >= MAX_2D_N)
                continue;

            // 获取当前位置的所有点
            const vector<point_soph *> &points_in_pixel = map_info.depth_map[pos_new];
            // 遍历所有点，判断深度一致性
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph *point = points_in_pixel[j];
                // 判断时间差和空间位置差
                if (fabs(point->time - p.time) < frame_dur && fabs(point->vec(0) - p.vec(0)) < depth_cons_hor_thr2 &&
                    fabs(point->vec(1) - p.vec(1)) < depth_cons_ver_thr2)
                {
                    all_num++;
                    if (point->dyn == STATIC)
                    {
                        // 计算深度差
                        float cur_minus = p.vec(2) - point->vec(2);
                        if (fabs(cur_minus) < depth_cons_depth_max_thr2)
                        {
                            num++;                                       // 统计满足深度差条件的静态点
                            all_minus += fabs(point->vec(2) - p.vec(2)); // 累加满足条件的深度差绝对值
                        }
                        else if (cur_minus > 0)
                        {
                            smaller_num++; // 统计深度小于p的点
                        }
                        else
                        {
                            greater_num++; // 统计深度大于p的点
                        }
                    }
                }
            }
        }
    }
    // 基于上述统计进行判断
    if (all_num > 0)
    {
        if (num > 1) // 如果有多于一个点与p深度接近
        {
            // 计算深度差异阈值
            float cur_depth_thr = max(depth_cons_depth_thr2, k_depth2 * p.vec(2));
            if (all_minus / (num - 1) > cur_depth_thr)
            {
                return false; // 如果平均深度差大于阈值，则认为深度不一致
            }
        }
        if (greater_num == 0 || smaller_num == 0)
        {
            return true; // 如果所有点要么全部在p深度之上，要么全部在p深度之下，则认为深度一致
        }
        else
        {
            return false; // 否则认为深度不一致
        }
    }
    else
    {
        return false; // 如果没有考虑的点，则认为深度不一致
    }
}

/**
 * @brief 检查两个点之间的速度差异是否在可接受的加速度范围内
 *
 * 这个函数是判断一个点是否因为另一个点的运动被遮挡的关键步骤之一。通过比较两点速度的差异是否小于等于
 * 允许的最大加速度乘以时间差，来判断这两个点之间的运动关系是否符合物理约束。
 *
 * @param v1 第一个点相对于观测点的速度
 * @param v2 第二个点（可能的遮挡点）相对于观测点的速度
 * @param delta_t 两点观测的时间差
 * @return true 如果速度差异在加速度阈值范围内，表示可能的遮挡关系；否则返回false
 */
bool DynObjFilter::Case2VelCheck(float v1, float v2, double delta_t)
{
    // 检查两个速度之间的差异是否小于等于加速度阈值乘以时间差
    if (fabs(v1 - v2) < delta_t * acc_thr2)
    {
        // 如果是，认为遮挡关系成立
        return true;
    }
    // 否则，认为遮挡关系不成立
    return false;
}

/**
 * @brief 检测当前点是否因遮挡之前深度图中的点而被认为是动态的（第三种情况）。
 *        第三种情况的动态点检测侧重于当前点遮挡了之前帧中的点。这通过比较当前点与之前帧中的点的深度关系来判断。
 *        如果当前点的深度小于被遮挡点，且满足一定的速度和一致性检查，则认为当前点是动态的。
 *
 * @param p 当前正在检查的点，包括位置、时间等信息。
 * @return 如果当前点满足动态点的条件，则返回true；否则返回false。
 */
bool DynObjFilter::Case3(point_soph &p)
{
    // 忽略KITTI的失真点
    if (dataset == 0 && p.is_distort)
        return false;

    // 获取最新深度图的索引
    int first_i = depth_map_list.size() - 1;
    if (first_i < 0) // 如果没有深度图，返回 false
        return false;

    // 对点进行球面投影
    point_soph p_spherical = p;
    SphericalProjection(p, depth_map_list[first_i]->map_index, depth_map_list[first_i]->project_R, depth_map_list[first_i]->project_T, p_spherical);

    // 如果点的位置超出了预设的边界或者深度为负，则认为点无效，继续检查下一个深度图
    if (fabs(p_spherical.hor_ind) >= MAX_1D || fabs(p_spherical.ver_ind) >= MAX_1D_HALF || p_spherical.vec(2) < 0.0f ||
        p_spherical.position < 0 || p_spherical.position >= MAX_2D_N)
    {
        p.dyn = INVALID;
        return false;
    }

    // 当前点遮挡其他点的次数
    int cur_occ_times = 0;

    // 如果点通过初步进入检查（即，它在深度上小于所有深度图的点）
    if (Case3Enter(p_spherical, *depth_map_list[first_i]))
    {
        // 如果不满足地图一致性
        if (!Case3MapConsistencyCheck(p_spherical, *depth_map_list[first_i], case3_interp_en))
        {
            // 初始化变量，用于记录遮挡检测和速度检测
            double ti = 0; // 时间插值
            float vi = 0;  // 速度插值
            bool map_cons = true;
            // 遍历周围像素，检查是否遮挡之前深度图中的点
            for (int ind_hor = -occ_hor_num3; ind_hor <= occ_hor_num3; ind_hor++)
            {
                for (int ind_ver = -occ_ver_num3; ind_ver <= occ_ver_num3; ind_ver++)
                {
                    // 计算新位置索引
                    int pos_new = ((p_spherical.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF + ((p_spherical.ver_ind + ind_ver) % MAX_1D_HALF);
                    if (pos_new < 0 || pos_new >= MAX_2D_N) // 跳过无效索引
                        continue;
                    // 获取像素位置的点集合
                    const vector<point_soph *> &points_in_pixel = depth_map_list[first_i]->depth_map[pos_new];
                    // 如果当前点的深度小于该位置的最大深度，说明当前点可能遮挡了之前深度图中的点
                    if (depth_map_list[first_i]->max_depth_all[pos_new] < p_spherical.vec(2))
                    {
                        continue;
                    }

                    // 检查每个点是否被遮挡
                    for (int k = 0; k < points_in_pixel.size() && map_cons; k++)
                    {
                        const point_soph *p_occ = points_in_pixel[k];
                        // 如果当前点遮挡了点p_occ，并且通过深度一致性检查
                        if (Case3IsOccluding(p_spherical, *p_occ) && Case3DepthConsistencyCheck(*p_occ, *depth_map_list[first_i]))
                        {

                            cur_occ_times = 1; // 记录遮挡次数
                            // 计算被遮挡点和当前点的平均时间和速度
                            ti = (p_occ->time + p.time) / 2;
                            vi = (p_occ->vec(2) - p_spherical.vec(2)) / (p.time - p_occ->time);

                            // 记录遮挡信息
                            p.is_occu_index[0] = depth_map_list[first_i]->map_index;
                            p.is_occu_index[1] = pos_new;
                            p.is_occu_index[2] = k;
                            p.is_occ_vec = p_spherical.vec;
                            p.is_occu_times = cur_occ_times;

                            // 在已有深度图列表中逆向遍历，寻找p1是否在更早的帧中也遮挡当前点
                            point_soph p1 = *points_in_pixel[k];
                            int i = depth_map_list.size() - 2;
                            while (i >= 0)
                            {
                                // 如果p1在当前帧没有被遮挡或者遮挡信息指向的帧在深度图列表范围之外
                                if (p1.is_occu_index[0] == -1 || p1.is_occu_index[0] < depth_map_list.front()->map_index)
                                {
                                    // 对p1进行球面投影到当前深度图i
                                    SphericalProjection(p1, depth_map_list[i]->map_index, depth_map_list[i]->project_R, depth_map_list[i]->project_T, p1);
                                    // 检查p1是否遮挡了更早深度图中的点
                                    if (Case3SearchPointOccludedbyP(p1, *depth_map_list[i]))
                                    {
                                        // 如果存在，更新p1的遮挡信息
                                        p1.is_occ_vec = p1.vec;
                                    }
                                    else
                                    {
                                        // 如果不存在遮挡点，跳出循环
                                        break;
                                    }
                                }
                                // 更新遍历的深度图索引为p1遮挡信息指向的深度图索引
                                i = p1.is_occu_index[0] - depth_map_list.front()->map_index;
                                // 获取p1遮挡的点p2
                                point_soph *p2 = depth_map_list[i]->depth_map[p1.is_occu_index[1]][p1.is_occu_index[2]];
                                SphericalProjection(p, depth_map_list[i]->map_index, depth_map_list[i]->project_R, depth_map_list[i]->project_T, p);
                                // 检查当前点p在深度图i中的地图一致性
                                if (Case3MapConsistencyCheck(p, *depth_map_list[i], case3_interp_en))
                                {
                                    // 如果不一致，则p点为静态点，更新地图一致性标志为false并跳出循环
                                    map_cons = false;
                                    break;
                                }
                                // 计算被遮挡点p2与p1之间的速度、时间的平均值
                                float vc = -(p1.is_occ_vec(2) - p2->vec(2)) / (p1.time - p2->time);
                                double tc = (p2->time + p1.time) / 2;
                                // 检查当前点p是否遮挡p2，并且深度一致性及速度检查通过
                                if (Case3IsOccluding(p, *p2) &&
                                    Case3DepthConsistencyCheck(*p2, *depth_map_list[i]) && Case3VelCheck(vi, vc, ti - tc))
                                {
                                    // 增加遮挡次数
                                    cur_occ_times += 1;
                                    // 如果遮挡次数达到阈值，标记为动态点并返回true
                                    if (cur_occ_times >= occluding_times_thr3)
                                    {
                                        p.is_occu_times = cur_occ_times;
                                        return true;
                                    }
                                    // 更新遮挡点信息以便下一轮检查c
                                    p1 = *p2;
                                    vi = vc;
                                    ti = tc;
                                }
                                else
                                {
                                    // 如果不满足遮挡条件，跳出循环
                                    break;
                                }
                                // 移至更早的深度图
                                i--;
                            }
                        }
                        if (cur_occ_times >= occluding_times_thr3)
                            break;
                    }
                    if (cur_occ_times >= occluding_times_thr3)
                        break;
                }
                if (cur_occ_times >= occluding_times_thr3)
                    break;
            }
        }
    }
    // 如果最终被遮挡次数超过阈值，则认为点是动态的
    if (cur_occ_times >= occluding_times_thr3)
    {
        p.is_occu_times = cur_occ_times;
        return true;
    }
    return false;
}

bool DynObjFilter::Case3Enter(point_soph &p, const DepthMap &map_info)
{
    if (p.dyn != STATIC)
    {
        return false;
    }
    float min_depth = 0;
    float depth_thr3_final = max(cutoff_value, k_depth_max_thr3 * (p.vec(2) - d_depth_max_thr3)) + occ_depth_thr3;
    if (map_info.depth_map[p.position].size() > 0)
    {
        const point_soph *min_point = map_info.depth_map[p.position][map_info.min_depth_index_all[p.position]];
        min_depth = min_point->vec(2);
        float delta_t = (p.time - min_point->time);
        depth_thr3_final = min(depth_thr3_final, v_min_thr3 * delta_t);
    }
    if (dataset == 0 && p.is_distort)
    {
        depth_thr3_final = enlarge_distort * depth_thr3_final;
    }
    if (p.vec(2) < min_depth - depth_thr3_final)
    {
        case3_num++;
        return true;
    }
    else
    {
        return false;
    }
}

bool DynObjFilter::Case3MapConsistencyCheck(point_soph &p, const DepthMap &map_info, bool interp)
{
    float cur_v_min = v_min_thr3;
    float cur_hor = map_cons_hor_thr3;
    float cur_ver = map_cons_ver_thr3;
    float cur_depth = max(cutoff_value, k_depth_max_thr3 * (p.vec(2) - d_depth_max_thr3)) + map_cons_depth_thr3;
    if (dataset == 0 && p.is_distort)
        cur_v_min = enlarge_distort * cur_v_min;
    for (int ind_hor = -map_cons_hor_num3; ind_hor <= map_cons_hor_num3; ind_hor++)
    {
        for (int ind_ver = -map_cons_ver_num3; ind_ver <= map_cons_ver_num3; ind_ver++)
        {
            int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF + ((p.ver_ind + ind_ver) % MAX_1D_HALF);
            if (pos_new < 0 || pos_new >= MAX_2D_N)
                continue;
            const vector<point_soph *> &points_in_pixel = map_info.depth_map[pos_new];
            if (map_info.max_depth_all[pos_new] > p.vec(2) + cur_depth &&
                map_info.min_depth_all[pos_new] < p.vec(2) - cur_depth)
            {
                continue;
            }
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph *point = points_in_pixel[j];
                if (point->dyn == STATIC &&
                    fabs(p.time - point->time) > frame_dur &&
                    (point->vec(2) - p.vec(2)) < cur_depth && \ 
                    fabs(p.vec(0) - point->vec(0)) < cur_hor &&
                    fabs(p.vec(1) - point->vec(1)) < cur_ver)
                {

                    return true;
                }
            }
        }
    }
    if (interp && (p.local(0) < self_x_b || p.local(0) > self_x_f || p.local(1) > self_y_l || p.local(1) < self_y_r))
    {
        float cur_interp = interp_thr3 * (depth_map_list.back()->map_index - map_info.map_index + 1);
        float depth_all = DepthInterpolationAll(p, map_info.map_index, map_info.depth_map);
        if (fabs(p.vec(2) - depth_all) < cur_interp)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

bool DynObjFilter::Case3SearchPointOccludedbyP(point_soph &p, const DepthMap &map_info)
{
    for (int ind_hor = -occ_hor_num3; ind_hor <= occ_hor_num3; ind_hor++)
    {
        for (int ind_ver = -occ_ver_num3; ind_ver <= occ_ver_num3; ind_ver++)
        {
            int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF + ((p.ver_ind + ind_ver) % MAX_1D_HALF);
            if (pos_new < 0 || pos_new >= MAX_2D_N)
                continue;
            const vector<point_soph *> &points_in_pixel = map_info.depth_map[pos_new];
            if (map_info.min_depth_all[pos_new] > p.vec(2))
            {
                continue;
            }
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph *p_cond = points_in_pixel[j];
                if (Case3IsOccluding(p, *p_cond) && Case3DepthConsistencyCheck(*p_cond, map_info))
                {
                    p.is_occu_index[0] = map_info.map_index;
                    p.is_occu_index[1] = pos_new;
                    p.is_occu_index[2] = j;
                    p.occ_vec = p.vec;
                    return true;
                }
            }
        }
    }
    return false;
}

bool DynObjFilter::Case3IsOccluding(const point_soph &p, const point_soph &p_occ)
{
    if ((dataset == 0 && p_occ.is_distort) || (dataset == 0 && p.is_distort) || p_occ.dyn == INVALID)
        return false;
    if ((p.local(0) > self_x_b && p.local(0) < self_x_f && p.local(1) < self_y_l && p.local(1) > self_y_r) ||
        (p_occ.local(0) > self_x_b && p_occ.local(0) < self_x_f && p_occ.local(1) < self_y_l && p_occ.local(1) > self_y_r))
    {
        return false;
    }
    float delta_t = p.time - p_occ.time;
    if (delta_t > 0)
    {
        float depth_thr3_final = min(max(cutoff_value, k_depth_max_thr3 * (p.vec(2) - d_depth_max_thr3)) + map_cons_depth_thr3, v_min_thr3 * delta_t);
        if (dataset == 0 && p.is_distort)
            depth_thr3_final = enlarge_distort * depth_thr3_final;
        if (p_occ.vec(2) > p.vec(2) + depth_thr3_final &&
            fabs(p.vec(0) - p_occ.vec(0)) < occ_hor_thr3 &&
            fabs(p.vec(1) - p_occ.vec(1)) < occ_ver_thr3)
        {
            return true;
        }
    }
    return false;
}

bool DynObjFilter::Case3DepthConsistencyCheck(const point_soph &p, const DepthMap &map_info)
{
    float all_minus = 0;
    int num = 0, smaller_num = 0, all_num = 0, greater_num = 0; //
    for (int ind_hor = -depth_cons_hor_num3; ind_hor <= depth_cons_hor_num3; ind_hor++)
    {
        for (int ind_ver = -depth_cons_ver_num3; ind_ver <= depth_cons_ver_num3; ind_ver++)
        {
            int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF + ((p.ver_ind + ind_ver) % MAX_1D_HALF);
            if (pos_new < 0 || pos_new >= MAX_2D_N)
                continue;
            const vector<point_soph *> &points_in_pixel = map_info.depth_map[pos_new];
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph *point = points_in_pixel[j];
                if (fabs(point->time - p.time) < frame_dur && fabs(point->vec(0) - p.vec(0)) < depth_cons_hor_thr3 &&
                    fabs(point->vec(1) - p.vec(1)) < depth_cons_ver_thr3)
                {
                    all_num++;
                    if (point->dyn == STATIC)
                    {
                        float cur_minus = p.vec(2) - point->vec(2);
                        if (fabs(cur_minus) < depth_cons_depth_max_thr3)
                        {
                            num++;
                            all_minus += fabs(point->vec(2) - p.vec(2));
                        }
                        else if (cur_minus > 0)
                        {
                            smaller_num++;
                        }
                        else
                        {
                            greater_num++;
                        }
                    }
                }
            }
        }
    }
    if (all_num > 0)
    {
        if (num > 1)
        {
            float cur_depth_thr = max(depth_cons_depth_thr3, k_depth3 * p.vec(2));
            if (all_minus / (num - 1) > cur_depth_thr)
            {
                return false;
            }
        }
        if (greater_num == 0 || smaller_num == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

bool DynObjFilter::Case3VelCheck(float v1, float v2, double delta_t)
{
    if (fabs(v1 - v2) < delta_t * acc_thr3)
    {
        return true;
    }
    return false;
}

void DynObjFilter::Points2Buffer(vector<point_soph *> &points, std::vector<int> &index_vector)
{
    int cur_tail = buffer.tail;
    buffer.push_parallel_prepare(points.size());
    std::for_each(std::execution::par, index_vector.begin(), index_vector.end(), [&](const int &i)
                  { buffer.push_parallel(points[i], cur_tail + i); });
}

void DynObjFilter::Buffer2DepthMap(double cur_time)
{
    int len = buffer.size();
    double total_0 = 0.0;
    double total_1 = 0.0;
    double total_2 = 0.0;
    double total_3 = 0.0;
    double t = 0.0;
    int max_point = 0;
    for (int k = 0; k < len; k++)
    {
        point_soph *point = buffer.front();
        if ((cur_time - point->time) >= buffer_delay - frame_dur / 2.0)
        {
            if (depth_map_list.size() == 0)
            {
                if (depth_map_list.size() < max_depth_map_num)
                {
                    map_index++;
                    DepthMap::Ptr new_map_pointer(new DepthMap(point->rot, point->transl, point->time, map_index));
                    depth_map_list.push_back(new_map_pointer);
                }
                else
                {
                    buffer.pop();
                    continue;
                }
            }
            else if ((point->time - depth_map_list.back()->time) >= depth_map_dur - frame_dur / 2.0)
            {
                map_index++;
                if (depth_map_list.size() == max_depth_map_num)
                {
                    depth_map_list.front()->Reset(point->rot, point->transl, point->time, map_index);
                    DepthMap::Ptr new_map_pointer = depth_map_list.front();
                    depth_map_list.pop_front();
                    depth_map_list.push_back(new_map_pointer);
                }
                else if (depth_map_list.size() < max_depth_map_num)
                {
                    DepthMap::Ptr new_map_pointer(new DepthMap(point->rot, point->transl, point->time, map_index));
                    depth_map_list.push_back(new_map_pointer);
                }
            }
            switch (point->dyn)
            {
                if (depth_map_list.back()->depth_map.size() <= point->position)
                case STATIC:
                    SphericalProjection(*point, depth_map_list.back()->map_index, depth_map_list.back()->project_R, depth_map_list.back()->project_T, *point);
                if (depth_map_list.back()->depth_map[point->position].size() < max_pixel_points)
                {
                    depth_map_list.back()->depth_map[point->position].push_back(point);
                    if (point->vec(2) > depth_map_list.back()->max_depth_all[point->position])
                    {
                        depth_map_list.back()->max_depth_all[point->position] = point->vec(2);
                        depth_map_list.back()->max_depth_index_all[point->position] = depth_map_list.back()->depth_map[point->position].size() - 1;
                    }
                    if (point->vec(2) < depth_map_list.back()->min_depth_all[point->position] ||
                        depth_map_list.back()->min_depth_all[point->position] < 10E-5)
                    {
                        depth_map_list.back()->min_depth_all[point->position] = point->vec(2);
                        depth_map_list.back()->min_depth_index_all[point->position] = depth_map_list.back()->depth_map[point->position].size() - 1;
                    }
                    if (point->vec(2) < depth_map_list.back()->min_depth_static[point->position] ||
                        depth_map_list.back()->min_depth_static[point->position] < 10E-5)
                    {
                        depth_map_list.back()->min_depth_static[point->position] = point->vec(2);
                    }
                    if (point->vec(2) > depth_map_list.back()->max_depth_static[point->position])
                    {
                        depth_map_list.back()->max_depth_static[point->position] = point->vec(2);
                    }
                }
                break;
            case CASE1:

            case CASE2:

            case CASE3:
                SphericalProjection(*point, depth_map_list.back()->map_index, depth_map_list.back()->project_R, depth_map_list.back()->project_T, *point);
                if (depth_map_list.back()->depth_map[point->position].size() < max_pixel_points)
                {
                    depth_map_list.back()->depth_map[point->position].push_back(point);
                    if (point->vec(2) > depth_map_list.back()->max_depth_all[point->position])
                    {
                        depth_map_list.back()->max_depth_all[point->position] = point->vec(2);
                        depth_map_list.back()->max_depth_index_all[point->position] = depth_map_list.back()->depth_map[point->position].size() - 1;
                    }
                    if (point->vec(2) < depth_map_list.back()->min_depth_all[point->position] ||
                        depth_map_list.back()->min_depth_all[point->position] < 10E-5)
                    {
                        depth_map_list.back()->min_depth_all[point->position] = point->vec(2);
                        depth_map_list.back()->min_depth_index_all[point->position] = depth_map_list.back()->depth_map[point->position].size() - 1;
                    }
                }
                break;
            default:
                break;
            }
            buffer.pop();
        }
        else
        {
            break;
        }
    }
    if (debug_en)
    {
        for (int i = 0; i < depth_map_list.size(); i++)
        {
            for (int j = 0; j < depth_map_list[i]->depth_map.size(); j++)
            {
                for (int k = 0; k < depth_map_list[i]->depth_map[j].size(); k++)
                {
                    PointType po;
                    point_soph *point = depth_map_list[i]->depth_map[j][k];
                    po.x = point->glob(0);
                    po.y = point->glob(1);
                    po.z = point->glob(2);
                    po.intensity = point->local(2);
                    po.curvature = point->local(1);
                    po.normal_x = point->hor_ind;
                    po.normal_y = point->ver_ind;
                    po.normal_z = point->dyn;
                    if (point->dyn == STATIC)
                        laserCloudSteadObj_hist->push_back(po);
                }
            }
        }
    }
}

void DynObjFilter::publish_dyn(const ros::Publisher &pub_point_out, const ros::Publisher &pub_frame_out, const ros::Publisher &pub_steady_points, const double &scan_end_time)
{
    if (cluster_coupled) // pubLaserCloudEffect pub_pcl_dyn_extend  pubLaserCloudEffect_depth
    {
        cout << "Found Dynamic Objects, numbers: " << laserCloudDynObj_clus->points.size() << " Total time: " << time_total << " Average total time: " << time_total_avr << endl;
    }
    else
    {
        cout << "Found Dynamic Objects, numbers: " << laserCloudDynObj->points.size() << " Total time: " << time_total << " Average total time: " << time_total_avr << endl;
    }
    cout << "case1 num: " << case1_num << " case2 num: " << case2_num << " case3 num: " << case3_num << endl;
    case1_num = 0;
    case2_num = 0;
    case3_num = 0;
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudDynObj_world, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(scan_end_time);
    laserCloudFullRes3.header.frame_id = frame_id;
    pub_point_out.publish(laserCloudFullRes3);
    if (cluster_coupled || cluster_future)
    {
        sensor_msgs::PointCloud2 laserCloudFullRes4;
        pcl::toROSMsg(*laserCloudDynObj_clus, laserCloudFullRes4);
        laserCloudFullRes4.header.stamp = ros::Time().fromSec(scan_end_time);
        laserCloudFullRes4.header.frame_id = frame_id;
        pub_frame_out.publish(laserCloudFullRes4);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes2;
    PointCloudXYZI::Ptr laserCloudSteadObj_pub(new PointCloudXYZI);
    if (cluster_coupled)
    {
        if (laserCloudSteadObj_accu_times < laserCloudSteadObj_accu_limit)
        {
            laserCloudSteadObj_accu_times++;
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj_clus);
            for (int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        else
        {
            laserCloudSteadObj_accu.pop_front();
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj_clus);
            for (int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        pcl::VoxelGrid<PointType> downSizeFiltermap;
        downSizeFiltermap.setLeafSize(voxel_filter_size, voxel_filter_size, voxel_filter_size);
        downSizeFiltermap.setInputCloud(laserCloudSteadObj_pub);
        PointCloudXYZI laserCloudSteadObj_down;
        downSizeFiltermap.filter(laserCloudSteadObj_down);
        pcl::toROSMsg(laserCloudSteadObj_down, laserCloudFullRes2);
    }
    else
    {
        cout << "Found Steady Objects, numbers: " << laserCloudSteadObj->points.size() << endl;
        if (laserCloudSteadObj_accu_times < laserCloudSteadObj_accu_limit)
        {
            laserCloudSteadObj_accu_times++;
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj);
            for (int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        else
        {
            laserCloudSteadObj_accu.pop_front();
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj);
            for (int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        pcl::toROSMsg(*laserCloudSteadObj_pub, laserCloudFullRes2);
    }
    laserCloudFullRes2.header.stamp = ros::Time().fromSec(scan_end_time);
    laserCloudFullRes2.header.frame_id = frame_id;
    pub_steady_points.publish(laserCloudFullRes2);
}

