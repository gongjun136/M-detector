/*
 * @Author: gongjun136 gongjun136@gmail.com
 * @Date: 2024-03-11 19:40:33
 * @LastEditors: gongjun136 gongjun136@gmail.com
 * @LastEditTime: 2024-03-22 11:20:11
 * @FilePath: /catkin_ws_M-detector/src/M-detector-noted/src/dynfilter_with_odom.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include <ros/ros.h>
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <iostream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <pcl/filters/random_sample.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <glog/logging.h>
#include <deque>

#include "types.h"
#include "m-detector-noted/dyn_obj_filter.h"

using namespace std;

shared_ptr<DynObjFilter> DynObjFilt(new DynObjFilter());

// int     QUAD_LAYER_MAX  = 1;
// int     occlude_windows = 3;
// int     point_index = 0;
// float   VER_RESOLUTION_MAX  = 0.01;
// float   HOR_RESOLUTION_MAX  = 0.01;
// float   angle_noise     = 0.001;
// float   angle_occlude     = 0.02;
// float   dyn_windows_dur = 0.5;
// bool    dyn_filter_en = true, dyn_filter_dbg_en = true;

// int     dataset = 0;

ros::Publisher pub_pcl_dyn, pub_pcl_dyn_extend, pub_pcl_std;

// 轨迹回调函数
deque<M3D> buffer_rots;     // 轨迹(旋转部分)缓存区
deque<V3D> buffer_poss;     // 轨迹(平移部分)缓存区
deque<double> buffer_times; // 轨迹时间戳缓存区
M3D cur_rot = Eigen::Matrix3d::Identity();
V3D cur_pos = Eigen::Vector3d::Zero();
double lidar_end_time = 0;
void OdomCallback(const nav_msgs::Odometry &cur_odom)
{
    Eigen::Quaterniond cur_q;
    geometry_msgs::Quaternion tmp_q;
    tmp_q = cur_odom.pose.pose.orientation;
    tf::quaternionMsgToEigen(tmp_q, cur_q);
    cur_rot = cur_q.matrix();
    cur_pos << cur_odom.pose.pose.position.x, cur_odom.pose.pose.position.y, cur_odom.pose.pose.position.z;
    buffer_rots.push_back(cur_rot);
    buffer_poss.push_back(cur_pos);
    lidar_end_time = cur_odom.header.stamp.toSec();
    buffer_times.push_back(lidar_end_time);
}

// 点云回调函数，把点云保存到缓存区中
deque<boost::shared_ptr<PointCloudXYZI>> buffer_pcs; // 订阅点云的缓存区
void PointsCallback(const sensor_msgs::PointCloud2ConstPtr &msg_in)
{
    boost::shared_ptr<PointCloudXYZI> feats_undistort(new PointCloudXYZI());
    pcl::fromROSMsg(*msg_in, *feats_undistort);
    buffer_pcs.push_back(feats_undistort);
}

// 周期性回调函数，进行动态物体检测
// out_folder：聚类前的动态标签文件保存路径，out_folder_origin：聚类后的动态标签文件保存路径
string out_folder, out_folder_origin;
int cur_frame = 0;
void TimerCallback(const ros::TimerEvent &e)
{
    // 检查缓存区是否有新数据
    if (buffer_pcs.size() > 0 && buffer_poss.size() > 0 && buffer_rots.size() > 0 && buffer_times.size() > 0)
    {
        // 获取并移除点云数据及其相关的位姿和时间戳
        boost::shared_ptr<PointCloudXYZI> cur_pc = buffer_pcs.at(0);
        buffer_pcs.pop_front();
        auto cur_rot = buffer_rots.at(0);
        buffer_rots.pop_front();
        auto cur_pos = buffer_poss.at(0);
        buffer_poss.pop_front();
        auto cur_time = buffer_times.at(0);
        buffer_times.pop_front();

        // 准备输出文件的路径，用于保存处理结果
        string file_name = out_folder;
        stringstream ss;
        ss << setw(6) << setfill('0') << cur_frame;
        file_name += ss.str();
        file_name.append(".label");
        string file_name_origin = out_folder_origin;
        stringstream sss;
        sss << setw(6) << setfill('0') << cur_frame;
        file_name_origin += sss.str();
        file_name_origin.append(".label");

        // 如果文件名长度超过预设值，则设置输出文件路径，记录动态标签
        if (file_name.length() > 15 || file_name_origin.length() > 15)
            DynObjFilt->set_path(file_name, file_name_origin);

        // 动态物体剔除
        DynObjFilt->filter(cur_pc, cur_rot, cur_pos, cur_time);
        // 发布动态物体处理后的点云
        DynObjFilt->publish_dyn(pub_pcl_dyn, pub_pcl_dyn_extend, pub_pcl_std, cur_time);
        cur_frame++;
    }
}

string points_topic, odom_topic;
int main(int argc, char **argv)
{
    // 初始化log
    google::InitGoogleLogging(argv[0]);
    std::string log_dir = std::string(ROOT_DIR) + "log";
    FLAGS_log_dir = log_dir;
    FLAGS_alsologtostderr = true;

    // 初始化(未封装)main函数需要用到的参数
    ros::init(argc, argv, "dynfilter_odom");
    ros::NodeHandle nh;
    nh.param<string>("dyn_obj/points_topic", points_topic, "");
    nh.param<string>("dyn_obj/odom_topic", odom_topic, "");
    nh.param<string>("dyn_obj/out_file", out_folder, "");
    nh.param<string>("dyn_obj/out_file_origin", out_folder_origin, "");
    // LOG(INFO) << "out_file: " << out_folder;

    // 初始化(封装)其他参数
    DynObjFilt->init(nh);
    // 初始化ROS发布者0
    pub_pcl_dyn_extend = nh.advertise<sensor_msgs::PointCloud2>("/m_detector/frame_out", 10000);
    pub_pcl_dyn = nh.advertise<sensor_msgs::PointCloud2>("/m_detector/point_out", 100000);
    pub_pcl_std = nh.advertise<sensor_msgs::PointCloud2>("/m_detector/std_points", 100000);
    // 订阅点云和轨迹，并进入回调函数保存到缓存区
    ros::Subscriber sub_pcl = nh.subscribe(points_topic, 200000, PointsCallback);
    ros::Subscriber sub_odom = nh.subscribe(odom_topic, 200000, OdomCallback);
    // 0.01s周期性回调，进行动态物体剔除
    ros::Timer timer = nh.createTimer(ros::Duration(0.01), TimerCallback);

    ros::spin();
    google::ShutdownGoogleLogging();
    return 0;
}