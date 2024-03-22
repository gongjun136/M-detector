/*
 * @Author: gongjun136 gongjun136@gmail.com
 * @Date: 2024-03-22 13:42:06
 * @LastEditors: gongjun136 gongjun136@gmail.com
 * @LastEditTime: 2024-03-22 16:14:40
 * @FilePath: /catkin_ws_M-detector/src/M-detector-noted/src/cal_recall_multi.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <iostream>
#include <csignal>
#include <unistd.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <unistd.h>
#include <dirent.h>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <glog/logging.h>

#include "m-detector-noted/dyn_obj_filter.h"

using namespace std;

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

pcl::PointCloud<pcl::PointXYZINormal> lastcloud;
PointCloudXYZI::Ptr last_pc(new PointCloudXYZI());
ros::Publisher pub_pointcloud, pub_marker, pub_iou_view;

// 结果数据总目录
string dataset_folder;
// label_folder：真实标签结果目录，pred_folder：预测标签结果目录
string label_folder, pred_folder;
// 输出结果目录
string recall_folder, recall_origin_folder;

string semantic_folder, out_folder;
string recall_file;  // 结果数据写入文件
ofstream recall_rec; // 结果数据写入文件流

// 初始化几个unordered_map变量来建立数据集中对象类别编号与其字符串表示之间的关系
std::unordered_map<int, std::string> objects_types_map_kitti;
std::unordered_map<int, std::string> objects_types_map_nuscenes;
std::unordered_map<int, int> objects_class_map_nuscenes;
std::unordered_map<int, std::string> objects_types_map_waymo;
void Init()
{
    // 初始化KITTI数据集的对象类型映射
    objects_types_map_kitti[0] = "Person";
    objects_types_map_kitti[1] = "Truck";
    objects_types_map_kitti[2] = "Car";
    objects_types_map_kitti[3] = "Tram";
    objects_types_map_kitti[4] = "Pedestrain"; // 注意：这里可能是“Pedestrian”的拼写错误
    objects_types_map_kitti[5] = "Cyclist";
    objects_types_map_kitti[6] = "Van";
    // 初始化NuScenes数据集的对象类型映射
    objects_types_map_nuscenes[0] = "Animal";
    objects_types_map_nuscenes[1] = "Pedestrian";
    objects_types_map_nuscenes[2] = "Movable_object";
    objects_types_map_nuscenes[3] = "Bicycle";
    objects_types_map_nuscenes[4] = "Bus";
    objects_types_map_nuscenes[5] = "Car";
    objects_types_map_nuscenes[6] = "Emergency";
    objects_types_map_nuscenes[7] = "Motorcycle";
    objects_types_map_nuscenes[8] = "Trailer";
    objects_types_map_nuscenes[9] = "Truck";
    objects_types_map_nuscenes[10] = "Ego";
    // 初始化NuScenes数据集的对象类别映射，将类别编号映射到特定的类别
    objects_class_map_nuscenes[1] = 0;
    objects_class_map_nuscenes[2] = 1;
    objects_class_map_nuscenes[3] = 1;
    objects_class_map_nuscenes[4] = 1;
    objects_class_map_nuscenes[5] = 1;
    objects_class_map_nuscenes[6] = 1;
    objects_class_map_nuscenes[7] = 1;
    objects_class_map_nuscenes[8] = 1;
    objects_class_map_nuscenes[9] = 2;
    objects_class_map_nuscenes[10] = 2;
    objects_class_map_nuscenes[11] = 2;
    objects_class_map_nuscenes[12] = 2;
    objects_class_map_nuscenes[14] = 3;
    objects_class_map_nuscenes[15] = 4;
    objects_class_map_nuscenes[16] = 4;
    objects_class_map_nuscenes[17] = 5;
    objects_class_map_nuscenes[18] = 6;
    objects_class_map_nuscenes[19] = 6;
    objects_class_map_nuscenes[20] = 6;
    objects_class_map_nuscenes[21] = 7;
    objects_class_map_nuscenes[22] = 8;
    objects_class_map_nuscenes[23] = 9;
    objects_class_map_nuscenes[31] = 10;
    // 初始化Waymo数据集的对象类型映射
    objects_types_map_waymo[0] = "Vehicle";
    objects_types_map_waymo[1] = "Pedestrian";
    objects_types_map_waymo[2] = "Cyclist";
}

/**
 * \brief 计算KITTI数据集上的召回率。
 *
 * 该函数遍历KITTI数据集的每个标签文件和对应的预测文件，通过比较预测结果和真实标签来计算真正例（TP）、
 * 假负例（FN）和假正例（FP）。基于这些统计量，函数计算并记录车辆、行人和骑行者三个类别的召回率。
 *
 * \param recalls_vehicle 存储车辆类别的召回率列表。
 * \param recalls_pedestrian 存储行人类别的召回率列表。
 * \param recalls_cyclist 存储骑行者类别的召回率列表。
 */
void KittiCalRecall(std::vector<float> &recalls_vehicle, std::vector<float> &recalls_pedestrian, std::vector<float> &recalls_cyclist)
{
    // 定义并初始化局部变量用于记录标签文件和预测文件的数量
    int label_file_num = 0;
    int pred_file_num = 0;

    // 检查并统计标签文件夹中的文件数量
    if (label_folder != "")
    {
        // 使用opendir尝试打开指定的标签文件夹c
        DIR *label_dir;
        label_dir = opendir(label_folder.c_str());
        struct dirent *label_ptr; // 使用opendir尝试打开指定的标签文件夹
        // 逐个读取文件夹中的文件或文件夹
        while ((label_ptr = readdir(label_dir)) != NULL)
        {
            // 忽略当前目录"."和上级目录".."的条目
            if (label_ptr->d_name[0] == '.')
            {
                continue;
            }
            label_file_num++; // 对于每个有效的文件，增加标签文件数量计数器
        }
        closedir(label_dir); // 完成遍历后关闭文件夹
    }
    else
    {
        // 如果标签文件夹路径为空，则输出错误信息并返回
        cout << "less label " << label_folder << endl;
        return;
    }
    // 检查预测结果文件夹中的文件数量
    if (pred_folder != "")
    {
        DIR *pred_dir;
        pred_dir = opendir(pred_folder.c_str());
        struct dirent *pred_ptr;
        while ((pred_ptr = readdir(pred_dir)) != NULL)
        {
            if (pred_ptr->d_name[0] == '.')
            {
                continue;
            }
            pred_file_num++;
        }
        closedir(pred_dir);
    }
    else
    {
        cout << "less pred " << pred_folder << endl;
        return;
    }

    // 确保标签文件和预测文件的数量匹配
    if (label_file_num != pred_file_num) // || pred_num != pred_origin_num
    {
        cout << "file num error " << label_folder << endl;
    }

    // 初始化用于召回率计算的变量
    vector<int> class_nums;
    vector<float> class_recalls;
    vector<float> class_var;
    int total_tp = 0, total_fn = 0, total_fp = 0, total_op = 0, total_tn = 0.0;
    std::vector<int> objects_numbers_kitti(7, 0);
    std::vector<float> average_recalls_kitti(7, 0.0);
    std::vector<float> average_recalls2_kitti(7, 0.0);

    // 遍历每个标签文件和对应的预测文件
    for (int frames = 0; frames < label_file_num; frames++)
    {
        // 输出当前处理的帧编号
        cout << "frame: " << frames << endl;
        stringstream ss;
        ss << setw(6) << setfill('0') << frames;

        // 构建当前帧的标签文件路径
        string label_file = label_folder;
        label_file += ss.str();
        label_file.append(".label");

        // 构建当前帧的预测文件路径
        string pred_file = pred_folder;
        pred_file += ss.str();
        pred_file.append(".label");

        // 打开标签文件，准备读取
        std::fstream label_input(label_file.c_str(), std::ios::in | std::ios::binary);
        if (!label_input.good()) // 如果文件打开失败，输出错误信息并退出程序
        {
            std::cerr << "Could not read label file: " << label_file << std::endl;
            exit(EXIT_FAILURE);
        }
        // 将文件指针移至文件开始位置
        label_input.seekg(0, std::ios::beg);

        // 打开预测文件，准备读取
        std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary);
        if (!pred_input.good())
        {
            std::cerr << "Could not read prediction file: " << pred_file << std::endl;
            exit(EXIT_FAILURE);
        }

        // 初始化统计变量
        int tp = 0, fn = 0, fp = 0, op = 0, tn = 0, count = 0; // 真正例、假负例、假正例、其他正例、真负例和总计数
        float iou = 0.0f;                                      // 交并比（Intersection over Union）

        // 用于记录每个类别标签和预测的映射，键为类别编号，值为该类别的计数
        std::unordered_map<int, int> labels_map;
        std::unordered_map<int, int> predicts_map;
        // 读取并处理预测文件中的每个预测结果
        while (pred_input.good() && !pred_input.eof())
        {
            int pred_num = -1;                               // 预测的类别编号
            pred_input.read((char *)&pred_num, sizeof(int)); // 读取预测的类别编号
            pred_num = pred_num & 0xFFFF;                    // 保留有效位

            int label_num = -1, id = -1;                       // 标签文件中的实际类别编号
            label_input.read((char *)&label_num, sizeof(int)); // 读取实际的类别编号
            label_num = label_num & 0xFFFF;                    // 保留有效位

            // 根据标签编号和预测编号更新统计变量（tp, fn, fp, tn, op）
            // 判断标签编号，分类更新统计变量
            if (label_num >= 251 && label_num < 65535) // 动态物体的标签编号范围
            {
                // 如果该标签编号在labels_map中不存在，则初始化为1，否则计数加1
                if (!labels_map.count(label_num))
                    labels_map[label_num] = 1;
                else
                    labels_map[label_num] += 1;

                // 如果预测编号也在动态物体范围内，则视为真正例（TP）
                if (pred_num >= 251 && pred_num < 65535)
                // if(pred_num < 40)
                {
                    tp += 1; // 真正例计数加1

                    // 同时更新预测映射计数
                    if (!predicts_map.count(label_num))
                        predicts_map[label_num] = 1;
                    else
                        predicts_map[label_num] += 1;
                }
                else if (pred_num == 9) // 如果预测编号不在动态物体范围内，视为假负例（FN）
                {
                    fn += 1; //
                }
                count += 1; // 处理的动态物体总数加1
            }
            else if (label_num < 251 && label_num < 65535) // 如果标签编号表示非动态物体
            {
                // 如果预测为动态物体，视为假正例（FP）
                if (pred_num >= 251 && pred_num < 65535)
                {
                    fp += 1;
                }
                else if (pred_num == 9) // 特定条件下，视为真负例（TN）
                {
                    tn += 1;
                }
            }
            else if (label_num >= 65535) // 标签编号在特定范围，可能表示其他类别
            {
                // 如果预测为动态物体，更新其他正例（OP）计数
                if (pred_num >= 251 && pred_num < 65535)
                {
                    op += 1;
                }
            }
        }

        // 计算当前帧的交并比（IoU）和更新总体统计变量
        iou = ((float)tp) / (float)(tp + fn + fp);
        total_tp += tp;
        total_fn += fn;
        total_fp += fp;
        total_tn += tn;
        total_op += op;
        // 关闭文件
        label_input.close();
        pred_input.close();
        cout << "tp: " << tp << "  fp: " << fp << " fn: " << fn << " op: " << op << " tn: " << tn << " count: " << count << " iou: " << iou << endl;

        // 遍历labels_map中的每个元素，其中it是指向当前元素的迭代器
        for (auto it = labels_map.begin(); it != labels_map.end(); it++)
        {
            // 计算类别编号，假设原始编号被设计为包含类别信息（例如，编号的千位数表示类别）
            int class_num = it->first / 1000;
            // 对指定类别的物体计数加一
            objects_numbers_kitti[class_num]++;
            // 输出当前处理的标签ID、标签计数、对应的预测计数和计算得到的召回率
            std::cout << "id: " << it->first << " labels: " << it->second << " predict: " << predicts_map[it->first] << " recall: " << (float)predicts_map[it->first] / (float)it->second << std::endl;
            // 更新该类别物体的平均召回率
            average_recalls_kitti[class_num] = average_recalls_kitti[class_num] * (objects_numbers_kitti[class_num] - 1) / objects_numbers_kitti[class_num] + (float)predicts_map[it->first] / (float)it->second / objects_numbers_kitti[class_num];
            // 更新该类别物体召回率的平方的平均值，用于后续计算方差
            average_recalls2_kitti[class_num] = average_recalls2_kitti[class_num] * (objects_numbers_kitti[class_num] - 1) / objects_numbers_kitti[class_num] + (float)predicts_map[it->first] / (float)it->second * (float)predicts_map[it->first] / (float)it->second / objects_numbers_kitti[class_num];
            // 根据类别编号将召回率加入到相应的类别召回率列表中
            if (class_num == 1 || class_num == 2 || class_num == 3 || class_num == 6)
                recalls_vehicle.push_back((float)predicts_map[it->first] / (float)it->second);
            if (class_num == 4)
                recalls_pedestrian.push_back((float)predicts_map[it->first] / (float)it->second);
            if (class_num == 6)
                recalls_cyclist.push_back((float)predicts_map[it->first] / (float)it->second);
        }

        // 遍历所有类别，输出每个类别的平均召回率、物体数量和召回率方差
        for (int i = 0; i < objects_numbers_kitti.size(); i++)
        {
            cout << "average_recall of " << objects_types_map_kitti[i] << " is: " << average_recalls_kitti[i] << " objects number: " << objects_numbers_kitti[i] << " variance: " << average_recalls2_kitti[i] - average_recalls_kitti[i] * average_recalls_kitti[i] << endl;
        }

        // 输出压制平均值，即非动态物体（如背景）被正确识别的比例
        cout << "average_suppress: " << (float)(total_fn + total_tn) / (float)(total_tp + total_fp + total_tn + total_fn) << endl;
        // 输出总体统计数据：真正例、假正例、假负例、其他正例、真负例和处理的总点数
        cout << "total_tp: " << total_tp << "  total_fp: " << total_fp << " total_fn: " << total_fn << " total_op: " << total_op << " total_tn: " << total_tn << " total_points: " << total_tp + total_fp + total_fn + total_tn << endl;
        // 输出平均交并比（IoU）和处理的帧数
        cout << "Average iou: " << ((float)total_tp) / (float)(total_tp + total_fn + total_fp) << " with frames: " << frames << " , " << label_file_num << endl;

        if ((frames + 1) == label_file_num)
        {

            class_nums.clear();
            class_recalls.clear();
            class_var.clear();
            for (int i = 0; i < objects_numbers_kitti.size(); i++)
            {
                class_nums.push_back(objects_numbers_kitti[i]);
                class_recalls.push_back(average_recalls_kitti[i]);
                class_var.push_back(average_recalls2_kitti[i] - average_recalls_kitti[i] * average_recalls_kitti[i]);
            }
        }
    }

    // 将总体统计数据写入文件
    // 写入的数据依次是：真正例总数、假正例总数、假负例总数、真负例总数，计算并写入平均交并比（IoU）
    recall_rec << "Total TP: " << total_tp << ", Total FP: " << total_fp << ", Total FN: " << total_fn << ", Total TN: " << total_tn << ", ";
    // 计算并写入平均交并比（IoU），这是一个衡量检测准确度的重要指标
    recall_rec << "Average IoU: " << ((float)total_tp) / (float)(total_tp + total_fn + total_fp) << "\n";
    // 对每个类别进行遍历，将每个类别的召回率、检测到的物体数量以及召回率的方差写入文件
    recall_rec << "Class Recall, Object Count, Variance:\n";
    for (int i = 0; i < class_recalls.size(); i++)
    {
        // 类别名称，前提是有一个数组或map可以根据类别索引找到对应的类别名称
        recall_rec << "Class " << i << " (" << objects_types_map_kitti[i] << "): ";
        // 写入当前类别的召回率，并加上说明
        recall_rec << "Recall: " << class_recalls[i] << ", ";
        // 写入当前类别检测到的物体数量，并加上说明
        recall_rec << "Object Count: " << class_nums[i] << ", ";
        // 写入当前类别召回率的方差，并加上说明
        recall_rec << "Variance: " << class_var[i] << "\n";
        // recall_rec << class_recalls[i] << " " << class_nums[i] << " " << class_var[i] << " ";
    }
    // 在写完所有类别信息后换行，表示这一帧的数据记录结束
    recall_rec << endl;
}

void AviaCalRecall(std::vector<float> &recalls_vehicle, std::vector<float> &recalls_pedestrian)
{
    int label_file_num = 0;
    if(label_folder != "")
    {
        DIR* label_dir;	
        label_dir = opendir(label_folder.c_str());
        struct dirent* label_ptr;
        while((label_ptr = readdir(label_dir)) != NULL)
        {
            if(label_ptr->d_name[0] == '.') {continue;}
            label_file_num++;
        }
        closedir(label_dir);
    } 
    else
    {
        cout<<"less label "<<label_folder<<endl;
        return;
    }
    int pred_file_num = 0;
    if(pred_folder != "")
    {
        DIR* pred_dir;	
        pred_dir = opendir(pred_folder.c_str());
        struct dirent* pred_ptr;
        while((pred_ptr = readdir(pred_dir)) != NULL)
        {
            if(pred_ptr->d_name[0] == '.') {continue;}
            pred_file_num++;
        }
        closedir(pred_dir);
    } 
    else
    {
        cout<<"less pred "<<pred_folder<<endl;
        return;
    }
   
    
    if(label_file_num != pred_file_num)// || pred_num != pred_origin_num
    {
        cout<<"file num error "<<label_folder<<endl;
    }

    vector<int> class_nums;
    vector<float> class_recalls;
    vector<float> class_var;
    int total_tp = 0, total_fn = 0, total_fp = 0, total_op = 0, total_tn = 0.0;
    std::vector<int> objects_numbers_kitti(7, 0);
    std::vector<float> average_recalls_kitti(7, 0.0);
    std::vector<float> average_recalls2_kitti(7, 0.0);
    for(int frames = 0; frames < label_file_num; frames ++)
    {   
        cout << "frame: " << frames << endl;
        stringstream ss;
        ss << setw(6) << setfill('0') << frames ;
        
        string label_file = label_folder;
        label_file += ss.str(); 
        label_file.append(".label");
        
        string pred_file = pred_folder;
        pred_file += ss.str(); 
        pred_file.append(".label");

        // string pred_origin_file = pred_origin_folder;
        // pred_origin_file += ss.str(); 
        // pred_origin_file.append(".label");

        

        std::fstream label_input(label_file.c_str(), std::ios::in | std::ios::binary);
        if(!label_input.good())
        {
            std::cerr << "Could not read label file: " << label_file << std::endl;
            exit(EXIT_FAILURE);
        }
        label_input.seekg(0, std::ios::beg);

        std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary);
        if(!pred_input.good())
        {
            std::cerr << "Could not read prediction file: " << pred_file << std::endl;
            exit(EXIT_FAILURE);
        }

        

        int tp = 0, fn = 0, fp = 0, op = 0, tn = 0, count = 0;
        float iou = 0.0f;

        std::unordered_map<int, int> labels_map;
        std::unordered_map<int, int> predicts_map;

        

        while(pred_input.good() && !pred_input.eof())
        {
            

            int pred_num = -1;
            pred_input.read((char *) &pred_num, sizeof(int));
            pred_num = pred_num & 0xFFFF;

            int  label_num = -1, id = -1;
            // pred_input >> pred_num;
            label_input.read((char *) &label_num, sizeof(int));
            label_num = label_num & 0xFFFF;

            

            if(label_num >= 251 && label_num < 65535)
            {   
                if(!labels_map.count(label_num)) labels_map[label_num] = 1;
                else labels_map[label_num] += 1;
                if(pred_num >= 251 && pred_num < 65535)
                // if(pred_num < 40)
                {
                    tp += 1;
                    if(!predicts_map.count(label_num)) predicts_map[label_num] = 1;
                    else predicts_map[label_num] += 1;
                }
                else if (pred_num == 9)
                {
                    fn += 1; //
                }
                count += 1;
            }
            else if (label_num < 251 && label_num < 65535)
            {
                if(pred_num >= 251 && pred_num < 65535)
                {
                    fp += 1;
                }
                else if (pred_num == 9)
                {
                    tn += 1;
                }
            }
            else if (label_num >= 65535)
            {
                if(pred_num >= 251 && pred_num < 65535)
                {
                    op += 1;
            
                }
            }

        }
        
        iou = ((float)tp)/(float)(tp+fn+fp);
        total_tp += tp;
        total_fn += fn;
        total_fp += fp;
        total_tn += tn;
        total_op += op;
        label_input.close();
        pred_input.close();
        cout<<"tp: "<<tp<<"  fp: "<<fp<<" fn: "<<fn<< " op: " <<op << " tn: " << tn << " count: "<<count<<" iou: "<<iou<<endl;
        for(auto it = labels_map.begin(); it != labels_map.end(); it++)
        {   
            int class_num = it->first/1000;
            objects_numbers_kitti[class_num] ++;
            std::cout << "id: " << it->first << " labels: " << it->second << " predict: " << predicts_map[it->first] << " recall: " << (float)predicts_map[it->first]/(float)it->second << std::endl; 
            average_recalls_kitti[class_num]  = average_recalls_kitti[class_num] * (objects_numbers_kitti[class_num] - 1)/objects_numbers_kitti[class_num] + (float)predicts_map[it->first]/(float)it->second / objects_numbers_kitti[class_num];
            average_recalls2_kitti[class_num] = average_recalls2_kitti[class_num] * (objects_numbers_kitti[class_num] - 1)/objects_numbers_kitti[class_num] + (float)predicts_map[it->first]/(float)it->second * (float)predicts_map[it->first]/(float)it->second/ objects_numbers_kitti[class_num];
            
        }
        for(int i = 0; i < objects_numbers_kitti.size(); i++)
        {
            cout << "average_recall of " << objects_types_map_kitti[i] << " is: " << average_recalls_kitti[i] << " objects number: " << objects_numbers_kitti[i]<< " variance: " << average_recalls2_kitti[i] - average_recalls_kitti[i] * average_recalls_kitti[i] << endl;
        }
        cout << "average_suppress: " << (float)(total_fn + total_tn)/(float)(total_tp + total_fp + total_tn + total_fn) << endl;
        cout<<"total_tp: "<<total_tp<<"  total_fp: "<<total_fp<<" total_fn: " <<total_fn << " total_op: " <<total_op << " total_tn: " << total_tn  << " total_points: " << total_tp+ total_fp + total_fn + total_tn<<endl;
        cout << "Average iou: " << ((float)total_tp)/(float)(total_tp+total_fn+total_fp) << endl;

        
        if((frames + 1) == label_file_num )
        {
            class_nums.clear();
            class_recalls.clear();
            class_var.clear();
            for(int i = 0; i < objects_numbers_kitti.size(); i++)
            {
                class_nums.push_back(objects_numbers_kitti[i]);
                class_recalls.push_back(average_recalls_kitti[i]);
                class_var.push_back(average_recalls2_kitti[i] - average_recalls_kitti[i] * average_recalls_kitti[i]);
            }
        }
    }
    
    recall_rec << total_tp << " " << total_fp << " "  << total_fn << " " << total_tn << " ";
    for(int i = 0; i < class_recalls.size(); i++)
    {
        recall_rec << class_recalls[i] << " " << class_nums[i] << " " << class_var[i] << " ";
    }
    recall_rec << endl;
}

void SemanticCombine()
{
    int semantic_file_num = 0;
    if(semantic_folder != "")
    {
        DIR* semantic_dir;	
        semantic_dir = opendir(semantic_folder.c_str());
        struct dirent* semantic_ptr;
        while((semantic_ptr = readdir(semantic_dir)) != NULL)
        {
            if(semantic_ptr->d_name[0] == '.') {continue;}
            semantic_file_num++;
        }
        closedir(semantic_dir);
    } 
    else
    {
        cout<<"less semantic "<<semantic_folder<<endl;
        return;
    }
    int pred_file_num = 0;
    if(pred_folder != "")
    {
        DIR* pred_dir;	
        pred_dir = opendir(pred_folder.c_str());
        struct dirent* pred_ptr;
        while((pred_ptr = readdir(pred_dir)) != NULL)
        {
            if(pred_ptr->d_name[0] == '.') {continue;}
            pred_file_num++;
        }
        closedir(pred_dir);
    } 
    else
    {
        cout<<"less pred "<<pred_folder<<endl;
        return;
    }

    
    if(semantic_file_num != pred_file_num)// || pred_num != pred_origin_num
    {
        cout<<"file num error "<<label_folder<<endl;
    }

    for(int frames = 0; frames < semantic_file_num; frames++)
    {
        stringstream ss;
        ss << setw(6) << setfill('0') << frames;
        string pred_file = pred_folder;
        pred_file += ss.str(); 
        pred_file.append(".label");

        string semantic_file = semantic_folder;
        semantic_file += ss.str(); 
        semantic_file.append(".label");
        
        string out_file = out_folder;
        out_file += ss.str(); 
        out_file.append(".label");
        ofstream out;
        out.open(out_file, ios::out  | ios::binary);


        std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary);
        if(!pred_input.good())
        {
            std::cerr << "Could not read prediction file: " << pred_file << std::endl;
            exit(EXIT_FAILURE);
        }

        std::fstream semantic_input(semantic_file.c_str(), std::ios::in | std::ios::binary);
        if(!semantic_input.good())
        {
            std::cerr << "Could not read semantic file: " << semantic_file << std::endl;
            exit(EXIT_FAILURE);
        }

        while(pred_input.good() && !pred_input.eof())
        {
            int pred_num = -1, semantic_num = -1;
            pred_input.read((char *) &pred_num, sizeof(int));
            pred_num = pred_num & 0xFFFF;

            semantic_input.read((char *) &semantic_num, sizeof(int));
            semantic_num = semantic_num & 0xFFFF;

            if(pred_num == 251)
            {
                if (semantic_num < 40)
                {
                    int tmp = 251;
                    out.write((char*)&tmp, sizeof(int));
                }
                else
                {
                    int tmp = 9;
                    out.write((char*)&tmp, sizeof(int));
                }
            }
            else
            {
                int tmp = 9;
                out.write((char*)&tmp, sizeof(int));
            }
        }
        out.close();

    }
    
}

void NuscenesCalRecall(std::vector<float> &recalls_vehicle, std::vector<float> &recalls_pedestrian, std::vector<float> &recalls_cyclist)
{
    int label_file_num = 0;
    if(label_folder != "")
    {
        DIR* label_dir;	
        label_dir = opendir(label_folder.c_str());
        struct dirent* label_ptr;
        while((label_ptr = readdir(label_dir)) != NULL)
        {
            if(label_ptr->d_name[0] == '.') {continue;}
            label_file_num++;
        }
        closedir(label_dir);
    } 
    else
    {
        cout<<"less label "<<label_folder<<endl;
        return;
    }
    int pred_file_num = 0;
    if(pred_folder != "")
    {
        DIR* pred_dir;	
        pred_dir = opendir(pred_folder.c_str());
        struct dirent* pred_ptr;
        while((pred_ptr = readdir(pred_dir)) != NULL)
        {
            if(pred_ptr->d_name[0] == '.') {continue;}
            pred_file_num++;
        }
        closedir(pred_dir);
    } 
    else
    {
        cout<<"less pred "<<pred_folder<<endl;
        return;
    }

    
    if(label_file_num != pred_file_num)// || pred_num != pred_origin_num
    {
        cout<<"file num error "<<label_folder<<endl;
    }

    vector<int> class_nums;
    vector<float> class_recalls;
    vector<float> class_var;
    int total_tp = 0, total_fn = 0, total_fp = 0, total_op = 0, total_tn = 0.0;
    std::vector<int> objects_numbers_nuscenes(11, 0), objects_numbers_nuscenes_origin(11, 0);
    std::vector<float> average_recalls_nuscenes(11, 0.0), average_recalls_nuscenes_origin(11, 0.0);
    std::vector<float> average_recalls2_nuscenes(11, 0.0);
    for(int frames = 0; frames < label_file_num; frames ++)
    {   
        cout << "frame: " << frames << endl;
        stringstream ss;
        ss << setw(6) << setfill('0') << frames ;
        
        string label_file = label_folder;
        label_file += ss.str(); 
        label_file.append(".label");
        
        string pred_file = pred_folder;
        pred_file += ss.str(); 
        pred_file.append(".label");

        

        std::fstream label_input(label_file.c_str(), std::ios::in | std::ios::binary);
        if(!label_input.good())
        {
            std::cerr << "Could not read label file: " << label_file << std::endl;
            exit(EXIT_FAILURE);
        }
        label_input.seekg(0, std::ios::beg);

        std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary);
        if(!pred_input.good())
        {
            std::cerr << "Could not read prediction file: " << pred_file << std::endl;
            exit(EXIT_FAILURE);
        }

        
        std::fstream test(label_file.c_str(), std::ios::in | std::ios::binary);
        if(!test.good())
        {
            std::cerr << "Could not read label file: " << label_file << std::endl;
            exit(EXIT_FAILURE);
        }
        test.seekg(0, std::ios::beg);
        int class_id;
        test.read((char *) &class_id, sizeof(int));
        test.read((char *) &class_id, sizeof(int));

        

        int tp = 0, fn = 0, fp = 0, op = 0, tn = 0, count = 0;
        float iou = 0.0f;
        float cur_self_vel;

        std::unordered_map<int, int> labels_map;
        std::unordered_map<int, int> predicts_map;

        if(test.eof())
        {
            cout<<" empty"<<endl;
        }
        else
        {
            while(pred_input.good() && !pred_input.eof())
            {
                

                int pred_num = -1;
                pred_input.read((char *) &pred_num, sizeof(int));
                pred_num = pred_num & 0xFFFF;

                int  label_num = -1, id = -1;
                // pred_input >> pred_num;
                label_input.read((char *) &label_num, sizeof(int));
                label_num = label_num & 0xFFFF;

                

                if(label_num >= 1000 && label_num < 65535)
                {
                    // if(label_num == 251) cout<<" --------251: "<<point.x<<" , "<<point.y<<" , "<<point.z<<" "<<pred_num<<endl;
                    if(!labels_map.count(label_num)) labels_map[label_num] = 1;
                    else labels_map[label_num] += 1;
                    if(pred_num >= 251 && pred_num < 65535)
                    {
                        if(!predicts_map.count(label_num)) predicts_map[label_num] = 1;
                        else predicts_map[label_num] += 1;
                        tp += 1;
                    }
                    else
                    {
                        fn += 1; //
                        // cout<<" , "<<label_num<<" "<<point.x<<" "<<point.y<<" "<<point.z;
                    }
                    count += 1;
                }
                else if(label_num < 251)
                {
                    if(pred_num >= 251 && pred_num < 65535)
                    {
                        fp += 1;
                    }
                    else
                    {
                        tn += 1;
                    }
                }
                else if (label_num >= 65535)
                {
                    if(pred_num >= 251 && pred_num < 65535)
                    // if(pred_num < 40)
                    {
                        op += 1;
                    }
                }
                else if (label_num >= 251 && label_num < 1000)
                {
                    tn += 1;
                }

            }
        }
        
        if(tp+fn+fp > 10e-5)iou = ((float)tp)/(float)(tp+fn+fp);
        total_tp += tp;
        total_fn += fn;
        total_fp += fp;
        total_tn += tn;
        total_op += op;
        label_input.close();
        // vel_input.close();
        pred_input.close();
        cout<<"tp: "<<tp<<"  fp: "<<fp<<" fn: "<<fn<<" count: "<<count<<" iou: "<<iou<<endl;
        for(auto it = labels_map.begin(); it != labels_map.end(); it++)
        {   
            int class_num = it->first/1000;
            if(objects_class_map_nuscenes.count(class_num) == 0)
            {
                cout<<"we do not find "<< class_num << "  " << it->first <<endl;
                continue;
            } 
            cout<<"remap class "<< class_num;  
            class_num = objects_class_map_nuscenes[class_num];\
            cout<<" to "<< class_num << endl;
            objects_numbers_nuscenes[class_num] ++;
            std::cout << "id: " << it->first << " labels: " << it->second << " predict: " << predicts_map[it->first] << " recall: " << (float)predicts_map[it->first]/(float)it->second << std::endl; 
            average_recalls_nuscenes[class_num]  = average_recalls_nuscenes[class_num] * (objects_numbers_nuscenes[class_num] - 1)/objects_numbers_nuscenes[class_num] + (float)predicts_map[it->first]/(float)it->second / objects_numbers_nuscenes[class_num];
            average_recalls2_nuscenes[class_num] = average_recalls2_nuscenes[class_num] * (objects_numbers_nuscenes[class_num] - 1)/objects_numbers_nuscenes[class_num] + (float)predicts_map[it->first]/(float)it->second * (float)predicts_map[it->first]/(float)it->second/ objects_numbers_nuscenes[class_num];        
            if(class_num == 4 || class_num == 5 || class_num == 7 || class_num == 9) recalls_vehicle.push_back((float)predicts_map[it->first]/(float)it->second);
            if(class_num == 1) recalls_pedestrian.push_back((float)predicts_map[it->first]/(float)it->second);
            if(class_num == 3) recalls_cyclist.push_back((float)predicts_map[it->first]/(float)it->second);
        }
        for(int i = 0; i < objects_numbers_nuscenes.size(); i++)
        {
            cout << "average_recall of " << objects_types_map_nuscenes[i] << " is: " << average_recalls_nuscenes[i] << " objects number: " << objects_numbers_nuscenes[i] << " variance: " << average_recalls2_nuscenes[i] - average_recalls_nuscenes[i] * average_recalls_nuscenes[i]<< endl;
        }
        cout << "average_suppress: " << (float)(total_fn + total_tn)/(float)(total_tp + total_fp + total_tn + total_fn) << endl;
        cout<<"total_tp: "<<total_tp<<"  total_fp: "<<total_fp<<" total_fn: " <<total_fn << " total_op: " <<total_op << " total_tn: " << total_tn  << " total_points: " << total_tp+ total_fp + total_fn + total_tn<<endl;
        cout << "Average iou: " << ((float)total_tp)/(float)(total_tp+total_fn+total_fp) << endl;
        if((frames + 1) == label_file_num )
        {
            class_nums.clear();
            class_recalls.clear();
            class_var.clear();
            for(int i = 0; i < objects_numbers_nuscenes.size(); i++)
            {
                class_nums.push_back(objects_numbers_nuscenes[i]);
                class_recalls.push_back(average_recalls_nuscenes[i]);
                class_var.push_back(average_recalls2_nuscenes[i] - average_recalls_nuscenes[i] * average_recalls_nuscenes[i]);
            }
        }
    }
    recall_rec << total_tp << " " << total_fp << " "  << total_fn << " " << total_tn << " ";
    for(int i = 0; i < class_recalls.size(); i++)
    {
        recall_rec << class_recalls[i] << " " << class_nums[i] << " " << class_var[i] << " ";
    }
    recall_rec << endl;
    // pred_input.seekg(0, std::ios::beg);
}

void WaymoCalRecall(std::vector<float> &recalls_vehicle, std::vector<float> &recalls_pedestrian, std::vector<float> &recalls_cyclist)
{
    int label_file_num = 0;
    if(label_folder != "")
    {
        DIR* label_dir;	
        label_dir = opendir(label_folder.c_str());
        struct dirent* label_ptr;
        while((label_ptr = readdir(label_dir)) != NULL)
        {
            if(label_ptr->d_name[0] == '.') {continue;}
            label_file_num++;
        }
        closedir(label_dir);
    } 
    else
    {
        cout<<"less label "<<label_folder<<endl;
        return;
    }
    int pred_file_num = 0;
    if(pred_folder != "")
    {
        DIR* pred_dir;	
        pred_dir = opendir(pred_folder.c_str());
        struct dirent* pred_ptr;
        while((pred_ptr = readdir(pred_dir)) != NULL)
        {
            if(pred_ptr->d_name[0] == '.') {continue;}
            pred_file_num++;
        }
        closedir(pred_dir);
    } 
    else
    {
        cout<<"less pred "<<pred_folder<<endl;
        return;
    }

    
    if(label_file_num != pred_file_num)// || pred_num != pred_origin_num
    {
        cout<<"file num error "<<label_folder<<endl;
    }

    vector<int> class_nums;
    vector<float> class_recalls;
    vector<float> class_var;
    int total_tp = 0, total_fn = 0, total_fp = 0, total_op = 0, total_tn = 0.0;
    std::vector<int> objects_numbers_waymo(3, 0);
    std::vector<float> average_recalls_waymo(3, 0.0);
    std::vector<float> average_recalls2_waymo(3, 0.0);
    for(int frames = 0; frames < label_file_num; frames ++)
    {   
        cout << "frame: " << frames << endl;
        stringstream ss;
        ss << setw(6) << setfill('0') << frames ;
        
        string label_file = label_folder;
        label_file += ss.str(); 
        label_file.append(".label");
        
        string pred_file = pred_folder;
        pred_file += ss.str(); 
        pred_file.append(".label");

        

        std::fstream label_input(label_file.c_str(), std::ios::in | std::ios::binary);
        if(!label_input.good())
        {
            std::cerr << "Could not read label file: " << label_file << std::endl;
            exit(EXIT_FAILURE);
        }
        label_input.seekg(0, std::ios::beg);

        std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary);
        if(!pred_input.good())
        {
            std::cerr << "Could not read prediction file: " << pred_file << std::endl;
            exit(EXIT_FAILURE);
        }

        

        int tp = 0, fn = 0, fp = 0, op = 0, tn = 0, count = 0;
        float iou = 0.0f;

        std::unordered_map<int, int> labels_map;
        std::unordered_map<int, int> predicts_map;

        

        while(pred_input.good() && !pred_input.eof())
        {
            

            int pred_num = -1;
            pred_input.read((char *) &pred_num, sizeof(int));
            pred_num = pred_num & 0xFFFF;

            int  label_num = -1, id = -1;
            // pred_input >> pred_num;
            label_input.read((char *) &label_num, sizeof(int));
            label_num = label_num & 0xFFFF;

            

            if(label_num >= 251 && label_num < 65535)
            {   
                if(!labels_map.count(label_num)) labels_map[label_num] = 1;
                else labels_map[label_num] += 1;
                if(pred_num >= 251 && pred_num < 65535)
                {
                    tp += 1;
                    if(!predicts_map.count(label_num)) predicts_map[label_num] = 1;
                    else predicts_map[label_num] += 1;
                    
                }
                else
                {
                    fn += 1; //
                }
                count += 1;
            }
            else if (label_num < 251 && label_num < 65535)
            {
                if(pred_num >= 251 && pred_num < 65535)
                {
                    fp += 1;               
                }
                else
                {
                    tn += 1;
                }
            }
            else if (label_num >= 65535)
            {
                if(pred_num >= 251 && pred_num < 65535)
                {
                    op += 1;
                }
            }

        }
        
        iou = ((float)tp)/(float)(tp+fn+fp);
        total_tp += tp;
        total_fn += fn;
        total_fp += fp;
        total_tn += tn;
        total_op += op;
        label_input.close();
        pred_input.close();
        cout<<"tp: "<<tp<<"  fp: "<<fp<<" fn: "<<fn<< " op: " <<op << " tn: " << tn << " count: "<<count<<" iou: "<<iou<<endl;
        for(auto it = labels_map.begin(); it != labels_map.end(); it++)
        {   
            int class_num = it->first/1000;
            objects_numbers_waymo[class_num] ++;
            std::cout << "id: " << it->first << " labels: " << it->second << " predict: " << predicts_map[it->first] << " recall: " << (float)predicts_map[it->first]/(float)it->second << std::endl; 
            average_recalls_waymo[class_num]  = average_recalls_waymo[class_num] * (objects_numbers_waymo[class_num] - 1)/objects_numbers_waymo[class_num] + (float)predicts_map[it->first]/(float)it->second / objects_numbers_waymo[class_num];
            average_recalls2_waymo[class_num] = average_recalls2_waymo[class_num] * (objects_numbers_waymo[class_num] - 1)/objects_numbers_waymo[class_num] + (float)predicts_map[it->first]/(float)it->second * (float)predicts_map[it->first]/(float)it->second/ objects_numbers_waymo[class_num];

        }
        for(int i = 0; i < objects_numbers_waymo.size(); i++)
        {
            cout << "average_recall of " << objects_types_map_waymo[i] << " is: " << average_recalls_waymo[i] << " objects number: " << objects_numbers_waymo[i] << " variance: " << average_recalls2_waymo[i] - average_recalls_waymo[i] * average_recalls_waymo[i] << endl;
        }
        cout << "average_suppress: " << (float)(total_fn + total_tn)/(float)(total_tp + total_fp + total_tn + total_fn) << endl;
        cout<<"total_tp: "<<total_tp<<"  total_fp: "<<total_fp<<" total_fn: " <<total_fn << " total_op: " <<total_op << " total_tn: " << total_tn  << " total_points: " << total_tp+ total_fp + total_fn + total_tn<<endl;
        cout << "Average iou: " << ((float)total_tp)/(float)(total_tp+total_fn+total_fp) << endl;
        
        if((frames + 1) == label_file_num )
        {
            class_nums.clear();
            class_recalls.clear();
            class_var.clear();
            for(int i = 0; i < objects_numbers_waymo.size(); i++)
            {
                class_nums.push_back(objects_numbers_waymo[i]);
                class_recalls.push_back(average_recalls_waymo[i]);
                class_var.push_back(average_recalls2_waymo[i] - average_recalls_waymo[i] * average_recalls_waymo[i]);
            }
        }
    }
    
    recall_rec << total_tp << " " << total_fp << " "  << total_fn << " " << total_tn << " ";
    for(int i = 0; i < class_recalls.size(); i++)
    {
        recall_rec << class_recalls[i] << " " << class_nums[i] << " " << class_var[i] << " ";
    }
    recall_rec << endl;
    // pred_input.seekg(0, std::ios::beg);
}


int main(int argc, char **argv)
{
    // 初始化log
    google::InitGoogleLogging(argv[0]);
    std::string log_dir = std::string(ROOT_DIR) + "log";
    FLAGS_log_dir = log_dir;
    FLAGS_alsologtostderr = true;

    // 初始化ROS节点
    ros::init(argc, argv, "check_dynamic");
    ros::NodeHandle nh;
    Init(); // 调用初始化函数，设置物体类型映射

    // 定义和获取ROS参数
    int dataset = -1;                   // 数据集类型
    int start_param = 0, end_param = 0; // 起始参数号，终止参数号
    int start_se = 0, end_se = 0;       // 起始和终止序列号
    bool is_origin = false;             // 是否是点输出模式结果
    nh.param<int>("dyn_obj/dataset", dataset, -1);
    nh.param<bool>("dyn_obj/is_origin", is_origin, false);
    nh.param<string>("dyn_obj/dataset_folder", dataset_folder, "/");
    nh.param<int>("dyn_obj/start_param", start_param, -1);
    nh.param<int>("dyn_obj/end_param", end_param, 0);
    nh.param<int>("dyn_obj/start_se", start_se, -1);
    nh.param<int>("dyn_obj/end_se", end_se, 0);

    // 根据指定的数据集，进行相应的处理
    if (dataset == 0) // 对于KITTI数据集
    {
        // 设置和创建存储召回率计算结果的文件夹
        recall_folder = dataset_folder + "recall/";
        recall_origin_folder = dataset_folder + "recall_origin/";
        string command;
        command = "mkdir -p " + recall_folder;
        system(command.c_str());
        command = "mkdir -p " + recall_origin_folder;
        system(command.c_str());

        // 遍历指定的参数和序列范围，进行召回率的计算
        for (int i = start_param; i < end_param; i++)
        {
            // 根据是否使用原始预测设置相应的文件路径
            if (is_origin)
            {
                recall_file = recall_origin_folder + "kitti" + to_string(i) + "_origin" + ".txt";
            }
            else
            {
                recall_file = recall_folder + "kitti" + to_string(i) + ".txt";
            }
            recall_rec.open(recall_file, std::ios::out);
            std::vector<float> recalls_vehicle;
            std::vector<float> recalls_pedestrian;
            std::vector<float> recalls_cyclist;

            // 对于每个序列，读取标签和预测结果，计算召回率
            for (int j = start_se; j < end_se; j++)
            {
                // 设置标签和预测文件夹的路径
                stringstream ss;
                ss << setw(4) << setfill('0') << j;
                if (is_origin)
                {
                    pred_folder = dataset_folder + "sequences/" + ss.str() + "/predictions" + to_string(i) + "_origin/";
                }
                else
                {
                    pred_folder = dataset_folder + "sequences/" + ss.str() + "/predictions" + to_string(i) + "/";
                }

                label_folder = dataset_folder + "sequences/" + ss.str() + "/labels/";
                cout << "pred: " << pred_folder << endl;
                cout << label_folder << endl;
                // 调用KITTI数据集的召回率计算函数
                KittiCalRecall(recalls_vehicle, recalls_pedestrian, recalls_cyclist);
            }
            // 计算四分位数索引，用于后续提取四分位数值
            int q1_vehicle = round(recalls_vehicle.size() / 8);
            int q2_vehicle = round(recalls_vehicle.size() / 2);
            int q3_vehicle = round(recalls_vehicle.size() * 7 / 8);

            int q1_pedestrian = round(recalls_pedestrian.size() / 8);
            int q2_pedestrian = round(recalls_pedestrian.size() / 2);
            int q3_pedestrian = round(recalls_pedestrian.size() * 7 / 8);

            int q1_cyclist = round(recalls_cyclist.size() / 8);
            int q2_cyclist = round(recalls_cyclist.size() / 2);
            int q3_cyclist = round(recalls_cyclist.size() * 7 / 8);
            // 对召回率列表进行排序，以便提取四分位数值
            std::sort(recalls_vehicle.begin(), recalls_vehicle.end());
            std::sort(recalls_pedestrian.begin(), recalls_pedestrian.end());
            std::sort(recalls_cyclist.begin(), recalls_cyclist.end());
            // 写入车辆类别的四分位数召回率，如果列表为空则写入-1作为占位符
            recall_rec << "Vehicle Quartiles Recall: ";
            if (recalls_vehicle.size() > 0)
            {
                recall_rec << recalls_vehicle.at(q1_vehicle) << " " << recalls_vehicle.at(q2_vehicle) << " " << recalls_vehicle.at(q3_vehicle) << endl;
            }
            else
            {
                recall_rec << -1 << " " << -1 << " " << -1 << endl;
            }
            // 写入行人类别的四分位数召回率，同样处理空列表情况
            recall_rec << "Pedestrian Quartiles Recall: ";
            if (recalls_pedestrian.size() > 0)
            {
                recall_rec << recalls_pedestrian.at(q1_pedestrian) << " " << recalls_pedestrian.at(q2_pedestrian) << " " << recalls_pedestrian.at(q3_pedestrian) << endl;
            }
            else
            {
                recall_rec << -1 << " " << -1 << " " << -1 << endl;
            }
            // 写入骑行者类别的四分位数召回率，同样处理空列表情况
            recall_rec << "Cyclist Quartiles Recall: ";
            if (recalls_cyclist.size() > 0)
            {
                recall_rec << recalls_cyclist.at(q1_cyclist) << " " << recalls_cyclist.at(q2_cyclist) << " " << recalls_cyclist.at(q3_cyclist) << endl;
            }
            else
            {
                recall_rec << -1 << " " << -1 << " " << -1 << endl;
            }
            recall_rec.close();
        }
    }
    if(dataset == 1)
    {
        recall_folder = dataset_folder + "recall/";
        recall_origin_folder = dataset_folder + "recall_origin/";
        string command;
        command = "mkdir -p " + recall_folder;
        system(command.c_str());
        command = "mkdir -p " + recall_origin_folder;
        system(command.c_str());
        for(int i = start_param; i < end_param; i++ )
        {
            if(is_origin)
            {
                recall_file = recall_origin_folder + "nuscenes" + to_string(i) + "_origin" + ".txt";
            }
            else
            {
                recall_file = recall_folder + "nuscenes" + to_string(i) + ".txt";
            }
            recall_rec.open(recall_file, std::ios::out);
            std::vector<float> recalls_vehicle;
            std::vector<float> recalls_pedestrian;
            std::vector<float> recalls_cyclist;
            for(int j = start_se; j < end_se; j++)
            {
                stringstream ss;
                ss << setw(4) << setfill('0') << j ;
                if(is_origin)
                {
                    pred_folder =  dataset_folder + "sequences/" + ss.str() + "/predictions" + to_string(i) + "_origin/";
                }
                else
                {
                    pred_folder = dataset_folder + "sequences/" + ss.str() + "/predictions" + to_string(i) + "/";
                }
                label_folder = dataset_folder + "sequences/" + ss.str() + "/labels/";
                cout<<"pred: "<<pred_folder<<endl;
                cout<<label_folder<<endl;
                NuscenesCalRecall(recalls_vehicle, recalls_pedestrian, recalls_cyclist);
            }
            int q1_vehicle = round(recalls_vehicle.size() / 4);
            int q2_vehicle = round(recalls_vehicle.size() / 2);
            int q3_vehicle = round(recalls_vehicle.size() * 3 / 4);

            int q1_pedestrian = round(recalls_pedestrian.size() / 4);
            int q2_pedestrian = round(recalls_pedestrian.size() / 2);
            int q3_pedestrian = round(recalls_pedestrian.size() * 3 / 4);

            int q1_cyclist = round(recalls_cyclist.size() / 4);
            int q2_cyclist = round(recalls_cyclist.size() / 2);
            int q3_cyclist = round(recalls_cyclist.size() * 3 / 4);
            std::sort(recalls_vehicle.begin(), recalls_vehicle.end());
            std::sort(recalls_pedestrian.begin(), recalls_pedestrian.end());
            std::sort(recalls_cyclist.begin(), recalls_cyclist.end());
            if(recalls_vehicle.size() > 0)
            {
                recall_rec << recalls_vehicle.at(q1_vehicle) <<  " " << recalls_vehicle.at(q2_vehicle) << " " << recalls_vehicle.at(q3_vehicle) << endl;
            }
            else
            {
                recall_rec << -1 <<  " " << -1 << " " << -1 << endl;
            }
            if(recalls_pedestrian.size() > 0)
            {
                recall_rec << recalls_pedestrian.at(q1_pedestrian) <<  " " << recalls_pedestrian.at(q2_pedestrian) << " " << recalls_pedestrian.at(q3_pedestrian) << endl;
            }
            else
            {
                recall_rec << -1 <<  " " << -1 << " " << -1 << endl;
            }
            if(recalls_cyclist.size() > 0)
            {
                recall_rec << recalls_cyclist.at(q1_cyclist) <<  " " << recalls_cyclist.at(q2_cyclist) << " " << recalls_cyclist.at(q3_cyclist) << endl;  
            }
            else
            {
                recall_rec << -1 <<  " " << -1 << " " << -1 << endl;
            }
            recall_rec.close();
        }
    }
    if(dataset == 2)
    {
        recall_folder = dataset_folder + "recall/";
        recall_origin_folder = dataset_folder + "recall_origin/";
        string command;
        command = "mkdir -p " + recall_folder;
        system(command.c_str());
        command = "mkdir -p " + recall_origin_folder;
        system(command.c_str());

        for(int i = start_param; i < end_param; i++ )
        {
            if(is_origin)
            {
                recall_file = recall_origin_folder + "waymo" + to_string(i) + "_origin" + ".txt";
            }
            else
            {
                recall_file = recall_folder + "waymo" + to_string(i) + ".txt";
            }
            recall_rec.open(recall_file, std::ios::out);
            std::vector<float> recalls_vehicle;
            std::vector<float> recalls_pedestrian;
            std::vector<float> recalls_cyclist;
            for(int j = start_se; j < end_se; j++)
            {
                stringstream ss;
                ss << setw(4) << setfill('0') << j ;
                if(is_origin)
                {
                    pred_folder = dataset_folder + "sequences/" + ss.str() + "/predictions" + to_string(i) + "_origin/";
                }
                else
                {
                    pred_folder = dataset_folder + "sequences/" + ss.str() + "/predictions" + to_string(i) + "/";
                }
                label_folder = dataset_folder + "sequences/" + ss.str() + "/labels/";
                cout<<"pred: "<<pred_folder<<endl;
                cout<<label_folder<<endl;
                WaymoCalRecall(recalls_vehicle, recalls_pedestrian, recalls_cyclist);
            }
            int q1_vehicle = round(recalls_vehicle.size() / 4);
            int q2_vehicle = round(recalls_vehicle.size() / 2);
            int q3_vehicle = round(recalls_vehicle.size() * 3 / 4);

            int q1_pedestrian = round(recalls_pedestrian.size() / 4);
            int q2_pedestrian = round(recalls_pedestrian.size() / 2);
            int q3_pedestrian = round(recalls_pedestrian.size() * 3 / 4);

            int q1_cyclist = round(recalls_cyclist.size() / 4);
            int q2_cyclist = round(recalls_cyclist.size() / 2);
            int q3_cyclist = round(recalls_cyclist.size() * 3 / 4);
            std::sort(recalls_vehicle.begin(), recalls_vehicle.end());
            std::sort(recalls_pedestrian.begin(), recalls_pedestrian.end());
            std::sort(recalls_cyclist.begin(), recalls_cyclist.end());
            if(recalls_vehicle.size() > 0)
            {
                recall_rec << recalls_vehicle.at(q1_vehicle) <<  " " << recalls_vehicle.at(q2_vehicle) << " " << recalls_vehicle.at(q3_vehicle) << endl;
            }
            else
            {
                recall_rec << -1 <<  " " << -1 << " " << -1 << endl;
            }
            if(recalls_pedestrian.size() > 0)
            {
                recall_rec << recalls_pedestrian.at(q1_pedestrian) <<  " " << recalls_pedestrian.at(q2_pedestrian) << " " << recalls_pedestrian.at(q3_pedestrian) << endl;
            }
            else
            {
                recall_rec << -1 <<  " " << -1 << " " << -1 << endl;
            }
            if(recalls_cyclist.size() > 0)
            {
                recall_rec << recalls_cyclist.at(q1_cyclist) <<  " " << recalls_cyclist.at(q2_cyclist) << " " << recalls_cyclist.at(q3_cyclist) << endl;  
            }
            else
            {
                recall_rec << -1 <<  " " << -1 << " " << -1 << endl;
            }
            recall_rec.close();
        }
    }
    if(dataset == 3)
    {
        recall_folder = dataset_folder + "recall/";
        recall_origin_folder = dataset_folder + "recall_origin/";
        string command;
        command = "mkdir -p " + recall_folder;
        system(command.c_str());
        command = "mkdir -p " + recall_origin_folder;
        system(command.c_str());

        for(int i = start_param; i < end_param; i++ )
        {
            if(is_origin)
            {
                recall_file = recall_origin_folder + "avia" + to_string(i) + + "_origin" + ".txt";
            }
            else
            {
                recall_file = recall_folder + "avia" + to_string(i) + ".txt";
            }
            recall_rec.open(recall_file, std::ios::out);
            std::vector<float> recalls_vehicle;
            std::vector<float> recalls_pedestrian;
            for(int j = start_se; j < end_se; j++)
            {   
                stringstream ss;
                ss << setw(4) << setfill('0') << j ;
                if(is_origin)
                {
                    pred_folder = dataset_folder + "sequences/" + ss.str() + "/predictions" + to_string(i) + "_origin/";
                }
                else
                {
                    pred_folder = dataset_folder + "sequences/" + ss.str() + "/predictions" + to_string(i) + "/";
                }
                label_folder = dataset_folder + "sequences/" + ss.str() + "/labels/";
                cout<<"pred: "<<pred_folder<<endl;
                cout<<label_folder<<endl;
                AviaCalRecall(recalls_vehicle, recalls_pedestrian);
            }
            int q1_vehicle = round(recalls_vehicle.size() / 4);
            int q2_vehicle = round(recalls_vehicle.size() / 2);
            int q3_vehicle = round(recalls_vehicle.size() * 3 / 4);

            int q1_pedestrian = round(recalls_pedestrian.size() / 4);
            int q2_pedestrian = round(recalls_pedestrian.size() / 2);
            int q3_pedestrian = round(recalls_pedestrian.size() * 3 / 4);

            std::sort(recalls_vehicle.begin(), recalls_vehicle.end());
            std::sort(recalls_pedestrian.begin(), recalls_pedestrian.end());
            if(recalls_vehicle.size() > 0)
            {
                recall_rec << recalls_vehicle.at(q1_vehicle) <<  " " << recalls_vehicle.at(q2_vehicle) << " " << recalls_vehicle.at(q3_vehicle) << endl;
            }
            else
            {
                recall_rec << -1 <<  " " << -1 << " " << -1 << endl;
            }
            if(recalls_pedestrian.size() > 0)
            {
                recall_rec << recalls_pedestrian.at(q1_pedestrian) <<  " " << recalls_pedestrian.at(q2_pedestrian) << " " << recalls_pedestrian.at(q3_pedestrian) << endl;
            }
            else
            {
                recall_rec << -1 <<  " " << -1 << " " << -1 << endl;
            }
            recall_rec.close();
        }
    }
    if(dataset == -1)
    {
        string all_pred = dataset_folder + "residual_1/";
        string all_semantic = dataset_folder + "semantic/";
        string all_out = dataset_folder + "residual_1_semantic/";
        vector<string> seq_names;
        if(dataset_folder != "")
        {
            DIR* pred_dir;	
            pred_dir = opendir(all_pred.c_str());
            struct dirent* pred_ptr;
            while((pred_ptr = readdir(pred_dir)) != NULL)
            {
                if(pred_ptr->d_name[0] == '.') {continue;}
                string cur_folder(pred_ptr->d_name);
                seq_names.push_back(cur_folder);
            }
            closedir(pred_dir);
        }
        
        for(int i = 0; i < seq_names.size(); i++ )
        {       
            pred_folder = all_pred + seq_names[i] + "/predictions/";
            semantic_folder = all_semantic + seq_names[i] + "/predictions/";
            out_folder = all_out + seq_names[i] + "/predictions/";
            cout<<pred_folder<<endl;
            cout<<semantic_folder<<endl;
            cout<<out_folder<< " " <<i <<endl;
            SemanticCombine();
        }
    }



    ros::spin();
    return 0;
}