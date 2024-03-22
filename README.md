# M-detector

## 1. 介绍

**M-detector**是一个移动事件检测包，它能够在LiDAR数据点到达后立即判断该点是否在移动，从而实现点对点的检测，延迟仅为几微秒。M-detector基于遮挡原理设计，可以在不同环境中使用各种类型的LiDAR传感器。

### **1.1 相关论文**

我们的相关论文已被 Nature Communications 接受：[Moving Event Detection from LiDAR Stream Points](https://www.nature.com/articles/s41467-023-44554-8)。

如果您在项目中使用了我们的代码，请引用我们的论文。

### **1.2 相关视频**

我们的配套视频现在可以在 **YouTube** (点击下面的图片打开) 和 [**Bilibili**](https://www.bilibili.com/video/BV1ke411i7t7/?share_source=copy_web) 上观看。

<div align="center">
<a href="https://www.youtube.com/watch?v=SYaig2eHV5I" target="_blank"><img src="img/cover.bmp" alt="视频" width="60%" /></a>
</div>

### 1.3 开发者

此仓库的代码由以下人员贡献：
[吴花洁 (Huajie Wu)](https://github.com/HuajieWu99)，[李一航 (Yihang Li)](https://github.com/yihangHKU) 和 [徐威 (Wei Xu)](https://github.com/XW-HKU)

## 2. 前提条件

### 2.1 **Ubuntu** 和 **ROS**

Ubuntu ≥ 18.04。

ROS     ≥ Melodic。跟随 [[ROS 安装](http://wiki.ros.org/ROS/Installation)]

### 2.2 **PCL** 和 **Eigen**

PCL      ≥ 1.8

`sudo apt install libpcl-dev`

Eigen    ≥ 3.3.4

`sudo apt install libeigen3-dev`

### 2.3 **livox_ros_driver**

跟随 [livox_ros_driver 安装](https://github.com/Livox-SDK/livox_ros_driver)。

*备注：*

* 由于M-detector首先支持Livox系列LiDAR，所以在运行任何M-detector启动文件之前必须安装并**源码**livox_ros_driver。
* 如何源码？最简单的方法是将行 `source $Livox_ros_driver_dir$/devel/setup.bash` 添加到文件 `~/.bashrc` 的末尾，其中 `$Livox_ros_driver_dir$` 是livox ros driver工作空间的目录（如果您完全跟随了livox官方文档，则应为 `ws_livox` 目录）。

### 2.4 TBB

安装 gcc-9 g++-9

`sudo add-apt-repository ppa:ubuntu-toolchain-r/test`

`sudo apt update`

`sudo apt install gcc-9 g++-9`

`cd /usr/bin`

`sudo rm gcc g++`

`sudo ln -s gcc-9  gcc`

`sudo ln -s g++-9 g++`

跟随 [[TBB 安装](https://solarianprogrammer.com/2019/05/09/cpp-17-stl-parallel-algorithms-gcc-intel-tbb-linux-macos/)] （**注意:** 将 gcc-9.1/g++-9.1 更改为 gcc-9/g++-9）

## 3. 构建

克隆仓库并 catkin_make：

`cd ~/catkin_ws/src`

`git clone git@github.com:hku-mars/M-detector.git`

`catkin_make`

`source devel/setup.bash`
（**注意:** 在CMakeList.txt中更改TBB的路径）


## 4. 关键信息

### 4.1 关键参数

```
dataset: 3    #0 表示kitti, 1 表示nuscenes, 2 表示waymo
buffer_delay: 0.1
buffer_size: 100000
points_num_perframe: 30000
depth_map_dur: 0.2
max_depth_map_num: 5
max_pixel_points: 5
frame_dur: 0.1
hor_resolution_max: 0.005
ver_resolution_max: 0.01
```

针对不同LiDAR的参数提供在 "config" 文件夹中。

关于参数调整的方法，请参考 [[补充信息](https://www.nature.com/articles/s41467-023-44554-8)] 中介绍的第8节。

要保存标签文件，请通过相应的启动文件传递参数。

### 4.2 数据集的文件夹结构

```
├── XXX (数据集名称)
│   ├── bags
│   │   ├── XXX_0000.bag
│   │   ├── ...
│   ├── sequences
│   │   ├── 0000
│   │   │   ├── labels
│   │   │   ├── predictionsx_origin (结果在x参数文件的点输出模式下)
│   │   │   ├── predictionsx (结果在x参数文件的帧输出模式下)
│   │   │   ├── ...
│   │   ├── ...
├── ...
```

数据集可在 [[此链接](https://drive.google.com/drive/folders/1ASNfrjZB7n9Q-nB4Pm2IwvArFWnTcFAj?usp=drive_link)] 下载。

## 5. 直接运行

### 5.1 与里程计和点云（在本地框架中）一起运行

首先，请运行一个里程计节点，例如 [[Fast Lio](https://github.com/hku-mars/FAST_LIO)]（将Fast Lio下载到与M-检测器相同的位置并编译它们）。

然后：

`roslaunch fast_lio mapping_XXX(对应数据集).launch`

`roslaunch m_detector detector_(数据集).launch`

`rosbag play YOURBAG.bag`

### 5.2 为每个点生成标签文件

`roslaunch m_detector detector_XXX.launch out_path:="您的帧输出结果路径" out_origin_path:="您的点输出结果路径"`

注意：按照之前介绍的文件夹结构，`out_path` 应该是 "(数据集文件夹路径)/(数据集名称)/sequences/(序列号)/predictionsx(x是参数文件的编号)/" 的格式，`out_origin_path` 应该是 "(数据集文件夹路径)/(数据集名称)/sequences/(序列号)/predictionsx_origin(x是参数文件的编号)/" 的格式。

### 5.3 计算结果的IoU

`roslaunch m_detector cal_recall.launch dataset:=(0 表示kitti, 1 表示nuscenes, 2 表示waymo, 3 表示avia) dataset_folder:="数据集文件夹路径" start_se:=(计算的第一个序列号) end_se:=(计算的最后一个序列号) start_param:=(计算的第一个参数文件的编号) end_param:=(计算的最后一个参数文件的编号) is_origin:=(true 表示点输出结果, false 表示帧输出结果)`

注意：按照之前介绍的文件夹结构，`dataset_folder` 应该是数据集文件夹的路径。这一步将计算数据集文件夹中列出的所有指定结果的所有IoU，并生成一个名为 "recall" 或 "recall_origin" 包含结果的新文件夹。

## 6. 嵌入在 FAST LIO 中运行

将发布版中提供的嵌入版本下载到一个新的工作空间并编译它们。

`roslaunch fast_lio mapping_(数据集).launch`

`rosbag play YOURBAG.bag`

## 7. Rosbag 下载

论文中使用的bags可以在 [[此链接](https://drive.google.com/drive/folders/1ASNfrjZB7n9Q-nB4Pm2IwvArFWnTcFAj?usp=sharing)] 下载。

## 8. 许可

此包的源代码根据 [**GPLv2**](http://www.gnu.org/licenses/) 许可证发布。我们仅允许其用于**学术用途**。商业用途，请联系张富博士 [fuzhang@hku.hk](mailto:fuzhang@hku.hk)。

如有任何技术问题，请通过电子邮件联系我 [wu2020@connect.hku.hk](mailto:wu2020@connect.hku.hk)。
``` &#8203;``【oaicite:0】``&#8203;
