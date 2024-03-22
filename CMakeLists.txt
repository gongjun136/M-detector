cmake_minimum_required(VERSION 3.0.2)
project(m_detector_noted)

# 设置构建类型为Release，以优化生成的二进制文件
set(CMAKE_BUILD_TYPE "Release")

# 添加编译选项，使用C++17标准
add_compile_options(-std=c++17)
# 设置C++编译器的标志，启用C++17标准，优化级别为O3，并链接TBB库
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -ltbb" ) 

# 添加宏定义，设置项目根目录路径
add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

# 设置C编译器标志，启用异常处理
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
# 设置C++标准为C++17
set(CMAKE_CXX_STANDARD 17)
# 要求编译器支持C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 关闭编译器特定的扩展
set(CMAKE_CXX_EXTENSIONS OFF)
# 设置C++编译器标志，重复了部分设置，并添加了线程库和异常处理
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -ltbb -pthread -std=c++1z -std=c++17 -fexceptions")

# 打印当前CPU架构
message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
# 检查CPU架构是否为x86或AMD64，如果是，则根据处理器数量定义一些宏
# matches是正则表达式，匹配字符串中是否含有x86、X86、amd64、或AMD64
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  # 如果是上述架构之一，引入ProcessorCount模块来获取CPU的核心数
  include(ProcessorCount)
  ProcessorCount(N)
  # 输出当前处理器的核心数
  message("Processor number:  ${N}")
  # 如果处理器核心数大于5，启用多处理并设置处理器数为4
  if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message("core for MP:  3")
  # 如果处理器核心数大于3但不超过5，启用多处理，并根据具体核心数减去2来设置处理器数
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
    message("core for MP:  ${PROC_NUM}")
  # 如果处理器核心数不超过3，设置处理器数为1
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
# 如果不匹配上述任何一种架构，也设置处理器数为1
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

# 尝试找到OpenMP库，如果找到，将其标志添加到编译器选项中
find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

# 必须找到Python库
find_package(PythonLibs REQUIRED)

# 设置Intel Threading Building Blocks（TBB）库的路径
# set(tbb_dir "/usr/local/tbb-2019_U8")
# 寻找ROS必需的组件
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
  message_generation
  eigen_conversions
)

# 寻找Eigen3、PCL、TBB和OpenCV库
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLOG REQUIRED libglog)

# 打印Eigen库的路径
message(Eigen: ${EIGEN3_INCLUDE_DIR})

# 包含头文件路径
include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include
  ${GLOG_INCLUDE_DIRS}
  )

# 生成ROS消息依赖
generate_messages(
 DEPENDENCIES
 geometry_msgs
)

# 定义catkin包，包括依赖项和包含路径
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

# 添加可执行文件目标，并链接库
add_executable(dynfilter_noted src/dynfilter_with_odom.cpp src/dyn_obj_filter.cpp src/dyn_obj_cluster.cpp)
target_link_libraries(dynfilter_noted ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenCV_LIBS} TBB::tbb  ${GLOG_LIBRARIES})
target_include_directories(dynfilter_noted PRIVATE ${PYTHON_INCLUDE_DIRS})

add_executable(display_prediction_noted src/display_prediction.cpp )
target_link_libraries(display_prediction_noted ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenCV_LIBS} TBB::tbb ${GLOG_LIBRARIES})
target_include_directories(display_prediction_noted PRIVATE ${PYTHON_INCLUDE_DIRS})


add_executable(cal_recall_noted src/cal_recall_multi.cpp)
target_link_libraries(cal_recall_noted ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenCV_LIBS} TBB::tbb ${GLOG_LIBRARIES})
target_include_directories(cal_recall_noted PRIVATE ${PYTHON_INCLUDE_DIRS})