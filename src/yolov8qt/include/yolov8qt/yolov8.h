#ifndef YOLOV8_H
#define YOLOV8_H

#include <std_msgs/Float32.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "NvInferPlugin.h"
#include "common.hpp"
#include "project.h"
#include <chrono>
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <yolov8qt/RJ6KData.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <vector>
using namespace det;

// 体素滤波参数
extern float LEAF_SIZE;

// 直通滤波参数
extern float x_min, x_max, y_min, y_max, z_min, z_max;
extern float deviation;

// 欧式聚类参数
extern int MIN_CLUSTER_SIZE;
extern int MAX_CLUSTER_SIZE;
extern float CLUSTER_DISTANCE;

//判断是否带电的阈值
extern float CHARGED_THRES;

//观测点坐标
extern float xc, yc, zc;

struct Detected_Obj {
    jsk_recognition_msgs::BoundingBox bounding_box_;
    pcl::PointXYZ min_point_;
    pcl::PointXYZ max_point_;
    pcl::PointXYZ centroid_;
};

extern bool compare_x(pcl::PointXYZ a, pcl::PointXYZ b);

extern bool compare_z(pcl::PointXYZ a, pcl::PointXYZ b);

class YOLOv8{
public:
    explicit YOLOv8(const std::string& engine_file_path);
    std::vector<float> findNearestPoint(float x0, float y0, float z0, const std::string& filename);
    void setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);
    void make_pipe(bool warmup = true);
    void copy_from_Mat(const cv::Mat& image);
    void copy_from_Mat(const cv::Mat& image, cv::Size& size);
    void letterbox(const cv::Mat& image, cv::Mat& out, cv::Size& size);
    void PointCloudVoxelFilter();
    void PointCloudPassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr bbox_cloud);
    void PointCloudPassThroughFilterKaiguan(pcl::PointCloud<pcl::PointXYZ>::Ptr bbox_cloud);

    void CloudCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr bbox_cloud, const std::string& bbox_name);
    void infer();
    void postprocess(std::vector<Object>& objs,
                     float score_thres = 0.25f,
                     float iou_thres = 0.65f,
                     int topk = 100,
                     int num_labels = 80);
    void draw_objects(const cv::Point& click_pos,
                      const cv::Mat& image,
                      cv::Mat& res,
                      const std::vector<Object>& objs,
                      const std::vector<std::string>& CLASS_NAMES,
                      const std::vector<std::vector<unsigned int>>& COLORS);
    void show_objects(const cv::Mat& image,
                      cv::Mat& show_image,
                      const std::vector<Object>& objs,
                      const std::vector<std::string>& CLASS_NAMES,
                      const std::vector<std::vector<unsigned int>>& COLORS);
    void process(const cv::Point& click_pos,
                const cv::Mat& image,
                cv::Mat& res,
                cv::Size& size,
                std::vector<Object>& objs,
                const std::vector<std::string>& CLASS_NAMES,
                const std::vector<std::vector<unsigned int>>& COLORS,
                float score_thres = 0.25f,
                float iou_thres = 0.65f,
                int topk = 100,
                int num_labels = 80);
    
    void stop_detect();
    ~YOLOv8();

    bool is_running;
    int num_bindings;
    int num_inputs = 0;
    int num_outputs = 0;
    std::vector<Binding> input_bindings;
    std::vector<Binding> output_bindings;
    std::vector<void*> host_ptrs;
    std::vector<void*> device_ptrs;
    PreParam pparam;
    std::vector<Detected_Obj> obj_list;
    std_msgs::Header point_cloud_header_;
    std_msgs::Float32 obj_height;   // 变电设备的高度，用于实时更新COMSOL模型的高度参数
    // RJ6K 实测电场强度数据,有5个数据：(综合场强， x方向分量， y方向分量， z方向分量， 综合场强@50 Hz)
    yolov8qt::RJ6KData rj6k_datas;

    //下面 6 个字符串用于在QT界面显示检测结果
    std::string result_modelname; 
    std::string result_class;  //选中设备类型
    float result_conf;  //置信度
    float result_distance;  //位姿
    float result_electricity;  //综合电场强度 @50 Hz
    std::string result_state;  //是否带电
    int result_FPS;  //检测帧率
private:
    nvinfer1::ICudaEngine* engine = nullptr;
    nvinfer1::IRuntime* runtime = nullptr;
    nvinfer1::IExecutionContext* context = nullptr;
    cudaStream_t stream = nullptr;
    Logger gLogger{nvinfer1::ILogger::Severity::kERROR};
    static projector proj;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    int print_num = 0;
    std::ifstream cloud_infile;
};


#endif  // YOLOV8_H