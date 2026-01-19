#ifndef QNODE_H
#define QNODE_H

#include <pcl_conversions/pcl_conversions.h>
#include "yolov8.h"
// #include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <QObject>
#include <QImage>
#include <QThread>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <mutex>

class QNode : public QThread{
    Q_OBJECT

public:
    QNode(int argc, char** argv);
    ~QNode();
    bool init();
    void run() override;
    void switchModel(const QString& model_path);
    void setVoltage(double voltage_kv);

signals:
    void rosShutdown();
    void DataReceived(const QImage& image, int fps, const QString& class_name, float conf, float distance, float electric, const QString& state);

public slots:
    void setClickPosition(const QPoint& pos);
    void continueDetection();

private:
    void DetectCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::PointCloud2ConstPtr& pc);
    void RJ6KCallback(const yolov8qt::RJ6KData::ConstPtr& rj6k_msg);
    // void DetectCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::PointCloud2ConstPtr& pc, const yolov8qt::RJ6KData::ConstPtr& rj6k_msg);
private:
    int init_argc;
    char** init_argv;
    // ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub;
    // message_filters::Subscriber<yolov8qt::RJ6KData> rj6k_sub;
    ros::Subscriber rj6k_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2, yolov8qt::RJ6KData> MySyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;
    std::shared_ptr<YOLOv8> yolov8;
    ros::Publisher image_pub;
    ros::Publisher pub_bounding_boxes;
    ros::Publisher pub_obj_height;
    cv::Mat image;
    cv::Mat image_result;
    std::vector<Object> objects;
    int num_labels = 2;
    int topk = 100;
    float score_thres = 0.25f;
    float iou_thres = 0.65f;
    cv::Size image_size = cv::Size(640, 640);
    const std::vector<std::string> CLASS_NAMES = {"muxian", "kaiguan"};
    const std::vector<std::vector<unsigned int>> COLORS = {{0, 114, 189}, {217, 83, 25}};
    QImage qimage;
    bool is_clicked;
    QPoint click_pos;
    std::mutex model_mutex;  //模型切换时的互斥锁
};


#endif // QNODE_H
