#include "../include/yolov8qt/QNode.h"
#include <cmath>
#include <Eigen/Geometry>
#include <QDebug>
#include <QDir>
#include <QCoreApplication>
#include <sensor_msgs/point_cloud2_iterator.h>

yolov8qt::RJ6KData rj6k_data;
sensor_msgs::Imu imu_data;

namespace {

std::string BuildGravityAlignedFrameId(const std::string& original_frame_id, const std::string& suffix) {
    if (suffix.empty()) {
        return original_frame_id;
    }

    if (original_frame_id.empty()) {
        return "gravity_aligned";
    }

    if (original_frame_id.size() >= suffix.size() &&
        original_frame_id.compare(original_frame_id.size() - suffix.size(), suffix.size(), suffix) == 0) {
        return original_frame_id;
    }

    return original_frame_id + suffix;
}

}  // namespace

QNode::QNode(int argc, char** argv) : init_argc(argc), init_argv(argv), is_clicked(false){
    qDebug() << "QNode constructor called";
}

QNode::~QNode(){
    if(ros::isStarted()){
        ros::shutdown();
        ros::waitForShutdown();
    }
    emit rosShutdown();
}

bool QNode::init(){
    qDebug() << "QNode init called";
    //ros::init(init_argc, init_argv, "yolov8_qt_node");
    //qDebug() << "ROS Node init success";
    if(!ros::master::check()){
        return false;
    }
    ros::start();
    qDebug() << "ROS Node start success";
    ros::NodeHandle nh;
    qDebug() << "ROS NodeHandle create success";
    /*加载模型*/
    QString modelPath = QCoreApplication::applicationDirPath() + "/../../../src/yolov8qt/weights/3216.engine";
    if (!QFile::exists(modelPath)) {
        qDebug() << "Model file not found:" << modelPath;
        return false;
    }
    yolov8 = std::make_shared<YOLOv8>(modelPath.toStdString());
    qDebug() << "模型 " << modelPath << " 初始化完成";

    cudaSetDevice(0);
    qDebug() << "设置 GPU 0 完成";

    yolov8->make_pipe(true);
    qDebug() << "模型初始化完成";
    image_pub = nh.advertise<sensor_msgs::Image>("/detect/image_raw", 1);
    pub_bounding_boxes = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detect/detected_bounding_boxes", 1);
    pub_obj_height = nh.advertise<std_msgs::Float32>("/detect/obj_height", 1);
    gravity_test_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_gravity_aligned", 1);
    nh.param("gravity_test_accel_points_up", gravity_test_accel_points_up, true);
    nh.param<std::string>("gravity_test_frame_suffix", gravity_test_frame_suffix, "_gravity_aligned");
    // sync.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), image_sub, pointcloud_sub));
    image_sub.subscribe(nh, "/camera/color/image_raw", 1);
    pointcloud_sub.subscribe(nh, "/livox/lidar", 1);
    // rj6k_sub.subscribe(nh, "/rj6k_data", 1);
    rj6k_sub = nh.subscribe<yolov8qt::RJ6KData>("/rj6k_data", 1, &QNode::RJ6KCallback, this);
    imu_sub = nh.subscribe<sensor_msgs::Imu>("/livox/imu", 1, &QNode::IMUCallback, this);
    gravity_test_pointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 1, &QNode::GravityCorrectPointCloudCallback, this);
    sync.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), image_sub, pointcloud_sub));
    // sync.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(50), image_sub, pointcloud_sub, rj6k_sub));
    // sync->setInterMessageLowerBound(ros::Duration(0.099));
    sync->registerCallback(boost::bind(&QNode::DetectCallback, this, _1, _2));
    // sync->registerCallback(boost::bind(&QNode::DetectCallback, this, _1, _2, _3));
    return true;
}

void QNode::run(){
    ros::spin();
}

void QNode::setClickPosition(const QPoint& pos) {
    click_pos = pos;
    is_clicked = true;
    qDebug() << "Clicked position: (" << click_pos.x() << ", " << click_pos.y() << ")";
}

void QNode::continueDetection() {
    is_clicked = false;
    qDebug() << "请重新选择检测框";
}

void QNode::switchModel(const QString& model_path){
    std::lock_guard<std::mutex> lock(model_mutex);
    yolov8.reset();
    yolov8 = std::make_shared<YOLOv8>(model_path.toStdString());
    yolov8->make_pipe(true);
    cudaSetDevice(0);
    qDebug() << "switch to model: " << model_path;
}

void QNode::setVoltage(double voltage_kv) {
    if (yolov8) {
        yolov8->setVoltage(voltage_kv);
        qDebug() << "Voltage set to: " << voltage_kv << " kV";
    }
}

void QNode::DetectCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::PointCloud2ConstPtr& pc){
    //qDebug() << "DetectCallback Thread ID: " << QThread::currentThreadId();
    std::lock_guard<std::mutex> lock(model_mutex);
    auto start_time = std::chrono::system_clock::now();
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e){
        qDebug() << "cv_bridge exception: " << e.what();
        return;
    }
    yolov8->rj6k_datas = rj6k_data;
    yolov8->imu_data = imu_data;
    yolov8->point_cloud_header_ = pc->header;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pc, *input);
    yolov8->setInputCloud(input);
    yolov8->PointCloudVoxelFilter();
    image = cv_ptr->image;
    objects.clear();
    yolov8->obj_list.clear();

    //yolov8->process(click_pos, image, image_result, image_size, objects, CLASS_NAMES, COLORS, score_thres, iou_thres, topk, num_labels);
    
    // 分步骤调用 YOLOv8 的处理函数
    yolov8->copy_from_Mat(image, image_size);
    yolov8->infer();
    yolov8->postprocess(objects, score_thres, iou_thres, topk, num_labels);
    cv::Mat show_image = image.clone();
    yolov8->show_objects(image, show_image, objects, CLASS_NAMES, COLORS);
    
    if(is_clicked == false){
        qimage = QImage(show_image.data, show_image.cols, show_image.rows, show_image.step, QImage::Format_RGB888);
        qimage = qimage.rgbSwapped();

        auto end_time = std::chrono::system_clock::now();
        auto duration_time = (double)std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000;
        yolov8->result_FPS = int(1000/duration_time);
        float rj6k_electric = (rj6k_data.data.size() > 4) ? rj6k_data.data[4] : 0.0;
        emit DataReceived(qimage, yolov8->result_FPS, QStringLiteral("未选择检测框"), 0.0, 0.0, rj6k_electric, QStringLiteral("未选择检测框"));
        return;
    }
    // auto end_time = std::chrono::system_clock::now();
    // auto duration_time = (double)std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000;
    
    // yolov8->result_FPS = int(1000/duration_time);

    cv::Point clicked_pos(click_pos.x(), click_pos.y());
    yolov8->draw_objects(clicked_pos, image, image_result, objects, CLASS_NAMES, COLORS);
    
    qimage = QImage(image_result.data, image_result.cols, image_result.rows, image_result.step, QImage::Format_RGB888);
    qimage = qimage.rgbSwapped();

    sensor_msgs::ImagePtr msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_result).toImageMsg();
    image_pub.publish(msg_to_pub);

    jsk_recognition_msgs::BoundingBoxArray bbox_array;
    for (size_t i = 0; i < yolov8->obj_list.size(); ++i) {
       bbox_array.boxes.push_back(yolov8->obj_list[i].bounding_box_);
    }
    bbox_array.header = pc->header;
    pub_bounding_boxes.publish(bbox_array);

    pub_obj_height.publish(yolov8->obj_height);  //向负责实时更新COMSOL仿真结果的ROS节点发送变电设备的高度信息
    auto end_time = std::chrono::system_clock::now();
    auto duration_time = (double)std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000;
    
    yolov8->result_FPS = int(1000/duration_time);
    emit DataReceived(qimage, yolov8->result_FPS, QString::fromStdString(yolov8->result_class), yolov8->result_conf, yolov8->result_distance, yolov8->result_electricity, QString::fromStdString(yolov8->result_state));
}

void QNode::GravityCorrectPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pc) {
    if (!pc) {
        return;
    }

    sensor_msgs::PointCloud2 corrected_cloud;
    if (!CorrectPointCloudByImu(*pc, corrected_cloud)) {
        ROS_WARN_THROTTLE(2.0, "Waiting for valid IMU acceleration before publishing gravity-aligned point cloud.");
        return;
    }

    gravity_test_pointcloud_pub.publish(corrected_cloud);
}

bool QNode::CorrectPointCloudByImu(const sensor_msgs::PointCloud2& input, sensor_msgs::PointCloud2& output) const {
    const auto& accel = imu_data.linear_acceleration;
    if (!std::isfinite(accel.x) || !std::isfinite(accel.y) || !std::isfinite(accel.z)) {
        return false;
    }

    Eigen::Vector3f accel_vector(accel.x, accel.y, accel.z);
    if (accel_vector.norm() < 1e-3f) {
        return false;
    }

    Eigen::Vector3f up_direction = gravity_test_accel_points_up ? accel_vector.normalized() : -accel_vector.normalized();
    Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(up_direction, Eigen::Vector3f::UnitZ());

    output = input;
    output.header.frame_id = BuildGravityAlignedFrameId(input.header.frame_id, gravity_test_frame_suffix);

    sensor_msgs::PointCloud2Iterator<float> iter_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(output, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
            continue;
        }

        const Eigen::Vector3f rotated_point = rotation * Eigen::Vector3f(*iter_x, *iter_y, *iter_z);
        *iter_x = rotated_point.x();
        *iter_y = rotated_point.y();
        *iter_z = rotated_point.z();
    }

    return true;
}




// void QNode::DetectCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::PointCloud2ConstPtr& pc, const yolov8qt::RJ6KData::ConstPtr& rj6k_msg){
//     qDebug() << "Received synchronized data!";
//     ROS_INFO("Image timestamp: %f", msg->header.stamp.toSec());
//     ROS_INFO("PointCloud timestamp: %f", pc->header.stamp.toSec());
//     ROS_INFO("RJ6KData timestamp: %f", rj6k_msg->header.stamp.toSec());
//     std::lock_guard<std::mutex> lock(model_mutex);
//     auto start_time = std::chrono::system_clock::now();
//     cv_bridge::CvImagePtr cv_ptr;
//     try{
//         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     }
//     catch(cv_bridge::Exception& e){
//         qDebug() << "cv_bridge exception: " << e.what();
//         return;
//     }

//     yolov8->point_cloud_header_ = pc->header;
//     yolov8->rj6k_datas = *rj6k_msg;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromROSMsg(*pc, *input);
//     yolov8->setInputCloud(input);
//     yolov8->PointCloudVoxelFilter();
//     image = cv_ptr->image;
//     objects.clear();
//     yolov8->obj_list.clear();

//     //yolov8->process(click_pos, image, image_result, image_size, objects, CLASS_NAMES, COLORS, score_thres, iou_thres, topk, num_labels);
    
//     // 分步骤调用 YOLOv8 的处理函数
//     yolov8->copy_from_Mat(image, image_size);
//     yolov8->infer();
//     yolov8->postprocess(objects, score_thres, iou_thres, topk, num_labels);
//     cv::Mat show_image = image.clone();
//     yolov8->show_objects(image, show_image, objects, CLASS_NAMES, COLORS);
    
//     if(is_clicked == false){
//         qimage = QImage(show_image.data, show_image.cols, show_image.rows, show_image.step, QImage::Format_RGB888);
//         qimage = qimage.rgbSwapped();

//         auto end_time = std::chrono::system_clock::now();
//         auto duration_time = (double)std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000;
//         yolov8->result_FPS = int(1000/duration_time);
//         emit DataReceived(qimage, yolov8->result_FPS, QStringLiteral("未选择检测框"), 0.0, 0.0, 0.0, QStringLiteral("未选择检测框"));
//         return;
//     }
//     cv::Point clicked_pos(click_pos.x(), click_pos.y());
//     yolov8->draw_objects(clicked_pos, image, image_result, objects, CLASS_NAMES, COLORS);
    
//     qimage = QImage(image_result.data, image_result.cols, image_result.rows, image_result.step, QImage::Format_RGB888);
//     qimage = qimage.rgbSwapped();

//     sensor_msgs::ImagePtr msg_to_pub = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_result).toImageMsg();
//     image_pub.publish(msg_to_pub);

//     jsk_recognition_msgs::BoundingBoxArray bbox_array;
//     for (size_t i = 0; i < yolov8->obj_list.size(); ++i) {
//        bbox_array.boxes.push_back(yolov8->obj_list[i].bounding_box_);
//     }
//     bbox_array.header = pc->header;
//     pub_bounding_boxes.publish(bbox_array);

//     auto end_time = std::chrono::system_clock::now();
//     auto duration_time = (double)std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000;
    
//     yolov8->result_FPS = int(1000/duration_time);
//     emit DataReceived(qimage, yolov8->result_FPS, QString::fromStdString(yolov8->result_class), yolov8->result_conf, yolov8->result_distance, yolov8->result_electricity, QString::fromStdString(yolov8->result_state));
// }


void QNode::RJ6KCallback(const yolov8qt::RJ6KData::ConstPtr& rj6k_msg){
    rj6k_data = *rj6k_msg;
    printf("Received RJ6KData!\n");
    std::cout << "综合电场强度：" << rj6k_data.data[0] << " V/m" << std::endl;
    std::cout << "X轴电场强度：" << rj6k_data.data[1] << " V/m" << std::endl;
    std::cout << "Y轴电场强度：" << rj6k_data.data[2] << " V/m" << std::endl;
    std::cout << "Z轴电场强度：" << rj6k_data.data[3] << " V/m" << std::endl;
    std::cout << "50Hz频点综合电场：" << rj6k_data.data[4] << " V/m" << std::endl;
}

void QNode::IMUCallback(const sensor_msgs::Imu::ConstPtr& imu_msg){
    imu_data = *imu_msg;
}
