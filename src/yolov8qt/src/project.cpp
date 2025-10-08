#include "../include/yolov8qt/project.h"
float maxX = 12.0, maxY = 5.0, minZ = -1.4;

projector::projector(){
    initParams();
}

void projector::initParams(){
    // 获取可执行文件的当前工作目录
    std::filesystem::path currentPath = std::filesystem::current_path();
    // 拼接相对路径形成绝对路径
    std::filesystem::path configPath = currentPath / "../../../src/yolov8qt/config/initial_params.txt";

    std::ifstream infile(configPath);
    if (!infile.good()) {
        return;
    }
    infile >> i_params.camera_topic;
    infile >> i_params.lidar_topic;
    double_t camtocam[12];
    double_t cameraIn[16];
    double_t RT[16];
    for (int i = 0; i < 16; i++){
        infile >> camtocam[i];
    }
    cv::Mat(4, 4, 6, &camtocam).copyTo(i_params.camtocam_mat);//cameratocamera params

    for (int i = 0; i < 12; i++){
        infile >> cameraIn[i];
    }
    cv::Mat(4, 4, 6, &cameraIn).copyTo(i_params.cameraIn);//cameraIn params

    for (int i = 0; i < 16; i++){
        infile >> RT[i];
    }
    cv::Mat(4, 4, 6, &RT).copyTo(i_params.RT);//lidar to camera params
}

projector::~projector(){}