#pragma once

#include <iostream>
#include <string>
#include <fstream>
#include <filesystem>
#include <opencv2/opencv.hpp>

extern float maxX, maxY, minZ;

class projector{
public:
    struct initial_parameters{
        std::string camera_topic;
        std::string lidar_topic;
        cv::Mat camtocam_mat;
        cv::Mat cameraIn;
        cv::Mat RT;
    }i_params;

    projector();
    void initParams();
    ~projector();
};