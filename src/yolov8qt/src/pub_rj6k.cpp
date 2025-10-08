#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <vector>
#include <cstring>
#include <yolov8qt/RJ6KData.h>
#include <std_msgs/Float64MultiArray.h>

// CRC计算函数（Modbus RTU标准）
uint16_t crc16_modbus(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

// 小端字节序转浮点数
float hexToFloat(const uint8_t *bytes) {
    union {
        uint8_t b[4];
        float f;
    } u;
    memcpy(u.b, bytes, 4); // 直接按小端模式拷贝
    return u.f;
}

// 检查电场单位是否为V/m（4字节全0）
bool isUnitValid(const uint8_t *data) {
    return (data[0] == 0 && data[1] == 0 && data[2] == 0 && data[3] == 0);
}

void readAndPublishData(const ros::TimerEvent& event, serial::Serial* sp, ros::Publisher* pub) {
    // 构造Modbus请求帧（读取24个寄存器）
    uint8_t request[] = {0x54, 0x03, 0x00, 0x00, 0x00, 0x18};
    uint16_t crc = crc16_modbus(request, sizeof(request));
    uint8_t full_request[8];
    memcpy(full_request, request, 6);
    full_request[6] = crc & 0xFF;  // CRC低字节在前
    full_request[7] = crc >> 8;    // CRC高字节在后
    sp->write(full_request, sizeof(full_request));

    std::vector<uint8_t> response(53);
    size_t bytes_read = 0;
    while (bytes_read < 53) {
        size_t read_bytes = sp->read(response.data() + bytes_read, 53 - bytes_read);
        if (read_bytes == 0) {
            ROS_WARN("No more data available.");
            break;
        }
        bytes_read += read_bytes;
    }

    ROS_INFO("bytes_read: %ld", bytes_read);
    printf("response data: ");
    for (int i = 0; i < bytes_read; i++) {
        printf("%02X ", response[i]);
    }
    printf("\n");

    //要发布的自定义消息
    yolov8qt::RJ6KData msg;
    msg.header.stamp = ros::Time::now();  // 设置时间戳

    if (bytes_read < 4) {
        ROS_ERROR("response data is too short!");
        msg.data.assign(5, 0.0);  // 填充5个0.0
        return;
    }
    uint16_t recv_crc = (response[bytes_read-1] << 8) | response[bytes_read-2];
    uint16_t calc_crc = crc16_modbus(response.data(), bytes_read-2);
    if (recv_crc != calc_crc) {
        ROS_ERROR("CRC check failed!");
        ROS_ERROR("received CRC: %04X", recv_crc);
        ROS_ERROR("calculated CRC: %04X", calc_crc);
        msg.data.assign(5, 0.0);  // 填充5个0.0
        return;
    }

    if (bytes_read < 3 + 12*4 + 2) { // 最小有效长度检查
        ROS_ERROR("response data is not complete!");
        msg.data.assign(5, 0.0);  // 填充5个0.0
        return;
    }
    const uint8_t *data = response.data() + 3; // 数据起始位置

    bool unit_valid = isUnitValid(data + 16);

    if (unit_valid) {
        float E_total = hexToFloat(data + 0);   // 综合场强（0x0000）
        float E_x     = hexToFloat(data + 4);   // X轴（0x0002）
        float E_y     = hexToFloat(data + 8);   // Y轴（0x0004）
        float E_z     = hexToFloat(data + 12);  // Z轴（0x0006）
        float E_50Hz  = hexToFloat(data + 40);  // 0x0014

        std::cout << "综合电场强度：" << E_total << " V/m" << std::endl;
        std::cout << "X轴电场强度：" << E_x << " V/m" << std::endl;
        std::cout << "Y轴电场强度：" << E_y << " V/m" << std::endl;
        std::cout << "Z轴电场强度：" << E_z << " V/m" << std::endl;
        std::cout << "50Hz频点综合电场：" << E_50Hz << " V/m" << std::endl;
        
        msg.data.push_back(E_total);
        msg.data.push_back(E_x);
        msg.data.push_back(E_y);
        msg.data.push_back(E_z);
        msg.data.push_back(E_50Hz);
    } else {
        msg.data.assign(5, 0.0f); // 单位无效时输出0
    }
    pub->publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "RJ6K_Sensor");
    ros::NodeHandle n;

    // 创建一个serial类
    serial::Serial sp;
    // 创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);  // 1秒超时
    // 设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    // 设置串口通信的波特率
    sp.setBaudrate(9600);
    // 串口设置timeout
    sp.setTimeout(to);

    try {
        // 打开串口
        sp.open();
    } catch (serial::IOException& e) {
        ROS_ERROR("Unable to open port.");
        return -1;
    }

    if (!sp.isOpen()) {
        ROS_ERROR("Failed to open port.");
        return -1;
    }

    // 创建一个发布者，发布到'sensor_data'话题
    // ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("rj6k_data", 10);
    ros::Publisher pub = n.advertise<yolov8qt::RJ6KData>("rj6k_data", 10);
    // 创建一个定时器，每隔1秒触发一次
    ros::Timer timer = n.createTimer(ros::Duration(0.5), boost::bind(&readAndPublishData, _1, &sp, &pub));

    ros::spin();
    return 0;
}
