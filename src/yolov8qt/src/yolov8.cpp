#include "../include/yolov8qt/yolov8.h"
#include "../include/yolov8qt/electric_field.h"
#include <ros/ros.h>
#include <cmath>  // 添加此头文件以确保sqrt函数可用

float LEAF_SIZE;
float x_min, x_max, y_min, y_max, z_min, z_max;
float deviation;
int MIN_CLUSTER_SIZE, MAX_CLUSTER_SIZE;
float CLUSTER_DISTANCE;
float CHARGED_THRES;
float  xc, yc, zc;

bool compare_x(pcl::PointXYZ a, pcl::PointXYZ b){
    return (a.x < b.x);
}

bool compare_z(pcl::PointXYZ a, pcl::PointXYZ b){
    return (a.z < b.z);
}


projector YOLOv8::proj;
// pcl::PointCloud<pcl::PointXYZ>::Ptr YOLOv8::cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

YOLOv8::YOLOv8(const std::string& engine_file_path) : is_running(true), result_class("未检测到"), result_conf(0.0f), result_distance(0.0f), result_electricity(0.0f), result_state("待定"), result_FPS(0)
{
    // 找到最后一个'/'的位置
    std::size_t lastSlashPos = engine_file_path.find_last_of("/\\");
    // 从最后一个'/'之后开始截取字符串，直到文件路径的末尾
    this->result_modelname = engine_file_path.substr(lastSlashPos + 1);

    obj_height.data = 6.0f;

    std::ifstream file(engine_file_path, std::ios::binary);
    assert(file.good());
    file.seekg(0, std::ios::end);
    auto size = file.tellg();
    file.seekg(0, std::ios::beg);
    char* trtModelStream = new char[size];
    assert(trtModelStream);
    file.read(trtModelStream, size);
    file.close();
    initLibNvInferPlugins(&this->gLogger, "");
    this->runtime = nvinfer1::createInferRuntime(this->gLogger);
    assert(this->runtime != nullptr);

    this->engine = this->runtime->deserializeCudaEngine(trtModelStream, size);
    assert(this->engine != nullptr);
    delete[] trtModelStream;
    this->context = this->engine->createExecutionContext();

    assert(this->context != nullptr);
    cudaStreamCreate(&this->stream);
    this->num_bindings = this->engine->getNbBindings();

    for (int i = 0; i < this->num_bindings; ++i) {
        Binding binding;
        nvinfer1::Dims dims;
        nvinfer1::DataType dtype = this->engine->getBindingDataType(i);
        std::string name = this->engine->getBindingName(i);
        binding.name = name;
        binding.dsize = type_to_size(dtype);

        bool IsInput = engine->bindingIsInput(i);
        if (IsInput) {
            this->num_inputs += 1;
            dims = this->engine->getProfileDimensions(i, 0, nvinfer1::OptProfileSelector::kMAX);
            binding.size = get_size_by_dims(dims);
            binding.dims = dims;
            this->input_bindings.push_back(binding);
            // set max opt shape
            this->context->setBindingDimensions(i, dims);
        }
        else {
            dims = this->context->getBindingDimensions(i);
            binding.size = get_size_by_dims(dims);
            binding.dims = dims;
            this->output_bindings.push_back(binding);
            this->num_outputs += 1;
        }
    }
    m_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    //target = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    //target_cloud_line_XYZ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    //viewer_srcCloud = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("source_pointcloud"));
    //viewer_bboxCloud = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("bbox_pointcloud"));
    //viewer_proj = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("powerline_cloud"));
    //viewer_projected = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("projected_pointcloud"));
    
    // 获取可执行文件的当前工作目录
    std::filesystem::path cur_path = std::filesystem::current_path();
    // 拼接相对路径形成绝对路径
    std::filesystem::path config_path = cur_path / "../../../src/yolov8qt/config/cloud_config.txt";

    cloud_infile.open(config_path);
    if(!cloud_infile.is_open()){
        ROS_ERROR("cloud_config.txt open failed");
        std::cout << config_path << std::endl;
        LEAF_SIZE = 0.2f;
        x_min = 2.0;
        x_max = 6.0;
        y_min = -2.0;
        y_max = 2.0;
        z_min = -4.0;
        z_max = 8.0;
        deviation = 0.5;
        CLUSTER_DISTANCE = 0.1;
        MIN_CLUSTER_SIZE = 20;
        MAX_CLUSTER_SIZE = 1000;
        CHARGED_THRES = 100.0;
        xc = 0.0;
        yc = 0.0;
        zc = 2.0;
    }
    else{
        std::cout << "cloud_config 配置文件加载成功" << std::endl;
        std::string param;
        while(cloud_infile >> param){
            if(param == "leaf_size") cloud_infile >> LEAF_SIZE;
            else if(param == "x_min") cloud_infile >> x_min;
            else if(param == "x_max") cloud_infile >> x_max;
            else if(param == "y_min") cloud_infile >> y_min;
            else if(param == "y_max") cloud_infile >> y_max;
            else if(param == "z_min") cloud_infile >> z_min;
            else if(param == "z_max") cloud_infile >> z_max;
            else if(param == "deviation") cloud_infile >> deviation;
            else if(param == "cluster_distance") cloud_infile >> CLUSTER_DISTANCE;
            else if(param == "min_cluster_size") cloud_infile >> MIN_CLUSTER_SIZE;
            else if(param == "max_cluster_size") cloud_infile >> MAX_CLUSTER_SIZE;
            else if(param == "charged_thres") cloud_infile >> CHARGED_THRES;
            else if(param == "xc") cloud_infile >> xc;
            else if(param == "yc") cloud_infile >> yc;
            else if(param == "zc") cloud_infile >> zc;
            else{
                std::cerr << "Unknown parameter: " << param << std::endl;
                LEAF_SIZE = 0.2f;
                x_min = 2.0;
                x_max = 6.0;
                y_min = -2.0;
                y_max = 2.0;
                z_min = -4.0;
                z_max = 8.0;
                deviation = 0.5;
                CLUSTER_DISTANCE = 0.1;
                MIN_CLUSTER_SIZE = 20;
                MAX_CLUSTER_SIZE = 1000;
                CHARGED_THRES = 100.0;
                xc = 0.0;
                yc = 0.0;
                zc = 2.0;
            }
        }
        cloud_infile.close();
        std::cout << "leaf_size: " << LEAF_SIZE << " x_min: " << x_min << " x_max: " << x_max << " deviation: " << deviation << " cluster_distance: " << CLUSTER_DISTANCE << " min_cluster_size: " << MIN_CLUSTER_SIZE << " max_cluster_size: " << MAX_CLUSTER_SIZE << " charged_thres: " << CHARGED_THRES << std::endl;
    }
}

YOLOv8::~YOLOv8()
{
    //std::cout << "YOLOv8 destructor called" << std::endl;
    this->context->destroy();
    this->engine->destroy();
    this->runtime->destroy();
    cudaStreamDestroy(this->stream);
    for (auto& ptr : this->device_ptrs) {
        CHECK(cudaFree(ptr));
    }

    for (auto& ptr : this->host_ptrs) {
        CHECK(cudaFreeHost(ptr));
    }
}

// 欧氏距离查找到观测点的最近点
// std::vector<float> YOLOv8::findNearestPoint(float x0, float y0, float z0, const std::string& filename)
// {
//     // 打开文件
//     std::ifstream file(filename);
//     if (!file.is_open())
//     {
//         std::cerr << "Failed to open comsol result file: " << filename << std::endl;
//         return {};  // 返回空vector
//     }

//     // 跳过前9行标题
//     std::string line;
//     for (int i = 0; i < 9; ++i)
//     {
//         std::getline(file, line);
//     }

//     // 初始化最近点和最小距离
//     std::vector<float> nearest_data(7, 0.0);  // x, y, z, Ex, Ey, Ez, E
//     double min_distance = std::numeric_limits<float>::max();

//     // 逐行读取数据
//     while (std::getline(file, line))
//     {
//         std::istringstream iss(line);
//         float x, y, z, Ex_real, Ex_imag, Ey_real, Ey_imag, Ez_real, Ez_imag, E;

//         // 解析数据
//         if (!(iss >> x >> y >> z >> Ex_real >> Ex_imag >> Ey_real >> Ey_imag >> Ez_real >> Ez_imag >> E))
//         {
//             std::cerr << "Failed to parse line: " << line << std::endl;
//             continue;
//         }

//         // 计算与目标点的距离
//         float dist = std::sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0) + (z - z0) * (z - z0));

//         // 如果当前点更接近，更新最近点数据
//         if (dist < min_distance)
//         {
//             min_distance = dist;
//             // 除以根号2得到的是有效值，不除是峰值。这里返回的是峰值，和模拟电荷法计算值是对应的
//             nearest_data = {x, y, z, std::sqrt(Ex_real * Ex_real + Ex_imag * Ex_imag), std::sqrt(Ey_real * Ey_real + Ey_imag * Ey_imag), std::sqrt(Ez_real * Ez_real + Ez_imag * Ez_imag), E};
//         }
//     }

//     // 关闭文件
//     file.close();

//     return nearest_data;
// }



std::vector<float> YOLOv8::findNearestPoint(float xc, float yc, float zc, const std::string& filename)
{
    // 打开文件
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open comsol result file: " << filename << std::endl;
        return {};  // 返回空vector
    }

    // 跳过前9行标题
    std::string line;
    for (int i = 0; i < 9; ++i)
    {
        std::getline(file, line);
    }

    // 初始化最近点和最小距离
    std::vector<float> nearest_data(7, 0.0);  // x, y, z, Ex, Ey, Ez, E
    float min_total_distance = std::numeric_limits<float>::max();

    // 逐行读取数据
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        float x, y, z, Ex_real, Ex_imag, Ey_real, Ey_imag, Ez_real, Ez_imag, E;

        // 解析数据
        if (!(iss >> x >> y >> z >> Ex_real >> Ex_imag >> Ey_real >> Ey_imag >> Ez_real >> Ez_imag >> E))
        {
            std::cerr << "Failed to parse line: " << line << std::endl;
            continue;
        }

        // 计算与目标点在x、y、z三个坐标上的绝对距离
        float dx = std::abs(x - xc);
        float dy = std::abs(y - yc);
        float dz = std::abs(z - zc);

        // 计算总距离（可以是三个距离的和，或者其他组合方式）
        float total_distance = dx + dy + dz;

        // 如果当前点的总距离更小，更新最近点数据
        if (total_distance < min_total_distance)
        {
            min_total_distance = total_distance;
            nearest_data = {x, y, z, std::sqrt(Ex_real * Ex_real + Ex_imag * Ex_imag), std::sqrt(Ey_real * Ey_real + Ey_imag * Ey_imag), std::sqrt(Ez_real * Ez_real + Ez_imag * Ez_imag), E};
        }
    }

    // 关闭文件
    file.close();

    return nearest_data;
}


void YOLOv8::setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud){
    this->m_cloud = input_cloud;  
}

void YOLOv8::make_pipe(bool warmup)
{
    for (auto& bindings : this->input_bindings) {
        void* d_ptr;
        CHECK(cudaMallocAsync(&d_ptr, bindings.size * bindings.dsize, this->stream));
        this->device_ptrs.push_back(d_ptr);
    }

    for (auto& bindings : this->output_bindings) {
        void * d_ptr, *h_ptr;
        size_t size = bindings.size * bindings.dsize;
        CHECK(cudaMallocAsync(&d_ptr, size, this->stream));
        CHECK(cudaHostAlloc(&h_ptr, size, 0));
        this->device_ptrs.push_back(d_ptr);
        this->host_ptrs.push_back(h_ptr);
    }

    if (warmup) {
        for (int i = 0; i < 10; i++) {
            for (auto& bindings : this->input_bindings) {
                size_t size  = bindings.size * bindings.dsize;
                void*  h_ptr = malloc(size);
                memset(h_ptr, 0, size);
                CHECK(cudaMemcpyAsync(this->device_ptrs[0], h_ptr, size, cudaMemcpyHostToDevice, this->stream));
                free(h_ptr);
            }
            this->infer();
        }
        printf("model warmup 10 times\n");
    }
}

void YOLOv8::letterbox(const cv::Mat& image, cv::Mat& out, cv::Size& size)
{
    const float inp_h = size.height;
    const float inp_w = size.width;
    float height = image.rows;
    float width  = image.cols;
    float r = std::min(inp_h / height, inp_w / width);
    int padw = std::round(width * r);
    int padh = std::round(height * r);

    cv::Mat tmp;
    if ((int)width != padw || (int)height != padh) {
        cv::resize(image, tmp, cv::Size(padw, padh));
    }
    else {
        tmp = image.clone();
    }

    float dw = inp_w - padw;
    float dh = inp_h - padh;

    dw /= 2.0f;
    dh /= 2.0f;
    int top = int(std::round(dh - 0.1f));
    int bottom = int(std::round(dh + 0.1f));
    int left = int(std::round(dw - 0.1f));
    int right = int(std::round(dw + 0.1f));

    cv::copyMakeBorder(tmp, tmp, top, bottom, left, right, cv::BORDER_CONSTANT, {114, 114, 114});
    cv::dnn::blobFromImage(tmp, out, 1 / 255.f, cv::Size(), cv::Scalar(0, 0, 0), true, false, CV_32F);
    this->pparam.ratio = 1 / r;
    this->pparam.dw = dw;
    this->pparam.dh = dh;
    this->pparam.height = height;
    this->pparam.width = width;
}

void YOLOv8::copy_from_Mat(const cv::Mat& image)
{
    cv::Mat nchw;
    auto& in_binding = this->input_bindings[0];
    auto width = in_binding.dims.d[3];
    auto height = in_binding.dims.d[2];
    cv::Size size{width, height};
    this->letterbox(image, nchw, size);
    this->context->setBindingDimensions(0, nvinfer1::Dims{4, {1, 3, height, width}});

    CHECK(cudaMemcpyAsync(this->device_ptrs[0], nchw.ptr<float>(), nchw.total() * nchw.elemSize(), cudaMemcpyHostToDevice, this->stream));
}

void YOLOv8::copy_from_Mat(const cv::Mat& image, cv::Size& size)
{
    cv::Mat nchw;
    this->letterbox(image, nchw, size);
    this->context->setBindingDimensions(0, nvinfer1::Dims{4, {1, 3, size.height, size.width}});
    CHECK(cudaMemcpyAsync(this->device_ptrs[0], nchw.ptr<float>(), nchw.total() * nchw.elemSize(), cudaMemcpyHostToDevice, this->stream));
}

//点云滤波
void YOLOv8::PointCloudVoxelFilter(){
    //std::cout << "PointCloudVoxelFilter called" << std::endl;
    if(this->m_cloud->empty()){
        ROS_WARN("输入点云 m_cloud 为空");
        return;
    }
    //打印点云的数量
    //ROS_INFO("输入 m_cloud 中点的数量: %d", this->m_cloud->points.size());
    /*体素滤波*/
    //pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(this->m_cloud);
    sor.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
    //sor.filter(*voxel_filtered_cloud);
    sor.filter(*this->cloud);

    if(cloud->empty()){
        ROS_WARN("体素滤波后的 cloud 点云为空");
    }
    // // 可视化过滤后的点云
    // viewer_srcCloud->removeAllPointClouds();
    // viewer_srcCloud->removeAllShapes();
    // viewer_srcCloud->addPointCloud(cloud, "filtered_cloud");
    // viewer_srcCloud->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "filtered_cloud");
    // viewer_srcCloud->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "filtered_cloud"); // 设置体素滤波后的点云颜色为绿色

}

void YOLOv8::PointCloudPassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr bbox_cloud){
    //std::cout << "PointCloudPassThroughFilter called" << std::endl;
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(bbox_cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_min, x_max);
    pass_x.filter(*bbox_cloud);

    //min_element返回一个迭代器，要解引用才能得到值
    pcl::PointXYZ min_x_point = *std::min_element(bbox_cloud->points.begin(), bbox_cloud->points.end(), compare_x);
    // pcl::PointXYZ max_x_point = *std::max_element(bbox_cloud->points.begin(), bbox_cloud->points.end(), compare_x);
    // std::cout << bbox_name + "中的 min_x_point: " << min_x_point << std::endl;
    // std::cout << bbox_name + "中的 min_x_point 到雷达的距离: " << std::sqrt(std::pow(min_x_point.x, 2) + std::pow(min_x_point.y, 2) + std::pow(min_x_point.z, 2)) << std::endl;
    // std::cout << bbox_name + "中的 max_x_point: " << max_x_point << std::endl;
    // std::cout << bbox_name + "中的 max_x_point 到雷达的距离: " << std::sqrt(std::pow(max_x_point.x, 2) + std::pow(max_x_point.y, 2) + std::pow(max_x_point.z, 2)) << std::endl;

    //再次进行X方向的直通滤波
    pass_x.setInputCloud(bbox_cloud);
    pass_x.setFilterLimits(min_x_point.x - deviation, min_x_point.x + deviation);
    pass_x.filter(*bbox_cloud);
}

void YOLOv8::PointCloudPassThroughFilterKaiguan(pcl::PointCloud<pcl::PointXYZ>::Ptr bbox_cloud){
    // //std::cout << "PointCloudPassThroughFilter called" << std::endl;
    // pcl::PassThrough<pcl::PointXYZ> pass_x;
    // pass_x.setInputCloud(bbox_cloud);
    // pass_x.setFilterFieldName("x");
    // pass_x.setFilterLimits(x_min, x_max);
    // pass_x.filter(*bbox_cloud);

    // //min_element返回一个迭代器，要解引用才能得到值
    // pcl::PointXYZ min_x_point = *std::min_element(bbox_cloud->points.begin(), bbox_cloud->points.end(), compare_x);
    // // pcl::PointXYZ max_x_point = *std::max_element(bbox_cloud->points.begin(), bbox_cloud->points.end(), compare_x);
    // // std::cout << bbox_name + "中的 min_x_point: " << min_x_point << std::endl;
    // // std::cout << bbox_name + "中的 min_x_point 到雷达的距离: " << std::sqrt(std::pow(min_x_point.x, 2) + std::pow(min_x_point.y, 2) + std::pow(min_x_point.z, 2)) << std::endl;
    // // std::cout << bbox_name + "中的 max_x_point: " << max_x_point << std::endl;
    // // std::cout << bbox_name + "中的 max_x_point 到雷达的距离: " << std::sqrt(std::pow(max_x_point.x, 2) + std::pow(max_x_point.y, 2) + std::pow(max_x_point.z, 2)) << std::endl;

    // //再次进行X方向的直通滤波
    // pass_x.setInputCloud(bbox_cloud);
    // pass_x.setFilterLimits(min_x_point.x - deviation, min_x_point.x + deviation);
    // pass_x.filter(*bbox_cloud);

    /*Z 轴直通滤波*/
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(bbox_cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_min, z_max);
    pass_z.filter(*bbox_cloud);

    /*Y 轴直通滤波*/
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(bbox_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y_min, y_max);
    pass_y.filter(*bbox_cloud);

    /*X 轴直通滤波*/
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(bbox_cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_min, x_max);
    pass_x.filter(*bbox_cloud);
    auto min_x_point = *std::min_element(bbox_cloud->points.begin(), bbox_cloud->points.end(), compare_x);
    std::cout << "PointCloud preprocess: nearest point x: " << min_x_point.x << std::endl;
    /*再来一次 X 轴直通滤波*/
    pcl::PassThrough<pcl::PointXYZ> pass_x_2;
    pass_x_2.setInputCloud(bbox_cloud);
    pass_x_2.setFilterFieldName("x");
    pass_x_2.setFilterLimits(min_x_point.x - deviation, min_x_point.x + deviation);
    pass_x_2.filter(*bbox_cloud);
}


// 点云聚类
void YOLOv8::CloudCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr bbox_cloud, const std::string& bbox_name){
    std::cout << "CloudCluster called" << std::endl;
    
    // 检查 bbox_cloud 是否为空
    if (bbox_cloud->empty()) {
        ROS_WARN("bbox_cloud is empty, can't create KDTree!");
        this->result_distance = 1000.0;  //1000代表bbox_cloud为空
        this->result_state = "未知";
        return;
    }
    std::cout << "bbox cloud 不为空" << std::endl;
    if (bbox_cloud->points.size() < MIN_CLUSTER_SIZE) {
        ROS_WARN("bbox_cloud 中的点数小于最小聚类点数，无法聚类!");
        std::cout << bbox_name + " 中的点数: " << bbox_cloud->points.size() << std::endl;
        this->result_distance = 2000.0;  //2000代表bbox_cloud中的点数小于最小聚类点数，无法聚类
        this->result_state = "未知";
        return;
    }
    std::cout << "bbox cloud 中的点数大于最小聚类点数" << std::endl;
    //创建KD-Tree用于加速最近邻搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // //转成Y-Z平面点云
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_YZ(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::copyPointCloud(*bbox_cloud, *cloud_YZ);
    // for(size_t i = 0; i < cloud_YZ->points.size(); ++i){
    //     cloud_YZ->points[i].x = 0;
    // }
    // //将YZ平面点云设为kdtree的输入
    // if(!cloud_YZ->points.empty()){
    //     tree->setInputCloud(cloud_YZ);
    // }

    tree->setInputCloud(bbox_cloud);
    std::cout << "setInputCloud success" << std::endl;
    //欧式聚类
    std::vector<pcl::PointIndices> local_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
    euclid.setClusterTolerance(CLUSTER_DISTANCE);
    euclid.setMinClusterSize(MIN_CLUSTER_SIZE);
    euclid.setMaxClusterSize(MAX_CLUSTER_SIZE);
    euclid.setSearchMethod(tree);
    //euclid.setInputCloud(cloud_YZ);
    euclid.setInputCloud(bbox_cloud);
    euclid.extract(local_indices);
    std::cout << "extract success" << std::endl;

    if(local_indices.empty()){
        //std::cerr << bbox_name + " 的bbox中没有聚到类!" << std::endl;
        ROS_WARN("bbox 中没有聚到类!");
        std::cout << "当前参数：" << "leaf_size: " << LEAF_SIZE << " x_min: " << x_min << " x_max: " << x_max << " deviation: " << deviation << " cluster_distance: " << CLUSTER_DISTANCE << " min_cluster_size: " << MIN_CLUSTER_SIZE << " max_cluster_size: " << MAX_CLUSTER_SIZE << " charged_thres: " << CHARGED_THRES << std::endl;
        this->result_distance = 3000.0;  //3000代表没有聚到类
        this->result_state = "未知";
        return;
    }
    std::cout << "local_indices 不为空" << std::endl;

    //std::cout << bbox_name + " 内聚类的数量: " << local_indices.size() << std::endl;
    for(size_t i = 0; i < local_indices.size(); ++i){
        Detected_Obj obj_info;
        float min_x = std::numeric_limits<float>::max();
        float max_x = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_z = -std::numeric_limits<float>::max();
        for(auto pit = local_indices[i].indices.begin(); pit != local_indices[i].indices.end(); ++pit){
            pcl::PointXYZ p;
            p.x = bbox_cloud->points[*pit].x;
            p.y = bbox_cloud->points[*pit].y;
            p.z = bbox_cloud->points[*pit].z;
            obj_info.centroid_.x += p.x;
            obj_info.centroid_.y += p.y;
            obj_info.centroid_.z += p.z;
            if(p.x < min_x) min_x = p.x;
            if(p.x > max_x) max_x = p.x;
            if(p.y < min_y) min_y = p.y;
            if(p.y > max_y) max_y = p.y;
            if(p.z < min_z) min_z = p.z;
            if(p.z > max_z) max_z = p.z;
        }
        obj_info.min_point_.x = min_x;
        obj_info.min_point_.y = min_y;
        obj_info.min_point_.z = min_z;
        obj_info.max_point_.x = max_x;
        obj_info.max_point_.y = max_y;
        obj_info.max_point_.z = max_z;
        if(!local_indices[i].indices.empty()){
            obj_info.centroid_.x /= local_indices[i].indices.size();
            obj_info.centroid_.y /= local_indices[i].indices.size();
            obj_info.centroid_.z /= local_indices[i].indices.size();
        }

        // 输出每个聚类中心点的坐标和到原点的距离
        float distance_to_origin = std::sqrt(std::pow(obj_info.centroid_.x, 2) +
                                             std::pow(obj_info.centroid_.y, 2) +
                                             std::pow(obj_info.centroid_.z, 2));

        this->result_distance = distance_to_origin;
        // std::cout << bbox_name + "中的 Cluster" << i << " 的中心点坐标: ("
        //           << obj_info.centroid_.x << ", "
        //           << obj_info.centroid_.y << ", "
        //           << obj_info.centroid_.z << ")"
        //           << " 中心到雷达的距离: " << distance_to_origin << std::endl;

        //计算点云bounding box
        double length_ = obj_info.max_point_.x - obj_info.min_point_.x;
        double width_ = obj_info.max_point_.y - obj_info.min_point_.y;
        double height_ = obj_info.max_point_.z - obj_info.min_point_.z;

        obj_info.bounding_box_.header = point_cloud_header_;

        obj_info.bounding_box_.pose.position.x = obj_info.min_point_.x + length_ / 2;
        obj_info.bounding_box_.pose.position.y = obj_info.min_point_.y + width_ / 2;
        obj_info.bounding_box_.pose.position.z = obj_info.min_point_.z + height_ / 2;

        obj_info.bounding_box_.dimensions.x = ((length_ < 0) ? -1 * length_ : length_);
        obj_info.bounding_box_.dimensions.y = ((width_ < 0) ? -1 * width_ : width_);
        obj_info.bounding_box_.dimensions.z = ((height_ < 0) ? -1 * height_ : height_);

        // viewer_srcCloud->addCube(obj_info.min_point_.x, obj_info.max_point_.x,
        //                          obj_info.min_point_.y, obj_info.max_point_.y,
        //                          obj_info.min_point_.z, obj_info.max_point_.z,
        //                          1.0, 1.0, 0.0, bbox_name + "cube_" + std::to_string(i));
        // viewer_srcCloud->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, bbox_name + "cube_" + std::to_string(i));

        obj_list.push_back(obj_info);
        
        std::cout << "开始处理电场强度" << std::endl;

        if(bbox_name == "muxian"){
            obj_height.data = obj_info.centroid_.z;  //要发布的变电设备高度话题
             //模拟电荷法计算观测点(xc,yc,zc)的电场强度:(Ex, Ey, Ez, normE)
            std::vector<double> E_simulation = calculateElectricField(xc, yc, zc, "/home/icebear/MyProjects/ROSProjects/yoloqt_ws/src/yolov8qt/config/powerline_config.txt");
            // std::vector<double> E_simulation = calculateElectricField(-obj_info.centroid_.y, -obj_info.centroid_.z, obj_info.centroid_.x, "/home/icebear/MyProjects/ROSProjects/yoloqt_ws/src/yolov8qt/config/powerline_config.txt");
            //从COMSOL仿真结果表中查找观测点的电场强度仿真值
            std::vector<float> comsol_point = findNearestPoint(xc, yc, zc, "/home/icebear/MyProjects/ROSProjects/yoloqt_ws/src/yolov8qt/result/comsol_result_final.txt");
            // std::vector<float> comsol_point = findNearestPoint(-obj_info.centroid_.y, -obj_info.centroid_.z, obj_info.centroid_.x, "/home/icebear/MyProjects/ROSProjects/yoloqt_ws/src/yolov8qt/result/comsol_result.txt");
            // std::cout << "距离 " << bbox_name << " 最近的点为：(" << comsol_point[0] << ", " << comsol_point[1] << ", " << comsol_point[2] << ") m" << std::endl;
            // COMSOL仿真结果数据格式为：(x, y, z, Ex, Ey, Ez, E)，这里为峰值，与模拟电荷法的计算值一致
            // std::cout << "仿真电场强度为：(" << comsol_point[3] << ", " << comsol_point[4] << ", " << comsol_point[5] << ", " << comsol_point[6] << ") V/m" << std::endl;
            // std::cout << "计算电场强度为：(" << E_simulation[0] << ", " << E_simulation[1] << ", " << E_simulation[2] << ", " << E_simulation[3] << ") V/m" << std::endl;
            // RJ6K 实测电场强度数据：(综合场强， x方向分量， y方向分量， z方向分量， 综合场强@50 Hz)，如果实测值是有效值，还要乘以根号2。才能与计算值和仿真值进行比较
            std::cout << "实测电场强度为：(" << rj6k_datas.data[1] << ", " << rj6k_datas.data[2] << ", " << rj6k_datas.data[3] << ", " << rj6k_datas.data[4] << ") V/m" << std::endl;
            // 打印模拟电荷法计算的电场强度（峰值）和转换后的有效值
            std::cout << "计算电场强度（峰值）为：(" << E_simulation[0] << ", " << E_simulation[1] << ", " << E_simulation[2] << ", " << E_simulation[3] << ") V/m" << std::endl;
            std::cout << "计算电场强度（有效值）为：(" << E_simulation[0]/std::sqrt(2.0) << ", " << E_simulation[1]/std::sqrt(2.0) << ", " << E_simulation[2]/std::sqrt(2.0) << ", " << E_simulation[3]/std::sqrt(2.0) << ") V/m" << std::endl;
            // 原代码：比较comsol和各个方向分量（已注释）
            // if((abs(comsol_point[3] - rj6k_datas.data[1]) <= CHARGED_THRES && abs(comsol_point[4] - rj6k_datas.data[2]) <= CHARGED_THRES && abs(comsol_point[5] - rj6k_datas.data[3]) <= CHARGED_THRES && abs(comsol_point[6] - rj6k_datas.data[4]) <= CHARGED_THRES) || (abs(E_simulation[0] - rj6k_datas.data[1]) <= CHARGED_THRES && abs(E_simulation[1] - rj6k_datas.data[2]) <= CHARGED_THRES && abs(E_simulation[2] - rj6k_datas.data[3]) <= CHARGED_THRES && abs(E_simulation[3] - rj6k_datas.data[4]) <= CHARGED_THRES)){
            // 新代码：将模拟电荷法计算的峰值转换为有效值（除以根号2），再与50Hz频点综合电场（有效值）比较
            if(abs(E_simulation[3] / std::sqrt(2.0) - rj6k_datas.data[4]) <= CHARGED_THRES){
                this->result_state = "带电";
            }
            else this->result_state = "不带电";
        }
        else if(bbox_name == "kaiguan"){
            //模拟电荷法计算观测点(xc,yc,zc)的电场强度:(Ex, Ey, Ez, normE)
            std::vector<double> E_simulation = calculateElectricField_kaiguan(xc, yc, zc, "/home/icebear/MyProjects/ROSProjects/yoloqt_ws/src/yolov8qt/config/kaiguan_config.txt");
            // std::cout << "计算电场强度为：(" << E_simulation[0] << ", " << E_simulation[1] << ", " << E_simulation[2] << ", " << E_simulation[3] << ") V/m" << std::endl;
            // RJ6K 实测电场强度数据：(综合场强， x方向分量， y方向分量， z方向分量， 综合场强@50 Hz)，如果实测值是有效值，还要乘以根号2。才能与计算值进行比较
            std::cout << "实测电场强度为：(" << rj6k_datas.data[1] << ", " << rj6k_datas.data[2] << ", " << rj6k_datas.data[3] << ", " << rj6k_datas.data[4] << ") V/m" << std::endl;
            // 打印模拟电荷法计算的电场强度（峰值）和转换后的有效值
            std::cout << "计算电场强度（峰值）为：(" << E_simulation[0] << ", " << E_simulation[1] << ", " << E_simulation[2] << ", " << E_simulation[3] << ") V/m" << std::endl;
            std::cout << "计算电场强度（有效值）为：(" << E_simulation[0]/std::sqrt(2.0) << ", " << E_simulation[1]/std::sqrt(2.0) << ", " << E_simulation[2]/std::sqrt(2.0) << ", " << E_simulation[3]/std::sqrt(2.0) << ") V/m" << std::endl;
            // 原代码：比较各个方向分量（已注释）
            // if(abs(E_simulation[0] - rj6k_datas.data[1]) <= CHARGED_THRES && abs(E_simulation[1] - rj6k_datas.data[2]) <= CHARGED_THRES && abs(E_simulation[2] - rj6k_datas.data[3]) <= CHARGED_THRES && abs(E_simulation[3] - rj6k_datas.data[4]) <= CHARGED_THRES){
            // 新代码：将模拟电荷法计算的峰值转换为有效值（除以根号2），再与50Hz频点综合电场（有效值）比较
            if(abs(E_simulation[3] / std::sqrt(2.0) - rj6k_datas.data[4]) <= CHARGED_THRES){
                this->result_state = "带电";
            }
            else this->result_state = "不带电";
        }

        this->result_electricity = rj6k_datas.data[4];  //QT软件显示的电场强度数据以实测值为准
    }
}

void YOLOv8::infer()
{
    this->context->enqueueV2(this->device_ptrs.data(), this->stream, nullptr);
    for (int i = 0; i < this->num_outputs; i++) {
        size_t osize = this->output_bindings[i].size * this->output_bindings[i].dsize;
        CHECK(cudaMemcpyAsync(this->host_ptrs[i], this->device_ptrs[i + this->num_inputs], osize, cudaMemcpyDeviceToHost, this->stream));
    }
    cudaStreamSynchronize(this->stream);
}


void YOLOv8::postprocess(std::vector<Object>& objs, float score_thres, float iou_thres, int topk, int num_labels)
{
    objs.clear();
    //输出张量的通道数，表示检测框的属性数（坐标、置信度、类别概率）
    auto num_channels = this->output_bindings[0].dims.d[1];
    //输出张量的高度（锚点数量），表示检测框的数量
    auto num_anchors = this->output_bindings[0].dims.d[2];
    auto& dw = this->pparam.dw; //预处理时的宽度填充
    auto& dh = this->pparam.dh; //预处理时的高度填充
    auto& width = this->pparam.width; //输入图像的宽度
    auto& height = this->pparam.height; //输入图像的高度
    auto& ratio = this->pparam.ratio;  //预处理时的缩放比例
    std::vector<cv::Rect> bboxes;  //检测到的目标框
    std::vector<float> scores;  //检测到的目标的置信度分数
    std::vector<int> labels;  //检测到的目标的类别标签
    std::vector<int> indices; //NMS后保留的目标框的索引

    //output是检测结果，static_cast<float*>(this->host_ptrs[0])用来初始化output
    cv::Mat output = cv::Mat(num_channels, num_anchors, CV_32F, static_cast<float*>(this->host_ptrs[0]));
    //转置output，每一行表示一个检测框的属性：[x, y, w, h, score, label_0, label_1, ..., label_n]
    output = output.t();
    //遍历每个锚点（检测框），只保留置信度大于score_thres的检测框
    for (int i = 0; i < num_anchors; i++) {
        auto row_ptr = output.row(i).ptr<float>();
        auto bboxes_ptr = row_ptr;
        auto scores_ptr = row_ptr + 4;
        auto max_s_ptr = std::max_element(scores_ptr, scores_ptr + num_labels);
        float score = *max_s_ptr;
        if (score > score_thres) {
            float x = *bboxes_ptr++ - dw;
            float y = *bboxes_ptr++ - dh;
            float w = *bboxes_ptr++;
            float h = *bboxes_ptr;
            float x0 = clamp((x - 0.5f * w) * ratio, 0.f, width);
            float y0 = clamp((y - 0.5f * h) * ratio, 0.f, height);
            float x1 = clamp((x + 0.5f * w) * ratio, 0.f, width);
            float y1 = clamp((y + 0.5f * h) * ratio, 0.f, height);

            int label = max_s_ptr - scores_ptr;
            cv::Rect_<float> bbox;
            bbox.x = x0;
            bbox.y = y0;
            bbox.width = x1 - x0;
            bbox.height = y1 - y0;
            bboxes.push_back(bbox);
            labels.push_back(label);
            scores.push_back(score);
        }
    }
    //NMS，去除置信度低的检测框
#ifdef BATCHED_NMS
    cv::dnn::NMSBoxesBatched(bboxes, scores, labels, score_thres, iou_thres, indices);
#else
    cv::dnn::NMSBoxes(bboxes, scores, score_thres, iou_thres, indices);
#endif
    int cnt = 0;
    for (auto& i : indices) {
        if (cnt >= topk) {
            break;  //最终保留的检测框数量不超过topk个
        }
        Object obj;
        obj.rect = bboxes[i];
        obj.prob = scores[i];
        obj.label = labels[i];
        objs.push_back(obj);  //objs存储最终保留的所有检测框
        cnt += 1;
    }
}


void YOLOv8::draw_objects(const cv::Point& click_pos, const cv::Mat& image, cv::Mat& res, const std::vector<Object>& objs, const std::vector<std::string>& CLASS_NAMES, const std::vector<std::vector<unsigned int>>& COLORS)
{
    if(objs.empty()){
        res = image.clone();  //如果没有检测到目标，直接返回相机原图
        return;
    }
    res = image.clone();
    //viewer_bboxCloud->removeAllPointClouds();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_bbox(new pcl::PointCloud<pcl::PointXYZ>);//存储在当前检测框内的点云
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);
    int leftTop_x = 0, leftTop_y = 0, rightBottom_x = 0, rightBottom_y = 0;
    det::Object target_obj;
    bool find_target = false;
    for(const auto& obj : objs){
        if(obj.label >= CLASS_NAMES.size() || obj.label >= COLORS.size()) continue;
        if(click_pos.x >= static_cast<int>(obj.rect.x) && click_pos.x <= static_cast<int>(obj.rect.x + obj.rect.width) && click_pos.y >= static_cast<int>(obj.rect.y) && click_pos.y <= static_cast<int>(obj.rect.y + obj.rect.height)){
            leftTop_x = static_cast<int>(obj.rect.x);
            leftTop_y = static_cast<int>(obj.rect.y);
            rightBottom_x = static_cast<int>(obj.rect.x + obj.rect.width);
            rightBottom_y = static_cast<int>(obj.rect.y + obj.rect.height);
            target_obj = obj;
            find_target = true;
            //ROS_INFO("Find Target Object");
            break;
        }
    }
    if(find_target == false){
        //ROS_INFO("No Target Object");
        return;
    }
    if(!cloud->empty()){
        //ROS_INFO("Start Projection");
        for(pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud->points.begin(); it != cloud->points.end(); ++it){
            if(it->x >maxX || it->x < 0.0 || abs(it->y) > maxY || it->z < minZ) continue;
            X.at<double>(0, 0) = it->x;
            X.at<double>(1, 0) = it->y;
            X.at<double>(2, 0) = it->z;
            X.at<double>(3, 0) = 1;
            Y = proj.i_params.cameraIn * proj.i_params.camtocam_mat * proj.i_params.RT * X;
            cv::Point pt;
            pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2);
            pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);
            if(pt.x >= leftTop_x && pt.x <= rightBottom_x && pt.y >= leftTop_y && pt.y <= rightBottom_y){
                float val = it->x;
                float maxVal = 20.0;
                int red = std::min(255, (int)(255 * abs((val - maxVal) / maxVal)));
                int green = std::min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
                cv::circle(res, pt, 1, cv::Scalar(0, green, red), -1);
                cloud_in_bbox->push_back(*it); //收集在当前检测框内的点云
            }
        }
        // if(!cloud_in_bbox->empty()){
        //     std::string class_name = CLASS_NAMES[target_obj.label].c_str();
        //     if(class_name == "muxian") this->PointCloudPassThroughFilter(cloud_in_bbox);
        //     else if(class_name == "kaiguan") this->PointCloudPassThroughFilterKaiguan(cloud_in_bbox);
        //     this->CloudCluster(cloud_in_bbox, class_name);  //CloudCluster是成员函数，必须通过类实例来调用
        // }
        // else ROS_WARN("cloud_in_bbox 点云为空");
        std::string class_name = CLASS_NAMES[target_obj.label].c_str();
        if(class_name == "muxian") this->PointCloudPassThroughFilter(cloud);
        else if(class_name == "kaiguan") this->PointCloudPassThroughFilterKaiguan(cloud);
        this->CloudCluster(cloud, class_name);  //CloudCluster是成员函数，必须通过类实例来调用
        std::cout << "cloud cluster end" << std::endl;

    }
    this->result_class = CLASS_NAMES[target_obj.label].c_str();
    this->result_conf = target_obj.prob * 100;
    //绘制检测框
    cv::Scalar color = cv::Scalar(COLORS[target_obj.label][0], COLORS[target_obj.label][1], COLORS[target_obj.label][2]);
    cv::rectangle(res, target_obj.rect, color, 2);
    //绘制标签 
    char text[256];
    sprintf(text, "%s %.1f%%", CLASS_NAMES[target_obj.label].c_str(), target_obj.prob * 100);
    int baseLine = 0;
    cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.8, 2, &baseLine);
    int x = (int)target_obj.rect.x;
    int y = (int)target_obj.rect.y + 1;
    if(y > res.rows) y = res.rows;
    cv::rectangle(res, cv::Rect(x, y, label_size.width, label_size.height + baseLine), {0, 0, 255}, -1);
    cv::putText(res, text, cv::Point(x, y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.8, {255, 255, 255}, 2);
    std::cout << "draw_objects end" << std::endl;
}

void YOLOv8::show_objects(const cv::Mat& image, cv::Mat& show_image, const std::vector<Object>& objs, const std::vector<std::string>& CLASS_NAMES, const std::vector<std::vector<unsigned int>>& COLORS)
{
    if(objs.empty()) return;
    for(const auto& obj : objs){
        if(obj.label >= CLASS_NAMES.size() || obj.label >= COLORS.size()) continue;
        cv::Scalar color = cv::Scalar(COLORS[obj.label][0], COLORS[obj.label][1], COLORS[obj.label][2]);
        cv::rectangle(show_image, obj.rect, color, 2);
        char text[256];
        sprintf(text, "%s %.1f%%", CLASS_NAMES[obj.label].c_str(), obj.prob * 100);
        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.8, 2, &baseLine);
        int x = (int)obj.rect.x;
        int y = (int)obj.rect.y + 1;
        if(y > show_image.rows) y = show_image.rows;
        cv::rectangle(show_image, cv::Rect(x, y, label_size.width, label_size.height + baseLine), {0, 0, 255}, -1);
        cv::putText(show_image, text, cv::Point(x, y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.8, {255, 255, 255}, 2);
    }
}

void YOLOv8::stop_detect(){
    if(this->is_running == false) return;
    this->is_running = false;
       // 释放 CUDA 资源
    if (this->stream) {
        cudaStreamDestroy(this->stream);
        this->stream = nullptr;
    }

    // 释放 TensorRT 资源
    if (this->context) {
        this->context->destroy();
        this->context = nullptr;
    }
    if (this->engine) {
        this->engine->destroy();
        this->engine = nullptr;
    }
    if (this->runtime) {
        this->runtime->destroy();
        this->runtime = nullptr;
    }

    // 释放分配的 CUDA 内存
    for (auto& ptr : this->device_ptrs) {
        if (ptr) {
            cudaFree(ptr);
            ptr = nullptr;
        }
    }

    // 释放分配的主机内存
    for (auto& ptr : this->host_ptrs) {
        if (ptr) {
            cudaFreeHost(ptr);
            ptr = nullptr;
        }
    }
}

void YOLOv8::process(const cv::Point& click_pos, const cv::Mat& image, cv::Mat& image_result, cv::Size& size, std::vector<Object>& objects, const std::vector<std::string>& CLASS_NAMES, const std::vector<std::vector<unsigned int>>& COLORS, float score_thres, float iou_thres, int topk, int num_labels){
    if(this->is_running == false) return;
    //ROS_INFO("——————————————Start process——————————————");
    //ROS_INFO("Start copy_from_Mat");
    //auto process_start = std::chrono::system_clock::now();
    this->copy_from_Mat(image, size);
    //ROS_INFO("copy_from_Mat success");
    //ROS_INFO("Start infer");
    // auto start_time = std::chrono::system_clock::now();
    this->infer();
    // auto end_time = std::chrono::system_clock::now();
    //ROS_INFO("infer success");
    //ROS_INFO("Start postprocess");
    this->postprocess(objects, score_thres, iou_thres, topk, num_labels);
    //ROS_INFO("postprocess success");
    //ROS_INFO("Starting PointCloudFilter");
    this->PointCloudVoxelFilter();
    //ROS_INFO("Finished PointCloudFilter");
    //ROS_INFO("Start draw_objects");
    this->draw_objects(click_pos, image, image_result, objects, CLASS_NAMES, COLORS);
    //ROS_INFO("draw_objects success");
    //auto process_end = std::chrono::system_clock::now();
    //auto infer_time = (double)std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000;
    //auto process_time = (double)std::chrono::duration_cast<std::chrono::microseconds>(process_end - process_start).count() / 1000;
    //cv::putText(image_result, "infer FPS: " + std::to_string(int(1000/infer_time)), cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 1, 8);
    //cv::putText(image_result, "process FPS: " + std::to_string(int(1000/process_time)), cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 1, 8);
    //this->result_FPS = int(1000/process_time);
    //ROS_INFO("detect inference time cost: %2.4f ms", infer_time);
    //ROS_INFO("detect process time cost: %2.4f ms", process_time);

    // 可视化检测结果
    // cv::imshow("detect result", image_result);
    // if(cv::waitKey(10) == 27){
    //     cv::destroyAllWindows();
    // }

    //ROS_INFO("——————————————Finish process——————————————");
}
