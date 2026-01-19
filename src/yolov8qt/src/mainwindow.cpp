#include <QtGui>
#include <QApplication>
#include "../include/yolov8qt/mainwindow.h"
#include <ui_mainwindow.h>
#include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>
// #include <pcl_conversions/pcl_conversions.h>

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , qnode(argc, argv)
{
    //qDebug() << "MainWindow constructor called";
    ui->setupUi(this);

    // 获取屏幕分辨率
    QRect screenRect = QApplication::primaryScreen()->geometry();
    int screenWidth = screenRect.width();
    int screenHeight = screenRect.height();

    // 设置窗口大小为屏幕的80%，并保持16:9比例
    int windowWidth = screenWidth * 0.8;
    int windowHeight = static_cast<int>(windowWidth / 1.777);

    // 如果高度超出屏幕80%，调整高度并重新计算宽度
    if (windowHeight > screenHeight * 0.8) {
        windowHeight = screenHeight * 0.8;
        windowWidth = static_cast<int>(windowHeight * 1.777);
    }

    // 调整窗口大小
    this->resize(windowWidth, windowHeight);

    // 将窗口居中显示
    int x = (screenWidth - windowWidth) / 2;
    int y = (screenHeight - windowHeight) / 2;
    this->move(x, y);

    if(!qnode.init()){
        qDebug() << "ROS Node init failed";
    }
    else{
        qDebug() << "ROS Node init success";
    }
    /*没开启相机和雷达时，显示默认的icon*/
    QString iconPath = QCoreApplication::applicationDirPath() + "/../../../src/yolov8qt/resources/images/logo.png";
    if (!QFile::exists(iconPath)) {
        qDebug() << "Icon file not found:" << iconPath;
        return;
    }
    icon.load(iconPath);
    ui->ResultImgShowLabel->setPixmap(QPixmap::fromImage(icon.scaled(1280, 720, Qt::IgnoreAspectRatio, Qt::SmoothTransformation)));
    qDebug() << "默认Icon加载完成";

    /*下拉列表框已经在Qt Creator中设置了，这里禁止通过代码再次添加！*/
    // qDebug() << "开始向 combo box 添加模型";
    ui->ModelListShowBox->addItem("3183.engine");
    // qDebug() << "向 combo box 添加 3216 模型完成";
    ui->ModelListShowBox->addItem("TCS.engine");
    ui->ModelListShowBox->addItem("TCS-fp16.engine");
    // qDebug() << "向 combo box 添加 536 模型完成";
    // //ui->ModelListShowBox->addItem("vertical_muxian.engine");
    qDebug() << "向 combo box 添加模型完成";


    connect(ui->ModelListShowBox, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(on_ModelListShowBox_currentIndexChanged(const QString&)));
    connect(ui->VListShowBox, SIGNAL(currentTextChanged(const QString&)), this, SLOT(on_VListShowBox_currentTextChanged(const QString&)));
    connect(ui->SaveDataButton, &QPushButton::clicked, this, &MainWindow::on_SaveDataButton_clicked);
    //clicked信号和ResultImgShowLabel的槽函数已经在Qt Creator中通过右键转到槽连接了，这里禁止再次connect连接，否则会出现单击一次，槽函数被调用两次的情况
    //connect(ui->ResultImgShowLabel, &ClickableLabel::clicked, this, &MainWindow::on_ResultImgShowLabel_clicked);
    //connect(ui->ContinueDetectButton, &QPushButton::clicked, this, &MainWindow::on_ContinueDetectButton_clicked);

    connect(&qnode, &QNode::DataReceived, this, &MainWindow::updateData);
    connect(&qnode, &QNode::rosShutdown, this, &MainWindow::handleRosShutdown);
    connect(this, &MainWindow::mouseClicked, &qnode, &QNode::setClickPosition);
    connect(this, &MainWindow::continueDetectionSignal, &qnode, &QNode::continueDetection);

    //安装事件过滤器
    ui->ResultImgShowLabel->installEventFilter(this);

    // 初始化时设置默认电压值（220kV是第二个选项，索引为1）
    ui->VListShowBox->setCurrentIndex(1);

    qnode.start();
}

MainWindow::~MainWindow()
{
    delete ui;
}

bool MainWindow::eventFilter(QObject* obj, QEvent* event){
    if(obj == ui->ResultImgShowLabel && event->type() == QEvent::MouseButtonPress){
        QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
        if(mouseEvent->button() == Qt::LeftButton){
            emit mouseClicked(mouseEvent->pos());
            return true;
        }
    }
    return QMainWindow::eventFilter(obj, event);
}

void MainWindow::closeEvent(QCloseEvent* event){
    {
        //std::unique_lock<std::mutex> lock(callback_mutex); // 加锁
        //callback_cv.wait(lock, [this] { return callback_done; });
        
        qDebug() << "APP is closed";
    }
    event->accept();
    //QMainWindow::closeEvent(event);
}

void MainWindow::updateData(const QImage& img, int fps, const QString& class_name, float conf, float distance, float electric, const QString& state){
    //qDebug() << "updateData slot called: " << QThread::currentThreadId();
    ui->ResultImgShowLabel->setPixmap(QPixmap::fromImage(img));
    ui->FPSShowLabel->setText(QString::number(fps) + " FPS");
    ui->ClassShowLabel->setText(class_name);
    ui->ConfShowLabel->setText(QString::number(conf) + " %");
    ui->PosShowLabel->setText(QString::number(distance) + " m");
    ui->ElectricShowLabel->setText(QString::number(electric) + " V/m");
    ui->StateShowLabel->setText(state);
}

void MainWindow::handleRosShutdown() {
    close();
}

void MainWindow::on_SaveDataButton_clicked()
{
    QString csv_filepath = QCoreApplication::applicationDirPath() + "/../../../src/yolov8qt/result/result.csv";
    QFile file(csv_filepath);
    if(!file.exists()){
        file.open(QIODevice::WriteOnly);
        QTextStream out(&file);
        out.setCodec("UTF-8");
        out << QStringLiteral("时间,设备类别,置信度,位姿,电场强度,是否带电\n");
        qDebug() << "Create new csv file";
    }
    else{
        file.open(QIODevice::Append);
        QTextStream out(&file);
        out.setCodec("UTF-8");
    }
    QString now_time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString deviceClass = ui->ClassShowLabel->text();
    QString confidence = ui->ConfShowLabel->text();
    QString position = ui->PosShowLabel->text();
    QString electricField = ui->ElectricShowLabel->text();
    QString state = ui->StateShowLabel->text();

    // 写入数据
    QTextStream out(&file);
    out << now_time << "," << deviceClass << ',' << confidence << ',' << position << ',' << electricField << ',' << state << '\n';

    // 关闭文件
    file.close();
}

void MainWindow::on_ModelListShowBox_currentIndexChanged(const QString &modelname)
{
    //切换模型时加锁
    //std::lock_guard<std::mutex> lock(model_mutex);

    //qDebug() << "ModelListShowBox Acquired lock";

    QString model_path = QCoreApplication::applicationDirPath() + "/../../../src/yolov8qt/weights/" + modelname;
    if (!QFile::exists(model_path)) {
        qDebug() << "Model file not found:" << model_path;
        return;
    }
    qnode.switchModel(model_path);
    // // 释放当前的 YOLOv8 模型资源
    // yolov8.reset();

    // //qDebug() << "ModelListShowBox release yolov8";

    // // 加载新的 YOLOv8 模型
    // yolov8 = std::make_shared<YOLOv8>(model_path.toStdString());
    // qDebug() << "ModelListShowBox make_pipe";
    // yolov8->make_pipe(true);

    // // 重新设置 CUDA 设备
    // cudaSetDevice(0);
}

void MainWindow::on_VListShowBox_currentTextChanged(const QString &voltage_text)
{
    // 从文本中提取电压值（例如 "110kV" -> 110.0）
    QString voltage_number = voltage_text;
    voltage_number.remove("kV");  // 移除单位

    bool ok;
    double voltage_kv = voltage_number.toDouble(&ok);

    if (ok && voltage_kv > 0) {
        qnode.setVoltage(voltage_kv);
        qDebug() << "主窗口：电压等级已切换为 " << voltage_kv << " kV";
    } else {
        qDebug() << "警告：无效的电压值 - " << voltage_text;
    }
}

// void MainWindow::on_ResultImgShowLabel_clicked(int x, int y){
//     //std::unique_lock<std::mutex> lock(callback_mutex); // 加锁
//     qDebug() << "on_ResultImgShowLabel_clicked thread ID: " << QThread::currentThreadId();

//     //qDebug() << "ResultImgShowLabel Acquired lock";
//     // 等待 DetectCallback 完成
//     //callback_cv.wait(lock, [this] { return callback_done; });
//     //qDebug() << "DetectCallback completed-ResultImgShowLabel";
//     is_clicked = true;
//     //callback_done = false;
//     click_pos = cv::Point(x, y);
//     qDebug() << "Clicked position: (" << click_pos.x << ", " << click_pos.y << ")";
//     // Clicked position: ( 501 ,  377 )
// }

void MainWindow::on_ContinueDetectButton_clicked(){

    //std::unique_lock<std::mutex> lock(callback_mutex); // 加锁
    //qDebug() << "ContinueDetectButton Acquired lock";
    //qDebug() << "on_ContinueDetectButton_clicked thread ID: " << QThread::currentThreadId();
    // 等待 DetectCallback 完成
    //callback_cv.wait(lock, [this] { return callback_done; });

    //qDebug() << "DetectCallback completed-ContinueDetectButton";

    emit continueDetectionSignal();
}
