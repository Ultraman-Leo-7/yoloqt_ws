#include <QApplication>
#include "../include/yolov8qt/mainwindow.h"

int main(int argc, char *argv[])
{
    // // 初始化 ROS
    ros::init(argc, argv, "yolov8_qt_node");
    
    QApplication a(argc, argv);
    QString style_filepath = QCoreApplication::applicationDirPath() + "/../../../src/yolov8qt/resources/styles/style.qss";
    QFile file(style_filepath);
    if (file.open(QFile::ReadOnly)) {
        QString styleSheet = QLatin1String(file.readAll());
        a.setStyleSheet(styleSheet);
        file.close();
    }
    MainWindow w(argc, argv);
    w.show();
    a.connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()));
    return a.exec();
}
