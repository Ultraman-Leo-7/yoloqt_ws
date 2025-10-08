#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//#include "yolov8.h"
#include "ClickableLabel.h"
#include "QNode.h"
//#include <memory>
//#include <mutex>
//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/synchronizer.h>
//#include <message_filters/sync_policies/approximate_time.h>
//#include <condition_variable>
#include <QMainWindow>
#include <QString>
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <QDebug>
#include <QTimer>
#include <QDateTime>
//#include <QImage>
//#include <QThread>
#include <QMouseEvent>


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = nullptr);
    ~MainWindow();

signals:
    void mouseClicked(const QPoint& pos);
    void continueDetectionSignal();

protected:
    bool eventFilter(QObject* obj, QEvent* event) override;

private slots:
    void on_SaveDataButton_clicked();
    void on_ModelListShowBox_currentIndexChanged(const QString &modelname);
    //void on_ResultImgShowLabel_clicked(int x, int y);
    void on_ContinueDetectButton_clicked();
    void updateData(const QImage& img, int fps, const QString& class_name, float conf, float distance, float electric, const QString& state);
    void handleRosShutdown();

private:
    void closeEvent(QCloseEvent* event) override;

private:
    Ui::MainWindow *ui;
    QNode qnode;
    QImage icon;
    // std::mutex model_mutex;  //模型切换时的互斥锁
    // std::mutex callback_mutex;  //回调函数的互斥锁，保证回调函数执行完了再执行saveData和continueDetect槽函数
    // std::condition_variable callback_cv;  //条件变量，保证槽函数在回调函数执行完毕后会执行
    // bool callback_done;  //callback_done 为 true 时，表示回调函数已经执行完毕
};
#endif // MAINWINDOW_H
