#include "../include/yolov8qt/ClickableLabel.h"

ClickableLabel::ClickableLabel(QWidget *parent) : QLabel(parent){}

void ClickableLabel::mousePressEvent(QMouseEvent *event){
    if(event->button() == Qt::LeftButton){
        emit clicked(event->x(), event->y());
    }
    QLabel::mousePressEvent(event);
}