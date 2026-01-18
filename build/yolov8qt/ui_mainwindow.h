/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "../include/yolov8qt/ClickableLabel.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_top;
    QLabel *ResultTextLabel;
    QLabel *WarnTextLabel;
    QHBoxLayout *horizontalLayout_2;
    ClickableLabel *ResultImgShowLabel;
    QVBoxLayout *verticalLayout;
    QLabel *DataTextLabel;
    QHBoxLayout *horizontalLayout_model;
    QLabel *ChangeModelLabel;
    QComboBox *ModelListShowBox;
    QHBoxLayout *horizontalLayout_fps;
    QLabel *FPSTextLabel;
    QLabel *FPSShowLabel;
    QHBoxLayout *horizontalLayout;
    QLabel *ChangeVLabel;
    QComboBox *VListShowBox;
    QVBoxLayout *verticalLayout_data;
    QHBoxLayout *horizontalLayout_class;
    QLabel *ClassTextLabel;
    QLabel *ClassShowLabel;
    QHBoxLayout *horizontalLayout_conf;
    QLabel *ConfTextLabel;
    QLabel *ConfShowLabel;
    QHBoxLayout *horizontalLayout_pos;
    QLabel *PosTextLabel;
    QLabel *PosShowLabel;
    QHBoxLayout *horizontalLayout_electric;
    QLabel *ElectricTextLabel;
    QLabel *ElectricShowLabel;
    QHBoxLayout *horizontalLayout_state;
    QLabel *StateTextLabel;
    QLabel *StateShowLabel;
    QPushButton *SaveDataButton;
    QPushButton *ContinueDetectButton;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(290, 567);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        verticalLayout_2 = new QVBoxLayout(centralwidget);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout_top = new QHBoxLayout();
        horizontalLayout_top->setObjectName(QString::fromUtf8("horizontalLayout_top"));
        ResultTextLabel = new QLabel(centralwidget);
        ResultTextLabel->setObjectName(QString::fromUtf8("ResultTextLabel"));

        horizontalLayout_top->addWidget(ResultTextLabel, 0, Qt::AlignHCenter);

        WarnTextLabel = new QLabel(centralwidget);
        WarnTextLabel->setObjectName(QString::fromUtf8("WarnTextLabel"));

        horizontalLayout_top->addWidget(WarnTextLabel);


        verticalLayout_2->addLayout(horizontalLayout_top);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        ResultImgShowLabel = new ClickableLabel(centralwidget);
        ResultImgShowLabel->setObjectName(QString::fromUtf8("ResultImgShowLabel"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(ResultImgShowLabel->sizePolicy().hasHeightForWidth());
        ResultImgShowLabel->setSizePolicy(sizePolicy);

        horizontalLayout_2->addWidget(ResultImgShowLabel);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        DataTextLabel = new QLabel(centralwidget);
        DataTextLabel->setObjectName(QString::fromUtf8("DataTextLabel"));
        DataTextLabel->setMinimumSize(QSize(0, 0));
        DataTextLabel->setMaximumSize(QSize(16777215, 20));

        verticalLayout->addWidget(DataTextLabel);

        horizontalLayout_model = new QHBoxLayout();
        horizontalLayout_model->setObjectName(QString::fromUtf8("horizontalLayout_model"));
        ChangeModelLabel = new QLabel(centralwidget);
        ChangeModelLabel->setObjectName(QString::fromUtf8("ChangeModelLabel"));
        ChangeModelLabel->setMaximumSize(QSize(16777215, 20));

        horizontalLayout_model->addWidget(ChangeModelLabel);

        ModelListShowBox = new QComboBox(centralwidget);
        ModelListShowBox->addItem(QString());
        ModelListShowBox->addItem(QString());
        ModelListShowBox->setObjectName(QString::fromUtf8("ModelListShowBox"));
        ModelListShowBox->setMaximumSize(QSize(16777215, 20));

        horizontalLayout_model->addWidget(ModelListShowBox);


        verticalLayout->addLayout(horizontalLayout_model);

        horizontalLayout_fps = new QHBoxLayout();
        horizontalLayout_fps->setObjectName(QString::fromUtf8("horizontalLayout_fps"));
        FPSTextLabel = new QLabel(centralwidget);
        FPSTextLabel->setObjectName(QString::fromUtf8("FPSTextLabel"));
        FPSTextLabel->setMaximumSize(QSize(16777215, 20));

        horizontalLayout_fps->addWidget(FPSTextLabel);

        FPSShowLabel = new QLabel(centralwidget);
        FPSShowLabel->setObjectName(QString::fromUtf8("FPSShowLabel"));
        FPSShowLabel->setMaximumSize(QSize(16777215, 20));

        horizontalLayout_fps->addWidget(FPSShowLabel, 0, Qt::AlignHCenter);


        verticalLayout->addLayout(horizontalLayout_fps);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        ChangeVLabel = new QLabel(centralwidget);
        ChangeVLabel->setObjectName(QString::fromUtf8("ChangeVLabel"));
        ChangeVLabel->setMaximumSize(QSize(16777215, 20));

        horizontalLayout->addWidget(ChangeVLabel);

        VListShowBox = new QComboBox(centralwidget);
        VListShowBox->addItem(QString());
        VListShowBox->addItem(QString());
        VListShowBox->addItem(QString());
        VListShowBox->addItem(QString());
        VListShowBox->setObjectName(QString::fromUtf8("VListShowBox"));
        VListShowBox->setMaximumSize(QSize(16777215, 20));

        horizontalLayout->addWidget(VListShowBox);


        verticalLayout->addLayout(horizontalLayout);

        verticalLayout_data = new QVBoxLayout();
        verticalLayout_data->setObjectName(QString::fromUtf8("verticalLayout_data"));
        horizontalLayout_class = new QHBoxLayout();
        horizontalLayout_class->setObjectName(QString::fromUtf8("horizontalLayout_class"));
        ClassTextLabel = new QLabel(centralwidget);
        ClassTextLabel->setObjectName(QString::fromUtf8("ClassTextLabel"));
        ClassTextLabel->setMinimumSize(QSize(0, 60));

        horizontalLayout_class->addWidget(ClassTextLabel);

        ClassShowLabel = new QLabel(centralwidget);
        ClassShowLabel->setObjectName(QString::fromUtf8("ClassShowLabel"));
        ClassShowLabel->setMinimumSize(QSize(0, 60));

        horizontalLayout_class->addWidget(ClassShowLabel, 0, Qt::AlignHCenter);


        verticalLayout_data->addLayout(horizontalLayout_class);

        horizontalLayout_conf = new QHBoxLayout();
        horizontalLayout_conf->setObjectName(QString::fromUtf8("horizontalLayout_conf"));
        ConfTextLabel = new QLabel(centralwidget);
        ConfTextLabel->setObjectName(QString::fromUtf8("ConfTextLabel"));
        ConfTextLabel->setMinimumSize(QSize(0, 60));

        horizontalLayout_conf->addWidget(ConfTextLabel);

        ConfShowLabel = new QLabel(centralwidget);
        ConfShowLabel->setObjectName(QString::fromUtf8("ConfShowLabel"));
        ConfShowLabel->setMinimumSize(QSize(0, 60));

        horizontalLayout_conf->addWidget(ConfShowLabel, 0, Qt::AlignHCenter);


        verticalLayout_data->addLayout(horizontalLayout_conf);

        horizontalLayout_pos = new QHBoxLayout();
        horizontalLayout_pos->setObjectName(QString::fromUtf8("horizontalLayout_pos"));
        PosTextLabel = new QLabel(centralwidget);
        PosTextLabel->setObjectName(QString::fromUtf8("PosTextLabel"));
        PosTextLabel->setMinimumSize(QSize(0, 60));

        horizontalLayout_pos->addWidget(PosTextLabel);

        PosShowLabel = new QLabel(centralwidget);
        PosShowLabel->setObjectName(QString::fromUtf8("PosShowLabel"));
        PosShowLabel->setMinimumSize(QSize(0, 60));

        horizontalLayout_pos->addWidget(PosShowLabel, 0, Qt::AlignHCenter);


        verticalLayout_data->addLayout(horizontalLayout_pos);

        horizontalLayout_electric = new QHBoxLayout();
        horizontalLayout_electric->setObjectName(QString::fromUtf8("horizontalLayout_electric"));
        ElectricTextLabel = new QLabel(centralwidget);
        ElectricTextLabel->setObjectName(QString::fromUtf8("ElectricTextLabel"));
        ElectricTextLabel->setMinimumSize(QSize(0, 60));

        horizontalLayout_electric->addWidget(ElectricTextLabel);

        ElectricShowLabel = new QLabel(centralwidget);
        ElectricShowLabel->setObjectName(QString::fromUtf8("ElectricShowLabel"));
        ElectricShowLabel->setMinimumSize(QSize(0, 60));

        horizontalLayout_electric->addWidget(ElectricShowLabel, 0, Qt::AlignHCenter);


        verticalLayout_data->addLayout(horizontalLayout_electric);

        horizontalLayout_state = new QHBoxLayout();
        horizontalLayout_state->setObjectName(QString::fromUtf8("horizontalLayout_state"));
        StateTextLabel = new QLabel(centralwidget);
        StateTextLabel->setObjectName(QString::fromUtf8("StateTextLabel"));
        StateTextLabel->setMinimumSize(QSize(0, 60));

        horizontalLayout_state->addWidget(StateTextLabel);

        StateShowLabel = new QLabel(centralwidget);
        StateShowLabel->setObjectName(QString::fromUtf8("StateShowLabel"));
        StateShowLabel->setMinimumSize(QSize(0, 60));

        horizontalLayout_state->addWidget(StateShowLabel, 0, Qt::AlignHCenter);


        verticalLayout_data->addLayout(horizontalLayout_state);


        verticalLayout->addLayout(verticalLayout_data);

        SaveDataButton = new QPushButton(centralwidget);
        SaveDataButton->setObjectName(QString::fromUtf8("SaveDataButton"));
        SaveDataButton->setMaximumSize(QSize(1000, 16777215));

        verticalLayout->addWidget(SaveDataButton);

        ContinueDetectButton = new QPushButton(centralwidget);
        ContinueDetectButton->setObjectName(QString::fromUtf8("ContinueDetectButton"));
        ContinueDetectButton->setMaximumSize(QSize(1000, 16777215));

        verticalLayout->addWidget(ContinueDetectButton);


        horizontalLayout_2->addLayout(verticalLayout);


        verticalLayout_2->addLayout(horizontalLayout_2);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 290, 21));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "\345\217\230\347\224\265\350\256\276\345\244\207\345\270\246\347\224\265\347\212\266\346\200\201\346\243\200\346\265\213\350\275\257\344\273\266", nullptr));
        ResultTextLabel->setText(QApplication::translate("MainWindow", "\346\243\200\346\265\213\347\273\223\346\236\234", nullptr));
        WarnTextLabel->setText(QApplication::translate("MainWindow", "\350\257\267\345\234\250\345\233\276\347\211\207\344\270\212\345\215\225\345\207\273\344\270\200\344\270\252\350\257\206\345\210\253\346\241\206\344\273\245\345\274\200\345\247\213\346\243\200\346\265\213\357\274\201", nullptr));
        ResultImgShowLabel->setText(QApplication::translate("MainWindow", "result", nullptr));
        DataTextLabel->setText(QApplication::translate("MainWindow", "\346\243\200\346\265\213\346\225\260\346\215\256", nullptr));
        ChangeModelLabel->setText(QApplication::translate("MainWindow", "\345\210\207\346\215\242\346\250\241\345\236\213\357\274\232", nullptr));
        ModelListShowBox->setItemText(0, QApplication::translate("MainWindow", "3216.engine", nullptr));
        ModelListShowBox->setItemText(1, QApplication::translate("MainWindow", "536.engine", nullptr));

        FPSTextLabel->setText(QApplication::translate("MainWindow", "\345\275\223\345\211\215\345\270\247\347\216\207\357\274\232", nullptr));
        FPSShowLabel->setText(QApplication::translate("MainWindow", "60 FPS", nullptr));
        ChangeVLabel->setText(QApplication::translate("MainWindow", "\345\210\207\346\215\242\347\224\265\345\216\213\347\255\211\347\272\247\357\274\232", nullptr));
        VListShowBox->setItemText(0, QApplication::translate("MainWindow", "110kV", nullptr));
        VListShowBox->setItemText(1, QApplication::translate("MainWindow", "220kV", nullptr));
        VListShowBox->setItemText(2, QApplication::translate("MainWindow", "330kV", nullptr));
        VListShowBox->setItemText(3, QApplication::translate("MainWindow", "500kV", nullptr));

        ClassTextLabel->setText(QApplication::translate("MainWindow", "\351\200\211\344\270\255\350\256\276\345\244\207\347\261\273\345\210\253\357\274\232", nullptr));
        ClassShowLabel->setText(QApplication::translate("MainWindow", "\346\257\215\347\272\277", nullptr));
        ConfTextLabel->setText(QApplication::translate("MainWindow", "\347\275\256\344\277\241\345\272\246\357\274\232", nullptr));
        ConfShowLabel->setText(QApplication::translate("MainWindow", "89%", nullptr));
        PosTextLabel->setText(QApplication::translate("MainWindow", "\344\275\215\345\247\277\357\274\232", nullptr));
        PosShowLabel->setText(QApplication::translate("MainWindow", "3.8 m", nullptr));
        ElectricTextLabel->setText(QApplication::translate("MainWindow", "\347\224\265\345\234\272\345\274\272\345\272\246\357\274\232", nullptr));
        ElectricShowLabel->setText(QApplication::translate("MainWindow", "23.4 V/m", nullptr));
        StateTextLabel->setText(QApplication::translate("MainWindow", "\346\230\257\345\220\246\345\270\246\347\224\265\357\274\232", nullptr));
        StateShowLabel->setText(QApplication::translate("MainWindow", "\346\230\257", nullptr));
        SaveDataButton->setText(QApplication::translate("MainWindow", "\344\277\235\345\255\230\346\225\260\346\215\256", nullptr));
        ContinueDetectButton->setText(QApplication::translate("MainWindow", "\347\273\247\347\273\255\346\243\200\346\265\213", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
