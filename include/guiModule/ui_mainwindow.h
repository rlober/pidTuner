/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.0.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QCustomPlot *posPlot;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout_6;
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *verticalLayout_5;
    QLabel *label_4;
    QComboBox *partList;
    QVBoxLayout *verticalLayout_4;
    QLabel *label_5;
    QComboBox *jointList;
    QSpacerItem *verticalSpacer;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *previousJointButton;
    QPushButton *nextJointButton;
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout_7;
    QHBoxLayout *horizontalLayout_4;
    QVBoxLayout *verticalLayout;
    QLabel *kp_label;
    QLineEdit *kp_in;
    QVBoxLayout *verticalLayout_2;
    QLabel *kd_label;
    QLineEdit *kd_in;
    QVBoxLayout *verticalLayout_3;
    QLabel *ki_label;
    QLineEdit *ki_in;
    QPushButton *gainTestButton;
    QSpacerItem *horizontalSpacer;
    QPushButton *saveGainsButton;
    QWidget *layoutWidget2;
    QVBoxLayout *verticalLayout_8;
    QLabel *label;
    QHBoxLayout *horizontalLayout_5;
    QPushButton *posContButton;
    QPushButton *velContButton;
    QPushButton *torContButton;
    QWidget *layoutWidget3;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *closeButton;
    QPushButton *homeButton;
    QSpacerItem *horizontalSpacer_3;
    QMenuBar *menuBar;
    QMenu *menuPID_Tuner;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1161, 600);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        posPlot = new QCustomPlot(centralWidget);
        posPlot->setObjectName(QStringLiteral("posPlot"));
        posPlot->setGeometry(QRect(560, 0, 591, 401));
        layoutWidget = new QWidget(centralWidget);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(13, 30, 351, 121));
        verticalLayout_6 = new QVBoxLayout(layoutWidget);
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setContentsMargins(11, 11, 11, 11);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        verticalLayout_6->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        label_4 = new QLabel(layoutWidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setAlignment(Qt::AlignCenter);

        verticalLayout_5->addWidget(label_4);

        partList = new QComboBox(layoutWidget);
        partList->setObjectName(QStringLiteral("partList"));

        verticalLayout_5->addWidget(partList);


        horizontalLayout_3->addLayout(verticalLayout_5);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        label_5 = new QLabel(layoutWidget);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setAlignment(Qt::AlignCenter);

        verticalLayout_4->addWidget(label_5);

        jointList = new QComboBox(layoutWidget);
        jointList->setObjectName(QStringLiteral("jointList"));

        verticalLayout_4->addWidget(jointList);


        horizontalLayout_3->addLayout(verticalLayout_4);


        verticalLayout_6->addLayout(horizontalLayout_3);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_6->addItem(verticalSpacer);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        previousJointButton = new QPushButton(layoutWidget);
        previousJointButton->setObjectName(QStringLiteral("previousJointButton"));

        horizontalLayout_2->addWidget(previousJointButton);

        nextJointButton = new QPushButton(layoutWidget);
        nextJointButton->setObjectName(QStringLiteral("nextJointButton"));

        horizontalLayout_2->addWidget(nextJointButton);


        verticalLayout_6->addLayout(horizontalLayout_2);

        layoutWidget1 = new QWidget(centralWidget);
        layoutWidget1->setObjectName(QStringLiteral("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(10, 390, 529, 132));
        verticalLayout_7 = new QVBoxLayout(layoutWidget1);
        verticalLayout_7->setSpacing(6);
        verticalLayout_7->setContentsMargins(11, 11, 11, 11);
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        verticalLayout_7->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        kp_label = new QLabel(layoutWidget1);
        kp_label->setObjectName(QStringLiteral("kp_label"));
        kp_label->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(kp_label);

        kp_in = new QLineEdit(layoutWidget1);
        kp_in->setObjectName(QStringLiteral("kp_in"));

        verticalLayout->addWidget(kp_in);


        horizontalLayout_4->addLayout(verticalLayout);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        kd_label = new QLabel(layoutWidget1);
        kd_label->setObjectName(QStringLiteral("kd_label"));
        kd_label->setAlignment(Qt::AlignCenter);

        verticalLayout_2->addWidget(kd_label);

        kd_in = new QLineEdit(layoutWidget1);
        kd_in->setObjectName(QStringLiteral("kd_in"));

        verticalLayout_2->addWidget(kd_in);


        horizontalLayout_4->addLayout(verticalLayout_2);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        ki_label = new QLabel(layoutWidget1);
        ki_label->setObjectName(QStringLiteral("ki_label"));
        ki_label->setAlignment(Qt::AlignCenter);

        verticalLayout_3->addWidget(ki_label);

        ki_in = new QLineEdit(layoutWidget1);
        ki_in->setObjectName(QStringLiteral("ki_in"));

        verticalLayout_3->addWidget(ki_in);


        horizontalLayout_4->addLayout(verticalLayout_3);

        gainTestButton = new QPushButton(layoutWidget1);
        gainTestButton->setObjectName(QStringLiteral("gainTestButton"));
        gainTestButton->setMinimumSize(QSize(0, 54));

        horizontalLayout_4->addWidget(gainTestButton);


        verticalLayout_7->addLayout(horizontalLayout_4);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        verticalLayout_7->addItem(horizontalSpacer);

        saveGainsButton = new QPushButton(layoutWidget1);
        saveGainsButton->setObjectName(QStringLiteral("saveGainsButton"));

        verticalLayout_7->addWidget(saveGainsButton);

        layoutWidget2 = new QWidget(centralWidget);
        layoutWidget2->setObjectName(QStringLiteral("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(10, 200, 361, 101));
        verticalLayout_8 = new QVBoxLayout(layoutWidget2);
        verticalLayout_8->setSpacing(6);
        verticalLayout_8->setContentsMargins(11, 11, 11, 11);
        verticalLayout_8->setObjectName(QStringLiteral("verticalLayout_8"));
        verticalLayout_8->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(layoutWidget2);
        label->setObjectName(QStringLiteral("label"));
        QFont font;
        font.setPointSize(19);
        label->setFont(font);
        label->setAlignment(Qt::AlignCenter);

        verticalLayout_8->addWidget(label);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        posContButton = new QPushButton(layoutWidget2);
        posContButton->setObjectName(QStringLiteral("posContButton"));

        horizontalLayout_5->addWidget(posContButton);

        velContButton = new QPushButton(layoutWidget2);
        velContButton->setObjectName(QStringLiteral("velContButton"));

        horizontalLayout_5->addWidget(velContButton);

        torContButton = new QPushButton(layoutWidget2);
        torContButton->setObjectName(QStringLiteral("torContButton"));

        horizontalLayout_5->addWidget(torContButton);


        verticalLayout_8->addLayout(horizontalLayout_5);

        layoutWidget3 = new QWidget(centralWidget);
        layoutWidget3->setObjectName(QStringLiteral("layoutWidget3"));
        layoutWidget3->setGeometry(QRect(710, 430, 291, 41));
        horizontalLayout = new QHBoxLayout(layoutWidget3);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_2);

        closeButton = new QPushButton(layoutWidget3);
        closeButton->setObjectName(QStringLiteral("closeButton"));

        horizontalLayout->addWidget(closeButton);

        homeButton = new QPushButton(layoutWidget3);
        homeButton->setObjectName(QStringLiteral("homeButton"));

        horizontalLayout->addWidget(homeButton);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_3);

        MainWindow->setCentralWidget(centralWidget);
        layoutWidget->raise();
        layoutWidget->raise();
        layoutWidget->raise();
        layoutWidget->raise();
        posPlot->raise();
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1161, 25));
        menuPID_Tuner = new QMenu(menuBar);
        menuPID_Tuner->setObjectName(QStringLiteral("menuPID_Tuner"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuPID_Tuner->menuAction());

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        label_4->setText(QApplication::translate("MainWindow", "Robot Part", 0));
        label_5->setText(QApplication::translate("MainWindow", "Joint", 0));
        previousJointButton->setText(QApplication::translate("MainWindow", "Previous Joint", 0));
        nextJointButton->setText(QApplication::translate("MainWindow", "Next Joint", 0));
        kp_label->setText(QApplication::translate("MainWindow", "Kp", 0));
        kd_label->setText(QApplication::translate("MainWindow", "Kd", 0));
        ki_label->setText(QApplication::translate("MainWindow", "Ki", 0));
        gainTestButton->setText(QApplication::translate("MainWindow", "Test Gains", 0));
        saveGainsButton->setText(QApplication::translate("MainWindow", "Save Gains", 0));
        label->setText(QApplication::translate("MainWindow", "Control Mode", 0));
        posContButton->setText(QApplication::translate("MainWindow", "Position", 0));
        velContButton->setText(QApplication::translate("MainWindow", "Velocity", 0));
        torContButton->setText(QApplication::translate("MainWindow", "Torque", 0));
        closeButton->setText(QApplication::translate("MainWindow", "Close", 0));
        homeButton->setText(QApplication::translate("MainWindow", "Go To Home", 0));
        menuPID_Tuner->setTitle(QApplication::translate("MainWindow", "PID Tuner", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
