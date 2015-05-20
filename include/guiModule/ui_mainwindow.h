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
#include <QtWidgets/QGridLayout>
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
    QGridLayout *gridLayout;
    QCustomPlot *posPlot;
    QSpacerItem *verticalSpacer_3;
    QVBoxLayout *contModeLayout;
    QLabel *contModeLabel;
    QHBoxLayout *contModeButtonLayout;
    QPushButton *posContButton;
    QPushButton *velContButton;
    QPushButton *torContButton;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *statusLayout;
    QLabel *statusLabel;
    QLabel *statusInfoLabel;
    QSpacerItem *verticalSpacer;
    QHBoxLayout *closeHomeLayout;
    QSpacerItem *closeSpacer;
    QPushButton *closeButton;
    QPushButton *homeButton;
    QSpacerItem *homeSpacer;
    QSpacerItem *verticalSpacer_2;
    QVBoxLayout *jointSelectorLayout_2;
    QHBoxLayout *selectorComboBoxLayout;
    QVBoxLayout *partLayout;
    QLabel *partLabel;
    QComboBox *partList;
    QVBoxLayout *jointLayout;
    QLabel *jointLabel;
    QComboBox *jointList;
    QSpacerItem *jointButtonSpacer;
    QHBoxLayout *jointSelectorLayout;
    QPushButton *previousJointButton;
    QPushButton *nextJointButton;
    QVBoxLayout *gainsLayout;
    QHBoxLayout *gainInputLayout;
    QVBoxLayout *kpLayout;
    QLabel *kp_label;
    QLineEdit *kp_in;
    QVBoxLayout *kdLayout;
    QLabel *kd_label;
    QLineEdit *kd_in;
    QVBoxLayout *kiLayout;
    QLabel *ki_label;
    QLineEdit *ki_in;
    QPushButton *gainTestButton;
    QSpacerItem *verticalSpacer_6;
    QPushButton *gainResetButton;
    QSpacerItem *verticalSpacer_5;
    QPushButton *saveGainsButton;
    QSpacerItem *verticalSpacer_4;
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
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        posPlot = new QCustomPlot(centralWidget);
        posPlot->setObjectName(QStringLiteral("posPlot"));

        gridLayout->addWidget(posPlot, 0, 1, 5, 1);

        verticalSpacer_3 = new QSpacerItem(20, 80, QSizePolicy::Minimum, QSizePolicy::Maximum);

        gridLayout->addItem(verticalSpacer_3, 1, 0, 1, 1);

        contModeLayout = new QVBoxLayout();
        contModeLayout->setSpacing(6);
        contModeLayout->setObjectName(QStringLiteral("contModeLayout"));
        contModeLabel = new QLabel(centralWidget);
        contModeLabel->setObjectName(QStringLiteral("contModeLabel"));
        QFont font;
        font.setPointSize(19);
        contModeLabel->setFont(font);
        contModeLabel->setAlignment(Qt::AlignCenter);

        contModeLayout->addWidget(contModeLabel);

        contModeButtonLayout = new QHBoxLayout();
        contModeButtonLayout->setSpacing(6);
        contModeButtonLayout->setObjectName(QStringLiteral("contModeButtonLayout"));
        posContButton = new QPushButton(centralWidget);
        posContButton->setObjectName(QStringLiteral("posContButton"));
        posContButton->setCheckable(true);

        contModeButtonLayout->addWidget(posContButton);

        velContButton = new QPushButton(centralWidget);
        velContButton->setObjectName(QStringLiteral("velContButton"));
        velContButton->setCheckable(true);

        contModeButtonLayout->addWidget(velContButton);

        torContButton = new QPushButton(centralWidget);
        torContButton->setObjectName(QStringLiteral("torContButton"));
        torContButton->setCheckable(true);

        contModeButtonLayout->addWidget(torContButton);


        contModeLayout->addLayout(contModeButtonLayout);


        gridLayout->addLayout(contModeLayout, 2, 0, 1, 1);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        statusLayout = new QHBoxLayout();
        statusLayout->setSpacing(6);
        statusLayout->setObjectName(QStringLiteral("statusLayout"));
        statusLabel = new QLabel(centralWidget);
        statusLabel->setObjectName(QStringLiteral("statusLabel"));
        QFont font1;
        font1.setPointSize(14);
        statusLabel->setFont(font1);
        statusLabel->setAlignment(Qt::AlignCenter);

        statusLayout->addWidget(statusLabel);

        statusInfoLabel = new QLabel(centralWidget);
        statusInfoLabel->setObjectName(QStringLiteral("statusInfoLabel"));
        QFont font2;
        font2.setFamily(QStringLiteral("Ubuntu Mono"));
        font2.setPointSize(14);
        statusInfoLabel->setFont(font2);
        statusInfoLabel->setAlignment(Qt::AlignCenter);

        statusLayout->addWidget(statusInfoLabel);


        verticalLayout->addLayout(statusLayout);

        verticalSpacer = new QSpacerItem(20, 20, QSizePolicy::Minimum, QSizePolicy::Maximum);

        verticalLayout->addItem(verticalSpacer);

        closeHomeLayout = new QHBoxLayout();
        closeHomeLayout->setSpacing(6);
        closeHomeLayout->setObjectName(QStringLiteral("closeHomeLayout"));
        closeSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        closeHomeLayout->addItem(closeSpacer);

        closeButton = new QPushButton(centralWidget);
        closeButton->setObjectName(QStringLiteral("closeButton"));

        closeHomeLayout->addWidget(closeButton);

        homeButton = new QPushButton(centralWidget);
        homeButton->setObjectName(QStringLiteral("homeButton"));

        closeHomeLayout->addWidget(homeButton);

        homeSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        closeHomeLayout->addItem(homeSpacer);


        verticalLayout->addLayout(closeHomeLayout);


        gridLayout->addLayout(verticalLayout, 6, 1, 1, 1);

        verticalSpacer_2 = new QSpacerItem(20, 20, QSizePolicy::Minimum, QSizePolicy::Maximum);

        gridLayout->addItem(verticalSpacer_2, 5, 1, 1, 1);

        jointSelectorLayout_2 = new QVBoxLayout();
        jointSelectorLayout_2->setSpacing(6);
        jointSelectorLayout_2->setObjectName(QStringLiteral("jointSelectorLayout_2"));
        selectorComboBoxLayout = new QHBoxLayout();
        selectorComboBoxLayout->setSpacing(6);
        selectorComboBoxLayout->setObjectName(QStringLiteral("selectorComboBoxLayout"));
        partLayout = new QVBoxLayout();
        partLayout->setSpacing(6);
        partLayout->setObjectName(QStringLiteral("partLayout"));
        partLabel = new QLabel(centralWidget);
        partLabel->setObjectName(QStringLiteral("partLabel"));
        partLabel->setAlignment(Qt::AlignCenter);

        partLayout->addWidget(partLabel);

        partList = new QComboBox(centralWidget);
        partList->setObjectName(QStringLiteral("partList"));

        partLayout->addWidget(partList);


        selectorComboBoxLayout->addLayout(partLayout);

        jointLayout = new QVBoxLayout();
        jointLayout->setSpacing(6);
        jointLayout->setObjectName(QStringLiteral("jointLayout"));
        jointLabel = new QLabel(centralWidget);
        jointLabel->setObjectName(QStringLiteral("jointLabel"));
        jointLabel->setAlignment(Qt::AlignCenter);

        jointLayout->addWidget(jointLabel);

        jointList = new QComboBox(centralWidget);
        jointList->setObjectName(QStringLiteral("jointList"));

        jointLayout->addWidget(jointList);


        selectorComboBoxLayout->addLayout(jointLayout);


        jointSelectorLayout_2->addLayout(selectorComboBoxLayout);

        jointButtonSpacer = new QSpacerItem(20, 10, QSizePolicy::Minimum, QSizePolicy::Maximum);

        jointSelectorLayout_2->addItem(jointButtonSpacer);

        jointSelectorLayout = new QHBoxLayout();
        jointSelectorLayout->setSpacing(6);
        jointSelectorLayout->setObjectName(QStringLiteral("jointSelectorLayout"));
        previousJointButton = new QPushButton(centralWidget);
        previousJointButton->setObjectName(QStringLiteral("previousJointButton"));

        jointSelectorLayout->addWidget(previousJointButton);

        nextJointButton = new QPushButton(centralWidget);
        nextJointButton->setObjectName(QStringLiteral("nextJointButton"));

        jointSelectorLayout->addWidget(nextJointButton);


        jointSelectorLayout_2->addLayout(jointSelectorLayout);


        gridLayout->addLayout(jointSelectorLayout_2, 0, 0, 1, 1);

        gainsLayout = new QVBoxLayout();
        gainsLayout->setSpacing(6);
        gainsLayout->setObjectName(QStringLiteral("gainsLayout"));
        gainInputLayout = new QHBoxLayout();
        gainInputLayout->setSpacing(6);
        gainInputLayout->setObjectName(QStringLiteral("gainInputLayout"));
        kpLayout = new QVBoxLayout();
        kpLayout->setSpacing(6);
        kpLayout->setObjectName(QStringLiteral("kpLayout"));
        kp_label = new QLabel(centralWidget);
        kp_label->setObjectName(QStringLiteral("kp_label"));
        kp_label->setAlignment(Qt::AlignCenter);

        kpLayout->addWidget(kp_label);

        kp_in = new QLineEdit(centralWidget);
        kp_in->setObjectName(QStringLiteral("kp_in"));

        kpLayout->addWidget(kp_in);


        gainInputLayout->addLayout(kpLayout);

        kdLayout = new QVBoxLayout();
        kdLayout->setSpacing(6);
        kdLayout->setObjectName(QStringLiteral("kdLayout"));
        kd_label = new QLabel(centralWidget);
        kd_label->setObjectName(QStringLiteral("kd_label"));
        kd_label->setAlignment(Qt::AlignCenter);

        kdLayout->addWidget(kd_label);

        kd_in = new QLineEdit(centralWidget);
        kd_in->setObjectName(QStringLiteral("kd_in"));

        kdLayout->addWidget(kd_in);


        gainInputLayout->addLayout(kdLayout);

        kiLayout = new QVBoxLayout();
        kiLayout->setSpacing(6);
        kiLayout->setObjectName(QStringLiteral("kiLayout"));
        ki_label = new QLabel(centralWidget);
        ki_label->setObjectName(QStringLiteral("ki_label"));
        ki_label->setAlignment(Qt::AlignCenter);

        kiLayout->addWidget(ki_label);

        ki_in = new QLineEdit(centralWidget);
        ki_in->setObjectName(QStringLiteral("ki_in"));

        kiLayout->addWidget(ki_in);


        gainInputLayout->addLayout(kiLayout);

        gainTestButton = new QPushButton(centralWidget);
        gainTestButton->setObjectName(QStringLiteral("gainTestButton"));
        gainTestButton->setMinimumSize(QSize(0, 54));

        gainInputLayout->addWidget(gainTestButton);


        gainsLayout->addLayout(gainInputLayout);

        verticalSpacer_6 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gainsLayout->addItem(verticalSpacer_6);

        gainResetButton = new QPushButton(centralWidget);
        gainResetButton->setObjectName(QStringLiteral("gainResetButton"));

        gainsLayout->addWidget(gainResetButton, 0, Qt::AlignHCenter|Qt::AlignVCenter);

        verticalSpacer_5 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gainsLayout->addItem(verticalSpacer_5);

        saveGainsButton = new QPushButton(centralWidget);
        saveGainsButton->setObjectName(QStringLiteral("saveGainsButton"));
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy.setHorizontalStretch(200);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(saveGainsButton->sizePolicy().hasHeightForWidth());
        saveGainsButton->setSizePolicy(sizePolicy);
        saveGainsButton->setMinimumSize(QSize(300, 40));
        saveGainsButton->setMaximumSize(QSize(80000, 80));

        gainsLayout->addWidget(saveGainsButton, 0, Qt::AlignHCenter|Qt::AlignVCenter);


        gridLayout->addLayout(gainsLayout, 4, 0, 3, 1);

        verticalSpacer_4 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Maximum);

        gridLayout->addItem(verticalSpacer_4, 3, 0, 1, 1);

        MainWindow->setCentralWidget(centralWidget);
        posPlot->raise();
        statusLabel->raise();
        statusInfoLabel->raise();
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
        contModeLabel->setText(QApplication::translate("MainWindow", "Control Mode", 0));
        posContButton->setText(QApplication::translate("MainWindow", "Position", 0));
        velContButton->setText(QApplication::translate("MainWindow", "Velocity", 0));
        torContButton->setText(QApplication::translate("MainWindow", "Torque", 0));
        statusLabel->setText(QApplication::translate("MainWindow", "Status:", 0));
        statusInfoLabel->setText(QApplication::translate("MainWindow", "status_info", 0));
        closeButton->setText(QApplication::translate("MainWindow", "Close", 0));
        homeButton->setText(QApplication::translate("MainWindow", "Go To Home", 0));
        partLabel->setText(QApplication::translate("MainWindow", "Robot Part", 0));
        jointLabel->setText(QApplication::translate("MainWindow", "Joint", 0));
        previousJointButton->setText(QApplication::translate("MainWindow", "Previous Joint", 0));
        nextJointButton->setText(QApplication::translate("MainWindow", "Next Joint", 0));
        kp_label->setText(QApplication::translate("MainWindow", "Kp", 0));
        kd_label->setText(QApplication::translate("MainWindow", "Kd", 0));
        ki_label->setText(QApplication::translate("MainWindow", "Ki", 0));
        gainTestButton->setText(QApplication::translate("MainWindow", "Test Gains", 0));
        gainResetButton->setText(QApplication::translate("MainWindow", "Reset Gains", 0));
        saveGainsButton->setText(QApplication::translate("MainWindow", "Save Gains", 0));
        menuPID_Tuner->setTitle(QApplication::translate("MainWindow", "PID Tuner", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
