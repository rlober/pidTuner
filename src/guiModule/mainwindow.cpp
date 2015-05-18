#include "mainwindow.h"
#include "ui_mainwindow.h"
//Foo *fooObject_,
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)//,
    //fooObject(fooObject_)
{
    ui->setupUi(this);
    createPlot();
    addPartsToList();
    controlType = "position";
    yPlotLabel = "y";
    isOnlyMajorJoints = true;
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::createPlot()
{
    // create graph and assign data to it:
    ui->posPlot->addGraph();
    ui->posPlot->graph(0)->setName("Input");
    ui->posPlot->addGraph();
    ui->posPlot->graph(1)->setName("Response");
    // give the axes some labels:
    ui->posPlot->xAxis->setLabel("ms");
    ui->posPlot->yAxis->setLabel(yPlotLabel);
    // set axes ranges, so we see all data:
    ui->posPlot->xAxis->setRange(-1, 1);
    ui->posPlot->yAxis->setRange(0, 1);

    QPen inputPen;
    inputPen.setColor(Qt::blue);
    inputPen.setWidthF(2);
    ui->posPlot->graph(0)->setPen(inputPen);

    QPen responsePen;
    responsePen.setColor(Qt::red);
    responsePen.setWidthF(2);
    ui->posPlot->graph(1)->setPen(responsePen);

}

void MainWindow::addPartsToList()
{
    ui->partList->addItem("head");
    ui->partList->addItem("torso");
    ui->partList->addItem("left_arm");
    ui->partList->addItem("right_arm");
    ui->partList->addItem("left_leg");
    ui->partList->addItem("right_leg");
}

void MainWindow::on_gainTestButton_clicked()
{
    // generate some data:
    QVector<double> x(101), y(101); // initialize with entries 0..100
    for (int i=0; i<101; ++i)
    {
      x[i] = i/50.0 - 1; // x goes from -1 to 1
      y[i] = x[i]*x[i]; // let's plot a quadratic function
    }
    ui->posPlot->graph(0)->setData(x, y);
    ui->posPlot->graph(1)->setData(y, x);
    ui->posPlot->replot();
}

void MainWindow::resetYLabel()
{
    ui->posPlot->yAxis->setLabel(yPlotLabel);
    ui->posPlot->replot();
}


void MainWindow::on_partList_currentIndexChanged(int partIndex)
{
    ui->jointList->clear();
    switch(partIndex)
    {
        case 0:
            ui->jointList->addItem("0 - neck_pitch");
            ui->jointList->addItem("1 - neck_roll");
            ui->jointList->addItem("2 - neck_yaw");
            if(!isOnlyMajorJoints){
            ui->jointList->addItem("3 - eyes_tilt");
            ui->jointList->addItem("4 - eyes_version");
            ui->jointList->addItem("5 - eyes_vergence");
            }
        break;

        case 1:
            ui->jointList->addItem("0 - torso_yaw");
            ui->jointList->addItem("1 - torso_roll");
            ui->jointList->addItem("2 - torso_pitch");
        break;

        case 2:
            ui->jointList->addItem("0 - l_shoulder_pitch");
            ui->jointList->addItem("1 - l_shoulder_roll");
            ui->jointList->addItem("2 - l_shoulder_yaw");
            ui->jointList->addItem("3 - l_elbow");
            ui->jointList->addItem("4 - l_wrist_prosup");
            ui->jointList->addItem("5 - l_wrist_pitch");
            ui->jointList->addItem("6 - l_wrist_yaw");
            if(!isOnlyMajorJoints){
            ui->jointList->addItem("7 - l_hand_finger");
            ui->jointList->addItem("8 - l_thumb_oppose");
            ui->jointList->addItem("9 - l_thumb_proximal");
            ui->jointList->addItem("10 - l_thumb_distal");
            ui->jointList->addItem("11 - l_index_proximal");
            ui->jointList->addItem("12 - l_index_distal");
            ui->jointList->addItem("13 - l_middle_proximal");
            ui->jointList->addItem("14 - l_middle_distal");
            ui->jointList->addItem("15 - l_pinky");
            }
        break;

        case 3:
            ui->jointList->addItem("0 - r_shoulder_pitch");
            ui->jointList->addItem("1 - r_shoulder_roll");
            ui->jointList->addItem("2 - r_shoulder_yaw");
            ui->jointList->addItem("3 - r_elbow");
            ui->jointList->addItem("4 - r_wrist_prosup");
            ui->jointList->addItem("5 - r_wrist_pitch");
            ui->jointList->addItem("6 - r_wrist_yaw");
            if(!isOnlyMajorJoints){
            ui->jointList->addItem("7 - r_hand_finger");
            ui->jointList->addItem("8 - r_thumb_oppose");
            ui->jointList->addItem("9 - r_thumb_proximal");
            ui->jointList->addItem("10 - r_thumb_distal");
            ui->jointList->addItem("11 - r_index_proximal");
            ui->jointList->addItem("12 - r_index_distal");
            ui->jointList->addItem("13 - r_middle_proximal");
            ui->jointList->addItem("14 - r_middle_distal");
            ui->jointList->addItem("15 - r_pinky");
            }
        break;

        case 4:
            ui->jointList->addItem("0 - l_hip_pitch");
            ui->jointList->addItem("1 - l_hip_roll");
            ui->jointList->addItem("2 - l_hip_yaw");
            ui->jointList->addItem("3 - l_knee");
            ui->jointList->addItem("4 - l_ankle_pitch");
            ui->jointList->addItem("5 - l_ankle_roll");
        break;

        case 5:
            ui->jointList->addItem("0 - r_hip_pitch");
            ui->jointList->addItem("1 - r_hip_roll");
            ui->jointList->addItem("2 - r_hip_yaw");
            ui->jointList->addItem("3 - r_knee");
            ui->jointList->addItem("4 - r_ankle_pitch");
            ui->jointList->addItem("5 - r_ankle_roll");
        break;
    }
}

void MainWindow::on_posContButton_clicked()
{
    controlType = "position";
    yPlotLabel = "q (deg)";
    resetYLabel();
}

void MainWindow::on_velContButton_clicked()
{
    controlType = "velocity";
    yPlotLabel = "dq (deg/sec)";
    resetYLabel();
}

void MainWindow::on_torContButton_clicked()
{
    controlType = "torque";
    yPlotLabel = "tau (Nm)";
    resetYLabel();
}




void MainWindow::on_closeButton_clicked()
{
    close();
}

void MainWindow::on_homeButton_clicked()
{
    //fooObject->bar();
}

void MainWindow::on_nextJointButton_clicked()
{

}
