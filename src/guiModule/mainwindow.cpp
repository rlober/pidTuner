#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>

using namespace yarp::os;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    createPlot();
    addPartsToList();
    controlType = "position";
    yPlotLabel = "y";
    isOnlyMajorJoints = true;
    gainsHaveBeenChanged = false;
    Kp_old=Kd_old=Ki_old=1.0;
    Kp_new=Kd_new=Ki_new=2.0;

    ///////////////////////////////////
    gainsBufPort_out.open("/pidTunerGui/gains/out");
    gainsPort_in.open("/pidTunerGui/gains/in");

    goToHomeBufPort_out.open("/pidTunerGui/goToHome/out");

    robotPartAndJointBufPort_out.open("/pidTunerGui/partAndJointIndexes/out");

    ///////////////////////////////////
    while(!yarp.connect("/pidTunerGui/gains/out", "/pidTunerController/gains/in") ){Time::delay(0.1);}
    while(!yarp.connect("/pidTunerController/gains/out", "/pidTunerGui/gains/in") ){Time::delay(0.1);}

    while(!yarp.connect("/pidTunerGui/goToHome/out", "/pidTunerController/goToHome/in") ){Time::delay(0.1);}

    while(!yarp.connect("/pidTunerGui/partAndJointIndexes/out", "/pidTunerController/partAndJointIndexes/in") ){Time::delay(0.1);}

    initializeGui();

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setCurrentPartAndJoint()
{
    ui->partList->setCurrentIndex(partIndex);
    ui->jointList->setCurrentIndex(jointIndex);
    // qDebug()<< "sent a message1";
    // setCurrentPartAndJoint();
}

void MainWindow::initializeGui()
{

    jointIndex = 0;
    partIndex = 0;
    setCurrentPartAndJoint();

    jointIndex = 1;
    partIndex = 1;
    setCurrentPartAndJoint();

    partIndex=ui->partList->count()-1;
    ui->partList->setCurrentIndex(partIndex);
    jointIndex = ui->jointList->count()-1;
    setCurrentPartAndJoint();

    ui->posContButton->setChecked(true);
    ui->statusInfoLabel->setText("ready");

    getPidGains();
    refreshGainDisplays();
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
    setPidGains();
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

    gainsHaveBeenChanged = true;

}

void MainWindow::resetYLabel()
{
    ui->posPlot->yAxis->setLabel(yPlotLabel);
    ui->posPlot->replot();
}


void MainWindow::on_partList_currentIndexChanged(int partId)
{
    ui->jointList->clear();
    jointIndex = 0;
    partIndex = partId;
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

    // sendPartAndJointIndexes();
}

// void MainWindow::on_jointList_currentIndexChanged(int jointId)
// {
//     jointIndex = jointId;
//     sendPartAndJointIndexes();
// }






void MainWindow::on_closeButton_clicked()
{
    close();
}

void MainWindow::on_homeButton_clicked()
{
    Bottle& goToHomeBottle_out = goToHomeBufPort_out.prepare();
    goToHomeBottle_out.clear();
    goToHomeBottle_out.addInt(1); // tells the receiver to go to home config
    goToHomeBufPort_out.write();
}

void MainWindow::on_previousJointButton_clicked()
{
    if(discardChanges())
    {
        int nJoints = ui->jointList->count();
        int numRobotParts = ui->partList->count();
        jointIndex++;

        if (jointIndex==nJoints)
        {jointIndex = 0; partIndex++;}

        if(partIndex==numRobotParts)
        {jointIndex=0; partIndex=0;}

        setCurrentPartAndJoint();

    }
}

void MainWindow::on_nextJointButton_clicked()
{
    if(discardChanges())
    {
        int numRobotParts = ui->partList->count();
        jointIndex--;

        if (jointIndex<0)
        {
            partIndex--;
            ui->partList->setCurrentIndex(partIndex);
            jointIndex = ui->jointList->count()-1;
        }

        if(partIndex<0)
        {
            jointIndex=0;
            partIndex=numRobotParts-1;
            ui->partList->setCurrentIndex(partIndex);
            jointIndex = ui->jointList->count()-1;
        }


        setCurrentPartAndJoint();
    }
}


void MainWindow::on_posContButton_clicked(bool checked)
{
    if(discardChanges()){
        if (checked){
            controlType = "position";
            yPlotLabel = "q (deg)";
            resetYLabel();
            ui->velContButton->setChecked(false);
            ui->torContButton->setChecked(false);
        }
    }
}


void MainWindow::on_velContButton_clicked(bool checked)
{
    if(discardChanges()){
        if (checked){
            controlType = "velocity";
            yPlotLabel = "dq (deg/sec)";
            resetYLabel();
            ui->posContButton->setChecked(false);
            ui->torContButton->setChecked(false);
        }
    }
    else{
        ui->velContButton->setChecked(false);
    }
}

void MainWindow::on_torContButton_clicked(bool checked)
{
    if(discardChanges()){
        if (checked){
            controlType = "torque";
            yPlotLabel = "tau (Nm)";
            resetYLabel();
            ui->posContButton->setChecked(false);
            ui->velContButton->setChecked(false);
        }
    }
}


void MainWindow::on_kp_in_editingFinished()
{
    bool ok;
    Kp_new = ui->kp_in->text().toDouble(&ok);
    if(!ok){
        ui->kp_in->setStyleSheet("QLineEdit { background: rgb(255, 0, 0)}");
        int ret = QMessageBox::warning(this, tr("Warning"),
                                       tr("The gains must be a number. Try again dummy."),
                                       QMessageBox::Ok);
        if(!ui->kp_in->hasFocus())
            ui->kp_in->setFocus();

        ui->kp_in->selectAll();

    }
    else
        ui->kp_in->setStyleSheet("QLineEdit { background: rgb(255, 255, 255)}");


}

void MainWindow::on_kd_in_editingFinished()
{
    bool ok;
    Kd_new = ui->kd_in->text().toDouble(&ok);
    if(!ok){
        ui->kd_in->setStyleSheet("QLineEdit { background: rgb(255, 0, 0)}");
        int ret = QMessageBox::warning(this, tr("Warning"),
                                       tr("The gains must be a number. Try again dummy."),
                                       QMessageBox::Ok);
        if(!ui->kd_in->hasFocus())
            ui->kd_in->setFocus();

        ui->kd_in->selectAll();

    }
    else
        ui->kd_in->setStyleSheet("QLineEdit { background: rgb(255, 255, 255)}");
}

void MainWindow::on_ki_in_editingFinished()
{
    bool ok;
    Ki_new = ui->ki_in->text().toDouble(&ok);
    if(!ok){
        ui->ki_in->setStyleSheet("QLineEdit { background: rgb(255, 0, 0)}");
        int ret = QMessageBox::warning(this, tr("Warning"),
                                       tr("The gains must be a number. Try again dummy."),
                                       QMessageBox::Ok);
        if(!ui->ki_in->hasFocus())
            ui->ki_in->setFocus();

        ui->ki_in->selectAll();

    }
    else
        ui->ki_in->setStyleSheet("QLineEdit { background: rgb(255, 255, 255)}");
}

void MainWindow::on_saveGainsButton_clicked()
{
    saveGains();
}


void MainWindow::on_gainResetButton_clicked()
{
    Kp_new = Kp_old; Kd_new = Kd_old; Ki_new = Ki_old;
    setPidGains();

}

void MainWindow::refreshGainDisplays()
{
    ui->kp_in->setText(QString::number(Kp_new));
    ui->kd_in->setText(QString::number(Kd_new));
    ui->ki_in->setText(QString::number(Ki_new));
}


bool MainWindow::discardChanges()
{
    if(gainsHaveBeenChanged)
    {
        QMessageBox msgBox;
        msgBox.setText("The gains for "+ui->partList->currentText()+" ("+ui->jointList->currentText()+") has been modified.");
        msgBox.setInformativeText("Do you want to save your changes?");
        msgBox.setStandardButtons(QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Save);
        int ret = msgBox.exec();

        switch (ret) {
          case QMessageBox::Save:
              qDebug()<<"save dat shit";
              saveGains();
              return false;
              break;
          case QMessageBox::Discard:
              qDebug()<<"discard dat shit";
              gainsHaveBeenChanged=false;
              return true;
              break;
          case QMessageBox::Cancel:
              qDebug()<<"cancel dat shit";
              return false;
              break;
          default:
              qDebug()<<"what da fuck";
              return false;
              break;
        }
    }
    else{return true;}

}

void MainWindow::saveGains()
{
    qDebug()<<"saved";
    gainsHaveBeenChanged=false;
}

void MainWindow::on_partList_highlighted(int index)
{
    if(discardChanges())
        qDebug()<<"highlighted";
}



void MainWindow::on_jointList_highlighted(int index)
{
    if(discardChanges())
        qDebug()<<"highlighted";
}


bool MainWindow::getPidGains()
{
    Bottle& gainsBottle_out = gainsBufPort_out.prepare(); // Get a place to store things.
    gainsBottle_out.clear();
    gainsBottle_out.addInt(0); // tells the receiver to just send the current gains
    gainsBufPort_out.write();

    Bottle controllerResponse;
    while(!gainsPort_in.read(controllerResponse))
    {
        Time::delay(0.001);
    }
    std::cout<< "Kp = " << controllerResponse.get(0).asDouble() << " Kd = " << controllerResponse.get(1).asDouble() << " Ki = " << controllerResponse.get(2).asDouble() << std::endl;
    Kp_new = controllerResponse.get(0).asDouble();
    Kd_new = controllerResponse.get(1).asDouble();
    Ki_new = controllerResponse.get(2).asDouble();

    refreshGainDisplays();
    return true;
}

bool MainWindow::setPidGains()
{
    // kpBufPort_out

    Bottle& gainsBottle_out = gainsBufPort_out.prepare(); // Get a place to store things.
    gainsBottle_out.clear();
    gainsBottle_out.addInt(1); // tells the receiver that new gains are comming and should be set
    gainsBottle_out.addDouble(Kp_new);
    gainsBottle_out.addDouble(Kd_new);
    gainsBottle_out.addDouble(Ki_new);
    gainsBufPort_out.write();

    Bottle controllerResponse;
    while(!gainsPort_in.read(controllerResponse))
    {
        Time::delay(0.001);
    }
    std::cout<< "Kp = " << controllerResponse.get(0).asDouble() << " Kd = " << controllerResponse.get(1).asDouble() << " Ki = " << controllerResponse.get(2).asDouble() << std::endl;
    Kp_new = controllerResponse.get(0).asDouble();
    Kd_new = controllerResponse.get(1).asDouble();
    Ki_new = controllerResponse.get(2).asDouble();

    refreshGainDisplays();
    return true;
}


void MainWindow::sendPartAndJointIndexes()
{

    Bottle& partAndJointIndexesBottle_out = robotPartAndJointBufPort_out.prepare(); // Get a place to store things.
    partAndJointIndexesBottle_out.clear();
    partAndJointIndexesBottle_out.addInt(partIndex);
    partAndJointIndexesBottle_out.addInt(jointIndex);
    qDebug()<< "sent a message";

    robotPartAndJointBufPort_out.write();


    getPidGains();


}
