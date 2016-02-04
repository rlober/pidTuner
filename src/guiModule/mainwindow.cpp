/*!
*  \file       mainwindow.cpp
*  \brief      MainWindow class implementation. This file implements the
               pidTunerGui module which communicates with pidTunerController
               and allows the user to modify PIDs and visualize their
               performance.
*  \author     Ryan Lober
*  \email      ryan.lober@isir.upmc.com
*  \version    1.0
*  \date       28 May 2015
*  \copyright  GNU Public License.
*
* This file is part of pidTuner, a set of modules for manual
* tuning of individual joint low level gains on the iCub robot.
* For more details please look at the README or check out the
* Github repository: https://github.com/rlober/pidTuner.git
*
* pidTuner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 2 of the License, or
* (at your option) any later version.
*
* pidTuner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with pidTuner.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "guiModule/mainwindow.h"
#include "ui_mainwindow.h"


using namespace yarp::os;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    testControlMode(POSITION_MODE),
    isOnlyMajorJoints(true),
    gainsHaveBeenChanged(false)
{
    signalAmplitude_POS_DEFAULT = 0.5;
    signalStartTime_POS_DEFAULT = 0.5;
    signalDuration_POS_DEFAULT = 1.5;

    signalAmplitude_VEL_DEFAULT = 0.1;
    signalStartTime_VEL_DEFAULT = 0.1;
    signalDuration_VEL_DEFAULT = 0.5;

    signalAmplitude_TOR_DEFAULT = 0.05;
    signalStartTime_TOR_DEFAULT = 0.1;
    signalDuration_TOR_DEFAULT = 0.5;


    partsListVector.push_back("head");
    partsListVector.push_back("torso");
    partsListVector.push_back("left_arm");
    partsListVector.push_back("right_arm");
    partsListVector.push_back("left_leg");
    partsListVector.push_back("right_leg");



}

MainWindow::~MainWindow()
{
    //delete ui;
}


bool MainWindow::init(ResourceFinder& rf)
{
    boost::thread init_th = boost::thread(boost::bind(&MainWindow::initialize, this,rf));
    if(! init_th.try_join_for(boost::chrono::milliseconds(2000)))
    {
        log.error() << " Timeout at init, please check that pidTunerController is launched";
        return false;
    }
    initFinished = false;

    log.info() << "Setting up Qt Ui.";
    ui->setupUi(this);

    log.info() << "Setting signal properties to default values.";
    setSignalPropertiesToDefaults();

    log.info() << "Initializing GUI.";
    initializeGui();

    log.info() << "Getting PID Gains.";
    getPidValues();

    log.info() << "Creating data logs.";
    createDataLogs();


    initFinished = true;

    log.info() << "Initialization finished!";
    return true;
}

bool MainWindow::initialize(ResourceFinder& rf)
{
    log.info()<< " Initializing connections ";
    if( rf.check("exclude") )
    {
        std::string excludedPart = rf.find("exclude").asString().c_str();

        if (excludedPart.size()!=0) {
            for(int i=0; i<partsListVector.size(); i++){
                if (excludedPart == partsListVector[i]){
                    doExcludePart = true;
                    excludedPartIndex = i;
                }
            }
        }else{doExcludePart = false; excludedPartIndex = 1000;}
    }


    dataPort_in.open("/pidTunerGui/data:i");
    rpcClientPort.open("/pidTunerGui/rpc:c");


    log.info() << "Connecting ports.";

    while(!yarp.connect("/pidTunerController/data:o", "/pidTunerGui/data:i") ){Time::delay(0.1);}
    while(!yarp.connect("/pidTunerGui/rpc:c", "/pidTunerController/rpc:s") ){Time::delay(0.1);}

    log.info() << "Ports connected.";
    return true;
}

void MainWindow::setCurrentPartAndJoint()
{
    ui->partList->setCurrentIndex(partIndex);
    ui->jointList->setCurrentIndex(jointIndex);
    sendPartAndJointIndexes();

    getPidValues();
    originalPid = newPid;
}

void MainWindow::initializeGui()
{


    addPartsToList();

    partIndex=ui->partList->count()-1;
    ui->partList->setCurrentIndex(partIndex);
    jointIndex = ui->jointList->count()-1;
    setCurrentPartAndJoint();

    ui->posContButton->click();
    ui->statusInfoLabel->setText("ready");

    // ui->posPlotdGains();
    createPlot();


}


void MainWindow::createPlot()
{
    // create graph and assign data to it:
    ui->posPlot->addGraph();
    ui->posPlot->graph(0)->setName("Input");
    ui->posPlot->addGraph();
    ui->posPlot->graph(1)->setName("Response");
    // give the axes some labels:
    ui->posPlot->xAxis->setLabel("time (s)");
    ui->posPlot->yAxis->setLabel(yPlotLabel);
    // set axes ranges, so we see all data:
    // ui->posPlot->xAxis->setRange(-1, 1);
    // ui->posPlot->yAxis->setRange(0, 1);
    // add title layout element:
    ui->posPlot->plotLayout()->insertRow(0);
    plotTitle.reset( new QCPPlotTitle(ui->posPlot, ui->jointList->currentText()));
    ui->posPlot->plotLayout()->addElement(0, 0, plotTitle.get());
    QFont titleFont= font();
    plotTitle->setFont(titleFont);




    QPen inputPen;
    inputPen.setColor(Qt::blue);
    inputPen.setWidthF(2);
    ui->posPlot->graph(0)->setPen(inputPen);

    QPen responsePen;
    responsePen.setColor(Qt::red);
    responsePen.setWidthF(2);
    ui->posPlot->graph(1)->setPen(responsePen);

    ui->posPlot->legend->setVisible(true);
    QFont legendFont = font();  // start out with MainWindow's font..
    legendFont.setPointSize(9); // and make a bit smaller for legend
    ui->posPlot->legend->setFont(legendFont);
    ui->posPlot->legend->setBrush(QBrush(QColor(255,255,255,230)));
    // by default, the legend is in the inset layout of the main axis rect. So this is how we access it to change legend placement:
    ui->posPlot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);

}

void MainWindow::addPartsToList()
{
    ui->partList->addItem("0-head");
    ui->partList->addItem("1-torso");
    ui->partList->addItem("2-left_arm");
    ui->partList->addItem("3-right_arm");
    ui->partList->addItem("4-left_leg");
    ui->partList->addItem("5-right_leg");

    if(doExcludePart){
        ui->partList->removeItem(excludedPartIndex);
    }
}

void MainWindow::on_gainTestButton_clicked()
{
    setPidValues();
    /*  To get a vector using yarp ports we have to use a little workaround basically
        the data is saved in a yarp vector  then when we are ready to send each entry
        is sent individually. The first value is an int (0 or 1) which indicates to
        the receiver when to listen. 0 = stop listening and 1 = start. The size of the
        vector is inferred from the number of messages sent with a first value of 1.
    */

    Bottle dataFromController;
    yarp::sig::Vector y_Time, y_Input, y_Response;
    log.info() << "Awaiting test data.";
    while(1)
    {
        if(dataPort_in.read(dataFromController))
        {
            if(dataFromController.get(0).asInt())
            {
                y_Time.push_back(dataFromController.get(1).asDouble());
                y_Input.push_back(dataFromController.get(2).asDouble());
                y_Response.push_back(dataFromController.get(3).asDouble());
            }else
                break;

        }

    }
    size_t vecLength = y_Time.size();
    if(!vecLength) return;

    log.info() << "Received" << vecLength << "data points.";

    QVector<double> q_Time(vecLength), q_Input(vecLength), q_Response(vecLength); // initialize with entries 0..100
    for (int i=0; i<vecLength; i++)
    {
        q_Time[i]       = y_Time[i];
        q_Input[i]      = y_Input[i];
        q_Response[i]   = y_Response[i];
    }


    double maxTime, yMin, yMax;

    maxTime = y_Time[vecLength-1];
    yMin = ( yarp::math::findMin(y_Input) < yarp::math::findMin(y_Response) ? yarp::math::findMin(y_Input) : yarp::math::findMin(y_Response) );
    yMax = ( yarp::math::findMax(y_Input) > yarp::math::findMax(y_Response) ? yarp::math::findMax(y_Input) : yarp::math::findMax(y_Response) );

    double axisScale = std::abs( (std::abs(yMax) -std::abs(yMin)) *0.1);
    yMin -= axisScale;
    yMax += axisScale;

    ui->posPlot->xAxis->setRange(0, maxTime);
    ui->posPlot->yAxis->setRange(yMin, yMax);
    ui->posPlot->graph(0)->setData(q_Time, q_Input);
    ui->posPlot->graph(1)->setData(q_Time, q_Response);
    resetYLabel();
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
    partIndex = partId;
    switch(partIndex)
    {
        case 0:
            ui->jointList->addItem("0-neck_pitch");
            ui->jointList->addItem("1-neck_roll");
            ui->jointList->addItem("2-neck_yaw");
            if(!isOnlyMajorJoints){
            ui->jointList->addItem("3-eyes_tilt");
            ui->jointList->addItem("4-eyes_version");
            ui->jointList->addItem("5-eyes_vergence");
            }
        break;

        case 1:
            ui->jointList->addItem("0-torso_yaw");
            ui->jointList->addItem("1-torso_roll");
            ui->jointList->addItem("2-torso_pitch");
        break;

        case 2:
            ui->jointList->addItem("0-l_shoulder_pitch");
            ui->jointList->addItem("1-l_shoulder_roll");
            ui->jointList->addItem("2-l_shoulder_yaw");
            ui->jointList->addItem("3-l_elbow");
            ui->jointList->addItem("4-l_wrist_prosup");
            ui->jointList->addItem("5-l_wrist_pitch");
            ui->jointList->addItem("6-l_wrist_yaw");
            if(!isOnlyMajorJoints){
            ui->jointList->addItem("7-l_hand_finger");
            ui->jointList->addItem("8-l_thumb_oppose");
            ui->jointList->addItem("9-l_thumb_proximal");
            ui->jointList->addItem("10-l_thumb_distal");
            ui->jointList->addItem("11-l_index_proximal");
            ui->jointList->addItem("12-l_index_distal");
            ui->jointList->addItem("13-l_middle_proximal");
            ui->jointList->addItem("14-l_middle_distal");
            ui->jointList->addItem("15-l_pinky");
            }
        break;

        case 3:
            ui->jointList->addItem("0-r_shoulder_pitch");
            ui->jointList->addItem("1-r_shoulder_roll");
            ui->jointList->addItem("2-r_shoulder_yaw");
            ui->jointList->addItem("3-r_elbow");
            ui->jointList->addItem("4-r_wrist_prosup");
            ui->jointList->addItem("5-r_wrist_pitch");
            ui->jointList->addItem("6-r_wrist_yaw");
            if(!isOnlyMajorJoints){
            ui->jointList->addItem("7-r_hand_finger");
            ui->jointList->addItem("8-r_thumb_oppose");
            ui->jointList->addItem("9-r_thumb_proximal");
            ui->jointList->addItem("10-r_thumb_distal");
            ui->jointList->addItem("11-r_index_proximal");
            ui->jointList->addItem("12-r_index_distal");
            ui->jointList->addItem("13-r_middle_proximal");
            ui->jointList->addItem("14-r_middle_distal");
            ui->jointList->addItem("15-r_pinky");
            }
        break;

        case 4:
            ui->jointList->addItem("0-l_hip_pitch");
            ui->jointList->addItem("1-l_hip_roll");
            ui->jointList->addItem("2-l_hip_yaw");
            ui->jointList->addItem("3-l_knee");
            ui->jointList->addItem("4-l_ankle_pitch");
            ui->jointList->addItem("5-l_ankle_roll");
        break;

        case 5:
            ui->jointList->addItem("0-r_hip_pitch");
            ui->jointList->addItem("1-r_hip_roll");
            ui->jointList->addItem("2-r_hip_yaw");
            ui->jointList->addItem("3-r_knee");
            ui->jointList->addItem("4-r_ankle_pitch");
            ui->jointList->addItem("5-r_ankle_roll");
        break;
    }


    jointIndex = ui->jointList->count()-1;
    ui->jointList->setCurrentIndex(jointIndex);

}

void MainWindow::on_jointList_currentIndexChanged(int jointId)
{
    jointIndex = jointId;
    if (initFinished && jointId>=0){
        setCurrentPartAndJoint();
        plotTitle->setText(ui->jointList->currentText());
        ui->posPlot->replot();
        log.info() << "Joint Index has been changed.";
    }

}






void MainWindow::on_closeButton_clicked()
{
    close();
}

void MainWindow::on_homeButton_clicked()
{
    Bottle send, reply;
    send.addInt(GO_TO_HOME_POSTURE);
    rpcClientPort.write(send,reply);
    if(!reply.get(0).asInt()){
        log.error() << "Could not send go to home posture command.";
    }
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

                if(partIndex<0)
                {
                    partIndex=numRobotParts-1;
                    ui->partList->setCurrentIndex(partIndex);
                }
                else
                {
                    ui->partList->setCurrentIndex(partIndex);
                }

            jointIndex = ui->jointList->count()-1;
        }


        ui->jointList->setCurrentIndex(jointIndex);

        setCurrentPartAndJoint();
    }
}

void MainWindow::disableTorqueParamInputs()
{
    ui->bemf_in->setEnabled(false);
    ui->bemf_scale_in->setEnabled(false);
    ui->coulombVelThresh_in->setEnabled(false);
    ui->frictionCompensation_in->setEnabled(false);
    ui->ktau_in->setEnabled(false);
    ui->ktau_scale_in->setEnabled(false);
}

void MainWindow::enableTorqueParamInputs()
{
    ui->bemf_in->setEnabled(true);
    ui->bemf_scale_in->setEnabled(true);
    ui->coulombVelThresh_in->setEnabled(true);
    ui->frictionCompensation_in->setEnabled(true);
    ui->ktau_in->setEnabled(true);
    ui->ktau_scale_in->setEnabled(true);
}

void MainWindow::on_posContButton_clicked(bool checked)
{
    if(discardChanges()){
        if (checked){
            testControlMode = POSITION_MODE;
            controlMode_string = "position";
            sendControlMode();
            yPlotLabel = "q (deg)";
            resetYLabel();
            ui->velContButton->setChecked(false);
            ui->torContButton->setChecked(false);

            signalAmplitude = signalAmplitude_POS;
            signalStartTime = signalStartTime_POS;
            signalDuration = signalDuration_POS;
            updateSignalPropertiesInGui();
            sendExcitationSignalProperties();
            disableTorqueParamInputs();
        }
    }
    else{
        ui->posContButton->setChecked(false);
    }
}


void MainWindow::on_velContButton_clicked(bool checked)
{
    if(discardChanges()){
        if (checked){
            testControlMode = VELOCITY_MODE;
            controlMode_string = "velocity";
            sendControlMode();
            yPlotLabel = "dq (deg/sec)";
            resetYLabel();
            ui->posContButton->setChecked(false);
            ui->torContButton->setChecked(false);
            signalAmplitude = signalAmplitude_VEL;
            signalStartTime = signalStartTime_VEL;
            signalDuration = signalDuration_VEL;
            updateSignalPropertiesInGui();
            sendExcitationSignalProperties();
            disableTorqueParamInputs();
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
            testControlMode = TORQUE_MODE;
            controlMode_string = "torque";
            usingJTC = checkIfUsingJtc();
            sendControlMode();
            yPlotLabel = "tau (Nm)";
            resetYLabel();
            ui->posContButton->setChecked(false);
            ui->velContButton->setChecked(false);
            signalAmplitude = signalAmplitude_TOR;
            signalStartTime = signalStartTime_TOR;
            signalDuration = signalDuration_TOR;
            updateSignalPropertiesInGui();
            sendExcitationSignalProperties();
            enableTorqueParamInputs();
            if (usingJTC) {
                ui->torque_header->setText("Torque (JTC)");
                ui->bemf_scale_in->setEnabled(false);
                ui->ktau_in->setEnabled(false);
                ui->ktau_scale_in->setEnabled(false);
            }else{
                ui->torque_header->setText("Torque");
                ui->coulombVelThresh_in->setEnabled(false);
                ui->frictionCompensation_in->setEnabled(false);
            }
        }
    }
    else{
        ui->torContButton->setChecked(false);
    }
}

double MainWindow::getValueFromUserInput(QLineEdit* userInputBox)
{
    bool ok;
    double dVal = userInputBox->text().toDouble(&ok);
    if(!ok){
        userInputBox->setStyleSheet("QLineEdit { background: rgb(255, 0, 0)}");
        int ret = QMessageBox::warning(this, tr("Warning"),
                                       tr("The gains must be a number. Try again dummy."),
                                       QMessageBox::Ok);
        if(!userInputBox->hasFocus())
            userInputBox->setFocus();

        userInputBox->selectAll();

    }
    else
        userInputBox->setStyleSheet("QLineEdit { background: rgb(255, 255, 255)}");

    return dVal;
}

void MainWindow::on_kp_in_editingFinished()
{
    newPid.Kp = getValueFromUserInput(ui->kp_in);
}

void MainWindow::on_kd_in_editingFinished()
{
    newPid.Kd = getValueFromUserInput(ui->kd_in);
}

void MainWindow::on_ki_in_editingFinished()
{
    newPid.Ki = getValueFromUserInput(ui->ki_in);
}

void MainWindow::on_kff_in_editingFinished()
{
    newPid.Kff = getValueFromUserInput(ui->kff_in);
}

void MainWindow::on_max_int_in_editingFinished()
{
    newPid.max_int = getValueFromUserInput(ui->max_int_in);
}

void MainWindow::on_scale_in_editingFinished()
{
    newPid.scale = getValueFromUserInput(ui->scale_in);
}

void MainWindow::on_max_output_in_editingFinished()
{
    newPid.max_output = getValueFromUserInput(ui->max_output_in);
}

void MainWindow::on_offset_in_editingFinished()
{
    newPid.offset = getValueFromUserInput(ui->offset_in);
}

void MainWindow::on_stiction_up_in_editingFinished()
{
    newPid.stiction_up = getValueFromUserInput(ui->stiction_up_in);
}

void MainWindow::on_stiction_down_in_editingFinished()
{
    newPid.stiction_down = getValueFromUserInput(ui->stiction_down_in);
}

void MainWindow::on_bemf_in_editingFinished()
{
    newPid.bemf = getValueFromUserInput(ui->bemf_in);
}

void MainWindow::on_coulombVelThresh_in_editingFinished()
{
    newPid.coulombVelThresh = getValueFromUserInput(ui->coulombVelThresh_in);
}

void MainWindow::on_frictionCompensation_in_editingFinished()
{
    newPid.frictionCompensation = getValueFromUserInput(ui->frictionCompensation_in);
}

void MainWindow::on_saveGainsButton_clicked()
{
    saveGains();
}


void MainWindow::on_gainResetButton_clicked()
{
    newPid = originalPid;

    refreshGainDisplays();

}

void MainWindow::refreshGainDisplays()
{
    ui->kp_in->setText(QString::number(newPid.Kp));
    ui->kd_in->setText(QString::number(newPid.Kd));
    ui->ki_in->setText(QString::number(newPid.Ki));
    ui->kff_in->setText(QString::number(newPid.Kff));
    ui->max_int_in->setText(QString::number(newPid.max_int));
    ui->scale_in->setText(QString::number(newPid.scale));
    ui->max_output_in->setText(QString::number(newPid.max_output));
    ui->offset_in->setText(QString::number(newPid.offset));
    ui->stiction_up_in->setText(QString::number(newPid.stiction_up));
    ui->stiction_down_in->setText(QString::number(newPid.stiction_down));
    if(testControlMode == TORQUE_MODE){
        ui->bemf_in->setText(QString::number(newPid.bemf));
        if(usingJTC){
            ui->coulombVelThresh_in->setText(QString::number(newPid.coulombVelThresh));
            ui->frictionCompensation_in->setText(QString::number(newPid.frictionCompensation));
        }else{
            ui->bemf_scale_in->setText(QString::number(newPid.bemf_scale));
            ui->ktau_in->setText(QString::number(newPid.Ktau));
            ui->ktau_scale_in->setText(QString::number(newPid.Ktau_scale));
        }
    }
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
              saveGains();
              return false;
              break;
          case QMessageBox::Discard:
              gainsHaveBeenChanged=false;
              return true;
              break;
          case QMessageBox::Cancel:
              return false;
              break;
          default:
              return false;
              break;
        }
    }
    else{return true;}

}

void MainWindow::saveGains()
{
    writeDataToLogs();
    log.info() << "Logs saved!";
    gainsHaveBeenChanged=false;
}


bool MainWindow::getPidValues()
{
    newPid.setControlMode(testControlMode, usingJTC);
    Bottle send, reply;
    send.addInt(GET_PID_VALUES);
    rpcClientPort.write(send,reply);
    if(newPid.extractFromBottle(reply)){
        refreshGainDisplays();
        return true;
    }else{
        log.error() << "Could not get PID values from control thread.";
        return false;
    }

}

bool MainWindow::setPidValues()
{
    newPid.setControlMode(testControlMode, usingJTC);
    Bottle send, reply;
    send.addInt(SET_PID_VALUES);
    newPid.putInBottle(send);
    rpcClientPort.write(send,reply);
    if(newPid.extractFromBottle(reply))
    {
        refreshGainDisplays();
        return true;
    }else{
        log.error() << "Could not read PID values from controller thread.";
        return false;
    }

}


void MainWindow::sendPartAndJointIndexes()
{
    log.info() << "Sending part and joint indexes: "<< partIndex << " & " << jointIndex;

    Bottle send, reply;
    send.addInt(SET_PART_AND_JOINT_INDEXES);
    send.addInt(partIndex);
    send.addInt(jointIndex);
    rpcClientPort.write(send,reply);
    if(reply.get(0).asInt()){
        getPidValues();
    }else{
        log.error() << "Could not set part and joint index.";
    }


}

void MainWindow::sendControlMode()
{
    Bottle send, reply;

    send.addInt(SET_CONTROL_MODE);
    send.addInt(testControlMode);
    rpcClientPort.write(send,reply);

    if (reply.get(0).asInt())
    {
        getPidValues();
        originalPid = newPid;
    }else{
        log.error() << "Could not set control mode.";
    }
}



/*

<?xml version="1.0" ?>

<!--
pidTuner Data Log
Last Updated: currentDateTime
-->

<part name="00-head">
    <joint name="00-neck_pitch">
        <mode name="position"   kp="Kp_new" kd="Kd_new" ki="Ki_new"></mode>
        <mode name="velocity"   kp="Kp_new" kd="Kd_new" ki="Ki_new"></mode>
        <mode name="torque"     kp="Kp_new" kd="Kd_new" ki="Ki_new"></mode>
    </joint>
</part>


*/





void MainWindow::createDataLogs()
{
    logFilePath = "pidTunerLog.xml";


    std::string updateComment = "\npidTuner Data Log\nLast Updated: " + currentDateTime()+ "\n";
    TiXmlDocument pidGainsLog;
	bool loadOkay = pidGainsLog.LoadFile(logFilePath.c_str());
	if (!loadOkay)
	{
        log.info() << "Generating a new PID log file.";
        TiXmlDeclaration* decl = new TiXmlDeclaration( "1.0", "", "" );
        pidGainsLog.LinkEndChild( decl );

        TiXmlComment* comment = new TiXmlComment();
    	comment->SetValue(updateComment.c_str());
        pidGainsLog.LinkEndChild( comment );
    }
    else
    {
        log.info() << "Modifying: " + logFilePath;
        for ( TiXmlNode* pChild = pidGainsLog.FirstChild(); pChild != 0; pChild = pChild->NextSibling())
    	{
            int TINYXML_COMMENT = 2;
    		if (pChild->Type() == TINYXML_COMMENT) {
                pChild->SetValue(updateComment.c_str());
    		}
    	}
    }

    pidGainsLog.SaveFile( logFilePath.c_str() );
}

void MainWindow::writeDataToLogs()
{
    TiXmlDocument pidGainsLog;
	bool loadOkay = pidGainsLog.LoadFile(logFilePath.c_str());


	if (loadOkay)
	{
        TiXmlElement * xml_part;
        TiXmlElement * xml_joint;
        TiXmlElement * xml_mode;

        bool partExists, jointExists, modeExists;
        partExists = jointExists = modeExists = false;


        for(TiXmlElement* partElem = pidGainsLog.FirstChildElement("part"); partElem != NULL; partElem = partElem->NextSiblingElement("part"))
        {
            if (partElem->Attribute("name") == ui->partList->currentText().toStdString())
            {
                partExists = true;
                xml_part = partElem;

                for(TiXmlElement* jointElem = partElem->FirstChildElement("joint"); jointElem != NULL; jointElem = jointElem->NextSiblingElement("joint"))
                {
                    if (jointElem->Attribute("name") == ui->jointList->currentText().toStdString())
                    {
                        jointExists = true;
                        xml_joint = jointElem;

                        for(TiXmlElement* modeElem = jointElem->FirstChildElement("mode"); modeElem != NULL; modeElem = modeElem->NextSiblingElement("mode"))
                        {
                            if (modeElem->Attribute("name") == controlMode_string)
                            {
                                modeExists = true;
                                xml_mode = modeElem;
                            }
                        }
                    }
                }
            }
        }



        if(!partExists)
        {
            xml_part = new TiXmlElement( "part" );
            pidGainsLog.LinkEndChild( xml_part );
            xml_part->SetAttribute("name", ui->partList->currentText().toStdString().c_str());
        }


        if(!jointExists)
        {
            xml_joint = new TiXmlElement( "joint" );
            xml_part->LinkEndChild( xml_joint );
            xml_joint->SetAttribute("name", ui->jointList->currentText().toStdString().c_str());
        }



        if(!modeExists)
        {
            xml_mode = new TiXmlElement( "mode" );
            xml_joint->LinkEndChild( xml_mode );
            xml_mode->SetAttribute("name", controlMode_string.c_str());
        }

        xml_mode->SetDoubleAttribute("Kp", newPid.Kp);
        xml_mode->SetDoubleAttribute("Kd", newPid.Kd);
        xml_mode->SetDoubleAttribute("Ki", newPid.Ki);
        xml_mode->SetDoubleAttribute("Kff", newPid.Kff);
        xml_mode->SetDoubleAttribute("max_int", newPid.max_int);
        xml_mode->SetDoubleAttribute("scale", newPid.scale);
        xml_mode->SetDoubleAttribute("max_output", newPid.max_output);
        xml_mode->SetDoubleAttribute("offset", newPid.offset);
        xml_mode->SetDoubleAttribute("stiction_up", newPid.stiction_up);
        xml_mode->SetDoubleAttribute("stiction_down", newPid.stiction_down);
        xml_mode->SetDoubleAttribute("bemf", newPid.bemf);
        xml_mode->SetDoubleAttribute("coulombVelThresh", newPid.coulombVelThresh);
        xml_mode->SetDoubleAttribute("frictionCompensation", newPid.frictionCompensation);
        xml_mode->SetDoubleAttribute("bemf_scale", newPid.bemf_scale);
        xml_mode->SetDoubleAttribute("Ktau", newPid.Ktau);
        xml_mode->SetDoubleAttribute("Ktau_scale", newPid.Ktau_scale);

        pidGainsLog.SaveFile( logFilePath.c_str() );

    }
    else{
        log.error() << "Couldn't find any existing log file... Ohhhhhhhhhhhhhhhhhhhhhhhh Shit!!!!! It hurts!! KILL ME!!!!";
    }

}

const std::string MainWindow::currentDateTime()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
    return buf;
}

void MainWindow::on_amplitude_in_editingFinished()
{
    bool ok;
    double amp_tmp = ui->amplitude_in->text().toDouble(&ok);
    if(!ok){
        ui->amplitude_in->setStyleSheet("QLineEdit { background: rgb(255, 0, 0)}");
        int ret = QMessageBox::warning(this, tr("Warning"),
                                       tr("The signal parameters must be numbers. Try again dummy."),
                                       QMessageBox::Ok);
        if(!ui->amplitude_in->hasFocus())
            ui->amplitude_in->setFocus();

        ui->amplitude_in->selectAll();

    }
    else{
        ui->amplitude_in->setStyleSheet("QLineEdit { background: rgb(255, 255, 255)}");

        switch (testControlMode) {
            case POSITION_MODE:
                signalAmplitude_POS = amp_tmp;
            break;

            case VELOCITY_MODE:
                signalAmplitude_VEL = amp_tmp;
            break;

            case TORQUE_MODE:
                signalAmplitude_TOR = amp_tmp;
            break;
        }
        signalAmplitude = amp_tmp;
        sendExcitationSignalProperties();
    }

}

void MainWindow::on_startTime_in_editingFinished()
{
    bool ok;
    double start_tmp = ui->startTime_in->text().toDouble(&ok);
    if(!ok){
        ui->startTime_in->setStyleSheet("QLineEdit { background: rgb(255, 0, 0)}");
        int ret = QMessageBox::warning(this, tr("Warning"),
                                       tr("The signal parameters must be numbers. Try again dummy."),
                                       QMessageBox::Ok);
        if(!ui->startTime_in->hasFocus())
            ui->startTime_in->setFocus();

        ui->startTime_in->selectAll();

    }
    else{
        ui->startTime_in->setStyleSheet("QLineEdit { background: rgb(255, 255, 255)}");

        switch (testControlMode) {
            case POSITION_MODE:
                signalStartTime_POS = start_tmp;
            break;

            case VELOCITY_MODE:
                signalStartTime_VEL = start_tmp;
            break;

            case TORQUE_MODE:
                signalStartTime_TOR = start_tmp;
            break;
        }
        signalStartTime = start_tmp;
        sendExcitationSignalProperties();
    }
}

void MainWindow::on_duration_in_editingFinished()
{
    bool ok;
    double dur_tmp = ui->duration_in->text().toDouble(&ok);
    if(!ok){
        ui->duration_in->setStyleSheet("QLineEdit { background: rgb(255, 0, 0)}");
        int ret = QMessageBox::warning(this, tr("Warning"),
                                       tr("The signal parameters must be numbers. Try again dummy."),
                                       QMessageBox::Ok);
        if(!ui->duration_in->hasFocus())
            ui->duration_in->setFocus();

        ui->duration_in->selectAll();

    }
    else{
        ui->duration_in->setStyleSheet("QLineEdit { background: rgb(255, 255, 255)}");

        switch (testControlMode) {
            case POSITION_MODE:
                signalDuration_POS = dur_tmp;
            break;

            case VELOCITY_MODE:
                signalDuration_VEL = dur_tmp;
            break;

            case TORQUE_MODE:
                signalDuration_TOR = dur_tmp;
            break;
        }
        signalDuration = dur_tmp;
        sendExcitationSignalProperties();
    }
}

void MainWindow::on_resetSignalPropButton_clicked()
{
    setSignalPropertiesToDefaults();
}

void MainWindow::sendExcitationSignalProperties()
{
    //TODO: get signal type from the signal type combo box.
    testSignalType = STEP;
    Bottle send, reply;
    send.addInt(SET_SIGNAL_PROPERTIES);
    send.addInt(testSignalType);
    send.addDouble(signalAmplitude);
    send.addDouble(signalStartTime);
    send.addDouble(signalDuration);
    rpcClientPort.write(send,reply);
    testSignalType = static_cast<SignalType>(reply.get(1).asInt());
    signalAmplitude = reply.get(2).asDouble();
    signalStartTime = reply.get(3).asDouble();
    signalDuration = reply.get(4).asDouble();

}


void MainWindow::setSignalPropertiesToDefaults()
{
    signalAmplitude_POS =   signalAmplitude_POS_DEFAULT;
    signalStartTime_POS =   signalStartTime_POS_DEFAULT;
    signalDuration_POS  =   signalDuration_POS_DEFAULT;

    signalAmplitude_VEL =   signalAmplitude_VEL_DEFAULT;
    signalStartTime_VEL =   signalStartTime_VEL_DEFAULT;
    signalDuration_VEL  =   signalDuration_VEL_DEFAULT;

    signalAmplitude_TOR =   signalAmplitude_TOR_DEFAULT;
    signalStartTime_TOR =   signalStartTime_TOR_DEFAULT;
    signalDuration_TOR  =   signalDuration_TOR_DEFAULT;


    switch (testControlMode) {
        case POSITION_MODE:
        signalAmplitude = signalAmplitude_POS;
        signalStartTime = signalStartTime_POS;
        signalDuration  = signalDuration_POS;
        break;

        case VELOCITY_MODE:
        signalAmplitude = signalAmplitude_VEL;
        signalStartTime = signalStartTime_VEL;
        signalDuration  = signalDuration_VEL;
        break;

        case TORQUE_MODE:
        signalAmplitude = signalAmplitude_TOR;
        signalStartTime = signalStartTime_TOR;
        signalDuration  = signalDuration_TOR;
        break;
    }

    updateSignalPropertiesInGui();
    sendExcitationSignalProperties();

}

void MainWindow::updateSignalPropertiesInGui()
{

    ui->amplitude_in->setText(QString::number(signalAmplitude));
    ui->startTime_in->setText(QString::number(signalStartTime));
    ui->duration_in->setText(QString::number(signalDuration));
}


void MainWindow::on_savePlotButton_clicked()
{
    std::string appendNotes = "_"+controlMode_string+".png";
    QString qFilePath(ui->jointList->currentText().append(QString::fromStdString(appendNotes)));
    if(ui->posPlot->savePng(qFilePath) )
    {
        log.info() << "File saved to "+ qFilePath.toStdString();
    }
    else{
        log.error() << "Failed to save file. Check filepath: \n" << qFilePath.toStdString();
    }
}


bool MainWindow::checkIfUsingJtc()
{
    Bottle send, reply;
    send.addInt(CHECK_IF_USING_JTC);
    rpcClientPort.write(send,reply);
    return reply.get(0).asInt();
}
