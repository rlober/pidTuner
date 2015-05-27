#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <cmath>
#include <iostream>



#define POSITION_MODE 0
#define VELOCITY_MODE 1
#define TORQUE_MODE 2


using namespace yarp::os;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
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





    controlMode = POSITION_MODE;



    isOnlyMajorJoints = true;
    gainsHaveBeenChanged = false;
    Kp_old=Kd_old=Ki_old=1.0;
    Kp_new=Kd_new=Ki_new=2.0;

    ///////////////////////////////////
    gainsBufPort_out.open("/pidTunerGui/gains/out");
    gainsPort_in.open("/pidTunerGui/gains/in");

    goToHomeBufPort_out.open("/pidTunerGui/goToHome/out");

    robotPartAndJointBufPort_out.open("/pidTunerGui/partAndJointIndexes/out");

    controlModeBufPort_out.open("/pidTunerGui/controlMode/out");

    dataPort_in.open("/pidTunerGui/data/in");

    signalPropertiesBufPort_out.open("/pidTunerGui/signalProperties/out");

    ///////////////////////////////////
    while(!yarp.connect("/pidTunerGui/gains/out", "/pidTunerController/gains/in") ){Time::delay(0.1);}
    while(!yarp.connect("/pidTunerController/gains/out", "/pidTunerGui/gains/in") ){Time::delay(0.1);}

    while(!yarp.connect("/pidTunerGui/goToHome/out", "/pidTunerController/goToHome/in") ){Time::delay(0.1);}

    while(!yarp.connect("/pidTunerGui/partAndJointIndexes/out", "/pidTunerController/partAndJointIndexes/in") ){Time::delay(0.1);}

    while(!yarp.connect("/pidTunerGui/controlMode/out", "/pidTunerController/controlMode/in") ){Time::delay(0.1);}

    while(!yarp.connect("/pidTunerController/data/out", "/pidTunerGui/data/in") ){Time::delay(0.1);}

    while(!yarp.connect("/pidTunerGui/signalProperties/out", "/pidTunerController/signalProperties/in") ){Time::delay(0.1);}

    initFinished = false;
    ui->setupUi(this);




    setSignalPropertiesToDefaults();
    initializeGui();
    getPidGains();
    createDataLogs();
    initFinished = true;

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setCurrentPartAndJoint()
{
    ui->partList->setCurrentIndex(partIndex);
    ui->jointList->setCurrentIndex(jointIndex);
}

void MainWindow::initializeGui()
{


    addPartsToList();

    partIndex=ui->partList->count()-1;
    ui->partList->setCurrentIndex(partIndex);
    jointIndex = ui->jointList->count()-1;
    setCurrentPartAndJoint();
    sendPartAndJointIndexes();

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
    plotTitle = new QCPPlotTitle(ui->posPlot, ui->jointList->currentText());
    ui->posPlot->plotLayout()->addElement(0, 0, plotTitle);
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
}

void MainWindow::on_gainTestButton_clicked()
{
    setPidGains();
    /*  To get a vector using yarp ports we have to use a little workaround basically
        the data is saved in a yarp vector  then when we are ready to send each entry
        is sent individually. The first value is an int (0 or 1) which indicates to
        the receiver when to listen. 0 = stop listening and 1 = start. The size of the
        vector is inferred from the number of messages sent with a first value of 1.
    */

    Bottle dataFromController;
    yarp::sig::Vector y_Time, y_Input, y_Response;

    int bufferLength = 1000;
    y_Time.resize(bufferLength);
    y_Input.resize(bufferLength);
    y_Response.resize(bufferLength);


    int vecLength=0;
    bool waitingForData = true;
    while(waitingForData)
    {
        if(dataPort_in.read(dataFromController))
        {
            if(dataFromController.get(0).asInt())
            {
                y_Time[vecLength] = dataFromController.get(1).asDouble();
                y_Input[vecLength] = dataFromController.get(2).asDouble();
                y_Response[vecLength] = dataFromController.get(3).asDouble();
                vecLength++;
            }

            else if(!dataFromController.get(0).asInt())
            {
                waitingForData = false;
            }

        }
        Time::delay(0.001);
    }

    // std::cout << "\n\n--------\nData received. Parsing "<< dataFromController.size()<<" items..." << std::endl;
    // int vecLength = dataFromController.get(0).asInt();

    y_Time.resize(vecLength);
    y_Input.resize(vecLength);
    y_Response.resize(vecLength);



    std::cout << "\nData length = " << vecLength << std::endl;
    // dataFromController.get(1).asList()->write(y_Time);
    // dataFromController.get(2).asList()->write(y_Input);
    // dataFromController.get(3).asList()->write(y_Response);



    // std::cout << "received data_time: \n" << y_Time.toString().c_str() << std::endl;
    // std::cout << "received data_input: \n" << y_Input.toString().c_str() << std::endl;
    // std::cout << "received data_response: \n" << y_Response.toString().c_str() << std::endl;




    std::cout << "Time size: "<< y_Time.size() << " Input size: "<< y_Input.size() <<" Response size: "<< y_Response.size() << std::endl;







    std::cout << "\nConverting data from Yarp to Qt vectors" << std::endl;

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

    std::cout << "\nData plotted." << std::endl;

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
        sendPartAndJointIndexes();
        plotTitle->setText(ui->jointList->currentText());
        ui->posPlot->replot();
    }

}






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

        // setCurrentPartAndJoint();
    }
}


void MainWindow::on_posContButton_clicked(bool checked)
{
    if(discardChanges()){
        if (checked){
            controlMode = POSITION_MODE;
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
            controlMode = VELOCITY_MODE;
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
            controlMode = TORQUE_MODE;
            controlMode_string = "torque";
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
        }
    }
    else{
        ui->torContButton->setChecked(false);
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
    qDebug()<<"saved";
    gainsHaveBeenChanged=false;
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
    robotPartAndJointBufPort_out.write();

    getPidGains();


}

void MainWindow::sendControlMode()
{
    Bottle& controlModeBottle_out = controlModeBufPort_out.prepare(); // Get a place to store things.
    controlModeBottle_out.clear();
    controlModeBottle_out.addInt(controlMode);
    controlModeBufPort_out.write();
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
        std::cout << "Generating a new PID log file." << std::endl;
        TiXmlDeclaration* decl = new TiXmlDeclaration( "1.0", "", "" );
        pidGainsLog.LinkEndChild( decl );

        TiXmlComment* comment = new TiXmlComment();
    	comment->SetValue(updateComment.c_str());
        pidGainsLog.LinkEndChild( comment );
    }
    else
    {
        std::cout << "Modifying: " + logFilePath << std::endl;
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

        xml_mode->SetDoubleAttribute("Kp", Kp_new);
        xml_mode->SetDoubleAttribute("Kd", Kd_new);
        xml_mode->SetDoubleAttribute("Ki", Ki_new);

        pidGainsLog.SaveFile( logFilePath.c_str() );

    }
    else{
        std::cout << "Couldn't find any existing log file... Ohhhhhhhhhhhhhhhhhhhhhhhh Shit!!!!! It hurts!! KILL ME!!!!" << std::endl;
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

        switch (controlMode) {
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

        switch (controlMode) {
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

        switch (controlMode) {
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
    Bottle& sigPropsBottle_out = signalPropertiesBufPort_out.prepare(); // Get a place to store things.
    sigPropsBottle_out.clear();
    sigPropsBottle_out.addInt(ui->signalTypeComboBox->currentIndex()); // tells the receiver that new gains are comming and should be set
    sigPropsBottle_out.addDouble(signalAmplitude);
    sigPropsBottle_out.addDouble(signalStartTime);
    sigPropsBottle_out.addDouble(signalDuration);
    signalPropertiesBufPort_out.write();
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


    switch (controlMode) {
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

    // std::string filePath = "test.png";
    // QString qFilePath = QString::fromStdString(filePath);
    // qDebug()<<qFilePath;
    // ui->posPlot->savePng(qFilePath);
    std::string appendNotes = "_"+controlMode_string+".png";
    QString qFilePath(ui->jointList->currentText().append(QString::fromStdString(appendNotes)));
    if(ui->posPlot->savePng(qFilePath) )
    {
        std::cout << "File saved to "+ qFilePath.toStdString() << std::endl;
    }
    else{
        std::cout << "[ERROR] Failed to save file. Check filepath:\n"+qFilePath.toStdString() << std::endl;
    }
}
