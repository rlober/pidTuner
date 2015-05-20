#include "thread.h"
#include <math.h>

#define DEG_TO_RAD M_PI / 180.
#define RAD_TO_DEG 180. / M_PI



CtrlThread::CtrlThread(const double period) : RateThread(int(period*1000.0))
{
    //do nothing
}



bool CtrlThread::threadInit()
{
    robotName = "icubGazeboSim";
    extension = ".txt";
    baseFilePath = "/home/ryan/Desktop/";
    // createPidLog();

    isPositionMode  = true;
    isVelocityMode  = false;
    isTorqueMode    = false;

    partIndex = 0;
    jointIndex = 0;

    robotParts.push_back("head");
    robotParts.push_back("torso");
    robotParts.push_back("left_arm");
    robotParts.push_back("right_arm");
    robotParts.push_back("left_leg");
    robotParts.push_back("right_leg");

    numRobotParts = robotParts.size();
    nJoints.resize(numRobotParts);
    // robotDevice.resize(numRobotParts);
    iEnc.resize(numRobotParts);
    iPos.resize(numRobotParts);
    iVel.resize(numRobotParts);
    iTrq.resize(numRobotParts);
    iLims.resize(numRobotParts);


    encoders.resize(numRobotParts);
    command.resize(numRobotParts);
    tmp.resize(numRobotParts);
    homeVectors.resize(numRobotParts);
    jointLimitsLower.resize(numRobotParts);
    jointLimitsUpper.resize(numRobotParts);

    openInterfaces();

    for (int i=0; i<numRobotParts; i++)
    {
        std::cout << homeVectors[i].toString().c_str() << std::endl;
    }

    jointCommandsHaveBeenUpdated = false;
    controlThreadFinished = false;


    reverseDirectrion = false;
    reversalCounter = 0;

    //Open yarp ports
    gainsBufPort_in.open("/pidTunerController/gains/in");
    gainsPort_out.open("/pidTunerController/gains/out");

    goToHomeBufPort_in.open("/pidTunerController/goToHome/in");

    robotPartAndJointBufPort_in.open("/pidTunerController/partAndJointIndexes/in");

    return true;

}

void CtrlThread::afterStart(bool s)
{
    if (s)
        fprintf(stdout,"Thread started successfully\n");
    else
        fprintf(stdout,"Thread did not start\n");
}

/*********************************************************
                        THREAD RUN
**********************************************************/

void CtrlThread::run()
{
    // Check if new gains have come in or if the user wants the current gains
    Bottle *gainsMessage = gainsBufPort_in.read(false);
    if (gainsMessage!=NULL) {
        parseIncomingGains(gainsMessage);
        sendPidGains();
    }

    // Check if the user wants to go to Home Pose
    Bottle *goToHomeMessage = goToHomeBufPort_in.read(false);
    if (goToHomeMessage!=NULL) {
        if(goToHomeMessage->get(0).asInt()==1){
            setCommandToHome();
        }
    }

    Bottle *robotPartAndJointMessage = robotPartAndJointBufPort_in.read(false);
    if (robotPartAndJointMessage!=NULL) {
        partIndex = robotPartAndJointMessage->get(0).asInt();
        jointIndex = robotPartAndJointMessage->get(1).asInt();
        updatePidInformation();
        std::cout << partIndex << " " << jointIndex << std::endl;
    }






    if(jointCommandsHaveBeenUpdated)
    {
        sendJointCommands();
    }

    // // Update encoders
    // encs_LeftArm->getEncoders(encoders_LeftArm.data());
    // encs_Head->getEncoders(encoders_Head.data());
    // writeToPidLog();

}

/********************************************************/

void CtrlThread::updatePidInformation()
{
    // when part and/or joints change reset the Kpdi gains
}

void CtrlThread::threadRelease()
{


    std::cout << "\n\n\t<-------------------------->\n";
    while(!goToHome())
    {
        // std::cout << "Going to Home Pose\n";
        Time::delay(0.1);
    }

    for (int rp=0; rp<numRobotParts; rp++)
    {
        robotDevice[rp]->close();
    }

}

bool CtrlThread::openInterfaces()
{
    std::cout << "\n\nOpening Interfaces:\n";
    for (int rp=0; rp<numRobotParts; rp++)
    {
        std::string curRobPart = robotParts[rp];

        std::cout << curRobPart << std::endl;

        Property options;
        options.put("device", "remote_controlboard");
        options.put("local", "/"+curRobPart+"/client");   //local port names
        options.put("remote", "/"+robotName+"/"+curRobPart);         //where we connect to


        PolyDriver *tmp_robotDevice = new PolyDriver(options);
        if (!tmp_robotDevice->isValid()) {
            printf("Device not available.  Here are the known devices:\n");
            printf("%s", Drivers::factory().toString().c_str());
            return false;
        }


        robotDevice.push_back(tmp_robotDevice);

        bool ok;
        ok = robotDevice[rp]->view(iPos[rp]);
        ok = ok && robotDevice[rp]->view(iEnc[rp]);
        ok = ok && robotDevice[rp]->view(iVel[rp]);
        ok = ok && robotDevice[rp]->view(iTrq[rp]);
        ok = ok && robotDevice[rp]->view(iLims[rp]);



        if (!ok) {
            printf("Problems acquiring interfaces\n");
            return false;
        }


        nJoints[rp]=0;
        iPos[rp]->getAxes(&nJoints[rp]);


        encoders[rp].resize(nJoints[rp]);
        tmp[rp].resize(nJoints[rp]);
        command[rp].resize(nJoints[rp]);

        jointLimitsLower[rp].resize(nJoints[rp]);
        jointLimitsUpper[rp].resize(nJoints[rp]);


        for (int i = 0; i < nJoints[rp]; i++) {
             tmp[rp][i] = 50.0;
             iLims[rp]->getLimits(i, &jointLimitsLower[rp][i], &jointLimitsUpper[rp][i]);
        }
        iPos[rp]->setRefAccelerations(tmp[rp].data());

        for (int i = 0; i < nJoints[rp]; i++) {
            tmp[rp][i] = 10.0;
            iPos[rp]->setRefSpeed(i, tmp[rp][i]);
        }

        homeVectors[rp].resize(nJoints[rp]);

        while(!iEnc[rp]->getEncoders(homeVectors[rp].data()) )
        {
            Time::delay(0.01);
        }

        while(!iEnc[rp]->getEncoders(command[rp].data()))
        {
            Time::delay(0.01);
        }



    }
    return true;
}


bool CtrlThread::goToHome()
{
    std::cout<<"\n\tMoving to home position.\n\n\n";
    for (int rp=0; rp<numRobotParts; rp++)
    {
        // std::string curRobPart = robotParts[rp];

        iPos[rp]->positionMove(homeVectors[rp].data());
    }

    for (int rp=0; rp<numRobotParts; rp++)
    {
        bool moveDone = false;
        while(!moveDone)
        {
            Time::delay(0.1);
            iPos[rp]->checkMotionDone(&moveDone);
        }

    }
    return true;
}

bool CtrlThread::sendJointCommands()
{
    for (int rp=0; rp<numRobotParts; rp++)
    {
        if (isPositionMode)
        {
            iPos[rp]->positionMove(command[rp].data());
        }

        else if (isVelocityMode)
        {
            //
        }

        else if (isTorqueMode)
        {
            //
        }

        else{return false;}

    }
    return true;
}


void CtrlThread::setCommandToHome()
{
    for (int rp=0; rp<numRobotParts; rp++)
    {
        for (int jnt = 0; jnt < nJoints[rp]; jnt++)
        {
            command[rp][jnt] = homeVectors[rp][jnt];
        }
    }
    jointCommandsHaveBeenUpdated = true;
}


void CtrlThread::parseIncomingGains(Bottle *newGainMessage)
{
    int messageIndicator = newGainMessage->get(0).asInt();
    if (messageIndicator == 1)
    {
        std::cout   << "Kp = " << newGainMessage->get(1).asDouble()
                    << " Kd = " << newGainMessage->get(2).asDouble()
                    << " Ki = " << newGainMessage->get(3).asDouble()
                    << std::endl;
        Kp = newGainMessage->get(1).asDouble();
        Kd = newGainMessage->get(2).asDouble();
        Ki = newGainMessage->get(3).asDouble();
    }

}




void CtrlThread::sendPidGains()
{
    Bottle gainsBottle_out; // Get a place to store things.
    gainsBottle_out.clear();
    gainsBottle_out.addDouble(Kp);
    gainsBottle_out.addDouble(Kd);
    gainsBottle_out.addDouble(Ki);
    gainsPort_out.write(gainsBottle_out);
}

//

void CtrlThread::createPidLog()
{
    std::string directoryName = "PID_GAIN_LOGS";
    baseFilePath += "/"+directoryName+"/" + currentDateTime() + "/";
    // Make directory if it doesn't already exist
    if (!boost::filesystem::exists(baseFilePath))
        boost::filesystem::create_directories(baseFilePath);


    pidLogFilePath = baseFilePath + "/jointPidGains"+extension;
    pidFile.open(pidLogFilePath.c_str());
    pidFile.close();


}

void CtrlThread::writeToPidLog()
{

    // std::ostringstream absStrs,
    // absStrs << absoluteT;
    // std::string absStr = absStrs.str();



    pidFile.open(pidLogFilePath.c_str(), std::ios::app);
    pidFile << "test" << std::endl;
    pidFile.close();



}

const std::string CtrlThread::currentDateTime()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
    return buf;
}

bool CtrlThread::isFinished()
{
    return controlThreadFinished;
}



// double degChange = 1.0;
//
// if ( (command[partIndex][jointIndex] < jointLimitsUpper[partIndex][jointIndex]) && !reverseDirectrion)
// {
//     command[partIndex][jointIndex] += degChange;
// }
//
// else if( (command[partIndex][jointIndex] > jointLimitsLower[partIndex][jointIndex]) && reverseDirectrion)
// {
//     command[partIndex][jointIndex] -= degChange;
// }
//
// else
// {
//     reverseDirectrion = !reverseDirectrion;
//     reversalCounter++;
// }
//
// if (reversalCounter>=2)
// {
//     command[partIndex][jointIndex] = homeVectors[partIndex][jointIndex];
//
//     jointIndex++;
//
//     if (jointIndex==nJoints[partIndex])
//     {jointIndex = 0; partIndex++;}
//
//     if(partIndex==numRobotParts)
//     {jointIndex=0; partIndex=0;}
//
//     reversalCounter = 0;
// }
