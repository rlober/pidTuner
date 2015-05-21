#include "thread.h"
#include <math.h>

#define DEG_TO_RAD M_PI / 180.
#define RAD_TO_DEG 180. / M_PI

#define POSITION_MODE 0
#define VELOCITY_MODE 1
#define TORQUE_MODE 2



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

    Kp_thread = Kd_thread = Ki_thread = 0.0;

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
    iPids.resize(numRobotParts);


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
    applyExcitationSignal = false;

    reverseDirectrion = false;
    reversalCounter = 0;

    //Open yarp ports
    gainsBufPort_in.open("/pidTunerController/gains/in");
    gainsPort_out.open("/pidTunerController/gains/out");

    goToHomeBufPort_in.open("/pidTunerController/goToHome/in");

    robotPartAndJointBufPort_in.open("/pidTunerController/partAndJointIndexes/in");

    controlModeBufPort_in.open("/pidTunerController/controlMode/in");

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
    }

    Bottle *controlModeMessage = controlModeBufPort_in.read(false);
    if (controlModeMessage!=NULL) {
        parseIncomingControlMode(controlModeMessage);
        updatePidInformation();
    }


    if (applyExcitationSignal) {
        double signalOutput = excitationSignal(signalStartTime);
        command[partIndex][jointIndex] = homeVectors[partIndex][jointIndex] + signalOutput;
    }


    if(jointCommandsHaveBeenUpdated || applyExcitationSignal)
    {
        sendJointCommands();
    }

    // // Update encoders
    // encs_LeftArm->getEncoders(encoders_LeftArm.data());
    // encs_Head->getEncoders(encoders_Head.data());
    // writeToPidLog();

}

/********************************************************/



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
        ok = ok && robotDevice[rp]->view(iPids[rp]);




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
        std::cout   << "trying to set part, "<<partIndex<<" & joint, "<< jointIndex << " to PID:\n"
                    << "Kp = " << newGainMessage->get(1).asDouble()
                    << " Kd = " << newGainMessage->get(2).asDouble()
                    << " Ki = " << newGainMessage->get(3).asDouble()
                    << std::endl;
        Pid newPid;
        newPid.setKp(newGainMessage->get(1).asDouble());
        newPid.setKd(newGainMessage->get(2).asDouble());
        newPid.setKi(newGainMessage->get(3).asDouble());

        //send new Pid to device
        std::cout<< "sending Kp = "<< newPid.kp <<" Kd = "<< newPid.kd << " Ki = "<< newPid.ki <<" to device\n"<<std::endl;
        if (!iPids[partIndex]->setPid(jointIndex, newPid)) {

            std::cout<<"send failed...\n";
            // Time::delay(0.001);
        }

        //Get those gains from the device to make sure they set properly

        updatePidInformation();
        std::cout<< "gains set to device:\nKp = "<< Kp_thread <<" Kd = "<< Kd_thread << " Ki = "<< Ki_thread <<"\n"<<std::endl;


        signalStartTime = Time::now();
        applyExcitationSignal = true;

    }

}

void CtrlThread::parseIncomingControlMode(Bottle *newControlModeMessage)
{
    int newControlMode = newControlModeMessage->get(0).asInt();
    switch (newControlMode) {
        case POSITION_MODE:
            isPositionMode = true;
            isVelocityMode = false;
            isTorqueMode = false;
            std::cout << "Switching to POSITION control." << std::endl;
            break;

        case VELOCITY_MODE:
            isPositionMode = false;
            isVelocityMode = true;
            isTorqueMode = false;
            std::cout << "Switching to VELOCITY control." << std::endl;
            break;

        case TORQUE_MODE:
            isPositionMode = false;
            isVelocityMode = false;
            isTorqueMode = true;
            std::cout << "Switching to TORQUE control." << std::endl;
            break;

        default:
            isPositionMode = true;
            isVelocityMode = false;
            isTorqueMode = false;
            std::cout << "[WARNING] Defaulting to POSITION control." << std::endl;
            break;
    }

    /*
        Need to implement control mode switch here...

     */
}

void CtrlThread::updatePidInformation()
{
    Pid* currentPid;
    if (iPids[partIndex]==NULL) {
        std::cout << "no iPid device" << std::endl;
    }


    if(iPids[partIndex]->getPid(jointIndex, currentPid))
    {
        Kp_thread = currentPid->kp;
        Kd_thread = currentPid->kd;
        Ki_thread = currentPid->ki;
    }
    else{
        std::cout << "[ERROR] Couldn't retrieve PID from part "<<partIndex << ", joint " << jointIndex<<"." << std::endl;
    }


}




void CtrlThread::sendPidGains()
{
    Bottle gainsBottle_out; // Get a place to store things.
    gainsBottle_out.clear();
    gainsBottle_out.addDouble(Kp_thread);
    gainsBottle_out.addDouble(Kd_thread);
    gainsBottle_out.addDouble(Ki_thread);
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



double CtrlThread::excitationSignal(double triggerTime)
{
    double relativeT = triggerTime - Time::now();
    double applyInputTime = 0.5;
    double signalDuration = 1.0;



    if (isPositionMode) { // need to implement different signal types
        double stepAmplitude = 5.5; //deg
        if (relativeT<applyInputTime) {
            return 0.0;
        }
        else if (relativeT>=applyInputTime && relativeT<(applyInputTime+signalDuration)) {
            return stepAmplitude;
        }
        else if (relativeT>=(applyInputTime+signalDuration)){
            applyExcitationSignal = false;
            return 0.0;
        }
    else
    {

        return 0.0;
    }


    }
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
