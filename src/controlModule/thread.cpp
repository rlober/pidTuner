 /*!
 *  \file       thread.cpp
 *  \brief      CtrlThread class implementation. This file implements the
                pidTunerController module which communicates with the robot's
                control boards and handles the control logic.
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



#include "thread.h"
#include <math.h>

#define DEG_TO_RAD M_PI / 180.
#define RAD_TO_DEG 180. / M_PI

#define POSITION_MODE 0
#define VELOCITY_MODE 1
#define TORQUE_MODE 2

#define SIG_STEP 0
#define SIG_SIGN 1
#define SIG_TRIANGLE 2
#define SIG_SQUARE 3
#define SIG_DIRAC 4



CtrlThread::CtrlThread(const double period, const std::string Robot_Name) :
    RateThread(int(period*1000.0)),
    robotName(Robot_Name)
{
    if (robotName.size()==0) {
        std::cout << "[WARNING] Robot name was not initialized. Defaulting to robotName=icubGazeboSim." << std::endl;
        std::cout << "To set the robot name simply use: --robot [name of robot] when launching the pidTunerController. Remember, no brackets around the name of the robot." << std::endl;

        robotName="icubGazeboSim";
    }
}



bool CtrlThread::threadInit()
{

    // robotName = "icubGazeboSim";
    extension = ".txt";
    baseFilePath = "/home/ryan/Desktop/";


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
    iCtrl.resize(numRobotParts);


    encoders.resize(numRobotParts);
    command.resize(numRobotParts);
    tmp.resize(numRobotParts);
    homeVectors.resize(numRobotParts);
    jointLimitsLower.resize(numRobotParts);
    jointLimitsUpper.resize(numRobotParts);

    openInterfaces();

    goToHome();

    jointCommandsHaveBeenUpdated = false;
    controlThreadFinished = false;
    applyExcitationSignal = false;
    dataReadyForDelivery = false;
    iterationCounter = 0;
    resizeDataVectors();



    //Open yarp ports
    gainsBufPort_in.open("/pidTunerController/gains/in");
    gainsPort_out.open("/pidTunerController/gains/out");

    goToHomeBufPort_in.open("/pidTunerController/goToHome/in");

    robotPartAndJointBufPort_in.open("/pidTunerController/partAndJointIndexes/in");

    controlModeBufPort_in.open("/pidTunerController/controlMode/in");

    dataPort_out.open("/pidTunerController/data/out");

    signalPropertiesBufPort_in.open("/pidTunerController/signalProperties/in");

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
        std::cout << "Received gains message." << std::endl;
        resizeDataVectors();
        parseIncomingGains(gainsMessage);
        sendPidGains();
    }

    // Check if the user wants to go to Home Pose
    Bottle *goToHomeMessage = goToHomeBufPort_in.read(false);
    if (goToHomeMessage!=NULL) {
        std::cout << "Received go to home message." << std::endl;
        if(goToHomeMessage->get(0).asInt()==1){
            // setCommandToHome();
            goToHome();
        }
    }

    Bottle *robotPartAndJointMessage = robotPartAndJointBufPort_in.read(false);
    if (robotPartAndJointMessage!=NULL) {
        std::cout << "Received part and joint index message." << std::endl;
        partIndex = robotPartAndJointMessage->get(0).asInt();
        jointIndex = robotPartAndJointMessage->get(1).asInt();
        updatePidInformation();
    }

    Bottle *controlModeMessage = controlModeBufPort_in.read(false);
    if (controlModeMessage!=NULL) {
        std::cout << "Received control mode message." << std::endl;
        parseIncomingControlMode(controlModeMessage);
        updatePidInformation();
    }

    Bottle *signalPropertiesMessage = signalPropertiesBufPort_in.read(false);
    if (signalPropertiesMessage!=NULL) {
        std::cout << "Received signal properties message." << std::endl;
        parseIncomingSignalProperties(signalPropertiesMessage);
    }

    double cmd;
    bool runningTest = excitationSignal(cmd);
    if(jointCommandsHaveBeenUpdated || runningTest)
    {
        if (runningTest){
            // data_input[iterationCounter] = command[partIndex][jointIndex];
            data_input[iterationCounter] = cmd;

        }

        sendJointCommand(cmd);

        if (runningTest){
            data_response[iterationCounter] = getJointResponse();


            if(dataReadyForDelivery){
                sendDataToGui();
                goToHome();
            }else{
                iterationCounter++;
            }
        }

        jointCommandsHaveBeenUpdated = false;
    }


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

        std::cout << "\nViewing Devices:" << std::endl;
        ok = robotDevice[rp]->view(iPos[rp]);
        std::cout << "iPos created" << std::endl;
        ok = ok && robotDevice[rp]->view(iEnc[rp]);
        std::cout << "iEnc created" << std::endl;
        ok = ok && robotDevice[rp]->view(iVel[rp]);
        std::cout << "iVel created" << std::endl;
        ok = ok && robotDevice[rp]->view(iTrq[rp]);
        std::cout << "iTrq created" << std::endl;
        ok = ok && robotDevice[rp]->view(iLims[rp]);
        std::cout << "iLims created" << std::endl;
        ok = ok && robotDevice[rp]->view(iPids[rp]);
        std::cout << "iPids created" << std::endl;
        ok = ok && robotDevice[rp]->view(iCtrl[rp]);
        std::cout << "iCtrl created" << std::endl;




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
        homeVectors[rp].resize(nJoints[rp]);


        for (int i = 0; i < nJoints[rp]; i++) {
             tmp[rp][i] = 50.0;
             iLims[rp]->getLimits(i, &jointLimitsLower[rp][i], &jointLimitsUpper[rp][i]);
        }
        iPos[rp]->setRefAccelerations(tmp[rp].data());

        for (int i = 0; i < nJoints[rp]; i++) {
            tmp[rp][i] = 10.0;
            homeVectors[rp][i] = 0.0;
            iPos[rp]->setRefSpeed(i, tmp[rp][i]);
        }

        if (rp==2 || rp==3) {
            homeVectors[rp][0] = -25.0;
            homeVectors[rp][1] = 20.0;
            homeVectors[rp][3] = 50.0;
        }



        // while(!iEnc[rp]->getEncoders(homeVectors[rp].data()) )
        // {
        //     Time::delay(0.01);
        // }
        //
        while(!iEnc[rp]->getEncoders(command[rp].data()))
        {
            std::cout << "getting encoder data" << std::endl;
            Time::delay(0.01);
        }





    }
    return true;
}


bool CtrlThread::goToHome()
{
    std::cout<<"\nMoving to home position.\n";


    for (int rp=0; rp<numRobotParts; rp++)
    {
        for (int jnt = 0; jnt < nJoints[rp]; jnt++)
        {
            iCtrl[rp]->setControlMode(jnt, VOCAB_CM_POSITION);
        }

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
    std::cout << "Finished." << std::endl;
    return true;
}

bool CtrlThread::sendJointCommand(double cmd)
{
/*
    for (int rp=0; rp<numRobotParts; rp++)
    {
        if (isPositionMode)
        {
            iPos[rp]->positionMove(command[rp].data());
        }

        else if (isVelocityMode)
        {
            iVel[rp]->velocityMove(command_velocity.data());
        }

        else if (isTorqueMode)
        {
            //
        }

        else{return false;}

    }
    return true;
*/

if (isPositionMode)
{
    iCtrl[partIndex]->setControlMode(jointIndex, VOCAB_CM_POSITION);
    iPos[partIndex]->positionMove(jointIndex, cmd);
}

else if (isVelocityMode)
{
    /*
        Using IVelocityControl2
    */

    iCtrl[partIndex]->setControlMode(jointIndex, VOCAB_CM_VELOCITY);
    // int jointVectorLength = 1;
    // Vector jointVector(1, jointIndex);
    // Vector cmdVector(1, cmd);
    iVel[partIndex]->velocityMove(1, &jointIndex, &cmd);//jointVector.data(), cmdVector.data());

    /*
        Another option is to use IVelocityControl and use:
        iVel[partIndex]->velocityMove(jointIndex, cmd);

    */


}

else if (isTorqueMode)
{
    iCtrl[partIndex]->setControlMode(jointIndex, VOCAB_CM_TORQUE);
    iTrq[partIndex]->setRefTorque(jointIndex, cmd);
}

else{return false;}

return true;

}

/*
void CtrlThread::setCommandToHome()
{
    for (int rp=0; rp<numRobotParts; rp++)
    {

        for (int jnt = 0; jnt < nJoints[rp]; jnt++)
        {
            iCtrl[rp]->setControlMode(jnt, VOCAB_CM_POSITION);
            command[rp][jnt] = homeVectors[rp][jnt];
            std::cout << "Going to home configuration..." << std::endl;
        }
    }
    jointCommandsHaveBeenUpdated = true;
}
*/

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

        if(isPositionMode){
            if (!iPids[partIndex]->setPid(jointIndex, newPid)) {
                std::cout<<"Position PID send failed...\n";
            }
        }
        else if(isVelocityMode){
            if (!iVel[partIndex]->setVelPid(jointIndex, newPid)) {
                std::cout<<"Velocity PID send failed...\n";
            }
        }
        else if(isTorqueMode){
            if (!iTrq[partIndex]->setTorquePid(jointIndex, newPid)) {
                std::cout<<"Torque PID send failed...\n";
            }
        }

        //Get those gains from the device to make sure they set properly

        updatePidInformation();
        std::cout<< "gains set to device:\nKp = "<< Kp_thread <<" Kd = "<< Kd_thread << " Ki = "<< Ki_thread <<"\n"<<std::endl;


        triggerTime = Time::now();

        if (isTorqueMode){
            while(!iTrq[partIndex]->getTorque(jointIndex, &stationaryTorque) )
            {
                Time::delay(0.001);
            }
        }

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

void CtrlThread::parseIncomingSignalProperties(Bottle *newSignalPropertiesMessage)
{
    int signalType = newSignalPropertiesMessage->get(0).asInt();
    switch (signalType) {
        case SIG_STEP:
        signalAmplitude = newSignalPropertiesMessage->get(1).asDouble();
        signalStartTime = newSignalPropertiesMessage->get(2).asDouble();
        signalDuration = newSignalPropertiesMessage->get(3).asDouble();
        break;

        case SIG_SIGN:
        /*
            Need to implement these signals...
        */
        signalAmplitude = newSignalPropertiesMessage->get(1).asDouble();
        signalStartTime = newSignalPropertiesMessage->get(2).asDouble();
        signalDuration = newSignalPropertiesMessage->get(3).asDouble();
        break;

        case SIG_TRIANGLE:
        /*
            Need to implement these signals...
        */
        signalAmplitude = newSignalPropertiesMessage->get(1).asDouble();
        signalStartTime = newSignalPropertiesMessage->get(2).asDouble();
        signalDuration = newSignalPropertiesMessage->get(3).asDouble();
        break;

        case SIG_SQUARE:
        /*
            Need to implement these signals...
        */
        signalAmplitude = newSignalPropertiesMessage->get(1).asDouble();
        signalStartTime = newSignalPropertiesMessage->get(2).asDouble();
        signalDuration = newSignalPropertiesMessage->get(3).asDouble();
        break;

        case SIG_DIRAC:
        /*
            Need to implement these signals...
        */
        signalAmplitude = newSignalPropertiesMessage->get(1).asDouble();
        signalStartTime = newSignalPropertiesMessage->get(2).asDouble();
        signalDuration = newSignalPropertiesMessage->get(3).asDouble();
        break;

        default:
        signalAmplitude = newSignalPropertiesMessage->get(1).asDouble();
        signalStartTime = newSignalPropertiesMessage->get(2).asDouble();
        signalDuration = newSignalPropertiesMessage->get(3).asDouble();
        break;
    }
    std::cout << "Received new signal properties:" << std::endl;
    std::cout << "Signal Amplitude = " << signalAmplitude << std::endl;
    std::cout << "Signal Start Time = " << signalStartTime << std::endl;
    std::cout << "Signal Duration  = " << signalDuration << std::endl;
}

void CtrlThread::updatePidInformation()
{
    Pid* currentPid;
    if (iPids[partIndex]==NULL) {
        std::cout << "no iPid device" << std::endl;
    }

    bool res;
    if(isPositionMode){
        res = iPids[partIndex]->getPid(jointIndex, currentPid);
    }
    else if (isVelocityMode) {
        res = iVel[partIndex]->getVelPid(jointIndex, currentPid);
    }
    else if (isTorqueMode) {
        res = iTrq[partIndex]->getTorquePid(jointIndex, currentPid);
    }



    if(res)
    {
        Kp_thread = currentPid->kp;
        Kd_thread = currentPid->kd;
        Ki_thread = currentPid->ki;
    }
    else{std::cout << "[ERROR] Couldn't retrieve PID from part "<<partIndex << ", joint " << jointIndex<< std::endl;}

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
/*
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
*/


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

void CtrlThread::resizeDataVectors()
{
    data_time.clear();
    data_input.clear();
    data_response.clear();

    int storageSize = 200;
    data_time.resize(storageSize);
    data_input.resize(storageSize);
    data_response.resize(storageSize);


    iterationCounter = 0;

    dataReadyForDelivery = false;

}

void CtrlThread::finalizeDataVectors()
{
    data_time.resize(iterationCounter);
    data_input.resize(iterationCounter);
    data_response.resize(iterationCounter);

    if (isVelocityMode) {
        Vector dq = data_response.subVector(1,data_response.size()-1) - data_response.subVector(0,data_response.size()-2);
        Vector dt = data_time.subVector(1,data_time.size()-1) - data_time.subVector(0,data_time.size()-2);

        data_response(0) = 0.0;
        data_response.subVector(1,data_response.size()-1) = dq / dt;
    }

    dataReadyForDelivery = true;
}

bool CtrlThread::excitationSignal(double &cmd)
{

    if (applyExcitationSignal)
    { // need to implement different signal types
        double relativeT = Time::now() - triggerTime;


        data_time[iterationCounter] = relativeT;

        if (relativeT<signalStartTime)
        {
            if(isPositionMode)
                cmd = homeVectors[partIndex][jointIndex];

            else if(isVelocityMode)
                cmd = 0.0;

            else if(isTorqueMode)
                cmd = stationaryTorque;

            return true;
        }
        else if (relativeT>=signalStartTime && relativeT<(signalStartTime+signalDuration))
        {
            if(isPositionMode)
                cmd = homeVectors[partIndex][jointIndex] + signalAmplitude;

            else if(isVelocityMode)
                cmd = signalAmplitude;

            else if(isTorqueMode)
                cmd = stationaryTorque + signalAmplitude;

            return true;
        }
        else if (relativeT>=(signalStartTime+signalDuration))
        {

            if(isPositionMode)
                cmd = homeVectors[partIndex][jointIndex];

            else if(isVelocityMode)
                cmd = 0.0;

            else if(isTorqueMode)
                cmd = stationaryTorque;

            finalizeDataVectors();
            applyExcitationSignal = false;
            return true;
        }


    }

    else
    {
        return false;
    }
}

double CtrlThread::getJointResponse()
{
    if (isPositionMode || isVelocityMode)
    {
        while(!iEnc[partIndex]->getEncoders(encoders[partIndex].data()) )
        {
            std::cout << "Waiting on Encoders..." << std::endl;
            Time::delay(0.001);
        }
        return encoders[partIndex][jointIndex];
    }

    else if (isTorqueMode)
    {
        double measuredTorque;
        while(!iTrq[partIndex]->getTorque(jointIndex, &measuredTorque) )
        {
            Time::delay(0.001);
        }
        return measuredTorque;
    }
}


void CtrlThread::sendDataToGui()
{
    /*  To get a vector using yarp ports we have to use a little workaround basically
        the data is saved in a yarp vector  then when we are ready to send each entry
        is sent individually. The first value is an int (0 or 1) which indicates to
        the receiver when to listen. 0 = stop listening and 1 = start. The size of the
        vector is inferred from the number of messages sent with a first value of 1.
    */
    std::cout << "sending data" << std::endl;

    for (int i=0; i<iterationCounter; i++)
    {
        Bottle dataBottle_out;
        dataBottle_out.clear();

        dataBottle_out.addInt(1);

        dataBottle_out.addDouble(data_time[i]);
        dataBottle_out.addDouble(data_input[i]);
        dataBottle_out.addDouble(data_response[i]);

        // dataBottle_out.addList().read(data_time);
        // dataBottle_out.addList().read(data_input);
        // dataBottle_out.addList().read(data_response);


        dataPort_out.write(dataBottle_out);
    }
    Bottle dataBottle_out;
    dataBottle_out.clear();
    dataBottle_out.addInt(0);
    dataPort_out.write(dataBottle_out);

    std::cout << "data sent\n-----\n" << std::endl;
}
