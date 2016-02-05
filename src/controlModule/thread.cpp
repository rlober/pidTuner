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



#include "controlModule/thread.h"


using namespace boost;
using namespace boost::filesystem;
using namespace std;

CtrlThread::CtrlThread(const int period, const std::string Robot_Name, const std::string Excluded_Part, bool isUsingJtc) :
    RateThread(period),
    robotName(Robot_Name),
    excludedPart(Excluded_Part),
    usingJTC(isUsingJtc),
    rpcCallback(*this),
    testControlMode(POSITION_MODE)
{
    if (robotName.size()==0) {
        log.info() << " [WARNING] Robot name was not initialized. Defaulting to robotName=icubGazeboSim.";
        log.info() << " To set the robot name simply use: --robot [name of robot] when launching the pidTunerController. Remember, no brackets around the name of the robot.";

        robotName="icub";
    }
    if(usingJTC)
    {
        robotName += "/jtc";
    }
    baseFilePath = boost::filesystem::current_path().string();
    boost::filesystem::path full_path( baseFilePath );
    log.info() << " Current record path is : "<<baseFilePath;
}

void CtrlThread::setRecordDirectory(const std::string& dir)
{
    if(boost::filesystem::is_directory(dir))
        this->baseFilePath = dir;
    else
        log.error() << " Path "<<dir<<" does not exists";
}

bool CtrlThread::threadInit()
{

    extension = ".txt";



    partIndex = 0;
    jointIndex = 0;

    robotParts.push_back("head");
    robotParts.push_back("torso");
    robotParts.push_back("left_arm");
    robotParts.push_back("right_arm");
    robotParts.push_back("left_leg");
    robotParts.push_back("right_leg");

    if (excludedPart.size()!=0) {
        for(stVecIt=robotParts.begin(); stVecIt!=robotParts.end(); stVecIt++){
            if (excludedPart == *stVecIt){
                robotParts.erase(stVecIt);
            }
        }
    }

    numRobotParts = robotParts.size();
    nJoints.resize(numRobotParts);
    // robotDevice.resize(numRobotParts);
    iEnc.resize(numRobotParts);
    iPos.resize(numRobotParts);
    iPosDirect.resize(numRobotParts);
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

    if(!openInterfaces()) return false;

    if(!goToHome()) return false;

    jointCommandsHaveBeenUpdated = false;
    controlThreadFinished = false;
    applyExcitationSignal = false;
    dataReadyForDelivery = false;
    iterationCounter = 0;
    resizeDataVectors();



    //Open yarp ports
    bool ok = true;

    ok &= dataPort_out.open("/pidTunerController/data:o");
    ok &= rpcServerPort.open("/pidTunerController/rpc:s");
    if(ok){
        rpcServerPort.setReader(rpcCallback);
    }

    return ok;

}

void CtrlThread::afterStart(bool s)
{
    if (s)
        log.info() << "Thread started successfully";
    else
        log.error() << "Thread did not start";
}

/*********************************************************
                        THREAD RUN
**********************************************************/

void CtrlThread::run()
{


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
            if(jointLimitsReached()){
                finalizeDataVectors();
                applyExcitationSignal = false;
                log.warning() << "Joint limit has been reached. Potentially unsafe behavior has been detected so I am stopping this test!\n";
            }

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


    while(!goToHome())
    {
        // log.info() << " Going to Home Pose\n";
        Time::delay(0.1);
    }

    for (int rp=0; rp<numRobotParts; rp++)
    {
        robotDevice[rp]->close();
    }

}

bool CtrlThread::openInterfaces()
{
    log.info() << " Opening Interfaces:";
    for (int rp=0; rp<numRobotParts; rp++)
    {
        std::string curRobPart = robotParts[rp];

        log.info() <<" -- "<<curRobPart;

        Property options;
        options.put("device", "remote_controlboard");
        options.put("local", "/"+curRobPart+"/client");   //local port names
        options.put("remote", "/"+robotName+"/"+curRobPart);         //where we connect to


        PolyDriver *tmp_robotDevice = new PolyDriver(options);
        if (!tmp_robotDevice->isValid()) {
            log.error() << " Device not available.  Here are the known devices: \n" <<Drivers::factory().toString();
            return false;
        }


        robotDevice.push_back(tmp_robotDevice);

        bool ok;

        log.info() << " \nViewing Devices:";
        ok = robotDevice[rp]->view(iPos[rp]);
        log.info() << " iPos created";
        ok = robotDevice[rp]->view(iPosDirect[rp]);
        log.info() << " iPosDirect created";
        ok = ok && robotDevice[rp]->view(iEnc[rp]);
        log.info() << " iEnc created";
        ok = ok && robotDevice[rp]->view(iVel[rp]);
        log.info() << " iVel created";
        ok = ok && robotDevice[rp]->view(iTrq[rp]);
        log.info() << " iTrq created";
        ok = ok && robotDevice[rp]->view(iLims[rp]);
        log.info() << " iLims created";
        ok = ok && robotDevice[rp]->view(iPids[rp]);
        log.info() << " iPids created";
        ok = ok && robotDevice[rp]->view(iCtrl[rp]);
        log.info() << " iCtrl created";




        if (!ok) {
            printf("Problems acquiring interfaces\n");
            return false;
        }


        nJoints[rp]=0;
        iPos[rp]->getAxes(&nJoints[rp]);
        iPosDirect[rp]->getAxes(&nJoints[rp]);


        encoders[rp].resize(nJoints[rp]);
        tmp[rp].resize(nJoints[rp]);
        command[rp].resize(nJoints[rp]);

        jointLimitsLower[rp].resize(nJoints[rp]);
        jointLimitsUpper[rp].resize(nJoints[rp]);
        homeVectors[rp].resize(nJoints[rp]);


        for (int i = 0; i < nJoints[rp]; i++) {
             tmp[rp][i] = 30.0;
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


        double timeout = 2.0; //seconds
        double delayTime = 0.01; //seconds
        double timeWaiting = 0.0; //seconds
        while(!iEnc[rp]->getEncoders(command[rp].data()) && timeWaiting <= timeout  )
        {
            Time::delay(delayTime);
            timeWaiting += delayTime;
            if (timeWaiting > timeout) {
                log.error() << "(line "<< __LINE__<< ") Timeout while waiting for "<< curRobPart <<" encoder data. Skipping.";
            }
        }




    }
    jointPositionVectors = homeVectors;
    return true;
}


bool CtrlThread::goToHome()
{
    log.info() << "Moving to home position.";


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
    log.info() << "Finished.";
    return true;
}

bool CtrlThread::sendJointCommand(double cmd)
{
    bool retVal = false;
    switch (testControlMode) {
        case POSITION_MODE:
            iPosDirect[partIndex]->setPosition(jointIndex, cmd);
            retVal = true;
            break;
        case VELOCITY_MODE:
            iVel[partIndex]->velocityMove(1, &jointIndex, &cmd);
            retVal = true;
            break;
        case TORQUE_MODE:
            iTrq[partIndex]->setRefTorque(jointIndex, cmd);
            retVal = true;
            break;
    }
    return retVal;

}

void CtrlThread::parseNewPid()
{

    yarp::dev::Pid newPid;
    yarp::dev::MotorTorqueParameters trqParams;
    yarpJtcParameters jtcParams;

    threadPid.parseToYarpParams(newPid, trqParams, jtcParams);

    if (iPids[partIndex]==NULL)
    {
        log.error() << " there is no iPid pointer here...";
    }
    else
    {
        switch (testControlMode) {
            case POSITION_MODE:
                if (!iPids[partIndex]->setPid(jointIndex, newPid)) {
                    log.error()<<"Position PID send failed...";
                }
                while(!iEnc[partIndex]->getEncoder(jointIndex, &jointPositionVectors[partIndex][jointIndex] ))
                {
                    Time::delay(0.001);
                }
                iCtrl[partIndex]->setControlMode(jointIndex, VOCAB_CM_POSITION_DIRECT);
                iPos[partIndex]->setRefSpeed(jointIndex, 10.0);
                break;

            case VELOCITY_MODE:
                if (!iVel[partIndex]->setVelPid(jointIndex, newPid)) {
                    log.error()<<"Velocity PID send failed...";
                }
                iCtrl[partIndex]->setControlMode(jointIndex, VOCAB_CM_VELOCITY);
                break;

            case TORQUE_MODE:
                if(usingJTC){
                    iTrq[partIndex]->setBemfParam(jointIndex, jtcParams.bemf);
                    // TODO: implement JTC param setters here.
                }else{
                    iTrq[partIndex]->setMotorTorqueParams(jointIndex, trqParams);
                }

                if (!iTrq[partIndex]->setTorquePid(jointIndex, newPid)) {
                    log.error()<<"Torque PID send failed...";
                }
                while(!iTrq[partIndex]->getTorque(jointIndex, &stationaryTorque) )
                {
                    Time::delay(0.001);
                }
                iCtrl[partIndex]->setControlMode(jointIndex, VOCAB_CM_TORQUE);
                break;
        }

    }
    updatePidInformation();


}

void CtrlThread::parseIncomingControlMode(Bottle *newControlModeMessage)
{
    testControlMode = static_cast<ControlMode>(newControlModeMessage->get(1).asInt());
    switch (testControlMode) {
        case POSITION_MODE:
            log.info() << " Switching to POSITION control.";
            break;

        case VELOCITY_MODE:
            log.info() << " Switching to VELOCITY control.";
            break;

        case TORQUE_MODE:
            log.info() << " Switching to TORQUE control.";
            break;

        default:
            log.warning() << " Defaulting to POSITION control.";
            break;
    }

    /*
        Need to implement control mode switch here...

     */
}

void CtrlThread::bottleSignalProperties(Bottle* bottle)
{
    bottle->clear();
    bottle->addInt(testSignalType);
    bottle->addDouble(signalAmplitude);
    bottle->addDouble(signalStartTime);
    bottle->addDouble(signalDuration);
}

void CtrlThread::parseIncomingSignalProperties(Bottle *newSignalPropertiesMessage)
{
    testSignalType = static_cast<SignalType>(newSignalPropertiesMessage->get(1).asInt());
    signalAmplitude = newSignalPropertiesMessage->get(2).asDouble();
    signalStartTime = newSignalPropertiesMessage->get(3).asDouble();
    signalDuration = newSignalPropertiesMessage->get(4).asDouble();

    switch (testSignalType) {
        case STEP:
            log.info() << "Got new signal type: STEP";
            break;

        case SIGN:
            log.info() << "Got new signal type: SIGN";
            /*
                Need to implement these signals...
            */
            break;

        case TRIANGLE:
            log.info() << "Got new signal type: TRIANGLE";
            /*
                Need to implement these signals...
            */
            break;

        case SQUARE:
            log.info() << "Got new signal type: SQUARE";
            /*
                Need to implement these signals...
            */
            break;

        case DIRAC:
            log.info() << "Got new signal type: DIRAC";
            /*
                Need to implement these signals...
            */
            break;

        default:
            break;
    }
    log.info() << " Received new signal properties:";
    log.info() << " -- Signal Amplitude = " << signalAmplitude;
    log.info() << " -- Signal Start Time = " << signalStartTime;
    log.info() << " -- Signal Duration  = " << signalDuration;
}

void CtrlThread::updatePidInformation()
{
    Pid currentPid;

    if (iPids[partIndex]==NULL) {
        log.error() << " no iPid device";
    }
    else{log.info() << " trying to get PID from joint.";}

    bool res;
    switch (testControlMode) {
        case POSITION_MODE:
            res = iPids[partIndex]->getPid(jointIndex, &currentPid);
            break;
        case VELOCITY_MODE:
            res = iVel[partIndex]->getVelPid(jointIndex, &currentPid);
            break;
        case TORQUE_MODE:
            res = iTrq[partIndex]->getTorquePid(jointIndex, &currentPid);
            break;
    }

    if(res)
    {
        threadPid.setControlMode(testControlMode, usingJTC);

        threadPid.Kp = currentPid.kp;
        threadPid.Kd = currentPid.kd;
        threadPid.Ki = currentPid.ki;
        threadPid.Kff = currentPid.kff;
        threadPid.max_int = currentPid.max_int;
        threadPid.scale = currentPid.scale;
        threadPid.max_output = currentPid.max_output;
        threadPid.offset = currentPid.offset;
        threadPid.stiction_up = currentPid.stiction_up_val;
        threadPid.stiction_down = currentPid.stiction_down_val;
        if (testControlMode == TORQUE_MODE) {
            yarp::dev::MotorTorqueParameters trqParams;
            iTrq[partIndex]->getMotorTorqueParams(jointIndex, &trqParams);
            threadPid.bemf = trqParams.bemf;
            if(usingJTC){
                // TODO: implement getters here.
                threadPid.coulombVelThresh = 0.0;
                threadPid.frictionCompensation = 0.0;
            }else{
                threadPid.bemf_scale = trqParams.bemf_scale;
                threadPid.Ktau = trqParams.ktau;
                threadPid.Ktau_scale = trqParams.ktau_scale;
            }
        }
    }
    else{log.error() << " Couldn't retrieve PID from part "<<partIndex << ", joint " << jointIndex;}

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

void CtrlThread::resizeDataVectors()
{
    data_time.clear();
    data_input.clear();
    data_response.clear();

    int storageSize = 200*signalDuration; //100 thread runs per second * signalDuration * 2 for safety
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
            switch (testControlMode) {
                case POSITION_MODE:
                    cmd = jointPositionVectors[partIndex][jointIndex];
                    break;
                case VELOCITY_MODE:
                    cmd = 0.0;
                    break;
                case TORQUE_MODE:
                    cmd = stationaryTorque;
                    break;
            }
            return true;
        }
        else if (relativeT>=signalStartTime && relativeT<(signalStartTime+signalDuration))
        {
            switch (testControlMode) {
                case POSITION_MODE:
                    cmd = jointPositionVectors[partIndex][jointIndex] + signalAmplitude;
                    break;
                case VELOCITY_MODE:
                    cmd = signalAmplitude;
                    break;
                case TORQUE_MODE:
                    cmd = stationaryTorque + signalAmplitude;
                    break;
            }
            return true;
        }
        else if (relativeT>=(signalStartTime+signalDuration))
        {
            switch (testControlMode) {
                case POSITION_MODE:
                    cmd = jointPositionVectors[partIndex][jointIndex];
                    break;
                case VELOCITY_MODE:
                    cmd = 0.0;
                    break;
                case TORQUE_MODE:
                    cmd = stationaryTorque;
                    break;
            }
            finalizeDataVectors();
            applyExcitationSignal = false;
            iCtrl[partIndex]->setControlMode(jointIndex, VOCAB_CM_POSITION);
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
    if (testControlMode == POSITION_MODE || testControlMode == VELOCITY_MODE)
    {
        while(!iEnc[partIndex]->getEncoders(encoders[partIndex].data()) )
        {
            log.info() << " Waiting on Encoders...";
            Time::delay(0.001);
        }
        return encoders[partIndex][jointIndex];
    }

    else if (testControlMode == TORQUE_MODE)
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
    log.info() << "Sending test data.";

    for (int i=0; i<iterationCounter; i++)
    {
        Bottle dataBottle_out;
        dataBottle_out.clear();

        dataBottle_out.addInt(1);

        dataBottle_out.addDouble(data_time[i]);
        dataBottle_out.addDouble(data_input[i]);
        dataBottle_out.addDouble(data_response[i]);

        dataPort_out.write(dataBottle_out);
    }
    Bottle dataBottle_out;
    dataBottle_out.clear();
    dataBottle_out.addInt(0);
    dataPort_out.write(dataBottle_out);

    log.info() << "Sent" << iterationCounter << "data points.";
}


bool CtrlThread::jointLimitsReached()
{
    while(!iEnc[partIndex]->getEncoders(encoders[partIndex].data())){Time::delay(0.001);}

    if(encoders[partIndex][jointIndex]<=jointLimitsLower[partIndex][jointIndex]){return true;}
    else if (encoders[partIndex][jointIndex]>=jointLimitsUpper[partIndex][jointIndex]){return true;}
    else{return false;}


}


/**************************************************************************************************
                                    Nested PortReader Class
**************************************************************************************************/
CtrlThread::RpcPortCallback::RpcPortCallback(CtrlThread& ctThreadRef):ctThread(ctThreadRef)
{
    //do nothing
}

bool CtrlThread::RpcPortCallback::read(ConnectionReader& connection)
{
    Bottle input, reply;
    bool ok = input.read(connection);
    if (!ok)
        return false;

    else{
        ctThread.parseRpcMessage(&input, &reply);
        ConnectionWriter *returnToSender = connection.getWriter();
        if (returnToSender!=NULL) {
            reply.write(*returnToSender);
        }
        return true;
    }
}
/**************************************************************************************************
**************************************************************************************************/


void CtrlThread::parseRpcMessage(Bottle *input, Bottle *reply)
{
    int tag = input->get(0).asInt();
    yarp::os::Bottle tmpBtl;
    tmpBtl.copy(*input,1,-1);
    switch (tag) {

        case SET_CONTROL_MODE:
            log.info() << "SET_CONTROL_MODE";
            parseIncomingControlMode(input);
            updatePidInformation();
            reply->clear();
            reply->addInt(1); // Change this to check check that the control mode was set.
            break;

        case GET_CONTROL_MODE:
            log.info() << "GET_CONTROL_MODE";
            reply->clear();
            reply->addInt(testControlMode);
            break;

        case SET_PID_VALUES:
            // Check if new gains have come in or if the user wants the current gains
            log.info() << "SET_PID_VALUES";
            resizeDataVectors();
            threadPid.extractFromBottle(tmpBtl);
            parseNewPid();
            initializeTest();
            threadPid.putInBottle(*reply);
            break;
            
        case SET_PID_VALUES_WITHOUT_TEST:
            // Check if new gains have come in or if the user wants the current gains
            log.info() << "SET_PID_VALUES_WITHOUT_TEST";
            threadPid.extractFromBottle(tmpBtl);
            parseNewPid();
            threadPid.putInBottle(*reply);
            break;

        case GET_PID_VALUES:
            log.info() << "GET_PID_VALUES";
            updatePidInformation();
            threadPid.putInBottle(*reply);
            break;

        case SET_PART_AND_JOINT_INDEXES:
            log.info() << "SET_PART_AND_JOINT_INDEXES";
            partIndex = input->get(1).asInt();
            jointIndex = input->get(2).asInt();
            updatePidInformation();
            reply->clear();
            reply->addInt(1); // Change this to check check that the control mode was set.
            break;

        case GET_PART_AND_JOINT_INDEXES:
            log.info() << "GET_PART_AND_JOINT_INDEXES";
            reply->clear();
            reply->addInt(partIndex);
            reply->addInt(jointIndex);
            break;

        case SET_SIGNAL_PROPERTIES:
            log.info() << "SET_SIGNAL_PROPERTIES";
            parseIncomingSignalProperties(input);
            bottleSignalProperties(reply);
            break;

        case GET_SIGNAL_PROPERTIES:
            log.info() << "GET_SIGNAL_PROPERTIES";
            bottleSignalProperties(reply);
            break;

        case GO_TO_HOME_POSTURE:
            log.info() << "GO_TO_HOME_POSTURE";
            goToHome();
            reply->clear();
            reply->addInt(1); // Change this to check check that the control mode was set.
            break;

        case CHECK_IF_USING_JTC:
            log.info() << "CHECK_IF_USING_JTC";
            reply->clear();
            reply->addInt(usingJTC); // Change this to check check that the control mode was set.
            break;

    }
}

void CtrlThread::initializeTest()
{
    triggerTime = Time::now();
    applyExcitationSignal = true;
}
