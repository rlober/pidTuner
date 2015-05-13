#include "thread.h"


 


CtrlThread::CtrlThread(const double period) : RateThread(int(period*1000.0))
{
    //do nothing
}



bool CtrlThread::threadInit()
{
    robotName = "icubSim";
    extension = ".txt";
    baseFilePath = "/home/ryan/Desktop/";
    // createPidLog();

    robotParts.push_back("head");
    robotParts.push_back("torso");
    robotParts.push_back("left_arm");
    robotParts.push_back("right_arm");
    robotParts.push_back("left_leg");
    robotParts.push_back("right_leg");

    numRobotParts = robotParts.size();

    // robotDevice.resize(numRobotParts);
    iEnc.resize(numRobotParts);
    iPos.resize(numRobotParts);
    iVel.resize(numRobotParts);
    iTrq.resize(numRobotParts);


    encoders.resize(numRobotParts);
    command.resize(numRobotParts);
    tmp.resize(numRobotParts);
    homeVectors.resize(numRobotParts);

    openInterfaces();

       
    

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
    
    
    std::cout<< "TTTest\n";

    // // Update encoders
    // encs_LeftArm->getEncoders(encoders_LeftArm.data());
    // encs_Head->getEncoders(encoders_Head.data());
    // writeToPidLog();
        
}

/********************************************************/

void CtrlThread::threadRelease()
{    
    
    
    std::cout << "\n\n\t<--------------->\n";
    while(!goToHome())
    {
        std::cout << "Going to Home Pose\n";
        Time::delay(0.1);
    }

    for (int rp=0; rp<numRobotParts; rp++)
    {
        robotDevice[rp]->close();
    }

}

bool CtrlThread::openInterfaces()
{   
    std::cout << "\n\nOpening Interfaces for:\n";
    for (int rp=0; rp<numRobotParts; rp++)
    {
        std::string curRobPart = robotParts[rp];

        std::cout << curRobPart << std::endl;

        Property options;
        options.put("device", "remote_controlboard");
        options.put("local", "/"+curRobPart+"/client");   //local port names
        options.put("remote", "/"+robotName+"/"+curRobPart);         //where we connect to

        // create a device
        

        // if (!robotDevice[rp]->open(options))
        //     return false;

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

        if (!ok) {
            printf("Problems acquiring interfaces\n");
            return false;
        }
        std::cout << "test..." << std::endl;

        nJoints[rp]=0;
        iPos[rp]->getAxes(&nJoints[rp]);
        
        encoders[rp].resize(nJoints[rp]);
        tmp[rp].resize(nJoints[rp]);
        command[rp].resize(nJoints[rp]);
        
        
        for (int i = 0; i < nJoints[rp]; i++) {
             tmp[rp][i] = 50.0;
        }
        iPos[rp]->setRefAccelerations(tmp[rp].data());

        for (int i = 0; i < nJoints[rp]; i++) {
            tmp[rp][i] = 10.0;
            iPos[rp]->setRefSpeed(i, tmp[rp][i]);
        }

        homeVectors[rp].resize(nJoints[rp]);
        iEnc[rp]->getEncoders(homeVectors[rp].data());
    }
    return true;
}


bool CtrlThread::goToHome()
{
    for (int rp=0; rp<numRobotParts; rp++)
    {
        currentRobotPart = robotParts[rp];
        
        iPos[rp]->positionMove(homeVectors[rp].data());

        bool moveDone = false;
        while(!moveDone)
        {
            Time::delay(0.1);
            iPos[rp]->checkMotionDone(&moveDone);
        }

    }
}




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
    return false;
}





