#include "module.h"
#include <yarp/os/Network.h>


using namespace yarp::os;
using namespace yarp::dev;

// YARP_DECLARE_DEVICES(icubmod)

bool CtrlModule::configure(ResourceFinder &rf)
{
    Time::turboBoost();

    ConstString robotName=rf.find("robot").asString();
    std::cout << robotName.c_str() << std::endl;
    thr=new CtrlThread(CTRL_THREAD_PER, robotName);
    if (!thr->start())
    {
        delete thr;
        return false;
    }

    return true;
}

bool CtrlModule::close()
{
    thr->stop();
    delete thr;

    return true;
}

double CtrlModule::getPeriod()
{
    return 1.0;
}


bool CtrlModule::updateModule()
{
    if (thr->isFinished())
    {
        std::cout << "\n\n ------------------- \n Tuning Finished! \n -------------------\n\n";
        return false;
    }

    else{return true;}
}




int main(int argc, char *argv[])
{
    // we need to initialize the drivers list
    YARP_REGISTER_DEVICES(icubmod)


    Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"Error: yarp server does not seem available\n");
        return -1;
    }

    CtrlModule mod;

    ResourceFinder rf;
    return mod.runModule(rf);
}
