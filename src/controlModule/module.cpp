#include "module.h"
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>


using namespace yarp::os;
using namespace yarp::dev;

// YARP_DECLARE_DEVICES(icubmod)

bool CtrlModule::configure(ResourceFinder &rf)
{
    Time::turboBoost();

    if( rf.check("robot") )
    {
        robotName = rf.find("robot").asString().c_str();
        std::cout << "\n\nRobot name is: " << robotName << "\n" << std::endl;
    }

    if( rf.check("exclude") )
    {
        excludedPart = rf.find("exclude").asString().c_str();
        std::cout << "\n\nExcluding: " << excludedPart << "\n" << std::endl;
    }

    thr.reset(new CtrlThread(CTRL_THREAD_PER, robotName, excludedPart));
    if (!thr->start())
    {
        return false;
    }

    return true;
}

bool CtrlModule::close()
{
    thr->stop();
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
    // YARP_REGISTER_DEVICES(icubmod)
    ResourceFinder rf;
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yarp::os::Log().error() << "Possible parameters \n";
        yarp::os::Log().error()<< "\t--robot :Robot name. Set to icub by default.";
        yarp::os::Log().error()<< "\t--exclude :A part you wish to exclude. Set to empty by default.";
        return 0;
    }


    Network yarp;
    if (!yarp.checkNetwork())
    {
        yarp::os::Log().error() << "Error: yarp server does not seem available";
        return -1;
    }

    CtrlModule mod;

    return mod.runModule(rf);
}
