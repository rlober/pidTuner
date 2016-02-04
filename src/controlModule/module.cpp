#include "controlModule/module.h"



using namespace yarp::os;
using namespace yarp::dev;

// YARP_DECLARE_DEVICES(icubmod)

bool CtrlModule::configure(ResourceFinder &rf)
{
    Time::turboBoost();

    if( rf.check("robot") )
    {
        robotName = rf.find("robot").asString().c_str();
        yarp::os::Log().info() << "Robot name is: " << robotName;
    }

    if( rf.check("exclude") )
    {
        excludedPart = rf.find("exclude").asString().c_str();
        yarp::os::Log().info() << "Excluding: " << excludedPart;
    }

    bool isUsingJtc = false;
    if( rf.check("jtc") )
    {
        isUsingJtc = true;
        yarp::os::Log().info() << "Using Joint Torque Control";
    }

    int period = 10; //ms
    thr.reset(new CtrlThread(period, robotName, excludedPart, isUsingJtc));
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
        yarp::os::Log().info() << "Possible parameters:";
        yarp::os::Log().info()<< "\t--robot :Robot name. Set to icub by default.";
        yarp::os::Log().info()<< "\t--exclude :A part you wish to exclude. Set to empty by default.";
        yarp::os::Log().info()<< "\t--jtc :using joint torque control.";
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
