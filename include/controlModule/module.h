#include <yarp/os/RFModule.h>
#include "thread.h"
#include <boost/scoped_ptr.hpp>

#define CTRL_THREAD_PER         0.02    // [s]

using namespace yarp::os;
using namespace yarp::dev;

class CtrlModule: public RFModule
{
    protected:
        boost::scoped_ptr<CtrlThread> thr;
        std::string robotName;
        std::string excludedPart;

    public:
        virtual bool configure(ResourceFinder &rf);

        virtual bool close();

        virtual double getPeriod();

        virtual bool   updateModule() ;
};
