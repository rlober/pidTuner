#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/all.h>


#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>


#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>


#include <gsl/gsl_math.h>

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <time.h>
#include <sstream>
#include <boost/filesystem.hpp>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ControlBoardPid.h>
#include <yarp/dev/IVelocityControl2.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IControlMode2.h>



YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


class CtrlThread: public RateThread
{
    protected:
        std::vector<PolyDriver*>            robotDevice;
        std::vector<IEncoders*>             iEnc;
        std::vector<IPositionControl*>      iPos;
        std::vector<IVelocityControl2*>     iVel;
        std::vector<ITorqueControl*>        iTrq;
        std::vector<IControlLimits*>        iLims;
        std::vector<IPidControl*>           iPids;
        std::vector<IControlMode2*>         iCtrl;



        std::vector<Vector>     encoders;
        std::vector<Vector>     command;
        std::vector<Vector>     tmp;
        std::vector<Vector>     homeVectors;
        std::vector<Vector>     jointLimitsLower;
        std::vector<Vector>     jointLimitsUpper;

        std::vector<int> nJoints;

        yarp::dev::IControlMode2 *iMode;

        std::vector<std::string> robotParts;

        std::string currentRobotPart, robotName, baseFilePath, extension, pidLogFilePath;

        std::ofstream pidFile;
        int numRobotParts;

        bool isPositionMode, isVelocityMode, isTorqueMode, jointCommandsHaveBeenUpdated, controlThreadFinished;

        int partIndex, jointIndex;


        double stationaryTorque;







    public:
        CtrlThread(const double period);

        virtual bool threadInit();

        virtual void afterStart(bool s);

        virtual void run();

        virtual void threadRelease();

        bool openInterfaces();

        bool goToHome();
        bool sendJointCommand(double cmd);
        void setCommandToHome();


        // void createPidLog();
        // void writeToPidLog();

        const std::string currentDateTime();

        bool isFinished();

        bool applyExcitationSignal;

        double signalAmplitude, triggerTime, signalStartTime, signalDuration;

        Vector data_time;
        Vector data_input;
        Vector data_response;

        int iterationCounter;
        bool dataReadyForDelivery;



    private:
        double Kp_thread, Kd_thread, Ki_thread;
        void sendPidGains();
        void parseIncomingGains(Bottle *newGainMessage);
        void parseIncomingControlMode(Bottle *newControlModeMessage);
        void parseIncomingSignalProperties(Bottle *newControlModeMessage);
        void updatePidInformation();
        bool excitationSignal(double &cmd);
        void resizeDataVectors();
        double getJointResponse();
        void finalizeDataVectors();
        void sendDataToGui();

        BufferedPort<Bottle>    gainsBufPort_in; // incoming new gains
        Port                    gainsPort_out; // outgoing current gains

        BufferedPort<Bottle>    goToHomeBufPort_in; // incoming gotToHome Command

        BufferedPort<Bottle>    robotPartAndJointBufPort_in; //incoming part and joint index

        BufferedPort<Bottle>    controlModeBufPort_in;

        Port                    dataPort_out;

        BufferedPort<Bottle>    signalPropertiesBufPort_in;

};
