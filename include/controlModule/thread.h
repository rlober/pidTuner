#ifndef THREAD_H
#define THREAD_H

/*!
*  \file       thread.h
*  \brief      CtrlThread class header. This file declares the
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

// #include <yarp/os/Network.h>
// #include <yarp/os/BufferedPort.h>
// #include <yarp/os/Port.h>
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

#include <math.h>
#include <yarp/logger/YarpLogger.h>

#include "ui_mainwindow.h"

#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <cmath>
#include <iostream>

#include <boost/chrono.hpp>
#include <boost/thread.hpp>

#include <MessageVocabulary.h>


#define DEG_TO_RAD M_PI / 180.
#define RAD_TO_DEG 180. / M_PI

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
        std::vector<std::string>::iterator stVecIt;

        std::string currentRobotPart, robotName, baseFilePath, extension, pidLogFilePath;

        std::ofstream pidFile;
        int numRobotParts;

        bool isPositionMode, isVelocityMode, isTorqueMode, jointCommandsHaveBeenUpdated, controlThreadFinished;

        int partIndex, jointIndex;


        double stationaryTorque;

        std::string excludedPart;


        /************** DataProcessor *************/
        class RpcPortCallback : public PortReader {
            private:
                CtrlThread& ctThread;

            public:
                RpcPortCallback(CtrlThread& ctThreadRef);

                virtual bool read(ConnectionReader& connection);
        };
        /************** DataProcessor *************/



    public:
        CtrlThread(const int period, const std::string Robot_Name, const std::string Excluded_Part, bool isUsingJtc=false);

        virtual bool threadInit();

        virtual void afterStart(bool s);

        virtual void run();

        virtual void threadRelease();

        bool openInterfaces();
        void setRecordDirectory(const std::string& dir);
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
        yarp::os::Log log;

        bool usingJTC;
        double  Kp_thread,
                Kd_thread,
                Ki_thread,
                Kff_thread,
                max_int_thread,
                scale_thread,
                max_output_thread,
                offset_thread,
                stiction_up_thread,
                stiction_down_thread,
                bemf_thread,
                coulombVelThresh_thread,
                frictionCompensation_thread;

        void bottlePid(Bottle* bottle);
        void bottleSignalProperties(Bottle* bottle);
        void parseIncomingPid(Bottle *newGainMessage);
        void parseIncomingControlMode(Bottle *newControlModeMessage);
        void parseIncomingSignalProperties(Bottle *newControlModeMessage);
        void updatePidInformation();
        bool excitationSignal(double &cmd);
        void resizeDataVectors();
        double getJointResponse();
        void finalizeDataVectors();
        void sendDataToGui();
        bool jointLimitsReached();
        void parseRpcMessage(Bottle *input, Bottle *reply);


        // BufferedPort<Bottle>    gainsBufPort_in; // incoming new gains
        // Port                    gainsPort_out; // outgoing current gains

        // BufferedPort<Bottle>    goToHomeBufPort_in; // incoming gotToHome Command

        // BufferedPort<Bottle>    robotPartAndJointBufPort_in; //incoming part and joint index

        // BufferedPort<Bottle>    controlModeBufPort_in;

        Port                    dataPort_out;

        // BufferedPort<Bottle>    signalPropertiesBufPort_in;
        RpcPortCallback rpcCallback;
        RpcServer rpcServerPort;

        ControlMode testControlMode;
        SignalType  testSignalType;

};
#endif
