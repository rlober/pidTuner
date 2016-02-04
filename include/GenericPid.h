#ifndef GENERICPID_H
#define GENERICPID_H

#include <iostream>
#include <yarp/os/Bottle.h>
#include <yarp/os/Log.h>
#include <MessageVocabulary.h>
#include <ostream>
#include <sstream>

#ifndef POSITION_MODE_BOTTLE_SIZE
#define POSITION_MODE_BOTTLE_SIZE 10
#endif

#ifndef VELOCITY_MODE_BOTTLE_SIZE
#define VELOCITY_MODE_BOTTLE_SIZE 10
#endif

#ifndef TORQUE_JTC_MODE_BOTTLE_SIZE
#define TORQUE_JTC_MODE_BOTTLE_SIZE 13
#endif

#ifndef TORQUE_MODE_BOTTLE_SIZE
#define TORQUE_MODE_BOTTLE_SIZE 14
#endif





class GenericPid{

public:
    //Constructor
    GenericPid():
        controlMode(POSITION_MODE),
        usingJTC(false)
    {
        clearPidValues();
    }

    //Copy Constructor
    GenericPid(const GenericPid& source)
    {
        copyPidData(source);
    }

    // assignment operator
    GenericPid& operator = (const GenericPid& source)
    {
        copyPidData(source);
        return *this;
    }

    void copyPidData(const GenericPid& source)
    {
        Kp = source.Kp; Kd = source.Kd; Ki = source.Ki; Kff = source.Kff; max_int = source.max_int; scale = source.scale; max_output = source.max_output; offset = source.offset; stiction_up = source.stiction_up; stiction_down = source.stiction_down; bemf = source.bemf; coulombVelThresh = source.coulombVelThresh; frictionCompensation = source.frictionCompensation; bemf_scale = source.bemf_scale; Ktau = source.Ktau; Ktau_scale = source.Ktau_scale;

        controlMode = source.controlMode;
        usingJTC = source.usingJTC;
    }

    void clearPidValues()
    {
        Kp = Kd = Ki = Kff = max_int = scale = max_output = offset = stiction_up = stiction_down = bemf = coulombVelThresh = frictionCompensation = bemf_scale = Ktau = Ktau_scale = 0.0;
    }

    void setControlMode(const ControlMode& newCtMode, bool isUsingJtc=false)
    {
        controlMode = newCtMode;
        usingJTC = isUsingJtc;
    }

    void putInBottle(yarp::os::Bottle& bottle)
    {
        bottle.addDouble(Kp);
        bottle.addDouble(Kd);
        bottle.addDouble(Ki);
        bottle.addDouble(Kff);
        bottle.addDouble(max_int);
        bottle.addDouble(scale);
        bottle.addDouble(max_output);
        bottle.addDouble(offset);
        bottle.addDouble(stiction_up);
        bottle.addDouble(stiction_down);
        if(controlMode == TORQUE_MODE){
            bottle.addDouble(bemf);
            if(usingJTC){
                bottle.addDouble(coulombVelThresh);
                bottle.addDouble(frictionCompensation);
            }else{
                bottle.addDouble(bemf_scale);
                bottle.addDouble(Ktau);
                bottle.addDouble(Ktau_scale);
            }
        }
    }
    bool extractFromBottle(yarp::os::Bottle& bottle){
        switch (controlMode)
        {
            case POSITION_MODE:
                if(bottle.size()!=POSITION_MODE_BOTTLE_SIZE){
                    log.error() << "Bottle is not the right size for POSITION_MODE";
                    return false;
                }
                break;
            case VELOCITY_MODE:
                if(bottle.size()!=VELOCITY_MODE_BOTTLE_SIZE){
                    log.error() << "Bottle is not the right size for VELOCITY_MODE";
                    return false;
                }
                break;
            case TORQUE_MODE:
                if (usingJTC) {
                    if(bottle.size()!=TORQUE_JTC_MODE_BOTTLE_SIZE){
                        log.error() << "Bottle is not the right size for TORQUE_MODE using JTC";
                        return false;
                    }
                }
                else {
                    if(bottle.size()!=TORQUE_MODE_BOTTLE_SIZE){
                        log.error() << "Bottle is not the right size for TORQUE_MODE";
                        return false;
                    }
                }
                break;
        }

        clearPidValues(); // Set everything to 0.0

        Kp              = bottle.get(0).asDouble();
        Kd              = bottle.get(1).asDouble();
        Ki              = bottle.get(2).asDouble();
        Kff             = bottle.get(3).asDouble();
        max_int         = bottle.get(4).asDouble();
        scale           = bottle.get(5).asDouble();
        max_output      = bottle.get(6).asDouble();
        offset          = bottle.get(7).asDouble();
        stiction_up     = bottle.get(8).asDouble();
        stiction_down   = bottle.get(9).asDouble();

        if(controlMode == TORQUE_MODE){
            bemf = bottle.get(10).asDouble();

            if(usingJTC){
                coulombVelThresh        = bottle.get(11).asDouble();
                frictionCompensation    = bottle.get(12).asDouble();
            }else{
                bemf_scale  = bottle.get(11).asDouble();
                Ktau        = bottle.get(12).asDouble();
                Ktau_scale  = bottle.get(13).asDouble();
            }
        }
        return true;

    }



    friend std::ostream& operator<<(std::ostream &out, const GenericPid& pid)
    {
        out << "Pid values:\n";
        out << "Kp = " << pid.Kp << "\n";
        out << "Kd = " << pid.Kd << "\n";
        out << "Ki = " << pid.Ki << "\n";
        out << "Kff = " << pid.Kff << "\n";
        out << "max_int = " << pid.max_int << "\n";
        out << "scale = " << pid.scale << "\n";
        out << "max_output = " << pid.max_output << "\n";
        out << "offset = " << pid.offset << "\n";
        out << "stiction_up = " << pid.stiction_up << "\n";
        out << "stiction_down = " << pid.stiction_down << "\n";
        out << "bemf = " << pid.bemf << "\n";
        out << "coulombVelThresh = " << pid.coulombVelThresh << "\n";
        out << "frictionCompensation = " << pid.frictionCompensation << "\n";
        out << "bemf_scale = " << pid.bemf_scale << "\n";
        out << "Ktau = " << pid.Ktau << "\n";
        out << "Ktau_scale = " << pid.Ktau_scale << "\n";
        return out;
    }

    std::string toString(std::string delimiter=" ")
    {
        std::stringstream pidStream;
        pidStream   << Kp << delimiter
                    << Kd << delimiter
                    << Ki << delimiter
                    << Kff << delimiter
                    << max_int << delimiter
                    << scale << delimiter
                    << max_output << delimiter
                    << offset << delimiter
                    << stiction_up << delimiter
                    << stiction_down << delimiter
                    << bemf << delimiter
                    << coulombVelThresh << delimiter
                    << frictionCompensation << delimiter
                    << bemf_scale << delimiter
                    << Ktau << delimiter
                    << Ktau_scale;

        return pidStream.str();
    }


    double  Kp,
            Kd,
            Ki,
            Kff,
            max_int,
            scale,
            max_output,
            offset,
            stiction_up,
            stiction_down,
            bemf,
            coulombVelThresh,
            frictionCompensation,
            bemf_scale,
            Ktau,
            Ktau_scale;

    ControlMode controlMode;
    bool        usingJTC;

private:
    yarp::os::Log log;



};


#endif // GENERICPID_H
