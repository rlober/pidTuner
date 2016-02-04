#ifndef GENERICPID_H
#define GENERICPID_H

#include <iostream>
#include <yarp/os/Bottle.h>
#include <yarp/os/Log.h>
#include <MessageVocabulary.h>
#include <ostream>


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

    void clearPidValues()
    {
        Kp = Kd = Ki = Kff = max_int = scale = max_output = offset = stiction_up = stiction_down = bemf = coulombVelThresh = frictionCompensation = bemf_scale = ktau = ktau_scale = 0.0;
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
                bottle.addDouble(ktau);
                bottle.addDouble(ktau_scale);
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
                ktau        = bottle.get(12).asDouble();
                ktau_scale  = bottle.get(13).asDouble();
            }
        }
        return true;

    }



    friend std::ostream& operator<<(std::ostream &out, const GenericPid& pid)
    {
        out << pid.Kp << " ";
        out << pid.Kd << " ";
        out << pid.Ki << " ";
        out << pid.Kff << " ";
        out << pid.max_int << " ";
        out << pid.scale << " ";
        out << pid.max_output << " ";
        out << pid.offset << " ";
        out << pid.stiction_up << " ";
        out << pid.stiction_down << " ";
        out << pid.bemf << " ";
        out << pid.coulombVelThresh << " ";
        out << pid.frictionCompensation << " ";
        out << pid.bemf_scale << " ";
        out << pid.ktau << " ";
        out << pid.ktau_scale;
        return out;
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
            ktau,
            ktau_scale;

    ControlMode controlMode;
    bool        usingJTC;

private:
    yarp::os::Log log;



};


#endif // GENERICPID_H
