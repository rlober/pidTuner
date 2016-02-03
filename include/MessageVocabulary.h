#ifndef MESSAGEVOCABULARY_H
#define MESSAGEVOCABULARY_H

enum MessageTag
{
    SET_CONTROL_MODE            = 100,
    GET_CONTROL_MODE            = 101,
    SET_PID_VALUES              = 200,
    GET_PID_VALUES              = 201,
    SET_PART_AND_JOINT_INDEXES  = 300,
    GET_PART_AND_JOINT_INDEXES  = 301,
    SET_SIGNAL_PROPERTIES       = 400,
    GET_SIGNAL_PROPERTIES       = 401,
    GO_TO_HOME_POSTURE          = 500
};

enum ControlMode
{
    POSITION_MODE   = 10,
    VELOCITY_MODE   = 11,
    TORQUE_MODE     = 12
};

enum SignalType
{
    STEP        = 1000,
    SIGN        = 1001,
    TRIANGLE    = 1002,
    SQUARE      = 1003,
    DIRAC       = 1004
};


#endif
