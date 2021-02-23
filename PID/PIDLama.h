#ifndef PID_H
#define PID_H

#include "mbed.h"

class PID
{
public :
        PID(double p , double i , double d , double _Ts) ;
        double createpwm( double setpoint , double feedback , double pwmMax) ;

private :
        double Kp ;
        double Kd ;
        double Ki ;
        double Ts ;
        double e2;
        double e1;
        double e0;
        double u2;
        double u1;
        double u0;
};
#endif

