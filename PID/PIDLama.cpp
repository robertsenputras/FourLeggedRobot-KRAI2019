#include "PID.h"


PID :: PID(double p , double i , double d, double _Ts)
{
    Kp = p ; Kd = d ; Ki = i ; Ts = _Ts;
}

double PID::createpwm( double setpoint , double feedback, double pwmMax )
{
    e2 = e1 ;
    e1 = e0 ;
    u2 = u1 ;
    u1 = u0 ;
    e0 = setpoint-feedback;
    //u0 = - (u1*4/3 )  + ( u2/3 )  + Kp*(e0-e1) + Ki*Ts*(e0-e1/3) + Kd*(e0-2*e1+e2);
    u0 = Kp*(e0-e1) + Ki*Ts*(e0-e1/3) + Kd*(e0-2*e1+e2);
    if (u0 >= pwmMax)
    {
        u0 = pwmMax ;
    }
    else if (u0 <= -1*pwmMax)
    {
        u0 = -1*pwmMax;
    }
    return u0 ;
}
