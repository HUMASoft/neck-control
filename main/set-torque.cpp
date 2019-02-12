
#include <iostream>
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"


int main ()
{
    SocketCanPort pm1("can1");
    CiA402Device m1 (1, &pm1);
    SocketCanPort pm2("can1");
    CiA402Device m2 (2, &pm2);
    SocketCanPort pm3("can1");
    CiA402Device m3 (3, &pm3);

    //comment if motors already started
//    m1.Reset();
//    m2.Reset();
    m3.Reset();

//    m1.SwitchOn();
//    m2.SwitchOn();
    m3.SwitchOn();

//    m1.Setup_Torque_Mode();
//    m2.Setup_Torque_Mode();
    m3.Setup_Torque_Mode();


//    m1.SetTorque(0);
//    m2.SetTorque(0);
    m3.SetTorque(2000);
    sleep (2);
    m3.SetTorque(0);

    return 0;

}
