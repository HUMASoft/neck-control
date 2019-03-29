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
//    m3.Reset();

//    m1.SwitchOn();
//    m2.SwitchOn();
//    m3.SwitchOn();

//    m1.SetupPositionMode(360,360);
//    m2.SetupPositionMode(360,360);
//    m3.SetupPositionMode(360,360);


    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);
    sleep (3);
    return 0;

}
