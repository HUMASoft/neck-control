#include <iostream>
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include "mainlib.h"
#include "math.h"




int main ()
{

    SocketCanPort pm1("can1");
    CiA402Device m1 (1, &pm1);
    SocketCanPort pm2("can1");
    CiA402Device m2 (2, &pm2);
    SocketCanPort pm3("can1");
    CiA402Device m3 (3, &pm3);


        m1.Reset();
        m1.SwitchOn();
        m1.SetupPositionMode(360,360);

    m2.Reset();
    m2.SwitchOn();
    m2.SetupPositionMode(360,360);

    m3.Reset();
    m3.SwitchOn();
    m3.SetupPositionMode(360,360);


    return 0;
}
