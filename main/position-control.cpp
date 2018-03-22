#include <iostream>
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"


main ()
{
    SocketCanPort pm1("can0");
    CiA402Device m1 (1, &pm1);
    SocketCanPort pm2("can0");
    CiA402Device m2 (2, &pm2);
    SocketCanPort pm3("can0");
    CiA402Device m3 (3, &pm3);

    m1.SwitchOn();


    m1.SetupPositionMode(360,360);
    m1.SetPosition(50);

}
