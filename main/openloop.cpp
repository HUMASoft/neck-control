#include <iostream>
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include "mainlib.h"
#include "math.h"


main ()
{
    SocketCanPort pm1("can0");
    CiA402Device m1 (1, &pm1);
    SocketCanPort pm2("can0");
    CiA402Device m2 (2, &pm2);
    SocketCanPort pm3("can0");
    CiA402Device m3 (3, &pm3);


    TableKinematics a;
    vector<double> lengths(3);
    long orient=1;
    long incli=1;

    a.GetIK(incli,orient,lengths);
    cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;
    double posan1, posan2, posan3;
    posan1=(0.1-lengths[0])*180/(0.01*M_PI);
    posan2=(0.1-lengths[1])*180/(0.01*M_PI);
    posan3=(0.1-lengths[2])*180/(0.01*M_PI);
    cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3;

    m1.Reset();
    m2.Reset();
    m3.Reset();

    m1.SwitchOn();
    m2.SwitchOn();
    m3.SwitchOn();

    m1.SetupPositionMode(360,360);
    m2.SetupPositionMode(360,360);
    m3.SetupPositionMode(360,360);
    m1.SetPosition(posan1);
    m2.SetPosition(posan2);
    m3.SetPosition(posan3);

     cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3;

}

