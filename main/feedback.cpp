
#include <iostream>
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include "mainlib.h"
#include "math.h"

#include "fcontrol.h"




main ()
{
    //Controllers

//    //pi
//    vector<double> npi ={1};
//    vector<double> dpi ={1};

//    //pd
//    vector<double> npd ={1};
//    vector<double> dpd ={1};

    //fpi
    vector<double> npi ={-0.0404, 0.3582, -0.0891, -1.0214, 0.7977};
    vector<double> dpi ={-0.1076, 0.4977, 0.1886, -1.5784, 1.0000};

    //fpd
    vector<double> npd ={-4.7853, 43.4275, -6.6602, -138.0768, 108.2231};
    vector<double> dpd ={0.0653, -0.2690, -0.8054, 0.5132, 1.0000};


    SystemBlock pi1(npi,dpi);
    SystemBlock pd1(npd,dpd);

    SystemBlock pi2(npi,dpi);
    SystemBlock pd2(npd,dpd);

    SystemBlock pi3(npi,dpi);
    SystemBlock pd3(npd,dpd);


    fstream graph("./graph.csv",ios::trunc);
    SocketCanPort pm1("can0");
    CiA402Device m1 (1, &pm1);
    SocketCanPort pm2("can0");
    CiA402Device m2 (2, &pm2);
    SocketCanPort pm3("can0");
    CiA402Device m3 (3, &pm3);


    TableKinematics a;
    vector<double> lengths(3);
    long orient=1;
    long incli=25;

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

//    m1.SetupPositionMode(360,360);
//    m2.SetupPositionMode(360,360);
//    m3.SetupPositionMode(360,360);
    m1.Setup_Torque_Mode();
    m2.Setup_Torque_Mode();
    m3.Setup_Torque_Mode();


    double interval=3;
    double dts=0.01;


    for (int i=0;i<11;i++)
    {

        orient += 30;

        a.GetIK(incli,orient,lengths);
        cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;
        double posan1, posan2, posan3;
        posan1=(0.1-lengths[0])*180/(0.01*M_PI);
        posan2=(0.1-lengths[1])*180/(0.01*M_PI);
        posan3=(0.1-lengths[2])*180/(0.01*M_PI);
        cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3;

        graph << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3 << endl;

        for (double t=0;t<interval; t+=dts)
        {
            m1.SetTorque(pd1.OutputUpdate(pi1.OutputUpdate(posan1-m1.GetPosition())-m1.GetVelocity()));
            m2.SetTorque(pd2.OutputUpdate(pi2.OutputUpdate(posan2-m2.GetPosition())-m2.GetVelocity()));
            m3.SetTorque(pd3.OutputUpdate(pi3.OutputUpdate(posan3-m3.GetPosition())-m3.GetVelocity()));
            usleep(dts*1000000);
            graph << t << " , " << posan1  << " , " << posan2 << " , " << posan3 << endl;

        }
//        m1.SetTorque(0);


    }

    graph.close();

    m1.SetupPositionMode(360,360);
    m2.SetupPositionMode(360,360);
    m3.SetupPositionMode(360,360);
    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);
    sleep(2);

     cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3;

}

