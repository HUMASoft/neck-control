

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
    double dts=0.01;

//        PIDBlock pd1 (12.8,0,0.597,dts);

    //pi: w=5 rad/s, phm=70 deg
    //pd: w=5 rad/s, phm=70 deg
//        PIDBlock pi1 (0.1278262,5.5535135,0,dts);
//        PIDBlock pd1 (5.5535135,0,0.1278262,dts);

//        PIDBlock pi2 (0.1278262,5.5535135,0,dts);
//        PIDBlock pd2 (5.5535135,0,0.1278262,dts);

//        PIDBlock pi3 (0.1278262,5.5535135,0,dts);
//        PIDBlock pd3 (5.5535135,0,0.1278262,dts);


    //pi //tustin=(2/dts)*(z-1)/(z+1)
//        4.35 z + 128.1
//         --------------
//             z - 1
    vector<double> npi ={128.1,4.35};
    vector<double> dpi ={-1,1};

//    //pd

//    13.99 z + 11.61
//    ---------------
//         z + 1

    vector<double> npd ={11.61,13.99};
    vector<double> dpd ={1,1};

        SystemBlock pi1(npi,dpi,dts/2);
        SystemBlock pd1(npd,dpd,2/dts);

        SystemBlock pi2(npi,dpi,dts/25);
        SystemBlock pd2(npd,dpd,2/dts);

        SystemBlock pi3(npi,dpi,dts/2);
        SystemBlock pd3(npd,dpd,2/dts);

//    //fpi
//    vector<double> npi ={-0.0404, 0.3582, -0.0891, -1.0214, 0.7977};
//    vector<double> dpi ={-0.1076, 0.4977, 0.1886, -1.5784, 1.0000};

//    //fpd
//    vector<double> npd ={-4.7853, 43.4275, -6.6602, -138.0768, 108.2231};
//    vector<double> dpd ={0.0653, -0.2690, -0.8054, 0.5132, 1.0000};


//    SystemBlock pi1(npi,dpi,5);
//    SystemBlock pd1(npd,dpd,10);

//    SystemBlock pi2(npi,dpi,5);
//    SystemBlock pd2(npd,dpd,10);

//    SystemBlock pi3(npi,dpi,10);
//    SystemBlock pd3(npd,dpd,10);


    ofstream graph("graph.csv",std::ofstream::out);
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
    cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3 <<endl;

    m1.Reset();
    m2.Reset();
    m3.Reset();

    m1.SwitchOn();
    sleep(1);
    m2.SwitchOn();
    sleep(1);
    m3.SwitchOn();
    sleep(1);

//    m1.SetupPositionMode(360,360);
//    m2.SetupPositionMode(360,360);
//    m3.SetupPositionMode(360,360);
    m1.Setup_Torque_Mode();
    m2.Setup_Torque_Mode();
    m3.Setup_Torque_Mode();



    double ep1,ev1;
    double ep2,ev2;
    double ep3,ev3;

    double interval=3;
    pd1.SetSaturation(-20,20);
    pd2.SetSaturation(-20,20);
    pd3.SetSaturation(-20,20);


double f=150;

incli=20;


for (int i=0;i<11;i++)
{

    orient += 30;

    a.GetIK(incli,orient,lengths);

    cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;
    cout << "incli " << incli  << ", orient " << orient << endl;

    graph << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;
    cout << "incli " << incli  << ", orient " << orient << endl;


    posan1=(0.1-lengths[0])*180/(0.01*M_PI);
    posan2=(0.1-lengths[1])*180/(0.01*M_PI);
    posan3=(0.1-lengths[2])*180/(0.01*M_PI);

    for (double t=0;t<interval; t+=dts)
    {
        ep1=posan1-m1.GetPosition();
        ev1= (ep1 > pd1)-m1.GetVelocity();
        m1.SetTorque(ev1 > pi1);

        ep2=posan2-m2.GetPosition();
        ev2= (ep2 > pd2)-m2.GetVelocity();
        m2.SetTorque(ev2 > pi2);

        ep3=posan3-m3.GetPosition();
        ev3= (ep3 > pd3)-m3.GetVelocity();
        m3.SetTorque(2.1*(ev3 > pi3));


//        m1.SetTorque(200);
        usleep(dts*1000*1000);
        cout << t << " , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl;
        cout << t << " , " << posan1  << " , " << posan2 << " , " << posan3 << endl;
        graph << t << " , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl;
        graph << t << " , " << posan1  << " , " << posan2 << " , " << posan3 << endl;

    }
//    cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3 <<endl;

}
    m1.SetTorque(0);
    m2.SetTorque(0);
    m3.SetTorque(0);
//    m1.SetupPositionMode(360,360);
//    m2.SetupPositionMode(360,360);
//    m3.SetupPositionMode(360,360);

//    m1.SetPosition(posan1);
//    m2.SetPosition(posan2);
//    m3.SetPosition(posan3);
//        sleep(2);
//    m1.SetPosition(0);
//    m2.SetPosition(0);
//    m3.SetPosition(0);
//    sleep(2);


}

