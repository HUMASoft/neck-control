

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

//    //pi: w=5 rad/s, phm=70 deg
//    //pd: w=5 rad/s, phm=70 deg
//    double pi[3]={ 0.1278262 , 5.5535135 , 0 };
//    double pd[3]={ 5.5535135 , 0 , 0.1278262 };

////    //not ok!!!
////    //pi: w=25 rad/s, phm=70 deg
////    //pd: w=25 rad/s, phm=70 deg
////    double pi[3]={ 2.0072114 , 44.868574 , 0 };
////    double pd[3]={ 44.868574 , 0 , 2.0072114 };

//        PIDBlock pi1 (pi[0],pi[1],pi[2],dts);
//        PIDBlock pd1 (pd[0],pd[1],pd[2],dts);

//        PIDBlock pi2 (pi[0],pi[1],pi[2],dts);
//        PIDBlock pd2 (pd[0],pd[1],pd[2],dts);

//        PIDBlock pi3 (pi[0],pi[1],pi[2],dts);
//        PIDBlock pd3 (pd[0],pd[1],pd[2],dts);



////        SystemBlock pi1(npi,dpi,dts/2);
////        SystemBlock pd1(npd,dpd,2/dts);

////        SystemBlock pi2(npi,dpi,dts/25);
////        SystemBlock pd2(npd,dpd,2/dts);

////        SystemBlock pi3(npi,dpi,dts/2);
////        SystemBlock pd3(npd,dpd,2/dts);


//    //fpi w=5
//    vector<double> npi ={-0.3913  ,  1.0335  ,  1.6085  , -5.0480  ,  2.8015};
//    vector<double> dpi ={ -0.1539  ,  0.3766  ,  0.6573 ,  -1.8799  ,  1.0000};

//    //fpd
//    //fpd w=5
//    vector<double> npd ={-5.4872  , 49.4863 ,  -6.7768, -157.0124 , 121.7188};
//    vector<double> dpd ={0.0653  , -0.2690,   -0.8054 ,   0.5132 ,   1.0000};

    //fpd w=25 pm=70
    vector<double> npd ={26.1251, -119.3468, -293.8182,  178.9746 , 361.7371};
    vector<double> dpd ={-0.1440 ,  -0.1031 ,   1.1445  ,  2.1032  ,  1.0000};

    //fpi w=25 pm=70
    vector<double> npi ={0.3354  ,  0.3724  , -1.8968 ,  -0.5654  ,  1.9306};
    vector<double> dpi ={0.2381  ,  0.3986  , -1.0645  , -0.5716  ,  1.0000};

//    //fpd w=5
//    vector<double> npd ={-5.4872  , 49.4863 ,  -6.7768, -157.0124 , 121.7188};
//    vector<double> dpd ={0.0653  , -0.2690,   -0.8054 ,   0.5132 ,   1.0000};


    SystemBlock pi1(npi,dpi,1);
    SystemBlock pd1(npd,dpd,1);

    SystemBlock pi2(npi,dpi,1);
    SystemBlock pd2(npd,dpd,1);

    SystemBlock pi3(npi,dpi,1);
    SystemBlock pd3(npd,dpd,1);


    ofstream targets("targets.csv",std::ofstream::out);
    ofstream responses("responses.csv",std::ofstream::out);

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

//    m1.Reset();
//    m2.Reset();
//    m3.Reset();

//    m1.SwitchOn();
//    sleep(1);
//    m2.SwitchOn();
//    sleep(1);
//    m3.SwitchOn();
//    sleep(1);

//    m1.SetupPositionMode(360,360);
//    m2.SetupPositionMode(360,360);
//    m3.SetupPositionMode(360,360);
    m1.Setup_Torque_Mode();
    m2.Setup_Torque_Mode();
    m3.Setup_Torque_Mode();



    double ep1,ev1;
    double ep2,ev2;
    double ep3,ev3;

    double interval=1;

    pd1.SetSaturation(-30,30);
    pd2.SetSaturation(-30,30);
    pd3.SetSaturation(-30,30);


double f=150;

incli=20;


for (int i=0;i<11;i++)
{

    orient += 30;

    a.GetIK(incli,orient,lengths);

    cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;
    cout << "incli " << incli  << ", orient " << orient << endl;

//    graph << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;
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
        responses << t << " , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl;
        targets << t << " , " << posan1  << " , " << posan2 << " , " << posan3 << endl;

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

