
#include <thread>
#include <iostream>
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include "mainlib.h"
#include "math.h"

#include "fcontrol.h"

//Controllers
double dts=0.01;

PIDBlock pi (3.956,247.8,0,dts);
PIDBlock pd (70.58,0,1.875,dts);

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


SystemBlock pi1(npi,dpi,10);
SystemBlock pd1(npd,dpd);

SystemBlock pi2(npi,dpi,10);
SystemBlock pd2(npd,dpd);

SystemBlock pi3(npi,dpi,10);
SystemBlock pd3(npd,dpd);


fstream graph("./graph.csv",ios::trunc);
SocketCanPort pm1("can0");
CiA402Device m1 (1, &pm1);
SocketCanPort pm2("can0");
CiA402Device m2 (2, &pm2);
SocketCanPort pm3("can0");
CiA402Device m3 (3, &pm3);

double ep1,ev1;
double ep2,ev2;
double ep3,ev3;

double interval=5;

void funcion1(CiA402Device * ob, double * position){
    while(1)
    {
        ep1=*position-ob->GetPosition();
        ev1= (ep1 > pd1)-ob->GetVelocity();
        ob->SetTorque(ev1 > pi1);
        usleep(dts*1000*1000);
        cout << " pos1 " << ob->GetPosition() << " , " << *position    <<endl;

    }

}

void funcion2(CiA402Device * ob, double * position){
    while(1)
    {
        ep1=*position-ob->GetPosition();
        ev1= (ep1 > pd1)-ob->GetVelocity();
        ob->SetTorque(ev1 > pi1);
        usleep(dts*1000*1000);
        //cout << " pos2 " << ob->GetPosition() << " , " << *position    <<endl;
    }

}
void funcion3(CiA402Device * ob, double * position){
    while(1)
    {
        ep1=*position-ob->GetPosition();
        ev1= (ep1 > pd1)-ob->GetVelocity();
        ob->SetTorque(2*(ev1 > pi1));
        usleep(dts*1000*1000);
        //cout << " pos3 " << ob->GetPosition() << " , " << *position    <<endl;
    }
}

int main ()
{




    TableKinematics a;
    vector<double> lengths(3);
    long orient=1;
    long incli=30;

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




    pd1.SetSaturation(-20,20);
    pd2.SetSaturation(-20,20);
    pd3.SetSaturation(-20,20);

        double posan1, posan2, posan3;
        double *thran1, *thran2, *thran3;

         thread th (funcion1,&m1,&posan1);
         thread th2 (funcion2,&m2,&posan2);
         thread th3 (funcion3,&m3,&posan3);
         th3.join();
         th2.join();
         th.join();

double f=150;


for (int i=0;i<11;i++)
{

    orient += 30;

    a.GetIK(incli,orient,lengths);
    cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;

    posan1=(0.1-lengths[0])*180/(0.01*M_PI);
    posan2=(0.1-lengths[1])*180/(0.01*M_PI);
    posan3=(0.1-lengths[2])*180/(0.01*M_PI);
    *thran1=posan1;
    *thran2=posan2;
    *thran3=posan3;
    cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3;

    graph << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3 << endl;

//    for (double t=0;t<interval; t+=dts)
//    {
//        ep1=posan1-m1.GetPosition();
//        ev1= (ep1 > pd1)-m1.GetVelocity();
//        m1.SetTorque(ev1 > pi1);

//        ep2=posan2-m2.GetPosition();
//        ev2= (ep2 > pd2)-m2.GetVelocity();
//        m2.SetTorque(ev2 > pi2);

//        ep3=posan3-m3.GetPosition();
//        ev3= (ep3 > pd3)-m3.GetVelocity();
//        m3.SetTorque(ev3 > pi3);
////        m2.SetTorque(pd2.OutputUpdate(pi2.OutputUpdate(posan2-m2.GetPosition())-m2.GetVelocity()));
////        m3.SetTorque(pd3.OutputUpdate(pi3.OutputUpdate(posan1-m3.GetPosition())-m3.GetVelocity()));

////        m1.SetTorque(200);
//        usleep(dts*1000*1000);
////        cout << t << " , GetPosition: " << m1.GetPosition() << " ,GetVelocity:  " << m1.GetVelocity() << endl;
//        cout << t << " , " << posan1  << " , " << posan2 << " , " << posan3 << endl;

//    }
//    cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3 <<endl;



    sleep(2);

}


m1.SetTorque(0);
m2.SetTorque(0);
m3.SetTorque(0);

th3.detach();
th2.detach();
th.detach();
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

return 0;

}

