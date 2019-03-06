#include <iostream>
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include "mainlib.h"
#include "math.h"
#include "ToolsFControl.h"

#include "SerialArduino.h"

#include "fcontrol.h"


int main ()
{
        //--sensors--
        SerialArduino tilt;
        float incSensor,oriSensor;
        sleep(3);


        double dts=0.01; //0.01
    ofstream graph("graph.csv",std::ofstream::out);


    //--Controllers--
    //fpi w=25 pm=70 //kept from last experiments.
    vector<double> npi ={0.2916 ,  -5.3981 ,   0.2415  ,  5.5243};
    vector<double> dpi ={0.5882 , -0.9921 , -0.5804 , 1.0000};
    SystemBlock internal1(npi,dpi);
    SystemBlock internal3(npi,dpi);
    SystemBlock internal2(npi,dpi);

//    string method("pi");
//    PIDBlock internal1(1,10,0,dts);
//    PIDBlock internal2(1,10,0,dts);
//    PIDBlock internal3(1,10,0,dts);

//    string method("2isomw10p100");
//    vector<double> npd ={0.7178 ,  -2.0893 ,  -1.3030  ,  2.9270};
//    vector<double> dpd ={0.0337 , -0.9032 , 0.0631 , 1.0000};
//    SystemBlock pd1(npd,dpd,1);
//    SystemBlock pd2(npd,dpd,1);
//    SystemBlock pd3(npd,dpd,1);

    string method("w10p60pid"); // entero
     PIDBlock external1(2,1,0.1,dts);
     PIDBlock external2(2,1,0.1,dts);
     PIDBlock external3(2,1,0.1,dts);




    //--Can port communications--
    SocketCanPort pm1("can1");
    CiA402Device m1 (1, &pm1);
    SocketCanPort pm2("can1");
    CiA402Device m2 (2, &pm2);
    SocketCanPort pm3("can1");
    CiA402Device m3 (3, &pm3);


    //--Neck Kinematics--
    double l0=0.1090;
    double lg0=l0+0.002;
    GeoInkinematics neck_ik(0.052,0.052,l0);
    vector<double> lengths(3);
    vector<double> sensorLengths(3,0);


    //vars
    double ep1,ev1,vcs1,tcs1;
    double ep2,ev2,vcs2,tcs2;
    double ep3,ev3,vcs3,tcs3;
    double targetAngle1, targetAngle2, targetAngle3;
    double realAngle1, realAngle2, realAngle3;


//    Motor setup
    m1.Reset();
    m1.SwitchOn();

    m2.Reset();
    m2.SwitchOn();

    m3.Reset();
    m3.SwitchOn();


    m3.Setup_Torque_Mode();
    m1.Setup_Torque_Mode();
    m2.Setup_Torque_Mode();


    ToolsFControl tools;
    tools.SetSamplingTime(dts);

    double incli = 14.5;
    double orient = 300.8;


    for (double t=0;t<5;t+=dts)
    {

        neck_ik.GetIK(incli,orient,lengths);
        targetAngle1=(lg0-lengths[0])*180/(0.01*M_PI);
        targetAngle2=(lg0-lengths[1])*180/(0.01*M_PI);
        targetAngle3=(lg0-lengths[2])*180/(0.01*M_PI);

        //Get real position through sensor
        tilt.readSensor(incSensor,oriSensor);
        neck_ik.GetIK(incSensor,oriSensor,sensorLengths);
        realAngle1=(l0-sensorLengths[0])*180/(0.01*M_PI);
        realAngle2=(l0-sensorLengths[1])*180/(0.01*M_PI);
        realAngle3=(l0-sensorLengths[2])*180/(0.01*M_PI);

        cout << "incSensor " << incSensor  << ", oriSensor " << oriSensor <<endl;

//        cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;
//        cout << "l1 " << sensorLengths[0]  << ", l2 " << sensorLengths[1] << ", l3 " << sensorLengths[2]<<endl;

        //Get error from real position
        ep1=targetAngle1-realAngle1;
        //Get control signal
        vcs1= external1.UpdateControl(ep1);
        //Send target to internal loop
        ev1= vcs1-m1.GetVelocity();
        tcs1=(ev1 > internal1);
        m1.SetTorque(tcs1);
//        cout << "ep1 " << ep1  << ", vcs1 " << vcs1 << ", ev1 " << ev1 << ", tcs1 " << tcs1 <<endl;


        ep2=targetAngle2-realAngle2;
        vcs2=ep2 > external2;
        ev2= vcs2-m2.GetVelocity();
        tcs2=(ev2 > internal2);
        m2.SetTorque(tcs2);

        ep3=targetAngle3-realAngle3;
        vcs3=ep3 > external3;
        ev3= vcs3-m3.GetVelocity();
        tcs3=(ev3 > internal3);
        m3.SetTorque(tcs3);

        tools.WaitSamplingTime();


    }




    cout << "FIN" << endl;
    m1.SetTorque(0);
    m2.SetTorque(0);
    m3.SetTorque(0);

    sleep(1);


    return 0;
}
