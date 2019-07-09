
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

        double dts=0.01; //0.01
    ofstream graph("graph.csv",std::ofstream::out);


    //--Controllers--
    //fpi w=25 pm=70 //kept from last experiments.
    vector<double> npi ={0.2916 ,  -5.3981 ,   0.2415  ,  5.5243};
    vector<double> dpi ={0.5882 , -0.9921 , -0.5804 , 1.0000};
    SystemBlock internal1(npi,dpi);
    SystemBlock internal3(npi,dpi);
    SystemBlock internal2(npi,dpi);


    // entero
//     PIDBlock external1(2,1,0.1,dts);
//     PIDBlock external2(2,1,0.1,dts);
//     PIDBlock external3(2,1,0.1,dts);

//     PIDBlock external1(3.2158394,29.966573,0.0000061,dts);
//     PIDBlock external2(3.2158394,29.966573,0.0000061,dts);
//     PIDBlock external3(3.2158394,29.966573,0.0000061,dts);

     // fraccionario (kp,kf,exp)
    // w=1, pm=60 (0.0142718,0.1553804,-0.35,dts);
    // w=12 (1.7807044, 4.8890371, -0.81, dts);
     FPDBlock external1(1.7807044, 4.8890371, -0.81, dts);
     FPDBlock external2(1.7807044, 4.8890371, -0.81, dts);
     FPDBlock external3(1.7807044, 4.8890371, -0.81, dts);


    //--Can port communications--
    SocketCanPort pm1("can1");
    CiA402Device m1 (1, &pm1);
    SocketCanPort pm2("can1");
    CiA402Device m2 (2, &pm2);
    SocketCanPort pm3("can1");
    CiA402Device m3 (3, &pm3);


    //--Neck Kinematics--
    TableKinematics a("../neck-control/spring097.csv");
    vector<double> lengths(3);


    //vars
    double ep1,ev1,cs1,tcs1;
    double ep2,ev2,cs2,tcs2;
    double ep3,ev3,cs3,tcs3;
    double posan1, posan2, posan3;

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

    long orient=1;
    long incli=1;
    float lg0=0.099;
    long incl_value=10;
    sleep(4);


    ToolsFControl tools;
    tools.SetSamplingTime(dts);

    //-----------Soft Start

//    for (int i=1; i<incl_value; i++){
//    incli ++;

    //
    incli=15;
    orient=0;
    for (int j=1; j<3; j++){    //para incli

    for (int i=1; i<36; i++){   //para orient
    orient=orient+10;

    for (double t=0;t<0.4;t+=dts){ //<0.05

        tilt.readSensor(incSensor,oriSensor);


    cout  << "incli " << incli << ",  orient " << orient  << endl;
    cout << "incli_sen: " << incSensor << " , orient_sen: " << oriSensor <<  endl;
    a.GetIK(incli,orient,lengths);
    posan1=(lg0-lengths[0])*180/(0.01*M_PI);
    posan2=(lg0-lengths[1])*180/(0.01*M_PI);
    posan3=(lg0-lengths[2])*180/(0.01*M_PI);
    cout << "TARGET: , " << posan1  << " , " << posan2 << " , " << posan3 << endl;


    ep1=posan1-m1.GetPosition();
    cs1=ep1 > external1;
    ev1= cs1-m1.GetVelocity();
    tcs1=(ev1 > internal1);
    m1.SetTorque(tcs1);

    ep2=posan2-m2.GetPosition();
    cs2=ep2 > external2;
    ev2= cs2-m2.GetVelocity();
    tcs2= (ev2 > internal2);
    m2.SetTorque(tcs2);

    ep3=posan3-m3.GetPosition();
    cs3=ep3 > external3;
    ev3= cs3-m3.GetVelocity();
    tcs3= (ev3 > internal3);
    m3.SetTorque(tcs3);

        cout << "ACTUAL: , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl;
       //graph << t << " , " << posan1 << " , " << m1.GetPosition() << " , " << posan2 << " , " << m2.GetPosition()  << " , " << posan3 <<  " , " << m3.GetPosition()  << " , " << incli << " , " << incSensor   << " , " << orient   << " , " << oriSensor <<endl;
        graph << t << " , " << posan1 << " , " << m1.GetPosition() << " , " << posan2 << " , " << m2.GetPosition()  << " , " << posan3 <<  " , " << m3.GetPosition()  << " , " << incli << " , " << incSensor   << " , " << orient   << " , " << oriSensor << " , " << ep1 << " , " << cs1 << " , " << ev1 << " , " << tcs1 << " , " << ep2 << " , " << cs2 << " , " << ev2 << " , " << tcs2 << " , " << ep3 << " , " << cs3 << " , " << ev3 << " , " << tcs3 <<endl;

        tools.WaitSamplingTime();
    }
    }

    incli=incli+10;
    orient=0;
     }

    sleep(2);


    cout << "FIN" << endl;
    m1.SetTorque(0);
    m2.SetTorque(0);
    m3.SetTorque(0);

    sleep(1);

    return 0;
}
