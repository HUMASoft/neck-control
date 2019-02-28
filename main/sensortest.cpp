#include <iostream>
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include "mainlib.h"
#include "math.h"

#include "ToolsFControl.h"


#include "SerialArduino.h"



int main ()
{


    SerialArduino tilt;

    ofstream graph("graph.csv",std::ofstream::out);
    SocketCanPort pm1("can1");
    CiA402Device m1 (1, &pm1);
    SocketCanPort pm2("can1");
    CiA402Device m2 (2, &pm2);
    SocketCanPort pm3("can1");
    CiA402Device m3 (3, &pm3);


    TableKinematics a("../neck-control/arco1075.csv");
    vector<double> lengths(3);
    long orient=1;
    long incli=1;
    float lg0=0.111;

    a.GetIK(incli,orient,lengths);
    cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;
    double posan1, posan2, posan3;
    posan1=(lg0-lengths[0])*180/(0.01*M_PI);
    posan2=(lg0-lengths[1])*180/(0.01*M_PI);
    posan3=(lg0-lengths[2])*180/(0.01*M_PI);

    m1.SetPosition(posan1);
    m2.SetPosition(posan2);
    m3.SetPosition(posan3);

    cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3 <<  endl;

    double incSensor,oriSensor;
    double dts=0.01;
//    incSensor = tilt.ReadInclination();
//    oriSensor = tilt.ReadOrientation();

    sleep(2);
    for (double t=0;t<2; t+=dts)
    {
        usleep(dts*1000*1000);
//          tools.WaitSamplingTime();
//            tilt.ReadSensor(incSensor,oriSensor);
//        incSensor = tilt.ReadInclination();
//        oriSensor = tilt.ReadOrientation();
        cout  << " Tiempo: " << t << endl;
        cout << " incli_sen: " << incSensor << " , orient_sen: " << oriSensor <<  endl;
//          graph << t << " , " << incli  << " , " << incSensor << " , " << orient << " , " << oriSensor << endl;
        graph << t << " , " << posan1 << " , " << m1.GetPosition() << " , " << posan2 << " , " << m2.GetPosition()  << " , " << posan3 <<  " , " << m3.GetPosition()  << " , " << incli << " , " << incSensor   << " , " << orient   << " , " << oriSensor <<endl;
    }
//    m1.Reset();
//    m2.Reset();
//    m3.Reset();

//    m1.SwitchOn();
//    m2.SwitchOn();
//    m3.SwitchOn();

//    m1.SetupPositionMode(360,360);
//    m2.SetupPositionMode(360,360);
//    m3.SetupPositionMode(360,360);


    for (int i=0;i<10;i++){

        //.............HIGHT

        incli =20;
        orient=180;

        a.GetIK(incli,orient,lengths);
        cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;

        posan1=(lg0-lengths[0])*180/(0.01*M_PI);
        posan2=(lg0-lengths[1])*180/(0.01*M_PI);
        posan3=(lg0-lengths[2])*180/(0.01*M_PI);
        cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3<<endl;

        m1.SetPosition(posan1);
        m2.SetPosition(posan2);
        m3.SetPosition(posan3);



        float incSensor,oriSensor;
        double dts=0.01;
//        ToolsFControl tools;
//        tools.SetSamplingTime(dts);

        for (double t=0;t<5; t+=dts)
        {
            usleep(dts*1000*1000);
//          tools.WaitSamplingTime();
            tilt.readSensor(incSensor,oriSensor);
//            incSensor = tilt.ReadInclination();
//            oriSensor = tilt.ReadOrientation();
            //cout  << " orin: " << orient <<endl;
            //cout  << " Real: " << t << " , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl;
            //cout << " Consigna: " << t << " , " << posan1  << " , " << posan2 << " , " << posan3 << endl;
            cout  << " Tiempo: " << t ;//<< endl;
            cout  << " incl: " << incli << " orien: " << orient << endl;
            cout << " incli_sen: " << incSensor << " , orient_sen: " << oriSensor <<  endl;
//          graph << t << " , " << incli  << " , " << incSensor << " , " << orient << " , " << oriSensor << endl;
            graph << t << " , " << posan1 << " , " << m1.GetPosition() << " , " << posan2 << " , " << m2.GetPosition()  << " , " << posan3 <<  " , " << m3.GetPosition()  << " , " << incli << " , " << incSensor   << " , " << orient   << " , " << oriSensor <<endl;
        }


        //.............LOW
    orient=1;
    incli=1;

    a.GetIK(incli,orient,lengths);
    cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;

    posan1=(lg0-lengths[0])*180/(0.01*M_PI);
    posan2=(lg0-lengths[1])*180/(0.01*M_PI);
    posan3=(lg0-lengths[2])*180/(0.01*M_PI);
    cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3;

    m1.SetPosition(posan1);
    m2.SetPosition(posan2);
    m3.SetPosition(posan3);


    for (double t=2;t<5; t+=dts)
    {
        usleep(dts*1000*1000);
//        incSensor = tilt.ReadInclination();
//        oriSensor = tilt.ReadOrientation();
        //cout  << " orin: " << orient <<endl;
        //cout  << " Real: " << t << " , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl;
        //cout << " Consigna: " << t << " , " << posan1  << " , " << posan2 << " , " << posan3 << endl;
        cout  << " Tiempo: " << t;// << endl;
        cout  << " incl: " << incli << " orien: " << orient <<endl;
        cout << " zincli_sen: " << incSensor << " , orient_sen: " << oriSensor <<  endl;
        //graph << t << " , " << incli  << " , " << incSensor << " , " << orient << " , " << oriSensor << endl;
        graph << t << " , " << posan1 << " , " << m1.GetPosition() << " , " << posan2 << " , " << m2.GetPosition()  << " , " << posan3 <<  " , " << m3.GetPosition()  << " , " << incli << " , " << incSensor   << " , " << orient   << " , " << oriSensor <<endl;
    }

}
        m1.SetPosition(0);
        m2.SetPosition(0);
        m3.SetPosition(0);
    return 0;


}


