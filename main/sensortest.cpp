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


    TableKinematics a("../neck-control/arco107.csv");
    vector<double> lengths(3);
    long orient=90;
    long incli=1;

    a.GetIK(incli,orient,lengths);
    cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;
    double posan1, posan2, posan3;
    posan1=(0.109-lengths[0])*180/(0.01*M_PI);
    posan2=(0.109-lengths[1])*180/(0.01*M_PI);
    posan3=(0.109-lengths[2])*180/(0.01*M_PI);
    cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3;
//    double incSensor,oriSensor;
//    tilt.ReadSensor(incSensor,oriSensor);
    sleep(3);
//    m1.Reset();
//    m2.Reset();
//    m3.Reset();

//    m1.SwitchOn();
//    m2.SwitchOn();
//    m3.SwitchOn();

//    m1.SetupPositionMode(360,360);
//    m2.SetupPositionMode(360,360);
//    m3.SetupPositionMode(360,360);

    for(int j=0;j<5;j++){

    for (int i=0;i<1;i++)
    {

        incli += 15;

        a.GetIK(incli,orient,lengths);
        cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;

        posan1=(0.109-lengths[0])*180/(0.01*M_PI);
        posan2=(0.109-lengths[1])*180/(0.01*M_PI);
        posan3=(0.109-lengths[2])*180/(0.01*M_PI);
        cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3;

        //graph << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3 << endl;

        m1.SetPosition(posan1);
        m2.SetPosition(posan2);
        m3.SetPosition(posan3);



        double incSensor,oriSensor;
        double dts=0.02;
//        ToolsFControl tools;
//        tools.SetSamplingTime(dts);

        for (double t=0;t<2; t+=dts)
        {
            usleep(dts*1000*1000);
//          tools.WaitSamplingTime();
//            tilt.ReadSensor(incSensor,oriSensor);
            incSensor = tilt.ReadInclination();
            oriSensor = tilt.ReadOrientation();
            //cout  << " orin: " << orient <<endl;
            cout  << " incl: " << incli << " orin: " << orient <<endl;
            //cout  << " Real: " << t << " , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl;
            //cout << " Consigna: " << t << " , " << posan1  << " , " << posan2 << " , " << posan3 << endl;
            cout  << " Tiempo: " << t << endl;
            cout << "incli_sen: " << incSensor << " , orient_sen: " << oriSensor <<  endl;
//          graph << t << " , " << incli  << " , " << incSensor << " , " << orient << " , " << oriSensor << endl;
            graph << t << " , "<< incSensor << " , " << oriSensor <<" , " << posan1  << " , " << posan2 << " , " << posan3  << " , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() << " , " << incli << " , " << orient << endl;


        }

    }

    orient=90;
    incli=1;

    a.GetIK(incli,orient,lengths);
    cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;

    posan1=(0.109-lengths[0])*180/(0.01*M_PI);
    posan2=(0.109-lengths[1])*180/(0.01*M_PI);
    posan3=(0.109-lengths[2])*180/(0.01*M_PI);
    cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3;

    //graph << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3 << endl;

    m1.SetPosition(posan1);
    m2.SetPosition(posan2);
    m3.SetPosition(posan3);

    double incSensor,oriSensor;
    double dts=0.02;
    for (double t=2;t<4; t+=dts)
    {
        usleep(dts*1000*1000);
        incSensor = tilt.ReadInclination();
        oriSensor = tilt.ReadOrientation();
        //cout  << " orin: " << orient <<endl;
        cout  << " incl: " << incli << " orin: " << orient <<endl;
        //cout  << " Real: " << t << " , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() <<endl;
        //cout << " Consigna: " << t << " , " << posan1  << " , " << posan2 << " , " << posan3 << endl;
        cout  << " Tiempo: " << t << endl;
        cout << "incli_sen: " << incSensor << " , orient_sen: " << oriSensor <<  endl;
        //graph << t << " , " << incli  << " , " << incSensor << " , " << orient << " , " << oriSensor << endl;
        graph << t << " , "<< incSensor << " , " << oriSensor <<" , " << posan1  << " , " << posan2 << " , " << posan3  << " , " << m1.GetPosition() << " , " << m2.GetPosition() <<  " , " << m3.GetPosition() << " , " << incli << " , " << orient << endl;

    }

    }

    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);
    return 0;


}


