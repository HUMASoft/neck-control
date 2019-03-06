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


//    TableKinematics a("../neck-control/arco1075.csv");
    //--Neck Kinematics--
    double l0=0.109;
    GeoInkinematics a(0.052,0.052,l0);
    vector<double> lengths(3);
//    long orient=1;
//    long incli=1;
    float lg0=l0+0.002;

    double posan1,posan2,posan3;

    float incSensor,oriSensor;

    double dts=0.01;
    sleep(3);


    ToolsFControl tools;
    tools.SetSamplingTime(dts);

        for (double t=0;t<25; t+=dts)
        {

            tools.WaitSamplingTime();

            tilt.readSensor(incSensor,oriSensor);



            a.GetIK(incSensor,oriSensor,lengths);
            cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;

            posan1=(lg0-lengths[0])*180/(0.01*M_PI);
            posan2=(lg0-lengths[1])*180/(0.01*M_PI);
            posan3=(lg0-lengths[2])*180/(0.01*M_PI);
            cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3<<endl;

        }



    return 0;


}


