#include <iostream>
#include "TestPort.h"
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"



using namespace std;

int main()
{
    SocketCanPort pm1;
    CiA402Device m1 (6, &pm1);
    SocketCanPort pm2;
    CiA402Device m2 (14, &pm2);
    SocketCanPort pm3;
    CiA402Device m3 (8, &pm3);
     m1.Reset();
//     m2.Reset();
//     m3.Reset();
     m1.SwitchOn();
//     m2.SwitchOn();
//     m3.SwitchOn();
     cout<<"Estado motor 1:\n"<<endl;
     m1.PrintStatus();
//     cout<<"Estado motor 2:\n"<<endl;
//     m2.PrintStatus();
//     cout<<"Estado motor 3:\n"<<endl;
//     m3.PrintStatus();


    return 0;
}

