#include <iostream>
#include "TestPort.h"
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <thread>
//#include "src/PIDBlock.h"



using namespace std;
void funcion1(CiA402Device * ob){
    ob->Reset();
    ob->SwitchOn();
    cout<<"Estado motor 1:\n"<<endl;
    ob->PrintStatus();

}
void funcion2(CiA402Device * ob){
    ob->Reset();
    ob->SwitchOn();
    cout<<"Estado motor 2:\n"<<endl;
    ob->PrintStatus();
}
void funcion3(CiA402Device * ob){
    ob->Reset();
    ob->SwitchOn();
    cout<<"Estado motor 3:\n"<<endl;
    ob->PrintStatus();
}



int main()
{
    SocketCanPort pm1("vcan0");
    double status_m1;
   // PIDBlock pid_m1;
    CiA402Device m1 (6, &pm1);
    SocketCanPort pm2("vcan0");
    CiA402Device m2 (14, &pm2);
    SocketCanPort pm3("vcan0");
    CiA402Device m3 (8, &pm3);

     thread th (funcion1,&m1); thread th2 (funcion2,&m2); thread th3 (funcion3,&m3);
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

     /* MAIN DE PRUEBA PARA CONTROL DE POSICION */

//     m1.SetupPositionMode(360*0.47,360); //(Max velocity in degrees/s, acceleration)

//     uint32_t pos_ideal_m1=120; //Write the position of m1 in degrees here

//     m1.SetPosition(m1.DegreeConv(posdegree_m1));
//     m1.FlushBuffer();

//     m1.PrintStatus();

//     sleep(1+posdegree_m1/360);
//     int posicion_correcta=-1;
//     while(posicion_correcta<0)
//     double pos_real_m1 = m1.GetPosition();
//       cout<< "actual pos: " << pos_real << " [deg]" <<endl;

//     double error_m1 = pos_real_m1-pos_ideal_m1;

//     if(error_m1<0)  //No ha llegado a la posición deseada
//     {
//         m1.Setup_Velocity_Mode(720,360);
//     }
//     else {
//         if(error_m1>0)  //Se ha pasado de la posición deseada
//         {
//            m1.Setup_Velocity_Mode(-720,360);
//         }
//         else{
//             cout<<"Posición deseada alcanzada"<<endl;
//             posicion_correcta=0;
//         }
//     }

     sleep(1);

    return 0;
}

