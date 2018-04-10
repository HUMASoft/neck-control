#include <iostream>
#include "TestPort.h"
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <thread>
#include "PIDBlock.h"
#include "mainlib.h"
#include <fstream>


using namespace std;
PIDBlock pid (10,10,0,0.01);
PIDBlock pid1 (5,0,1,0.01);

ofstream fout ("test.txt", std::ofstream::out);

//void funcion1(CiA402Device * ob){
//    ob->Reset();
//    ob->SwitchOn();
//    cout<<"Estado motor 1:\n"<<endl;
//    ob->PrintStatus();

//}
//void funcion2(CiA402Device * ob){
//    ob->Reset();
//    ob->SwitchOn();
//    cout<<"Estado motor 2:\n"<<endl;
//    ob->PrintStatus();
//}
//void funcion3(CiA402Device * ob){
//    ob->Reset();
//    ob->SwitchOn();
//    cout<<"Estado motor 3:\n"<<endl;
//    ob->PrintStatus();
//}



int main()
{
    SocketCanPort pm1("can0");
    CiA402Device m1 (1, &pm1);

//    SocketCanPort pm2("can0");
//    CiA402Device m2 (2, &pm2);
//    SocketCanPort pm3("can0");
//    CiA402Device m3 (3, &pm3);

//     thread th (funcion1,&m1); thread th2 (funcion2,&m2); thread th3 (funcion3,&m3);

    m1.Reset();
////     m2.Reset();
////     m3.Reset();
    m1.SwitchOn();
////     m2.SwitchOn();
////     m3.SwitchOn();
//     cout<<"Estado motor 1:\n"<<endl;
//     //m1.PrintStatus();
     int pos_ideal_m1=-180;
      double velocity_ideal_m1=0;

//     //m1.Setup_Velocity_Mode(0,360);
////     uint32_t margen_error=5;
//     double pv = 0;
//     double u=0;

//     while (margen_error<(pos_ideal_m1-pv)<-margen_error) {

//        u= pid.OutputUpdate(pos_ideal_m1-pv);
//        m1.SetVelocity((uint32_t)u);
//        pv=m1.GetPosition();

//     }

//CONTROL
//m1.SetupPositionMode(360,360);
//m1.SetPosition(pos_ideal  double x=t-(int)t;
//      cout<<x<<endl;
//    const vector<uint8_t> data={0x00,0x00,0xFF,0x01};
//   //   long decimal=t-(int)t;
//      cout<<"------------"<<t<<endl;_m1);
//sleep(1);
//pv=m1.GetPosition();
//while(pv != pos_ideal_m1){
//    //m1.SetupPositionMode(360,360);
//    pv=m1.GetPosition();
//    cout<<"POS"<<pv<<endl;
//    u=pid.OutputUpdate(pos_ideal_m1-pv);
//    cout<<"UUUUUUUUUU"<<u<<endl;
//    m1.Setup_Velocity_Mode(u,360);


////     //Control TORQUE
//     m1.Setup_Torque_Mode();

//     double u=0;

//    // while(-0.1>pv-velocity_ideal_m1 || pv-velocity_ideal_m1>0.1)//

//         while(pv!=velocity_ideal_m1)
//     {
//         pv=m1.GetVelocity();
//         fout<<pv<<" , ";
//         u=pid.OutputUpdate(velocity_ideal_m1-pv);
//         fout<<u<<endl;
//         m1.SetTorque(u);
//         //  cout<<"UUUUUUUUUUUUU"<<u<<endl;
//         //usleep(1000);

//     }

//     m1.SetupPositionMode(360,360);
//     m1.SetPosition(pos_ideal_m1);
//     sleep(1);
    // m1.GetPosition();
     //m1.SetTorque(00);

//double up=0;
//double uv=0;

//while(pv != pos_ideal_m1){

//    m1.SetupPositionMode(360,360);
//    m1.SetPosition(pos_ideal_m1);
//    up =pid.OutputUpdate(pos_ideal_m1-pv);

//    uv =pid1.OutputUpdate(up-m1.GetVelocity());
//    m1.Setup_Torque_Mode();
//    m1.SetTorque(uv);


//}


////sleep(2);
////m1.PrintStatus();
//sleep(1+pos_ideal_m1/360);

////sleep(3);
m1.Setup_Velocity_Mode(0,360);
//cout<<"Getposition"<<pv<<endl;
m1.SetVelocity(velocity_ideal_m1);

//cout<<m1.GetVelocity();







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

//     sleep(1+posdegree_m1/360150);
//     int posicion_correcta=-1;
//     while(posicion_correcta<0)
//     double pos_real_m1 = m1.GetPosition();
//       cout<< "actual pos: " << pos_real << " [deg]" <<endl;

//     double error_m1 = pos_real_m1-pos_ideal_m1;

//     if(error_m1<0)  //No ha llegado a la posición deseada
//     {controlword
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

