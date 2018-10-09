#include <iostream>
#include "TestPort.h"
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <thread>
#include "PIDBlock.h"
#include "mainlib.h"
#include <fstream>
#include "fcontrol.h"
#include "Kinematics.h"

//#include "SerialArduino.h"


using namespace std;



ofstream fout ("test.txt", std::ofstream::out);

void funcion1(CiA402Device * ob, long pos_ideal){
    double dtse=0.01;
//    PIDBlock pidi (1.956,58.38,0,dtse);  //PI 1
//    PIDBlock pide (10.61,0,0.08305,dtse);  //PD 1
    PIDBlock pidi (5.742,35.75,0,dtse);  //PI 1
    PIDBlock pide (7.934,0,0.265,dtse);  //PD 1
    SystemBlock con(vector<double>{   -0.0404  ,  0.3582  , -0.0891  , -1.0214  ,  0.7977},
                              vector<double>{ -0.1076 ,   0.4977 ,   0.1886  , -1.5784  ,  1.0000},
   0.78689 );
    //con.SetSaturation(-800,800);
    ob->Reset();
    ob->SwitchOn();
    //cout<<"Estado motor 1:\n"<<endl;
    //ob->PrintStatus();
    ob->Setup_Torque_Mode();
   int pos_ideal_m1=pos_ideal;
   for (int i=0;i<400;i++){
        ob->SetTorque(pidi.OutputUpdate(pide.OutputUpdate(pos_ideal_m1-ob->GetPosition())-ob->GetVelocity()));
        usleep(dtse*1000000);
   }
   ob->SetTorque(0);
   cout<<"MOTOR_1 "<<ob->GetPosition()<<endl;

}

void funcion2(CiA402Device * ob, long pos_ideal){
    double dtse=0.01;
   PIDBlock pidi (0.0694,8.7,0,dtse);  //PI 2
   PIDBlock pide (8.662,0,0.2506,dtse);  //PD 2
    SystemBlock con(vector<double>{   -0.0404  ,  0.3582  , -0.0891  , -1.0214  ,  0.7977},
                              vector<double>{ -0.1076 ,   0.4977 ,   0.1886  , -1.5784  ,  1.0000},
   0.78689 );
    //con.SetSaturation(-800,800);
    ob->Reset();
    ob->SwitchOn();
//    //cout<<"Estado motor 1:\n"<<endl;
//    //ob->PrintStatus();
//    ob->Setup_Torque_Mode();
//   int pos_ideal_m1=pos_ideal;
//   for (int i=0;i<400;i++){
//        ob->SetTorque(pidi.OutputUpdate(pide.OutputUpdate(pos_ideal_m1-ob->GetPosition())-ob->GetVelocity()));
//        usleep(dtse*1000000);
//   }
//   ob->SetTorque(0);
//   cout<<"MOTOR_2 "<<ob->GetPosition()<<endl;
}
void funcion3(CiA402Device * ob,long pos_ideal){
    double dtse=0.01;
    double s3;
//    PIDBlock pidi (3.956,247.8,0,dtse);  //PI 3
//    PIDBlock pide (21.61,0,0.008305,dtse);  //PD 3
        PIDBlock pidi (0.6284,41.65,0,dtse);  //PI 3
        PIDBlock pide (8.631,0,0.246,dtse);  //PD 3
        SystemBlock con(vector<double>{   -0.0404  ,  0.3582  , -0.0891  , -1.0214  ,  0.7977},
                                  vector<double>{ -0.1076 ,   0.4977 ,   0.1886  , -1.5784  ,  1.0000},
       0.78689 );
        //con.SetSaturation(-800,800);
        ob->Reset();
        ob->SwitchOn();
        //cout<<"Estado motor 1:\n"<<endl;
        //ob->PrintStatus();
        ob->Setup_Torque_Mode();
       int pos_ideal_m1=pos_ideal;
       for (int i=0;i<400;i++){
            ob->SetTorque(pidi.OutputUpdate(pide.OutputUpdate(pos_ideal_m1-ob->GetPosition())-ob->GetVelocity()));
        ob->SetTorque(s3);
        usleep(dtse*1000000);
        cout<<"s3: " << s3 <<endl;
   }
   ob->SetTorque(0);
   cout<<"MOTOR_3 "<<ob->GetPosition()<<endl;
}



int main()
{
    ofstream graph11("graph11.csv",std::ofstream::out);
    SocketCanPort pm1("can0");
    CiA402Device m1 (1, &pm1);

    SocketCanPort pm2("can0");
    CiA402Device m2 (2, &pm2);
    SocketCanPort pm3("can0");

    CiA402Device m3 (3, &pm3);
    TableKinematics a;
    vector<double> lengths(3);
//    long orient=180;
//    long incli=20;

//    a.GetIK(incli,orient,lengths);
//    cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;
//    double posan1, posan2, posan3;
//    posan1=(0.1-lengths[0])*180/(0.01*M_PI);
//    posan2=(0.1-lengths[1])*180/(0.01*M_PI);
//    posan3=(0.1-lengths[2])*180/(0.01*M_PI);
//    cout << "pos1 " << posan1  << ", pos2 " << posan2 << ", pos3 " << posan3;

//     thread th (funcion1,&m1,posan1);
//     thread th2 (funcion2,&m2,posan2);
//     thread th3 (funcion3,&m3,posan3);
//     th3.join();
//     th2.join();
//     th.join();
//     cout<<"MOTOR_1 "<<m1.GetPosition()<<" pos1 "<<posan1<<endl;
//     cout<<"MOTOR_2 "<<m2.GetPosition()<<" pos2 "<<posan2<<endl;
//     cout<<"MOTOR_3 "<<m3.GetPosition()<<" pos3 "<<posan3<<endl;
    double dtse=0.01;
    PIDBlock pide (0.25,0.18,0,dtse);
    PIDBlock pidi (10.5,10,0,dtse);



    m1.Reset();
    m1.SwitchOn();
    //cout<<"Estado motor 1:\n"<<endl;
    //ob->PrintStatus();
    m1.Setup_Torque_Mode();
    double u=0,posan1=180;
   int pos_ideal_m1=posan1;
   for (double t=0;t<15; t+=dtse){
        m1.SetTorque(pidi.OutputUpdate(pide.OutputUpdate(pos_ideal_m1-m1.GetPosition())-m1.GetVelocity()));
        u= pidi.OutputUpdate(pide.OutputUpdate(pos_ideal_m1-m1.GetPosition())-m1.GetVelocity());
        graph11 << t  << "," << posan1  << "," << m1.GetPosition() << "," << posan1-m1.GetPosition() << "," << u<<endl;
        usleep(dtse*1000000);
   }
   m1.SetTorque(0);
   cout<<"MOTOR_1 "<<m1.GetPosition()<<endl;
    //graph11 << "t" << t  <<<< "pos1 " << posan1  << ", posreal " << m1.GetPosition() << ", error " << posan1-m1.GetPosition() << ", U " << 0<<endl;



     return 0;
}


//    m1.Reset();
////////     m2.Reset();
////////     m3.Reset();
//    m1.SwitchOn();
////     m2.SwitchOn();
////     m3.SwitchOn();
//     cout<<"Estado motor 1:\n"<<endl;
//     //m1.PrintStatus();
//     int pos_ideal_m1=0;
//     double velocity_ideal_m1=45;

////     //m1.Setup_Velocity_Mode(0,360);
//////     uint32_t margen_error=5;
//     double pv = 0;
//     double pvv = 0;
//     double uv=0;
//     double up=0;
//     double ut=0;


////     while (margen_error<(pos_ideal_m1-pv)<-margen_error) {

////        u= pid.OutputUpdate(pos_ideal_m1-pv);
////        m1.SetVelocity((uint32_t)u);
////        pv=m1.GetPosition();

////     }

////CONTROL
////m1.Setup_Velocity_Mode(0,360);
////m1.SetVelocity(velocity_ideal_m1);
// m1.Setup_Torque_Mode();




//for (int i=0;i<3000;i++){



//    m1.SetTorque(pid1.OutputUpdate(pid.OutputUpdate(pos_ideal_m1-m1.GetPosition())-m1.GetVelocity()));
//    usleep(dtse*1000000);


//}

//m1.SetTorque(0);



////     usleep(dtse*1000000);


//cout<<"GETPOSITION"<<m1.GetPosition()<<endl;
////m1.SetupPositionMode(0,360);

//////     //Control TORQUE


////     m1.Setup_Velocity_Mode(0,360);

////     double old = 0;
////     double nueva = m1.GetPosition();
////     double pv =0;

////     m1.SetVelocity(15);

////     for (int i=0;i<500;i++){
////         old=nueva;
////            usleep(dts*1000000);


////         nueva=m1.GetPosition();
////         pv= ((nueva-old)/dts)*60/360;
////          fout<<pv<<" , ";
////          fout<<m1.GetVelocity()<<endl;
////         cout<<"pv en rpm"<<nueva<<endl;

////        // cout<<"pv en rpm"<<m1.GetVelocity()<<endl;


////     }

////     double u=0;

////     while(-0.1>pv-velocity_ideal_m1 || pv-velocity_ideal_m1>0.1)

////    while(pv!=velocity_ideal_m1)
////    {
////        pv=m1.GetVelocity();

////        fout<<pv<<" , ";
////        u=pid.OutputUpdate(velocity_ideal_m1-pv);
////        fout<<u<<endl;
////        m1.SetTorque(u);
////        //  cout<<"UUUUUUUUUUUUU"<<u<<endl;
////        usleep(dts*1000000);
////     }

////     m1.SetupPositionMode(360,360);
////     m1.SetPosition(pos_ideal_m1);
//////     sleep(1);
////     m1.GetPosition();
////     m1.SetTorque(00);

////m1.SetupPositionMode(360,360);
////m1.Setup_Torque_Mode();

////while((pv<pos_ideal_m1-5)||(pv>pos_ideal_m1+5)){

////    pv=m1.GetPosition();
////    cout<<"----- PV    "<<pv<<endl;
//////    m1.SetPosition(pos_ideal_m1);
////    up =pid.OutputUpdate(pos_ideal_m1-pv);

////    uv =pid1.OutputUpdate(up-m1.GetVelocity());
////    m1.SetTorque(uv);
////    pv=m1.GetPosition();
////    if (pos_ideal_m1-5<pv<pos_ideal_m1+5){
////        cout<<"----- PV2222    "<<pv<<endl;
////        m1.SetTorque(00);
////    }
////}
////m1.SetTorque(00);

//////sleep(2);
//////m1.PrintStatus();
////sleep(1+pos_ideal_m1/360);

//////sleep(3);
////m1.Setup_Velocity_Mode(0,360);
//////cout<<"Getposition"<<pv<<endl;
////m1.SetVelocity(velocity_ideal_m1);

////cout<<m1.GetVelocity();







////     cout<<"Estado motor 2:\n"<<endl;
////     m2.PrintStatus();
////     cout<<"Estado motor 3:\n"<<endl;
////     m3.PrintStatus();

//     /* MAIN DE PRUEBA PARA CONTROL DE POSICION */

////     m1.SetupPositionMode(360*0.47,360); //(Max velocity in degrees/s, acceleration)

////     uint32_t pos_ideal_m1=120; //Write the position of m1 in degrees here

////     m1.SetPosition(m1.DegreeConv(posdegree_m1));
////     m1.FlushBuffer();

////     m1.PrintStatus();

////     sleep(1+posdegree_m1/360150);
////     int posicion_correcta=-1;
////     while(posicion_correcta<0)
////     double pos_real_m1 = m1.GetPosition();
////       cout<< "actual pos: " << pos_real << " [deg]" <<endl;

////     double error_m1 = pos_real_m1-pos_ideal_m1;

////     if(error_m1<0)  //No ha llegado a la posición deseada
////     {controlword
////         m1.Setup_Velocity_Mode(720,360);
////     }
////     else {
////         if(error_m1>0)  //Se ha pasado de la posición deseada
////         {
////            m1.Setup_Velocity_Mode(-720,360);
////         }
////         else{
////             cout<<"Posición deseada alcanzada"<<endl;
////             posicion_correcta=0;
////         }
////     }


////     long x=-180;
////     uint32_t y=x;
////     cout <<y<<endl;
//      sleep(1);



