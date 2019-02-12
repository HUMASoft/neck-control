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


  // sleep(3);



       double incSensor,oriSensor;
       double dts=0.02;
//        ToolsFControl tools;
//        tools.SetSamplingTime(dts);

       for (double t=0;t<20; t+=dts)
       {
           usleep(dts*1000*1000);
//          tools.WaitSamplingTime();
//            tilt.ReadSensor(incSensor,oriSensor);
           incSensor = tilt.ReadInclination();
           oriSensor = tilt.ReadOrientation();
           cout << "incli_sen: " << incSensor << " , orient_sen: " << oriSensor <<  endl;
//          graph << t << " , " << incli  << " , " << incSensor << " , " << orient << " , " << oriSensor << endl;

       }



   return 0;


}


