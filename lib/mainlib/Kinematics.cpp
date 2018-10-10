#include "Kinematics.h"

Kinematics::Kinematics()
{

}

///Table lookup kinematics implementation

TableKinematics::TableKinematics()
{

    Initialize("ik.csv", vector<int>{40,360});
}



TableKinematics::TableKinematics(string path)
{

    Initialize(path, vector<int>{40,360});
}

long TableKinematics::GetIK(long theta, long phi, vector<double> & lengths)
{
    //  cout << lookupIndex.size();
    long index = lookupIndex[theta][phi];
    if (lengths.size()!=lookupTable[index].size())
    {
        cout << "Wrong size." << endl;
        return -1;
    }

    lengths = lookupTable[index];

    return 0;
}

long TableKinematics::Initialize(string csvfileName, vector<int> tableDimensions)
{

    ifstream csv;
    csv.open(csvfileName);
    csv.seekg(0);

    string line;

    double i1,i2;
    double l1,l2,l3;

    getline(csv,line);


    lookupIndex.resize(tableDimensions[0]);
    for (int i=1; i<lookupIndex.size(); i++)
    {
        lookupIndex[i].resize(tableDimensions[1]);

        for (int j=0; j<lookupIndex[i].size(); j++)
        {
            //get line from file
            getline(csv,line);
            istringstream ss(line);
            //compare index
            ss >> i1;
            ss >> i2;

            if( (i1!=i) | (i2!=j) )
            {
                //cout << "line : " << line;
                cout << "index : ";
                cout << "i " << i << ", i1 " << i1 << ", j " << j << ", i2 " << i2 << endl;
            }
            //compression
            ss >> line;
            //and the values
            ss >> l1;
            ss >> l2;
            ss >> l3;


            //store index
            lookupIndex[i][j]=lookupTable.size();
            //add another line to table
            lookupTable.push_back(vector<double>{l1,l2,l3});


        }

    }

    return 0;

}

#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
//#include "pid.h"

using namespace std;

class PIDImpl
{
    public:
        PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki );
        ~PIDImpl();
        double calculate( double setpoint, double pv );

    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
};


PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki )
{
    pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}
double PID::calculate( double setpoint, double pv )
{
    return pimpl->calculate(setpoint,pv);
}
PID::~PID()
{
    delete pimpl;
}


/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
}

double PIDImpl::calculate( double setpoint, double pv )
{

    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

PIDImpl::~PIDImpl()
{
}

#endif
