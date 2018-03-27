#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
using namespace std;

///base abstract kinematics class

class Kinematics
{
public:
    Kinematics();

private:
};

///Table lookup kinematics

class TableKinematics : public Kinematics
{
public:
    TableKinematics();
    long GetIK(long theta, long phi, vector<double> &lengths);

private:
    long Initialize(string csvfileName, vector<int> tableDimensions);
    vector < vector<long> > lookupIndex;
    vector < vector<double> > lookupTable;
};

#endif // KINEMATICS_H

#ifndef _PID_H_
#define _PID_H_

class PIDImpl;
class PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID( double dt, double max, double min, double Kp, double Kd, double Ki );

        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double setpoint, double pv );
        ~PID();

    private:
        PIDImpl *pimpl;
};

#endif
