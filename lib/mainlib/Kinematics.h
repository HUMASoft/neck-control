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
    TableKinematics(string path);
    long GetIK(long theta, long phi, vector<double> &lengths);

private:
    long Initialize(string csvfileName, vector<int> tableDimensions);
    vector < vector<long> > lookupIndex;
    vector < vector<double> > lookupTable;
};


#endif // KINEMATICS_H

