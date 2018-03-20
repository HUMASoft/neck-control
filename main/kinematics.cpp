#include <iostream>

#include "mainlib.h"


using namespace std;

int main()
{


    TableKinematics a;
    vector<double> lengths(3);
    a.GetIK(10,10,lengths);
    cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2];


    return 0;
}

