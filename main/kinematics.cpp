#include <iostream>

#include "mainlib.h"
#include "math.h"


using namespace std;

int main()
{
    TableKinematics a("../neck-control/arco107.csv");
    vector<double> lengths2(3);

    GeoInkinematics b;
    vector<double> lengths(3);
    double orient=35;
    double incli=10;


    b.GetIK(incli,orient,lengths);
    cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;

    a.GetIK(incli,orient,lengths2);
    cout << "l_1 " << lengths2[0]  << ", l_2 " << lengths2[1] << ", l_3 " << lengths2[2]<<endl;

    return 0;
}

