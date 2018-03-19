#include "Kinematics.h"

Kinematics::Kinematics()
{

}

///Table lookup kinematics implementation

TableKinematics::TableKinematics()
{

    Initialize("ik.csv", vector<int>{40,360});
}

long TableKinematics::Initialize(string csvfileName, vector<int> tableDimensions)
{

    ifstream csv;
    csv.open(csvfileName);
    csv.seekg(0);

    string line;


    for (int i=0; i<10; i++)
    {
        getline(csv,line);
        istringstream ss(line);
        ss >> line;
        cout << line << endl;
    }


//    lookupIndex.resize(tableDimensions[0]);
//    for (int i=0; i<lookupIndex.size(); i++)
//    {
//        vector<long> subindex1(tableDimensions[1],0);
//        for (int j=0; j<subindex1.size(); j++)
//        {
//            //add another line to table
//            lookupTable.push_back(vector<double>{i,j,0});
//            //and store index
//            lookupIndex[i][j]=lookupTable.size();

//        }

//    }

    return 0;

}
