#include "Kinematics.h"

Kinematics::Kinematics()
{

}

///Table lookup kinematics implementation

TableKinematics::TableKinematics()
{

    Initialize("ik.csv", vector<int>{41,360});
}



TableKinematics::TableKinematics(string path)
{

    Initialize(path, vector<int>{41,360});
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
