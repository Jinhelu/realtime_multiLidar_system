#include <bits/stdc++.h>
using namespace std;

int main() {
    string InAdress = "../../ImuRecord_raw.txt";
    string OutAdress = "../record_modi/ImuRecord_modi.txt";

    ifstream infile(InAdress);
    if(!infile) cout << "can not find input file!" << endl;
    ofstream outfile(OutAdress);
    if(!outfile) cout << "can not find output file!" << endl;

    string temp;


    while(getline(infile,temp)){

        if(temp.size()<4) continue;

        stringstream temp_ss(temp);
        string firstWords;
        temp_ss >> firstWords;


        outfile << endl;
        if(firstWords == "VisitLidar:")
            outfile << string(10,'*');
        outfile <<  temp << endl;


    }
    return 0;
}
