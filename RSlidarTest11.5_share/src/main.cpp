#include "bits/stdc++.h"
#include "cyn_include/MutiLidar.h"
#include "cyn_include/LidarMap_t.h"
using namespace std;

string paramAdress = "../cyn_Param/parameter.txt";
string startIconAdrress = "../cyn_Param/icon/start_15.jpg";
string endIconAdrress = "../cyn_Param/icon/end_15.jpg";


int main(){
    LidarMap_t lidarMap_th(100);

    volatile bool RunFlag = true;
    //string pcapAdress = "../11-07-11-15-mix.pcap";
    string pcapAdress = "../11-07-11-15-mix.pcap";
    thread LidarThread(LidarMapThreadFun,ref(RunFlag),ref(lidarMap_th),pcapAdress);

    usleep(100*1000*1000);
    RunFlag = false;
    
    LidarThread.join();

    cout << endl << "main exit in usually." << endl;

    return 0;
};


