//
// Created by cyn on 20-8-10.
//

#ifndef MUTILIDARTEST_AUXILIARY_IMU_H
#define MUTILIDARTEST_AUXILIARY_IMU_H


#include <bits/stdc++.h>
#include <fstream>
#include "unistd.h"
#include "imu_get_data.h"
using namespace std;

template <typename T,typename PCtimeType>
class queue_ts_IMU{
public:
    queue_ts_IMU(int capacity,string ImuRecordAdress = ""){
        _capacity = capacity;

        if(ImuRecordAdress == "")   {
            cout<< "ImuRecord adress is empty,No IMU record" << endl;
            recordFlag = false;
        }
        else{
            ImuRecordFile.open(ImuRecordAdress);
            if(!ImuRecordFile) {
                cout<< "ImuRecord file open failed, can not record IMU!!!" << endl;
                recordFlag = false;
            }
            else recordFlag = true;
        }

    }

    void push(T &newElem,PCtimeType &PCtime){
        lock_guard<mutex> lk(_mut);
        _container.push(newElem);
        _PCtimeContainer.push(PCtime);

        if(_container.size()>_capacity){
            _container.pop();
            _PCtimeContainer.pop();
        }
    }

    bool latestElem(T* receiver,PCtimeType* PCtimePtr){
        lock_guard<mutex> lk(_mut);
        if(_container.empty()) return false;
        *receiver = _container.back();
        *PCtimePtr = _PCtimeContainer.back();
        return true;
    }

    bool writeLatest(){
        lock_guard<mutex> lk(_mut);
        if(_container.empty() || !recordFlag) return false;

        T receiver = _container.back();
        receiver.write(ImuRecordFile);

        PCtimeType PCtimeRec = _PCtimeContainer.back();
        ImuRecordFile << "PCtime: " << PCtimeRec << endl;
        return true;
    }

    bool writeLidarVisit(int LidarScanNum,long long PCtime,double LidarTimestamp){
        lock_guard<mutex> lk(_mut);
        if(!recordFlag) return false;

        ImuRecordFile << endl << string(10,'*') << "VisitLidar: " << (LidarScanNum==32?"32":"16")
                      << " LidarTimestamp_ms: " <<  (long long)(LidarTimestamp)
                      << " VisitPCtime: " <<  PCtime
                      << endl;
        return true;
    }

private:
    queue<T> _container;
    queue<PCtimeType> _PCtimeContainer;
    int _capacity;
    ofstream ImuRecordFile;
    bool recordFlag;

    mutex _mut;
};

void listenIMU(CallbackHandler &callback, queue_ts_IMU<IMUState,long long> *IMUdata_Q);

bool getLatestIMUdata(IMUState &receiver, queue_ts_IMU<IMUState,long long> &IMUdata_Q,long long &PCTime_u);


#endif //MUTILIDARTEST_AUXILIARY_IMU_H
