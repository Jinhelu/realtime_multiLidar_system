//
// Created by cyn on 20-8-10.
//

#ifndef MUTILIDARTEST_AUXILIARY_IMU_H
#define MUTILIDARTEST_AUXILIARY_IMU_H


#include <bits/stdc++.h>
#include "imu_get_data.h"
#include "unistd.h"
using namespace std;

class queue_ts_IMU{
public:
    queue_ts_IMU(int capacity,string ImuRecordAdress = ""){
        _capacity = capacity;

        if(ImuRecordAdress == "")   {
            cout<< "ImuRecord adress is empty,No IMU record" << endl;
            recordFlag = false;
        }
        else{
            ImuRecordFile32.open(ImuRecordAdress);
            ImuRecordFile16.open(ImuRecordAdress);
            if(!ImuRecordFile32) {
                cout<< "ImuRecord file open failed, can not record IMU!!!" << endl;
                recordFlag = false;
            }
            else {
                cout << "Reading IMU Record: " << ImuRecordAdress << endl;
                recordFlag = true;
            }
        }

    }

    void push(IMUState &newElem,long long &PCtime){
        lock_guard<mutex> lk(_mut);
        _container.push(newElem);
        _PCtimeContainer.push(PCtime);

        if(_container.size()>_capacity){
            _container.pop();
            _PCtimeContainer.pop();
        }
    }

    bool latestElem(IMUState* receiver,long long* PCtimePtr){
        lock_guard<mutex> lk(_mut);
        if(_container.empty()) return false;
        *receiver = _container.back();
        *PCtimePtr = _PCtimeContainer.back();
        return true;
    }

    bool ifOffline(){
        return recordFlag;
    }

    long long readLidarVisitRecord(int LidarType_find,double LidarTimeStamp_find){
        lock_guard<mutex> lk(_mut);

        string temp;
        long long LidarVisitPCTime=0;

        while(getline( (LidarType_find==32?ImuRecordFile32:ImuRecordFile16), temp)){

            if(temp.size()<4) continue;

            stringstream temp_ss(temp);
            string firstWords;
            temp_ss >> firstWords;

            if(firstWords == "IMUtime:"){
                IMUState tempImuState;
                vector<double> Quaternion_temp(4);

                temp_ss >> tempImuState.timeStamp_ms
                        >>firstWords>>Quaternion_temp[0]>>Quaternion_temp[1]>>Quaternion_temp[2]>>Quaternion_temp[3]
                        >>firstWords>> tempImuState.roll >> tempImuState.pitch >> tempImuState.yaw
                        >>firstWords>> tempImuState.acceleration[0] >> tempImuState.acceleration[1] >> tempImuState.acceleration[2]
                        >>firstWords>> tempImuState.angularVelo[0] >> tempImuState.angularVelo[1] >> tempImuState.angularVelo[2]
                        >>firstWords>> tempImuState.velocity[0] >> tempImuState.velocity[1] >> tempImuState.velocity[2]
                        >>firstWords>> tempImuState.PCtime_us;

                tempImuState.Quaternion.x() = Quaternion_temp[0];
                tempImuState.Quaternion.y() = Quaternion_temp[1];
                tempImuState.Quaternion.z() = Quaternion_temp[2];
                tempImuState.Quaternion.w() = Quaternion_temp[3];

                _container.push(tempImuState);
                _PCtimeContainer.push(tempImuState.PCtime_us);

                if(_container.size()>_capacity){
                    _container.pop();
                    _PCtimeContainer.pop();
                }
            }
            else if(firstWords[6] == '*'){
                long long LidarTimestamp;
                int LidarType;
                temp_ss >> LidarType;

                if(LidarType==LidarType_find){
                    temp_ss >> firstWords >> LidarTimestamp >> firstWords >> LidarVisitPCTime;//录数据格式决定以ms进行读取

                    long long deltaTime = (long long)(LidarTimeStamp_find) - LidarTimestamp;

                    if(abs(deltaTime)<700){//认为相等
                        break;
                    }
                    else if(deltaTime>700)//认为IMU中的访问记录太旧
                        continue;
                    else {//认为目前的Lidar数据太旧了
                        LidarVisitPCTime = 0;
                        break;
                    }

                }
            }
        }

        return LidarVisitPCTime;
    }

private:
    mutex _mut;

    queue<IMUState> _container;
    queue<long long> _PCtimeContainer;
    int _capacity;

    //以下均为在读取录的数据时候才用
    ifstream ImuRecordFile32;
    ifstream ImuRecordFile16;
    bool recordFlag;
};

void listenIMU(CallbackHandler &callback, queue_ts_IMU *IMUdata_Q);

bool getLatestIMUdata(IMUState &receiver, queue_ts_IMU &IMUdata_Q,long long &PCTime_u);


#endif //MUTILIDARTEST_AUXILIARY_IMU_H
