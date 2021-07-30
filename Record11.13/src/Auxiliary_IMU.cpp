//
// Created by cyn on 20-8-10.
//
#include "Auxiliary_IMU.h"

bool getLatestIMUdata(IMUState &receiver, queue_ts_IMU<IMUState,long long> &IMUdata_Q, long long &PCTime_u){
    return IMUdata_Q.latestElem(&receiver,&PCTime_u);
}


void listenIMU(CallbackHandler &callback, queue_ts_IMU<IMUState,long long> *IMUdata_Q){
    IMUState imuData;
    int64_t oldIMUtime = 0;

    while(true){
        if(!imuGet(callback, imuData)) continue;
        if(imuData.timeStamp_ms == oldIMUtime) {
            usleep(1*1000);
            continue;
        }
        else oldIMUtime = imuData.timeStamp_ms;
        /*
        cout << "Q: " << imuData.Quaternion.w() << ", "
             << imuData.Quaternion.x() << ", "<< imuData.Quaternion.y() << ", "<< imuData.Quaternion.z()  <<endl;
        cout << "roll: " <<imuData.roll << " pitch: " <<imuData.pitch << " yaw: " <<imuData.yaw << endl;
        */
        long long PCTime_u;
        PCTime_u = imuData.PCtime_us;
        IMUdata_Q->push(imuData,PCTime_u);


        if( !(IMUdata_Q->writeLatest()) ) cout << "record IMU failed!" << endl;

    }
}


