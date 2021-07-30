#include "Auxiliary.h"


bool getParameter(string& adress,IniParam& param) {
  fstream paramFile(adress,fstream::in);
  string name;
  if(paramFile){
    if(!(paramFile >> name >> param.sor_vox_LeafSize)) return false;
    if(!(paramFile >> name >> param.ThresholdIntensity)) return false;
    if(!(paramFile >> name >> param.ErrrorPointSearchRadius)) return false;
    if(!(paramFile >> name >> param.ErrrorPointNearNum)) return false;
    if(!(paramFile >> name >> param.StdTargetSize)) return false;
    if(!(paramFile >> name >> param.max_x)) return false;
    if(!(paramFile >> name >> param.min_x)) return false;
    if(!(paramFile >> name >> param.max_y)) return false;
    if(!(paramFile >> name >> param.min_y)) return false;
    if(!(paramFile >> name >> param.max_z)) return false;
    if(!(paramFile >> name >> param.min_z)) return false;
    if(!(paramFile >> name >> param.gridmapNum_x)) return false;
    if(!(paramFile >> name >> param.gridmapNum_y)) return false;
    if(!(paramFile >> name >> param.gridScale)) return false;
    if(!(paramFile >> name >> param.PixelPerGrid)) return false;

      if(!(paramFile >> name >> param.aboveGround_NoPrecise)) return false;
      if(!(paramFile >> name >> param.aboveGround)) return false;
      if(!(paramFile >> name >> param.groundEstimateNum)) return false;
      if(!(paramFile >> name >> param.outPlaneDistance)) return false;
      if(!(paramFile >> name >> param.cutAngleYaw)) return false;
      if(!(paramFile >> name >> param.cutAnglePitch)) return false;
      if(!(paramFile >> name >> param.maxDeviaAngle_deg)) return false;

      return true;
  }
  else return false;
}


void listenRs_32(robosense::rslidar::RSLidarDecoder<PointXYZITS> &decoder,RslidarInput::Input &InputObj,
              queue_ts<vector<PointCloud_I>,double,long long,IMUState>* PC_I_Ptr_queue,int scanIDTotal,
              queue_ts_IMU &IMUdata_Q){
    //确定雷达类型和几个全局变量引用
    int *FrequencePtr;
    int stopReadNum;

    if(scanIDTotal==32) {
        FrequencePtr = &Frequence32;
    }
    else {
        FrequencePtr = &Frequence16;
    }


    vector<PointCloud_I> PC_I_raw_RS(scanIDTotal);
    double timeStamp;
    long long PCTime_u;
    IMUState IMUdata;
    while(true){


        if(IMUdata_Q.ifOffline()){
            int timePerFrame = int(1.0f/(*FrequencePtr)*1000*1000);
            usleep(timePerFrame);

            bool readFlag;
            if(scanIDTotal==32) readFlag=ReadLidar32;
            else readFlag=ReadLidar16;

            if(!readFlag) {
                usleep(timePerFrame);
                cout<< "Lidar" << (scanIDTotal==32?32:16) << " is waiting..." << endl;
                continue;
            }
        }

        cout<< "Lidar" << (scanIDTotal==32?32:16) << " is running..." << endl;

        if(GetFrame_RS(PC_I_raw_RS, decoder, InputObj, timeStamp, PCTime_u,IMUdata,IMUdata_Q)){
            PC_I_Ptr_queue->push(PC_I_raw_RS,timeStamp,PCTime_u,IMUdata);
            cout << " Get " << (scanIDTotal==32?32:16) << "-line pointcloud" << 
            "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl;
        }
    }
}


void listenRs_16(robosense::rslidar::RSLidarDecoder<PointXYZITS> &decoder,RslidarInput::Input &InputObj,
              queue_ts<vector<PointCloud_I>,double,long long,IMUState>* PC_I_Ptr_queue,int scanIDTotal,
              queue_ts_IMU &IMUdata_Q){
    //确定雷达类型和几个全局变量引用
    int *FrequencePtr;
    int stopReadNum;

    if(scanIDTotal==32) {
        FrequencePtr = &Frequence32;
    }
    else {
        FrequencePtr = &Frequence16;
    }


    vector<PointCloud_I> PC_I_raw_RS(scanIDTotal);
    double timeStamp;
    long long PCTime_u;
    IMUState IMUdata;
    while(true){


        if(IMUdata_Q.ifOffline()){
            int timePerFrame = int(1.0f/(*FrequencePtr)*1000*1000);
            usleep(timePerFrame);

            bool readFlag;
            if(scanIDTotal==32) readFlag=ReadLidar32;
            else readFlag=ReadLidar16;

            if(!readFlag) {
                usleep(timePerFrame);
                cout<< "Lidar" << (scanIDTotal==32?32:16) << " is waiting..." << endl;
                continue;
            }
        }

        cout<< "Lidar" << (scanIDTotal==32?32:16) << " is running..." << endl;

        if(GetFrame_RS(PC_I_raw_RS, decoder, InputObj, timeStamp, PCTime_u,IMUdata,IMUdata_Q)){
            PC_I_Ptr_queue->push(PC_I_raw_RS,timeStamp,PCTime_u,IMUdata);
            cout << " Get " << (scanIDTotal==32?32:16) << "-line pointcloud" << 
            "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl;
        }
    }
}


bool getLatestFrame(vector<PointCloud_I>& receiver,
                    queue_ts<vector<PointCloud_I>,double,long long,IMUState> &PC_I_Ptr_queue,
                    double& timeStamp, long long& PCTime_u,IMUState &IMUdata){
    return PC_I_Ptr_queue.latestElem(&receiver,&timeStamp,&PCTime_u,&IMUdata);
}

