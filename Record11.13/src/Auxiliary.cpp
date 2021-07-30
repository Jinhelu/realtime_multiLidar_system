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
    return true;
  }
  else return false;
}


void listenRs(robosense::rslidar::RSLidarDecoder<PointXYZITS> &decoder,RslidarInput::Input &InputObj,
              queue_ts<vector<PointCloud_I>,double,long long,IMUState>* PC_I_Ptr_queue,int csanIDTotal,
              queue_ts_IMU<IMUState,long long> &IMUdata_Q){
    vector<PointCloud_I> PC_I_raw_RS(csanIDTotal);
    double timeStamp,timeStampOld = 0;  //单位是us
    long long PCTime_u;
    IMUState IMUdata;

    while(GetFrame_RS(PC_I_raw_RS, decoder, InputObj, timeStamp, PCTime_u,IMUdata,IMUdata_Q)){

        if(fabs(timeStamp - timeStampOld )<1000) {
            cout << "RSlidar " << (csanIDTotal==32?"32":"16") << " get an old Frame." << endl;
            usleep(5*1000);
            continue;
        }

        timeStampOld = timeStamp;

        PC_I_Ptr_queue->push(PC_I_raw_RS,timeStamp,PCTime_u,IMUdata);
    }
}


bool getLatestFrame(vector<PointCloud_I>& receiver,
                    queue_ts<vector<PointCloud_I>,double,long long,IMUState> &PC_I_Ptr_queue,
                    double& timeStamp, long long& PCTime_u,IMUState &IMUdata){
    return PC_I_Ptr_queue.latestElem(&receiver,&timeStamp,&PCTime_u,&IMUdata);
}

