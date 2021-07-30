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


void listenRs(robosense::rslidar::RSLidarDecoder<PointXYZITS> &decoder,RslidarInput::Input &InputObj,
              queue_ts<vector<PointCloud_I>,double,long long>* PC_I_Ptr_queue,int csanIDTotal,
              bool offLineFlag,int frequence,volatile bool &SensorFlag){
    vector<PointCloud_I> PC_I_raw_RS(csanIDTotal);
    double timeStamp;
    long long PCTime_u;
    while(SensorFlag && GetFrame_RS(PC_I_raw_RS, decoder, InputObj, timeStamp, PCTime_u,offLineFlag,frequence)){
        PC_I_Ptr_queue->push(PC_I_raw_RS,timeStamp,PCTime_u);
    }
}



bool getLatestFrame(vector<PointCloud_I>& receiver,
                    queue_ts<vector<PointCloud_I>,double,long long> &PC_I_Ptr_queue,
                    double& timeStamp, long long& PCTime_u){
    return PC_I_Ptr_queue.latestElem(&receiver,&timeStamp,&PCTime_u);
}

