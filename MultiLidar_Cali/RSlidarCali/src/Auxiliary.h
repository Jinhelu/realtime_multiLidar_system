#ifndef AUXILIARY_H
#define AUXILIARY_H

#include <bits/stdc++.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "GetFrame_RS.h"
#include "Auxiliary_IMU.h"
#include "PointCloudManage.h"
using namespace std;

extern volatile bool ReadLidar32;
extern volatile bool ReadLidar16;
extern int Frequence32;
extern int Frequence16;





typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef pcl::PointXYZI PointT_I;
typedef pcl::PointCloud<PointT_I> PointCloud_I;

typedef pcl::PointXYZRGB PointT_C;
typedef pcl::PointCloud<PointT_C> PointCloud_C;

namespace RslidarInput = robosense::rslidar_input;
namespace Rslidar = robosense::rslidar;

struct IniParam{
    float sor_vox_LeafSize;
    int ThresholdIntensity;
    float ErrrorPointSearchRadius;
    int ErrrorPointNearNum;
    float StdTargetSize;//对角线长度
    double max_x,min_x,max_y,min_y,max_z,min_z;//min_z是雷达离地面高度

    int gridmapNum_x, gridmapNum_y;//x,y两边的删格数目
    double gridScale;//每个删格代表的实际长度 单位：米
    int PixelPerGrid;//显示地图时候一个删格边长占据的像素

    float aboveGround_NoPrecise; //地面精度低时删除点距地面的高度
    float aboveGround; //地面精度正常时删除点距地面的高度
    int groundEstimateNum; //估计地面时迭代随机采样的次数
    float outPlaneDistance; //随机采样时局外点的距离
    int cutAngleYaw; //估计地面时采样的左右边缘角度（deg）
    int cutAnglePitch; //估计地面时采样的俯角（deg）
    float maxDeviaAngle_deg; //有精度地面的最大偏离均值角度
};


template <typename T,typename timeType,typename PCtimeType,typename otherMessType>
class queue_ts{
public:
    queue_ts(int capacity){
        _capacity = capacity;
    }

    void push(T &newElem,timeType &newTime,PCtimeType &PCtime,otherMessType &otherMess){
        lock_guard<mutex> lk(_mut);
        _container.push(newElem);
        _timeContainer.push(newTime);
        _PCtimeContainer.push(PCtime);
        _otherMessContainer.push(otherMess);

        if(_container.size()>_capacity){
            _container.pop();
            _timeContainer.pop();
            _PCtimeContainer.pop();
            _otherMessContainer.pop();
        }
    }

    bool latestElem(T* receiver,timeType* timeReceiver,PCtimeType* PCtimePtr,otherMessType* otherMessPtr){
        lock_guard<mutex> lk(_mut);
        if(_container.empty()) return false;

        *receiver = _container.back();
        *timeReceiver = _timeContainer.back();
        *PCtimePtr = _PCtimeContainer.back();
        *otherMessPtr = _otherMessContainer.back();
        return true;
    }

private:
    queue<T> _container;
    queue<timeType> _timeContainer;
    queue<PCtimeType> _PCtimeContainer;
    queue<otherMessType> _otherMessContainer;
    mutex _mut;
    int _capacity;
};


/*
class LidarReadFlag{
public:
    LidarReadFlag(){
        ReadLidar16 = true;
        ReadLidar32 = true;
    }

    void ReadLidar32Only(){
        lock_guard<mutex> lk(_mut);
        ReadLidar16 = false;
        ReadLidar32 = true;
    }

    void ReadLidar16Only(){
        lock_guard<mutex> lk(_mut);
        ReadLidar32 = false;
        ReadLidar16 = true;

    }

    void ReadLidarBoth(){
        lock_guard<mutex> lk(_mut);
        ReadLidar16 = true;
        ReadLidar32 = true;
    }

    void StopReadLidar(){
        lock_guard<mutex> lk(_mut);
        ReadLidar16 = false;
        ReadLidar32 = false;
    }

    bool Lidar32ReadState(){
        lock_guard<mutex> lk(_mut);
        return ReadLidar32;
    }

    bool Lidar16ReadState(){
        lock_guard<mutex> lk(_mut);
        return ReadLidar16;
    }

private:
    mutex _mut;
    bool ReadLidar16;
    bool ReadLidar32;
};
 */



bool getParameter(string& adress,IniParam& param);

void listenRs_32(robosense::rslidar::RSLidarDecoder<PointXYZITS> &decoder,RslidarInput::Input &InputObj,
              queue_ts<vector<PointCloud_I>,double,long long,IMUState>* PC_I_Ptr_queue,int scanIDTotal,
              queue_ts_IMU &IMUdata_Q);

void listenRs_16(robosense::rslidar::RSLidarDecoder<PointXYZITS> &decoder,RslidarInput::Input &InputObj,
              queue_ts<vector<PointCloud_I>,double,long long,IMUState>* PC_I_Ptr_queue,int scanIDTotal,
              queue_ts_IMU &IMUdata_Q);

bool getLatestFrame(vector<PointCloud_I>& receiver,
                    queue_ts<vector<PointCloud_I>,double,long long,IMUState> &PC_I_Ptr_queue,
                    double& timeStamp, long long& PCTime_u,IMUState &IMUdata);

#endif