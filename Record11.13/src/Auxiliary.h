#ifndef AUXILIARY_H
#define AUXILIARY_H

#include <bits/stdc++.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "GetFrame_RS.h"
#include "Auxiliary_IMU.h"
#include "PointCloudManage.h"
using namespace std;


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



bool getParameter(string& adress,IniParam& param);

void listenRs(robosense::rslidar::RSLidarDecoder<PointXYZITS> &decoder,RslidarInput::Input &InputObj,
              queue_ts<vector<PointCloud_I>,double,long long,IMUState>* PC_I_Ptr_queue,int csanIDTotal,
              queue_ts_IMU<IMUState,long long> &IMUdata_Q);

bool getLatestFrame(vector<PointCloud_I>& receiver,
                    queue_ts<vector<PointCloud_I>,double,long long,IMUState> &PC_I_Ptr_queue,
                    double& timeStamp, long long& PCTime_u,IMUState &IMUdata);

#endif