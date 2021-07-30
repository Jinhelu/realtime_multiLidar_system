//
// Created by cyn on 20-1-11.
//

#ifndef RSLIDARTEST_GETFRAME_RS32_H
#define RSLIDARTEST_GETFRAME_RS32_H

#include <iostream>
#include <vector>
using namespace std;
#include "../RSDecoder/input.h"
#include "../RSDecoder/rslidar_decoder.hpp"
#include "../RSDecoder/rslidar_packet.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sys/time.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef pcl::PointXYZI PointT_I;
typedef pcl::PointCloud<PointT_I> PointCloud_I;

typedef pcl::PointXYZRGB PointT_C;
typedef pcl::PointCloud<PointT_C> PointCloud_C;

namespace RslidarInput = robosense::rslidar_input;
namespace Rslidar = robosense::rslidar;

struct PointXYZITS{
    float x;
    float y;
    float z;
    float intensity;
    float timeStamp;
    int scanID;
};

bool initializeParam32(robosense::rslidar::ST_Param &param);
bool initializeParam16(robosense::rslidar::ST_Param &param);

bool GetFrame_RS(vector<PointCloud_I>& receiver, robosense::rslidar::RSLidarDecoder<PointXYZITS> &decoder,
                 RslidarInput::Input &InputObj, double &timestampFirst,long long &PCTime_u,
                 bool offLineFlag,int frequence);


#endif //RSLIDARTEST_GETFRAME_RS32_H
