#ifndef GRID_MANAGE_H
#define GRID_MANAGE_H

#include <iostream>
#include <string>
using namespace std;
#include <opencv2/opencv.hpp>
using namespace cv;

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <math.h>
#include "grid.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


extern bool RECORD_TARGET;

bool LinePointJudge (double x,double y,double EndX,double EndY);

bool LineRecJudge (double x,double y,double deltaX,double deltaY,double EndX,double EndY);

bool Pcl2DtoGridMap(const PointCloud::Ptr& raw_data,GridMap& gridmap,
        //  要转换的X坐标的范围    Y坐标的范围
                    double Xscope_max,double Xscope_min,double Yscope_max,double Yscope_min, double gridScale) ;

Mat GridShow(GridMap gridmap,int delta,const string& ImageName,bool delay=true);

//           起点终点标志位 栅格x 栅格y 每个删格占据的像素   地图容器
bool PointShow(int flag,int x,int y,int delta,const Mat& MapImage,
//             窗口名字  是否等待按键
               string ImageName,bool delay=true);

void RealCordToGridCord(float x,float y,float gridScale,GridMap& gridmap,int &coloum_grid,int &row_grid);

//#include "grid_manage.cpp"

#endif
