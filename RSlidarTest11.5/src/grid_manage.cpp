#include <iostream>
#include <string>
using namespace std;
#include <opencv2/opencv.hpp>
using namespace cv;

#include "grid.h"
#include "grid_manage.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <math.h>

//判断点在线段哪一侧      横坐标   纵坐标    端点横坐标   端点纵坐标
//上侧为 true , 下侧为false;
bool LinePointJudge (double x,double y,double EndX,double EndY){
  double yflag = (x*EndY)/EndX ;
  return (y>yflag);

}

//判断线段是否经过矩形 矩形顶点 x最小值 y最小值        长       宽   过原点线段端点 x    y
//经过该矩形为true  不经过为false
bool LineRecJudge (double x,double y,double deltaX,double deltaY,double EndX,double EndY){
  bool criterion_1 = LinePointJudge (x,       y,       EndX,EndY);
  bool criterion_2 = LinePointJudge (x+deltaX,y,       EndX,EndY);
  bool criterion_3 = LinePointJudge (x,       y+deltaY,EndX,EndY);
  bool criterion_4 = LinePointJudge (x+deltaX,y+deltaY,EndX,EndY);

  return !(criterion_1 == criterion_2 && criterion_3 == criterion_4 && criterion_1 == criterion_3);
}

//把pcl投影点云数据转化为GridMap类   输入 点云投影指针     删格地图类
bool Pcl2DtoGridMap(const PointCloud::Ptr& raw_data,GridMap& gridmap,
           //  要转换的X坐标的范围    Y坐标的范围
                    double Xscope_max,double Xscope_min,double Yscope_max,double Yscope_min, double gridScale) {

  //int Xzero = (gridmap.LaserNumX()+1);
  //int Yzero = (gridmap.LaserNumY()+1);              //地图对应于删格时的原点序列(grid坐标系)

  //double deltaX = Xscope*2/gridmap.WidthNum();       //每个删格的X轴分辨率 单位：米
  //double deltaY = Yscope*2/gridmap.HeightNum();      //每个删格的Y轴分辨率 单位：米

  double deltaX = gridScale;       //每个删格的X轴分辨率 单位：米
  double deltaY = gridScale;      //每个删格的Y轴分辨率 单位：米

  //对所有点进行分析
  for (int i=0; i<raw_data->points.size(); i++) {

    //对scope范围内的点进行分析,雷达中的点一定是占位点，是1
    if (raw_data -> points[i].x < Xscope_max && raw_data -> points[i].x > Xscope_min  &&
        raw_data -> points[i].y < Yscope_max && raw_data -> points[i].y > Yscope_min  ) {

      int End_XGrid_No,End_YGrid_No;
      int LineGridNumX = abs(int(raw_data->points[i].x/deltaX) );
      int LineGridNumY = abs(int(raw_data->points[i].y/deltaY) );  //Y坐标删格整数（不带fuhao）

      //提取坐标的符号 正号为1 负号为-1
      int SignDataX,SignDataY;
      if (raw_data->points[i].x > 0){SignDataX=1;}
      else {SignDataX=-1;}
      if (raw_data->points[i].y > 0){SignDataY=1;}
      else {SignDataY=-1;}

//////////////////////////////端点删格判断///////////////////////////////////////
      if (raw_data->points[i].x < 0){
        End_XGrid_No = (SignDataX)*LineGridNumX +(gridmap.LaserNumX()+1);}
      else{ End_XGrid_No = (SignDataX)*LineGridNumX +(gridmap.LaserNumX()+1) +1; }

      if (raw_data->points[i].y < 0){
        End_YGrid_No = (-SignDataY)*LineGridNumY +(gridmap.LaserNumY()+1) +1;}
      else{ End_YGrid_No = (-SignDataY)*LineGridNumY +(gridmap.LaserNumY()+1);}

      //更新删格状态
      gridmap.ModifyGridState(End_XGrid_No,End_YGrid_No,OCCU);
    }
  }


    //对激光无反射回来值的点，添加到点云中，认为其在无穷远处返回  点的间隔为gridScale/2
    float tempZ = raw_data -> points[0].z;
    float tempNumber;
    //填充极限Yscope_max
    tempNumber=2;
    while(tempNumber<Xscope_max){
        tempNumber+=gridScale;
        PointT tempPoint(tempNumber,float(Yscope_max-0.2),tempZ);
        raw_data->points.push_back(tempPoint);
    }
    //填充极限Yscope_min
    tempNumber=2;
    while(tempNumber<Xscope_max){
        tempNumber+=gridScale;
        PointT tempPoint(tempNumber,float(Yscope_min+0.2),tempZ);
        raw_data->points.push_back(tempPoint);
    }
    //填充极限Xscope_max
    tempNumber=(float)Yscope_min;
    while(tempNumber<(float)Yscope_max){
        tempNumber+=gridScale;
        PointT tempPoint(float(Xscope_max-0.2),tempNumber,tempZ);
        raw_data->points.push_back(tempPoint);
    }



  //对删格的所有点进行占位判断后再进行激光束经过判断
  //防止造成后面点对前面点的干扰,出现两个障碍物之间的区域为未占用的情况
  //对所有点进行分析
  for (int i=0; i<raw_data->points.size(); i++){

    //对scope范围内的点进行分析
    if (raw_data -> points[i].x < Xscope_max && raw_data -> points[i].x > Xscope_min  &&
        raw_data -> points[i].y < Yscope_max && raw_data -> points[i].y > Yscope_min  ) {

      int End_XGrid_No,End_YGrid_No;
      int LineGridNumX = abs(int(raw_data->points[i].x/deltaX) );
      int LineGridNumY = abs(int(raw_data->points[i].y/deltaY) );  //Y坐标删格整数（不带fuhao）

      ////提取坐标的符号 正号为1 负号为-1
      int SignDataX,SignDataY;
      if (raw_data->points[i].x > 0){SignDataX=1;}
      else {SignDataX=-1;}
      if (raw_data->points[i].y > 0){SignDataY=1;}
      else {SignDataY=-1;}

/////////////////////////////从原点开始追溯到端点///////////////////////////////////
//当前判断删格横坐标 X  删格横坐标 X    预判断X    预判断Y
      int startNumX, startNumY, judgeNumX, judgeNumY;

      //对象限的分析确定最初始的实际point原点附近的删格坐标
      if(SignDataX == 1) {startNumX = (gridmap.LaserNumX()+1)+1;}
      else {startNumX = (gridmap.LaserNumX()+1);}
      if(SignDataY == 1) {startNumY = (gridmap.LaserNumY()+1);}
      else {startNumY = (gridmap.LaserNumY()+1)+1;}

      //向前追溯
      for(int j=0 ; j<LineGridNumX+LineGridNumY; j++){

        //如果追溯到已经占位的删格点则停止,后面的删格必然是未知的
        if (gridmap.GetGridState(startNumX,startNumY)==UNKNOW ){
          gridmap.ModifyGridState(startNumX,startNumY,FREE);
        }
        else if (gridmap.GetGridState(startNumX,startNumY)==OCCU){
          break;
        }

        //向X方向向前一步进行预判
        judgeNumX = startNumX+SignDataX;
        judgeNumY = startNumY;

        //预判删格的左下角point坐标
        double judgeRecX = (judgeNumX - (gridmap.LaserNumX()+1) -1)*deltaX;
        double judgeRecY = -(judgeNumY - (gridmap.LaserNumY()+1))*deltaX;

        //判断该删格是否被经过
        bool flag= LineRecJudge (judgeRecX,judgeRecY,deltaX,deltaY,
                                 raw_data->points[i].x,raw_data->points[i].y);

        //如果该删格被经过,该删格作为现在判断点
        if(flag){ startNumX=judgeNumX; }
        //如果该删格未被经过,向Y方向试探作为现在判断点
        else { startNumY = startNumY - SignDataY; }
      }

    }
  }

  return true;
};

//把GridMap数据转化为opencv图像并显示
//                  删格数据  每个删格占据的像素
Mat GridShow(GridMap gridmap,int delta,const string& ImageName,bool delay){

  //地图总像素大小
  int width = gridmap.WidthNum()*delta;
  int height = gridmap.HeightNum()*delta;

  Mat MapImage(width,height,CV_8UC1,Scalar(180));//创建原始图像

  for(int j=0;j<gridmap.HeightNum();j++){
    //对每一行y
    for(int i=0;i<gridmap.WidthNum();i++){
      //对每行的每个删格x
        if (gridmap.GetGridState(i,j) == FREE){
          //空闲状态为白色
          //画矩形（图像，一个顶点，对角顶点，颜色，线条粗细，线条类型）
          rectangle(MapImage,
                    Point(i*delta,j*delta),Point(i*delta+delta,j*delta+delta),
                    Scalar(255),-1,8);
        }
        else if(gridmap.GetGridState(i,j) == OCCU){
          //占用状态为黑色
          rectangle(MapImage,
                    Point(i*delta,j*delta),Point(i*delta+delta,j*delta+delta),
                    Scalar(0),-1,8);
        }
    }//对每行的每个删格x
  }//对每一行y

  imshow(ImageName,MapImage);

  if (delay){cout<<"按下回车键继续"<< endl; waitKey(0);}
  else waitKey(3);//3ms等待

  return MapImage;
}


//opencv图像并显示起点和终点
//             起点终点标志位 栅格x 栅格y 每个删格占据的像素   地图容器     窗口名字
bool PointShow(int flag, int x,int y,int delta,const Mat& MapImage,string ImageName,bool delay){
  if(flag!=0 && flag!=1) cerr << "输入起点或者终点标志位有误！" << endl;
  else{
    //flag=0表示是起点
    if(flag == 0){
      Mat StartIcon = imread(startIconAdrress,0);
      if(StartIcon.data == nullptr) {cout<< "图标文件未找到！"<< endl; return 0;}
      Mat imageROI = MapImage(Rect(x*delta-StartIcon.cols/2,y*delta-StartIcon.rows/2,
                                   StartIcon.cols,StartIcon.rows));
      addWeighted(imageROI,0.3,StartIcon,0.7,0.,imageROI);
      imshow(ImageName,MapImage);
    }
    //flag=1表示是终点
    else{
      Mat StartIcon = imread(endIconAdrress,0);//设为0 通道数为1
      if(StartIcon.data == nullptr) {cout<< "图标文件未找到！"<< endl; return 0;}

      Mat imageROI = MapImage(Rect(x*delta-StartIcon.cols/2,y*delta-StartIcon.rows/2,
                                   StartIcon.cols,StartIcon.rows));
      addWeighted(imageROI,0.3,StartIcon,0.7,0.,imageROI);
      imshow(ImageName,MapImage);
    }
  }

  if (delay){cout<<"按下回车键继续"<< endl; waitKey(0);}
  else waitKey(3);//3ms等待
  return true;
}


void RealCordToGridCord(float x,float y,float gridScale,GridMap& gridmap,int &coloum_grid,int &row_grid){

  int LineGridNumX = abs(int(x/gridScale) );
  int LineGridNumY = abs(int(y/gridScale) );  //Y坐标删格整数（不带fuhao）

  //提取坐标的符号 正号为1 负号为-1
  int SignDataX,SignDataY;
  if (x > 0){SignDataX=1;}
  else {SignDataX=-1;}
  if (y > 0){SignDataY=1;}
  else {SignDataY=-1;}

//////////////////////////////端点删格判断///////////////////////////////////////
  if (x < 0){
    coloum_grid = (SignDataX)*LineGridNumX +(gridmap.LaserNumX()+1);}
  else{ coloum_grid = (SignDataX)*LineGridNumX +(gridmap.LaserNumX()+1) +1; }

  if (y < 0){
    row_grid = (-SignDataY)*LineGridNumY +(gridmap.LaserNumY()+1) +1;}
  else{ row_grid = (-SignDataY)*LineGridNumY +(gridmap.LaserNumY()+1);}

}



