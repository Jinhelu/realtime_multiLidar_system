#ifndef POINTCLOUDMANAGE_H
#define POINTCLOUDMANAGE_H

#include <iostream>
using namespace std;

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/keypoints/sift_keypoint.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
//#include "PacketDriver.h"
//#include "PacketDecoder.h"

#include <cmath>

extern bool RECORD_TARGET;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

typedef pcl::PointXYZI PointT_I;
typedef pcl::PointCloud<PointT_I> PointCloud_I;

typedef pcl::PointXYZRGB PointT_C;
typedef pcl::PointCloud<PointT_C> PointCloud_C;

typedef pcl::FPFHSignature33 PointT_FPFH33;
typedef pcl::PointCloud<PointT_FPFH33> PointCloud_FPFH33;

typedef pcl::PointCloud<pcl::Normal> Normal;

struct velocity {
    double Vx;
    double Vy;
    double combine;

    velocity(){
      setVelocity(0,0);
    }

    void setVelocity(double Velox,double Veloy){
      Vx = Velox;
      Vy = Veloy;
      combine = sqrt(pow(Velox,2) + pow(Veloy,2));
    }
};

struct GuidObject{
    PointT posi;
    velocity velo;
    double timeStamp_us;

    GuidObject() {
      posi.x = 0;
      posi.y = 0;
      posi.z = 0;

      velo.setVelocity(0, 0);

      timeStamp_us = 0;
    }

    GuidObject(float posiX,float posiY,float posiZ) {
        posi.x = posiX;
        posi.y = posiY;
        posi.z = posiZ;

        velo.setVelocity(0, 0);

        timeStamp_us = 0;
    }

    double target_distance(){
      return
       sqrt(pow(posi.x, 2) + pow(posi.y, 2) + pow(posi.z, 2));
    }

    double horizontal_angel() {
      return
              180.0 * (atan(posi.y / posi.x)) / 3.1415926f;
    }

    double vertical_angel() {
      return
              180.0 * (atan(posi.z / posi.x)) / 3.1415926f;
    }

};

bool XYZI2XYZ(const PointCloud_I::Ptr&,PointCloud::Ptr&);
bool XYZI2XYZRGB(const PointCloud_I::Ptr&,PointCloud_C::Ptr&);
bool XYZI2XYZRGB_NoRaw(PointCloud_I & raw,PointCloud_C::Ptr result);

/*
//补偿激光雷达角度调整引起的坐标系偏差
//alpha是转角，低头为正  deltaZ是激光雷达中心到安装转轴的距离(单位 米)
bool AdjustPointClound(PacketDecoder::HDLFrame * raw_data,double alpha,double deltaZ);
 */

//直通滤波,限制点云的xyz范围
bool ThroughFilter(const PointCloud_I::Ptr& raw_data,double max_x,double min_x,
                   double max_y,double min_y, double max_z,double min_z);

void outlinerRemove(PointCloud_I& PC_I,int meanK,float StddevMulThresh);

//将点云投影到平面
//                                        原始数据           低头角度      投影平面高度(雷达坐标系下)
bool cast_filter_plane(const PointCloud::Ptr& raw_data,double alpha,double z,
                       const PointCloud::Ptr& data_filtered);

//SIFT关键点检测
//                              输入点云           最小尺度标准差      高斯金字塔组数
bool SIFTkeypoint(const PointCloud_I::Ptr& pointcloud_I, float min_scale, int n_octaves,
//                 组的计算的尺度数              设置关键点检测阈值      输出的关键点
                  int n_scales_per_octave,float min_contrast,const PointCloud_I::Ptr& cloud_temp);

//法线估计////////////////////////////////////////
//                            输入点云        k近邻搜索的k值    估计法线的结果
bool normal(const PointCloud_I::Ptr& pointcloud_I, int k, pcl::PointCloud<pcl::Normal>::Ptr normals);


//PointCloud_FPFH33::Ptr fpfhs(new  PointCloud_FPFH33);//fpfh特征向量
//         特征点    点云整体  法向量   fpfh特征向量   fpfh搜索半径
bool fpfh(const PointCloud_I::Ptr& cloud_temp,PointCloud_I::Ptr pointcloud_I,
          const pcl::PointCloud<pcl::Normal>::Ptr& normals,
          const PointCloud_FPFH33::Ptr& fpfhs,float RadiusSearch);


//用方框框选目标点　待框选点云　方框序号向量　显示handle 方框边长　显示窗口序号
bool BoxPoints(const PointCloud_I::Ptr& pointcloudI_target,  vector<int> &BoxAppendix,
                   const boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,
                   float size = 0.05, int viewport=0);

void GetHighItsityGroup(PointCloud_I::Ptr &RawPCPtr, PointCloud_I::Ptr &ResPCPtr,
                        int &ThresholdItsity,float &ErrrorPointSearchRadius, int &ErrrorPointNearNum);

bool targetPosiAnaly(PointCloud_I::Ptr &PCItsityTarget,GuidObject &guidObject,
                     float &StdTargetSize,float &sor_vox_LeafSize,double &time_stt_us,ofstream &outfile);

bool targetVeloAnaly(GuidObject &guidObject,GuidObject &lastGuidObj);

void RemoveGuid(GuidObject &guidObject,PointCloud_I::Ptr &PC_I_in,PointCloud_I::Ptr &PC_I_out);

void convetPC(PointCloud_I::Ptr PC_I_raw_RS16,Eigen::Isometry3f& Tba);

void loamFeature(vector<PointCloud_I> &laserCloudScans, PointCloud_I &cornerPointsSharp,
                 PointCloud_I &cornerPointsLessSharp, PointCloud_I &surfPointsFlat,
                 PointCloud_I &surfPointsLessFlat, int N_SCANS);

//给定点云匹配其中的平面
//               原始点云   面内的点云的序号向量    拟合后的参数   拟合精度(默认0.01米)
bool plane_match(PointCloud::Ptr &pointcloud,
                 vector<int> &inliers,pcl::ModelCoefficients &PlaneCoeff2Show,
                 float precision = 0.01);

void VoxFilter(PointCloud_I::Ptr &PC_I_input,float sor_vox_LeafSize);

void groundEstimate(int estimateNum,PointCloud::Ptr &pointcloud,
                    pcl::ModelCoefficients &PlaneCoeff2Show,float outDistance,
                    float MaxDeviaAngle_deg,bool &precise);


void cutRawGround(PointCloud_I::Ptr &PC_I_raw_RS16,PointCloud_I::Ptr &PC_I_ground,
                  float vox_LeafSize,int cutAngleYaw_deg,int cutAnglePitch_deg);

void GroundFilter(PointCloud_I::Ptr& PC_I_Input,pcl::ModelCoefficients &PlaneCoeff2Show,
                  float outlineDis,float abovePlaneAngle_deg);

bool cast_filter_Ground(const PointCloud::Ptr& raw_data,pcl::ModelCoefficients &coefficients,
                        const PointCloud::Ptr& data_filtered);
#endif
