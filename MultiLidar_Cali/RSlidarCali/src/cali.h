//
// Created by cyn on 20-12-9.
//

#ifndef MUTILIDARCALI_CALI_H
#define MUTILIDARCALI_CALI_H

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
//pcl::RandomSampleConsensus

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "PointCloudManage.h"
//#include "PacketDriver.h"
//#include "PacketDecoder.h"

#include <cmath>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

typedef pcl::PointXYZI PointT_I;
typedef pcl::PointCloud<PointT_I> PointCloud_I;

typedef pcl::PointXYZRGB PointT_C;
typedef pcl::PointCloud<PointT_C> PointCloud_C;


void cutCloud(PointCloud_I::Ptr &PC_I_raw,int pitchMax_deg,int pitchMin_deg,int yawMax_deg,int yawMin_deg );

bool groundEstimate_cali(int estimateNum,PointCloud::Ptr &pointcloud,
                         pcl::ModelCoefficients &PlaneCoeff2Show,float outDistance,
                         float MaxDeviaAngle_deg,bool &precise,ofstream &outfile);

bool wallEstimate_cali(int estimateNum,PointCloud::Ptr &pointcloud,
                       pcl::ModelCoefficients &PlaneCoeff2Show,float outDistance,
                       float MaxDeviaAngle_deg,bool &precise,ofstream &outfile);

void PlaneFilter(PointCloud_I::Ptr& PC_I_Input,pcl::ModelCoefficients &PlaneCoeff2Show,float outlineDis);

bool plane_cali16(int estimateNum,PointCloud::Ptr &pointcloud_raw,
                  pcl::ModelCoefficients &PlaneCoeff2Show,float outDistance,
                  int divideAngle_deg,bool planeFlag,float MaxDeviaAngle_deg,bool &precise);
#endif //MUTILIDARCALI_CALI_H
