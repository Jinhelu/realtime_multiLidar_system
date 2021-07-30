#include <iostream>
#include <cstring>
#include <vector>
using namespace std;
#include "../RSDecoder/input.h"
#include "../RSDecoder/rslidar_decoder.hpp"
#include "../RSDecoder/rslidar_packet.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "PointCloudManage.h"
#include "grid_manage.h"
#include "GetFrame_RS.h"
#include "Auxiliary.h"
#include "ctime"
#include <math.h>

#include "MutiLidar.h"

bool RECORD_TARGET=false;
extern string paramAdress;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef pcl::PointXYZI PointT_I;
typedef pcl::PointCloud<PointT_I> PointCloud_I;

typedef pcl::PointXYZRGB PointT_C;
typedef pcl::PointCloud<PointT_C> PointCloud_C;


namespace RslidarInput = robosense::rslidar_input;
namespace Rslidar = robosense::rslidar;

/*
    IniParam Param;
    string paramAdress = "../src/parameter.txt";
    if(!getParameter(paramAdress,Param)) {
        cout << "read paramter file error!" << endl;
        return 0;
    }
*/


int MutLidar(IniParam &Param,LidarMap_t &lidarMap_th,volatile bool &RunFlag,string pcapAdress) {

    volatile bool &AllSensorButton = RunFlag;
    int Frequence_32 = 10;
    int Frequence_16 = 20;

    ofstream outfile("target_size.txt");//默认为覆盖模式
    if(!outfile) cout<< "!!! error" << "file open failed!!!" << endl;

    bool offLineFlag = false;
    if(pcapAdress!="")  offLineFlag = true;

    ////////////////////////////////初始化RS_Liar32////////////////////////////////////
    robosense::rslidar::ST_Param param_RS32;
    initializeParam32(param_RS32); //修改参数时候直接修改此函数
    // initialize decoder_RS32
    robosense::rslidar::RSLidarDecoder<PointXYZITS> decoder_RS32(param_RS32);
    string device_ip32 = "192.168.1.200";

    string pcap_file_dir32 = pcapAdress;  //赋值则用于读取离线pcap数据
    //string pcap_file_dir32 = "/home/cyn/RSLIdar32/wireshark/guidhuman.pcap";
    //string pcap_file_dir32 = argc>1?argv[1]:"";
            //"/home/cyn/RSlidarTest/wireshark/target_walk_600.pcap";
    //if(argc==1) cout << "***WARRNING*** If it is offline,NO pacp file!"<< endl;

    uint16_t msop_port32 = 6699,difop_port32 = 7788;
    RslidarInput::Input InputObj_RS32(device_ip32,msop_port32,difop_port32,pcap_file_dir32);

    queue_ts<vector<PointCloud_I>,double,long long> PCIPtr_Q32(5);
    thread ListenRs32(listenRs,ref(decoder_RS32),ref(InputObj_RS32),&PCIPtr_Q32,32,
                      offLineFlag,Frequence_32,ref(AllSensorButton));
    //ListenRs32.detach();


    ////////////////////////////////初始化RS_Liar16////////////////////////////////////
    robosense::rslidar::ST_Param param_RS16;
    initializeParam16(param_RS16); //修改参数时候直接修改此函数
    // initialize decoder_RS16
    robosense::rslidar::RSLidarDecoder<PointXYZITS> decoder_RS16(param_RS16);
    string device_ip16 = "192.168.1.216";

    string pcap_file_dir16 = pcapAdress;
    //string pcap_file_dir16 = "/home/cyn/RSLIdar32/wireshark/guidhuman.pcap";
    //string pcap_file_dir16 = argc>1?argv[1]:"";
    //"/home/cyn/RSlidarTest/wireshark/target_walk_600.pcap";
    //if(argc==1) cout << "***WARRNING*** If it is offline,NO pacp file!"<< endl;

    uint16_t msop_port16 = 6616,difop_port16 = 7716;
    RslidarInput::Input InputObj_RS16(device_ip16,msop_port16,difop_port16,pcap_file_dir16);


    queue_ts<vector<PointCloud_I>,double,long long> PCIPtr_Q16(5);
    thread ListenRs16(listenRs,ref(decoder_RS16),ref(InputObj_RS16),&PCIPtr_Q16,32,
                      offLineFlag,Frequence_16,ref(AllSensorButton)); //16线ID号不连续
    //ListenRs16.detach();

    ///////////////////////////显示界面初始化////////////////////////////////

    PointCloud_I::Ptr PC_I_raw_RS16(new PointCloud_I),PC_I_raw_RS32(new PointCloud_I),PC_I_raw_RS(new PointCloud_I),
            PC_I_target(new PointCloud_I),PC_I_NoGuid(new PointCloud_I),//存储高反射率点群
            PC_I_Combine(new PointCloud_I),PC_I_UnGround(new PointCloud_I);
    PointCloud_C::Ptr PC_C_RsRaw(new PointCloud_C),PC_C_NoGuid(new PointCloud_C),
            PC_C_Rs32_Raw(new PointCloud_C),PC_C_Rs16_Raw(new PointCloud_C),
                    PC_C_Combine(new PointCloud_C),PC_C_UnGround(new PointCloud_C),PC_C_WithGround(new PointCloud_C);
    PointCloud::Ptr PC_temp(new PointCloud),pointcloud_2d(new PointCloud);


#ifdef PCLShow
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
          viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    vector<int> BoxAppendix;


    int v1 (0);
    viewer->createViewPort(0,0.5,0.5,1.0,v1);
    viewer->addPointCloud(PC_C_Rs32_Raw,"RsRaw32",v1);
    viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"RsRaw32");
    viewer->addCoordinateSystem(1.0);//显示坐标轴

    int v2 (1);
    viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v2);
    viewer->addPointCloud(PC_C_Rs16_Raw, "RsRaw16", v2);
    viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"RsRaw16");
    viewer->addCoordinateSystem(1.0);//显示坐标轴

    int v3 (2);
    viewer->createViewPort(0, 0, 0.5, 0.5, v3);
    viewer->addPointCloud(PC_C_Combine, "PC_I_Combine", v3);
    viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"PC_I_Combine");
    viewer->addCoordinateSystem(1.0);//显示坐标轴

    int v4 (3);
    viewer->createViewPort(0.5, 0, 1.0, 0.5, v4);
    viewer->addPointCloud(PC_C_NoGuid, "PC_I_UnGround", v4);
    viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"PC_I_UnGround");
    viewer->addCoordinateSystem(1.0);//显示坐标轴

    /*
    //设置只有骨架
    viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
          pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,"plane1");
    */

#endif

    //////////////////////////////主程序运行参数初始化//////////////////////////////////

    GuidObject guidObject,lastGuidObj;
    double time_stt_us,last_time_stt_us = 0;


    /////////////////////////////点云变换参数（应该由标定得到）/////////////////////////////////////
    float s35 = 0.5735764;
    float c35 = 0.8191520;
    float B0InA_X = -0.02376608f;//米
    float B0InA_Y = 0;
    float B0InA_Z = 0.17115920f;
    Eigen::Matrix3f Rba = Eigen::Matrix3f::Identity();

    Rba <<  0,  -c35,   s35,
            1,   0,     0,
            0,  s35,  c35;

    Eigen::Isometry3f Tba = Eigen::Isometry3f::Identity();
    Tba.rotate(Rba);
    Tba.pretranslate(Eigen::Vector3f(B0InA_X,B0InA_Y,B0InA_Z));
    ///////////////////////////////////////////////////////////////////////

    double time_stt_us16;
    long long arriveTime32,arriveTime16;

    vector<PointCloud_I> PC_I_raw32_scanID(32),PC_I_raw16_scanID(32);

    usleep(0.3*1000*1000);//等待雷达缓冲区获取数据
    while(RunFlag && getLatestFrame(PC_I_raw32_scanID,PCIPtr_Q32,time_stt_us,arriveTime32)
        && getLatestFrame(PC_I_raw16_scanID,PCIPtr_Q16,time_stt_us16,arriveTime16)
        ) {

        PC_C_RsRaw->clear();
        pointcloud_2d->clear();
        PC_I_NoGuid->clear();
        PC_C_NoGuid->clear();
        PC_I_raw_RS32->clear();
        PC_C_Rs32_Raw->clear();
        PC_I_raw_RS->clear();
        PC_I_raw_RS16->clear();
        PC_C_Rs16_Raw->clear();

        PC_I_Combine->clear();
        PC_C_Combine->clear();

        PC_I_UnGround->clear();
        PC_C_UnGround->clear();

        PC_C_WithGround->clear();

        /*
        cout << "IMUdata32.yaw: " << IMUdata32.yaw << "  IMUdata16.yaw: " << IMUdata16.yaw << endl;
        cout << "IMU下的时间差： "<< IMUdata16.timeStamp_ms - IMUdata32.timeStamp_ms << " ms"<< endl;
         */

        ////////////////////////////////时间测算模块/////////////////////////////////
        int time_interval_ms;
        if(last_time_stt_us!=0){
            time_interval_ms = int((time_stt_us-last_time_stt_us)/1000);
            if(!time_interval_ms) {
                usleep(15*1000);
                continue;
            }
            cout << "两帧时间间隔：　" << time_interval_ms << "ms" << endl;
        }
        last_time_stt_us = time_stt_us ;

        cout << "两雷达时差：" << (arriveTime16 - arriveTime32)/1000 << " ms" << endl;
        ////////////////////////////////时间测算模块结束/////////////////////////////////

        /*
         void feature_odo(vector<PointCloud_I>& PC_I_raw32_scanID,float sor_vox_LeafSize,IMUState& IMUdata32,
                 PointCloud_C::Ptr PC_C_raw_RS,PointCloud_C::Ptr PC_C_whole,
                 PointCloud_C::Ptr cornerPointsSharp_RGB,PointCloud_C::Ptr cornerPointsFlat_RGB)
        */

        for(auto &PC_I:PC_I_raw32_scanID){
            *PC_I_raw_RS += PC_I;
            PC_I.clear();
        }
        if(PC_I_raw_RS->size()<500){
            cout << "镜头遮挡，无法计算" << endl;
            continue;
        }
        *PC_I_raw_RS32 = *PC_I_raw_RS;


        for(auto &PC_I:PC_I_raw16_scanID){

            *PC_I_raw_RS16 += PC_I;
            PC_I.clear();
        }
        if(PC_I_raw_RS16->size()<500){
            cout << "镜头遮挡，无法计算" << endl;
            continue;
        }



        ////////////////////////////////点云转换拼接/////////////////////////////////
        convetPC(PC_I_raw_RS16,Tba);
        *PC_I_Combine = *PC_I_raw_RS + *PC_I_raw_RS16;


        //提取地面点
        bool precise = false;
        static bool preciseOld = true;

        PointCloud::Ptr pointcloud(new PointCloud);
        PointCloud_I::Ptr PC_I_ground(new PointCloud_I);
        cutRawGround(PC_I_raw_RS16,PC_I_ground,Param.sor_vox_LeafSize*1.6f,Param.cutAngleYaw,
                     preciseOld?Param.cutAnglePitch:int(Param.cutAnglePitch*1.3f));
        /////////////////显示切割地面点//////////////////
        /*
            PointCloud_C::Ptr PC_C_temp(new PointCloud_C);
            XYZI2XYZRGB(PC_I_ground,PC_C_temp);

            viewer->removeShape("PC_C_temp",v4);
            viewer->addPointCloud(PC_C_temp,"PC_C_temp",v4);
            viewer->updatePointCloud(PC_C_temp,"PC_C_temp");
            viewer->spinOnce();
        */
        //////////////////////////////////////
        if(PC_I_ground->size()<50) continue;
        XYZI2XYZ(PC_I_ground,pointcloud);
        //对地面参数进行估计
        pcl::ModelCoefficients PlaneCoeff2Show;

        groundEstimate(Param.groundEstimateNum,pointcloud,PlaneCoeff2Show,
                       Param.outPlaneDistance,Param.maxDeviaAngle_deg,precise);
        preciseOld = precise;
        //显示估计平面

#ifdef PCLShow
        viewer->removeShape("PlaneMatching",v3);
        viewer->addPlane(PlaneCoeff2Show,"PlaneMatching",v3);
        //设置只有骨架
        viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                "PlaneMatching",v3);
#endif


        //带地面的点云显示
        VoxFilter(PC_I_Combine,Param.sor_vox_LeafSize);
        XYZI2XYZRGB(PC_I_Combine,PC_C_WithGround);


        //除去地面点
        float aboveGround_deg = Param.aboveGround;
        if(!precise) aboveGround_deg = Param.aboveGround_NoPrecise;

        GroundFilter(PC_I_Combine,PlaneCoeff2Show,precise?Param.outPlaneDistance:Param.outPlaneDistance*1.6f,
                     aboveGround_deg);

        //直通滤波,限制点云的xyz范围
        ThroughFilter(PC_I_Combine,Param.max_x,Param.min_x, Param.max_y,
                      Param.min_y, Param.max_z, Param.min_z);


        //cout<< "Get a Frame" << endl;
        XYZI2XYZRGB(PC_I_Combine,PC_C_Combine);

        VoxFilter(PC_I_raw_RS16,Param.sor_vox_LeafSize);
        XYZI2XYZRGB(PC_I_raw_RS16,PC_C_Rs16_Raw);

        VoxFilter(PC_I_raw_RS32,Param.sor_vox_LeafSize);
        XYZI2XYZRGB(PC_I_raw_RS32,PC_C_Rs32_Raw);


        //由原始点云通过反射率滤波和标准形状滤波获得高反射率点云
        GetHighItsityGroup(PC_I_Combine, PC_I_target, Param.ThresholdIntensity,
                           Param.ErrrorPointSearchRadius, Param.ErrrorPointNearNum);

        //框选显示每个高反射率点
#ifdef PCLShow
        BoxPoints(PC_I_target, BoxAppendix, viewer, 0.02, v1);
#endif

        lastGuidObj = guidObject;

#ifdef PCLShow
        viewer->removeShape("target_sphere",v1);
#endif



        bool targetFlag = false;
        //分析高反射率点云是否是引导对象
        if(targetPosiAnaly(PC_I_target, guidObject, Param.StdTargetSize,
                                 Param.sor_vox_LeafSize, time_stt_us,outfile)) {


            PointT aboveGuidLogo(guidObject.posi.x, guidObject.posi.y, guidObject.posi.z+0.5f);

#ifdef PCLShow
            viewer->addSphere(aboveGuidLogo,0.25,255,0,0,"target_sphere",v1);
#endif

            //计算速度（目标相对激光雷达坐标系的速度）
            targetVeloAnaly(guidObject,lastGuidObj);

            //删除引导人员周围点云，防止污染地图
            RemoveGuid(guidObject,PC_I_Combine,PC_I_NoGuid);

            targetFlag = true;
        }
        else *PC_I_NoGuid = *PC_I_Combine;


        //////////////////////////////////栅格地图转换程序//////////////////////////////////
        XYZI2XYZRGB(PC_I_NoGuid, PC_C_NoGuid);
        XYZI2XYZ(PC_I_NoGuid, PC_temp);

        //平面投影滤波
        cast_filter_Ground(PC_temp,PlaneCoeff2Show, pointcloud_2d);

        //调用栅格转换算法成图
        GridMap gridmap(Param.gridmapNum_x,Param.gridmapNum_y);
        Pcl2DtoGridMap(pointcloud_2d,gridmap, Param.max_x,Param.min_x,
                       Param.max_y,Param.min_y,Param.gridScale);//注意两个范围都是单边


#ifdef OpenCVShow
        Mat gridMat = GridShow(gridmap,Param.PixelPerGrid,"grid",0);
#endif

        //输出生成的地图等信息
        if(targetFlag){
            LidarMap lidarmap;
            lidarmap.start_column = gridmap.LaserNumX();
            lidarmap.start_row = gridmap.LaserNumY();
            lidarmap.map = gridmap.GetGridVector();
            RealCordToGridCord(guidObject.posi.x, guidObject.posi.y,
                               (float)Param.gridScale, gridmap,
                               lidarmap.target_column,lidarmap.target_row);
            lidarMap_th.push(lidarmap);

#ifdef OpenCVShow
            PointShow(0,lidarmap.start_column,lidarmap.start_row,Param.PixelPerGrid,gridMat,"grid",false);
            PointShow(1,lidarmap.target_column,lidarmap.target_row,Param.PixelPerGrid,gridMat,"grid",false);
#endif
        }

#ifdef PCLShow
        viewer->updatePointCloud(PC_C_Rs32_Raw,"RsRaw32");
        viewer->updatePointCloud(PC_C_Rs16_Raw,"RsRaw16");
        viewer->updatePointCloud(PC_C_WithGround,"PC_I_Combine");
        viewer->updatePointCloud(PC_C_NoGuid,"PC_I_UnGround");
        viewer->spinOnce();
#endif


        cout << "*************************************************"<<endl << endl;
    }

    ListenRs16.join();
    ListenRs32.join();

  return 0;
}


void LidarMapThreadFun(volatile bool &RunFlag,LidarMap_t& lidarMap_t,string pcapAdress){
    while(RunFlag){

        IniParam Param;
        if(!getParameter(paramAdress,Param)) {
            cout << "read paramter file error!" << endl;
        }

        thread MutilidarThread(MutLidar,ref(Param),ref(lidarMap_t),ref(RunFlag),pcapAdress);
        MutilidarThread.join();

        //cout << "雷达线程意外退出，正重启……" << endl;

    }
}


