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
#include "GetFrame_RS.h"
#include "Auxiliary.h"
#include "Auxiliary_IMU.h"
#include "imu_get_data.h"
#include "ctime"
#include <math.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef pcl::PointXYZI PointT_I;
typedef pcl::PointCloud<PointT_I> PointCloud_I;

typedef pcl::PointXYZRGB PointT_C;
typedef pcl::PointCloud<PointT_C> PointCloud_C;


namespace RslidarInput = robosense::rslidar_input;
namespace Rslidar = robosense::rslidar;


int main(int argc, char** argv) {

    IniParam Param;
    string paramAdress = "../src/parameter.txt";
    if(!getParameter(paramAdress,Param)) {
        cout << "read paramter file error!" << endl;
        return 0;
    }

    string ImuRecordAdress = "../Record/ImuRecord.txt";
    //string ImuRecordAdress = ""; //没有地址表示不录IMU数据


    ///////////////////////////初始化IMU/////////////////////////////////////
    CallbackHandler callback;
    initIMU(callback);
    queue_ts_IMU<IMUState,long long> IMUdata_Q(200,ImuRecordAdress);

    thread ListenIMUdata(listenIMU,ref(callback), &IMUdata_Q);
    //ListenIMUdata.detach();

    ////////////////////////////////初始化RS_Liar32////////////////////////////////////
    robosense::rslidar::ST_Param param_RS32;
    initializeParam32(param_RS32); //修改参数时候直接修改此函数
    // initialize decoder_RS32
    robosense::rslidar::RSLidarDecoder<PointXYZITS> decoder_RS32(param_RS32);
    string device_ip32 = "192.168.1.200";

    //string pcap_file_dir32 = "";  //赋值则用于读取离线pcap数据
    //string pcap_file_dir32 = "/home/cyn/RSLIdar32/wireshark/guidhuman.pcap";
    string pcap_file_dir32 = argc>1?argv[1]:"";
            //"/home/cyn/RSlidarTest/wireshark/target_walk_600.pcap";
    if(argc==1) cout << "***WARRNING*** If it is offline,NO pacp file!"<< endl;

    uint16_t msop_port32 = 6699,difop_port32 = 7788;
    RslidarInput::Input InputObj_RS32(device_ip32,msop_port32,difop_port32,pcap_file_dir32);

    queue_ts<vector<PointCloud_I>,double,long long,IMUState> PCIPtr_Q32(5);
    thread ListenRs32(listenRs,ref(decoder_RS32),ref(InputObj_RS32),&PCIPtr_Q32,32,ref(IMUdata_Q) );
    //ListenRs32.detach();

    ////////////////////////////////初始化RS_Liar16////////////////////////////////////
    robosense::rslidar::ST_Param param_RS16;
    initializeParam16(param_RS16); //修改参数时候直接修改此函数
    // initialize decoder_RS16
    robosense::rslidar::RSLidarDecoder<PointXYZITS> decoder_RS16(param_RS16);
    string device_ip16 = "192.168.1.216";

    //string pcap_file_dir16 = "";  //赋值则用于读取离线pcap数据
    //string pcap_file_dir16 = "/home/cyn/RSLIdar32/wireshark/guidhuman.pcap";
    string pcap_file_dir16 = argc>1?argv[1]:"";
    //"/home/cyn/RSlidarTest/wireshark/target_walk_600.pcap";
    if(argc==1) cout << "***WARRNING*** If it is offline,NO pacp file!"<< endl;

    uint16_t msop_port16 = 6616,difop_port16 = 7716;
    RslidarInput::Input InputObj_RS16(device_ip16,msop_port16,difop_port16,pcap_file_dir16);


    queue_ts<vector<PointCloud_I>,double,long long,IMUState> PCIPtr_Q16(5);
    thread ListenRs16(listenRs,ref(decoder_RS16),ref(InputObj_RS16),&PCIPtr_Q16,36,ref(IMUdata_Q)); //32线ID号不连续
    //ListenRs16.detach();

    ///////////////////////////显示界面初始化////////////////////////////////

    PointCloud_I::Ptr PC_I_raw_RS16(new PointCloud_I),PC_I_raw_RS32(new PointCloud_I),
            PC_I_Combine(new PointCloud_I);
    PointCloud_C::Ptr PC_C_raw_RS16(new PointCloud_C),PC_C_raw_RS32(new PointCloud_C),
            PC_C_Combine(new PointCloud_C);

    boost::shared_ptr<pcl::visualization::PCLVisualizer>
          viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    vector<int> BoxAppendix;

    int v1 (0);
    viewer->createViewPort(0,0.5,0.5,1.0,v1);
    viewer->addPointCloud(PC_C_raw_RS16,"RsRaw16",v1);
    viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"RsRaw16");
    viewer->addCoordinateSystem(1.0);//显示坐标轴

    int v2 (1);
    viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v2);
    viewer->addPointCloud(PC_C_raw_RS32, "RsRaw32", v2);
    viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"RsRaw32");
    viewer->addCoordinateSystem(1.0);//显示坐标轴

    int v3 (2);
    viewer->createViewPort(0, 0, 0.5, 0.5, v3);
    viewer->addPointCloud(PC_C_Combine, "PC_I_Combine", v3);
    viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"PC_I_Combine");
    viewer->addCoordinateSystem(1.0);//显示坐标轴


    /*
    int v4 (3);
    viewer->createViewPort(0.5, 0, 1.0, 0.5, v4);
    viewer->addPointCloud(PC_C_whole, "PC_C_whole", v4);
    viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"PC_C_whole");
    viewer->addCoordinateSystem(1.0);//显示坐标轴
     */

    /*
    //设置只有骨架
    viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
          pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,"plane1");
    */

    //////////////////////////////主程序运行参数初始化//////////////////////////////////
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

    IMUState IMUdata32,IMUdata32_last,IMUdata16;

    while(getLatestFrame(PC_I_raw32_scanID,PCIPtr_Q32,time_stt_us,arriveTime32,IMUdata32)
        && getLatestFrame(PC_I_raw16_scanID,PCIPtr_Q16,time_stt_us16,arriveTime16,IMUdata16)
        ) {

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



        vector<PointCloud_I> PC_I_raw32_scanID_temp(PC_I_raw32_scanID);


        for(auto &PC_I:PC_I_raw32_scanID){
            *PC_I_raw_RS32 += PC_I;
            PC_I.clear();
        }
        if(PC_I_raw_RS32->size()<500){
            cout << "镜头遮挡，无法计算" << endl;
            continue;
        }


        for(auto &PC_I:PC_I_raw16_scanID){
            *PC_I_raw_RS16 += PC_I;
            PC_I.clear();
        }
        if(PC_I_raw_RS16->size()<500){
            cout << "镜头遮挡，无法计算" << endl;
            continue;
        }

        ////////////////////////////////点云拼接/////////////////////////////////
        convetPC(PC_I_raw_RS16,Tba);


        //直通滤波,限制点云的xyz范围
        ThroughFilter(PC_I_raw_RS16,Param.max_x,Param.min_x, Param.max_y,
                      Param.min_y, Param.max_z, Param.min_z);
        ThroughFilter(PC_I_raw_RS32,Param.max_x,Param.min_x, Param.max_y,
                      Param.min_y, Param.max_z, Param.min_z);


        *PC_I_Combine = *PC_I_raw_RS32 + *PC_I_raw_RS16;


        //cout<< "Get a Frame" << endl;
        VoxFilter(PC_I_raw_RS16,Param.sor_vox_LeafSize);
        XYZI2XYZRGB(PC_I_raw_RS16,PC_C_raw_RS16);

        VoxFilter(PC_I_raw_RS32,Param.sor_vox_LeafSize);
        XYZI2XYZRGB(PC_I_raw_RS32,PC_C_raw_RS32);

        VoxFilter(PC_I_Combine,Param.sor_vox_LeafSize);
        XYZI2XYZRGB(PC_I_Combine,PC_C_Combine);


        viewer->updatePointCloud(PC_C_raw_RS16,"RsRaw16");
        viewer->updatePointCloud(PC_C_raw_RS32, "RsRaw32");
        viewer->updatePointCloud(PC_C_Combine, "PC_I_Combine");
        //viewer->updatePointCloud(PC_C_whole,"PC_C_whole");
        viewer->spinOnce();



        PC_I_raw_RS16->clear();
        PC_I_raw_RS32->clear();
        PC_I_Combine->clear();
        PC_C_raw_RS16->clear();
        PC_C_raw_RS32->clear();
        PC_C_Combine->clear();

        IMUState TempIMUReceiver;
        long long TempPCTime;
        IMUdata_Q.latestElem(&TempIMUReceiver,&TempPCTime);

        if(TempIMUReceiver.velocity[0] == 0){
            cout << "[IMU State] No Velocity data, No GPS Probably!" << endl;
        };


        cout << "*************************************************"<<endl << endl;
    }

    ListenIMUdata.join();
    ListenRs16.join();
    ListenRs32.join();




  return 0;
}


