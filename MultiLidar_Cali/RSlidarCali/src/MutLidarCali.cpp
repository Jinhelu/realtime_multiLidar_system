#include <atomic>
#include <iostream>
#include <cstring>
#include <vector>
#include <iostream>
#include <fstream>
using namespace std;
#include "../RSDecoder/input.h"
#include "../RSDecoder/rslidar_decoder.hpp"
#include "../RSDecoder/rslidar_packet.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "PointCloudManage.h"
#include "GetFrame_RS.h"
#include "Auxiliary.h"
#include "Auxiliary_IMU.h"
#include "imu_get_data.h"
#include "ctime"
#include "cali.h"
#include <math.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef pcl::PointXYZI PointT_I;
typedef pcl::PointCloud<PointT_I> PointCloud_I;

typedef pcl::PointXYZRGB PointT_C;
typedef pcl::PointCloud<PointT_C> PointCloud_C;


namespace RslidarInput = robosense::rslidar_input;
namespace Rslidar = robosense::rslidar;


bool RECORD_TARGET=false;

volatile bool ReadLidar32 = true;//控制读取进度，保证两雷达时间同步
volatile bool ReadLidar16 = true;//控制读取进度，保证两雷达时间同步

int Frequence32 = 5;
int Frequence16 = 5;


int main(int argc, char** argv) {

    ofstream outfileCali("cali.txt");//默认为覆盖模式
    if(!outfileCali) cout<< "!!! error" << "file open failed!!!" << endl;

    IniParam Param;
    string paramAdress = "../src/parameter.txt";
    if(!getParameter(paramAdress,Param)) {
        cout << "read paramter file error!" << endl;
        return 0;
    }
    ///////////////////////////初始化IMU/////////////////////////////////////

    string imuRecord_file = argc>2?argv[2]:"";//imu录数据的路径
    queue_ts_IMU IMUdata_Q(200,imuRecord_file);
    CallbackHandler callback;
    initIMU(callback);
    thread ListenIMUdata;
    if(imuRecord_file == "") {
        ListenIMUdata = thread(listenIMU,ref(callback), &IMUdata_Q);
        ListenIMUdata.detach();
    }

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
    thread ListenRs32(listenRs_32,ref(decoder_RS32),ref(InputObj_RS32),&PCIPtr_Q32,32,ref(IMUdata_Q) );
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
    thread ListenRs16(listenRs_16,ref(decoder_RS16),ref(InputObj_RS16),&PCIPtr_Q16,36,ref(IMUdata_Q)); //16线ID号不连续
    //ListenRs16.detach();


    ////////////////////////////////显示界面初始化////////////////////////////////

    PointCloud_I::Ptr PC_I_raw_RS16(new PointCloud_I),PC_I_raw_RS32(new PointCloud_I),
            PC_I_combine(new PointCloud_I),PC_I_Combine_final(new PointCloud_I);//存储高反射率点群
    PointCloud_C::Ptr PC_C_raw_RS16(new PointCloud_C),PC_C_raw_RS32(new PointCloud_C),
            PC_C_combine(new PointCloud_C),PC_C_Combine_final(new PointCloud_C);

    boost::shared_ptr<pcl::visualization::PCLVisualizer>
          viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    vector<int> BoxAppendix;

    int v1 (0);
    viewer->createViewPort(0,0.5,0.5,1.0,v1);
    viewer->addPointCloud(PC_C_raw_RS16,"PC_C_raw_RS16",v1);
    viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"PC_C_raw_RS16");
    viewer->addCoordinateSystem(1.0);//显示坐标轴

    int v2 (1);
    viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v2);
    viewer->addPointCloud(PC_C_raw_RS32, "PC_C_raw_Rs32", v2);
    viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"PC_C_raw_Rs32");
    viewer->addCoordinateSystem(1.0);//显示坐标轴

    int v3 (2);
    viewer->createViewPort(0, 0, 0.5, 0.5, v3);
    viewer->addPointCloud(PC_C_combine, "PC_C_combine", v3);
    viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"PC_C_combine");
    viewer->addCoordinateSystem(1.0);//显示坐标轴

    int v4 (3);
    viewer->createViewPort(0.5, 0, 1.0, 0.5, v4);
    viewer->addPointCloud(PC_C_Combine_final, "PC_C_Combine_final", v4);
    viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"PC_C_Combine_final");
    viewer->addCoordinateSystem(1.0);//显示坐标轴

    /*
    //设置只有骨架
    viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
          pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,"plane1");
    */


    //////////////////////////////主程序运行参数初始化//////////////////////////////////
    double time_stt_us16,last_time_stt_us16 = 0;
    double time_stt_us32,last_time_stt_us32 = 0;

    long long arriveTime32,arriveTime16,LastArriveTime32=0,LastArriveTime16=0;

    vector<PointCloud_I> PC_I_raw32_scanID(32),PC_I_raw16_scanID(36);

    IMUState IMUdata32,IMUdata32_last,IMUdata16;
    int MinTimeInterval = int(1.0f/max(Frequence32,Frequence16)*1000 + 3);//单位ms


    while(true) {

        bool continueFlag = getLatestFrame(PC_I_raw32_scanID,PCIPtr_Q32,time_stt_us32,arriveTime32,IMUdata32)
        && getLatestFrame(PC_I_raw16_scanID,PCIPtr_Q16,time_stt_us16,arriveTime16,IMUdata16);

        if(!continueFlag) {
            cout << "No Frame, waiting next frame..." << endl;
            usleep(25*1000);
            continue;
        }

        /*
        cout << "IMUdata32.yaw: " << IMUdata32.yaw << "  IMUdata16.yaw: " << IMUdata16.yaw << endl;
        cout << "IMU下的时间差： "<< IMUdata16.timeStamp_ms - IMUdata32.timeStamp_ms << " ms"<< endl;
         */

        ////////////////////////////////时间测算模块/////////////////////////////////


        int time_interval_ms16,time_interval_ms32;
        if(last_time_stt_us16!=0 && last_time_stt_us32!=0){
            time_interval_ms16 = int((time_stt_us16-last_time_stt_us16)/1000);
            time_interval_ms32 = int((time_stt_us32-last_time_stt_us32)/1000);
            if(!time_interval_ms16 && !time_interval_ms32) {
                //cout << "No New Liar32 Frame, waiting next Frame..." << endl;
                usleep(25*1000);
                continue;
            }
        }
        last_time_stt_us16 = time_stt_us16;
        last_time_stt_us32 = time_stt_us32;

        ///////////////////////////////两帧时间对齐模块
        long long twoLidarDeltaTime = (arriveTime16 - arriveTime32)/1000;
        //cout << "Lidar16 arriveTime:" << arriveTime16 << endl << "Lidar32 arriveTime:" << arriveTime32 << endl;

        if(IMUdata_Q.ifOffline()){
            //相差50ms是正常；
            if(twoLidarDeltaTime>MinTimeInterval){
                ReadLidar16 = false;//16线雷达等待
                ReadLidar32 = true;
                //cout << "main: " << "  Lidar16 waiting,32 go." <<endl;
                //cout << "*************************************************"<<endl << endl;
                usleep(1.0f/Frequence32*1000*1000/2);//主线程等待32线刷新
                continue;
            }
            else if(twoLidarDeltaTime<(-MinTimeInterval) ){
                ReadLidar32 = false;//32线雷达等待
                ReadLidar16 = true;
                //cout << "main: " << "  Lidar32 waiting,16 go." <<endl;
                //cout << "*************************************************"<<endl << endl;
                usleep(1.0f/Frequence16*1000*1000/2);//主线程等待16线刷新
                continue;
            }
            else{
                ReadLidar32 = true;
                ReadLidar16 = true;
            }
        }
        if((arriveTime32-LastArriveTime32)/1000 < 5) continue;

        /*
        cout << "PC ABS Time(Lidar32): " << arriveTime32 << endl;
        if(LastArriveTime16 && LastArriveTime32)
            cout << "两帧时间间隔：　" << "Lidar16: " << (arriveTime16-LastArriveTime16)/1000 << "ms"
                 << "  Lidar32: " << (arriveTime32-LastArriveTime32)/1000  << "ms" << endl;
        cout << "两雷达时差：" << twoLidarDeltaTime << " ms" << endl;
         */

        LastArriveTime32 = arriveTime32;
        LastArriveTime16 = arriveTime16;
        ////////////////////////////////时间测算模块结束/////////////////////////////////


        cout << "*************************************************"<<endl << endl;
        PC_I_raw_RS16->clear();
        PC_I_raw_RS32->clear();
        PC_I_combine->clear();
        PC_I_Combine_final->clear();
        PC_C_raw_RS16->clear();
        PC_C_raw_RS32->clear();
        PC_C_combine->clear();
        PC_C_Combine_final->clear();



        for(int i=0;i<32;i++){
            *PC_I_raw_RS32 += PC_I_raw32_scanID[i];
        }
        if(PC_I_raw_RS32->size()<500){
            cout << "Lidar32： 镜头遮挡，无法计算" << endl;
            continue;
        }


        for(int i=0;i<32;i++){
            *PC_I_raw_RS16 += PC_I_raw16_scanID[i];
        }
        if(PC_I_raw_RS16->size()<500){
            cout << "Lidar16： 镜头遮挡，无法计算" << endl;
            continue;
        }



        /*************************************32线平面估计***************************************/

        //前处理，切割、滤波
        float doubleEdge = 1.50;
        //cutCloud(PC_I_raw_RS32,-85,0, 300,360);
        //cutCloud(PC_I_raw_RS32,-85,0,300,60);
        ThroughFilter(PC_I_raw_RS32,1.7,0.52, doubleEdge,
                      -doubleEdge, 5, -5);
        VoxFilter(PC_I_raw_RS32,Param.sor_vox_LeafSize*0.5f);

        PointCloud_I::Ptr PC_I_RS32(new PointCloud_I);
        *PC_I_RS32 = *PC_I_raw_RS32;

        XYZI2XYZRGB(PC_I_raw_RS32,PC_C_raw_RS32);
        viewer->updatePointCloud(PC_C_raw_RS32,"PC_C_raw_Rs32");
        viewer->spinOnce();


        int EstimateNum = 40;
        float oulineDis = 0.015;
        float deviaAngle_deg = 1.2;
        PointCloud::Ptr PC_raw_RS32(new PointCloud);


        //墙面估计
        XYZI2XYZ(PC_I_raw_RS32,PC_raw_RS32);


        pcl::ModelCoefficients PlaneCoeffWall;
        bool precise_wall=false;

        if(!wallEstimate_cali(EstimateNum,PC_raw_RS32,PlaneCoeffWall,oulineDis,
                              deviaAngle_deg,precise_wall,outfileCali)){
            continue;
        }
        PlaneFilter(PC_I_raw_RS32,PlaneCoeffWall,oulineDis*3);
        //显示估计平面
        viewer->removeShape("PlaneMatchingWall",v2);
        viewer->addPlane(PlaneCoeffWall,"PlaneMatchingWall",v2);
        //设置只有骨架
        viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                "PlaneMatchingWall",v2);

        float a3 = PlaneCoeffWall.values[0];
        float b3 = PlaneCoeffWall.values[1];
        float c3 = PlaneCoeffWall.values[2];
        float d3 = PlaneCoeffWall.values[3];
        outfileCali << " d3: " << d3 ;

        //地面估计
        XYZI2XYZ(PC_I_raw_RS32,PC_raw_RS32);
        pcl::ModelCoefficients PlaneCoeffGround;
        bool precise_ground=false;
        if(!groundEstimate_cali(EstimateNum,PC_raw_RS32,PlaneCoeffGround,oulineDis,
                                deviaAngle_deg,precise_ground,outfileCali)){
            continue;
        }
        PlaneFilter(PC_I_raw_RS32,PlaneCoeffGround,oulineDis*3);
        //显示估计平面
        viewer->removeShape("PlaneMatchingGround",v2);
        viewer->addPlane(PlaneCoeffGround,"PlaneMatchingGround",v2);
        //设置只有骨架
        viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                "PlaneMatchingGround",v2);

        float a4 = PlaneCoeffGround.values[0];
        float b4 = PlaneCoeffGround.values[1];
        float c4 = PlaneCoeffGround.values[2];
        float d4 = PlaneCoeffGround.values[3];
        outfileCali << " d4: " << d4 ;


        /*************************************16线平面估计***************************************/
        //分割，滤波，显示
        cutCloud(PC_I_raw_RS16,-80,80, 35,145); //角度要从小到大写，　后两个在0-360之间
        ThroughFilter(PC_I_raw_RS16,doubleEdge,-doubleEdge,0,
                      -5, 5, -5);
        VoxFilter(PC_I_raw_RS16,Param.sor_vox_LeafSize*0.5f);

        PointCloud_I::Ptr PC_I_RS16(new PointCloud_I),PC_I_RS16_final(new PointCloud_I);
        *PC_I_RS16 = *PC_I_raw_RS16;
        *PC_I_RS16_final = *PC_I_raw_RS16;

        XYZI2XYZRGB(PC_I_raw_RS16,PC_C_raw_RS16);
        viewer->updatePointCloud(PC_C_raw_RS16,"PC_C_raw_RS16");
        viewer->spinOnce();



        //墙面估计
        PointCloud::Ptr PC_raw_RS16(new PointCloud);
        XYZI2XYZ(PC_I_raw_RS16,PC_raw_RS16);
        bool preciseWall16 = false;

        pcl::ModelCoefficients PlaneCoeffWall16;
        if(!plane_cali16(EstimateNum,PC_raw_RS16,
                     PlaneCoeffWall16,oulineDis,0,true,deviaAngle_deg,preciseWall16))
            continue;

        PlaneFilter(PC_I_raw_RS16,PlaneCoeffWall16,oulineDis*3);
        //显示估计平面
        viewer->removeShape("PlaneMatchingWall16",v1);
        viewer->addPlane(PlaneCoeffWall16,"PlaneMatchingWall16",v1);
        //设置只有骨架
        viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                "PlaneMatchingWall16",v1);

        float a2 = PlaneCoeffWall16.values[0];
        float b2 = PlaneCoeffWall16.values[1];
        float c2 = PlaneCoeffWall16.values[2];
        float d2 = PlaneCoeffWall16.values[3];


        //地面估计
        XYZI2XYZ(PC_I_raw_RS16,PC_raw_RS16);
        bool preciseGround16 = false;

        pcl::ModelCoefficients PlaneCoeffGround16;
        if(!plane_cali16(EstimateNum,PC_raw_RS16,
                     PlaneCoeffGround16,oulineDis,5,false,deviaAngle_deg,preciseGround16))
            continue;

        PlaneFilter(PC_I_raw_RS16,PlaneCoeffGround16,oulineDis*3);
        //显示估计平面
        viewer->removeShape("PlaneMatchingGround16",v1);
        viewer->addPlane(PlaneCoeffGround16,"PlaneMatchingGround16",v1);
        //设置只有骨架
        viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                "PlaneMatchingGround16",v1);

        float a1 = PlaneCoeffGround16.values[0];
        float b1 = PlaneCoeffGround16.values[1];
        float c1 = PlaneCoeffGround16.values[2];
        float d1 = PlaneCoeffGround16.values[3];

        //cout << "a1: " << a1 << endl;

				/*************************************雷达与机架参数计算***************************************/
				float d7=0.766, d8=-1.533 ;
				float gamma_0 = atanf(b4/c4);
				float beta_0 = -atanf(a4*sinf(gamma_0)/b4);
				float delta_Z0 = -(d4*sinf(beta_0)/a4)-d7;
				float alpha_0 = atanf(b3/cosf(gamma_0)*(sinf(beta_0)*sinf(gamma_0)/b3-cosf(beta_0)/a3));
				float delta_X0 = d3*cosf(alpha_0)*cosf(beta_0)/a3-d8;

				float gamma_0_deg = gamma_0/3.1415926f*180;
				float beta_0_deg = beta_0/3.1415926f*180;
				float alpha_0_deg = alpha_0/3.1415926f*180;


				cout << " gamma_0_deg :" << gamma_0_deg << " beta_0_deg: " << beta_0_deg << " alpha_0_deg: " << alpha_0_deg
						 << " delta_X0: " << delta_X0 << " delta_Z0: " << delta_Z0 ;

				outfileCali << " gamma_0_deg: " << gamma_0_deg << " beta_0_deg: " << beta_0_deg << " alpha_0_deg: " << alpha_0_deg
										<< " delta_X0: " << delta_X0 << " delta_Z0: " << delta_Z0 ;


        /*************************************雷达间参数计算***************************************/
        float alpha = atanf(b1/c1);
        float alpha_deg = alpha/3.1415926f*180;
        float sin_alpha = sin(alpha);
        float cos_alpha = cos(alpha);

        float deltaZ = d1*cos_alpha/c1-d4;

        float beta = atan(c2/(sin_alpha*a2));
        if(beta<0) beta+=3.1415926f;
        float beta_deg = beta/3.1415926f*180;
        float sin_beta = sin(beta);
        float cos_beta = cos(beta);

        float deltaX = d3+(d2*cos_alpha*sin_beta/b2);


        cout << " beta_deg :" << beta_deg << " alpha_deg: " << alpha_deg
             << " deltaX: " << deltaX << " deltaZ: " << deltaZ << endl;

        outfileCali << " beta_deg: " << beta_deg << " alpha_deg: " << alpha_deg
                    << " deltaX: " << deltaX << " deltaZ: " << deltaZ ;



        ///////////////////////////////点云变换参数（由标定得到）////////////////////////////////////

        Eigen::Matrix3f Rab = Eigen::Matrix3f::Identity();


        Rab <<  cos_beta,  -sin_beta*cos_alpha,   sin_beta*sin_alpha,
                sin_beta,   cos_beta*cos_alpha,   -cos_beta*sin_alpha,
                0,          sin_alpha,            cos_alpha;

        Eigen::Isometry3f Tab = Eigen::Isometry3f::Identity();
        Tab.rotate(Rab);
        Tab.pretranslate(Eigen::Vector3f(-deltaX,0,deltaZ));


        convetPC(PC_I_RS16,Tab);
        *PC_I_combine = *PC_I_RS16 + *PC_I_RS32;
        XYZI2XYZRGB(PC_I_combine,PC_C_combine);
        viewer->updatePointCloud(PC_C_combine,"PC_C_combine");
        viewer->spinOnce();



        ///////////////////////////////离线分析之后求误差/////////////////////////////////////
        /*
        float s35 = 0.5735764;
        float c35 = 0.8191520;
        float B0InA_X = -0.02376608f;//米
        float B0InA_Y = 0;
        float B0InA_Z = 0.17115920f;
         */


        float alpha_final = (35.2943f/180)*3.1415926f;
        float beta_final = (91.641f/180)*3.1415926f;
        float deltaX_final = -0.0149215f;
        float deltaZ_final = 0.140982f;

        Eigen::Matrix3f Rab_final = Eigen::Matrix3f::Identity();

        Rab_final <<  cos(beta_final),  -sin(beta_final)*cos(alpha_final),   sin(beta_final)*sin(alpha_final),
                sin(beta_final),   cos(beta_final)*cos(alpha_final),   -cos(beta_final)*sin(alpha_final),
                0,                 sin(alpha_final),                   cos(alpha_final);

        Eigen::Isometry3f Tab_final = Eigen::Isometry3f::Identity();
        Tab_final.rotate(Rab_final);
        Tab_final.pretranslate(Eigen::Vector3f(-deltaX_final,0,deltaZ_final));

        convetPC(PC_I_RS16_final,Tab_final);



        //误差分析
        float eta = 0;
        for(int i=0;i<PC_I_RS16_final->size();i++){
            PointT_I temp;
            temp = PC_I_RS16_final->points[i];

            float D = sqrtf(temp.x*temp.x + temp.y*temp.y +temp.z*temp.z);
            float L3 = abs(a3*temp.x + b3*temp.y + c3*temp.z +d3)/sqrtf(a3*a3 + b3*b3 + c3*c3);
            float L4 = abs(a4*temp.x + b4*temp.y + c4*temp.z +d4)/sqrtf(a4*a4 + b4*b4 + c4*c4);

            eta+=(L3<L4?L3:L4)/D;
        }
        eta /= PC_I_RS16_final->size();
        outfileCali << " eta: " << eta << endl;




        *PC_I_Combine_final = *PC_I_RS16_final + *PC_I_RS32;
        XYZI2XYZRGB(PC_I_Combine_final,PC_C_Combine_final);
        viewer->updatePointCloud(PC_C_Combine_final,"PC_C_Combine_final");
        viewer->spinOnce();


        cout << "*************************************************"<<endl << endl;
    }

  return 0;
}


