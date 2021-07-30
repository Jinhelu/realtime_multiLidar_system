//
// Created by cyn on 20-1-11.
//
#include "GetFrame_RS.h"

bool initializeParam32(robosense::rslidar::ST_Param &param){

    param.lidar = robosense::rslidar::RS_Type_Lidar32;
    param.resolution = robosense::rslidar::RS_Resolution_5mm;
    //param.intensity = robosense::rslidar::RS_INTENSITY_EXTERN;
    param.echo = robosense::rslidar::RS_Echo_Last;//RS_Echo_Strongest;
    //param.echo = robosense::rslidar::RS_Echo_Dual;
    //param.echo = robosense::rslidar::RS_Echo_Last;
    param.cut_angle = 0.1f;
    param.max_distance = 30.0f;
    param.min_distance = 0.4f;
    param.start_angle = 280.0f;//0.0f;
    param.end_angle = 80.0f;//360.0f;
    //param.cali_path = "./RS32";

    return true;
}

bool initializeParam16(robosense::rslidar::ST_Param &param){

    param.lidar = robosense::rslidar::RS_Type_Lidar16;
    param.resolution = robosense::rslidar::RS_Resolution_5mm;
    //param.intensity = robosense::rslidar::RS_INTENSITY_EXTERN;
    param.echo = robosense::rslidar::RS_Echo_Last;//RS_Echo_Strongest;
    //param.echo = robosense::rslidar::RS_Echo_Dual;
    //param.echo = robosense::rslidar::RS_Echo_Last;
    param.cut_angle = 0.1f;
    param.max_distance = 30.0f;
    param.min_distance = 0.4f;
    param.start_angle = 10.0f;//0.0f;
    param.end_angle = 170.0f;//360.0f;
    //param.cali_path = "./RS32";

    return true;
}

//用于接收RS32激光雷达的一帧数据 输入XYZI数据　　//其中timestamp是雷达返回的以us计数的绝对时刻
bool GetFrame_RS(vector<PointCloud_I>& receiver, robosense::rslidar::RSLidarDecoder<PointXYZITS> &decoder,
                 RslidarInput::Input &InputObj, double &timestampFirst,long long &PCTime_u,IMUState &IMUdata,
                 queue_ts_IMU &IMUdata_Q){
    double timestamp;
    bool timeFlag=true;
    for(auto &PC_I:receiver)
        PC_I.clear();

    int pktNum = 0;

    vector<PointXYZITS> pointcloudI_buf;

    while(true){

        uint8_t pkt_buf[1248];//1248
        RslidarInput::InputState bufret = InputObj.getPacket(pkt_buf);
        //cout << "Lidar" << receiver.size() << ": " << " GetFrame 60:  "<< endl;
        //pointcloudI_buf.clear();

        ///////////////////////扫描数据输出处理
        if(bufret == RslidarInput::INPUT_MSOP){
            //process MSOP packet
            //int ret = decoder.processMsopPkt(pkt_buf+12, pointcloudI_buf, timestamp);
            int ret = decoder.processMsopPkt(pkt_buf, pointcloudI_buf, timestamp);

            if (ret == Rslidar::RS_Decode_ok) {
                if(timeFlag) {
                    timestampFirst=timestamp; timeFlag=false;

                    if(IMUdata_Q.ifOffline()){
                        int LidarType = (receiver.size()==32?32:16);

                        //更新PCTime_u，找到相对应的IMU数据
                        cout << "Aux.cpp " << (receiver.size()==32?32:16) 
                        << " GetFrame time:  " << timestamp << endl;
                        PCTime_u = IMUdata_Q.readLidarVisitRecord(LidarType,timestamp);
                        cout << "Aux.cpp " << (receiver.size()==32?32:16) << " PCTime_u: " << PCTime_u << endl;
                    }
                    else{
                         struct timeval tv;
                        gettimeofday(&tv,NULL);
                        PCTime_u = tv.tv_sec*1000000 + tv.tv_usec;
                    }



                    long long IMUPCtime;
                    getLatestIMUdata(IMUdata,IMUdata_Q,IMUPCtime);

                }
                //cout << "Pkt共计点数：" << pointcloudI_buf.size() << endl << "时间：" << timestamp << endl;
                //cout <<"**************************************************"<< endl << endl;
                //for(const auto & i : pointcloudI_buf){
                //    pointcloud_I->points.push_back(i);
                //}
                //*receiver += *pointcloud_I;//点云内容可以直接加，指针不可以直接加
                pktNum++;

            }
            else if(ret== Rslidar::RS_Frame_Split){//一帧结束，新一帧开始

                //cout << "一帧点数: " << pointcloudI_buf.size() << endl;
                for(int i=0;i<pointcloudI_buf.size();i++){
                    PointT_I temp;

                    double deltaTime = pointcloudI_buf[i].timeStamp - timestampFirst;//us为单位

                    
                    temp.x = pointcloudI_buf[i].x;
                    temp.y = pointcloudI_buf[i].y;
                    temp.z = pointcloudI_buf[i].z;
                    temp.intensity = pointcloudI_buf[i].intensity;
                    int scanID = pointcloudI_buf[i].scanID;

                    //对于16线，ID号不连续
                    receiver[scanID].points.push_back(temp);
                }

                //cout << "GetFrame 135:  " << PCTime_u << endl;
                if(PCTime_u) return true;
                else return false;
            }
            else{
                if(ret== Rslidar::RS_Decode_Fail){
                    cout << "ERROR: packet MSOP decode error accure!" << endl;
                }
                else if(ret== Rslidar::RS_Param_Invalid){
                    cout << "ERROR: input MSOP packet buffer pointer invalid!" << endl;
                }
            }
        }//INPUT_MSOP

            ///////////////////////设备信息输出处理
        else if(bufret == RslidarInput::INPUT_DIFOP){
            int ret = decoder.processDifopPkt(pkt_buf);

            if (ret== Rslidar::RS_Decode_ok){

                //cout << "RslidarInput::INPUT_DIFOP : RS_Decode_ok " << endl;

            }
            else if(ret== Rslidar::RS_Frame_Split){
                cout << "Warning: packet DIFOP decode ok and match frame split!" << endl;
            }
            else{
                if(ret== Rslidar::RS_Decode_Fail){
                    cout << "ERROR: packet DIFOP decode error accure!" << endl;
                }
                else if(ret== Rslidar::RS_Param_Invalid){
                    cout << "ERROR: input packet DIFOP buffer pointer invalid!" << endl;
                }
            }
        }//INPUT_DIFOP
        else {
            //cout<< "Lidar" << receiver.size() << ": "<<"Failed to get pkt_buf or resolve pkt_buf!" << endl;
        }

    }
}