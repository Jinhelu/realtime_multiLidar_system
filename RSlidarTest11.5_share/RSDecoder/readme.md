# 驱动rslidar_decoder使用说明
### 一、概述  
rslidar_decoder是新版本的16/32线雷达驱动，为C++程序实现。相比之前的雷达驱动，区别如下：  
1. 新驱动以模板类方式实现，适应不同point格式；而旧驱动只支持pcl库的PointXYZI
2. 新驱动关键几个功能合在一起：解析数据包，读取校准参数，数据分帧；旧驱动分帧和解包分开
3. 新驱动跟平台无关，方便移植不同平台；旧驱动依赖于ROS系统

### 二、程序组成
rslidar_decoder由两个文件组成：  
+ rslidar_packet.h: 这文件定义雷达数据包格式的数据结构
+ rslidar_decoder.hpp: 这文件驱动核心程序，定义模板类用于雷达数据解析输出点云数据

代码组成如下：
+ 雷达数据包结构体（MSOP/DIFOP包结构体, 初始化参数结构体)
+ 数据解析类RSLidarDecoder类

### 三、接口
1. RSLidarDecoder类对外提供3个接口：  
  ```
  + 构造函数：RSLidarDecoder(ST_Param &param)
  + MSOP包解析函数：E_DECODER_RESULT processMsopPkt(const uint8_t* pkt, std::vector<vpoint> &pointcloud_vec, double& timestamp)
  + DIFOP包解析函数：int32_t processDifopPkt(const uint8_t* pkt)
  ```

2. 构造函数要求传入参数，参数是一些初始化变量的结构体，具体参数定义如下：
  ```
  typedef struct
  {
    RS_LIDAR_TYPE lidar;				//雷达类型：RS_LIDAR_16 或者 RS_LIDAR_32
    RS_RESOLUTION_TYPE resolution;	//精度：0.5cm 或者 1cm
    RS_INTENSITY_TYPE intensity;		//反射率模式
    RS_ECHO_MODE echo;				//回波模式
    float cut_angle;					//分帧角度
    float max_distance;				//点云距离上限
    float min_distance;				//点云距离下限
    float start_angle;				//起始角度
    float end_angle;					//结束角度
    std::string cali_path;			//校准参数文件存放路径
  }ST_Param
  ```
  
3. MSOP包解析函数参数和返回值说明：  
  ```
  i）参数
  + const uint8_t* pkt: MSOP包数据头指针，指向MSOP包数据缓存
  + std::vector<vpoint> &pointcloud_vec: 点云存放缓存，为vector类型；解析函数生成的点云push_back到这个vector
  + double& timestamp: 存放MSOP解析出的时间戳变量，单位为us
  ii）返回值
  enum E_DECODER_RESULT
  {
    E_DECODE_FAIL = -2,		//解析出错，帧头数据不匹配
    E_PARAM_INVALID = -1,	//pkt指针为空
    E_DECODE_OK = 0,		//解析正常
    E_FRAME_SPLIT = 1		//解析正常并且满足分帧条件
  };
  ```
  
4. DIFOP包解析函数参数和返回值说明：
  ```
  i）参数
  + const uint8_t* pkt: DIFOP包数据头指针，指向DIFOP包数据缓存
  ii）返回值
  enum E_DECODER_RESULT
  {
    E_DECODE_FAIL = -2,		//解析出错，帧头数据不匹配
    E_PARAM_INVALID = -1,	//pkt指针为空
    E_DECODE_OK = 0,		//解析正常
  };
  ```

### 四、调用示例

1. 创建RSLidarDecoder类实例  
  ```
  //initialize param
  robosense::rslidar::ST_Param param;
  param.lidar = robosense::rslidar::RS_LIDAR_32;
  param.resolution = robosense::rslidar::RS_RESOLUTION_10mm;
  param.intensity = robosense::rslidar::RS_INTENSITY_EXTERN;
  param.echo = robosense::rslidar::RS_ECHO_STRONGEST;
  param.cut_angle = 0.1f;
  param.max_distance = 200.0f;
  param.min_distance = 0.2f;
  param.start_angle = 0.0f;
  param.end_angle = 360.0f;
  param.cali_path = "./RS32";
  // initialize decoder
  robosense::rslidar::RSLidarDecoder<dwLidarPointXYZI> decoder(param);
  ```
  
2. 调用processMsopPkt()处理MSOP数据包并处理输出点云数据
  ```
  //get the MSOP packet data
  status = dwSensor_readRawData(&data, &size, 800, msopHandler);
  if (status == DW_SUCCESS)
  {
    //process MSOP packet
    int ret = decoder.processMsopPkt(data+12, point_buf, timestamp);
    status = dwSensor_returnRawData(data, msopHandler);
    if (status != DW_SUCCESS) {
      std::cout << "[ERROR] msop Error returning packet " << std::endl;
    }
    packetCount ++;
    if (ret == robosense::rslidar::E_FRAME_SPLIT)
    {
      updateRenderBuffer(point_buf);
      point_buf.clear();
    }
  }
  ```
  
3. 调用processDifopPkt()处理DIFOP数据包
  ```
  status = dwSensor_readRawData(&data, &size, 300, difopHandler);//get the DIFOP packet data
  if (status == DW_SUCCESS)
  {
    decoder.processDifopPkt(data+12);//process DIFOP packet
    status = dwSensor_returnRawData(data, difopHandler);
    if (status != DW_SUCCESS)
    {
      std::cout << "[ERROR] difop Error returning packet " << std::endl;
    }
  }
  ```
  
