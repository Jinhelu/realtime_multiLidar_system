#include "PointCloudManage.h"

//带反射率的点云转换为坐标点云
bool XYZI2XYZ(const PointCloud_I::Ptr& raw,PointCloud::Ptr& result){
  result->clear();
  for(int i=0; i < raw->size() ;i++){
    PointT p;

    p.x = raw->points[i].x;
    p.y = raw->points[i].y;
    p.z = raw->points[i].z;


    result->points.push_back(p);
  }
return true;
}


//带反射率的点云转换为颜色点云，用于显示
bool XYZI2XYZRGB(const PointCloud_I::Ptr& raw,PointCloud_C::Ptr& result){
  result->clear();
  for(int i=0; i < raw->size();i++){
    PointT_C p;

    p.x = raw->points[i].x;
    p.y = raw->points[i].y;
    p.z = raw->points[i].z;

    //cout << "intensity： " << (int)raw->points[i].intensity <<endl;

    if((int)raw->points[i].intensity < 90){
      p.r = int(0);
      p.g = int(0 + raw->points[i].intensity*2.82f);
      p.b = int(255 - raw->points[i].intensity*2.82f);
    }
    else if ((int)raw->points[i].intensity >= 90) {
      p.r = int((raw->points[i].intensity-90)*1.53f);
      p.g = int(255 - (raw->points[i].intensity-90)*1.53f);
      p.b = int(0);
    }
    result->points.push_back(p);
  }

  return true;
}


bool XYZI2XYZRGB_NoRaw(PointCloud_I & raw,PointCloud_C::Ptr result){
    result->clear();
    for(int i=0; i < raw.size();i++){
        PointT_C p;

        p.x = raw.points[i].x;
        p.y = raw.points[i].y;
        p.z = raw.points[i].z;

        //cout << "intensity： " << (int)raw->points[i].intensity <<endl;

        if((int)raw.points[i].intensity < 90){
            p.r = int(0);
            p.g = int(0 + raw.points[i].intensity*2.82f);
            p.b = int(255 - raw.points[i].intensity*2.82f);
        }
        else if ((int)raw.points[i].intensity >= 90) {
            p.r = int((raw.points[i].intensity-90)*1.53f);
            p.g = int(255 - (raw.points[i].intensity-90)*1.53f);
            p.b = int(0);
        }
        result->points.push_back(p);
    }

    return true;
}


//#include "PacketDriver.h"
//#include "PacketDecoder.h"
//TODO 目前该函数仅仅适用于Velodyne 要用头文件 #include "PacketDriver.h" #include "PacketDecoder.h"
/*
//补偿激光雷达角度调整引起的坐标系偏差
//alpha是转角，低头为正  deltaZ是激光雷达中心到安装转轴的距离(单位 米)
bool AdjustPointClound(PacketDecoder::HDLFrame * raw_data,double alpha,double deltaZ) {

  Eigen::Isometry3d transform_Mat = Eigen::Isometry3d::Identity();

  //向下平移
  Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
  T1.pretranslate(Eigen::Vector3d(0,0,-deltaZ));

  //绕x轴旋转
  Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();
  Eigen::AngleAxisd rotation_vector (-alpha*M_PI/180.0,Eigen::Vector3d(1,0,0) );
  T2.rotate(rotation_vector);

  //向上平移
  Eigen::Isometry3d T3 = Eigen::Isometry3d::Identity();
  T3.pretranslate(Eigen::Vector3d(0,0,deltaZ));

  //总体的变换矩阵
  transform_Mat = T1*T2*T3;

  ////////////////////对每个点进行坐标变换////////////////////////
  for (int i=0;i<raw_data->x.size();i++){
    Eigen::Vector3d v (raw_data->x[i],
                       raw_data->y[i],
                       raw_data->z[i]);
    v = transform_Mat*v;

    raw_data->x[i] = v[0];
    raw_data->y[i] = v[1];
    raw_data->z[i] = v[2];

  }
  ////////////////////////////////////////////////////////////

  return true;
}
*/

//直通滤波,限制点云的xyz范围
bool ThroughFilter(const PointCloud_I::Ptr& raw_data,double max_x,double min_x,
                   double max_y,double min_y, double max_z,double min_z){
    PointCloud_I::Ptr result_data(new PointCloud_I);
    for(int i=0;i< raw_data->size();i++){
        if(raw_data->points[i].x > min_x && raw_data->points[i].x < max_x &&
            raw_data->points[i].y > min_y && raw_data->points[i].y < max_y &&
                raw_data->points[i].z > min_z && raw_data->points[i].z < max_z){
            result_data->points.push_back(raw_data->points[i]);
        }
    }
    raw_data->clear();
    *raw_data=*result_data;
    return true;
}

void outlinerRemove(PointCloud_I& PC_I,int meanK,float StddevMulThresh){
    PointCloud_I::Ptr temp (new PointCloud_I);
    *temp = PC_I;

    pcl::StatisticalOutlierRemoval<PointT_I> sor;
    sor.setInputCloud(temp);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(StddevMulThresh);
    sor.filter(PC_I);
};



//将点云投影到平面
//                                        原始数据           低头角度      投影平面高度(雷达坐标系下)
bool cast_filter_plane(const PointCloud::Ptr& raw_data,double alpha,double z,
        const PointCloud::Ptr& data_filtered){

  //平面方程 ax+by+cz+d=0
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = 0.0f;                                         //系数a
  coefficients->values[1] = static_cast<float>(tan(alpha*M_PI/180.0));    //系数b
  coefficients->values[2] = -1.0f;                                        //系数c
  coefficients->values[3] = static_cast<float>(z);                        //系数d

  //投影的初始化
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (raw_data);
  proj.setModelCoefficients (coefficients);
  proj.filter (*data_filtered);

  return true;
}


//SIFT关键点检测
//                              输入点云           最小尺度标准差      高斯金字塔组数
bool SIFTkeypoint(const PointCloud_I::Ptr& pointcloud_I, float min_scale, int n_octaves,
//                 组的计算的尺度数              设置关键点检测阈值      输出的关键点
                  int n_scales_per_octave,float min_contrast,const PointCloud_I::Ptr& cloud_temp) {
    pcl::SIFTKeypoint<pcl::PointXYZI, pcl::PointWithScale> sift;//创建sift关键点检测对象
    pcl::PointCloud<pcl::PointWithScale> result;//创建sift关键点检测结果


    sift.setInputCloud(pointcloud_I);//设置输入点云
    pcl::search::KdTree<PointT_I>::Ptr tree(new pcl::search::KdTree<PointT_I>());
    sift.setSearchMethod(tree);//创建一个空的kd树对象tree，并把它传递给sift检测对象
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);//指定搜索关键点的尺度范围
    //               0.05        6               4
    sift.setMinimumContrast(min_contrast);//设置限制关键点检测的阈值
    //                           0.05
    sift.compute(result);//执行sift关键点检测，保存结果在result
    //将点类型pcl::PointWithScale的数据转换为点类型pcl::PointXYZI的数据
    copyPointCloud(result, *cloud_temp);
    cout<<"SIFT关键点数目： "<<cloud_temp->size()<<endl;

    return true;
}

//法线估计////////////////////////////////////////
//                            输入点云        k近邻搜索的k值    估计法线的结果
bool normal(const PointCloud_I::Ptr& pointcloud_I,  int k,
        pcl::PointCloud<pcl::Normal>::Ptr normals){
    //pcl::NormalEstimation<PointT_I, pcl::Normal> n;//法线估计对象
    pcl::NormalEstimationOMP<PointT_I, pcl::Normal> n;//法线估计对象
    //pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储法线容器
    pcl::search::KdTree<PointT_I>::Ptr tree_normal(new pcl::search::KdTree<PointT_I>);
    tree_normal->setInputCloud(pointcloud_I);//初始化kdtree
    n.setInputCloud(pointcloud_I);//输入点云
    n.setSearchMethod(tree_normal);//搜索方法
    n.setKSearch(k);//k近邻搜索的k值 10
    n.compute(*normals);//估计法线的结果

    return true;
}



//PointCloud_FPFH33::Ptr fpfhs(new  PointCloud_FPFH33);//fpfh特征向量
//         特征点    点云整体  法向量   fpfh特征向量   fpfh搜索半径
bool fpfh(const PointCloud_I::Ptr& cloud_temp,PointCloud_I::Ptr pointcloud_I,
          const pcl::PointCloud<pcl::Normal>::Ptr& normals,
          const PointCloud_FPFH33::Ptr& fpfhs,float RadiusSearch){
//fpfh特征提取////////////////////////////////////////
    //pcl::FPFHEstimation<PointT_I, pcl::Normal, PointT_FPFH33> fpfh;//fpfh特征提取对象
    pcl::FPFHEstimationOMP<PointT_I, pcl::Normal, PointT_FPFH33> fpfh;//fpfh特征提取对象
    fpfh.setInputCloud(cloud_temp);//输入点云是关键点
    fpfh.setSearchSurface(pointcloud_I);//搜索表面是整个点云
    fpfh.setInputNormals(normals);//输入点云的法向量
    pcl::search::KdTree<PointT_I>::Ptr tree_fpfh(new pcl::search::KdTree<PointT_I>);//初始化树
    fpfh.setSearchMethod(tree_fpfh);//kd树输入
    //PointCloud_FPFH33::Ptr fpfhs(new  PointCloud_FPFH33);//fpfh特征向量
    fpfh.setRadiusSearch(RadiusSearch);//0.07
    fpfh.compute(*fpfhs);

    return true;
}



//用方框框选目标点　待框选点云　方框序号向量　显示handle 方框边长　显示窗口序号
bool BoxPoints(const PointCloud_I::Ptr& pointcloudI_target,  vector<int> &BoxAppendix,
               const boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,
               float size,int viewport){

    //清除上次的矩形框
    for(int BoxAppendixID : BoxAppendix){
        //cout << "clear BOX......" << endl;
        viewer->removeShape("box" + std::to_string(BoxAppendixID));
        //c++11的函数std::to_string
        //cout << "removeCube" << "box" << std::to_string(BoxAppendix[BoxAppendixID]) <<endl;
    }
    BoxAppendix.clear();


    Eigen::AngleAxisf RotateVector (0.0,Eigen::Vector3f(0, 0, 1));//用于设置矩形框的旋转角度
    //框选目标
    for(int i=0;i<pointcloudI_target->size();i++){
        BoxAppendix.push_back(i);
        string boxname = "box"+std::to_string(i);
        //矩形框点坐标
        Eigen::Vector3f CorePoint(pointcloudI_target->points[i].x,
                                  pointcloudI_target->points[i].y,
                                  pointcloudI_target->points[i].z);
        viewer->addCube(CorePoint,//矩形中心坐标
                        Eigen::Quaternionf(RotateVector),//旋转向量转换成四元数
                        size,size, size,boxname,viewport);//长宽高(不是半个矩形) 名称
        //cout << "addCube" << boxname <<endl;
    }

    return true;
}

void GetHighItsityGroup(PointCloud_I::Ptr &RawPCPtr, PointCloud_I::Ptr &ResPCPtr,
                        int &ThresholdItsity,float &ErrrorPointSearchRadius, int &ErrrorPointNearNum) {
    PointCloud_I::Ptr IntensityFilted(new PointCloud_I);
    ///////////////////////////////////////////以下用于特目标检测//////////////////////////////////////////////////
    //滤波找高反射率点(一次滤波)
    for(int i=0;i<RawPCPtr->size();i++){
      if(RawPCPtr->points[i].intensity > static_cast<float>(ThresholdItsity) ){
        IntensityFilted->points.push_back( RawPCPtr->points[i] );
      }
    }
    //BoxPoints(pointcloudI_target_first,BoxAppendix,viewer,0.05);
    //cout << "强度高于" << ThresholdItsity << "的点的个数: " << pointcloudI_target_first->size() << endl<< endl;

    //二次滤波，用kd树去除散点
    if(!IntensityFilted->empty()){

      ResPCPtr -> clear();
      //近邻搜索滤除误判点
      float SearchRadius = ErrrorPointSearchRadius;//搜索半径
      pcl::KdTreeFLANN<PointT_I> kdtree; //建立kd树求近邻
      kdtree.setInputCloud(IntensityFilted);//设置输入点云

      for(int i=0;i<IntensityFilted->size();i++) {
        vector<int> pointIdxRadiusSearch;//距离搜索结果ID向量
        vector<float> pointRadiusSquaredDistance;//距离搜索结果距离向量

        PointT_I searchPoint(IntensityFilted->points[i]);//核心点设置
        //指定半径的近邻搜索   中心点        半径           结果ID向量  结果距离向量
        kdtree.radiusSearch(searchPoint,SearchRadius,
                            pointIdxRadiusSearch,pointRadiusSquaredDistance);

        //没有足够的近邻点认为是误判,不加入二次目标当中
        if(pointIdxRadiusSearch.size()>ErrrorPointNearNum){
          ResPCPtr->points.push_back(IntensityFilted->points[i]);
        }
      }

      //BoxPoints(pointcloudI_target_second,BoxAppendix,viewer,0.02);
    }
}



bool targetPosiAnaly(PointCloud_I::Ptr &PCItsityTarget,GuidObject &guidObject,
                    float &StdTargetSize,float &sor_vox_LeafSize,double &time_stt_us,ofstream &outfile) {

    //两个距离极值
    float maxDistance = 0;
    float secDistance = 0;

    PointT heartPoint(0,0,0);//中心点坐标
    for(int i=0;i<PCItsityTarget->size();i++){

        heartPoint.x += PCItsityTarget->points[i].x;
        heartPoint.y += PCItsityTarget->points[i].y;
        heartPoint.z += PCItsityTarget->points[i].z;


        for(int j=i+1;j<PCItsityTarget->size();j++){
            float deltaX = PCItsityTarget->points[j].x - PCItsityTarget->points[i].x;
            float deltaY = PCItsityTarget->points[j].y - PCItsityTarget->points[i].y;
            float deltaZ = PCItsityTarget->points[j].z - PCItsityTarget->points[i].z;

            float distance = sqrtf(deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ);
            if(distance>maxDistance) {
                secDistance = maxDistance ;
                maxDistance = distance;
            }
            else if(distance>secDistance){
                secDistance = distance;
            }
        }
    }

    heartPoint.x /= PCItsityTarget->size();
    heartPoint.y /= PCItsityTarget->size();
    heartPoint.z /= PCItsityTarget->size();


    //用三维的最大差值近似计算最大差值
    /*
    float min_x = 100, max_x = -100, min_y = 100, max_y = -100, min_z = 100, max_z = -100;
    for(int i=0; i < PCItsityTarget->size(); i++){
      if (PCItsityTarget->points[i].x < min_x)
        min_x = PCItsityTarget->points[i].x;
      if (PCItsityTarget->points[i].x > max_x)
        max_x = PCItsityTarget->points[i].x;

      if (PCItsityTarget->points[i].y < min_y)
        min_y = PCItsityTarget->points[i].y;
      if (PCItsityTarget->points[i].y > max_y)
        max_y = PCItsityTarget->points[i].y;

      if (PCItsityTarget->points[i].z < min_z)
        min_z = PCItsityTarget->points[i].z;
      if (PCItsityTarget->points[i].z > max_z)
        max_z = PCItsityTarget->points[i].z;
    }

    //找到目标标签的xyz三个方向占据宽度
    float target_deltaX = max_x-min_x;
    float target_deltaY = max_y-min_y;
    float target_deltaZ = max_z-min_z;
    //cout << "目标的三维坐标跨度: " << " deltaX: " << target_deltaX
    //     << " deltaY: " << target_deltaY << " deltaZ: " << target_deltaZ << endl;
    //cout << "min_y: " << min_y << "  max_y: " << max_y  << endl;
    */


    //求实际测得的对角线的长度
    double target_delta;
    if(maxDistance-secDistance<0.1 || secDistance==0) target_delta = maxDistance;
    else target_delta = secDistance;

    float target_delta_threshold = StdTargetSize - sor_vox_LeafSize;//理论的精确的对角线长度
    float allow_deviation = 3*sor_vox_LeafSize; //上述两个长度的可允许偏差
    double deviation = target_delta - target_delta_threshold ; //减去计算实际对角线的偏大误差

    if(RECORD_TARGET && target_delta<1.0 && target_delta>0)  outfile << target_delta << endl;
    //cout << "计算对角线偏差: " << deviation << endl;

    //target_position.x = 0;    target_position.y = 0;    target_position.z = 0;  //默认目标位置点为原点
    //如果误差在合理范围内,就计算出目标点的坐标 对角线长度 目标对象的距离
    if( fabs(deviation) < allow_deviation  ) {
        guidObject.timeStamp_us = time_stt_us;
        guidObject.posi.x = heartPoint.x;
        guidObject.posi.y = heartPoint.y;
        guidObject.posi.z = heartPoint.z;

        cout << "目标对象对角线长度: " << target_delta ;
        //cout << "目标对象位置坐标: " << "x:" <<target_position.x
        //     << " y:" << target_position.y << " z:" << target_position.z << endl;
        double target_distance = guidObject.target_distance();
        double horizontal_angel = guidObject.horizontal_angel();
        double vertical_angel = guidObject.vertical_angel();
        cout << "  水平角: " << (horizontal_angel>0?"左":"右") << fabs(horizontal_angel) << "°"
             <<"  垂直角: " << (vertical_angel>0?"上":"下") << fabs(vertical_angel) << "°" << endl;
        cout << "目标对象距离: " << target_distance << endl << "*****************************" << endl << endl;

        return true;
    }
    else {
        //GuidObject iniGuidObj;
        //guidObject = iniGuidObj;

        cout << "目标点云非跟随对象" << endl;
        return false;
    }
}


bool targetVeloAnaly(GuidObject &guidObject,GuidObject &lastGuidObj) {


    double delta_time_s = (guidObject.timeStamp_us-lastGuidObj.timeStamp_us)/(1000*1000);
    //目标与雷达的相对速度计算　拟合背部平面求出法向量作为人员朝向　
    if(lastGuidObj.posi.x != 0 && delta_time_s != 0){
      double Vx = (guidObject.posi.x - lastGuidObj.posi.x)/delta_time_s;
      double Vy = (guidObject.posi.y - lastGuidObj.posi.y)/delta_time_s;
      guidObject.velo.setVelocity(Vx,Vy);//此处已经计算过合速度
      cout << "delta_time_s: " << delta_time_s << " velo_x:" << guidObject.velo.Vx
           <<" velo_y:" <<  guidObject.velo.Vy << " velo_combine:" <<  guidObject.velo.combine << endl;

    return true;
    }
    else {
      cout << "Velocity Analysis failed!" << endl;
      return false;
    }

}

void RemoveGuid(GuidObject &guidObject,PointCloud_I::Ptr &PC_I_in,PointCloud_I::Ptr &PC_I_out) {
    //人体模型，长宽1.2m，高2m
    float Guid_Xmin = guidObject.posi.x - 0.6f;
    float Guid_Xmax = guidObject.posi.x + 0.6f;
    float Guid_Ymin = guidObject.posi.y - 0.6f;
    float Guid_Ymax = guidObject.posi.y + 0.6f;
    float Guid_Zmin = guidObject.posi.z - 1.4f;
    float Guid_Zmax = guidObject.posi.z + 0.8f;


    PC_I_out->clear();
    for(int i=0;i<PC_I_in->size();i++){
        bool removeFlag =
                PC_I_in->points[i].x>Guid_Xmin && PC_I_in->points[i].x<Guid_Xmax &&
                PC_I_in->points[i].y>Guid_Ymin && PC_I_in->points[i].y<Guid_Ymax &&
                PC_I_in->points[i].z>Guid_Zmin && PC_I_in->points[i].z<Guid_Zmax;

        if(!removeFlag){
            PC_I_out->points.push_back(PC_I_in->points[i]);
        }
    }
}

void convetPC(PointCloud_I::Ptr PC_I_raw_RS16,Eigen::Isometry3f& Tba){

    for(int i=0;i<PC_I_raw_RS16->points.size();i++){
        float &point16_x = PC_I_raw_RS16->points[i].x;
        float &point16_y = PC_I_raw_RS16->points[i].y;
        float &point16_z = PC_I_raw_RS16->points[i].z;

        Eigen::Vector3f point16_E(point16_x,point16_y,point16_z);
        point16_E = Tba*point16_E;
        point16_x = point16_E.x();
        point16_y = point16_E.y();
        point16_z = point16_E.z();
    }
};

void loamFeature(vector<PointCloud_I> &laserCloudScans, PointCloud_I &cornerPointsSharp,
                 PointCloud_I &cornerPointsLessSharp, PointCloud_I &surfPointsFlat,
                 PointCloud_I &surfPointsLessFlat, int N_SCANS) {
    cornerPointsSharp.clear();
    cornerPointsLessSharp.clear();
    surfPointsFlat.clear();
    surfPointsLessFlat.clear();

    float cloudCurvature[400000];
    int cloudSortInd[400000];
    int cloudNeighborPicked[400000];
    int cloudLabel[400000];

    int cloudSize = 0;
    for(auto &temp:laserCloudScans)
        cloudSize += temp.points.size();

    std::vector<int> scanStartInd(N_SCANS, 0);// 记录每个scan的开始index
    std::vector<int> scanEndInd(N_SCANS, 0);// 记录每个scan的结束index

    PointCloud_I::Ptr laserCloud(new PointCloud_I);
    for (int i = 0; i < N_SCANS; i++)
    {
        scanStartInd[i] = laserCloud->size() + 5;// 记录每个scan的开始index，忽略前5个点

        /***************************************************************/
        PointCloud_I temp (laserCloudScans[i]);
        for(int j=0;j<temp.size();j++){
            temp[j].intensity = i + temp[j].intensity - (int)temp[j].intensity;
        }
        *laserCloud += temp;
        /***************************************************************/

        scanEndInd[i] = laserCloud->size() - 6;// 记录每个scan的结束index，忽略后5个点，开始和结束处的点云scan容易产生不闭合的“接缝”，对提取edge feature不利
    }

    for (int i = 5; i < cloudSize - 5; i++)
    {
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x
                      + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x
                      + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x
                      + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
                      + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
                      + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y
                      + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y
                      + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y
                      + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
                      + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
                      + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z
                      + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z
                      + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z
                      + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
                      + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
                      + laserCloud->points[i + 5].z;

        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd[i] = i;
        cloudNeighborPicked[i] = 0;// 点有没有被选选择为feature点
        cloudLabel[i] = 0;// Label 2: corner_sharp
        // Label 1: corner_less_sharp, 包含Label 2
        // Label -1: surf_flat
        // Label 0: surf_less_flat， 包含Label -1，因为点太多，最后会降采样
    }


    //TicToc t_pts;

    for (int i = 0; i < N_SCANS; i++)// 按照scan的顺序提取4种特征点
    {
        if( scanEndInd[i] - scanStartInd[i] < 6)// 如果该scan的点数少于7个点，就跳过
            continue;
        PointCloud_I::Ptr surfPointsLessFlatScan(new PointCloud_I);
        for (int j = 0; j < 6; j++)// 将该scan分成6小段执行特征检测
        {
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;// subscan的起始index
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;// subscan的结束index

            //TicToc t_tmp;
            //todo 优化comp的写法
            sort (cloudSortInd + sp, cloudSortInd + ep + 1,
                  [&cloudCurvature](int firstNum,int secondNum)
                  {return cloudCurvature[firstNum]<cloudCurvature[secondNum]; }
            );
            // 根据曲率有小到大对subscan的点进行sort
            //t_q_sort += t_tmp.toc();

            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)// 从后往前，即从曲率大的点开始提取corner feature
            {
                int ind = cloudSortInd[k];

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.1)// 如果该点没有被选择过，并且曲率大于0.1
                {
                    largestPickedNum++;
                    if (largestPickedNum <= 2)// 该subscan中曲率最大的前2个点认为是corner_sharp特征点
                    {
                        cloudLabel[ind] = 2;
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else if (largestPickedNum <= 20)// 该subscan中曲率最大的前20个点认为是corner_less_sharp特征点
                    {
                        cloudLabel[ind] = 1;
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;// 标记该点被选择过了

                    // 与当前点距离的平方 <= 0.05的点标记为选择过，避免特征点密集分布
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            // 提取surf平面feature，与上述类似，选取该subscan曲率最小的前4个点为surf_flat
            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSortInd[k];

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1)
                {

                    cloudLabel[ind] = -1;
                    surfPointsFlat.push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }


            // 其他的非corner特征点与surf_flat特征点一起组成surf_less_flat特征点
            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[k] <= 0)
                {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }

        // 最后对该scan点云中提取的所有surf_less_flat特征点进行降采样，因为点太多了
        PointCloud_I surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointT_I> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        surfPointsLessFlat += surfPointsLessFlatScanDS;
    }
}

// undistort lidar point
void TransformToStart(PointT_I const *const pi, PointT_I *const po,
                      Eigen::Quaterniond &q_last_curr, Eigen::Vector3d &t_last_curr) {
    //interpolation ratio
    double s;
    //if (DISTORTION)
    //    s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
    //else
    //    s = 1.0;
    //s = 1;
    s = (pi->intensity - int(pi->intensity));
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
    Eigen::Vector3d t_point_last = {s * t_last_curr[0],s * t_last_curr[1],s * t_last_curr[2],};
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;

    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}

// transform all lidar points to the start of the next frame
void TransformToEnd(PointT_I const *const pi, PointT_I *const po,
                    Eigen::Quaterniond &q_last_curr, Eigen::Vector3d &t_last_curr) {
    // undistort point first
    pcl::PointXYZI un_point_tmp;
    TransformToStart(pi, &un_point_tmp,q_last_curr,t_last_curr);

    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();

    //Remove distortion time info
    po->intensity = int(pi->intensity);
}


//给定点云匹配其中的平面
//               原始点云   面内的点云的序号向量    拟合后的参数   拟合精度(默认0.01米)
bool plane_match(PointCloud::Ptr &pointcloud,
                 vector<int> &inliers,pcl::ModelCoefficients &PlaneCoeff2Show,
                 float precision){
    //PlaneCoeff2Show.values.clear();
    //inliers.clear();
    Eigen::VectorXf coefficients;//声明变量存储参数
    pointcloud->points.pop_back();//随机采样一致性的随机数根据点个数给出，点数不变结果不变
    //创建随机采样一致性对象
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
            model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (pointcloud));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);



    ransac.setDistanceThreshold (precision);   //与平面的距离小于0.01作为局内点考虑
    ransac.computeModel();                //执行随机参数估计
    ransac.getInliers(inliers);           //存储局内点
    //ransac.setMaxIterations(6);
    //ransac.setProbability(0.85);
    ransac.getModelCoefficients(coefficients);

    //cout<<"开始参数转换"<<endl;
    //参数转换
    PlaneCoeff2Show.values.push_back(coefficients.x());
    PlaneCoeff2Show.values.push_back(coefficients.y());
    PlaneCoeff2Show.values.push_back(coefficients.z());
    PlaneCoeff2Show.values.push_back(coefficients.w());

    //cout<<"参数转换完成"<<endl;

    return true;
}

void VoxFilter(PointCloud_I::Ptr &PC_I_input,float sor_vox_LeafSize){
    //基于体素网格的滤波  输入输出不要选同一个容器！！！！！！
    PointCloud_I::Ptr filter_temp(new PointCloud_I);
    filter_temp->points = PC_I_input->points;

    pcl::VoxelGrid<PointT_I> sor_vox;//创建滤波对象
    sor_vox.setInputCloud(filter_temp);
    sor_vox.setLeafSize(sor_vox_LeafSize,
                        sor_vox_LeafSize,sor_vox_LeafSize);//体素网格大小
    sor_vox.filter(*PC_I_input);
//cout << "体素网格的滤波完成" << endl;
}


void groundEstimate(int estimateNum,PointCloud::Ptr &pointcloud,
                    pcl::ModelCoefficients &PlaneCoeff2Show,float outDistance,
                    float MaxDeviaAngle_deg,bool &precise){
    vector<vector<float> > esti_Result;

    int exec_estimateNum=0;


    //多次进行随机采样 estimateNum采样次数
    while(exec_estimateNum != estimateNum){
        exec_estimateNum++;
        //随机采样一致性计算出地面方程
        vector<int> inliers;//inliers存储结果点ID
        pcl::ModelCoefficients PlaneCoeff2Show_try;//用于显示的平面参数变量

        plane_match(pointcloud,inliers,PlaneCoeff2Show_try,outDistance);

        float c = PlaneCoeff2Show_try.values[2];
        float a = PlaneCoeff2Show_try.values[0]/c;
        float b = PlaneCoeff2Show_try.values[1]/c;
        float d = PlaneCoeff2Show_try.values[3]/c;
        c=1;

        vector<float> temp(3);
        temp[0] = atanf(a)*180/3.1415926f;//pitch角度
        temp[1] = atanf(b)*180/3.1415926f;//roll角度
        temp[2] = fabs(d/(sqrtf(a*a+b*b+1)));//面到原点距离

        esti_Result.push_back(temp);
    }


    //求平均值 pitch、roll、L的平均值
    vector<float> average_esti(3);
    float average_d = 0;
    for(auto& temp:esti_Result){
        average_esti[0] += temp[0];
        average_esti[1] += temp[1];
        average_esti[2] += temp[2];
    }
    average_esti[0]/=estimateNum;
    average_esti[1]/=estimateNum;
    average_esti[2]/=estimateNum;


    //去除极值
    vector<int> removeNum;
    for(int i=0;i<esti_Result.size();i++){
        vector<float> temp(3);
        temp[0] = esti_Result[i][0];
        temp[1] = esti_Result[i][1];
        temp[2] = esti_Result[i][2];

        if(temp[0]>MaxDeviaAngle_deg || temp[1]>MaxDeviaAngle_deg) {
            removeNum.push_back(i);
        }
    }
    for(int i=0;i<removeNum.size();i++){
        esti_Result.erase(esti_Result.begin()+removeNum[i]);
    }


    //再求平均值
    vector<float> average_esti_2(3);
    float average_d_2 = 0;
    int newEstimateNum=0;
    for(auto temp:esti_Result){
        average_esti_2[0] += temp[0];
        average_esti_2[1] += temp[1];
        average_esti_2[2] += temp[2];

        newEstimateNum++;
    }

    //确定最终的pitch、roll、L估计值
    if(newEstimateNum){
        average_esti_2[0]/=newEstimateNum;
        average_esti_2[1]/=newEstimateNum;
        average_esti_2[2]/=newEstimateNum;

        average_esti = average_esti_2;
        if(newEstimateNum>= estimateNum/5) precise = true;
        else cout << "Ground Plane is not precise!!!!!!!!!!!!!!!!!!!!!!!" <<endl;
    }
    else {
        cout << "Ground Plane is not precise!!!!!!!!!!!!!!!!!!!!!!!" <<endl;
    }


    //最终的pitch、roll、L估计值转换为 平面参数
    float c_esti = 1;
    float a_esti = tanf((average_esti[0]*3.1415926f)/180)*c_esti;
    float b_esti = tanf((average_esti[1]*3.1415926f)/180)*c_esti;
    float d_esti = average_esti[2]*(sqrt(a_esti*a_esti + b_esti*b_esti+1));

    PlaneCoeff2Show.values.clear();
    PlaneCoeff2Show.values.push_back(a_esti);
    PlaneCoeff2Show.values.push_back(b_esti);
    PlaneCoeff2Show.values.push_back(c_esti);
    PlaneCoeff2Show.values.push_back(d_esti);
}

void cutRawGround(PointCloud_I::Ptr &PC_I_raw_RS16,PointCloud_I::Ptr &PC_I_ground,
                  float vox_LeafSize,int cutAngleYaw_deg,int cutAnglePitch_deg){

    float cutAngleYaw = cutAngleYaw_deg;
    float cutAnglePitch_rad = cutAnglePitch_deg*3.1415926f/180;


    //16线点云的地面提取(限定一小块范围)

    for(int i=0;i<PC_I_raw_RS16->size();i++){
        PointT_I temp = PC_I_raw_RS16->points[i];
        float zone = temp.intensity-(int)temp.intensity;
        float angleNum = fabs(temp.x/temp.z);
        if(zone<(0.25+cutAngleYaw/360) && zone>(0.25-cutAngleYaw/360) && angleNum<tan(cutAnglePitch_rad)){
            PC_I_ground->points.push_back(temp);
        }
    }
    VoxFilter(PC_I_ground,vox_LeafSize*1.5f);

}

//梯形方法去除地面点
void GroundFilter(PointCloud_I::Ptr& PC_I_Input,pcl::ModelCoefficients &PlaneCoeff2Show,
                  float outlineDis,float abovePlaneAngle_deg){

    //计算考虑误差在内的平面方程
    //令c=1;
    float c = PlaneCoeff2Show.values[2];
    float a = PlaneCoeff2Show.values[0]/c;
    float b = PlaneCoeff2Show.values[1]/c;
    float d = PlaneCoeff2Show.values[3]/c;
    c = 1.0f;

    float temp = sqrtf(a*a+b*b+1);

    PointCloud_I::Ptr result_data(new PointCloud_I);
    for(int i=0;i<PC_I_Input->size();i++){
        PointT_I p0= PC_I_Input->points[i];
        PointT_I p1;//投影点坐标

        p1.y = ((a*a+1)*p0.y-a*b*p0.x-b*p0.z-b*d)/(temp*temp);
        p1.x = (p0.x - a*p0.z - a*d -a*b*p1.y)/(a*a+1);
        p1.z = (-d-a*p1.x-b*p1.y);
        p1.intensity = p0.intensity;

        float H = (a*p0.x+b*p0.y+p0.z+d)/temp; //该点到平面距离(在平面上为正)
        float L = sqrtf(p1.x*p1.x + p1.y*p1.y + p1.z*p1.z);//投影点到原点距离

        float alpha = (atan((H-outlineDis)/L)/3.1415926f)*180;//原点到该点的向量与局外点平面的夹角

        if(alpha > abovePlaneAngle_deg){
            result_data->points.push_back(p0);
        }
    }
    *PC_I_Input = *result_data;
}


bool cast_filter_Ground(const PointCloud::Ptr& raw_data,pcl::ModelCoefficients &coefficients,
                       const PointCloud::Ptr& data_filtered){

    //平面方程 ax+by+cz+d=0
    pcl::ModelCoefficients::Ptr coefficientsPtr (new pcl::ModelCoefficients ());
    coefficientsPtr->values.resize (4);
    coefficientsPtr->values = coefficients.values;

    //投影的初始化
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (raw_data);
    proj.setModelCoefficients (coefficientsPtr);
    proj.filter (*data_filtered);

    return true;
}