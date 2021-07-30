//
// Created by cyn on 20-12-9.
#include "cali.h"


void cutCloud(PointCloud_I::Ptr &PC_I_raw,int pitchMin_deg,int pitchMax_deg,int yawMin_deg,int yawMax_deg){
    float PI = 3.1415926f;

    float pitchMax = pitchMax_deg*PI/180;
    float pitchMin = pitchMin_deg*PI/180;
    float yawMax = yawMax_deg*PI/180;
    float yawMin = yawMin_deg*PI/180;

    //16线点云的地面提取(限定一小块范围)
    PointCloud_I::Ptr PC_I_ground(new PointCloud_I);

    for(int i=0;i<PC_I_raw->size();i++) {
        PointT_I temp = PC_I_raw->points[i];
        float pointYaw_rad = (temp.intensity-(int)temp.intensity)*2*PI;
        float pointPitch_rad = atanf(temp.z/sqrtf(temp.x*temp.x + temp.y*temp.y));

        if(yawMin>yawMax){//300-60这种情况
            if(pointPitch_rad<pitchMax && pointPitch_rad>pitchMin &&
                    ((pointYaw_rad<2*PI && pointYaw_rad>yawMin)||(pointYaw_rad<yawMax && pointYaw_rad>0))){
                PC_I_ground->points.push_back(temp);
            }
        }
        else{
            if(pointPitch_rad<pitchMax && pointPitch_rad>pitchMin && pointYaw_rad<yawMax && pointYaw_rad>yawMin){
                PC_I_ground->points.push_back(temp);
            }
        }
    }

    PC_I_raw->points = PC_I_ground->points;

}



bool groundEstimate_cali(int estimateNum,PointCloud::Ptr &pointcloud,
                    pcl::ModelCoefficients &PlaneCoeff2Show,float outDistance,
                    float MaxDeviaAngle_deg,bool &precise,ofstream &outfile){
    vector<vector<float> > esti_Result;

    int exec_estimateNum=0;


    //多次进行随机采样 estimateNum采样次数
    while(exec_estimateNum != estimateNum){

        //随机采样一致性计算出地面方程
        vector<int> inliers;//inliers存储结果点ID
        pcl::ModelCoefficients PlaneCoeff2Show_try;//用于显示的平面参数变量

        if(pointcloud->size()<20) return false;
        plane_match(pointcloud,inliers,PlaneCoeff2Show_try,outDistance);

        float c = PlaneCoeff2Show_try.values[2];
        float a = PlaneCoeff2Show_try.values[0]/c;
        float b = PlaneCoeff2Show_try.values[1]/c;
        float d = PlaneCoeff2Show_try.values[3]/c;
        c=1;

        vector<float> temp(3);
        temp[0] = atanf(a)*180/3.1415926f;//pitch角度
        temp[1] = atanf(b)*180/3.1415926f;//roll角度
        temp[2] = d/(sqrtf(a*a+b*b+1));//面到原点距离

        if(abs(temp[0])>20 || abs(temp[1])>20)
            continue;

        esti_Result.push_back(temp);
        exec_estimateNum++;
    }


    if(esti_Result.empty())
        return false;

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
        else{
            //cout << "Ground Plane is not precise!!!!!!!!!!!!!!!!!!!!!!!" <<endl;
        }
    }
    else {
        //cout << "Ground Plane is not precise!!!!!!!!!!!!!!!!!!!!!!!" <<endl;
    }


    if(precise){
        cout << "*** ground distance: " << average_esti[2]
             << "  pitch: " << average_esti[0] << "   roll: " << average_esti[1] << endl;

        outfile << " ground_pitch: " << average_esti[0] << " ground_roll: " << average_esti[1];
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

    return true;
}


bool wallEstimate_cali(int estimateNum,PointCloud::Ptr &pointcloud,
                         pcl::ModelCoefficients &PlaneCoeff2Show,float outDistance,
                         float MaxDeviaAngle_deg,bool &precise,ofstream &outfile){
    vector<vector<float> > esti_Result;

    int exec_estimateNum=0;


    //多次进行随机采样 estimateNum采样次数
    while(exec_estimateNum != estimateNum){

        //随机采样一致性计算出地面方程
        vector<int> inliers;//inliers存储结果点ID
        pcl::ModelCoefficients PlaneCoeff2Show_try;//用于显示的平面参数变量

        if(pointcloud->size()<20) return false;
        plane_match(pointcloud,inliers,PlaneCoeff2Show_try,outDistance);

        float a = PlaneCoeff2Show_try.values[0];

        float c = PlaneCoeff2Show_try.values[2]/a;
        float b = PlaneCoeff2Show_try.values[1]/a;
        float d = PlaneCoeff2Show_try.values[3]/a;
        a=1;

        vector<float> temp(3);
        temp[0] = atanf(b)*180/3.1415926f;//yaw角度
        temp[1] = atanf(c)*180/3.1415926f;//pitch角度
        temp[2] = d/(sqrtf(b*b+c*c+1));//面到原点距离

        if(abs(temp[0])>20 || abs(temp[1])>20)
            continue;

        esti_Result.push_back(temp);
        exec_estimateNum++;
    }


    if(esti_Result.empty())
        return false;

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

    //确定最终的yaw、pitch、L估计值
    if(newEstimateNum){
        average_esti_2[0]/=newEstimateNum;
        average_esti_2[1]/=newEstimateNum;
        average_esti_2[2]/=newEstimateNum;

        average_esti = average_esti_2;
        if(newEstimateNum>= estimateNum/5) precise = true;
        else {
            //cout << "Wall Plane is not precise!!!!!!!!!!!!!!!!!!!!!!!" <<endl;
        }
    }
    else {
        //cout << "Wall Plane is not precise!!!!!!!!!!!!!!!!!!!!!!!" <<endl;
    }

    if(precise){
        cout << "*** Wall distance: " << -average_esti[2]
             << "  yaw: " << average_esti[0] << "   pitch: " << average_esti[1]<< endl;

        outfile << " wall_yaw: " << average_esti[0] << " wall_pitch: " << average_esti[1];
    }

    //最终的pitch、roll、L估计值转换为 平面参数
    float a_esti = 1;
    float b_esti = tanf((average_esti[0]*3.1415926f)/180)*a_esti;
    float c_esti = tanf((average_esti[1]*3.1415926f)/180)*a_esti;
    float d_esti = average_esti[2]*(sqrt(b_esti*b_esti + c_esti*c_esti+1));

    PlaneCoeff2Show.values.clear();
    PlaneCoeff2Show.values.push_back(a_esti);
    PlaneCoeff2Show.values.push_back(b_esti);
    PlaneCoeff2Show.values.push_back(c_esti);
    PlaneCoeff2Show.values.push_back(d_esti);

    return true;
}


//梯形方法去除地面点
void PlaneFilter(PointCloud_I::Ptr& PC_I_Input,pcl::ModelCoefficients &PlaneCoeff2Show,float outlineDis){

    //计算考虑误差在内的平面方程
    //令c=1;

    float a = PlaneCoeff2Show.values[0];
    float b = PlaneCoeff2Show.values[1];
    float c = PlaneCoeff2Show.values[2];
    float d = PlaneCoeff2Show.values[3];

    float temp = sqrtf(a*a+b*b+c*c);

    PointCloud_I::Ptr result_data(new PointCloud_I);
    for(int i=0;i<PC_I_Input->size();i++){
        PointT_I p0= PC_I_Input->points[i];

        float H = abs((a*p0.x+b*p0.y+c*p0.z+d)/temp); //该点到平面距离(在平面上为正)


        if(H > outlineDis){
            result_data->points.push_back(p0);
        }
    }
    *PC_I_Input = *result_data;
}



bool plane_cali16(int estimateNum,PointCloud::Ptr &pointcloud_raw,
                       pcl::ModelCoefficients &PlaneCoeff2Show,float outDistance,
                       int divideAngle_deg,bool planeFlag,float MaxDeviaAngle_deg,bool &precise){//ture墙面 false地面

    PointCloud::Ptr pointcloud(new PointCloud);
    for(int i=0;i<pointcloud_raw->size();i++){
        PointT tempPoint;
        tempPoint.x = pointcloud_raw->points[i].x;
        tempPoint.y = pointcloud_raw->points[i].y;
        tempPoint.z = pointcloud_raw->points[i].z;


        if(planeFlag && atanf(tempPoint.z/abs(tempPoint.y))*180/3.1415926f > divideAngle_deg)//z/|y|
            pointcloud->push_back(tempPoint);
        else if(!planeFlag && atanf(tempPoint.z/abs(tempPoint.y))*180/3.1415926f < divideAngle_deg)//地面
            pointcloud->push_back(tempPoint);
    }

    vector<vector<float> > esti_Result;

    int exec_estimateNum=0;


    //多次进行随机采样 estimateNum采样次数
    while(exec_estimateNum != estimateNum){

        //随机采样一致性计算出地面方程
        vector<int> inliers;//inliers存储结果点ID
        pcl::ModelCoefficients PlaneCoeff2Show_try;//用于显示的平面参数变量

        if(pointcloud->size()<20) return false;
        plane_match(pointcloud,inliers,PlaneCoeff2Show_try,outDistance);


        float c = PlaneCoeff2Show_try.values[2];
        float a = PlaneCoeff2Show_try.values[0]/c;
        float b = PlaneCoeff2Show_try.values[1]/c;
        float d = PlaneCoeff2Show_try.values[3]/c;
        c=1;

        esti_Result.push_back({a,b,c,d});
        exec_estimateNum++;
    }


    if(esti_Result.empty())
        return false;

    //求平均值
    vector<float> average_esti(4);
    float average_d = 0;
    for(auto& temp:esti_Result){
        average_esti[0] += temp[0];
        average_esti[1] += temp[1];
        average_esti[2] += temp[2];
        average_esti[3] += temp[3];
    }
    average_esti[0]/=estimateNum;
    average_esti[1]/=estimateNum;
    average_esti[2]/=estimateNum;
    average_esti[3]/=estimateNum;








    //去除极值
    vector<int> removeNum;
    for(int i=0;i<esti_Result.size();i++){
        vector<float> temp(3);

        temp[1] = esti_Result[i][1];//y坐标
        temp[2] = esti_Result[i][2];//z坐标
        float pitch_deg_residual =
                abs( atanf(temp[2]/abs(temp[1])) - atanf(average_esti[2]/abs(average_esti[1])) )/3.1415926f*180;

        if(pitch_deg_residual>MaxDeviaAngle_deg) {
            removeNum.push_back(i);
        }
    }
    for(int i=0;i<removeNum.size();i++){
        esti_Result.erase(esti_Result.begin()+removeNum[i]);
    }


    //再求平均值
    vector<float> average_esti_2(4);
    float average_d_2 = 0;
    int newEstimateNum=0;
    for(auto temp:esti_Result){
        average_esti_2[0] += temp[0];
        average_esti_2[1] += temp[1];
        average_esti_2[2] += temp[2];
        average_esti_2[3] += temp[3];

        newEstimateNum++;
    }


    if(newEstimateNum){
        average_esti_2[0]/=newEstimateNum;
        average_esti_2[1]/=newEstimateNum;
        average_esti_2[2]/=newEstimateNum;
        average_esti_2[3]/=newEstimateNum;

        average_esti = average_esti_2;
        if(newEstimateNum>= estimateNum/5) precise = true;
        else {
            //cout << "Wall Plane is not precise!!!!!!!!!!!!!!!!!!!!!!!" <<endl;
        }
    }
    else {
        //cout << "Wall Plane is not precise!!!!!!!!!!!!!!!!!!!!!!!" <<endl;
    }


    PlaneCoeff2Show.values.clear();
    PlaneCoeff2Show.values.push_back(average_esti[0]);
    PlaneCoeff2Show.values.push_back(average_esti[1]);
    PlaneCoeff2Show.values.push_back(average_esti[2]);
    PlaneCoeff2Show.values.push_back(average_esti[3]);

    return precise;
}

