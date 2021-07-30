#ifndef IMU_GET_DATA_H
#define IMU_GET_DATA_H

#include <iostream>
#include <fstream>
//#include <chrono>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xstime.h>
#include <xscommon/xsens_mutex.h>

#include <iomanip>
#include <list>
#include <string>
using namespace std;

/* A standard non-ROS alternative to ros::Time.*/
//using Time = std::chrono::system_clock::time_point;
//将时间戳计算成s
/*
double toSec(Time::duration duration)
{
    return std::chrono::duration<double>(duration).count();
};*/

typedef struct IMUState//自己定义的imu数据类型，后面还需要根据实际情况更改对接接口
{
    /** The time of the measurement leading to this state (in seconds). */
    //Time stamp;
    int64_t timeStamp_ms;
    long long PCtime_us;
    Eigen::Quaterniond Quaternion;
    /** The current roll angle. */
    double roll;
    /** The current pitch angle. */
    double pitch;
    /** The current yaw angle. */
    double yaw;
    /** The current (local) IMU acceleration in 3D space. */
    vector<double>acceleration;
    vector<double>angularVelo;
    /** The accumulated global IMU position in 3D space. */
    vector<double> velocity;

    IMUState(){
        timeStamp_ms = 0;
        PCtime_us = 0;
        Quaternion = {0,0,0,0};
        /** The current roll angle. */
        roll = 0;
        /** The current pitch angle. */
        pitch = 0;
        /** The current yaw angle. */
        yaw = 0;
        /** The current (local) IMU acceleration in 3D space. */
        acceleration = {0,0,0};
        angularVelo = {0,0,0};
        /** The accumulated global IMU position in 3D space. */
        velocity = {0,0,0};
    }

    void write(ofstream &ImuRecordFile){
        ImuRecordFile << endl
                      <<"IMUtime: " <<timeStamp_ms << " "
                      <<"Quaternion: "<<Quaternion.x()<<" "<<Quaternion.y()<<" "<<Quaternion.z()<<" "<< Quaternion.w()<< " "
                      <<"RPY: "<< roll << " " << pitch << " " << yaw << " "
                      <<"acceleration: "<< acceleration[0] << " " << acceleration[1] << " " << acceleration[2] << " "
                      <<"angularVelo: "<< angularVelo[0] << " " << angularVelo[1] << " " << angularVelo[2] << " "
                      <<"velocity: " << velocity[0] << " " << velocity[1] << " " << velocity[2] << " ";
    }


} IMUState;




class CallbackHandler : public XsCallback
{
public:
    CallbackHandler(size_t maxBufferSize = 5)
            : m_maxNumberOfPacketsInBuffer(maxBufferSize)
            , m_numberOfPacketsInBuffer(0)
    {
    }

    virtual ~CallbackHandler() throw()
    {
    }

    bool packetAvailable() const
    {
        xsens::Lock locky(&m_mutex);
        return m_numberOfPacketsInBuffer > 0;
    }

    XsDataPacket getNextPacket()
    {
        assert(packetAvailable());
        xsens::Lock locky(&m_mutex);
        XsDataPacket oldestPacket(m_packetBuffer.front());
        m_packetBuffer.pop_front();
        --m_numberOfPacketsInBuffer;
        return oldestPacket;
    }

protected:
    void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override
    {
        xsens::Lock locky(&m_mutex);
        assert(packet != 0);
        while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
            (void)getNextPacket();

        m_packetBuffer.push_back(*packet);
        ++m_numberOfPacketsInBuffer;
        assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
    }
private:
    mutable xsens::Mutex m_mutex;

    size_t m_maxNumberOfPacketsInBuffer;
    size_t m_numberOfPacketsInBuffer;
    list<XsDataPacket> m_packetBuffer;
};



int initIMU(CallbackHandler& callback);
int imuGet(CallbackHandler &callback, IMUState &imuData);


#endif