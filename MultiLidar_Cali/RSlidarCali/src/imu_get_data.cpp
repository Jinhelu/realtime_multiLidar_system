#include "imu_get_data.h"
#include <unistd.h>
#include <sys/time.h>

Journaller* gJournal = 0;

int initIMU(CallbackHandler& callback){
    //Journaller* gJournal = 0;
    //cout << "Creating XsControl object..." << endl;
    XsControl* control = XsControl::construct();
    assert(control != 0);

// Lambda function for error handling
    auto handleError = [=](string errorString)
    {
        control->destruct();
        cout << errorString << endl;
        cout << "Press [ENTER] to continue." << endl;
        cin.get();
        return -1;
    };

//cout << "Scanning for devices..." << endl;
    XsPortInfoArray portInfoArray = XsScanner::scanPorts();

// Find an MTi device
    XsPortInfo mtPort;
    for (auto const &portInfo : portInfoArray){
        if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig()){
            mtPort = portInfo;
            break;
        }
    }

    if (mtPort.empty())
        return handleError("No MTi device found. Aborting.");

//cout << "Found a device with ID: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << endl;

//cout << "Opening port..." << endl;
    if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
        return handleError("Could not open port. Aborting.");

// Get the device object
    XsDevice* device = control->device(mtPort.deviceId());
    assert(device != 0);

//cout << "Device: " << device->productCode().toStdString() << ", with ID: " << device->deviceId().toString() << " opened." << endl;

// Create and attach callback handler to device
    //CallbackHandler callback;
    device->addCallbackHandler(&callback);

// Put the device into configuration mode before configuring the device
//cout << "Putting device into configuration mode..." << endl;
    if (!device->gotoConfig())
        return handleError("Could not put device into configuration mode. Aborting.");

    cout << "Configuring the device..." << endl;

// Important for Public XDA!
// Call this function if you want to record a mtb file:
    device->readEmtsAndDeviceConfiguration();

    XsOutputConfigurationArray configArray;
    configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
    configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));

    //configArray.push_back(XsOutputConfiguration(XDI_DeltaV, 0));//速度变化值
    //configArray.push_back(XsOutputConfiguration(XDI_DeltaQ, 0));//角度变化值

    if (device->deviceId().isImu())
    {
        cout << "*************isImu*********************" << endl;
        configArray.push_back(XsOutputConfiguration(XDI_DeltaV, 0));//速度变化值
        configArray.push_back(XsOutputConfiguration(XDI_DeltaQ, 0));//角度变化值
        configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 0));
        configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 0));
        configArray.push_back(XsOutputConfiguration(XDI_Acceleration,400));
    }
    else if (device->deviceId().isVru() || device->deviceId().isAhrs())
    {
        //AHRS:航姿参考系统，由加速度计、磁场计、陀螺仪组成
        //VRU：动态倾角单元
        cout << "*************isVru  isAhrs********************" << endl;
        configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 0));
    }
    else if (device->deviceId().isGnss())
    {
        cout << "*************isGnss********************" << endl;


        configArray.push_back(XsOutputConfiguration(XDI_Acceleration,0));
        configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 0));
        configArray.push_back(XsOutputConfiguration(XDI_LatLon, 0));
        configArray.push_back(XsOutputConfiguration(XDI_AltitudeEllipsoid, 0));
        configArray.push_back(XsOutputConfiguration(XDI_VelocityXYZ, 0));

        configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 0));
        configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 0));
        configArray.push_back(XsOutputConfiguration(XDI_Acceleration,0));
    }
    else
    {
        cout << "*************UNknow********************" << endl;
        return handleError("Unknown device while configuring. Aborting.");
    }

    if (!device->setOutputConfiguration(configArray))
        return handleError("Could not configure MTi device. Aborting.");

    /*输出.mtb文件部分暂时注释掉
	cout << "Creating a log file..." << endl;
	string logFileName = "logfile.mtb";
	if (device->createLogFile(logFileName) != XRV_OK)
		return handleError("Failed to create a log file. Aborting.");
	else
		cout << "Created a log file: " << logFileName.c_str() << endl;
	*/

//cout << "Putting device into measurement mode..." << endl;
    if (!device->gotoMeasurement())
        return handleError("Could not put device into measurement mode. Aborting.");

//cout << "Starting recording..." << endl;
    if (!device->startRecording())
        return handleError("Failed to start recording. Aborting.");

    cout << "\nMain loop. Recording data for 10 seconds." << endl;
    cout << string(79, '-') << endl;

    sleep(2);//弃用前三秒数据
}

int imuGet(CallbackHandler &callback, IMUState &imuData)
{
    //int64_t startTime = XsTime::timeStampNow();
//while (XsTime::timeStampNow() - startTime <= 10000)
//{
    if (callback.packetAvailable())
    {

        cout <<  setw(4) << fixed << setprecision(4);//精度位数和保留小数位数
        imuData.timeStamp_ms = XsTime::timeStampNow();

        //获取此时的PC时间
        struct timeval tv;
        gettimeofday(&tv,NULL);
        imuData.PCtime_us = tv.tv_sec*1000000 + tv.tv_usec;

        // Retrieve a packet
        XsDataPacket packet = callback.getNextPacket();
        if (packet.containsCalibratedData())
        {

            XsVector acc = packet.calibratedAcceleration();
            imuData.acceleration = { acc[0],acc[1],acc[2]};//加速度：m/s*s


            XsVector gyr = packet.calibratedGyroscopeData();
            imuData.angularVelo = { gyr[0],gyr[1],gyr[2]};//陀螺仪单位：rad/s

            /*
          cout << "\r"
            << "Acc X:" << acc[0]
            << ", Acc Y:" << acc[1]
            << ", Acc Z:" << acc[2];//加速度：m/s*s

          XsVector gyr = packet.calibratedGyroscopeData();
          cout << " |Gyr X:" << gyr[0]
               << ", Gyr Y:" << gyr[1]
               << ", Gyr Z:" << gyr[2];//陀螺仪单位：rad/s


          XsVector mag = packet.calibratedMagneticField();
          cout << " |Mag X:" << mag[0]
            << ", Mag Y:" << mag[1]
            << ", Mag Z:" << mag[2];//磁场单位：a.u.
             */
        }

        if (packet.containsOrientation())
        {
            XsQuaternion quaternion = packet.orientationQuaternion();
            imuData.Quaternion = {quaternion.w(), quaternion.x(),quaternion.y(),quaternion.z()};
            /*
            cout << "\r"
              << "q0:" << quaternion.w()
              << ", q1:" << quaternion.x()
              << ", q2:" << quaternion.y()
              << ", q3:" << quaternion.z();
            */

            XsEuler euler = packet.orientationEuler();//z-y-x欧拉角
            imuData.roll= euler.roll();
            imuData.pitch = euler.pitch();
            imuData.yaw = euler.yaw();

            /*
            cout << " |Roll:" << imuData.roll
              << ", Pitch:" << imuData.pitch
              << ", Yaw:" << imuData.yaw <<endl;//欧拉角：deg
              */
            /*
            cout << " |Roll:" << euler.roll()
              << ", Pitch:" << euler.pitch()
              << ", Yaw:" << euler.yaw();//欧拉角：deg
              */

        }

        if (packet.containsLatitudeLongitude())
        {
            XsVector latLon = packet.latitudeLongitude();
            cout << " |Lat:" << latLon[0]
                 << ", Lon:" << latLon[1];
            //latitude 维度/longtitude 经度 单位：deg
        }

        if (packet.containsAltitude())
            cout << " |Alt:" << packet.altitude();//海拔单位：m

        if (packet.containsVelocity())
        {
            XsVector vel = packet.velocity(XDI_CoordSysEnu);
            imuData.velocity = {vel[0],vel[1],vel[2]};
            /*
          cout << " |E:" << vel[0]
            << ", N:" << vel[1]
            << ", U:" << vel[2];
            //E：东 N：北 U：高 速度单位：m/s
             */
        }



        cout << flush;//将缓冲区的数据全部输出到终端
    }
    //XsTime::msleep(0);
//}

    /*
	cout << "\n" << string(79, '-') << "\n";
	cout << endl;\
     */
/*
cout << "Stopping recording..." << endl;
if (!device->stopRecording())
  return handleError("Failed to stop recording. Aborting.");

cout << "Closing log file..." << endl;
if (!device->closeLogFile())
  return handleError("Failed to close log file. Aborting.");
*/

    /*
	//cout << "Stopping recording..." << endl;
	if (!device->stopRecording())
		return handleError("Failed to stop recording. Aborting.");
	//cout << "Closing port..." << endl;
	control->closePort(mtPort.portName().toStdString());

	//cout << "Freeing XsControl object..." << endl;
	control->destruct();

	//cout << "Successful exit." << endl;

	cout << "Press [ENTER] to continue." << endl;
	cin.get();
     */
    return 1;
}

/*
int main(void)
{
  CallbackHandler callback;
  initIMU(callback);
	IMUState imuData;
  while(true){
    imuGet(callback, imuData);
    cout << "Q: " << imuData.Quaternion.w() << ", "
         << imuData.Quaternion.x() << ", "<< imuData.Quaternion.y() << ", "<< imuData.Quaternion.z()  <<endl;
  }

	return 0;
}
 */