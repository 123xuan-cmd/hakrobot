#ifndef CAMERA_HPP
#define CAMERA_HPP
#include "ros/ros.h"
#include <stdio.h>
#include <pthread.h>
#include <opencv2/opencv.hpp>
#include "MvErrorDefine.h"
#include "CameraParams.h"
#include "MvCameraControl.h"

namespace camera
{
//********** define ************************************/
#define MAX_IMAGE_DATA_SIZE (4* 2048 * 3072)

//********** frame ************************************/
    cv::Mat frame;
    //********** frame_empty******************************/
    bool frame_empty = 0;
    //********** mutex ************************************/
    pthread_mutex_t mutex;

    //********** CameraProperties config ************************************/
    enum CamerProperties
    {
        CAP_PROP_FRAMERATE_ENABLE,  // 帧数可调
        CAP_PROP_FRAMERATE,         // 帧数
        CAP_PROP_BURSTFRAMECOUNT,   // 外部一次触发帧数
        CAP_PROP_HEIGHT,            // 图像高度
        CAP_PROP_WIDTH,             // 图像宽度
        CAP_PROP_EXPOSURE_TIME,     // 曝光时间
        CAP_PROP_GAMMA_ENABLE,      // 伽马因子可调
        CAP_PROP_GAMMA,             // 伽马因子
        CAP_PROP_GAIN,              // ✅ 直接手动增益，去掉GainAuto
        CAP_PROP_SATURATION_ENABLE, // 饱和度可调
        CAP_PROP_SATURATION,        // 饱和度
        CAP_PROP_OFFSETX,           // X偏置
        CAP_PROP_OFFSETY,           // Y偏置
        CAP_PROP_TRIGGER_MODE,      // 外部触发
        CAP_PROP_TRIGGER_SOURCE,    // 触发源
        CAP_PROP_LINE_SELECTOR      // 触发线
    };

    //^ *********************************************************************************** //
    //^ ********************************** Camera Class*********************************** //
    //^ *********************************************************************************** //
    class Camera
    {
    public:
        Camera(ros::NodeHandle &node);
        ~Camera();
        static void *HKWorkThread(void *p_handle);
        bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
        bool set(camera::CamerProperties type, float value);
        bool reset();
        void ReadImg(cv::Mat &image);

    private:
        void *handle;
        pthread_t nThreadID;
        int nRet;
        int width;
        int height;
        int Offset_x;
        int Offset_y;
        bool FrameRateEnable;
        int FrameRate;
        int BurstFrameCount;
        int ExposureTime;
        bool GammaEnable;
        float Gamma;
        float Gain;             // ✅ 只保留手动增益
        bool SaturationEnable;
        int Saturation;
        int TriggerMode;
        int TriggerSource;
        int LineSelector;
    };

    //^ *********************************************************************************** //
    //^ ********************************** Camera constructor ***************************** //
    //^ *********************************************************************************** //
    Camera::Camera(ros::NodeHandle &node)
    {
        handle = NULL;

        //********** 读取yaml参数**********//
        node.param("width",           width,           3072);
        node.param("height",          height,          2048);
        node.param("FrameRateEnable", FrameRateEnable, false);
        node.param("FrameRate",       FrameRate,       10);
        node.param("BurstFrameCount", BurstFrameCount, 10);
        node.param("ExposureTime",    ExposureTime,    150000); // ✅ 默认150ms，比原来50ms更亮
        node.param("GammaEnable",     GammaEnable,     false);
        node.param("Gamma",           Gamma,           (float)0.7);
        node.param("Gain",            Gain,            (float)15.0); // ✅ 手动增益默认15.0
        node.param("SaturationEnable",SaturationEnable,false);       // ✅ 默认关闭，避免报错
        node.param("Saturation",      Saturation,      128);
        node.param("Offset_x",        Offset_x,        0);
        node.param("Offset_y",        Offset_y,        0);
        node.param("TriggerMode",     TriggerMode,     1);
        node.param("TriggerSource",   TriggerSource,   2);
        node.param("LineSelector",    LineSelector,2);

        //********** 枚举设备 **********//
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        unsigned int nIndex = 0;
        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)break;
                PrintDeviceInfo(pDeviceInfo);
            }
        }
        else
        {
            printf("Find No Devices!\n");
            exit(-1);
        }

        //********** 选择设备（支持IP选择）**********//
        std::string camera_ip;
        node.param<std::string>("camera_ip", camera_ip, "");
        if (!camera_ip.empty())
        {
            bool found = false;
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE)
                {
                    unsigned int nIp = pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp;
                    char ip_str[16];
                    sprintf(ip_str, "%d.%d.%d.%d",(nIp >> 24) & 0xFF,
                            (nIp >> 16) & 0xFF,
                            (nIp >> 8)  & 0xFF,nIp        & 0xFF);
                    if (camera_ip == ip_str)
                    {
                        nIndex = i;
                        found = true;
                        printf("Found camera with IP: %s\n", ip_str);
                        break;
                    }
                }
            }
            if (!found)
                printf("Camera with IP %s not found! Using first device.\n", camera_ip.c_str());
        }

        //********** 创建句柄 **********//
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        //********** 打开设备 **********//
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        //********** ✅ 先强制关闭自动增益，再设手动Gain **********//
        nRet = MV_CC_SetEnumValue(handle, "GainAuto", 0);
        if (MV_OK == nRet)
            printf("Set GainAuto=Off OK!\n");
        else
            printf("Set GainAuto=Off Failed! nRet=[%x], try continue...\n", nRet);

        //********** 设置yaml参数 **********//
        this->set(CAP_PROP_FRAMERATE_ENABLE, FrameRateEnable);
        if (FrameRateEnable)
            this->set(CAP_PROP_FRAMERATE, FrameRate);
        this->set(CAP_PROP_HEIGHT,height);
        this->set(CAP_PROP_WIDTH,         width);
        this->set(CAP_PROP_OFFSETX,       Offset_x);
        this->set(CAP_PROP_OFFSETY,       Offset_y);
        this->set(CAP_PROP_EXPOSURE_TIME, ExposureTime);
        this->set(CAP_PROP_GAMMA_ENABLE,  GammaEnable);
        if (GammaEnable)
            this->set(CAP_PROP_GAMMA, Gamma);
        this->set(CAP_PROP_GAIN, Gain); // ✅ 直接设手动增益

        //********** ✅ 白平衡：失败不退出，只打印警告 **********//
        nRet = MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", 0);
        if (MV_OK == nRet)
            printf("Set BalanceWhiteAuto=Off OK!\n");
        else
            printf("Set BalanceWhiteAuto Failed! nRet=[%x], skip...\n", nRet); // ✅ 不影响运行

        //********** ✅ 饱和度：失败不退出，只打印警告 **********//
        if (SaturationEnable){
            this->set(CAP_PROP_SATURATION_ENABLE, SaturationEnable);
            this->set(CAP_PROP_SATURATION, Saturation);
        }
        else
        {
            printf("SaturationEnable=false, skip saturation setting.\n"); // ✅ 直接跳过
        }

        //********** 软件触发模式 **********//
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK == nRet)
            printf("set TriggerMode OK!\n");
        else
            printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);

        //********** 读取当前图像格式 **********//
        MVCC_ENUMVALUE t = {0};
        nRet = MV_CC_GetEnumValue(handle, "PixelFormat", &t);
        if (MV_OK == nRet)
            printf("PixelFormat: %d\n", t.nCurValue);
        else
            printf("get PixelFormat fail! nRet [%x]\n", nRet);

        //********** 开始取流 **********//
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        //********** 初始化互斥量 **********//
        nRet = pthread_mutex_init(&mutex, NULL);
        if (nRet != 0)
        {
            perror("pthread_mutex_init failed\n");
            exit(-1);
        }

        //********** 创建工作线程 **********//
        nRet = pthread_create(&nThreadID, NULL, HKWorkThread, handle);
        if (nRet != 0)
        {
            printf("thread create failed. ret = %d\n", nRet);
            exit(-1);
        }
    }

    //^ *********************************************************************************** //
    //^ ********************************** Camera destructor ****************************** //
    //^ *********************************************************************************** //
    Camera::~Camera()
    {
        int nRet;
        pthread_join(nThreadID, NULL);

        nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_StopGrabbing succeed.\n");

        nRet = MV_CC_CloseDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_CloseDevice succeed.\n");

        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_DestroyHandle succeed.\n");

        pthread_mutex_destroy(&mutex);
    }

    //^ *********************************************************************************** //
    //^ ********************************** Camera set ************************************* //
    //^ *********************************************************************************** //
    bool Camera::set(CamerProperties type, float value)
    {
        switch (type)
        {case CAP_PROP_FRAMERATE_ENABLE:{
            nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", value);
            if (MV_OK == nRet)
                printf("set AcquisitionFrameRateEnable OK! value=%f\n", value);
            else
                printf("Set AcquisitionFrameRateEnable Failed! nRet = [%x]\n\n", nRet);
            break;
        }
        case CAP_PROP_FRAMERATE:
        {
            nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", value);
            if (MV_OK == nRet)
                printf("set AcquisitionFrameRate OK! value=%f\n", value);
            else
                printf("Set AcquisitionFrameRate Failed! nRet = [%x]\n\n", nRet);
            break;
        }
        case CAP_PROP_BURSTFRAMECOUNT:
        {
            nRet = MV_CC_SetIntValue(handle, "AcquisitionBurstFrameCount", value);
            if (MV_OK == nRet)
                printf("set AcquisitionBurstFrameCount OK!\n");
            else
                printf("Set AcquisitionBurstFrameCount Failed! nRet = [%x]\n\n", nRet);
            break;
        }
        case CAP_PROP_HEIGHT:
        {
            nRet = MV_CC_SetIntValue(handle, "Height", value);
            if (MV_OK == nRet)
                printf("set Height OK!\n");
            else
                printf("Set Height Failed! nRet = [%x]\n\n", nRet);
            break;
        }
        case CAP_PROP_WIDTH:
        {
            nRet = MV_CC_SetIntValue(handle, "Width", value);
            if (MV_OK == nRet)
                printf("set Width OK!\n");
            else
                printf("Set Width Failed! nRet = [%x]\n\n", nRet);
            break;
        }
        case CAP_PROP_OFFSETX:
        {
            nRet = MV_CC_SetIntValue(handle, "OffsetX", value);
            if (MV_OK == nRet)
                printf("set Offset X OK!\n");
            else
                printf("Set Offset X Failed! nRet = [%x]\n\n", nRet);
            break;
        }
        case CAP_PROP_OFFSETY:
        {
            nRet = MV_CC_SetIntValue(handle, "OffsetY", value);
            if (MV_OK == nRet)
                printf("set Offset Y OK!\n");
            else
                printf("Set Offset Y Failed! nRet = [%x]\n\n", nRet);
            break;
        }
        case CAP_PROP_EXPOSURE_TIME:
        {
            nRet = MV_CC_SetFloatValue(handle, "ExposureTime", value);
            if (MV_OK == nRet)
                printf("set ExposureTime OK! value=%f\n", value);
            else
                printf("Set ExposureTime Failed! nRet = [%x]\n\n", nRet);
            break;
        }
        case CAP_PROP_GAMMA_ENABLE:
        {
            nRet = MV_CC_SetBoolValue(handle, "GammaEnable", value);
            if (MV_OK == nRet)
                printf("set GammaEnable OK! value=%f\n", value);
            else
                printf("Set GammaEnable Failed! nRet = [%x]\n\n", nRet);
            break;
        }
        case CAP_PROP_GAMMA:
        {
            nRet = MV_CC_SetFloatValue(handle, "Gamma", value);
            if (MV_OK == nRet)
                printf("set Gamma OK! value=%f\n", value);
            else
                printf("Set Gamma Failed! nRet = [%x]\n\n", nRet);
            break;
        }
        case CAP_PROP_GAIN:
        {
            // ✅ 直接设置手动增益，使用FloatValue
            nRet = MV_CC_SetFloatValue(handle, "Gain", value);
            if (MV_OK == nRet)
                printf("set Gain OK! value=%f\n", value);
            else
                printf("Set Gain Failed! nRet = [%x]\n\n", nRet);
            break;
        }
        case CAP_PROP_SATURATION_ENABLE:
        {
            nRet = MV_CC_SetBoolValue(handle, "SaturationEnable", value);
            if (MV_OK == nRet)
                printf("set SaturationEnable OK! value=%f\n", value);
            else
                printf("Set SaturationEnable Failed! nRet = [%x], skip...\n\n", nRet); // ✅ 不退出
            break;
        }
        case CAP_PROP_SATURATION:
        {
            nRet = MV_CC_SetIntValue(handle, "Saturation", value);
            if (MV_OK == nRet)
                printf("set Saturation OK! value=%f\n", value);
            else
                printf("Set Saturation Failed! nRet = [%x], skip...\n\n", nRet); // ✅ 不退出
            break;
        }
        case CAP_PROP_TRIGGER_MODE:
        {
            nRet = MV_CC_SetEnumValue(handle, "TriggerMode", value);
            if (MV_OK == nRet)
                printf("set TriggerMode OK!\n");
            else
                printf("Set TriggerMode Failed! nRet = [%x]\n\n", nRet);
            break;
        }
        case CAP_PROP_TRIGGER_SOURCE:
        {
            nRet = MV_CC_SetEnumValue(handle, "TriggerSource", value);
            if (MV_OK == nRet)
                printf("set TriggerSource OK!\n");
            else
                printf("Set TriggerSource Failed! nRet = [%x]\n\n", nRet);
            break;
        }
        case CAP_PROP_LINE_SELECTOR:
        {
            nRet = MV_CC_SetEnumValue(handle, "LineSelector", value);
            if (MV_OK == nRet)
                printf("set LineSelector OK!\n");
            else
                printf("Set LineSelector Failed! nRet = [%x]\n\n", nRet);
            break;
        }
        default:
            return 0;
        }
        return nRet;
    }

    //^ *********************************************************************************** //
    //^ ********************************** Camera reset *********************************** //
    //^ *********************************************************************************** //
    bool Camera::reset()
    {
        nRet = this->set(CAP_PROP_FRAMERATE_ENABLE, FrameRateEnable);
        nRet = this->set(CAP_PROP_FRAMERATE,FrameRate)|| nRet;
        nRet = this->set(CAP_PROP_HEIGHT,           height)          || nRet;
        nRet = this->set(CAP_PROP_WIDTH,            width)           || nRet;
        nRet = this->set(CAP_PROP_OFFSETX,          Offset_x)        || nRet;
        nRet = this->set(CAP_PROP_OFFSETY,          Offset_y)        || nRet;
        nRet = this->set(CAP_PROP_EXPOSURE_TIME,    ExposureTime)    || nRet;
        nRet = this->set(CAP_PROP_GAMMA_ENABLE,     GammaEnable)     || nRet;
        if (GammaEnable)
            nRet = this->set(CAP_PROP_GAMMA,        Gamma)           || nRet;
        nRet = this->set(CAP_PROP_GAIN,Gain)            || nRet; // ✅ 只用手动Gain
        if (SaturationEnable){
            nRet = this->set(CAP_PROP_SATURATION_ENABLE, SaturationEnable) || nRet;
            nRet = this->set(CAP_PROP_SATURATION,        Saturation)       || nRet;
        }
        nRet = this->set(CAP_PROP_TRIGGER_MODE,     TriggerMode)     || nRet;
        nRet = this->set(CAP_PROP_TRIGGER_SOURCE,   TriggerSource)   || nRet;
        nRet = this->set(CAP_PROP_LINE_SELECTOR,    LineSelector)    || nRet;
        return nRet;
    }

    //^ *********************************************************************************** //
    //^ ********************************** PrintDeviceInfo ******************************** //
    //^ *********************************************************************************** //
    bool Camera::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
    {
        if (NULL == pstMVDevInfo)
        {
            printf("%s\n", "The Pointer of pstMVDevInfoList is NULL!");
            return false;
        }
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            printf("%s %x\n","nCurrentIp:",pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp);
            printf("%s %s\n\n","chUserDefinedName:", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
        {
            printf("UserDefinedName:%s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        }
        else
        {
            printf("Not support.\n");
        }
        return true;
    }

    //^ *********************************************************************************** //
    //^ ********************************** ReadImg **************************************** //
    //^ *********************************************************************************** //
    void Camera::ReadImg(cv::Mat &image)
    {
        pthread_mutex_lock(&mutex);
        if (frame_empty)
        {
            image = cv::Mat();}
        else
        {
            image = camera::frame.clone();
            frame_empty = 1;
        }
        pthread_mutex_unlock(&mutex);
    }

    //^ *********************************************************************************** //
    //^ ********************************** HKWorkThread *********************************** //
    //^ *********************************************************************************** //
    void *Camera::HKWorkThread(void *p_handle)
    {
        double start;
        int nRet;
        unsigned char *m_pBufForDriver= (unsigned char *)malloc(sizeof(unsigned char) * MAX_IMAGE_DATA_SIZE);
        unsigned char *m_pBufForSaveImage = (unsigned char *)malloc(MAX_IMAGE_DATA_SIZE);
        MV_FRAME_OUT_INFO_EX stImageInfo  = {0};
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
        cv::Mat tmp;
        int image_empty_count = 0;

        while (ros::ok())
        {
            start = static_cast<double>(cv::getTickCount());
            nRet = MV_CC_GetOneFrameTimeout(p_handle, m_pBufForDriver, MAX_IMAGE_DATA_SIZE, &stImageInfo, 3000);
            if (nRet != MV_OK)
            {
                if (++image_empty_count > 100)
                    ROS_INFO("The Number of Failed Reading Exceed The Set Value!\n");
                continue;
            }
            image_empty_count = 0;

            //********** 转换图像格式为BGR8 **********//
            stConvertParam.nWidth= stImageInfo.nWidth;
            stConvertParam.nHeight         = stImageInfo.nHeight;
            stConvertParam.pSrcData        = m_pBufForDriver;
            stConvertParam.nSrcDataLen     = MAX_IMAGE_DATA_SIZE;
            stConvertParam.enDstPixelType  = PixelType_Gvsp_BGR8_Packed;
            stConvertParam.pDstBuffer      = m_pBufForSaveImage;
            stConvertParam.nDstBufferSize  = MAX_IMAGE_DATA_SIZE;
            stConvertParam.enSrcPixelType  = stImageInfo.enPixelType;
            MV_CC_ConvertPixelType(p_handle, &stConvertParam);

            pthread_mutex_lock(&mutex);
            camera::frame = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, m_pBufForSaveImage).clone();
            frame_empty = 0;
            pthread_mutex_unlock(&mutex);

            double time = ((double)cv::getTickCount() - start) / cv::getTickFrequency();
            // std::cout << "HK_camera,Time:" << time << "\tFPS:" << 1/time << std::endl;
        }
        free(m_pBufForDriver);
        free(m_pBufForSaveImage);
        return 0;
    }

} // namespace camera
#endif