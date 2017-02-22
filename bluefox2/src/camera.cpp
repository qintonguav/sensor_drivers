#include "camera.h"

namespace bluefox2
{

void Camera::callback(bluefox2::on_offConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %d", config.camera_cover);
    for (int i = 0; i < cam_cnt; i++, config.camera_cover >>= 1)
    {
        if (config.camera_cover & 1)
            on_off[i] = true;
        else
            on_off[i] = false;
    }
}

Camera::Camera(ros::NodeHandle _comm_nh, ros::NodeHandle _param_nh) : node(_comm_nh), pnode(_param_nh)
{
    pnode.param("use_color", use_color, false);
    pnode.param("use_hdr", use_hdr, true);
    pnode.param("has_hdr", has_hdr, true);
    pnode.param("use_binning", use_binning, false);
    pnode.param("use_auto_exposure", use_auto_exposure, false);

    pnode.param("fps", fps, 30.0);
    pnode.param("gain", gain, 5.0);

    pnode.param("cam_cnt", cam_cnt, 0);
    serial.resize(cam_cnt);
    on_off.resize(cam_cnt);
    ids.resize(cam_cnt);
    ids_inv.resize(cam_cnt);
    exposure_time_us.resize(cam_cnt);
    for (int i = 0; i < cam_cnt; i++)
    {
        on_off[i] = false;
        pnode.param(std::string("exposure_time_us") + char('a' + i), exposure_time_us[i], 10000);
        pnode.param(std::string("serial") + char('a' + i), serial[i], std::string(""));
    }

    pnode.param("pub_cnt", pub_cnt, 0);
    pub_img.resize(pub_cnt);
    masks.resize(pub_cnt);
    img_buf.resize(pub_cnt);
    for (int i = 0; i < pub_cnt; i++)
    {
        pnode.param(std::string("mask") + char('0' + i), masks[i], std::string(""));
        pub_img[i] = node.advertise<sensor_msgs::Image>(masks[i], 10);
    }

    // Count cameras
    devCnt = devMgr.deviceCount();
    ROS_INFO("Camera Cnt:  %d", devCnt);

    // Init cameras
    ok = true;
    for (int i = 0; i < cam_cnt; i++)
        for (unsigned int k = 0; k < devCnt; k++)
            if (devMgr[k]->serial.read() == serial[i])
            {
                ids[i] = k;
                ids_inv[k] = i;
                if (!initSingleMVDevice(k))
                    ok = false;
            }
    if (!ok)
        ROS_ERROR("Camera Init Failed.");
}

Camera::~Camera()
{
    for (int i = 0; i < cam_cnt; i++)
    {
        fi[ids[i]]->imageRequestReset(0, 0);
        devMgr[ids[i]]->close();
    }
    ok = false;
}

bool Camera::isOK()
{
    return ok;
}

bool Camera::initSingleMVDevice(unsigned int id)
{
    ROS_INFO("Camera Found:  %s(%s)", devMgr[id]->family.read().c_str(), devMgr[id]->serial.read().c_str());

    try
    {
        devMgr[id]->open();
    }
    catch (const mvIMPACT::acquire::ImpactAcquireException &e)
    {
        std::cout << "An error occurred while opening the device " << devMgr[id]->serial.read()
                  << "(error code: " << e.getErrorCode() << "(" << e.getErrorCodeAsString() << "))."
                  << std::endl
                  << "Press [ENTER] to end the application..." << std::endl;
        return false;
    }

    try
    {
        fi[id] = new mvIMPACT::acquire::FunctionInterface(devMgr[id]);
    }
    catch (const mvIMPACT::acquire::ImpactAcquireException &e)
    {
        std::cout << "An error occurred while creating the function interface on device " << devMgr[id]->serial.read()
                  << "(error code: " << e.getErrorCode() << "(" << e.getErrorCodeAsString() << "))."
                  << std::endl
                  << "Press [ENTER] to end the application..." << std::endl;
        return false;
    }

    try
    {
        statistics[id] = new mvIMPACT::acquire::Statistics(devMgr[id]);
    }
    catch (const mvIMPACT::acquire::ImpactAcquireException &e)
    {
        std::cout << "An error occurred while initializing the statistical information on device " << devMgr[id]->serial.read()
                  << "(error code: " << e.getErrorCode() << "(" << e.getErrorCodeAsString() << "))."
                  << std::endl
                  << "Press [ENTER] to end the application..." << std::endl;
        return false;
    }

    // Set Properties
    mvIMPACT::acquire::SettingsBlueFOX settings(devMgr[id]); // Using the "Base" settings (default)

    // Binning
    if (use_binning)
    {
        settings.cameraSetting.binningMode.write(cbmBinningHV);
        ROS_INFO("2X Binning");
    }
    else
    {
        ROS_INFO("No Binning");
    }

    // Gain
    settings.cameraSetting.autoGainControl.write(agcOff);
    if (gain >= 0.0)
    {
        settings.cameraSetting.gain_dB.write(gain);
        ROS_INFO("Gain:  %f", gain);
    }
    else
    {
        settings.cameraSetting.autoGainControl.write(agcOn);
        ROS_INFO("Auto Gain");
    }

    // Auto exposure, modified controller for better results, be careful about the minimum exposure time
    if (use_auto_exposure)
    {
        settings.cameraSetting.autoControlParameters.controllerSpeed.write(acsUserDefined);
        settings.cameraSetting.autoControlParameters.controllerGain.write(0.5);
        settings.cameraSetting.autoControlParameters.controllerIntegralTime_ms.write(100);
        settings.cameraSetting.autoControlParameters.controllerDerivativeTime_ms.write(0.0001);
        settings.cameraSetting.autoControlParameters.desiredAverageGreyValue.write(100);
        settings.cameraSetting.autoControlParameters.controllerDelay_Images.write(0);
        settings.cameraSetting.autoControlParameters.exposeLowerLimit_us.write(50);
        settings.cameraSetting.autoControlParameters.exposeUpperLimit_us.write(exposure_time_us[ids[id]]);
        settings.cameraSetting.autoExposeControl.write(aecOn);
        ROS_INFO("Auto Exposure w/ Max Exposure Time (us) :  %d", settings.cameraSetting.autoControlParameters.exposeUpperLimit_us.read());
    }
    else
    {
        settings.cameraSetting.expose_us.write(exposure_time_us[ids[id]]);
        ROS_INFO("Exposure Time (us) :  %d", settings.cameraSetting.expose_us.read());
    }

    // HDR
    if (has_hdr)
    {
        if (use_hdr)
        {
            settings.cameraSetting.getHDRControl().HDRMode.write(cHDRmFixed0);
            settings.cameraSetting.getHDRControl().HDREnable.write(bTrue);
            ROS_INFO("Enable HDR ...");
            ROS_INFO("KneePoint 0:");
            ROS_INFO("  Voltage (mV):      %d", settings.cameraSetting.getHDRControl().getHDRKneePoint(0).HDRControlVoltage_mV.read());
            ROS_INFO("  Parts per Million: %d", settings.cameraSetting.getHDRControl().getHDRKneePoint(0).HDRExposure_ppm.read());
            ROS_INFO("KneePoint 1:");
            ROS_INFO("  Voltage (mV):      %d", settings.cameraSetting.getHDRControl().getHDRKneePoint(1).HDRControlVoltage_mV.read());
            ROS_INFO("  Parts per Million: %d", settings.cameraSetting.getHDRControl().getHDRKneePoint(1).HDRExposure_ppm.read());
        }
        else
        {
            settings.cameraSetting.getHDRControl().HDREnable.write(bFalse);
            ROS_INFO("HDR Off");
        }
    }
    else
    {
        ROS_INFO("No HDR");
    }

    // Color
    if (use_color)
    {
        // RGB image
        settings.imageDestination.pixelFormat.write(idpfBGR888Packed);
        ROS_INFO("Color Images");
    }
    else
    {
        // Raw image
        settings.imageDestination.pixelFormat.write(idpfRaw);
        ROS_INFO("Grayscale/Bayer Images");
    }

    // prefill the capture queue. There can be more then 1 queue for some device, but only one for now
    mvIMPACT::acquire::SystemSettings ss(devMgr[id]);
    ss.requestCount.write(1);

    // Only for stereo, skip if only one camera exists
    if (cam_cnt >= 2)
    {
        if (ids_inv[id] == 0) // Master camera
        {
            ROS_INFO("Set Master Camera\n");
            //settings.cameraSetting.triggerMode.write(ctmOnDemand);
            settings.cameraSetting.flashMode.write(cfmDigout0);
            settings.cameraSetting.flashType.write(cftStandard);
            settings.cameraSetting.flashToExposeDelay_us.write(0);
        }
        else // Slave camera
        {
            ROS_INFO("Set Slave Camera\n");
            settings.cameraSetting.triggerMode.write(ctmOnHighLevel);
            settings.cameraSetting.triggerSource.write(ctsDigIn0);
            settings.cameraSetting.frameDelay_us.write(0);
        }
    }

    return true;
}

void Camera::feedImages()
{
    dynamic_reconfigure::Server<bluefox2::on_offConfig> server;
    dynamic_reconfigure::Server<bluefox2::on_offConfig>::CallbackType f;
    f = boost::bind(&bluefox2::Camera::callback, this, _1, _2);
    //f = boost::bind(&subcallback, _1, _2);
    server.setCallback(f);

    ros::Rate r(fps);
    sensor_msgs::ImagePtr image(new sensor_msgs::Image);
    sensor_msgs::ImagePtr left(new sensor_msgs::Image);
    sensor_msgs::ImagePtr right(new sensor_msgs::Image);
    while (pnode.ok())
    {
        if (grab_image_data())
        {
            for (int i = 0; i < pub_cnt; i++)
            {
                img_buf[i].header.stamp = capture_time;
                img_buf[i].header.frame_id = std::string("image");
                pub_img[i].publish(img_buf[i]);
            }
        }
        ros::spinOnce();
        r.sleep();
    }
}

bool Camera::grab_image_data()
{
    // Request images from both cameras
    for (int i = 0; i < cam_cnt; i++)
        fi[ids[i]]->imageRequestSingle();
    usleep(10000); // necessary short sleep to warm up the camera
    capture_time = ros::Time::now();

    int requestNr[10] = {INVALID_ID, INVALID_ID, INVALID_ID, INVALID_ID, INVALID_ID, INVALID_ID, INVALID_ID, INVALID_ID, INVALID_ID, INVALID_ID};
    for (int i = 0; i < cam_cnt; i++)
        requestNr[ids[i]] = fi[ids[i]]->imageRequestWaitFor(300);

    bool status = true;
    for (int i = 0; i < cam_cnt; i++)
        if (!(fi[ids[i]]->isRequestNrValid(requestNr[ids[i]])))
            status = false;

    if (status)
    {
        int ok_cnt = 0;
        for (int i = 0; i < cam_cnt; i++)
        {
            pRequest[ids[i]] = fi[ids[i]]->getRequest(requestNr[ids[i]]);
            ok_cnt += pRequest[ids[i]]->isOK();
        }

        if (ok_cnt == cam_cnt)
        {
            // Shared properties
            unsigned int Channels = pRequest[ids[0]]->imageChannelCount.read();
            unsigned int Height = pRequest[ids[0]]->imageHeight.read();
            unsigned int Width = pRequest[ids[0]]->imageWidth.read();
            unsigned int Step = Width * Channels;
            unsigned int Data_cnt = Height * Step;
            std::string Encoding = Channels == 1 ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8;

            for (int i = 0; i < pub_cnt; i++)
            {
                int sub_img_cnt = masks[i].length();

                // Set image properties
                img_buf[i].height = Height * sub_img_cnt;
                img_buf[i].width = Width;
                img_buf[i].step = Step;
                img_buf[i].encoding = Encoding;

                // Resize image
                img_buf[i].data.resize(img_buf[i].step * img_buf[i].height);

                // Copy data
                for (unsigned int j = 0; j < sub_img_cnt; j++)
                {
                    int k = masks[i][j] - 'a';
                    if (on_off[k])
                        memset(&img_buf[i].data[Data_cnt * j], 0, Data_cnt);
                    else
                        memcpy(&img_buf[i].data[Data_cnt * j], pRequest[ids[k]]->imageData.read(), Data_cnt);
                }
            }
            // Release capture request
            for (int i = 0; i < cam_cnt; i++)
                fi[ids[i]]->imageRequestUnlock(requestNr[ids[i]]);
            status = true;
        }
        else
        {
            ROS_ERROR("Invalid Image");
            // Clear all image received and reset capture
            for (int i = 0; i < cam_cnt; i++)
                fi[ids[i]]->imageRequestUnlock(requestNr[ids[i]]);
            status = false;
        }
    }
    else
    {
        ROS_ERROR("Invalid Image Request");
        // Clear all image received and reset capture
        for (int i = 0; i < cam_cnt; i++)
            if (fi[ids[i]]->isRequestNrValid(requestNr[ids[i]]))
            {
                pRequest[ids[i]] = fi[ids[i]]->getRequest(requestNr[ids[i]]);
                fi[ids[i]]->imageRequestUnlock(requestNr[ids[i]]);
            }
        status = false;
    }
    return status;
}
}
