#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <opencv2/opencv.hpp>
#include "Error.h"
#include "Thread.h"
#include <memory>
#include <Argus/Argus.h>
#include <Argus/Ext/SensorTimestampTsc.h>
#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>
#include <cv_bridge/cv_bridge.h>
#include <regex>
#include <memory>

#include <nvbuf_utils.h>
#include <NvEglRenderer.h>
//#include <NvJpegEncoder.h>

// add the camerainfo
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/image_encodings.h>
//#include <camera_info_manager/camera_info_manager.h>
#include <argus_multicam/MultiStamps.h>
#include "pipeline.h"
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <iostream>

#include <nvidia_to_ros/nv2ros.h>

using namespace Argus;
using namespace EGLStream;

/* Constants */
static const uint32_t MAX_CAMERA_NUM = 6;
static const uint32_t DEFAULT_FRAME_COUNT = 100;
static const uint32_t DEFAULT_FPS = 24;
static bool DO_STAT = false;
static bool DO_ROS = true;

uint32_t frame_duration = 1e9/DEFAULT_FPS;
int selected_mode = 0;
int ae_top = -1;
int ae_bottom = -1;
int ae_left = -1;
int ae_right = -1;
float exposure_compensation = -1.f;
int exposure_time_min = -1;
int exposure_time_max = -1;
float isp_digital_gain_min = -1.f;
float isp_digital_gain_max = -1.f;
float gain_min = -1.f;
float gain_max = -1.f;
int denoise_mode = -1;
float denoise_strength = -1.f;
int edge_enhance_mode = -1;
float edge_enhance_strength = -1.f;
int image_width = -1;
int image_height = -1;
bool print_time_debug = false;

//static const Size2D<uint32_t>    STREAM_SIZE(1280, 960);
//static const Size2D<uint32_t>    STREAM_SIZE(3440, 1440);
//static const Size2D<uint32_t>    STREAM_SIZE(2448, 2058);
//static const Size2D<uint32_t>    CAPTURE_SIZE(2448, 2058);
static Size2D<uint32_t> STREAM_SIZE(1280, 960); //2448, 2058);
// static const Size2D<uint32_t> STREAM_SIZE(640, 480); //2448, 2058);
// static const Size2D<uint32_t> STREAM_SIZE(816, 686);  //2448, 2058);
static Size2D<uint32_t> CAPTURE_SIZE(1280, 960); //2448, 2058);
// static const Size2D<uint32_t> CAPTURE_SIZE(640, 480); //2448, 2058);

// static const Size2D<uint32_t> CAPTURE_SIZE(816, 686); //2448, 2058);

//Don't use these numbers...but others might work well
//static const Range<uint64_t> EXPOSURE_TIME_RANGE(44000, 1000000);

/* Globals */
UniqueObj<CameraProvider> g_cameraProvider;
uint32_t g_stream_num = MAX_CAMERA_NUM;
uint32_t g_frame_count = DEFAULT_FRAME_COUNT;
bool g_renderscreen = false;
struct timespec systemtime1, systemtime2, monotonic, monotonic_raw;
uint32_t g_frame_number = 0;

cv_bridge::CvImage img_bridge;

std::vector<ros::Publisher> img_pub;
nv2ros::Publisher* publisher;
std::vector<uint32_t> camera_index_vector;
std::vector<int> camera_lookup_index_vector(MAX_CAMERA_NUM, false);;
std::vector<bool> camera_index_to_ros_vector(MAX_CAMERA_NUM, false);
std::vector<bool> camera_index_to_udp_vector(MAX_CAMERA_NUM, false);

std::vector<std::unique_ptr<Pipeline>> gst_pipeline_vector;

// adding here---------------------
std::vector<ros::Publisher> camerainfo_pub;

// ros::Publisher debug_pub;
//camera_info_manager::CameraInfoManager camera_info_manager_;
// ending here------------------------

/* Debug print macros */
#define PRODUCER_PRINT(...) printf("PRODUCER: " __VA_ARGS__)
#define CONSUMER_PRINT(...) printf("CONSUMER: " __VA_ARGS__)
#define JPEG_BUFFER_SIZE (CAPTURE_SIZE.area() * 3 / 2)

namespace ArgusSamples
{

    /* An utility class to hold all resources of one capture session */
    class CaptureHolder : public Destructable
    {
    public:
        explicit CaptureHolder();
        virtual ~CaptureHolder();

        bool initialize(CameraDevice *device);

        CaptureSession *getSession() const
        {
            return m_captureSession.get();
        }

        OutputStream *getStream() const
        {
            return m_outputStream.get();
        }

        Request *getRequest() const
        {
            return m_request.get();
        }

        virtual void destroy()
        {
            delete this;
        }

    private:
        UniqueObj<CaptureSession> m_captureSession;
        UniqueObj<OutputStream> m_outputStream;
        UniqueObj<Request> m_request;
    };

    CaptureHolder::CaptureHolder()
    {
    }

    CaptureHolder::~CaptureHolder()
    {
        /* Destroy the output stream */
        m_outputStream.reset();
    }

    bool CaptureHolder::initialize(CameraDevice *device)
    {
      ICameraProperties* props = interface_cast<ICameraProperties>(device);

      std::vector<SensorMode*> modes;
      props->getAllSensorModes(&modes);
      for(int i = 0; i < modes.size(); i++){
	ISensorMode* sensor_mode = interface_cast<ISensorMode>(modes[i]);
	ROS_INFO_STREAM("sensor mode " << i << ":");
	ROS_INFO_STREAM("\tresolution: " << sensor_mode->getResolution().width() << " x " << sensor_mode->getResolution().height());
	ROS_INFO_STREAM("\texposure time range: " << sensor_mode->getExposureTimeRange().min()
			<< ", " << sensor_mode->getExposureTimeRange().max());
	ROS_INFO_STREAM("\thdr ratio range: " << sensor_mode->getHdrRatioRange().min()
			<< ", " << sensor_mode->getHdrRatioRange().max());
	ROS_INFO_STREAM("\tframe duration range: " << sensor_mode->getFrameDurationRange().min()
			<< ", " << sensor_mode->getFrameDurationRange().max());
	ROS_INFO_STREAM("\tanalog gain range: " << sensor_mode->getAnalogGainRange().min()
			<< ", " << sensor_mode->getAnalogGainRange().max());
	ROS_INFO_STREAM("\tinput bit depth: " << sensor_mode->getInputBitDepth());
	ROS_INFO_STREAM("\toutput bit depth: " << sensor_mode->getOutputBitDepth());
	//ROS_INFO_STREAM("\tsensor mode type: " << sensor_mode->getSensorModeType());
	//ROS_INFO_STREAM("\tbayer phase: " << sensor_mode->getBayerPhase());


	if(i == selected_mode){
	  STREAM_SIZE = sensor_mode->getResolution();
	  CAPTURE_SIZE = sensor_mode->getResolution();
	}
      }

      if(image_width > 0 && image_height > 0){
	STREAM_SIZE = Size2D<uint32_t>(image_width, image_height);
	CAPTURE_SIZE = Size2D<uint32_t>(image_width, image_height);
      }

      
        ICameraProvider *iCameraProvider = interface_cast<ICameraProvider>(g_cameraProvider);
        if (!iCameraProvider)
            ORIGINATE_ERROR("Failed to get ICameraProvider interface");

        /* Create the capture session using the first device and get the core interface */
        m_captureSession.reset(iCameraProvider->createCaptureSession(device));
        ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(m_captureSession);
        IEventProvider *iEventProvider = interface_cast<IEventProvider>(m_captureSession);
        if (!iCaptureSession || !iEventProvider)
            ORIGINATE_ERROR("Failed to create CaptureSession");

        /* Create the OutputStream */
        UniqueObj<OutputStreamSettings> streamSettings(
            iCaptureSession->createOutputStreamSettings(STREAM_TYPE_EGL));
        IEGLOutputStreamSettings *iEglStreamSettings =
            interface_cast<IEGLOutputStreamSettings>(streamSettings);
        if (!iEglStreamSettings)
            ORIGINATE_ERROR("Failed to create EglOutputStreamSettings");

        iEglStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
        //iEglStreamSettings->setEGLDisplay(g_renderer->getEGLDisplay());
        iEglStreamSettings->setResolution(STREAM_SIZE);
        iEglStreamSettings->setMode(Argus::EGL_STREAM_MODE_MAILBOX);
        iEglStreamSettings->setMetadataEnable(true);

        m_outputStream.reset(iCaptureSession->createOutputStream(streamSettings.get()));

        /* Create capture request and enable the output stream */
        m_request.reset(iCaptureSession->createRequest());
        IRequest *iRequest = interface_cast<IRequest>(m_request);
        if (!iRequest)
            ORIGINATE_ERROR("Failed to create Request");
        iRequest->enableOutputStream(m_outputStream.get());

        ISourceSettings *iSourceSettings =
            interface_cast<ISourceSettings>(iRequest->getSourceSettings());
        if (!iSourceSettings)
	  ORIGINATE_ERROR("Failed to get ISourceSettings interface");

	if(selected_mode >= 0)
	  iSourceSettings->setSensorMode(modes[selected_mode]);
	if(frame_duration >= 0)
	  iSourceSettings->setFrameDurationRange(Range<uint64_t>(frame_duration));
	
	if(gain_min >= 0 && gain_max >= 0)
	  iSourceSettings->setGainRange(Range<float>(gain_min, gain_max));
	if(exposure_time_min >= 0 && exposure_time_max >= 0)
	  iSourceSettings->setExposureTimeRange(Range<uint64_t>(exposure_time_min, exposure_time_max));
	//iSourceSettings->setExposureTimeRange(EXPOSURE_TIME_RANGE);
    
	// denoise settings
	IDenoiseSettings *denoiseSettings = interface_cast<IDenoiseSettings>(m_request);
	if (!denoiseSettings)
	  ORIGINATE_ERROR("Failed to get DenoiseSettings interface");
	if(denoise_mode == 0)
	  denoiseSettings->setDenoiseMode(DENOISE_MODE_OFF);
	else if(denoise_mode == 1)
	  denoiseSettings->setDenoiseMode(DENOISE_MODE_FAST);
	else if(denoise_mode == 2)
	  denoiseSettings->setDenoiseMode(DENOISE_MODE_HIGH_QUALITY);
	if(denoise_strength >= 0.f)
	  denoiseSettings->setDenoiseStrength(denoise_strength);

	// auto control settings
	IAutoControlSettings *autoControlSettings = interface_cast<IAutoControlSettings>(iRequest->getAutoControlSettings());
	if (!autoControlSettings)
	  ORIGINATE_ERROR("Failed to get AutoControlSettings interface");
	if(ae_top >= 0 && ae_bottom >= 0 && ae_left >= 0 && ae_right >= 0){
	  std::vector<AcRegion> regions;
	  AcRegion region(ae_left, ae_top, ae_right, ae_bottom, 1.f);
	  regions.push_back(region);
	  autoControlSettings->setAeRegions(regions);
	}
	if(exposure_compensation >= 0.f)
	  autoControlSettings->setExposureCompensation(exposure_compensation);
	if(isp_digital_gain_min >= 0.f && isp_digital_gain_max >= 0.f)
	  autoControlSettings->setIspDigitalGainRange(Range<float>(isp_digital_gain_min, isp_digital_gain_max));

	// edge enhance settings
	IEdgeEnhanceSettings *edgeEnhanceSettings = interface_cast<IEdgeEnhanceSettings>(m_request);
	if (!edgeEnhanceSettings)
	  ORIGINATE_ERROR("Failed to get EdgeEnhanceSettings interface");
	if(edge_enhance_mode == 0)
	  edgeEnhanceSettings->setEdgeEnhanceMode(EDGE_ENHANCE_MODE_OFF);
	else if(edge_enhance_mode == 1)
	  edgeEnhanceSettings->setEdgeEnhanceMode(EDGE_ENHANCE_MODE_FAST);
	else if(edge_enhance_mode == 2)
	  edgeEnhanceSettings->setEdgeEnhanceMode(EDGE_ENHANCE_MODE_HIGH_QUALITY);
	if(edge_enhance_strength >= 0.f)
	  edgeEnhanceSettings->setEdgeEnhanceStrength(edge_enhance_strength);
	
        return true;
    }

    /*
 * Argus Consumer Thread:
 * This is the thread acquires buffers from each stream and composite them to
 * one frame. Finally it renders the composited frame through EGLRenderer.
 */
    class ConsumerThread : public Thread
    {
    public:
        explicit ConsumerThread(std::vector<OutputStream *> &streams) : m_streams(streams),
                                                                        m_framesRemaining(g_frame_count),
                                                                        m_compositedFrame(0),
                                                                        //m_JpegEncoder(NULL),
                                                                        m_OutputBuffer(NULL)
        {
        }
        virtual ~ConsumerThread();

    protected:
        /** @name Thread methods */
        /**@{*/
        virtual bool threadInitialize();
        virtual bool threadExecute();
        virtual bool threadShutdown();
        /**@}*/

        std::vector<OutputStream *> &m_streams;
        uint32_t m_framesRemaining;
        UniqueObj<FrameConsumer> m_consumers[MAX_CAMERA_NUM];
        int m_dmabufs[MAX_CAMERA_NUM];
        NvBufferCompositeParams m_compositeParam;
        int m_compositedFrame;

        bool processV4L2Fd(int32_t fd, uint64_t frameNumber, uint64_t camNumber);
        bool processOpenCV(int32_t fd, uint64_t frameNumber, uint64_t camNumber, const ros::Time &camera_time);

        //NvJPEGEncoder *m_JpegEncoder;
        unsigned char *m_OutputBuffer;
    };

    ConsumerThread::~ConsumerThread()
    {
        if (m_compositedFrame)
            NvBufferDestroy(m_compositedFrame);

        /*if (m_JpegEncoder)
      delete m_JpegEncoder;
    */
        if (m_OutputBuffer)
            delete[] m_OutputBuffer;

        for (uint32_t i = 0; i < m_streams.size(); i++)
            if (m_dmabufs[i])
                NvBufferDestroy(m_dmabufs[i]);
    }

    bool ConsumerThread::threadInitialize()
    {

        // Initialize buffer handles. Buffer will be created by FrameConsumer
        memset(m_dmabufs, 0, sizeof(m_dmabufs));

        // Create the FrameConsumer
        for (uint32_t i = 0; i < m_streams.size(); i++)
        {
            m_consumers[i].reset(FrameConsumer::create(m_streams[i]));
        }

        m_OutputBuffer = new unsigned char[JPEG_BUFFER_SIZE];
        if (!m_OutputBuffer)
            return false;

        /*m_JpegEncoder = NvJPEGEncoder::createJPEGEncoder("jpenenc");
    if (!m_JpegEncoder)
        ORIGINATE_ERROR("Failed to create JPEGEncoder.");

    if (DO_STAT)
        m_JpegEncoder->enableProfiling();
    */
        return true;
    }

    bool ConsumerThread::threadExecute()
    {
        IEGLOutputStream *iEglOutputStreams[MAX_CAMERA_NUM];
        IFrameConsumer *iFrameConsumers[MAX_CAMERA_NUM];

        for (uint32_t i = 0; i < m_streams.size(); i++)
        {
            iEglOutputStreams[i] = interface_cast<IEGLOutputStream>(m_streams[i]);
            iFrameConsumers[i] = interface_cast<IFrameConsumer>(m_consumers[i]);
            if (!iFrameConsumers[i])
                ORIGINATE_ERROR("Failed to get IFrameConsumer interface");

            /* Wait until the producer has connected to the stream */
            CONSUMER_PRINT("Waiting until producer is connected...\n");
            if (iEglOutputStreams[i]->waitUntilConnected() != STATUS_OK)
                ORIGINATE_ERROR("Stream failed to connect.");
            CONSUMER_PRINT("Producer has connected; continuing.\n");
        }

        //while (m_framesRemaining--)
        NV::IImageNativeBuffer *image_buffer[6];
        UniqueObj<Frame> frame_buffer[6];
        auto stream_size = m_streams.size();
        ros::Rate r(50.0);

        uint64_t system_nano, system_nano2, mono_raw_nano, argus_offset_ns;
        int64_t offset_ros_to_monoraw;

        while (ros::ok())
        {
	  //ROS_INFO_STREAM("print: " << print_time_debug);
            r.sleep();
            g_frame_number += 1;
            //printf("Start of frame %u\n", g_frame_number);
            for (uint32_t i = 0; i < camera_index_vector.size(); i++)
            {
                //Status status;
                /* Acquire a frame */
                frame_buffer[i] = UniqueObj<Frame>(iFrameConsumers[i]->acquireFrame());
                IFrame *iFrame = interface_cast<IFrame>(frame_buffer[i]);
                if (!iFrame)
                    break;

                /* Get the IImageNativeBuffer extension interface */
                image_buffer[i] = interface_cast<NV::IImageNativeBuffer>(iFrame->getImage());
                if (!image_buffer[i])
                    ORIGINATE_ERROR("IImageNativeBuffer not supported by Image.");
            }

            clock_gettime(CLOCK_REALTIME, &systemtime1);
            clock_gettime(CLOCK_MONOTONIC_RAW, &monotonic_raw);
            clock_gettime(CLOCK_REALTIME, &systemtime2);

            if(systemtime1.tv_sec == systemtime2.tv_sec && (systemtime2.tv_nsec - systemtime1.tv_nsec)<5000)
            {
                system_nano = systemtime1.tv_sec * 1e9 + systemtime1.tv_nsec;
                system_nano2 = systemtime2.tv_sec * 1e9 + systemtime2.tv_nsec;
                system_nano = (system_nano + system_nano2)/2;
                mono_raw_nano = monotonic_raw.tv_sec * 1e9 + monotonic_raw.tv_nsec;
                offset_ros_to_monoraw = system_nano - mono_raw_nano;

                // offset_ns provided by driver at /sys/devices/system/clocksource/clocksource0/offset_ns 
                FILE * of = fopen("/sys/devices/system/clocksource/clocksource0/offset_ns","r");
                int result = fscanf(of, "%ld", &argus_offset_ns);
                fclose(of);
            }
	    
            for (uint32_t i = 0; i < stream_size; ++i)
            {

                // Print out some capture metadata from the frame.
                IArgusCaptureMetadata *iArgusCaptureMetadata = interface_cast<IArgusCaptureMetadata>(frame_buffer[i]);
                if (!iArgusCaptureMetadata)
                    ORIGINATE_ERROR("Failed to get IArgusCaptureMetadata interface.");
                CaptureMetadata *metadata = iArgusCaptureMetadata->getMetadata();
                ICaptureMetadata *iMetadata = interface_cast<ICaptureMetadata>(metadata);

                // getSensorTimestamp() is start of MIPI packet, but timestamp is wrong
                // Use getSensorSofTimestampTSC instead, accounting for published offset from kernel
                // Get time in middle of frame by subtracting exposure time
                // Convert to ros time by adding in offset of ROS time to monotonic_raw/tsc
                const Ext::ISensorTimestampTsc *sensorTSC = interface_cast<const Ext::ISensorTimestampTsc>(metadata);
                uint64_t cam_to_ros = sensorTSC->getSensorSofTimestampTsc() - argus_offset_ns - iMetadata->getSensorExposureTime() + offset_ros_to_monoraw ;

                ros::Time camera_time;
                camera_time.fromNSec(cam_to_ros);

		if(print_time_debug)
		  ROS_INFO_STREAM(i << " timestamp: " << sensorTSC->getSensorSofTimestampTsc()
				  << " argus offset: " << argus_offset_ns
				  << " exposure time: " << iMetadata->getSensorExposureTime()
				  << " offset_ros: " << offset_ros_to_monoraw
				  << " cam_to_ros: " << cam_to_ros
				  << " ros sec: " << camera_time.sec << " ros nsec: " << camera_time.nsec);

                // if(i==5)
                // {
                //     argus_multicam::MultiStamps ms_msg;

                //     ms_msg.header.stamp.fromNSec(cam_to_ros);
                //     ms_msg.stamp1.fromNSec(ros_time_nano);
                //     ms_msg.stamp2.fromNSec(offset_ros_to_mono);
                //     ms_msg.stamp3.fromNSec(mono_nano);
                //     ms_msg.stamp4.fromNSec(iMetadata->getSensorTimestamp());
                //     debug_pub.publish(ms_msg);
                // }
                //ros::Time camera_time_raw;
                //camera_time_raw.fromNSec(cam_to_rosraw);

                // ROS_INFO_STREAM("i: " << i << ": " << iMetadata->getSensorTimestamp() + iMetadata->getSensorExposureTime());

                if (!iMetadata)
                {
                    printf("Failed to get ICaptureMetadata interface.");
                }
                else
                {
                    //const Ext::ISensorTimestampTsc *iSensorTimestampTsc = interface_cast<const Ext::ISensorTimestampTsc>(metadata);
                    //if (iSensorTimestampTsc){
                    //printf("sof= %lu, eof= %lu \n", iSensorTimestampTsc->getSensorSofTimestampTsc(), iSensorTimestampTsc->getSensorEofTimestampTsc());
                    //} else {
                    //	printf("TSC stamp fail!\n");
                    // }
                    //printf("device %d: Sensor Timestamp: %llu SYSTEM: %llu ROS_NANO:%llu MONO:%llu MONO_RAW:%llu Exposure time:%llu \n",i, static_cast<unsigned long long>(iMetadata->getSensorTimestamp()), static_cast<unsigned long long>(system_nano), static_cast<unsigned long long>(ros_time_nano), static_cast<unsigned long long>(mono_nano), static_cast<unsigned long long>(mono_raw_nano), static_cast<unsigned long long>(iMetadata->getSensorExposureTime()));
                }

                /* If we don't already have a buffer, create one from this image.
               Otherwise, just blit to our buffer */
                if (!m_dmabufs[i])
                {
                    if (!DO_ROS)
                    {
                        /// This is better for Argus only stuff...
                        m_dmabufs[i] = image_buffer[i]->createNvBuffer(iEglOutputStreams[i]->getResolution(),
                                                                       NvBufferColorFormat_YUV420,
                                                                       NvBufferLayout_BlockLinear);
                    }
                    else
                    {
                        /// ROS/OpenCV needs Pitch layout
                        m_dmabufs[i] = image_buffer[i]->createNvBuffer(iEglOutputStreams[i]->getResolution(),
                                                                       NvBufferColorFormat_ABGR32,
                                                                       NvBufferLayout_Pitch);
                    }

                    if (!m_dmabufs[i])
                        CONSUMER_PRINT("\tFailed to create NvBuffer\n");
                }
                else if (image_buffer[i]->copyToNvBuffer(m_dmabufs[i]) != STATUS_OK)
                {
                    ORIGINATE_ERROR("Failed to copy frame to NvBuffer.");
                }

                /* Process frame. */
                processOpenCV(m_dmabufs[i], m_framesRemaining, i, camera_time);
            }

            //CONSUMER_PRINT("Render frame %d\n", g_frame_count - m_framesRemaining);
        }

        CONSUMER_PRINT("Done.\n");

        requestShutdown();

        return true;
    }

    bool ConsumerThread::threadShutdown()
    {
        return true;
    }

    bool ConsumerThread::processOpenCV(int32_t fd, uint64_t frameNumber, uint64_t camNumber, const ros::Time &camera_time)
    {
      //ROS_INFO_STREAM("cam number: " << camNumber << " index: " << camera_index_vector[camNumber] << " ros: " << camera_index_to_ros_vector[camera_index_vector[camNumber]]);
      if (camera_index_to_ros_vector[camera_index_vector[camNumber]]){
	NvBufferParams buf_params;
	NvBufferGetParams (fd, &buf_params);
	NvBufferParamsEx buf_params_ex;
	NvBufferGetParamsEx(fd, &buf_params_ex);

	nv2ros::NvImageMessage img_msg(fd, buf_params, buf_params_ex, camera_time);
	publisher->publish(img_msg, camNumber);
      }
  
      /*
        void *pdata = NULL;

        NvBufferParams buf_params;
        NvBufferGetParams(fd, &buf_params);

        NvBufferMemMap(fd, 0, NvBufferMem_Read, &pdata);
        NvBufferMemSyncForCpu(fd, 0, &pdata);

        cv::Mat imgbuf = cv::Mat(CAPTURE_SIZE.height(),
                                 CAPTURE_SIZE.width(),
                                 CV_8UC4, pdata, buf_params.pitch[0]);
        cv::Mat display_img;
        cvtColor(imgbuf, display_img, cv::COLOR_RGBA2BGR);

        //adding here----------------
        // sensor_msgs::CameraInfo cur_cinfo = camera_info_manager_.getCameraInfo();
        // sensor_msgs::CameraInfoPtr cur_cinfo(new sensor_msgs::CameraInfo(camera_info_manager_.getCameraInfo()));
        sensor_msgs::CameraInfo cinfo;
        // cinfo.reset(new sensor_msgs::CameraInfo(cur_cinfo));

        cinfo.header.stamp = camera_time; //ros::Time::now();

        cinfo.header.frame_id = frameNumber;

        //ending here----------------

        sensor_msgs::Image img_msg; // >> message to be sent
        std_msgs::Header header;    // empty header
        header.stamp = camera_time; //ros::Time::now(); // time
        header.frame_id = frameNumber;
        NvBufferMemUnMap(fd, 0, &pdata);

        if (camera_index_to_udp_vector[camera_index_vector[camNumber]])
        {
            gst_pipeline_vector[camNumber]->write(display_img);
        }

        if (camera_index_to_ros_vector[camera_index_vector[camNumber]])
        {

            img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, display_img);
            img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
            image_pub[camNumber].publish(img_msg);

            //adding here----------------
            camerainfo_pub[camNumber].publish(cinfo);
            //ending here----------------
        }
        else
        {
            ROS_INFO_STREAM("Skip cam!" << camNumber);
        }

        //Don't use this in less you have to...
        //cv::imshow("img", display_img);
        //cv::waitKey(1);
	*/
        return true;
    }

    /*
 * Argus Producer Thread:
 * Open the Argus camera driver and detect how many camera devices available.
 * Create one OutputStream for each camera device. Launch consumer thread
 * and then submit FRAME_COUNT capture requests.
 */
    static bool execute()
    {
        /* Initialize the Argus camera provider */
        g_cameraProvider = UniqueObj<CameraProvider>(CameraProvider::create());
        ICameraProvider *iCameraProvider = interface_cast<ICameraProvider>(g_cameraProvider);
        if (!iCameraProvider)
            ORIGINATE_ERROR("Failed to get ICameraProvider interface");
        printf("Argus Version: %s\n", iCameraProvider->getVersion().c_str());

        /* Get the camera devices */
        std::vector<CameraDevice *> cameraDevices;
        iCameraProvider->getCameraDevices(&cameraDevices);
        if (cameraDevices.size() == 0)
            ORIGINATE_ERROR("No cameras available");

        UniqueObj<CaptureHolder> captureHolders[MAX_CAMERA_NUM];
        uint32_t streamCount = cameraDevices.size() < MAX_CAMERA_NUM ? cameraDevices.size() : MAX_CAMERA_NUM;
        if (streamCount > g_stream_num)
            streamCount = g_stream_num;

        for (const uint32_t &i : camera_index_vector)
        {
            captureHolders[i].reset(new CaptureHolder);
            if (!captureHolders[i].get()->initialize(cameraDevices[i]))
                ORIGINATE_ERROR("Failed to initialize Camera session %d", i);
        }

        std::vector<OutputStream *> streams;
        for (const uint32_t &i : camera_index_vector)
            streams.push_back(captureHolders[i].get()->getStream());

        /* Start the rendering thread */
        ConsumerThread consumerThread(streams);
        PROPAGATE_ERROR(consumerThread.initialize());
        PROPAGATE_ERROR(consumerThread.waitRunning());

        for (const uint32_t &j : camera_index_vector)
        {
            ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(captureHolders[j].get()->getSession());
            Request *request = captureHolders[j].get()->getRequest();

            if (iCaptureSession->repeat(request) != STATUS_OK)
            {
                ORIGINATE_ERROR("Failed to start repeat capture request for preview");
            }
        }

        ros::spin();
        /* Wait for idle */
        for (const uint32_t &i : camera_index_vector)
        {
            ICaptureSession *iCaptureSession =
                interface_cast<ICaptureSession>(captureHolders[i].get()->getSession());
            iCaptureSession->stopRepeat();
            iCaptureSession->waitForIdle();
        }

        ROS_INFO_STREAM("Destroying resources....");
        /* Destroy the capture resources */

        for (const uint32_t &i : camera_index_vector)
        {
            captureHolders[i].reset();
        }

        /* Wait for the rendering thread to complete */
        PROPAGATE_ERROR(consumerThread.shutdown());

        /* Shut down Argus */
        g_cameraProvider.reset();

        return true;
    }

}; /* namespace ArgusSamples */

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "argus_stereo_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    selected_mode = nhPrivate.param("sensor_mode", 1);
    frame_duration = nhPrivate.param("frame_duration", 1e9/DEFAULT_FPS);  
    ae_top = nhPrivate.param("ae_top", -1);
    ae_bottom = nhPrivate.param("ae_bottom", -1);
    ae_left = nhPrivate.param("ae_left", -1);
    ae_right = nhPrivate.param("ae_right", -1);
    exposure_compensation = nhPrivate.param("exposure_compensation", -1);
    exposure_time_min = nhPrivate.param("exposure_time_min", -1.0);
    exposure_time_max = nhPrivate.param("exposure_time_max", -1.0);
    isp_digital_gain_min = nhPrivate.param("isp_digital_gain_min", -1.0);
    isp_digital_gain_max = nhPrivate.param("isp_digital_gain_max", -1.0);
    gain_min = nhPrivate.param("gain_min", -1.0);
    gain_max = nhPrivate.param("gain_max", -1.0);
    denoise_mode = nhPrivate.param("denoise_mode", -1);
    denoise_strength = nhPrivate.param("denoise_strength", -1.0);
    edge_enhance_mode = nhPrivate.param("edge_enhance_mode", -1);
    edge_enhance_strength = nhPrivate.param("edge_enhance_strength", -1.0);
    image_width = nhPrivate.param("image_width", -1);
    image_height = nhPrivate.param("image_height", -1);

    
    // camera_info_manager::CameraInfoManager cinfo(nh, "camera","");
    //camera_info_manager_(cinfo);

    //   camera_info_manager_.setCameraName(camera_name_);

    //  if(camera_info_manager_.validateURL(camera_info_url_)) {
    //       camera_info_manager_.loadCameraInfo(camera_info_url_);
    //       ROS_INFO_STREAM("Loaded camera calibration from "<<camera_info_url_);
    //     } else {
    //       ROS_WARN_STREAM("Camera info at: "<<camera_info_url_<<" not found. Using an uncalibrated config.");
    //     }

    // debug_pub = nhPrivate.advertise<argus_multicam::MultiStamps>("ts_cam0_debug", 100);
    XmlRpc::XmlRpcValue v;

    if (nhPrivate.param("active_cameras", v, v))
    {
      std::vector<std::string> topic_names;
        for (int i = 0; i < v.size(); i++)
        {
	    int cam = (int)v[i];
  	    topic_names.push_back(std::string("nv_image") + std::to_string(cam));
            ROS_INFO_STREAM("Active CAM: " << v[i]);
            camera_index_vector.push_back(cam);
	    camera_lookup_index_vector[cam] = i;
	    camera_index_to_ros_vector[cam] = true;
        }
	publisher = new nv2ros::Publisher(nh, topic_names, 4);
    }
    else
    {
        //If you don't specify a set of cameras, default to MAX_CAMERA_NUM
        ROS_WARN_STREAM("Active cameras not specified...defaulting to MAX " << MAX_CAMERA_NUM);
        for (int i = 0; i < MAX_CAMERA_NUM; i++)
        {
            ROS_INFO_STREAM("Active CAM: " << i);
            camera_index_vector.push_back(i);
        }
    }

    if (nhPrivate.param("cams_to_ros", v, v))
    {
      /*
        for (int i = 0; i < v.size(); i++)
        {
            ROS_INFO_STREAM("Publish CAM to ROS: " << v[i]);
            int cam_index = (int)v[i];
            if (cam_index >= 0 && cam_index < camera_index_to_ros_vector.size())
            {
                camera_index_to_ros_vector[cam_index] = true;

                /// Change this if you want, I don't care
                std::stringstream cam_topic_stream;
                cam_topic_stream << "camera_" << v[i] << "/image_raw";

                std::stringstream cam_info_stream;
                cam_info_stream << "camera_info"
                                << "/image_raw" << v[i];

                //image_pub.push_back(nh.advertise<sensor_msgs::Image>(cam_topic_stream.str(), 100));
                camerainfo_pub.push_back(nh.advertise<sensor_msgs::CameraInfo>(cam_info_stream.str(), 100));
            }
        }
      */
    }

    int32_t base_port = 6000;
    nhPrivate.getParam("base_port", base_port);
    std::string base_pipeline;
    nhPrivate.getParam("udp_pipeline", base_pipeline);

    ROS_INFO_STREAM("udp_pipeline: " << base_pipeline);

    if (base_pipeline.empty())
    {
        return EXIT_FAILURE;
    }
    int udp_width;
    int udp_height;

    nhPrivate.param("width", udp_width, (int)CAPTURE_SIZE.width());
    nhPrivate.param("height", udp_height, (int)CAPTURE_SIZE.height());
    if (nhPrivate.param("cams_to_udp", v, v))
    {
        for (int i = 0; i < v.size(); i++)
        {

            int cam_index = (int)v[i];
            if (cam_index >= 0 && cam_index < camera_index_to_udp_vector.size())
            {
                camera_index_to_udp_vector[cam_index] = true;

                uint32_t cam_port = base_port + cam_index;
                ROS_INFO_STREAM("Publish CAM to UDP: " << v[i] << " with port " << cam_port);
                std::string updated_pipeline = std::regex_replace(base_pipeline, std::regex("PORT"), std::to_string(cam_port));

                gst_pipeline_vector.push_back(std::make_unique<Pipeline>(updated_pipeline, udp_width, udp_height, DEFAULT_FPS));
            }
        }
    }

    print_time_debug = nhPrivate.param("print_time_debug", false);

    if (!ArgusSamples::execute())
        return EXIT_FAILURE;

    return EXIT_SUCCESS;
}
