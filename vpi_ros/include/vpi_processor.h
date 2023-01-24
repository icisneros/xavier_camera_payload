#ifndef VPI_PROCESSOR_H
#define VPI_PROCESSOR_H

#include <opencv2/core/version.hpp>

#if CV_MAJOR_VERSION >= 3
#include <opencv2/imgcodecs.hpp>
#else
#include <opencv2/highgui/highgui.hpp>
#endif

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vpi/OpenCVInterop.hpp>
#include <vpi/algo/HarrisCorners.h>
#include <vpi/Array.h>

#include <string.h> // for basename(3) that doesn't modify its argument
#include <unistd.h> // for getopt
#include <vpi/Context.h>
#include <vpi/Image.h>
#include <vpi/LensDistortionModels.h>
#include <vpi/Status.h>
#include <vpi/Stream.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/Remap.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <experimental/filesystem>
#include <iostream>
#include <sstream>
#include <chrono>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#define CHECK_STATUS(STMT)                                    \
    do                                                        \
    {                                                         \
        VPIStatus status = (STMT);                            \
        if (status != VPI_SUCCESS)                            \
        {                                                     \
            char buffer[VPI_MAX_STATUS_MESSAGE_LENGTH];       \
            vpiGetLastStatusMessage(buffer, sizeof(buffer));  \
            std::ostringstream ss;                            \
            ss << vpiStatusGetName(status) << ": " << buffer; \
            throw std::runtime_error(ss.str());               \
        }                                                     \
    } while (0);

/**
 * @brief A base class for VPI functionality that is common to any invocation.
 * 
 */
class VPIProcessor
{

public:
    VPIProcessor(ros::NodeHandle &nh_, ros::NodeHandle &nh_private_)
    {

        backendType = parseBackendType(nh_, nh_private_);

        /// Create the Stream to be used for processing....
        CHECK_STATUS(vpiStreamCreate(backendType, &stream));
    }

    void setData(cv_bridge::CvImagePtr _cv_ptr)
    {

        if (_cv_ptr->image.rows != active_height || _cv_ptr->image.cols != active_width)
        {
            cv_ptr = _cv_ptr;
            frame = cv_ptr->image;
            CHECK_STATUS(vpiImageCreateOpenCVMatWrapper(frame, 0, &vImage));
        }
        else
        {
            CHECK_STATUS(vpiImageSetWrappedOpenCVMat(vImage, frame));
        }

        _cv_ptr->image.copyTo(frame);
    }

    static sensor_msgs::PointCloud2 KeypointsToCloud(VPIKeypoint *kpts, uint32_t *scores, int numKeypoints)
    {

        /// Initialize the PCL cloud for keypoints....
        pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

        /// Publish the accumulated cloud
        sensor_msgs::PointCloud2 keypoint_cloud_msg;

        
        if (numKeypoints == 0)
        {
            /// This is a temporary hack...
            keypoint_cloud_msg.header.stamp = ros::Time::now();
            keypoint_cloud_msg.header.frame_id = "map";

            return keypoint_cloud_msg;
        }

        float maxScore = *std::max_element(scores, scores + numKeypoints);

        pcl::PointXYZ point;

        for (int i = 0; i < numKeypoints; ++i)
        {
            point.x = kpts[i].x;
            point.y = kpts[i].y;
            point.z = ((float) scores[i]) / maxScore;
            keypoint_cloud_ptr->points.push_back(point);
        }

        pcl::toROSMsg(*keypoint_cloud_ptr, keypoint_cloud_msg);
        /// This is a temporary hack...
        keypoint_cloud_msg.header.stamp = ros::Time::now();
        keypoint_cloud_msg.header.frame_id = "map";

        return keypoint_cloud_msg;
    }

    static cv::Mat DrawKeypoints(cv::Mat img, VPIKeypoint *kpts, uint32_t *scores, int numKeypoints)
    {
        cv::Mat out;
        img.convertTo(out, CV_8UC1);
        cvtColor(out, out, cv::COLOR_GRAY2BGR);

        if (numKeypoints == 0)
        {
            return out;
        }

        // prepare our colormap
        cv::Mat cmap(1, 256, CV_8UC3);
        {
            cv::Mat gray(1, 256, CV_8UC1);
            for (int i = 0; i < 256; ++i)
            {
                gray.at<unsigned char>(0, i) = i;
            }
            applyColorMap(gray, cmap, cv::COLORMAP_HOT);
        }

        float maxScore = *std::max_element(scores, scores + numKeypoints);

        for (int i = 0; i < numKeypoints; ++i)
        {
            cv::Vec3b color = cmap.at<cv::Vec3b>(scores[i] / maxScore * 255);
            circle(out, cv::Point(kpts[i].x, kpts[i].y), 3, cv::Scalar(color[0], color[1], color[2]), -1);
        }

        return out;
    }

    void destroyResources()
    {
        // Wait until conversion finishes.
        CHECK_STATUS(vpiStreamSync(stream));

        // Destroy stream first, it'll make sure all processing
        // submitted to it is finished.
        vpiStreamDestroy(stream);
        vpiImageDestroy(vImage);
        ROS_INFO_STREAM("VPI Stream Destroyed!");
    }

protected:
    static VPIBackend parseBackendType(ros::NodeHandle &nh_, ros::NodeHandle &nh_private_)
    {

        std::string strBackend;
        nh_private_.getParam("backend_type", strBackend);
        if (strBackend == "cpu")
        {
            return VPI_BACKEND_CPU;
        }
        else if (strBackend == "cuda" || strBackend == "gpu")
        {
            return VPI_BACKEND_CUDA;
        }
        else if (strBackend == "pva")
        {
            return VPI_BACKEND_PVA;
        }
        else if (strBackend == "vic")
        {
            return VPI_BACKEND_VIC;
        }

        throw std::runtime_error("Backend '" + strBackend +
                                 "' not recognized, it must be either cpu, vic, gpu, cuda or pva.");
    }

    ///This contains all of the VPI objects -- when the class is destroyed the objects should be destroyed.
    ///Seems like using this is optional depending on the actual VPI kernel being run...
    //VPIContext ctx;

    /// The Stream used for processing...
    VPIStream stream = NULL;

    /// VPI_BACKEND_CUDA
    /// VPI_BACKEND_CPU
    /// VPI_BACKEND_VIC
    /// VPI_BACKEND_PVA
    /// VPI_BACKEND_NVENC
    VPIBackend backendType;

    /// Keep track of the image size...if we want to re-init the node all we do
    /// is re-create the CVMatWrapper
    uint32_t active_width;
    uint32_t active_height;

    VPIImage vImage = nullptr;
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat frame;

    bool listenROS;
    bool publishROS;   
};

#endif