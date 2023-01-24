#ifndef VPI_HARRIS_H
#define VPI_HARRIS_H

#include <vpi_processor.h>

class HarrisCornerDetector : public VPIProcessor
{

public:
    HarrisCornerDetector(ros::NodeHandle &nh_, ros::NodeHandle &nh_private_) : VPIProcessor(nh_, nh_private_)
    {
        /// TODO: Get this from roslaunch?
        harris_params.gradientSize = 3;
        harris_params.blockSize = 5;
        harris_params.strengthThresh = 20;
        harris_params.sensitivity = 0.1;
        harris_params.minNMSDistance = 8; // must be 8 for PVA backend

        // Create the output keypoint array. Currently for PVA backend it must have 8192 elements.
        CHECK_STATUS(vpiArrayCreate(8192, VPI_ARRAY_TYPE_KEYPOINT, 0, &keypoints));

        // Create the output scores array. It also must have 8192 elements and elements must be uint32_t.
        CHECK_STATUS(vpiArrayCreate(8192, VPI_ARRAY_TYPE_U32, 0, &scores));

        /// If you want to use a ROS topic as input, just specify an in_topic in the roslaunch
        std::string in_topic;
        if (nh_private_.getParam("in_topic", in_topic)){
            listenROS = true;
            harris_sub = nh_.subscribe(in_topic, 1000, &HarrisCornerDetector::imageCallback, this);
        }

        /// 
        /// Usually it is good to have a ROS IMAGE topic to visualize the 'relevant' output of the VPI algorithm
        /// 
        /// In this case, we will publish the image with the Harris features drawn on the image with OpenCV.
        /// If you do not want to publish, then do not specify the topic in the ROS Launch.
        std::string out_topic;
        if (nh_private_.getParam("out_topic", out_topic)){
            publishROS = true;
            harris_pub = nh_.advertise<sensor_msgs::Image>(out_topic, 1000);
        } else {
            publishROS = false;
        }

        std::string points_out_topic;
        if (nh_private_.getParam("points_out_topic", points_out_topic)){
            publishPoints = true;
            ROS_INFO_STREAM("OOPPP " << points_out_topic);
            harris_points_pub = nh_.advertise<sensor_msgs::PointCloud2>(points_out_topic, 1000);
        } else {
            publishPoints = false;
        }
    }

    void setData(cv_bridge::CvImagePtr _cv_ptr)
    {
        ///
        VPIProcessor::setData(_cv_ptr);
 
        if (_cv_ptr->image.rows != active_height || _cv_ptr->image.cols != active_width)
        {
            active_width = _cv_ptr->image.cols;
            active_height = _cv_ptr->image.rows;

            /// Create the payload for Harris Corners Detector algorithm
            CHECK_STATUS(vpiCreateHarrisCornerDetector(backendType, active_width, active_height, &harris));
            /*try
            {
                cv::cvtColor(_cv_ptr->image, cv_ptr->image, CV_BGR2GRAY);

                // Currently we only accept signed 16bpp
                cv_ptr->image.convertTo(cv_ptr->image, CV_16SC1);
            }
            catch (cv_bridge::Exception &e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }*/

        }
    }

    void vpiSubmit()
    {
        CHECK_STATUS(vpiSubmitHarrisCornerDetector(stream, backendType, harris, vImage, keypoints, scores, &harris_params));
        // Wait until the algorithm finishes processing
        CHECK_STATUS(vpiStreamSync(stream));

        // Now let's retrieve the output

        // Lock output keypoints and scores to retrieve its data on cpu memory
        VPIArrayData outKeypointsData;
        VPIArrayData outScoresData;
        CHECK_STATUS(vpiArrayLock(keypoints, VPI_LOCK_READ, &outKeypointsData));
        CHECK_STATUS(vpiArrayLock(scores, VPI_LOCK_READ, &outScoresData));

        VPIKeypoint *outKeypoints = (VPIKeypoint *)outKeypointsData.data;
        uint32_t *outScores = (uint32_t *)outScoresData.data;

        //printf("%d keypoints found\n", *outKeypointsData.sizePointer);

        if (publishROS){
            /// Draw the keypoints on the image only if publishing it to ROS
            cv::Mat outImage = VPIProcessor::DrawKeypoints(frame, outKeypoints, outScores, *outKeypointsData.sizePointer);
            cv_ptr->image = outImage;
            cv_ptr->encoding= "bgr8";
            harris_pub.publish(cv_ptr->toImageMsg());     
        }

        if (publishPoints){
            harris_points_pub.publish(VPIProcessor::KeypointsToCloud(outKeypoints, outScores, *outKeypointsData.sizePointer));
        }

        // Done handling outputs, don't forget to unlock them.
        CHECK_STATUS(vpiArrayUnlock(scores));
        CHECK_STATUS(vpiArrayUnlock(keypoints));
    }

    void processImage(cv_bridge::CvImagePtr& _cv_ptr)
    {
        //cv::cvtColor(_cv_ptr->image, _cv_ptr->image, CV_RGB2GRAY);

        // Currently we only accept signed 16bpp
        _cv_ptr->image.convertTo(_cv_ptr->image, CV_16SC1);

        /// Need to do the Convert because the camera message might not be the required format for Harris
        /// Grayscale with CV_16SC1
        setData(_cv_ptr);
        vpiSubmit();
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        /// Need to do the Convert because the camera message might not be the required format for Harris
        /// Grayscale with CV_16SC1
        cv_bridge::CvImagePtr _cv_ptr = copyAndConvert(msg);
        processImage(_cv_ptr);
    }


    void destroyResources(){
        VPIProcessor::destroyResources();
        vpiPayloadDestroy(harris);
    }

private:
    /**
         * @brief This function is used to ensure that the incoming image is in an acceptable
         * format for the VPI Harris Corner detector which requires a Grayscale image with a
         * format of CV_16SC1.
         * 
         * @param msg The ROS image to convert to the Harris format
         * @return cv_bridge::CvImagePtr 
         */
    static cv_bridge::CvImagePtr copyAndConvert(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr tmp_cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        cv::cvtColor(tmp_cv_ptr->image, tmp_cv_ptr->image, CV_BGR2GRAY);

        // Currently we only accept signed 16bpp
        tmp_cv_ptr->image.convertTo(tmp_cv_ptr->image, CV_16SC1);
        return tmp_cv_ptr;
    }

    /// For Keypoint extraction....
    VPIArray keypoints;
    VPIArray scores;
    VPIHarrisCornerDetectorParams harris_params;
    VPIPayload harris;

    /// We definite these in the child class, because the logic is too complicated/different depending
    /// on the the circumstances of the VPI node. i.e. in this case we publish a debug output image as
    /// well as the list of key points for consumption by SLAM.
    ros::Subscriber harris_sub;
    ros::Publisher harris_pub;

    bool publishPoints;    
    ros::Publisher harris_points_pub;
};

#endif