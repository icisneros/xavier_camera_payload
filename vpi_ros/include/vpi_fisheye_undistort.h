#ifndef VPI_FISH_UNDISTORT_H
#define VPI_FISH_UNDISTORT_H

#include <vpi_processor.h>

class FisheyeUndistort : public VPIProcessor
{

public:
    FisheyeUndistort(ros::NodeHandle &nh_, ros::NodeHandle &nh_private_) : VPIProcessor(nh_, nh_private_)
    {

        std::string in_topic;
        if (nh_private_.getParam("in_topic", in_topic)){
           listenROS = true;
           fish_sub = nh_.subscribe(in_topic, 1000, &FisheyeUndistort::imageCallback, this);
        } 

        if (!nh_private_.getParam("camera_matrix", camera_matrix) ){
           throw std::runtime_error("Please specify ros params camera_matrix if using VPI for Fisheye Undistortion");
        } 
      
        if (!nh_private_.getParam("fisheye_coeff", fisheye_coeff)){
           throw std::runtime_error("Please specify ros params fisheye_coeff if using VPI for Fisheye Undistortion");
        } 
      
        distModel = {};
        distModel.mapping = VPI_FISHEYE_EQUIDISTANT;
        distModel.k1 = fisheye_coeff[0];
        distModel.k2 = fisheye_coeff[1];
        distModel.k3 = fisheye_coeff[2];
        distModel.k4 = fisheye_coeff[3];

        //Fisheye coefficients: -0.073234 0.121251 -0.120230 0.038480
        // Camera matrix:
        //[440.956750 0.000000 659.822931; 0.000000 439.322113 451.685233; 0.000000 0.000000 1.000000]

        for (int i = 0; i < 2; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                int32_t source_index = j + i * 3;
                K[i][j] = camera_matrix[source_index];
                ROS_INFO_STREAM("Source IN " << i << " " << j << " " << source_index << " " << K[i][j]);
            }
        }
        X[0][0] = X[1][1] = X[2][2] = 1;

        /// 
        /// Usually it is good to have a ROS IMAGE topic to visualize the 'relevant' output of the VPI algorithm
        /// 
        /// In this case, we will publish the undistorted image to a ROS topic.
        /// If you do not want to publish, then do not specify the topic in the ROS Launch.
        std::string out_topic;
        if (nh_private_.getParam("out_topic", out_topic)){
            publishROS = true;
            fish_pub = nh_.advertise<sensor_msgs::Image>(out_topic, 1000);
        }
    }

    void processImage(cv_bridge::CvImagePtr& _cv_ptr)
    {
        setData(_cv_ptr);
        vpiSubmit();
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr _cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        processImage(_cv_ptr);
    }

    void destroyResources(){
        // Destroy stream first, it'll make sure all processing
        // submitted to it is finished.
        // vpiStreamDestroy(stream);
        VPIProcessor::destroyResources();
        // Now we can destroy other VPI objects, since they aren't being
        // used anymore.
        vpiImageDestroy(tmpIn);
        vpiImageDestroy(tmpOut);
        vpiPayloadDestroy(remap);
        vpiWarpMapFreeData(&map);
        ROS_ERROR_STREAM("SHUT FISH RESOURCE!");

    }   

    void vpiSubmit(){

        // Convert BGR -> NV12
        CHECK_STATUS(vpiSubmitConvertImageFormat(stream, backendType, vImage, tmpIn, NULL));

        // Undistorts the input image.
        CHECK_STATUS(vpiSubmitRemap(stream, backendType, remap, tmpIn, tmpOut, VPI_INTERP_CATMULL_ROM,
                                    VPI_BORDER_ZERO, 0));

        // Convert the result NV12 back to BGR, writing back to the input image.
        CHECK_STATUS(vpiSubmitConvertImageFormat(stream, backendType, tmpOut, vImage, NULL));

        // Wait until conversion finishes.
        CHECK_STATUS(vpiStreamSync(stream));

        //CHECK_STATUS(vpiImageLock(vImage, VPI_LOCK_READ, &out_data));

        //CHECK_STATUS(vpiImageDataExportOpenCVMat(out_data, &sinkImg));
        
        //CHECK_STATUS(vpiImageUnlock(vImage));

        frame.copyTo(cv_ptr->image);
        //sinkImg.copyTo(cv_ptr->image);
        //char buf[64];
        //snprintf(buf, sizeof(buf), "Dundistort_%03d.jpg", write_count);

        //cv::imwrite(buf, cv_ptr->image);
  
        // Since vimg is wrapping the OpenCV image, the result is already there.
        // We can publish the undistored image to ROS if we want.
        if (publishROS){      
            fish_pub.publish(cv_ptr->toImageMsg());
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
            
            memset(&map, 0, sizeof(map));
            map.grid.numHorizRegions = 1;
            map.grid.numVertRegions = 1;
            map.grid.regionWidth[0] = active_width;
            map.grid.regionHeight[0] = active_height;
            map.grid.horizInterval[0] = 1;
            map.grid.vertInterval[0] = 1;
            CHECK_STATUS(vpiWarpMapAllocData(&map));

            // Generate a warp map to undistort an image taken from fisheye lens with
            // given parameters calculated above.
            vpiWarpMapGenerateFromFisheyeLensDistortionModel(K, X, K, &distModel, &map);

            // Create a stream where operations will take place. We're using CUDA
            // processing.
            //CHECK_STATUS(vpiStreamCreate(backendType, &stream));

            // Create the Remap payload for undistortion given the map generated above.
            CHECK_STATUS(vpiCreateRemap(backendType, &map, &remap));

            // Temporary input and output images in NV12 format.
            CHECK_STATUS(vpiImageCreate(active_width, active_height, VPI_IMAGE_FORMAT_NV12_ER, 0, &tmpIn));

            CHECK_STATUS(vpiImageCreate(active_width, active_height, VPI_IMAGE_FORMAT_NV12_ER, 0, &tmpOut));

            try
            {
                cv_ptr = _cv_ptr;
                //frame = cv_ptr->image;
                cv_ptr->image.copyTo(frame);
            }
            catch (cv_bridge::Exception &e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            //char buf[64];
            //snprintf(buf, sizeof(buf), "startingframe.jpg");
            //cv::imwrite(buf, frame);
        }
    }

private:
    VPIWarpMap map;
    VPIPayload remap = NULL;
    VPIFisheyeLensDistortionModel distModel;
    VPICameraIntrinsic K;
    VPICameraExtrinsic X;
    VPIImage tmpIn = NULL;
    VPIImage tmpOut= NULL;
    VPIImageData out_data;

    ros::Subscriber fish_sub;
    ros::Publisher fish_pub;

    cv::Mat sinkImg;
    /// Read from the launch file....
    std::vector<double> camera_matrix;
    std::vector<double> fisheye_coeff;
};

#endif