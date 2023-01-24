#include <signal.h>
#include <vpi_harris.h>


// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

int main(int argc, char *argv[])
{
    // We'll create all vpi objects under this context, so that
    // we don't have to track what objects to destroy. Just destroying
    // the context will destroy all objects.

    ros::Subscriber fish_sub;
    ros::Publisher fish_pub;

    ros::Subscriber harris_sub;
    ros::Publisher harris_pub;
    ros::init(argc, argv, "vpi_harris_ros_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_("~");
    signal(SIGINT, mySigIntHandler);
 
    //VPIBridge vpiBridge(nh_, nh_private_);
    //HarrisCornerDetector hcd(nh_, nh_private_); 
    HarrisCornerDetector hcd(nh_, nh_private_); 
    //FisheyeUndistort fishUn(nh_, nh_private_); 
    //VPIFileProcessor vpiFile(nh_, nh_private_);

    // Do our own spin loop
    while (!g_request_shutdown)
    {
        // Do non-callback stuff

        ros::spinOnce();
        usleep(100000);
    }

    // Do pre-shutdown tasks
    hcd.destroyResources();
    ros::shutdown();
    ROS_INFO_STREAM("EXIT SPIN!");

return 0;
}