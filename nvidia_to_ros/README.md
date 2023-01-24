
# NVIDIA to ROS Overview

This library is for quickly transferring images between processes with low CPU usage on jetson platforms. The API is designed to be similar to ROS.
The user of the library creates publishers and subscribers with callback functions similar to ROS, but under the hood on jetson platforms
the image transfer is done with an nvidia library that transfers the images more efficiently than ROS. The library also provides an efficient way
of doing color format conversions, scaling, rotating, flipping, and cropping images.

When the library is built on non jetson platforms, detected automaticaly in the CMakeLists.txt,
it instead uses ROS under the hood and is intended for being used to do developement without access to a jetson rather than being efficient.

# Demo Programs

This demo runs a node, `nv_producer`, which publishes images on two topics, and another node, `nv_consumer`, which subscribes to the two image topics and publishes them as ROS messages, along with two `rqt_image_view`s for visualization.

```
mon launch nvidia_to_ros demo.launch
```

This demo runs the nodes from the previous demo, plus another node, `nv_transform`, which demonstrates the use of the transform function for doing color conversions, scaling, rotating, flipping, and cropping images.
It also launches another `rqt_image_view` for viewing the results of the transform.

```
mon launch nvidia_to_ros transform_demo.launch
```

This demo subscribes to a ROS image topic and republishes it as an NvImageMessage topic using an nv2ros::Publisher. The launch file takes an `input_topic` and `output_topic` argument.
The `input_topic` is the name of a ROS topic published from some other node, for example a bag file. The `output_topic` is the name of the NvImageMesssage topic that will be published by this node.

```
mon launch nvidia_to_ros ros2nv.launch input_topic:=/camera_image0 output_topic:=/nv_image3
```

The previous examples can be run on either a jetson or non jetson platform. The following can only be run on a jetson in conjuction with the argus_mutlicam package. It runs the camera driver which publishes images using this library and runs
an `nv_consumer` node to publish ROS image messages.

```
mon launch nvidia_to_ros argus_camera.launch
```

# Tools

## Logging to mp4

To log a camera stream to an mp4 file you can add the following to a launch file. Note that the path in `filaname` should not include a `~`.

```
<node name="nv2gst0" pkg="nvidia_to_ros" type="nv2gst" output="screen">
  <param name="topic" value="/nv_image0" />
  <param name="filename" value="/home/user/bags/cam0.mp4" />
</node>
```

## Converting mp4 to bags

You can do the following to convert the mp4 files to bags. Note that the paths should not contain `~`s. Unfortunately this conversion takes the same duration as the mp4 file.

```
mon launch nvidia_to_ros gst2bag input_filename:=[the full path and filename of the mp4 file] output_bag_filename:=[the bag file that will be generated] output_topic:=[what you want the image topic in the bag to be named]
```
# API

## Publisher

Publishers can be created by using the following constructors:

```
Publisher(ros::NodeHandle nh, std::string topic_name, int queue_size=1);
Publisher(ros::NodeHandle nh, std::vector<std::string> topic_names, int queue_size=1);
```

The queue size determines how many images will be stored in the publisher, before being discarded.
Having a larger queue size gives subscribers more time to receive the image.

If the constructor using a vector of topic_names is used, the images published on each topic are assumed to be
time synchronized with each other using the stamp in the published NvImageMessages. NvImageMessages will not
reach the subscriber until the full set of synchonized images is published. In this case the queue_size parameter
also determines how many images will be kept on each topic, so a larger size allows more time for images
with syncrhonized stamps to be found on each topic.

As an example, consider the following publisher created with two image topics.
```
ros::NodeHandle nh;
std::vector<std::string> topic_names;
topic_names.push_back("topic0");
topic_names.push_back("topic1");
nv2ros::Publisher pub(nh, topic_names);
```
If only one image is published nothing will reach the subscriber
```
ros::Time now = ros::Time::now();
nv2ros::NvImageMessage image0(width, height, now);
pub.publish(image0, 0);
```
It won't be until the complete set of two synchronzied images are published, that they will reach the subscriber.
```
nv2ros::NvImageMessage image1(width, height, now);
pub.publish(image1, 1);
```

The `publish` function takes an NvImageMessage and an index. The index corresponds to the topic_names vector
provided to the Publisher's constructor. In the example above, index 0 corresponds to publishing on "topic0"
and index 1 correspondes to publishing on "topic1".

The index parameter defaults to 0, so if a publisher was created with only one image topic, an image
could be published by doing `pub.publish(image)` rather than `pub.publish(image, 0)`.

## Subscriber

Subscriber are created with callback functions similar to ROS. Consider the following callback functions:

```
void callback(nv2ros::NvImageMessage msg){

}

void multi_callback(nv2ros::NvImageMessage msg){

}

class Class {
	void class_callback(nv2ros::NvImageMessage msg){

	}

	void class_multi_callback(nv2ros::NvImageMessage msg){

	}
};
```

Subscribers can be created with one or more image topics using the callbacks outside of the class in the
following ways:

```
nv2ros::Subscriber sub(nh, "topic", 1, callback);
nv2ros::Subscriber sub(nh, "topic", 1, multi_callback);
nv2ros::Subscriber sub(nh, topic_names, 1, multi_callback);
```

Subscribers can be created within the class with one or more image topics in the following ways:

```
nv2ros::Subscriber sub(nh, "topic", 1, &Class::class_callback, this);
nv2ros::Subscriber sub(nh, "topic", 1, &Class::class_multi_callback, this);
nv2ros::Subscriber sub(nh, topic_names, 1, &Class::class_multi_callback, this);
```

The NvImageMessages should only be used within the callback. After the callback returns, the underlying image data
is freed, so the NvImageMessages should not be copied for use after the callback function.


## NvImageMessage

#### NvImageMessage(int width, int height, ros::Time stamp, NvBufferColorFormat color_format=NvBufferColorFormat_ABGR32, NvBufferLayout layout=NvBufferLayout_Pitch, NvBufferPayloadType payload_type=NvBufferPayload_SurfArray, NvBufferTag tag=NvBufferTag_NONE)
Creates an image with the given width, height, and stamp. The default color format is `NvBufferColorFormat_ABGR32`, which despite the name uses a BGRA format for the pixel data.
A full list of color formats is [here](https://docs.nvidia.com/jetson/l4t-multimedia/group__ee__nvbuffering__group.html#gaae53b45fe3f04b8f9135cb80baeac6e4).
The layout NvBufferLayout_Pitch makes it compatible with OpenCV Mats.

Example usage in [nv_producer](https://bitbucket.org/castacks/nvidia_to_ros/src/893a8564bb1e3957975496fd88ae105034856fe9/src/nv_producer.cpp#lines-91).

#### void mem_map(void** pointer)
Gets the memory-mapped virtual address of the plane.

The client must call `mem_map` with the virtual address returned by this function before accessing the mapped memory in CPU.

After memory mapping is complete, mapped memory modification must be coordinated between the CPU and hardware device as follows:

CPU: If the CPU modifies any mapped memory, the client must call `mem_sync_for_device` before any hardware device accesses the memory.

Hardware device: If the mapped memory is modified by any hardware device, the client must call `mem_sync_for_cpu` before CPU accesses the memory.

Example usage in [nv_consumer](https://bitbucket.org/castacks/nvidia_to_ros/src/893a8564bb1e3957975496fd88ae105034856fe9/src/nv_consumer.cpp#lines-29)

#### void mem_unmap(void** pointer)
Unmaps the mapped virtual address of the plane. There should be a corresponding `mem_unmap` call for each `mem_map` call.

If the following conditions are both true, the client must call `mem_sync_for_device` before unmapping the memory:

Mapped memory was modified by the CPU.

Mapped memory will be accessed by a hardware device.

Example usage in [nv_consumer](https://bitbucket.org/castacks/nvidia_to_ros/src/893a8564bb1e3957975496fd88ae105034856fe9/src/nv_consumer.cpp#lines-42)

#### void mem_sync_for_cpu(void** pointer)
Syncs the hardware memory cache for the CPU.

Example usage in [nv_consumer](https://bitbucket.org/castacks/nvidia_to_ros/src/893a8564bb1e3957975496fd88ae105034856fe9/src/nv_consumer.cpp#lines-30)

#### void mem_sync_for_device(void** pointer)
Syncs the hardware memory cache for the device.

Example usage in [nv_producer](https://bitbucket.org/castacks/nvidia_to_ros/src/893a8564bb1e3957975496fd88ae105034856fe9/src/nv_producer.cpp#lines-52)

#### void transform(NvImageMessage& destination, NvBufferTransformParams* transform_params)
This function handles copying, color format conversion, scaling, flipping, rotating, and cropping source and destination images.

Color format conversion and resizing are defined by the destination image. The rest of the operations are specified in the `transform_params` struct.
Examples of all the operations can be found in src/nv_transform.cpp.

Cropping of the source images happens first, then the scaling, flipping and rotating operations. "Cropping" of the destination image actually places a scaled version of the image into the
destination rectangle in the `transform_params` struct and the remainder of the image outside the rectangle gets filled with black pixels.

To run the nv_transform program on a jetson with a camera, run:
```
mon launch nvidia_to_ros transform_camera.launch
```
To run the nv_transform program along with the sample bouncing circle demo, run:
```
mon launch nvidia_to_ros transform_demo.launch
```

Example usage in [nv_transform](https://bitbucket.org/castacks/nvidia_to_ros/src/893a8564bb1e3957975496fd88ae105034856fe9/src/nv_transform.cpp#lines-75)

#### int get_width()
Returns the width of the image's first plane. If you need the width of other planes, you can do `get_params().width[i]`.

Example usage in [nv_consumer](https://bitbucket.org/castacks/nvidia_to_ros/src/893a8564bb1e3957975496fd88ae105034856fe9/src/nv_consumer.cpp#lines-32)

#### int get_height()
Returns the height of the image's first plane. If you need the width of other planes, you can do `get_params().height[i]`.

Example usage in [nv_consumer](https://bitbucket.org/castacks/nvidia_to_ros/src/893a8564bb1e3957975496fd88ae105034856fe9/src/nv_consumer.cpp#lines-32)

#### int get_pitch()
Returns the pitch of the image's first plane. If you need the width of other planes, you can do `get_params().pitch[i]`.

Example usage in [nv_consumer](https://bitbucket.org/castacks/nvidia_to_ros/src/893a8564bb1e3957975496fd88ae105034856fe9/src/nv_consumer.cpp#lines-32)

#### ros::Time get_stamp()
returns the time stamp of the image.

Example usage in [nv_transform](https://bitbucket.org/castacks/nvidia_to_ros/src/893a8564bb1e3957975496fd88ae105034856fe9/src/nv_transform.cpp#lines-48)

#### void set_stamp(ros::Time stamp)
Sets the timestamp of the image.

Example usage in [nv_producer](https://bitbucket.org/castacks/nvidia_to_ros/src/893a8564bb1e3957975496fd88ae105034856fe9/src/nv_producer.cpp#lines-56)

#### NvBufferParams get_params()
Returns the NvBufferParams of the image. The only fields of this struct that are filled in by the non jetson version of the library are the width, height, and pitch fields. Otherwise, it should not be used on a non jetson.

The NvBufferParams struct is documented [here](https://docs.nvidia.com/jetson/l4t-multimedia/struct__NvBufferParams.html)

#### NvBufferParamsEx get_params_ex()
Returns the NvBufferParamsEx of the image. This should only be used on the jetson.

The NvBufferParamsEx struct is documented [here](https://docs.nvidia.com/jetson/l4t-multimedia/struct__NvBufferParamsEx.html)

#### NvImageMessage(int dmabuf_fd, NvBufferParams params, NvBufferParamsEx params_ex, ros::Time stamp)
This function should only be used on the jetson. The non jetson version of the library abstracts away the dma buffer.

This creates an NvImageMessage from a pre-existing dma bufffer file descriptor associated with the params and params_ex.

Example usage in [argus_multicam](https://bitbucket.org/castacks/argus_multicam/src/d4235d8075decf5f13eb0e144010d69149db16df/src/argus_main_fast.cpp#lines-542)

#### int get_dmabuf_fd()
Returns the fd of the image. This function should only be used on the jetson.

Example usage in [nv2gst](https://bitbucket.org/castacks/nvidia_to_ros/src/893a8564bb1e3957975496fd88ae105034856fe9/src/nv2gst.cpp#lines-267)

## Using with OpenCV

An OpenCV Mat object can be created using the data in the NvImageMessage without copying. The data needs to be mapped to a pointer and synced for usage on the cpu.
Then, the pointer can be used to initilaize an OpenCV Mat. Here is an example of those three steps in [nv_consumer](https://bitbucket.org/castacks/nvidia_to_ros/src/893a8564bb1e3957975496fd88ae105034856fe9/src/nv_consumer.cpp#lines-29:32)