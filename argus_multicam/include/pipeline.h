#ifndef PIPELINE_H
#define PIPELINE_H
#include <string>
#include <gst/gst.h>
#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <gst/app/gstappsrc.h>
#include <gst/pbutils/missing-plugins.h>

class Pipeline
{

 public:

 Pipeline(const std::string& pipeline_, const int width_, const int height_, const int framerate) : fps(framerate), width(width_), height(height_), frameCount(0), source(nullptr), pipeline(nullptr), buffer(nullptr) {
    gst_init (NULL, NULL);
    GError *err = NULL;
    ROS_INFO_STREAM(pipeline_);
    
    pipeline = gst_parse_launch(pipeline_.c_str(), &err);
    if (err || !pipeline)
        ROS_ERROR_STREAM("gst_parse_launch() FAILED! " << pipeline_);

    //source = gst_bin_get_by_name(GST_BIN(pipeline), "myname");
    
    GstIterator *it = gst_bin_iterate_sources(GST_BIN(pipeline));
    GValue value = G_VALUE_INIT;
    GstIteratorResult r;
    do {
        r = gst_iterator_next(it, &value);
        if (r == GST_ITERATOR_OK) {
            GstElement *element = GST_ELEMENT (g_value_get_object (&value));
            if(gchar* name = gst_element_get_name(element)) {
                if(strstr(name, "appsrc"))
                    source = GST_ELEMENT (gst_object_ref(element));
                g_free(name);
            }

        } else
            if (r == GST_ITERATOR_RESYNC)
                gst_iterator_resync (it);
    } while (!source && r != GST_ITERATOR_DONE && r != GST_ITERATOR_ERROR);
    gst_iterator_free(it);
    
    
   if (!source)
        ROS_FATAL("GStreamer: cannot find appsrc in pipeline");

    GstCaps *caps = gst_caps_new_simple("video/x-raw",
                               "format", G_TYPE_STRING, "BGR",
                               "width", G_TYPE_INT, width,
                               "height", G_TYPE_INT, height,
                               "framerate", GST_TYPE_FRACTION, int(framerate), 1,
                               NULL);
    caps = gst_caps_fixate(caps);
    gst_app_src_set_caps(GST_APP_SRC(source), caps);
    gst_app_src_set_stream_type(GST_APP_SRC(source), GST_APP_STREAM_TYPE_STREAM);
    gst_app_src_set_size (GST_APP_SRC(source), -1);
    g_object_set(G_OBJECT(source), "format", GST_FORMAT_TIME, NULL);
    g_object_set(G_OBJECT(source), "block", 1, NULL);
    g_object_set(G_OBJECT(source), "is-live", 0, NULL);

   if(gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        handleMessages();
        ROS_FATAL("gst_element_set_state(): cannot put pipeline to play");
   }
    handleMessages();
  }

void handleMessages()
{
    GError *err = NULL;
    gchar *debug = NULL;
    GstBus *bus = gst_element_get_bus(pipeline);
    while(gst_bus_have_pending(bus)) {
        GstMessage *msg = gst_bus_pop(bus);
        if(gst_is_missing_plugin_message(msg)) {
            ROS_ERROR("gst_is_missing_plugin_message(): your gstreamer installation is missing a required plugin");
        } else {
            switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_STATE_CHANGED:
                GstState oldstate, newstate, pendstate;
                gst_message_parse_state_changed(msg, &oldstate, &newstate, &pendstate);
                /*ROS_INFO("%s: state changed from %s to %s (pending: %s)",
                    gst_element_get_name(GST_MESSAGE_SRC (msg)),
                    gst_element_state_get_name(oldstate),
                    gst_element_state_get_name(newstate), gst_element_state_get_name(pendstate));*/
                break;
            case GST_MESSAGE_ERROR:
                gst_message_parse_error(msg, &err, &debug);
                ROS_ERROR("GStreamer Plugin: Embedded video playback halted; module %s reported: %s",
                    gst_element_get_name(GST_MESSAGE_SRC (msg)), err->message);
                g_error_free(err);
                g_free(debug);
                gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_NULL);
                break;
            case GST_MESSAGE_STREAM_STATUS:
                GstStreamStatusType tp;
                GstElement *elem;
                gst_message_parse_stream_status(msg, &tp, &elem);
                //ROS_INFO("stream status: elem %s, %i", GST_ELEMENT_NAME(elem), tp);
                break;
            default:
	      //ROS_WARN("unhandled message %s",GST_MESSAGE_TYPE_NAME(msg));
                break;
            }
            gst_message_unref(msg);
        }
    }
    gst_object_unref(GST_OBJECT(bus));
}

 
void write(const cv::Mat &image)
{
    handleMessages();
    
    if (image.cols != width || image.rows != height)
        ROS_FATAL("Size %d x %d changed to %d x %d", width, height, image.cols, image.rows);

    size_t size = image.dataend - image.datastart;
    buffer = gst_buffer_new_allocate (NULL, size, NULL);
    GstMapInfo info;
    gst_buffer_map(buffer, &info, (GstMapFlags)GST_MAP_READ);
    memcpy(info.data, image.datastart, size);
    gst_buffer_unmap(buffer, &info);
   
    GstClockTime duration = ((double)1/fps) * GST_SECOND, timestamp = frameCount * duration;
    GST_BUFFER_DURATION(buffer) = duration;
    GST_BUFFER_PTS(buffer) = timestamp;
    GST_BUFFER_DTS(buffer) = timestamp;
    GST_BUFFER_OFFSET(buffer) =  frameCount;
   
    if (gst_app_src_push_buffer(GST_APP_SRC(source), buffer) != GST_FLOW_OK) {
        ROS_WARN("gst_app_src_push_buffer(): Error pushing buffer to GStreamer pipeline");
        return;
    } 
   
    ++frameCount;
    
}


~Pipeline()
{
    if (pipeline)
    {
        handleMessages();

        if (gst_app_src_end_of_stream(GST_APP_SRC(source)) != GST_FLOW_OK)
        {
            ROS_WARN("Cannot send EOS to GStreamer pipeline\n");
            return;
        }

        //wait for EOS to trickle down the pipeline. This will let all elements finish properly
        GstBus* bus = gst_element_get_bus(pipeline);
        GstMessage *msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE, (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR)
        {
            ROS_WARN("gst_bus_timed_pop_filtered(): Error during finalization");
            return;
        }

        if(msg != NULL)
        {
            gst_message_unref(msg);
            g_object_unref(G_OBJECT(bus));
        }

        GstStateChangeReturn status = gst_element_set_state (pipeline, GST_STATE_NULL);
        if (status == GST_STATE_CHANGE_ASYNC)
        {
            // wait for status update
            GstState st1;
            GstState st2;
            status = gst_element_get_state(pipeline, &st1, &st2, GST_CLOCK_TIME_NONE);
        }
        if (status == GST_STATE_CHANGE_FAILURE)
        {
            handleMessages();
            gst_object_unref (GST_OBJECT (pipeline));
            ROS_WARN("Unable to stop gstreamer pipeline");
            return;
        }
        gst_object_unref (GST_OBJECT (pipeline));
    }
}


 private:
  int fps;
int width;
int height;
 uint32_t frameCount;

  GstBuffer* buffer;
  GstElement* pipeline, *source;
  

};

#endif // PIPELINE_H
