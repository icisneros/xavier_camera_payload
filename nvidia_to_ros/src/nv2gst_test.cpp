/*
 * Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// https://forums.developer.nvidia.com/t/opencv-gpu-mat-into-gstreamer-without-downloading-to-cpu/186703/14

#include <cstdlib>
#include <gst/gst.h>
#include <gst/gstinfo.h>
#include <gst/app/gstappsrc.h>
#include <glib-unix.h>
#include <dlfcn.h>

#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>
#include <string.h>

#include "nvbuf_utils.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <cudaEGL.h>

#ifdef ENABLE_OCV_CUDA
#include <opencv2/cudafilters.hpp>
#endif

using namespace std;

#define USE(x) ((void)(x))

static GstPipeline *gst_pipeline = nullptr;
static string launch_string;
static GstElement *appsrc_;

GstClockTime timestamp = 0;
static int w = 1920;
static int h = 1080;
EGLDisplay egl_display;
static FILE *fp;
static guint size;

#ifdef ENABLE_OCV_CUDA
static bool create_filter = true;
static cv::Ptr< cv::cuda::Filter > filter;
#endif

static void
notify_to_destroy (gpointer user_data)
{
    GST_INFO ("NvBufferDestroy(%d)", *(int *)user_data);
    NvBufferDestroy(*(int *)user_data);
    g_free(user_data);
}

static gboolean feed_function(gpointer d) {
    GstBuffer *buffer;
    GstFlowReturn ret;
    GstMapInfo map = {0};
    int dmabuf_fd = 0;
    gpointer data = NULL, user_data = NULL;
    NvBufferParams par;
    GstMemoryFlags flags = (GstMemoryFlags)0;
    void *ptr;
   
    NvBufferCreate(&dmabuf_fd, w, h, NvBufferLayout_Pitch, NvBufferColorFormat_ABGR32);
    NvBufferMemMap(dmabuf_fd, 0, NvBufferMem_Read_Write, &ptr);
    NvBufferMemSyncForCpu(dmabuf_fd, 0, &ptr);
    //memset(ptr, 255, size);
    //fread(ptr, size, 1, fp);
    NvBufferMemSyncForDevice(dmabuf_fd, 0, &ptr);
    NvBufferMemUnMap(dmabuf_fd, 0, &ptr);
    //CUDA process
    {
        EGLImageKHR egl_image;
        egl_image = NvEGLImageFromFd(egl_display, dmabuf_fd);
        CUresult status;
        CUeglFrame eglFrame;
        CUgraphicsResource pResource = NULL;
        cudaFree(0);
        status = cuGraphicsEGLRegisterImage(&pResource,
                    egl_image,
                    CU_GRAPHICS_MAP_RESOURCE_FLAGS_NONE);
        if (status != CUDA_SUCCESS)
        {
            printf("cuGraphicsEGLRegisterImage failed: %d \n",status);
        }
        status = cuGraphicsResourceGetMappedEglFrame(&eglFrame, pResource, 0, 0);
        status = cuCtxSynchronize();

        // CUDA code here
#ifdef ENABLE_OCV_CUDA
        if (create_filter) {
            //filter = cv::cuda::createSobelFilter(CV_8UC4, CV_8UC4, 1, 0, 3, 1, cv::BORDER_DEFAULT);
            filter = cv::cuda::createGaussianFilter(CV_8UC4, CV_8UC4, cv::Size(31,31), 0, 0, cv::BORDER_DEFAULT);
            create_filter = false;
        }
        cv::cuda::GpuMat d_mat(h, w, CV_8UC4, eglFrame.frame.pPitch[0]);
        filter->apply (d_mat, d_mat);
#endif

        status = cuCtxSynchronize();
        status = cuGraphicsUnregisterResource(pResource);
        NvDestroyEGLImage(egl_display, egl_image);
    }
    user_data = g_malloc(sizeof(int));
    GST_INFO ("NvBufferCreate %d", dmabuf_fd);
    *(int *)user_data = dmabuf_fd;
    NvBufferGetParams (dmabuf_fd, &par);
    std::cout << "payload type: " << par.payloadType << " pixel: " << par.pixel_format << std::endl;;
    data = g_malloc(par.nv_buffer_size);

    buffer = gst_buffer_new_wrapped_full(flags,
                                         data,
                                         par.nv_buffer_size,
                                         0,
                                         par.nv_buffer_size,
                                         user_data,
                                         notify_to_destroy);
    buffer->pts = timestamp;

    gst_buffer_map (buffer, &map, GST_MAP_WRITE);
    memcpy(map.data, par.nv_buffer , par.nv_buffer_size);
    gst_buffer_unmap(buffer, &map);

    g_signal_emit_by_name (appsrc_, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    timestamp += 33333333;
    return G_SOURCE_CONTINUE;
}

int main(int argc, char** argv) {
    USE(argc);
    USE(argv);

    gst_init (&argc, &argv);

    GMainLoop *main_loop;
    main_loop = g_main_loop_new (NULL, FALSE);
    ostringstream launch_stream;

    egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    eglInitialize(egl_display, NULL, NULL);
    launch_stream
    << "appsrc name=mysource ! "
    << "video/x-raw(memory:NVMM),width="<< w <<",height="<< h <<",framerate=30/1,format=RGBA ! "
    << "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! "
    << "nvv4l2h264enc ! h264parse ! matroskamux ! filesink location=a.mkv ";

    launch_string = launch_stream.str();

    g_print("Using launch string: %s\n", launch_string.c_str());

    GError *error = nullptr;
    gst_pipeline  = (GstPipeline*) gst_parse_launch(launch_string.c_str(), &error);

    if (gst_pipeline == nullptr) {
        g_print( "Failed to parse launch: %s\n", error->message);
        return -1;
    }
    if(error) g_error_free(error);

    appsrc_ = gst_bin_get_by_name(GST_BIN(gst_pipeline), "mysource");
    gst_app_src_set_stream_type(GST_APP_SRC(appsrc_), GST_APP_STREAM_TYPE_STREAM);

    gst_element_set_state((GstElement*)gst_pipeline, GST_STATE_PLAYING); 

    size = (w*h*4);
    //fp = fopen ("1080.yuv", "rb");
    for (int i=0; i<150; i++) {
        feed_function(nullptr);
    }
    //fclose(fp);

    // Wait for EOS message
    gst_element_send_event ((GstElement*)gst_pipeline, gst_event_new_eos ());
    GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(gst_pipeline));
    gst_bus_poll(bus, GST_MESSAGE_EOS, GST_CLOCK_TIME_NONE);

    gst_element_set_state((GstElement*)gst_pipeline, GST_STATE_NULL);
    gst_object_unref(GST_OBJECT(gst_pipeline));
    g_main_loop_unref(main_loop);
    eglTerminate(egl_display);

    g_print("going to exit \n");
    return 0;
}
