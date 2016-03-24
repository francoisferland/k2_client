/*************************************************************************************
Copyright (c) 2013, Carnegie Mellon University
All rights reserved.
Authors: Anurag Jakhotia<ajakhoti@andrew.cmu.edu>, Prasanna Velagapudi<pkv@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without modification, are 
permitted provided that the following conditions are met:

 -  Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
 -  Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the     distribution.
 -  Neither the name of Carnegie Mellon University nor the names of its contributors 
    may be used to endorse or promote products derived from this software without 
    specific prior written  permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED 
TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY 
WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/
#include "k2_client/k2_client.h"

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <yaml-cpp/yaml.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

using boost::asio::ip::tcp;
using namespace k2_client;

constexpr size_t image_width = 1920;
constexpr size_t image_height = 1080;
constexpr size_t image_size = image_width * image_height * 3; // 24-bit RGB image
constexpr size_t frame_size = image_size + sizeof(unsigned long); // image + timestamp 

constexpr size_t H264_BUFFER_SIZE = 16384; // Should normally be arbitrary.

unsigned char frame_buffer[frame_size];

// libavcodec-related objects.
AVFormatContext*    avc_format_context;
AVCodecContext*     avc_codec_context;
AVCodec*            avc_codec;
AVFrame*            avc_frame;
AVFrame*            avc_frame_rgb;
int                 avc_frame_rgb_size;
AVPacket*           avc_packet;
SwsContext*         sws_context;
AVInputFormat*      avc_input_format;
//AVFormatParameters* avc_format_params;

uint8_t             avc_buffer_raw[H264_BUFFER_SIZE];
uint8_t*            avc_buffer_rgb;

int main(int argc, char *argv[])
{
   
    // Initialize this ROS node.
    ros::init(argc, argv, "k2_rgb", ros::init_options::AnonymousName);
    ros::NodeHandle n("~");

    // Init avcodec (exits if it can't find something).
    avcodec_register_all();
    av_register_all();

    avc_codec = avcodec_find_decoder(CODEC_ID_H264);
    if (!avc_codec) {
        ROS_FATAL("Cannot find H.264 codec, aborting.");
        exit(-1);
    }

    avc_codec_context = avcodec_alloc_context3(avc_codec);
    avc_codec_context->width  = image_width;
    avc_codec_context->height = image_height;
    
    if (avcodec_open2(avc_codec_context, avc_codec, NULL) < 0) {
        ROS_FATAL("Could not open H.264 codec, aborting");
        exit(-1);
    }

    avc_frame     = avcodec_alloc_frame();
    avc_frame_rgb = avcodec_alloc_frame();

    int frame_size = avpicture_get_size(PIX_FMT_RGB24, image_width, image_height);
    avc_buffer_rgb = new uint8_t[frame_size];

    avpicture_fill((AVPicture*)avc_frame_rgb, avc_buffer_rgb, PIX_FMT_RGB24, image_width, image_height);
    
    sws_context = sws_getContext(image_width, image_height, avc_codec_context->pix_fmt,
                                 image_width, image_height, PIX_FMT_RGB24,
                                 SWS_FAST_BILINEAR, NULL, NULL, NULL);

    // Retrieve the hostname and port of the k2_server.
    std::string server_host, server_port, frame_id;
    n.getParam("host", server_host);
    n.param<std::string>("port", server_port, "9100"); // default for k2_server RGB (H.264)
    n.param<std::string>("frame_id", frame_id, "/k2/rgb_frame");

    // Create a Boost ASIO service to handle server connection.
    boost::asio::io_service io_service;

    // Get a list of endpoints corresponding to the server hostname and port.
    tcp::resolver resolver(io_service);
    tcp::resolver::iterator endpoint_iterator = resolver.resolve({server_host, server_port});

    // Try each endpoint until we successfully establish a connection.
    tcp::socket tcp_socket(io_service);
    try
    {
        boost::asio::connect(tcp_socket, endpoint_iterator);
    }
    catch (boost::system::system_error const& e)
    {
        ROS_FATAL("Failed to connect to k2 server '%s:%s': %s",
                  server_host.c_str(), server_port.c_str(), e.what());
        return -1;
    }

    // Create a ROS publisher for the deserialized stream output.
    image_transport::ImageTransport image_transport(n);
    image_transport::CameraPublisher camera_publisher =
        image_transport.advertiseCamera("image", 1);
    camera_info_manager::CameraInfoManager camera_info_manager(n, "rgb");
    camera_info_manager.loadCameraInfo("");
    cv::Mat image(cv::Size(image_width, image_height), CV_8UC3, frame_buffer);

    while(ros::ok())
    {
        // Read the next image from the server.
        boost::asio::read(tcp_socket, boost::asio::buffer(avc_buffer_raw, H264_BUFFER_SIZE));
        
        AVPacket packet;
        av_init_packet(&packet);
        av_new_packet(&packet, H264_BUFFER_SIZE);
        std::copy(&avc_buffer_raw[0], &avc_buffer_raw[H264_BUFFER_SIZE], packet.data);

        int frame_finished = 0;
        int dec_res = avcodec_decode_video2(avc_codec_context, avc_frame, &frame_finished, &packet);

        if (dec_res >= 0 && frame_finished >= 0) {
            // Extract the timestamp (packed at end of image) (TODO).
            //unsigned long timestamp = *reinterpret_cast<unsigned long *>(&frame_buffer[image_size]);

            // Converts to RGB and push the image through image_transport.
            sws_scale(sws_context,
                      avc_frame->data,
                      avc_frame->linesize,
                      0,
                      image_height,
                      avc_frame_rgb->data,
                      avc_frame_rgb->linesize);

            sensor_msgs::Image ros_image;
            ros_image.data.resize(image_size);
            std::copy(&avc_buffer_rgb[0], &avc_buffer_rgb[image_size], ros_image.data.data());

            sensor_msgs::CameraInfo camera_info = camera_info_manager.getCameraInfo();
            camera_info.header.frame_id = frame_id;

            // Send out the resulting message and request a new message.
            camera_publisher.publish(ros_image, camera_info, ros::Time(0)); // TODO: timestamp.
        }

        // Request a new packet (TODO: Not terribly important).
        boost::asio::write(tcp_socket, boost::asio::buffer("OK\n"));

        ros::spinOnce();
    }

    // Bring down avcodec items.
    avcodec_close(avc_codec_context);
    av_free(avc_frame);
    av_free(avc_frame_rgb);
    sws_freeContext(sws_context);

    delete[] avc_buffer_rgb;

    return 0;
}
