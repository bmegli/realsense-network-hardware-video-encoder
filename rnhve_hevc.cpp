/*
 * Realsense Network Hardware Video Encoder
 *
 * Realsense hardware encoded UDP HEVC streaming
 * - color/infrared (Main)
 * - depth (Main10)
 * - depth with infrared (Main10)
 *
 * Copyright 2020 (C) Bartosz Meglicki <meglickib@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 */

// Network Hardware Video Encoder
#include "nhve.h"

// Realsense API
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <fstream>
#include <iostream>
#include <cassert>
using namespace std;

int hint_user_on_failure(char *argv[]);

enum StreamType {COLOR, INFRARED, DEPTH, DEPTH_WITH_INFRARED};

//user supplied input
struct input_args
{
	int width;
	int height;
	int framerate;
	int seconds;
	float depth_units;
	StreamType stream;
};

bool main_loop_color_infrared(const input_args& input, rs2::pipeline& realsense, nhve *streamer);
bool main_loop_depth(const input_args& input, rs2::pipeline& realsense, nhve *streamer);
bool main_loop_depth_with_infrared(const input_args& input, rs2::pipeline& realsense, nhve *streamer);
void init_realsense(rs2::pipeline& pipe, const input_args& input);
int process_user_input(int argc, char* argv[], input_args* input, nhve_net_config *net_config, nhve_hw_config *hw_config);

const uint16_t P010LE_MAX = 0xFFC0; //in binary 10 ones followed by 6 zeroes

int main(int argc, char* argv[])
{
	//nhve_hw_config {WIDTH, HEIGHT, FRAMERATE, DEVICE, ENCODER, PIXEL_FORMAT, PROFILE, BFRAMES, BITRATE, QP, GOP_SIZE};
	//prepare NHVE Network Hardware Video Encoder
	struct nhve_net_config net_config = {0};
	struct nhve_hw_config hw_config = {0};
	struct nhve *streamer;

	struct input_args user_input = {0};
	user_input.depth_units=0.0001f; //optionally override with user input

	rs2::pipeline realsense;

	if(process_user_input(argc, argv, &user_input, &net_config, &hw_config) < 0)
		return 1;

	init_realsense(realsense, user_input);

	if( (streamer = nhve_init(&net_config, &hw_config, 1)) == NULL )
		return hint_user_on_failure(argv);

	bool status = false;

	if(user_input.stream == COLOR || user_input.stream == INFRARED)
		status = main_loop_color_infrared(user_input, realsense, streamer);
	if(user_input.stream == DEPTH)
		status = main_loop_depth(user_input, realsense, streamer);
	if(user_input.stream == DEPTH_WITH_INFRARED)
		status = main_loop_depth_with_infrared(user_input, realsense, streamer);

	nhve_close(streamer);

	if(status)
		cout << "Finished successfully." << endl;

	return 0;
}

//true on success, false on failure
bool main_loop_color_infrared(const input_args& input, rs2::pipeline& realsense, nhve *streamer)
{
	const int frames = input.seconds * input.framerate;
	int f;
	nhve_frame frame = {0};
	uint8_t *color_data = NULL; //data of dummy color plane for NV12

	for(f = 0; f < frames; ++f)
	{
		rs2::frameset frameset = realsense.wait_for_frames();

		rs2::video_frame video_frame = input.stream == COLOR ? frameset.get_color_frame() : frameset.get_infrared_frame(1);

		if(!(input.stream == COLOR) && !color_data)
		{   //prepare dummy color plane for NV12 format, half the size of Y
			//we can't alloc it in advance, this is the first time we know realsense stride
			int size = video_frame.get_stride_in_bytes()*video_frame.get_height()/2;
			color_data = new uint8_t[size];
			memset(color_data, 128, size);
		}

		frame.linesize[0] =  video_frame.get_stride_in_bytes();
		frame.data[0] = (uint8_t*) video_frame.get_data();

		//if we are streaming infrared we have 2 planes (luminance and color)
		frame.linesize[1] = (input.stream == COLOR) ? 0 : frame.linesize[0];
		frame.data[1] = color_data; //dummy color plane for infrared

		if(nhve_send(streamer, f, &frame) != NHVE_OK)
		{
			cerr << "failed to send" << endl;
			break;
		}
	}

	//flush the streamer by sending NULL frame
	nhve_send(streamer, f, NULL);

	delete [] color_data;

	//all the requested frames processed?
	return f==frames;
}

//true on success, false on failure
bool main_loop_depth(const input_args& input, rs2::pipeline& realsense, nhve *streamer)
{
	const int frames = input.seconds * input.framerate;
	int f;
	nhve_frame frame = {0};
	uint16_t *color_data = NULL; //data of dummy color plane for P010LE

	for(f = 0; f < frames; ++f)
	{
		rs2::frameset frameset = realsense.wait_for_frames();
		rs2::depth_frame depth = frameset.get_depth_frame();

		const int w = depth.get_width();
		const int h = depth.get_height();
		const int stride=depth.get_stride_in_bytes();

		if(!color_data)
		{  //prepare dummy color plane for P010LE format, half the size of Y
			//we can't alloc it in advance, this is the first time we know realsense stride
			//the stride will be at least width * 2 (Realsense Z16, VAAPI P010LE)
			color_data = new uint16_t[stride/2*h/2];
			for(int i=0;i<w*h/2;++i)
				color_data[i] = UINT16_MAX / 2; //dummy middle value for U/V, equals 128 << 8, equals 32768
		}

		//supply realsense frame data as ffmpeg frame data
		frame.linesize[0] = frame.linesize[1] =  stride; //the stride of Y and interleaved UV is equal
		frame.data[0] = (uint8_t*) depth.get_data();
		frame.data[1] = (uint8_t*) color_data;

		if(nhve_send(streamer, f, &frame) != NHVE_OK)
		{
			cerr << "failed to send" << endl;
			break;
		}
	}

	//flush the streamer by sending NULL frame
	nhve_send(streamer, f, NULL);

	delete [] color_data;

	//all the requested frames processed?
	return f==frames;
}

//true on success, false on failure
bool main_loop_depth_with_infrared(const input_args& input, rs2::pipeline& realsense, nhve *streamer)
{
	const int frames = input.seconds * input.framerate;
	int f;
	nhve_frame frame = {0};

	for(f = 0; f < frames; ++f)
	{
		rs2::frameset frameset = realsense.wait_for_frames();
		rs2::depth_frame depth = frameset.get_depth_frame();
		rs2::video_frame video = frameset.get_infrared_frame(1);

		const int w = depth.get_width();
		const int h = depth.get_height();
		const int stride=depth.get_stride_in_bytes();
		const int ir_stride=video.get_stride_in_bytes();

		assert(video.get_width() == ir_stride && 2*ir_stride == stride);

		//supply realsense frame data as ffmpeg frame data
		frame.linesize[0] = frame.linesize[1] =  stride; //the stride of Y and interleaved UV is equal
		frame.data[0] = (uint8_t*) depth.get_data();
		frame.data[1] = (uint8_t*) video.get_data();

		if(nhve_send(streamer, f, &frame) != NHVE_OK)
		{
			cerr << "failed to send" << endl;
			break;
		}
	}

	//flush the streamer by sending NULL frame
	nhve_send(streamer, f, NULL);

	//all the requested frames processed?
	return f==frames;
}


void init_realsense(rs2::pipeline& pipe, const input_args& input)
{
	rs2::config cfg;

	if(input.stream == COLOR)
		cfg.enable_stream(RS2_STREAM_COLOR, input.width, input.height, RS2_FORMAT_YUYV, input.framerate);
	else if(input.stream == INFRARED || input.stream == DEPTH_WITH_INFRARED)
	{// depth stream seems to be required for infrared to work
		cfg.enable_stream(RS2_STREAM_DEPTH, input.width, input.height, RS2_FORMAT_Z16, input.framerate);
		cfg.enable_stream(RS2_STREAM_INFRARED, 1, input.width, input.height, RS2_FORMAT_Y8, input.framerate);
	}
	else if(input.stream == DEPTH)
		cfg.enable_stream(RS2_STREAM_DEPTH, input.width, input.height, RS2_FORMAT_Z16, input.framerate);

	rs2::pipeline_profile profile = pipe.start(cfg);

	if(input.stream != DEPTH && input.stream != DEPTH_WITH_INFRARED)
		return;

	rs2::depth_sensor depth_sensor = profile.get_device().first<rs2::depth_sensor>();

	try
	{
		depth_sensor.set_option(RS2_OPTION_DEPTH_UNITS, input.depth_units);
	}
	catch(const exception &)
	{
		rs2::option_range range = depth_sensor.get_option_range(RS2_OPTION_DEPTH_UNITS);
		cerr << "failed to set depth units to " << input.depth_units << " (range is " << range.min << "-" << range.max << ")" << endl;
		throw;
	}

	cout << "Setting realsense depth units to " << input.depth_units << endl;
	cout << "This will result in:" << endl;
	cout << "-range " << input.depth_units * P010LE_MAX << " m" << endl;
	cout << "-precision " << input.depth_units*64.0f << " m (" << input.depth_units*64.0f*1000 << " mm)" << endl;

	try
	{
		 rs400::advanced_mode advanced = profile.get_device();
		 pipe.stop(); //workaround the problem with setting advanced_mode on running stream
		 STDepthTableControl depth_table = advanced.get_depth_table();
		 depth_table.depthClampMax = P010LE_MAX;
		 advanced.set_depth_table(depth_table);
		 profile = pipe.start(cfg);
	}
	catch(const exception &)
	{
		cerr << "failed to set depth clamp max (rs400:advanced_mode)";
		throw;
	}

	cout << "Clamping range at " << input.depth_units * P010LE_MAX << " m" << endl;

	rs2::video_stream_profile depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
	rs2_intrinsics i = depth_stream.get_intrinsics();

	cout << "The camera intrinsics:" << endl;
	cout << "-width=" << i.width << " height=" << i.height << " ppx=" << i.ppx << " ppy=" << i.ppy << " fx=" << i.fx << " fy=" << i.fy << endl;
	cout << "-distortion model " << i.model << " [" <<
		i.coeffs[0] << "," << i.coeffs[2] << "," << i.coeffs[3] << "," << i.coeffs[4] << "]" << endl;
}

int process_user_input(int argc, char* argv[], input_args* input, nhve_net_config *net_config, nhve_hw_config *hw_config)
{
	if(argc < 8)
	{
		cerr << "Usage: " << argv[0] << " <host> <port> <color/ir/depth/depth+ir> <width> <height> <framerate> <seconds> [device] [bitrate] [depth units]" << endl;
		cerr << endl << "examples: " << endl;
		cerr << argv[0] << " 127.0.0.1 9766 color 640 360 30 5" << endl;
		cerr << argv[0] << " 127.0.0.1 9766 infrared 640 360 30 5" << endl;
		cerr << argv[0] << " 127.0.0.1 9766 depth 640 360 30 5" << endl;
		cerr << argv[0] << " 127.0.0.1 9766 color 640 360 30 5 /dev/dri/renderD128" << endl;
		cerr << argv[0] << " 127.0.0.1 9766 infrared 640 360 30 5 /dev/dri/renderD128" << endl;
		cerr << argv[0] << " 127.0.0.1 9766 depth 640 360 30 5 /dev/dri/renderD128" << endl;
		cerr << argv[0] << " 127.0.0.1 9766 depth+ir 640 360 30 5 /dev/dri/renderD128" << endl;
		cerr << argv[0] << " 192.168.0.125 9766 color 640 360 30 50 /dev/dri/renderD128 500000" << endl;
		cerr << argv[0] << " 127.0.0.1 9768 depth 848 480 30 50 /dev/dri/renderD128 2000000" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 depth 848 480 30 500 /dev/dri/renderD128 2000000 0.0001" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 depth 848 480 30 500 /dev/dri/renderD128 2000000 0.00005" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 depth 848 480 30 500 /dev/dri/renderD128 2000000 0.000025" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 depth 848 480 30 500 /dev/dri/renderD128 2000000 0.0000125" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 depth+ir 848 480 30 500 /dev/dri/renderD128 2000000 0.0000125" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 depth+ir 848 480 30 500 /dev/dri/renderD128 8000000 0.00003125f" << endl;

		return -1;
	}

	net_config->ip = argv[1];
	net_config->port = atoi(argv[2]);

	char c = argv[3][0]; //color, infrared, depth
	if(c == 'c') input->stream = COLOR;
	else if(c == 'i') input->stream = INFRARED;
	else if(c == 'd')
		input->stream = (strlen(argv[3]) <= 5) ? DEPTH : DEPTH_WITH_INFRARED;
	else
	{
		cerr << "unknown stream: " << argv[3];
		return -1;
	}

	//native format of Realsense RGB sensor is YUYV (YUY2, YUYV422)
	//see https://github.com/IntelRealSense/librealsense/issues/3042

	//on the other hand native format for VAAPI is nv12
	//we will match:
	//- Realsense RGB sensor YUYV with VAAPI YUYV422 (same format)
	//- Realsense IR sensor Y8 with VAAPI NV12 (luminance plane with dummy color plane)
	//this way we always have optimal format at least on one side and hardware conversion on other

	//for depth encoding we use 10 bit P010LE pixel format
	//that can be directly matched with Realsense output as P016LE Y plane
	//with precision/range trade-off controlled by Realsense Depth Units
	//for explanation see:
	//https://github.com/bmegli/realsense-depth-to-vaapi-hevc10/wiki/How-it-works

	//for depth with infrared encoding we use 10 bit P010LE pixel format
	//with depth encoded as above
	//the infrared plane is encoded in chroma U/V plane
	//for explanation see:
	//https://github.com/bmegli/hardware-video-streaming/issues/2

	hw_config->profile = FF_PROFILE_HEVC_MAIN;

	if(input->stream == COLOR)
		hw_config->pixel_format = "yuyv422";
	else if(input->stream == INFRARED)
		hw_config->pixel_format = "nv12";
	else //DEPTH, DEPTH_WITH_INFRARED
	{
		hw_config->pixel_format = "p010le";
		hw_config->profile = FF_PROFILE_HEVC_MAIN_10;
	}

	hw_config->encoder = "hevc_vaapi";
	hw_config->width = input->width = atoi(argv[4]);
	hw_config->height = input->height = atoi(argv[5]);
	hw_config->framerate = input->framerate = atoi(argv[6]);

	input->seconds = atoi(argv[7]);

	hw_config->device = argv[8]; //NULL as last argv argument, or device path

	if(argc > 9)
		hw_config->bit_rate = atoi(argv[9]);

	//optionally set qp instead of bit_rate for CQP mode
	//hw_config->qp = ...

	//optionally set gop_size (determines keyframes period)
	//hw_config->gop_size = ...;

	//set highest quality and slowest encoding
	//this adds around 3 ms and 10% GPU usage on my 2017 KabyLake
	//with 848x480 HEVC Main10 encoding
	hw_config->compression_level = 1;

	if(argc > 10)
		input->depth_units = strtof(argv[10], NULL);

	return 0;
}

int hint_user_on_failure(char *argv[])
{
	cerr << "unable to initalize, try to specify device e.g:" << endl << endl;
	cerr << argv[0] << " 127.0.0.1 9766 color 640 360 30 5 /dev/dri/renderD128" << endl;
	cerr << argv[0] << " 127.0.0.1 9766 infrared 640 360 30 5 /dev/dri/renderD128" << endl;
	cerr << argv[0] << " 127.0.0.1 9766 depth 640 360 30 5 /dev/dri/renderD128" << endl;
	cerr << argv[0] << " 127.0.0.1 9766 depth+infrared 640 360 30 5 /dev/dri/renderD128" << endl;
	return -1;
}
