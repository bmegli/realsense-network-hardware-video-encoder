/*
 * Realsense Network Hardware Video Encoder
 *
 * Realsense hardware encoded UDP HEVC multi-streaming
 * - depth (Main10) + infrared (Main)
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

//user supplied input
struct input_args
{
	int width;
	int height;
	int framerate;
	int seconds;
	float depth_units;
};

bool main_loop(const input_args& input, rs2::pipeline& realsense, nhve *streamer);
void init_realsense(rs2::pipeline& pipe, const input_args& input);
int process_user_input(int argc, char* argv[], input_args* input, nhve_net_config *net_config, nhve_hw_config *hw_config);

const uint16_t P010LE_MAX = 0xFFC0; //in binary 10 ones followed by 6 zeroes

const int DEPTH = 0; //depth hardware encoder index
const int IR = 1; //ir hardware encoder index

int main(int argc, char* argv[])
{
	//prepare NHVE Network Hardware Video Encoder
	struct nhve_net_config net_config = {0};
	struct nhve_hw_config hw_configs[2] = { {0}, {0} };
	struct nhve *streamer;

	struct input_args user_input = {0};
	user_input.depth_units=0.0001f; //optionally override with user input

	rs2::pipeline realsense;

	if(process_user_input(argc, argv, &user_input, &net_config, hw_configs) < 0)
		return 1;

	init_realsense(realsense, user_input);

	if( (streamer = nhve_init(&net_config, hw_configs, 2)) == NULL )
		return hint_user_on_failure(argv);

	bool status = main_loop(user_input, realsense, streamer);

	nhve_close(streamer);

	if(status)
		cout << "Finished successfully." << endl;

	return 0;
}

//true on success, false on failure
bool main_loop(const input_args& input, rs2::pipeline& realsense, nhve *streamer)
{
	const int frames = input.seconds * input.framerate;
	int f;
	nhve_frame frame[2] = { {0}, {0} };

	uint16_t *depth_uv = NULL; //data of dummy color plane for P010LE
	uint8_t *ir_uv = NULL; //data of dummy color plane for NV12

	for(f = 0; f < frames; ++f)
	{
		rs2::frameset frameset = realsense.wait_for_frames();
		rs2::depth_frame depth = frameset.get_depth_frame();
		rs2::video_frame ir = frameset.get_infrared_frame(1);

		const int w = depth.get_width();
		const int h = depth.get_height();
		const int depth_stride=depth.get_stride_in_bytes();
		const int ir_stride=ir.get_stride_in_bytes();

		if(!depth_uv || !ir_uv)
		{  //prepare dummy color plane for P010LE format, half the size of Y
			//we can't alloc it in advance, this is the first time we know realsense stride
			//the stride will be at least width * 2 (Realsense Z16, VAAPI P010LE)
			depth_uv = new uint16_t[depth_stride/2*h/2];

			for(int i=0;i<depth_stride/2*h/2;++i)
				depth_uv[i] = UINT16_MAX / 2; //dummy middle value for U/V, equals 128 << 8, equals 32768

			//prepare dummy color plane for NV12 format, half the size of Y
			ir_uv = new uint8_t[ir_stride * h /2];
			memset(ir_uv, 128, ir_stride * h /2);
		}

		//supply realsense depth frame data as ffmpeg frame data
		frame[0].linesize[0] = frame[0].linesize[1] =  depth_stride; //the strides of Y and UV are equal
		frame[0].data[0] = (uint8_t*) depth.get_data();
		frame[0].data[1] = (uint8_t*) depth_uv;

		if(nhve_send(streamer, &frame[0], 0) != NHVE_OK)
		{
			cerr << "failed to send" << endl;
			break;
		}

		//supply realsense infrared frame data as ffmpeg frame data
		frame[1].linesize[0] = frame[1].linesize[1] =  ir_stride; //the strides of Y and UV are equal
		frame[1].data[0] = (uint8_t*) ir.get_data();
		frame[1].data[1] = ir_uv;

		if(nhve_send(streamer, &frame[1], 1) != NHVE_OK)
		{
			cerr << "failed to send" << endl;
			break;
		}
	}

	//flush the hardware by sending NULL frames
	nhve_send(streamer, NULL, 0);
	nhve_send(streamer, NULL, 1);

	delete [] depth_uv;
	delete [] ir_uv;

	//all the requested frames processed?
	return f==frames;
}

void init_realsense(rs2::pipeline& pipe, const input_args& input)
{
	rs2::config cfg;

	cfg.enable_stream(RS2_STREAM_DEPTH, input.width, input.height, RS2_FORMAT_Z16, input.framerate);
	cfg.enable_stream(RS2_STREAM_INFRARED, 1, input.width, input.height, RS2_FORMAT_Y8, input.framerate);

	rs2::pipeline_profile profile = pipe.start(cfg);

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
	if(argc < 7)
	{
		cerr << "Usage: " << argv[0] << " <host> <port> <width> <height> <framerate> <seconds> [device] [bitrate_depth] [bitrate_ir] [depth units]" << endl;
		cerr << endl << "examples: " << endl;
		cerr << argv[0] << " 127.0.0.1 9766 640 360 30 5" << endl;
		cerr << argv[0] << " 127.0.0.1 9766 640 360 30 5 /dev/dri/renderD128" << endl;
		cerr << argv[0] << " 192.168.0.125 9766 640 360 30 50 /dev/dri/renderD128 4000000 1000000" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 848 480 30 500 /dev/dri/renderD128 8000000 1000000 0.0001" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 848 480 30 500 /dev/dri/renderD128 8000000 1000000 0.00005" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 848 480 30 500 /dev/dri/renderD128 8000000 1000000 0.000025" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 848 480 30 500 /dev/dri/renderD128 8000000 1000000 0.0000125" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 848 480 30 500 /dev/dri/renderD128 8000000 1000000 0.0000125" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 848 480 30 500 /dev/dri/renderD128 8000000 1000000 0.00003125f" << endl;

		return -1;
	}

	net_config->ip = argv[1];
	net_config->port = atoi(argv[2]);

	//for depth encoding we use 10 bit P010LE pixel format
	//that can be directly matched with Realsense output as P016LE Y plane
	//with precision/range trade-off controlled by Realsense Depth Units
	//for explanation see:
	//https://github.com/bmegli/realsense-depth-to-vaapi-hevc10/wiki/How-it-works

	//for infrared encoding we use native VAAPI 8 bit NV12 pixel format
	//which Y plane can be directly matched with Realsense Y8 infrared data

	//for both cases (depth and infrared) we use native hardware formats
	//and there is no need for data processing on the host CPU

	//DEPTH hardware encoding configuration
	hw_config[DEPTH].profile = FF_PROFILE_HEVC_MAIN_10;
	hw_config[DEPTH].pixel_format = "p010le";
	hw_config[DEPTH].encoder = "hevc_vaapi";
	hw_config[DEPTH].width = input->width = atoi(argv[3]);
	hw_config[DEPTH].height = input->height = atoi(argv[4]);
	hw_config[DEPTH].framerate = input->framerate = atoi(argv[5]);

	input->seconds = atoi(argv[6]);

	hw_config[DEPTH].device = argv[7]; //NULL as last argv argument, or device path

	if(argc > 8)
		hw_config[DEPTH].bit_rate = atoi(argv[8]);

	//INFRARED hardware encoding configuration
	hw_config[IR].profile = FF_PROFILE_HEVC_MAIN;
	hw_config[IR].pixel_format = "nv12";
	hw_config[IR].encoder = "hevc_vaapi";
	hw_config[IR].width = input->width = atoi(argv[3]);
	hw_config[IR].height = input->height = atoi(argv[4]);
	hw_config[IR].framerate = input->framerate = atoi(argv[5]);

	hw_config[IR].device = argv[7]; //NULL as last argv argument, or device path

	if(argc > 9)
		hw_config[IR].bit_rate = atoi(argv[9]);

	//set highest quality and slowest encoding
	//this adds around 3 ms and 10% GPU usage on my 2017 KabyLake
	//with 848x480 HEVC Main10 encoding
	hw_config[DEPTH].compression_level = 1;
	hw_config[IR].compression_level = 0;

	//optionally set qp instead of bit_rate for CQP mode
	//hw_config[].qp = ...

	//optionally set gop_size (determines keyframes period)
	//hw_config[].gop_size = ...;

	if(argc > 10)
		input->depth_units = strtof(argv[10], NULL);

	return 0;
}

int hint_user_on_failure(char *argv[])
{
	cerr << "unable to initalize, try to specify device e.g:" << endl << endl;
	cerr << argv[0] << " 127.0.0.1 9766 640 360 30 5 /dev/dri/renderD128" << endl;
	cerr << argv[0] << " 127.0.0.1 9766 640 360 30 5 /dev/dri/renderD129" << endl;
	return -1;
}
