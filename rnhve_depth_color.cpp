/*
 * Realsense Network Hardware Video Encoder
 *
 * Realsense hardware encoded UDP HEVC aligned multi-streaming
 * - depth (Main10) + color (Main)
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
#include <streambuf> //loading json config
#include <iostream>
#include <math.h>

#include <chrono> //benchmark related

using namespace std;

int hint_user_on_failure(char *argv[]);

//encoding index, alignment direction
enum Stream {Depth = 0, Color = 1};

//user supplied input
struct input_args
{
	int depth_width;
	int depth_height;
	int color_width;
	int color_height;
	int framerate;
	int seconds;
	float depth_units;
	Stream align_to;
	std::string json;
	bool needs_postprocessing;
};

bool main_loop(const input_args& input, rs2::pipeline& realsense, nhve *streamer);
void process_depth_data(const input_args &input, rs2::depth_frame &depth);

void init_realsense(rs2::pipeline& pipe, input_args& input);
void init_realsense_depth(rs2::pipeline& pipe, const rs2::config &cfg, input_args& input);
void print_intrinsics(const rs2::pipeline_profile& profile, rs2_stream stream);

int process_user_input(int argc, char* argv[], input_args* input, nhve_net_config *net_config, nhve_hw_config *hw_config);

const uint16_t P010LE_MAX = 0xFFC0; //in binary 10 ones followed by 6 zeroes

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

	rs2::align aligner( (input.align_to == Color) ? RS2_STREAM_COLOR : RS2_STREAM_DEPTH);

	for(f = 0; f < frames; ++f)
	{
		rs2::frameset frameset = realsense.wait_for_frames();
		
		auto start = chrono::high_resolution_clock::now();
		
		frameset = aligner.process(frameset);

		rs2::depth_frame depth = frameset.get_depth_frame();
		rs2::video_frame color = frameset.get_color_frame();

		const int h = depth.get_height();
		const int depth_stride=depth.get_stride_in_bytes();

		auto pproc_start = chrono::high_resolution_clock::now();
		//L515 doesn't support setting depth units and clamping
		if(input.needs_postprocessing)
			process_depth_data(input, depth);
		auto pproc_stop = chrono::high_resolution_clock::now();

		if(!depth_uv)
		{  //prepare dummy color plane for P010LE format, half the size of Y
			//we can't alloc it in advance, this is the first time we know realsense stride
			//the stride will be at least width * 2 (Realsense Z16, VAAPI P010LE)
			depth_uv = new uint16_t[depth_stride/2*h/2];

			for(int i=0;i<depth_stride/2*h/2;++i)
				depth_uv[i] = UINT16_MAX / 2; //dummy middle value for U/V, equals 128 << 8, equals 32768
		}

		//supply realsense frame data as ffmpeg frame data
		frame[0].linesize[0] = frame[0].linesize[1] =  depth_stride; //the strides of Y and UV are equal
		frame[0].data[0] = (uint8_t*) depth.get_data();
		frame[0].data[1] = (uint8_t*) depth_uv;

		frame[1].linesize[0] = color.get_stride_in_bytes();
		frame[1].data[0] = (uint8_t*) color.get_data();

		if(nhve_send(streamer, &frame[0], 0) != NHVE_OK)
		{
			cerr << "failed to send" << endl;
			break;
		}

		if(nhve_send(streamer, &frame[1], 1) != NHVE_OK)
		{
			cerr << "failed to send" << endl;
			break;
		}
		
		auto stop = chrono::high_resolution_clock::now();
		
		cout << "duration:" << chrono::duration_cast<chrono::milliseconds>(stop - start).count() << " ms pproc "
		     << chrono::duration_cast<chrono::milliseconds>(pproc_stop - pproc_start).count() << " ms" << endl;
	}

	//flush the streamer by sending NULL frame
	nhve_send(streamer, NULL, 0);
	nhve_send(streamer, NULL, 1);

	delete [] depth_uv;

	//all the requested frames processed?
	return f==frames;
}

void process_depth_data(const input_args &input, rs2::depth_frame &depth)
{
	const int half_stride = depth.get_stride_in_bytes()/2;
	const int height = depth.get_height();

	const float depth_units_set = depth.get_units();
	const float multiplier = depth_units_set / input.depth_units;

	//note - we process data in place rather than making a copy
	uint16_t* data = (uint16_t*)depth.get_data();

	for(int i = 0;i < half_stride * height; ++i)
	{
		uint32_t val = data[i] * multiplier;
		data[i] = val <= P010LE_MAX ? val : 0;
	}
}

void init_realsense(rs2::pipeline& pipe, input_args& input)
{
	rs2::config cfg;
	//use YUYV/RGBA when aligning to color/depth (aligning YUYV not possible in librealsense)
	rs2_format color_format = (input.align_to == Color) ? RS2_FORMAT_YUYV : RS2_FORMAT_RGBA8;

	cfg.enable_stream(RS2_STREAM_DEPTH, input.depth_width, input.depth_height, RS2_FORMAT_Z16, input.framerate);
	cfg.enable_stream(RS2_STREAM_COLOR, input.color_width, input.color_height, color_format, input.framerate);

	rs2::pipeline_profile profile = pipe.start(cfg);

	init_realsense_depth(pipe, cfg, input);

	if(input.align_to == Color)
		print_intrinsics(profile, RS2_STREAM_COLOR);
	else
		print_intrinsics(profile, RS2_STREAM_DEPTH);
}

void init_realsense_depth(rs2::pipeline& pipe, const rs2::config &cfg, input_args& input)
{
	rs2::pipeline_profile profile = pipe.get_active_profile();

	rs2::depth_sensor depth_sensor = profile.get_device().first<rs2::depth_sensor>();

	if(!input.json.empty())
	{
		cout << "loading settings from json:" << endl << input.json  << endl;
		auto serializable  = profile.get_device().as<rs2::serializable_device>();
		serializable.load_json(input.json);
	}

	bool supports_depth_units = depth_sensor.supports(RS2_OPTION_DEPTH_UNITS) &&
										!depth_sensor.is_option_read_only(RS2_OPTION_DEPTH_UNITS);

	float depth_unit_set = input.depth_units;

	if(supports_depth_units)
	{
		try
		{
			depth_sensor.set_option(RS2_OPTION_DEPTH_UNITS, input.depth_units);
			depth_unit_set = depth_sensor.get_option(RS2_OPTION_DEPTH_UNITS);
			if(depth_unit_set != input.depth_units)
				cerr << "WARNING - device corrected depth units to value: " << depth_unit_set << endl;
		}
		catch(const exception &)
		{
			rs2::option_range range = depth_sensor.get_option_range(RS2_OPTION_DEPTH_UNITS);
			cerr << "failed to set depth units to " << input.depth_units << " (range is " << range.min << "-" << range.max << ")" << endl;
			throw;
		}
	}
	else
	{
		cerr << "WARNING - device doesn't support setting depth units!" << endl;
		input.needs_postprocessing = true;
	}

	cout << (supports_depth_units ? "Setting" : "Simulating") <<
		" realsense depth units: " << depth_unit_set << endl;
	cout << "This will result in:" << endl;
	cout << "-range " << input.depth_units * P010LE_MAX << " m" << endl;
	cout << "-precision " << input.depth_units*64.0f << " m (" << input.depth_units*64.0f*1000 << " mm)" << endl;

	bool supports_advanced_mode = depth_sensor.supports(RS2_CAMERA_INFO_ADVANCED_MODE);

	if(supports_advanced_mode)
	{
		 rs400::advanced_mode advanced = profile.get_device();
		 pipe.stop(); //workaround the problem with setting advanced_mode on running stream
		 STDepthTableControl depth_table = advanced.get_depth_table();
		 depth_table.depthClampMax = P010LE_MAX;
		 advanced.set_depth_table(depth_table);
		 profile = pipe.start(cfg);
	}
	else
	{
		cerr << "WARNING - device doesn't support advanced mode depth clamping!" << endl;
		input.needs_postprocessing = true;
	}
	cout << (supports_advanced_mode ?  "Clamping" : "Simulating clamping") <<
	" range at " << input.depth_units * P010LE_MAX << " m" << endl;
}

void print_intrinsics(const rs2::pipeline_profile& profile, rs2_stream stream)
{
	rs2::video_stream_profile stream_profile = profile.get_stream(stream).as<rs2::video_stream_profile>();
	rs2_intrinsics i = stream_profile.get_intrinsics();

	const float rad2deg = 180.0f / M_PI;
	float hfov = 2 * atan(i.width / (2*i.fx)) * rad2deg;
	float vfov = 2 * atan(i.height / (2*i.fy)) * rad2deg;

	cout << "The camera intrinsics (" << stream << "):" << endl;
	cout << "-width=" << i.width << " height=" << i.height << " hfov=" << hfov << " vfov=" << vfov << endl <<
           "-ppx=" << i.ppx << " ppy=" << i.ppy << " fx=" << i.fx << " fy=" << i.fy << endl;
	cout << "-distortion model " << i.model << " [" <<
		i.coeffs[0] << "," << i.coeffs[2] << "," << i.coeffs[3] << "," << i.coeffs[4] << "]" << endl;
}

int process_user_input(int argc, char* argv[], input_args* input, nhve_net_config *net_config, nhve_hw_config *hw_config)
{
	if(argc < 10)
	{
		cerr << "Usage: " << argv[0] << endl
		     << "       <host> <port>" << endl //1, 2
		     << "       <color/depth> # alignment direction" << endl //3
		     << "       <width_depth> <height_depth> <width_color> <height_color>" << endl //4, 5, 6, 7
			  << "       <framerate> <seconds>" << endl //8, 9
			  << "       [device] [bitrate_depth] [bitrate_color] [depth units] [json]" << endl; //10, 11, 12, 13, 14

		cerr << endl << "examples: " << endl;
		cerr << argv[0] << " 127.0.0.1 9766 color 640 360 640 360 30 5" << endl;
		cerr << argv[0] << " 127.0.0.1 9766 color 640 360 640 360 30 5 /dev/dri/renderD128" << endl;
		cerr << argv[0] << " 192.168.0.125 9766 color 640 360 640 360 30 50 /dev/dri/renderD128 4000000 1000000" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 color 848 480 848 480 30 500 /dev/dri/renderD128 8000000 1000000 0.0001" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 color 848 480 848 480 30 500 /dev/dri/renderD128 8000000 1000000 0.00005" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 color 848 480 848 480 30 500 /dev/dri/renderD128 8000000 1000000 0.000025" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 color 848 480 848 480 30 500 /dev/dri/renderD128 8000000 1000000 0.0000125" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 depth 848 480 848 480 30 500 /dev/dri/renderD128 8000000 1000000 0.0000125" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 color 848 480 848 480 30 500 /dev/dri/renderD128 8000000 1000000 0.00003125" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 depth 848 480 1280 720 30 500 /dev/dri/renderD128 8000000 1000000 0.00003125" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 depth 640 480 1280 720 30 500 /dev/dri/renderD128 8000000 1000000 0.0000390625 my_config.json" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 color 640 480 1280 720 30 500 /dev/dri/renderD128 8000000 1000000 0.0000390625 my_config.json" << endl;

		return -1;
	}

	net_config->ip = argv[1];
	net_config->port = atoi(argv[2]);

	char c = argv[3][0]; //color, depth
	if(c == 'c') input->align_to = Color;
	else if(c == 'd') input->align_to = Depth;
	else
	{
		cerr << "unnkown alignment target '" << argv[3] <<"', valid targets: 'color', 'depth'" << endl;
		return -1;
	}

	cout << "Aligning to " << ((input->align_to == Color) ? "color" : "depth") << endl;

	//for depth encoding we use 10 bit P010LE pixel format
	//that can be directly matched with Realsense output as P016LE Y plane
	//with precision/range trade-off controlled by Realsense Depth Units
	//for explanation see:
	//https://github.com/bmegli/realsense-depth-to-vaapi-hevc10/wiki/How-it-works

	//native format of Realsense RGB sensor is YUYV (YUY2, YUYV422)
	//see https://github.com/IntelRealSense/librealsense/issues/3042
	//however librealsense is unable to align color with YUYV to depth
	//see https://github.com/IntelRealSense/librealsense/blob/master/src/proc/align.cpp#L123

	//we will match:
	//- Realsense RGB sensor YUYV with VAAPI YUYV422 (same format) when aligning to color
	//- Realsense RGB sensor RGBA8 with VAAPI RGB0 (alpha ignored) when aligning to depth

	input->depth_width = atoi(argv[4]);
	input->depth_height = atoi(argv[5]);
	input->color_width = atoi(argv[6]);
	input->color_height = atoi(argv[7]);

	//DEPTH hardware encoding configuration
	hw_config[Depth].profile = FF_PROFILE_HEVC_MAIN_10;
	hw_config[Depth].pixel_format = "p010le";
	hw_config[Depth].encoder = "hevc_vaapi";

	//dimmensions will match alignment target
	hw_config[Depth].width = (input->align_to == Color) ? input->color_width : input->depth_width;
	hw_config[Depth].height = (input->align_to == Color) ? input->color_height : input->depth_height;

	hw_config[Depth].framerate = input->framerate = atoi(argv[8]);

	input->seconds = atoi(argv[9]);

	hw_config[Depth].device = argv[10]; //NULL as last argv argument, or device path

	if(argc > 11)
		hw_config[Depth].bit_rate = atoi(argv[11]);

	//COLOR hardware encoding configuration
	hw_config[Color].profile = FF_PROFILE_HEVC_MAIN;
	//use YUYV/RGBA when aligning to color/depth (aligning YUYV not possible in librealsense)
	hw_config[Color].pixel_format = (input->align_to == Color) ? "yuyv422" : "rgb0";
	hw_config[Color].encoder = "hevc_vaapi";

	//dimmensions will match alignment target
	hw_config[Color].width = (input->align_to == Color) ? input->color_width : input->depth_width;
	hw_config[Color].height = (input->align_to == Color) ? input->color_height : input->depth_height;

	hw_config[Color].framerate = input->framerate = atoi(argv[8]);

	hw_config[Color].device = argv[10]; //NULL as last argv argument, or device path

	if(argc > 12)
		hw_config[Color].bit_rate = atoi(argv[12]);

	//set highest quality and slowest encoding
	//this adds around 3 ms and 10% GPU usage on my 2017 KabyLake
	//with 848x480 HEVC Main10 encoding
	hw_config[Depth].compression_level = 1;
	hw_config[Color].compression_level = 0;

	//optionally set qp instead of bit_rate for CQP mode
	//hw_config[].qp = ...

	//optionally set gop_size (determines keyframes period)
	//hw_config[].gop_size = ...;

	if(argc > 13)
		input->depth_units = strtof(argv[13], NULL);

	if(argc > 14)
	{
		ifstream file(argv[14]);
		if(!file)
		{
			cerr << "unable to open file " << argv[14] << endl;
			return -1;
		}

		input->json = string((istreambuf_iterator<char>(file)), istreambuf_iterator<char>());
	}

	input->needs_postprocessing = false;

	return 0;
}

int hint_user_on_failure(char *argv[])
{
	cerr << "unable to initalize, try to specify device e.g:" << endl << endl;
	cerr << argv[0] << " 127.0.0.1 9766 color 640 360 640 360 30 5 /dev/dri/renderD128" << endl;
	cerr << argv[0] << " 127.0.0.1 9766 color 640 360 640 360 30 5 /dev/dri/renderD129" << endl;
	return -1;
}
