/*
 * Realsense Network Hardware Video Encoder
 * (Realsense hardware encoded UDP H.264 streaming)
 *
 * Copyright 2019 (C) Bartosz Meglicki <meglickib@gmail.com>
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

#include <fstream>
#include <iostream>
using namespace std;

int hint_user_on_failure(char *argv[]);

//user supplied input
struct input_args
{
	int width;
	int height;
	int framerate;
	int seconds;
	bool stream_color; //true for color, infrared otherwise
};

bool main_loop(const input_args& input, rs2::pipeline& realsense, nhve *streamer);
void init_realsense(rs2::pipeline& pipe, const input_args& input);
int process_user_input(int argc, char* argv[], input_args* input, nhve_net_config *net_config, nhve_hw_config *hw_config);

int main(int argc, char* argv[])
{	
	//struct nhve_hw_config hw_config = {WIDTH, HEIGHT, FRAMERATE, DEVICE, PIXEL_FORMAT, PROFILE, BFRAMES, BITRATE};
	//prepare NHVE Network Hardware Video Encoder
	struct nhve_net_config net_config = {0};
	struct nhve_hw_config hw_config = {0};
	struct nhve *streamer;

	struct input_args user_input = {0};

	rs2::pipeline realsense;

	if(process_user_input(argc, argv, &user_input, &net_config, &hw_config) < 0)
		return 1;

	init_realsense(realsense, user_input);

	if( (streamer = nhve_init(&net_config, &hw_config)) == NULL )
		return hint_user_on_failure(argv);
	
	bool status=main_loop(user_input, realsense, streamer);

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
	nhve_frame frame = {0};
	uint8_t *color_data = NULL; //data of dummy color plane for NV12
		
	for(f = 0; f < frames; ++f)
	{
		rs2::frameset frameset = realsense.wait_for_frames();
		
		rs2::video_frame video_frame = input.stream_color ? frameset.get_color_frame() : frameset.get_infrared_frame(1);		
	
		if(!input.stream_color && !color_data)
		{   //prepare dummy color plane for NV12 format, half the size of Y
			//we can't alloc it in advance, this is the first time we know realsense stride
			int size = video_frame.get_stride_in_bytes()*video_frame.get_height()/2;
			color_data = new uint8_t[size];
			memset(color_data, 128, size);
		}
	
		frame.linesize[0] =  video_frame.get_stride_in_bytes();
		frame.data[0] = (uint8_t*) video_frame.get_data();
		
		//if we are streaming infrared we have 2 planes (luminance and color)
		frame.linesize[1] = input.stream_color ? 0 : frame.linesize[0]; 
		frame.data[1] = color_data; //dummy color plane for infrared

		frame.framenumber = f;

		if(nhve_send_frame(streamer, &frame) != NHVE_OK)
		{
			cerr << "failed to send" << endl;
			break;
		}				
	}
	
	//flush the streamer by sending NULL frame
	nhve_send_frame(streamer, NULL);
		
	delete [] color_data;

	//all the requested frames processed?
	return f==frames;
}

void init_realsense(rs2::pipeline& pipe, const input_args& input)
{
	rs2::config cfg;
	
	if(input.stream_color)
		cfg.enable_stream(RS2_STREAM_COLOR, input.width, input.height, RS2_FORMAT_YUYV, input.framerate);	
	else
	{// depth stream seems to be required for infrared to work
		cfg.enable_stream(RS2_STREAM_DEPTH, input.width, input.height, RS2_FORMAT_Z16, input.framerate);
		cfg.enable_stream(RS2_STREAM_INFRARED, 1, input.width, input.height, RS2_FORMAT_Y8, input.framerate);
	}

	rs2::pipeline_profile profile = pipe.start(cfg);
}

int process_user_input(int argc, char* argv[], input_args* input, nhve_net_config *net_config, nhve_hw_config *hw_config)
{
	if(argc < 8)
	{
		cerr << "Usage: " << argv[0] << " <host> <port> <color/ir> <width> <height> <framerate> <seconds> [device]" << endl;
		cerr << endl << "examples: " << endl;
		cerr << argv[0] << " 127.0.0.1 9766 color 640 360 30 5" << endl;
		cerr << argv[0] << " 127.0.0.1 9766 infrared 640 360 30 5" << endl;
		cerr << argv[0] << " 127.0.0.1 9766 color 640 360 30 5 /dev/dri/renderD128" << endl;
		cerr << argv[0] << " 127.0.0.1 9766 infrared 640 360 30 5 /dev/dri/renderD128" << endl;

		return -1;
	}

	net_config->ip = argv[1];
	net_config->port = atoi(argv[2]);
	
	input->stream_color = argv[3][0] == 'c';
	
	//native format of Realsense RGB sensor is YUYV (YUY2, YUYV422)
	//see https://github.com/IntelRealSense/librealsense/issues/3042

	//on the other hand native format for VAAPI is nv12
	//we will match:
	//- Realsense RGB sensor YUYV with VAAPI YUYV422 (same format)
	//- Realsense IR sensor Y8 with VAAPI NV12 (luminance plane with dummy color plane)
	//this way we always have optimal format at least on one side and hardware conversion on other
	hw_config->pixel_format = input->stream_color ? "yuyv422" : "nv12";

	hw_config->width = input->width = atoi(argv[4]);
	hw_config->height = input->height = atoi(argv[5]);
	hw_config->framerate = input->framerate = atoi(argv[6]);
	
	input->seconds = atoi(argv[7]);
	
	hw_config->device = argv[8]; //NULL as last argv argument, or device path
	
	return 0;
}
 
int hint_user_on_failure(char *argv[])
{
	cerr << "unable to initalize, try to specify device e.g:" << endl << endl;
	cerr << argv[0] << " 127.0.0.1 9766 color 640 360 30 5 /dev/dri/renderD128" << endl;
	cerr << argv[0] << " 127.0.0.1 9766 infrared 640 360 30 5 /dev/dri/renderD128" << endl;
	return -1;
}
