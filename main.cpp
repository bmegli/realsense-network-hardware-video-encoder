/*
 * Realsense Network Hardware Video Encoder - hardware encoded UDP streaming H.264
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
};

bool main_loop(const input_args& input, rs2::pipeline& realsense, nhve *streamer);
void init_realsense(rs2::pipeline& pipe, const input_args& input);
int process_user_input(int argc, char* argv[], input_args* input, nhve_net_config *net_config, nhve_hw_config *hw_config);

int main(int argc, char* argv[])
{	
	//struct nhve_net_config net_config = {IP, PORT};
	//struct nhve_hw_config hw_config = {WIDTH, HEIGHT, FRAMERATE, DEVICE, PIXEL_FORMAT, PROFILE, BFRAMES, BITRATE};
	//prepare NHVE Network Hardware Video Encoder
	struct nhve_net_config net_config = {0};
	struct nhve_hw_config hw_config = {0};
	struct nhve *streamer;

	//hw_config.pixel_format = "nv12";
	//native format of Realsense RGB sensor is YUYV (YUY2)
	hw_config.pixel_format = "yuyv422";

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
	int f, failed;
	nhve_frame frame = {0};
	uint8_t *color_data = NULL; //data of dummy color plane for NV12
		
	for(f = 0; f < frames; ++f)
	{
		rs2::frameset frameset = realsense.wait_for_frames();
		//rs2::video_frame ir_frame = frameset.get_infrared_frame(1);
		rs2::video_frame video_frame = frameset.get_color_frame();
		
	/*
		if(!color_data)
		{   //prepare dummy color plane for NV12 format, half the size of Y
			//we can't alloc it in advance, this is the first time we know realsense stride
			int size = ir_frame.get_stride_in_bytes()*ir_frame.get_height()/2;
			color_data = new uint8_t[size];
			memset(color_data, 128, size);
		}
	*/
		//frame.linesize[0] = frame.linesize[1] =  ir_frame.get_stride_in_bytes();
		frame.linesize[0] =  video_frame.get_stride_in_bytes();
		frame.data[0] = (uint8_t*) video_frame.get_data();

		frame.framenumber = f;

		if(nhve_send_frame(streamer, &frame) != NHVE_OK)
		{
			cerr << "failed to send" << endl;
			break;
		}				
	}
	
	//flush the streamer by sending NULL frame
	nhve_send_frame(streamer, NULL);
		
	//delete [] color_data;

	//all the requested frames processed?
	return f==frames;
}

void init_realsense(rs2::pipeline& pipe, const input_args& input)
{
	rs2::config cfg;
	// depth stream seems to be required for infrared to work
	//cfg.enable_stream(RS2_STREAM_DEPTH, input.width, input.height, RS2_FORMAT_Z16, input.framerate);
	//cfg.enable_stream(RS2_STREAM_INFRARED, 1, input.width, input.height, RS2_FORMAT_Y8, input.framerate);
	
	//native format of Realsense D415 RGV sesnor is YUYV
	//https://github.com/IntelRealSense/librealsense/issues/3042
	cfg.enable_stream(RS2_STREAM_COLOR, input.width, input.height, RS2_FORMAT_YUYV, input.framerate);

	rs2::pipeline_profile profile = pipe.start(cfg);
}

int process_user_input(int argc, char* argv[], input_args* input, nhve_net_config *net_config, nhve_hw_config *hw_config)
{
	if(argc < 7)
	{
		cerr << "Usage: " << argv[0] << " <host> <port> <width> <height> <framerate> <seconds> [device]" << endl;
		cerr << endl << "examples: " << endl;
		cerr << argv[0] << " 127.0.0.1 9766 640 360 30 5" << endl;
		cerr << argv[0] << " 127.0.0.1 9766 640 360 30 5 /dev/dri/renderD128" << endl;

		return -1;
	}

	net_config->ip = argv[1];
	net_config->port = atoi(argv[2]);
	
	hw_config->width = input->width = atoi(argv[3]);
	hw_config->height = input->height = atoi(argv[4]);
	hw_config->framerate = input->framerate = atoi(argv[5]);
	
	input->seconds = atoi(argv[6]);
	
	hw_config->device = argv[7]; //NULL as last argv argument, or device path
	
	return 0;
}
 
int hint_user_on_failure(char *argv[])
{
	cerr << "unable to initalize, try to specify device e.g:" << endl << endl;
	cerr << argv[0] << " 127.0.0.1 9766 640 360 30 5 /dev/dri/renderD128" << endl;
	return -1;
}
