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

	hw_config.pixel_format = "nv12";
	//native format of Realsense RGB sensor is YUYV (YUY2)
	//hw_config.pixel_format = "yuyv422";

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
		rs2::video_frame ir_frame = frameset.get_infrared_frame(1);

		if(!color_data)
		{   //prepare dummy color plane for NV12 format, half the size of Y
			//we can't alloc it in advance, this is the first time we know realsense stride
			int size = ir_frame.get_stride_in_bytes()*ir_frame.get_height()/2;
			color_data = new uint8_t[size];
			memset(color_data, 128, size);
		}
		
		frame.linesize[0] = frame.linesize[1] =  ir_frame.get_stride_in_bytes();
		frame.data[0] = (uint8_t*) ir_frame.get_data();
		frame.data[1] = color_data;

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
	// depth stream seems to be required for infrared to work
	cfg.enable_stream(RS2_STREAM_DEPTH, input.width, input.height, RS2_FORMAT_Z16, input.framerate);
	cfg.enable_stream(RS2_STREAM_INFRARED, 1, input.width, input.height, RS2_FORMAT_Y8, input.framerate);
	
	//native format of Realsense D415 RGV sesnor is YUYV
	//https://github.com/IntelRealSense/librealsense/issues/3042
	//cfg.enable_stream(RS2_STREAM_COLOR, input.width, input.height, RS2_FORMAT_YUYV, input.framerate);

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
 
/*
#include <stdio.h> //printf, fprintf
#include <inttypes.h> //uint8_t

#include "nhve.h"

const char *IP; //e.g "127.0.0.1"
unsigned short PORT; //e.g. 9667 

const int WIDTH=640;
const int HEIGHT=360;
const int FRAMERATE=30;
int SECONDS=10;
const char *DEVICE; //NULL for default or device e.g. "/dev/dri/renderD128"
const char *PIXEL_FORMAT="nv12"; //NULL / "" for default (NV12) or pixel format e.g. "rgb0"
const int PROFILE=FF_PROFILE_H264_HIGH; //or FF_PROFILE_H264_MAIN, FF_PROFILE_H264_CONSTRAINED_BASELINE, ...
const int BFRAMES=0; //max_b_frames, set to 0 to minimize latency, non-zero to minimize size
const int BITRATE=0; //average bitrate in VBR

//IP, PORT, SECONDS and DEVICE are read from user input

int streaming_loop(struct nhve *streamer);
int process_user_input(int argc, char* argv[]);
int hint_user_on_failure(char *argv[]);
void hint_user_on_success();

int main(int argc, char* argv[])
{
	//get SECONDS and DEVICE from the command line
	if( process_user_input(argc, argv) < 0 )
		return -1;

	//prepare library data
	struct nhve_net_config net_config = {IP, PORT};
	struct nhve_hw_config hw_config = {WIDTH, HEIGHT, FRAMERATE, DEVICE, PIXEL_FORMAT, PROFILE, BFRAMES, BITRATE};
	struct nhve *streamer;

	//initialize library with nhve_init
	if( (streamer = nhve_init(&net_config, &hw_config)) == NULL )
		return hint_user_on_failure(argv);

	//do the actual encoding
	int status = streaming_loop(streamer);

	nhve_close(streamer);

	if(status == 0)
		hint_user_on_success();

	return status;
}

int streaming_loop(struct nhve *streamer)
{
	struct nhve_frame frame = { 0 };
	int frames=SECONDS*FRAMERATE, f;
	const useconds_t useconds_per_frame = 1000000/FRAMERATE;

	//we are working with NV12 because we specified nv12 pixel format
	//when calling nhve_init, in principle we could use other format
	//if hardware supported it (e.g. RGB0 is supported on my Intel)
	uint8_t Y[WIDTH*HEIGHT]; //dummy NV12 luminance data
	uint8_t color[WIDTH*HEIGHT/2]; //dummy NV12 color data

	//fill with your stride (width including padding if any)
	frame.linesize[0] = frame.linesize[1] = WIDTH;

	for(f=0;f<frames;++f)
	{
		//prepare dummy image date, normally you would take it from camera or other source
		memset(Y, f % 255, WIDTH*HEIGHT); //NV12 luminance (ride through greyscale)
		memset(color, 128, WIDTH*HEIGHT/2); //NV12 UV (no color really)

		//fill hve_frame with pointers to your data in NV12 pixel format
		frame.data[0]=Y;
		frame.data[1]=color;
		
		//increase the framenumber, this is mandatory for the network protocol
		frame.framenumber = f;

		//encode and send this frame
		if( nhve_send_frame(streamer, &frame) != NHVE_OK)
			break; //break on error

		//simulate real time source (sleep according to framerate)
	}

	//flush the encoder by sending NULL frame, encode some last frames returned from hardware
	nhve_send_frame(streamer, NULL);

	//did we encode everything we wanted?
	//convention 0 on success, negative on failure
	return f == frames ? 0 : -1;
}

int process_user_input(int argc, char* argv[])
{
	if(argc < 4)
	{
		fprintf(stderr, "Usage: %s <ip> <port> <seconds> [device]\n", argv[0]);
		fprintf(stderr, "\nexamples:\n");
		fprintf(stderr, "%s 127.0.0.1 9766 10\n", argv[0]);
		fprintf(stderr, "%s 127.0.0.1 9766 10 /dev/dri/renderD128\n", argv[0]);
		return -1;
	}
	
	IP = argv[1];
	PORT = atoi(argv[2]);
	SECONDS = atoi(argv[3]);
	DEVICE=argv[4]; //NULL as last argv argument, or device path

	return 0;
}

int hint_user_on_failure(char *argv[])
{
	fprintf(stderr, "unable to initalize, try to specify device e.g:\n\n");
	fprintf(stderr, "%s 127.0.0.1 9766 10 /dev/dri/renderD128\n", argv[0]);
	return -1;
}
void hint_user_on_success()
{
	printf("finished successfully\n");
}
*/