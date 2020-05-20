# RNHVE Realsense Network Hardware Video Encoder

Realsense hardware video/depth encoding and streaming over custom [MLSP](https://github.com/bmegli/minimal-latency-streaming-protocol) protocol.

This includes streaming:
- color (H.264, HEVC Main)
- infrared (H.264, HEVC Main)
- depth (HEVC Main10)
- textured depth (HEVC Main10 + HEVC Main)

See [unity-network-hardware-video-decoder](https://github.com/bmegli/unity-network-hardware-video-decoder) as example network decoder & renderer (color, infrared and depth).

See [how it works](https://github.com/bmegli/realsense-network-hardware-video-encoder/wiki/How-it-works) on wiki to understand the code.

See [hardware-video-streaming](https://github.com/bmegli/hardware-video-streaming) for other related projects.

See videos to understand point cloud streaming features:

| Point Cloud Streaming | Infrared Textured Point Cloud Streaming |
|-----------------------|-----------------------------------------|
| [![Hardware Accelerated Point Cloud Streaming](http://img.youtube.com/vi/qnTxhfNW-_4/0.jpg)](http://www.youtube.com/watch?v=qnTxhfNW-_4) | [![Hardware Accelerated Infrared Textured Point Cloud Streaming](http://img.youtube.com/vi/zVIuvWMz5mU/0.jpg)](https://www.youtube.com/watch?v=zVIuvWMz5mU) |


## Platforms 

Unix-like operating systems (e.g. Linux), [more info](https://github.com/bmegli/realsense-network-hardware-video-encoder/wiki/Platforms).

Tested on Ubuntu 18.04.

## Hardware

Tested on Intel KabyLake with Realsense D435:

Requires Intel VAAPI compatible hardware encoder (QuickSync Video). For depth encoding at least KabyLake.

[Other technologies](https://github.com/bmegli/realsense-network-hardware-video-encoder/wiki/Hardware) may also work but were not tested.

Textured depth encoding is implemented only for D435.

## Dependencies

This program depends on:
- [librealsense2](https://github.com/IntelRealSense/librealsense) 
- [NHVE Network Hardware Video Encoder](https://github.com/bmegli/network-hardware-video-encoder)
	- [HVE Hardware Video Encoder](https://github.com/bmegli/hardware-video-encoder)
		- FFmpeg `avcodec` and `avutil` (at least 3.4 version)
	- [MLSP Minimal Latency Streaming Protocol](https://github.com/bmegli/minimal-latency-streaming-protocol)

Install RealSense™ SDK 2.0 as described on [github](https://github.com/IntelRealSense/librealsense) 

NHVE and its dependencies are included as submodules so you only need to satifisy HVE dependencies.

Works with system FFmpeg on Ubuntu 18.04 and doesn't on 16.04 (outdated FFmpeg and VAAPI ecosystem).

## Building Instructions

Tested on Ubuntu 18.04.

``` bash
# update package repositories
sudo apt-get update 
# get avcodec and avutil
sudo apt-get install ffmpeg libavcodec-dev libavutil-dev
# get compilers and make 
sudo apt-get install build-essential
# get cmake - we need to specify libcurl4 for Ubuntu 18.04 dependencies problem
sudo apt-get install libcurl4 cmake
# get git
sudo apt-get install git
# clone the repository with *RECURSIVE* for submodules
git clone --recursive https://github.com/bmegli/realsense-network-hardware-video-encoder.git

# finally build the program
cd realsense-network-hardware-video-encoder
mkdir build
cd build
cmake ..
make
```

## Running

Stream H.264 Realsense color/infrared video over UDP.

```bash
# Usage: ./realsense-nhve-h264 <host> <port> <color/ir> <width> <height> <framerate> <seconds> [device] [bitrate]
./realsense-nhve-h264 127.0.0.1 9766 color 640 360 30 5
#./realsense-nhve-h264 127.0.0.1 9766 infrared 640 360 30 5
#./realsense-nhve-h264 127.0.0.1 9766 color 640 360 30 5 /dev/dri/renderD128
#./realsense-nhve-h264 127.0.0.1 9766 infrared 640 360 30 5 /dev/dri/renderD128
#./realsense-nhve-h264 192.168.0.125 9766 color 640 360 30 50 /dev/dri/renderD128 500000
```

Stream Realsense:
- color/infrared with HEVC Main
- depth with HEVC Main10


```bash
# Usage: ./realsense-nhve-hevc <host> <port> <color/ir/depth/depth+ir> <width> <height> <framerate> <seconds> [device] [bitrate] [depth units]
./realsense-nhve-hevc 127.0.0.1 9766 color 640 360 30 5
#./realsense-nhve-hevc 127.0.0.1 9766 infrared 640 360 30 5
#./realsense-nhve-hevc 127.0.0.1 9766 depth 640 360 30 5
#./realsense-nhve-hevc 127.0.0.1 9766 depth+ir 848 480 30 5
#./realsense-nhve-hevc 127.0.0.1 9766 color 640 360 30 5 /dev/dri/renderD128
#./realsense-nhve-hevc 127.0.0.1 9766 infrared 640 360 30 5 /dev/dri/renderD128
#./realsense-nhve-hevc 127.0.0.1 9766 depth 640 360 30 5 /dev/dri/renderD128
#./realsense-nhve-hevc 127.0.0.1 9766 depth+ir 848 480 30 5 /dev/dri/renderD128
#./realsense-nhve-hevc 192.168.0.125 9766 color 640 360 30 50 /dev/dri/renderD128 500000
#./realsense-nhve-hevc 192.168.0.125 9768 depth 848 480 30 50 /dev/dri/renderD128 2000000
#./realsense-nhve-hevc 192.168.0.125 9768 depth 848 480 30 50 /dev/dri/renderD128 8000000 0.0001
#./realsense-nhve-hevc 192.168.0.100 9768 depth+ir 848 480 30 50 /dev/dri/renderD128 8000000 0.0001
```

Stream Realsense D435:
- depth with HEVC Main10, infrared with HEVC

```bash
./realsense-nhve-depth-ir <host> <port> <width> <height> <framerate> <seconds> [device] [bitrate_depth] [bitrate_ir] [depth units]

examples: 
./realsense-nhve-depth-ir 127.0.0.1 9766 640 360 30 5
./realsense-nhve-depth-ir 127.0.0.1 9766 640 360 30 5 /dev/dri/renderD128
./realsense-nhve-depth-ir 192.168.0.125 9766 640 360 30 50 /dev/dri/renderD128 4000000 1000000
./realsense-nhve-depth-ir 192.168.0.100 9768 848 480 30 500 /dev/dri/renderD128 8000000 1000000 0.0001
./realsense-nhve-depth-ir 192.168.0.100 9768 848 480 30 500 /dev/dri/renderD128 8000000 1000000 0.00005
./realsense-nhve-depth-ir 192.168.0.100 9768 848 480 30 500 /dev/dri/renderD128 8000000 1000000 0.000025
./realsense-nhve-depth-ir 192.168.0.100 9768 848 480 30 500 /dev/dri/renderD128 8000000 1000000 0.0000125
./realsense-nhve-depth-ir 192.168.0.100 9768 848 480 30 500 /dev/dri/renderD128 8000000 1000000 0.0000125
```

If you don't have receiving end you will just see if hardware encoding worked/didn't work.

You may need to specify VAAPI device if you have more than one (e.g. NVIDIA GPU + Intel CPU).

If you get errors see also HVE [troubleshooting](https://github.com/bmegli/hardware-video-encoder/wiki/Troubleshooting).

## License

Code in this repository and my dependencies are licensed under Mozilla Public License, v. 2.0

This is similiar to LGPL but more permissive:
- you can use it as LGPL in prioprietrary software
- unlike LGPL you may compile it statically with your code

Like in LGPL, if you modify the code, you have to make your changes available.
Making a github fork with your changes satisfies those requirements perfectly.

Since you are linking to FFmpeg libraries consider also `avcodec` and `avutil` licensing.

## Additional information

The implementation down to FFmpeg (encoding) and up to MLSP (sending) is zero copy (passing the pointers to data) whenever possible.

