# Realsense Network Hardware Video Encoder

Realsense hardware video encoding and streaming over custom [MLSP](https://github.com/bmegli/minimal-latency-streaming-protocol) protocol.
See [unity-network-hardware-video-decoder](https://github.com/bmegli/unity-network-hardware-video-decoder) as example network decoder & renderer.

The intent behind the program:
- private experiments
- minimize video latency
- minimize CPU usage (hardware encoding, color conversions)

## Platforms 

Unix-like operating systems (e.g. Linux).

The dependency is through [MLSP](https://github.com/bmegli/minimal-latency-streaming-protocol) socket use (easily portable).

Tested on Ubuntu 18.04.

## Hardware

Intel VAAPI compatible hardware encoders ([Quick Sync Video](https://ark.intel.com/Search/FeatureFilter?productType=processors&QuickSyncVideo=true)).

ATI/AMD may also work through VAAPI (libva-mesa-driver, not tested however).

The dependency is through [HVE](https://github.com/bmegli/hardware-video-encoder) implementation (see [HVE issues](https://github.com/bmegli/hardware-video-encoder/issues/5)).

Tested on LattePanda Alpha and i7-7820HK laptop.

## Dependencies

This program depends on:
- [librealsense2](https://github.com/IntelRealSense/librealsense) 
- [NHVE Network Hardware Video Encoder](https://github.com/bmegli/network-hardware-video-encoder)
	- [HVE Hardware Video Encoder](https://github.com/bmegli/hardware-video-encoder)
		- FFmpeg `avcodec` and `avutil` (at least 3.4 version)
	- [MLSP Minimal Latency Streaming Protocol](https://github.com/bmegli/minimal-latency-streaming-protocol)

Install RealSense™ SDK 2.0 as described on [github](https://github.com/IntelRealSense/librealsense) 

NHVE and its dependencies are included as submodules so you only need to satifisy HVE dependencies.

Works with system FFmpeg on Ubuntu 18.04 and doesn't on 16.04 (outdated FFmpeg).

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
cd realsense-network-hardware-video-encoder.git
mkdir build
cd build
cmake ..
make
```

## Running

Stream Realsense color/infrared video over UDP.

```bash
# Usage: ./realsense-nhve <host> <port> <color/ir> <width> <height> <framerate> <seconds> [device] [bitrate]
./realsense-nhve 127.0.0.1 9766 color 640 360 30 5
#./realsense-nhve 127.0.0.1 9766 infrared 640 360 30 5
#./realsense-nhve 127.0.0.1 9766 color 640 360 30 5 /dev/dri/renderD128
#./realsense-nhve 127.0.0.1 9766 infrared 640 360 30 5 /dev/dri/renderD128
#./realsense-nhve 192.168.0.125 9766 color 640 360 30 50 /dev/dri/renderD128 500000
```

You may need to specify VAAPI device if you have more than one (e.g. NVIDIA GPU + Intel CPU).

If you don't have receiving end you will just see if hardware encoding worked/didn't work.

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

