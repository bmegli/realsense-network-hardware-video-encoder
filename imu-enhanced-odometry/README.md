# IEO IMU Enhanced Odometry C++ library

TO DO

## Platforms 

TO DO

## Dependencies

TODO (eigen)

## Building Instructions

Tested on Ubuntu 18.04.

``` bash
# update package repositories
sudo apt-get update 
# get compilers and make 
sudo apt-get install build-essential
# get cmake - we need to specify libcurl4 for Ubuntu 18.04 dependencies problem
sudo apt-get install libcurl4 cmake
# get git
sudo apt-get install git
# clone the repository with *RECURSIVE* for submodules
# git clone https://github.com/bmegli/TODO.git

# finally build the program
cd imu-enhanced-odometry
mkdir build
cd build
cmake ..
make
```

## Using

## License

Code in this repository is licensed under Mozilla Public License, v. 2.0

This is similiar to LGPL but more permissive:
- you can use it as LGPL in prioprietrary software
- unlike LGPL you may compile it statically with your code

Like in LGPL, if you modify the code, you have to make your changes available.
Making a github fork with your changes satisfies those requirements perfectly.

