# RobotFramework
Robot Client in C++

To use this repository, you will need several dependencies.  

First, install the required packages for building and OpenCV:

```bash
sudo apt install build-essential pkg-config libopencv-dev
```

This will install the required dependencies to run the ball camera detector.  

Next, install the Raspberry Pi libraries needed for running moteus code:

```bash
sudo apt-get install libraspberrypi-dev raspberrypi-kernel-headers
```

Finally, you will need **CMake** to compile the repository.  
Build the project with the following commands:

```bash
mkdir build
cd build
cmake <repo-destination>
make
```