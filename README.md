# Perception Layer  
基于xnxpilot的感知层模块，改写成Cyber RT中Component形式进行通信，需要在apollo项目根目录运行  
# Installation
## third_party install  
### OpenCV  
`sudo bash docker/build/installers/install_opencv.sh`  
### GLES3  
`sudo apt update`
`sudo apt install libgles2-mesa-dev`  
### OpenCL  
`sudo apt install clinfo opencl-headers opencl-dev libpocl2`  
### ZeroMQ  
`sudo apt install libzmq3-dev`  
### libyuv  
`git clone https://chromium.googlesource.com/libyuv/libyuv  
cd libyuv  
mkdir build && cd build  
cmake ..  
make  
make install`  
## third_party config  


