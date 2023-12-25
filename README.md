# Perception Layer  
基于xnxpilot的感知层模块，改写成Cyber RT中Component形式进行通信，需要在***apollo***项目根目录运行  
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
`git clone https://chromium.googlesource.com/libyuv/libyuv`  
`cd libyuv`  
`mkdir build && cd build`  
`cmake ..`  
`make`  
`make install`  
## third_party config  
### OpenCV  
在third_party/opencv/opencv.BUILD中，新增`cc_library`规则  
`cc_library(`  
`    name = "videoio",`  
`    includes = ["."],`  
`    linkopts = [`  
`        "-L/opt/apollo/sysroot/lib",`  
`        "-lopencv_videoio"`  
`    ],`  
`    deps = [`  
`        ":core",`  
`        ":imgproc",`  
`        ":highgui"`  
`    ]`  
### tools  
在根目录下的tools/workspace.bzl文件内导入配置的第三方依赖库  
新增`load("//third_party/openpilot:workspace.bzl", openpilot = "repo")`  
然后在`initialize_third_party()`方法内添加`openpilot()`，之后就可以在BUILD文件中的deps内使用`@openpilot//:repo`的形式依赖自定义安装的三方库，`repo`换成通过`cc_library`自定义的`name`  
## Build project  
下载该项目解压到apollo项目根目录，然后执行  
`bazel build perception:libperception.so`  
Debug模式，执行`bazel build perception:libperception.so -c dbg`  
## Run  
通过cyber_launch和mainboard命令行启动各模块（camerad、calibrationd、modeld和radard）  
e.g.:  
`cyber_launch start perception/camerad_component/camerad.launch`  
`mainboard -d perception/camerad_compoent/camerad.dag`  
通过`cyber_monitor`监测通道数据
