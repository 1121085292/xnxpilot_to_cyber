#pragma once
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <map>
#include "zmq.h"

#include "perception/camerad_component/messaging/messaging.h"
#include "cyber/cyber.h"
// #include "visionipc/visionipc.h"
#include "visionbuf.h"
#include "ipc.h"

std::string get_endpoint_name(std::string name, VisionStreamType type);

class VisionIpcServer {
 private:
  std::string name;
  cl_device_id device_id = nullptr;
  cl_context ctx = nullptr;
  uint64_t server_id;

  std::atomic<bool> should_exit{false};
  // std::thread listener_thread;

  std::map<VisionStreamType, std::atomic<size_t> > cur_idx;
  std::map<VisionStreamType, std::vector<VisionBuf*> > buffers;
  std::map<VisionStreamType, std::map<VisionBuf*, size_t> > idxs;

  Context * msg_ctx;
  std::map<VisionStreamType, PubSocket*> sockets;

  // void listener(void);

 public:
  VisionIpcServer() = default;
  void setVisionIpcServer(std::string name, cl_device_id device_id=nullptr, cl_context ctx=nullptr);
  ~VisionIpcServer();

  VisionBuf * get_buffer(VisionStreamType type);

  void create_buffers(VisionStreamType type, size_t num_buffers, bool rgb, size_t width, size_t height);
  void send(VisionBuf * buf, VisionIpcBufExtra * extra, bool sync=true);
  // void start_listener();
  void listener(void);

};
