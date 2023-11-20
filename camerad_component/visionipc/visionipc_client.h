#pragma once
#include <vector>
#include <string>
#include <unistd.h>

#include "cyber/cyber.h"
#include "camerad_component/messaging/messaging.h"
#include "camerad_component/visionipc/visionipc.h"
#include "camerad_component/visionipc/visionbuf.h"

class VisionIpcClient {
private:
  std::string name;
  Context * msg_ctx;
  SubSocket * sock;
  Poller * poller;

  VisionStreamType type;

  cl_device_id device_id = nullptr;
  cl_context ctx = nullptr;

  void init_msgq(bool conflate);

public:
  bool connected = false;
  size_t num_buffers = 0;
  VisionBuf buffers[VISIONIPC_MAX_FDS];
  VisionIpcClient () {}
  VisionIpcClient(std::string name, VisionStreamType type, bool conflate, cl_device_id device_id=nullptr, cl_context ctx=nullptr);
  ~VisionIpcClient();
  VisionBuf * recv(VisionIpcBufExtra * extra=nullptr, const int timeout_ms=100);
  bool connect(bool blocking=true);
};
