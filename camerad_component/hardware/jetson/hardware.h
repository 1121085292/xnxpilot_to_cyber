#pragma once

#include <cstdlib>

#include "camerad_component/common/util.h"
#include "camerad_component/hardware/base.h"

class HardwareJetson : public HardwareNone {
public:

  static bool JETSON() { return true; }

  static void reboot() { std::system("sudo reboot"); };
  static void poweroff() { std::system("sudo poweroff"); };
};
