#pragma once

#include <cstdlib>

#include "perception/camerad_component/common/util.h"
#include "perception/camerad_component/hardware/base.h"

class HardwareJetson : public HardwareNone {
public:

  static bool JETSON() { return true; }

  static void reboot() { std::system("sudo reboot"); };
  static void poweroff() { std::system("sudo poweroff"); };
};
