#pragma once

struct service
{
   char name[0x100];
   int port;
   bool should_log;
   int frequency;
   int decimation;
};

static struct service services[] = {
    // service: (should_log, frequency, qlog decimation (optional))
    // { "sensorEvents", 8001, true, 100., 100 },
    // { "gpsNMEA", 8002, true, 9., -1},
//   "deviceState": (True, 2., 1),
//   "can": (True, 100.),
//   "controlsState": (True, 100., 10),
//   "pandaState": (True, 2., 1),
    { "radarState", 8007, true, 20, 5 },
//   "roadEncodeIdx": (True, 20., 1),
    { "liveTracks", 8009, true, 20, -1 },
//   "sendcan": (True, 100.),
//   "logMessage": (True, 0.),
    { "liveCalibration", 8012, true, 4, -4 },
//   "androidLog": (True, 0., 1),
    { "carState", 8014, true, 100, 10 },
//   "carControl": (True, 100., 10),
//   "longitudinalPlan": (True, 20., 5),
//   "procLog": (True, 0.5),
//   "gpsLocationExternal": (True, 10., 1),
//   "ubloxGnss": (True, 10.),
//   "clocks": (True, 1., 1),
//   "ubloxRaw": (True, 20.),
//   "liveLocationKalman": (True, 20., 2),
//   "liveParameters": (True, 20., 2),
    { "cameraOdometry", 8025, true, 20, 5 },
    { "lateralPlan", 8026, true, 20, 5 },
    { "thumbnail", 8027, true, 0, 1 },
//   "carEvents": (True, 1., 1),
//   "carParams": (True, 0.02, 1),
    { "roadCameraState", 8030, true, 20, 20 },
//   "driverCameraState": (True, DCAM_FREQ, DCAM_FREQ),
//   "driverEncodeIdx": (True, DCAM_FREQ, 1),
//   "driverState": (True, DCAM_FREQ, DCAM_FREQ / 2),
//   "driverMonitoringState": (True, DCAM_FREQ, DCAM_FREQ / 2),
//   "wideRoadEncodeIdx": (True, 20., 1),
//   "wideRoadCameraState": (True, 20., 20),
    { "modelV2", 8037, true, 20, 40 },
//   "managerState": (True, 2., 1),
//   "uploaderState": (True, 0., 1),
//   "liveMapData": (True, 0.),

//   # debug
//   "testJoystick": (False, 0.),

//   # dp
//   "thermal": (True, 2., 1),
//   "dragonConf": (False, 1.),
};