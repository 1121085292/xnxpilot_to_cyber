#pragma once

#include "cyber/cyber.h"
#include "cyber/record/record_writer.h"
#include "cyber/record/record_reader.h"
#include "common_msgs/calibrationd/live_calibration_data.pb.h"

using ::apollo::cyber::record::RecordMessage;
using ::apollo::cyber::record::RecordReader;
using ::apollo::cyber::record::RecordWriter;
using common_msgs::calibration::LiveCalibrationData;

const char CHANNEL_NAME_1[] = "CalibrationParams";

const char MESSAGE_TYPE_1[] = "common_msgs.calibration.LiveCalibrationData";

const char PROTO_DESC[] = "common_msgs.calibration.LiveCalibrationData";

void calRecordWriter(const std::string& writefile, LiveCalibrationData* calibration);

std::string calRecordReader(const std::string& readfile);