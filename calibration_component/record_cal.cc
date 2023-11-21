#include "record_cal.h"

void calRecordWriter(const std::string &writefile, LiveCalibrationData *calibration)
{
  RecordWriter writer;
  writer.SetSizeOfFileSegmentation(0);
  writer.SetIntervalOfFileSegmentation(0);
  writer.Open(writefile);

  writer.WriteChannel(CHANNEL_NAME_1, MESSAGE_TYPE_1, PROTO_DESC);
  LiveCalibrationData& cal = *calibration;
  writer.WriteMessage(CHANNEL_NAME_1, cal, apollo::cyber::Time::Now().ToNanosecond());
  
  writer.Close();
}

std::string calRecordReader(const std::string &readfile)
{
  std::ifstream file(readfile);
  if(!file.is_open()){
    return std::string();
  }

  RecordReader reader(readfile);
  RecordMessage message;
  uint64_t msg_count = reader.GetMessageNumber(CHANNEL_NAME_1);
  AINFO << "MSGTYPE: " << reader.GetMessageType(CHANNEL_NAME_1);
  AINFO << "MSGDESC: " << reader.GetProtoDesc(CHANNEL_NAME_1);

  // read all message
  uint64_t i = 0;
  uint64_t valid = 0;
  for (i = 0; i < msg_count; ++i) {
    if (reader.ReadMessage(&message)) {
      AINFO << "msg[" << i << "]-> "
            << "channel name: " << message.channel_name
            << "; content: " << message.content
            << "; msg time: " << message.time;
      valid++;
    } else {
      AERROR << "read msg[" << i << "] failed";
    }
  }
  return message.content;
}
