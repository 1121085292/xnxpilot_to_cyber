#include "cyber/cyber.h"
#include "common_msgs/plannerd/car_state.pb.h"
#include "common_msgs/plannerd/lateral_plan.pb.h"
#include "common_msgs/camerad/frame_data.pb.h"

using apollo::cyber::Writer;
using common_msgs::lateral_plan::LateralPlan;
using common_msgs::car_state::CarState;
using common_msgs::camerad::FrameData;

int main(int argc, char *argv[]){
  apollo::cyber::Init(argv[0]);

  auto talker_node1 = apollo::cyber::CreateNode("car");
  auto talker_node2 = apollo::cyber::CreateNode("plan");
  // auto talker_node3 = apollo::cyber::CreateNode("camera");

  auto talker1 = talker_node1->CreateWriter<CarState>("CarState");
  auto talker2 = talker_node2->CreateWriter<LateralPlan>("lateralPlan");
  // auto talker3 = talker_node3->CreateWriter<FrameData>("roadCameraState");

  uint64_t seq = 0;
  apollo::cyber::Rate rate(0.5);
  while (apollo::cyber::OK())
  {
    seq++;
    AINFO << "发布第" << seq << "条数据！";

    auto msg1 = std::make_shared<CarState>();
    msg1->set_v_ego(20.0f);
    
    auto msg2 = std::make_shared<LateralPlan>();
    msg2->set_desire(common_msgs::lateral_plan::turnLeft);
    
    // auto msg3 = std::make_shared<FrameData>();
    // msg3->set_frame_id(seq);

    talker1->Write(msg1);
    talker2->Write(msg2);
    // talker3->Write(msg3);

    rate.Sleep();
  }
  apollo::cyber::WaitForShutdown();
  return 0;
}
