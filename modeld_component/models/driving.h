#pragma once

// gate this here
#define TEMPORAL
#define DESIRE
#define TRAFFIC_CONVENTION

#define USE_CPU_RUNTIME 0
#define USE_GPU_RUNTIME 1
#define USE_DSP_RUNTIME 2

#include <memory>

#include "camerad_component/messaging/messaging.h"
#include "cyber/cyber.h"
#include "common_msgs/modeld/model_data_v2.pb.h"

#include "camerad_component/common/mat.h"
#include "camerad_component/common/modeldata.h"
#include "camerad_component/common/util.h"
#include "commonmodel.h"
#include "modeld_component/runners/onnxmodel.h"

constexpr int DESIRE_LEN = 8;
constexpr int TRAFFIC_CONVENTION_LEN = 2;
constexpr int MODEL_FREQ = 20;

struct ModelDataRaw {
  float *plan;
  float *lane_lines;
  float *lane_lines_prob;
  float *road_edges;
  float *lead;
  float *lead_prob;
  float *desire_state;
  float *meta;
  float *desire_pred;
  float *pose;
};

typedef struct ModelState {
  ModelFrame *frame;
  std::vector<float> output;
  std::unique_ptr<RunModel> m;
#ifdef DESIRE
  float prev_desire[DESIRE_LEN] = {};
  float pulse_desire[DESIRE_LEN] = {};
#endif
#ifdef TRAFFIC_CONVENTION
  float traffic_convention[TRAFFIC_CONVENTION_LEN] = {};
#endif
} ModelState;

void model_init(std::shared_ptr<ModelState> s, cl_device_id device_id, cl_context context);
ModelDataRaw model_eval_frame(std::shared_ptr<ModelState> s, cl_mem yuv_cl, int width, int height,
                           const mat3 &transform, float *desire_in);
void model_free(ModelState* s);
void poly_fit(float *in_pts, float *in_stds, float *out);

void fill_model(std::shared_ptr<common_msgs::model_data_v2::ModelDataV2> &framed, const ModelDataRaw &net_outputs);