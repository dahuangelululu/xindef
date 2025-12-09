/**
 ******************************************************************************
 * @file    app_postprocess.c
 * @author  GPM Application Team
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */


#include "app_postprocess.h"
#include "app_config.h"
#include <assert.h>
#include <math.h>

#if POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V5_UU
static od_pp_outBuffer_t out_detections[AI_OBJDETECT_YOLOV5_PP_TOTAL_BOXES];
#elif POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V8_UF || POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V8_UI
static od_pp_outBuffer_t out_detections[AI_OBJDETECT_YOLOV8_PP_TOTAL_BOXES];
#elif POSTPROCESS_TYPE == POSTPROCESS_OD_ST_SSD_UF
static od_pp_outBuffer_t out_detections[AI_OD_SSD_ST_PP_TOTAL_DETECTIONS];
#elif POSTPROCESS_TYPE == POSTPROCESS_MPE_YOLO_V8_UF
static mpe_pp_outBuffer_t out_detections[AI_MPE_YOLOV8_PP_TOTAL_BOXES];
static mpe_pp_keyPoints_t out_keyPoints[AI_MPE_YOLOV8_PP_TOTAL_BOXES * AI_POSE_PP_POSE_KEYPOINTS_NB];
#elif POSTPROCESS_TYPE == POSTPROCESS_MPE_PD_UF
/* Must be in app code */
#include "pd_anchors.c"
/* post process algo will not write more than AI_PD_MODEL_PP_MAX_BOXES_LIMIT */
static pd_pp_box_t out_detections[AI_PD_MODEL_PP_MAX_BOXES_LIMIT];
static pd_pp_point_t out_keyPoints[AI_PD_MODEL_PP_MAX_BOXES_LIMIT][AI_PD_MODEL_PP_NB_KEYPOINTS];
#elif POSTPROCESS_TYPE == POSTPROCESS_SPE_MOVENET_UF
static spe_pp_outBuffer_t out_detections[AI_POSE_PP_POSE_KEYPOINTS_NB];
#elif POSTPROCESS_TYPE == POSTPROCESS_ISEG_YOLO_V8_UI
uint8_t _iseg_mask[AI_YOLOV8_SEG_PP_MASK_SIZE * AI_YOLOV8_SEG_PP_MASK_SIZE * AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT];
iseg_postprocess_outBuffer_t out_detections[AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT];
iseg_postprocess_scratchBuffer_s8_t scratch_detections[AI_YOLOV8_SEG_PP_TOTAL_BOXES];
float32_t _out_buf_mask[AI_YOLOV8_SEG_PP_MASK_NB];
int8_t _out_buf_mask_s8[AI_YOLOV8_SEG_PP_MASK_NB * AI_YOLOV8_SEG_PP_TOTAL_BOXES];
#elif POSTPROCESS_TYPE == POSTPROCESS_SSEG_DEEPLAB_V3_UF
static uint8_t out_sseg_map[AI_SSEG_DEEPLABV3_PP_WIDTH * AI_SSEG_DEEPLABV3_PP_HEIGHT];
#elif POSTPROCESS_TYPE == POSTPROCESS_OD_METER_YOLOV5_UI
static meter_pp_detection_t out_detections[METER_PP_MAX_BOXES_LIMIT];
#endif

#if POSTPROCESS_TYPE == POSTPROCESS_OD_METER_YOLOV5_UI
static float clampf(float v, float lo, float hi)
{
  if (v < lo)
  {
    return lo;
  }

  if (v > hi)
  {
    return hi;
  }

  return v;
}

static float iou_normalized(const meter_pp_detection_t *a, const meter_pp_detection_t *b)
{
  const float ax0 = a->x_center - (a->width * 0.5f);
  const float ay0 = a->y_center - (a->height * 0.5f);
  const float ax1 = a->x_center + (a->width * 0.5f);
  const float ay1 = a->y_center + (a->height * 0.5f);

  const float bx0 = b->x_center - (b->width * 0.5f);
  const float by0 = b->y_center - (b->height * 0.5f);
  const float bx1 = b->x_center + (b->width * 0.5f);
  const float by1 = b->y_center + (b->height * 0.5f);

  const float inter_x0 = fmaxf(ax0, bx0);
  const float inter_y0 = fmaxf(ay0, by0);
  const float inter_x1 = fminf(ax1, bx1);
  const float inter_y1 = fminf(ay1, by1);

  const float inter_w = fmaxf(0.0f, inter_x1 - inter_x0);
  const float inter_h = fmaxf(0.0f, inter_y1 - inter_y0);
  const float inter_area = inter_w * inter_h;

  const float area_a = a->width * a->height;
  const float area_b = b->width * b->height;
  const float union_area = area_a + area_b - inter_area + 1e-6f;

  return inter_area / union_area;
}

static void sort_by_score(meter_pp_detection_t *dets, uint32_t count)
{
  for (uint32_t i = 1; i < count; i++)
  {
    meter_pp_detection_t key = dets[i];
    int32_t j = (int32_t)i - 1;
    while (j >= 0 && dets[j].score < key.score)
    {
      dets[j + 1] = dets[j];
      j--;
    }
    dets[j + 1] = key;
  }
}
#endif

int32_t app_postprocess_init(void *params_postprocess)
{
#if POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V2_UF
  int32_t error = AI_OD_POSTPROCESS_ERROR_NO;
  yolov2_pp_static_param_t *params = (yolov2_pp_static_param_t *) params_postprocess;
  params->conf_threshold = AI_OBJDETECT_YOLOV2_PP_CONF_THRESHOLD;
  params->iou_threshold = AI_OBJDETECT_YOLOV2_PP_IOU_THRESHOLD;
  params->nb_anchors = AI_OBJDETECT_YOLOV2_PP_NB_ANCHORS;
  params->nb_classes = AI_OBJDETECT_YOLOV2_PP_NB_CLASSES;
  params->grid_height = AI_OBJDETECT_YOLOV2_PP_GRID_HEIGHT;
  params->grid_width = AI_OBJDETECT_YOLOV2_PP_GRID_WIDTH;
  params->nb_input_boxes = AI_OBJDETECT_YOLOV2_PP_NB_INPUT_BOXES;
  params->pAnchors = AI_OBJDETECT_YOLOV2_PP_ANCHORS;
  params->max_boxes_limit = AI_OBJDETECT_YOLOV2_PP_MAX_BOXES_LIMIT;
  error = od_yolov2_pp_reset(params);
#elif POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V5_UU
  int32_t error = AI_OD_POSTPROCESS_ERROR_NO;
  yolov5_pp_static_param_t *params = (yolov5_pp_static_param_t *) params_postprocess;
  params->nb_classes = AI_OBJDETECT_YOLOV5_PP_NB_CLASSES;
  params->nb_total_boxes = AI_OBJDETECT_YOLOV5_PP_TOTAL_BOXES;
  params->max_boxes_limit = AI_OBJDETECT_YOLOV5_PP_MAX_BOXES_LIMIT;
  params->conf_threshold = AI_OBJDETECT_YOLOV5_PP_CONF_THRESHOLD;
  params->iou_threshold = AI_OBJDETECT_YOLOV5_PP_IOU_THRESHOLD;
@@ -161,55 +224,70 @@ int32_t app_postprocess_init(void *params_postprocess)
  params->conf_threshold = AI_YOLOV8_SEG_PP_CONF_THRESHOLD;
  params->iou_threshold = AI_YOLOV8_SEG_PP_IOU_THRESHOLD;
  params->size_masks = AI_YOLOV8_SEG_PP_MASK_SIZE;
  params->raw_output_scale = AI_YOLOV8_SEG_SCALE;
  params->raw_output_zero_point = AI_YOLOV8_SEG_ZERO_POINT;
  params->nb_masks = AI_YOLOV8_SEG_PP_MASK_NB;
  params->mask_raw_output_zero_point = AI_YOLOV8_SEG_MASK_ZERO_POINT;
  params->mask_raw_output_scale = AI_YOLOV8_SEG_MASK_SCALE;
  params->pMask = _out_buf_mask;
  params->pTmpBuff = scratch_detections;
  for (size_t i = 0; i < AI_YOLOV8_SEG_PP_TOTAL_BOXES; i++) {
    scratch_detections[i].pMask = &_out_buf_mask_s8[i * AI_YOLOV8_SEG_PP_MASK_NB];
  }
  for (size_t i = 0; i < AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT; i++) {
    out_detections[i].pMask = &_iseg_mask[i * AI_YOLOV8_SEG_PP_MASK_SIZE * AI_YOLOV8_SEG_PP_MASK_SIZE];
  }
  error = iseg_yolov8_pp_reset(params);
#elif POSTPROCESS_TYPE == POSTPROCESS_SSEG_DEEPLAB_V3_UF
  int32_t error = AI_SSEG_POSTPROCESS_ERROR_NO;
  sseg_deeplabv3_pp_static_param_t *params = (sseg_deeplabv3_pp_static_param_t *) params_postprocess;
  params->nb_classes = AI_SSEG_DEEPLABV3_PP_NB_CLASSES;
  params->width = AI_SSEG_DEEPLABV3_PP_WIDTH;
  params->height = AI_SSEG_DEEPLABV3_PP_HEIGHT;
  params->type = AI_SSEG_DATA_UINT8;
  error = sseg_deeplabv3_pp_reset(params);
#elif POSTPROCESS_TYPE == POSTPROCESS_OD_METER_YOLOV5_UI
  meter_pp_static_param_t *params = (meter_pp_static_param_t *) params_postprocess;
  params->conf_threshold = METER_PP_CONF_THRESHOLD;
  params->iou_threshold = METER_PP_IOU_THRESHOLD;
  params->num_boxes = METER_PP_NUM_BOXES;
  params->max_boxes_limit = METER_PP_MAX_BOXES_LIMIT;
  params->input_height = NN_HEIGHT;
  params->input_width = NN_WIDTH;
  params->out_scale = METER_PP_OUT_SCALE;
  params->out_zero_point = METER_PP_OUT_ZERO_POINT;
  return 0;
#else
  #error "PostProcessing type not supported"
#endif

#if POSTPROCESS_TYPE == POSTPROCESS_OD_METER_YOLOV5_UI
  return 0;
#else
  return error;
#endif
}

int32_t app_postprocess_run(void *pInput[], int nb_input, void *pOutput, void *pInput_param)
{
#if POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V2_UF
  assert(nb_input == 1);
  int32_t error = AI_OD_POSTPROCESS_ERROR_NO;
  yolov2_pp_in_t pp_input = {
    .pRaw_detections = (float32_t *) pInput[0]
  };
  error = od_yolov2_pp_process(&pp_input, (od_pp_out_t *) pOutput,
                               (yolov2_pp_static_param_t *) pInput_param);
#elif POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V5_UU
  assert(nb_input == 1);
  int32_t error = AI_OD_POSTPROCESS_ERROR_NO;
  od_pp_out_t *pObjDetOutput = (od_pp_out_t *) pOutput;
  pObjDetOutput->pOutBuff = out_detections;
  yolov5_pp_in_centroid_uint8_t pp_input = {
      .pRaw_detections = (uint8_t *) pInput[0]
  };
  error = od_yolov5_pp_process_uint8(&pp_input, pObjDetOutput,
                                     (yolov5_pp_static_param_t *) pInput_param);
#elif POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V8_UF
  assert(nb_input == 1);
  int32_t error = AI_OD_POSTPROCESS_ERROR_NO;
@@ -288,31 +366,113 @@ int32_t app_postprocess_run(void *pInput[], int nb_input, void *pOutput, void *p
  };
  error = spe_movenet_pp_process(&pp_input, pPoseOutput,
                                 (spe_movenet_pp_static_param_t *) pInput_param);
#elif POSTPROCESS_TYPE == POSTPROCESS_ISEG_YOLO_V8_UI
  assert(nb_input == 2);
  int32_t error = AI_ISEG_POSTPROCESS_ERROR_NO;
  iseg_postprocess_out_t *pSegOutput = (iseg_postprocess_out_t *) pOutput;
  pSegOutput->pOutBuff = out_detections;
  yolov8_seg_pp_in_centroid_int8_t pp_input =
  {
      .pRaw_detections = (int8_t *) pInput[0],
      .pRaw_masks = (int8_t *) pInput[1]
  };
  error = iseg_yolov8_pp_process(&pp_input, pOutput,
                                 (yolov8_seg_pp_static_param_t *) pInput_param);
#elif POSTPROCESS_TYPE == POSTPROCESS_SSEG_DEEPLAB_V3_UF
  assert(nb_input == 1);
  int32_t error = AI_SSEG_POSTPROCESS_ERROR_NO;
  sseg_pp_out_t *pSsegOutput = (sseg_pp_out_t *) pOutput;
  pSsegOutput->pOutBuff = out_sseg_map;
  sseg_deeplabv3_pp_in_t pp_input = {
    .pRawData = (float32_t *) pInput[0]
  };
  error = sseg_deeplabv3_pp_process(&pp_input, (sseg_pp_out_t *) pOutput,
                                    (sseg_deeplabv3_pp_static_param_t *) pInput_param);
#elif POSTPROCESS_TYPE == POSTPROCESS_OD_METER_YOLOV5_UI
  assert(nb_input == 1);
  meter_pp_out_t *pMeterOutput = (meter_pp_out_t *) pOutput;
  meter_pp_static_param_t *params = (meter_pp_static_param_t *) pInput_param;
  const int8_t *raw = (const int8_t *) pInput[0];

  meter_pp_detection_t candidates[METER_PP_MAX_BOXES_LIMIT];
  uint32_t candidate_count = 0;

  for (uint32_t i = 0; i < params->num_boxes; i++)
  {
    const uint32_t base = i * 14U;

    const float cx = ((float)raw[base + 0] - (float)params->out_zero_point) * params->out_scale;
    const float cy = ((float)raw[base + 1] - (float)params->out_zero_point) * params->out_scale;
    const float w = ((float)raw[base + 2] - (float)params->out_zero_point) * params->out_scale;
    const float h = ((float)raw[base + 3] - (float)params->out_zero_point) * params->out_scale;
    const float obj = ((float)raw[base + 4] - (float)params->out_zero_point) * params->out_scale;
    const float cls = ((float)raw[base + 5] - (float)params->out_zero_point) * params->out_scale;

    const float conf = obj * cls;

    if (conf < params->conf_threshold)
    {
      continue;
    }

    if (candidate_count >= METER_PP_MAX_BOXES_LIMIT)
    {
      continue;
    }

    meter_pp_detection_t det = {0};
    det.score = conf;
    det.class_index = 0;
    det.x_center = clampf(cx / (float)params->input_width, 0.0f, 1.0f);
    det.y_center = clampf(cy / (float)params->input_height, 0.0f, 1.0f);
    det.width = clampf(w / (float)params->input_width, 0.0f, 1.0f);
    det.height = clampf(h / (float)params->input_height, 0.0f, 1.0f);

    for (uint32_t k = 0; k < 4; k++)
    {
      const float kx = ((float)raw[base + 6 + 2 * k] - (float)params->out_zero_point) * params->out_scale;
      const float ky = ((float)raw[base + 7 + 2 * k] - (float)params->out_zero_point) * params->out_scale;
      det.keypoints[k][0] = clampf(kx / (float)params->input_width, 0.0f, 1.0f);
      det.keypoints[k][1] = clampf(ky / (float)params->input_height, 0.0f, 1.0f);
    }

    candidates[candidate_count++] = det;
  }

  sort_by_score(candidates, candidate_count);

  uint32_t selected_count = 0;
  for (uint32_t i = 0; i < candidate_count; i++)
  {
    uint8_t keep = 1U;
    for (uint32_t j = 0; j < selected_count; j++)
    {
      if (iou_normalized(&candidates[i], &out_detections[j]) > params->iou_threshold)
      {
        keep = 0U;
        break;
      }
    }

    if (keep)
    {
      out_detections[selected_count++] = candidates[i];
      if (selected_count >= params->max_boxes_limit)
      {
        break;
      }
    }
    }

    pMeterOutput->pOutBuff = out_detections;
    pMeterOutput->nb_detect = selected_count;
  #else
    #error "PostProcessing type not supported"
  #endif

#if POSTPROCESS_TYPE == POSTPROCESS_OD_METER_YOLOV5_UI
  return 0;
#else
  return error;
#endif
  }