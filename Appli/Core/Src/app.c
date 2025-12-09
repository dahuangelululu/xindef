/**
 ****************************************************************************************************
 * @file        app.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-13
 * @brief       app.c文件
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 * 
 * 实验平台:正点原子 N647开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 * 
 ****************************************************************************************************
 */

#include "app.h"
#include "app_config.h"
#include "app_utils.h"
#include "app_lcd.h"
#include "app_camera.h"
#include "app_bqueue.h"
#include "app_cpuload.h"
#include "app_postprocess.h"
#include "tx_api.h"
#include "cmw_camera.h"
#include "ll_aton_runtime.h"
#include <string.h>

#if POSTPROCESS_TYPE == POSTPROCESS_OD_METER_YOLOV5_UI
typedef meter_pp_detection_t app_detection_t;
typedef meter_pp_out_t app_pp_out_t;
typedef meter_pp_static_param_t app_pp_param_t;
#else
typedef od_pp_outBuffer_t app_detection_t;
typedef od_pp_out_t app_pp_out_t;
typedef yolov2_pp_static_param_t app_pp_param_t;
#endif

typedef struct {
    int32_t nb_detect;
    app_detection_t detects[APP_MAX_BOXES_LIMIT];
    //od_pp_outBuffer_t detects[AI_OBJDETECT_YOLOV2_PP_MAX_BOXES_LIMIT];
    uint32_t nn_period_ms;
    uint32_t inf_ms;
    uint32_t pp_ms;
    uint32_t disp_ms;
    
    uint32_t cam_disp_frames;
    uint32_t cam_nn_frames;
    uint32_t cam_disp_age_ms;
    uint32_t cam_nn_age_ms;
    
    
} app_display_info_t;

typedef struct {
    TX_SEMAPHORE update;
    TX_MUTEX lock;
    app_display_info_t info;
} app_display_t;

static TX_SEMAPHORE isp_semaphore;

static void app_camera_display_pipe_vsync_cb(void);
static void app_camera_display_pipe_frame_cb(void);
static void app_camera_nn_pipe_frame_cb(void);

static TX_THREAD nn_thread;
static UCHAR nn_thread_stack[4096];
static TX_THREAD pp_thread;
static UCHAR pp_thread_stack[4096];
static TX_THREAD dp_thread;
static UCHAR dp_thread_stack[4096];
static TX_THREAD isp_thread;
static UCHAR isp_thread_stack[4096];

static VOID nn_thread_entry(ULONG id);
static VOID pp_thread_entry(ULONG id);
static VOID dp_thread_entry(ULONG id);
static VOID isp_thread_entry(ULONG id);

static void app_camera_try_rearm(uint32_t now_ms);
//static void app_display_boot_screen(void);


static app_display_t display;

LL_ATON_DECLARE_NAMED_NN_INSTANCE_AND_INTERFACE(Default);
static uint8_t nn_input_buffers[2][NN_WIDTH * NN_HEIGHT * NN_BPP] __attribute__((aligned(32))) __attribute__((section(".EXTRAM")));
static app_bqueue_t nn_input_queue;

static uint8_t nn_output_buffers[2][ALIGN_VALUE(NN_BUFFER_OUT_SIZE, 32)] __attribute__((aligned(32))) __attribute__((section(".EXTRAM")));
//static uint8_t nn_output_buffers[2][ALIGN_VALUE(NN_BUFFER_OUT_SIZE, 32)] __attribute__((aligned(32)));
//static uint8_t nn_output_buffers[2][ALIGN_VALUE(NN_BUFFER_OUT_SIZE, 32)] __attribute__((aligned(32))) __attribute__((section(".EXTRAM")));

static app_bqueue_t nn_output_queue;
static const char *nn_classes_table[NN_CLASSES] = NN_CLASSES_TABLE;

static app_cpuload_t cpuload;
static volatile uint32_t cam_display_frames;
static volatile uint32_t cam_nn_frames;
static volatile uint32_t cam_display_last_ms;
static volatile uint32_t cam_nn_last_ms;
static uint32_t cam_last_rearm_ms;
static void app_display_network_output(app_display_info_t *display_info);

void app_run(void)
{
    app_lcd_init();
    
     //app_display_boot_screen();
    
    app_bqueue_init(&nn_input_queue, 2, (uint8_t *[2]){nn_input_buffers[0], nn_input_buffers[1]});
    app_bqueue_init(&nn_output_queue, 2, (uint8_t *[2]){nn_output_buffers[0], nn_output_buffers[1]});
    app_cpuload_init(&cpuload);
    app_camera_init(app_camera_display_pipe_vsync_cb, app_camera_display_pipe_frame_cb, NULL, app_camera_nn_pipe_frame_cb);

    tx_semaphore_create(&isp_semaphore, NULL, 0);
    tx_semaphore_create(&display.update, NULL, 0);
    tx_mutex_create(&display.lock, NULL, TX_INHERIT);
 cam_display_frames = 0;
    cam_nn_frames = 0;
    cam_display_last_ms = HAL_GetTick();
    cam_nn_last_ms = cam_display_last_ms;
    cam_last_rearm_ms = cam_display_last_ms;
 tx_mutex_get(&display.lock, TX_WAIT_FOREVER);
    memset(&display.info, 0, sizeof(display.info));
    tx_mutex_put(&display.lock);
    tx_semaphore_ceiling_put(&display.update, 1);


    app_camera_display_pipe_start(app_lcd_get_bg_buffer(), CMW_MODE_CONTINUOUS);

    tx_thread_create(&nn_thread, "NN Thread", nn_thread_entry, 0, nn_thread_stack, sizeof(nn_thread_stack), TX_MAX_PRIORITIES - 3, TX_MAX_PRIORITIES - 3, 10, TX_AUTO_START);
    tx_thread_create(&pp_thread, "PP Thread", pp_thread_entry, 0, pp_thread_stack, sizeof(pp_thread_stack), TX_MAX_PRIORITIES - 2, TX_MAX_PRIORITIES - 2, 10, TX_AUTO_START);
    tx_thread_create(&dp_thread, "DP Thread", dp_thread_entry, 0, dp_thread_stack, sizeof(dp_thread_stack), TX_MAX_PRIORITIES - 2, TX_MAX_PRIORITIES - 2, 10, TX_AUTO_START);
    tx_thread_create(&isp_thread, "ISP Thread", isp_thread_entry, 0, isp_thread_stack, sizeof(isp_thread_stack), TX_MAX_PRIORITIES - 4, TX_MAX_PRIORITIES - 4, 10, TX_AUTO_START);
}

//新加
/*static void app_display_boot_screen(void)
{
    UTIL_LCD_SetLayer(1);
    UTIL_LCD_Clear(0x00000000);
    UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_WHITE);
    UTIL_LCD_DisplayStringAt(0, 20, (uint8_t *)"AI meter demo", LEFT_MODE);
    UTIL_LCD_DisplayStringAt(0, 50, (uint8_t *)"Waiting for camera frames...", LEFT_MODE);
    UTIL_LCD_DisplayStringAt(0, 80, (uint8_t *)"If this text is invisible, LCD path is stuck", LEFT_MODE);
    app_lcd_draw_area_update();
    app_lcd_draw_area_commit();
}*/
//到这结束


static void app_camera_display_pipe_vsync_cb(void)
{
    tx_semaphore_put(&isp_semaphore);
}

static void app_camera_display_pipe_frame_cb(void)
{
//为新加   
   uint8_t *bg_buffer;

    bg_buffer = app_lcd_get_bg_buffer();
    cam_display_frames++;
    cam_display_last_ms = HAL_GetTick();
    //SCB_InvalidateDCache_by_Addr(bg_buffer, LCD_BG_WIDTH * LCD_BG_HEIGHT * 2);
    //为新加
    SCB_CleanDCache_by_Addr(bg_buffer, ALIGN_VALUE(LCD_BG_WIDTH * LCD_BG_HEIGHT * 2, 32));
    
    app_lcd_switch_bg_buffer();
    app_camera_display_pipe_set_address(app_lcd_get_bg_buffer());
}

static void app_camera_nn_pipe_frame_cb(void)
{
    uint8_t *buffer;

    buffer = app_bqueue_get_free(&nn_input_queue, 0);
     cam_nn_frames++;
    cam_nn_last_ms = HAL_GetTick();
    if (buffer != NULL)
    {
        app_camera_nn_pipe_set_address(buffer);
        app_bqueue_put_ready(&nn_input_queue);
    }
}

static VOID nn_thread_entry(ULONG id)
{
    uint32_t nn_out_len;
    uint32_t nn_in_len;
    uint8_t *nn_pipe_dst;
    uint8_t *capture_buffer;
    uint8_t *output_buffer;
    uint32_t nn_period[2];
    uint32_t nn_period_ms;
    uint32_t time_stamp;
    uint32_t inf_ms;

    nn_in_len = LL_Buffer_len(LL_ATON_Input_Buffers_Info_Default());
   nn_out_len = ALIGN_VALUE(LL_Buffer_len(LL_ATON_Output_Buffers_Info_Default()), 32);
   //nn_out_len = ALIGN_VALUE(LL_Buffer_len(LL_ATON_Output_Buffers_Info_Default()), 32);
  // nn_out_len = ALIGN_VALUE(NN_BUFFER_OUT_SIZE, 32);
  //  nn_out_len = LL_Buffer_len(LL_ATON_Output_Buffers_Info_Default());
 //nn_out_len = ALIGN_VALUE(LL_Buffer_len(LL_ATON_Output_Buffers_Info_Default()), 32);
 //nn_out_len = ALIGN_VALUE(LL_Buffer_len(LL_ATON_Output_Buffers_Info_Default()), 32);

//    if (nn_out_len > ALIGN_VALUE(NN_BUFFER_OUT_SIZE, 32))
  //  {
   //     nn_out_len = ALIGN_VALUE(NN_BUFFER_OUT_SIZE, 32);
    //}



    nn_period[1] = HAL_GetTick();

    nn_pipe_dst = app_bqueue_get_free(&nn_input_queue, 0);

    app_camera_nn_pipe_start(nn_pipe_dst, CMW_MODE_CONTINUOUS);

    while (1)
    {
        nn_period[0] = nn_period[1];
        nn_period[1] = HAL_GetTick();
        nn_period_ms = nn_period[1] - nn_period[0];

        capture_buffer = app_bqueue_get_ready(&nn_input_queue);
        output_buffer = app_bqueue_get_free(&nn_output_queue, 1);

        time_stamp = HAL_GetTick();
 SCB_CleanDCache_by_Addr(capture_buffer, ALIGN_VALUE(nn_in_len, 32));    



    LL_ATON_Set_User_Input_Buffer_Default(0, capture_buffer, nn_in_len);
        SCB_InvalidateDCache_by_Addr(output_buffer, nn_out_len);
        LL_ATON_Set_User_Output_Buffer_Default(0, output_buffer, nn_out_len);
        LL_ATON_RT_Main(&NN_Instance_Default);
        
         SCB_InvalidateDCache_by_Addr(output_buffer, nn_out_len);
        
        inf_ms = HAL_GetTick() - time_stamp;

        app_bqueue_put_free(&nn_input_queue);
        app_bqueue_put_ready(&nn_output_queue);

        tx_mutex_get(&display.lock, TX_WAIT_FOREVER);
        display.info.inf_ms = inf_ms;
        display.info.nn_period_ms = nn_period_ms;
        tx_mutex_put(&display.lock);
    }
}

static VOID pp_thread_entry(ULONG id)
{
    app_pp_param_t pp_params;
    //yolov2_pp_static_param_t pp_params;
    uint8_t *output_buffer;
    app_pp_out_t pp_output;
    //od_pp_out_t pp_output;
    uint32_t nn_pp[2];
    int32_t i;

    app_postprocess_init(&pp_params);

    while (1)
    {
        output_buffer = app_bqueue_get_ready(&nn_output_queue);
        pp_output.pOutBuff = NULL;

        nn_pp[0] = HAL_GetTick();
        app_postprocess_run((void *[]){(void *)output_buffer}, 1, &pp_output, &pp_params);
        nn_pp[1] = HAL_GetTick();

        tx_mutex_get(&display.lock, TX_WAIT_FOREVER);
        display.info.nb_detect = pp_output.nb_detect;
        for (i = 0; i < pp_output.nb_detect; i++)
        {
            display.info.detects[i] = pp_output.pOutBuff[i];
        }
        display.info.pp_ms = nn_pp[1] - nn_pp[0];
        tx_mutex_put(&display.lock);
        app_bqueue_put_free(&nn_output_queue);
        tx_semaphore_ceiling_put(&display.update, 1);
    }
}

static VOID dp_thread_entry(ULONG id)
{
    uint32_t disp_ms = 0;
    app_display_info_t display_info;
    uint32_t time_stamp;
uint32_t now_ms;
    while (1)
    {
        tx_semaphore_get(&display.update, TX_WAIT_FOREVER);
        tx_mutex_get(&display.lock, TX_WAIT_FOREVER);
        display_info = display.info;       
       tx_mutex_put(&display.lock);
        display_info.disp_ms = disp_ms;
 now_ms = HAL_GetTick();
        display_info.cam_disp_frames = cam_display_frames;
        display_info.cam_nn_frames = cam_nn_frames;
        display_info.cam_disp_age_ms = now_ms - cam_display_last_ms;
        display_info.cam_nn_age_ms = now_ms - cam_nn_last_ms;
        time_stamp = HAL_GetTick();
        app_display_network_output(&display_info);
        disp_ms = HAL_GetTick() - time_stamp;
        app_camera_try_rearm(now_ms);
    }
}

static VOID isp_thread_entry(ULONG id)
{
    while (1)
    {
        tx_semaphore_get(&isp_semaphore, TX_WAIT_FOREVER);

        app_camera_isp_update();
    }
}
static void app_camera_try_rearm(uint32_t now_ms)
{
    const uint32_t stale_ms = 500U;
    uint8_t need_rearm = 0U;

    if (((now_ms - cam_display_last_ms) > stale_ms && cam_display_frames == 0U) ||
        ((now_ms - cam_nn_last_ms) > stale_ms && cam_nn_frames == 0U))
    {
        need_rearm = 1U;
    }

    if (need_rearm == 0U || (now_ms - cam_last_rearm_ms) < stale_ms)
    {
        return;
    }

    cam_last_rearm_ms = now_ms;

    app_camera_display_pipe_start(app_lcd_get_bg_buffer(), CMW_MODE_CONTINUOUS);

    uint8_t *nn_buf = app_bqueue_get_free(&nn_input_queue, 0);
    if (nn_buf != NULL)
    {
        app_camera_nn_pipe_start(nn_buf, CMW_MODE_CONTINUOUS);
    }
}

static uint8_t app_clamp_point(int32_t *x, int32_t *y)
{
    int32_t xi;
    int32_t yi;

    xi = *x;
    yi = *y;

    if (*x < 0)
    {
        *x = 0;
    }

    if (*y < 0)
    {
        *y = 0;
    }

    if (*x >= LCD_BG_WIDTH)
    {
        *x = LCD_BG_WIDTH - 1;
    }

    if (*y >= LCD_BG_HEIGHT)
    {
        *y = LCD_BG_HEIGHT - 1;
    }

    return (xi != *x) || (yi != *y);
}

//static void app_display_detection(od_pp_outBuffer_t *detect)
#if POSTPROCESS_TYPE == POSTPROCESS_OD_METER_YOLOV5_UI
static void app_display_detection(app_detection_t *detect)
{
    int32_t xc;
    int32_t yc;
    int32_t x0;
    int32_t y0;
    int32_t x1;
    int32_t y1;

    xc = (int32_t)(LCD_BG_WIDTH * detect->x_center);
    yc = (int32_t)(LCD_BG_HEIGHT * detect->y_center);
    x0 = xc - (int32_t)((detect->width * LCD_BG_WIDTH + 1) / 2);
    y0 = yc - (int32_t)((detect->height * LCD_BG_HEIGHT + 1) / 2);
    x1 = xc + (int32_t)((detect->width * LCD_BG_WIDTH + 1) / 2);
    y1 = yc + (int32_t)((detect->height * LCD_BG_HEIGHT + 1) / 2);

    app_clamp_point(&x0, &y0);
    app_clamp_point(&x1, &y1);

    UTIL_LCD_DrawRect(x0, y0, x1 - x0, y1 - y0, UTIL_LCD_COLOR_GREEN);
    UTIL_LCDEx_PrintfAt(x0, y0, LEFT_MODE, nn_classes_table[detect->class_index]);

    for (uint32_t k = 0; k < 4; k++)
    {
        int32_t kx = (int32_t)(LCD_BG_WIDTH * detect->keypoints[k][0]);
        int32_t ky = (int32_t)(LCD_BG_HEIGHT * detect->keypoints[k][1]);
        app_clamp_point(&kx, &ky);
        UTIL_LCD_DrawCircle(kx, ky, 3U, UTIL_LCD_COLOR_RED);
    }
}
#else
static void app_display_detection(app_detection_t *detect)
{
    int32_t xc;
    int32_t yc;
    int32_t x0;
    int32_t y0;
    int32_t x1;
    int32_t y1;
    int32_t w;
    int32_t h;

    xc = (int32_t)(LCD_BG_WIDTH * detect->x_center);
    yc = (int32_t)(LCD_BG_HEIGHT * detect->y_center);
    w = (int32_t)(LCD_BG_WIDTH * detect->width);
    h = (int32_t)(LCD_BG_HEIGHT * detect->height);

    x0 = xc - (w + 1) / 2;
    y0 = yc - (h + 1) / 2;
    x1 = xc + (w + 1) / 2;
    y1 = yc + (h + 1) / 2;

    app_clamp_point(&x0, &y0);
    app_clamp_point(&x1, &y1);

    UTIL_LCD_DrawRect(x0, y0, x1 - x0, y1 - y0, UTIL_LCD_COLOR_GREEN);
    UTIL_LCDEx_PrintfAt(x0, y0, LEFT_MODE, nn_classes_table[detect->class_index]);
}
#endif


static void app_display_network_output(app_display_info_t *display_info)
{
    float cpuload_one_second;
    uint8_t line_nb = 0;
    int32_t i;

    app_lcd_draw_area_update();

    UTIL_LCD_FillRect(0, 0, LCD_FG_WIDTH, LCD_FG_HEIGHT, 0x00000000);

    app_cpuload_update(&cpuload);
    app_cpuload_get_info(&cpuload, NULL, &cpuload_one_second, NULL);

    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "CPU load");
    line_nb += 1;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "%.1f%%", cpuload_one_second);
    line_nb += 2;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "Inference");
    line_nb += 1;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "%ums", display_info->inf_ms);
    line_nb += 2;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "FPS");
    line_nb += 1;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "%.2f", 1000.0 / display_info->nn_period_ms);
    line_nb += 2;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "Detections");
    line_nb += 1;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "%u", display_info->nb_detect);
line_nb += 2;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "Cam1 frames");
    line_nb += 1;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "%lu (%lums)",
                        (unsigned long)display_info->cam_disp_frames,
                        (unsigned long)display_info->cam_disp_age_ms);
    line_nb += 2;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "Cam2 frames");
    line_nb += 1;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "%lu (%lums)",
                        (unsigned long)display_info->cam_nn_frames,
                        (unsigned long)display_info->cam_nn_age_ms);
    for (i = 0; i < display_info->nb_detect; i++)
    {
        app_display_detection(&display_info->detects[i]);
    }

    app_lcd_draw_area_commit();
}
