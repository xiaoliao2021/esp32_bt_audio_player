// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include <driver/i2s.h>
#include "driver/i2c.h"

#include "esp_bt.h"
#include "bt_app_core.h"
#include "bt_app_av.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "driver/i2s.h"
#include "lvgl.h"
#include "lvgl_helpers.h"
#include "lv_examples/src/lv_demo_widgets/lv_demo_widgets.h"
/*********************
 *      DEFINES
 *********************/
#define TAG "demo"
#define LV_TICK_PERIOD_MS 1

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_tick_task(void *arg);
static void guiTask(void *pvParameter);
static void create_demo_application(void);

/* Test Tone Sample Array */
const short int SIN64[64] = {0, 3136, 6242, 9289, 12245, 15084, 17778, 20300, 22627, 24736, 26607, 28221, 29564, 30622, 31385, 31845, 32000,
                             31845, 31385, 30622, 29564, 28221, 26607, 24736, 22627, 20300, 17778, 15084, 12245, 9289, 6242, 3136, 0,
                             -3137, -6243, -9290, -12246, -15085, -17779, -20301, -22628, -24737, -26608, -28222, -29565, -30623, -31386, -31846, -32000,
                             -31846, -31386, -30623, -29565, -28222, -26608, -24737, -22628, -20301, -17779, -15085, -12246, -9290, -6243, -3137};
/* Creates a semaphore to handle concurrent call to lvgl stuff
 * If you wish to call *any* lvgl function from other threads/tasks
 * you should lock on the very same semaphore! */
SemaphoreHandle_t xGuiSemaphore;

static void guiTask(void *pvParameter)
{

    (void)pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();

    lv_init();

    /* Initialize SPI or I2C bus used by the drivers */
    lvgl_driver_init();

    lv_color_t *buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);

    /* Use double buffered when not working with monochrome displays */
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    lv_color_t *buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2 != NULL);
#else
    static lv_color_t *buf2 = NULL;
#endif

    static lv_disp_buf_t disp_buf;

    uint32_t size_in_px = DISP_BUF_SIZE;

    /* Initialize the working buffer depending on the selected display.
     * NOTE: buf2 == NULL when using monochrome displays. */
    lv_disp_buf_init(&disp_buf, buf1, buf2, size_in_px);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;

    /* When using a monochrome display we need to register the callbacks:
     * - rounder_cb
     * - set_px_cb */
    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"};
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    /* Create the demo application */
    create_demo_application();

    while (1)
    {
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* Try to take the semaphore, call lvgl related function on success */
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
        {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
        }
    }

    /* A task should NEVER return */
    free(buf1);
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    free(buf2);
#endif
    vTaskDelete(NULL);
}

static void create_demo_application(void)
{
    /* When using a monochrome display we only show "Hello World" centered on the
     * screen */
    lv_demo_widgets();
}

static void lv_tick_task(void *arg)
{
    (void)arg;
    lv_tick_inc(LV_TICK_PERIOD_MS);
}

/* Task Tone Generator plays 100ms long beeps on left channel every second. */
void audio_play_task(void *pvParameter)
{
    signed short samples[256];
    size_t written;
    int total_chunks = 375;

    // fs = 48000 kHz
    // SIN64 - frequency: fs / 64 = 750 Hz
    // We need to generate N samples per second: fs*2 = 96000
    // each time we send 256 so total amount of chuncks per second is 96000/256=375.
    // samples (i&1)==0 (0, 2, 4, ...) go to the left channel
    // samples (i&1)==1 (1, 3, 5, ...) go to the right channel
    while (1)
    {
        for (int j = 0; j < total_chunks; j++)
        {
            for (int i = 0; i < 256; i++)
            {
                if ((i & 1) == 0)
                {
                    samples[i] = SIN64[((i >> 1) & 63)] * (j < 37);
                }
                else
                {
                    samples[i] = 0;
                }
            }
            i2s_write(0, (const char *)samples, 512, &written, portMAX_DELAY);
        }
    }
}

#define MCLK_GPIO 0
#define I2C_ADDR 26
#define TAG "Main"

/* event for handler "bt_av_hdl_stack_up */
enum
{
    BT_APP_EVT_STACK_UP = 0,
};

/* handler for bluetooth stack enabled events */
static void bt_av_hdl_stack_evt(uint16_t event, void *p_param);

// initiation table for non-readbale 9-bit i2c registers
static uint16_t WM8978_REGVAL_TBL[58] =
    {
        0X0000, 0X0000, 0X0000, 0X0000, 0X0050, 0X0000, 0X0140, 0X0000,
        0X0000, 0X0000, 0X0000, 0X00FF, 0X00FF, 0X0000, 0X0100, 0X00FF,
        0X00FF, 0X0000, 0X012C, 0X002C, 0X002C, 0X002C, 0X002C, 0X0000,
        0X0032, 0X0000, 0X0000, 0X0000, 0X0000, 0X0000, 0X0000, 0X0000,
        0X0038, 0X000B, 0X0032, 0X0000, 0X0008, 0X000C, 0X0093, 0X00E9,
        0X0000, 0X0000, 0X0000, 0X0000, 0X0003, 0X0010, 0X0010, 0X0100,
        0X0100, 0X0002, 0X0001, 0X0001, 0X0039, 0X0039, 0X0039, 0X0039,
        0X0001, 0X0001};

/****************************************************************************************
 * handler for writing to WN8978 i2c register using 9 bits
 */
static esp_err_t i2c_write_reg(uint8_t reg, uint16_t val)
{
    esp_err_t ret;
    char buf[2];
    buf[0] = (reg << 1) | ((val >> 8) & 0X01);
    buf[1] = val & 0XFF;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (I2C_ADDR << 1) | I2C_MASTER_WRITE, I2C_MASTER_NACK);
    i2c_master_write_byte(cmd, buf[0], I2C_MASTER_NACK);
    i2c_master_write_byte(cmd, buf[1], I2C_MASTER_NACK);

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(0, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "I2C write failed");
    }
    else
    {
        // update local register
        WM8978_REGVAL_TBL[reg] = val;
        ESP_LOGI(TAG, "I2C Write OK %d=%d (bytes %d %d)", reg, val, buf[0], buf[1]);
    }

    return ret;
}

/****************************************************************************************
 *  Return local register value
 */
static uint16_t i2c_read_reg(uint8_t reg)
{
    return WM8978_REGVAL_TBL[reg];
}

void app_main()
{
    /* Initialize NVS — it is used to store PHY calibration data */
    xTaskCreatePinnedToCore(guiTask, "gui", 4096 * 2, NULL, 0, NULL, 1);
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // configure i2c bus
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 19,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 18,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 250000,
    };

    i2c_param_config(0, &i2c_config);
    i2c_driver_install(0, I2C_MODE_MASTER, false, false, false);

    ESP_LOGI(TAG, "DAC uses I2C @%d with sda:%d, scl:%d", I2C_ADDR, i2c_config.sda_io_num, i2c_config.scl_io_num);

    // initialialize DAC
    i2c_write_reg(0, 0);   // set defaults
    i2c_write_reg(1, 11);  // enable ports
    i2c_write_reg(2, 384); // enable ports
    i2c_write_reg(3, 111); // enable ports
    i2c_write_reg(4, 16);  // configure I2C
    i2c_write_reg(6, 0);   // configure MCLK
    i2c_write_reg(10, 8);  //
    i2c_write_reg(14, 0);
    i2c_write_reg(43, 16);
    i2c_write_reg(49, 102);

    i2c_write_reg(52, 63);  // set max volume
    i2c_write_reg(53, 319); // set max volume
    i2c_write_reg(54, 63);  // set max volume
    i2c_write_reg(55, 319); // set max volume

    // initialize I2C bus
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX, // Only TX
        .sample_rate = 44100,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, //2-channels
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .dma_buf_count = 6,
        .dma_buf_len = 60,
        .intr_alloc_flags = 0, //ESP_INTR_FLAG_LEVEL1,                                                  //Default interrupt priority
        //.use_apll = true,
        .tx_desc_auto_clear = true //Auto clear tx descriptor on underflow
    };

    i2s_driver_install(0, &i2s_config, 0, NULL);

    i2s_pin_config_t pin_config = {
        .mck_io_num = 3,
        .bck_io_num = CONFIG_EXAMPLE_I2S_BCK_PIN,
        .ws_io_num = CONFIG_EXAMPLE_I2S_LRCK_PIN,
        .data_out_num = CONFIG_EXAMPLE_I2S_DATA_PIN,
        .data_in_num = -1 //Not used
    };

    i2s_set_pin(0, &pin_config);

    // Configure system clk to MCLK pin
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
    REG_WRITE(PIN_CTRL, 0xFFFFFFF0);

    ESP_LOGI(TAG, "DAC using I2S bck:%d, ws:%d, do:%d, mclk:%d", pin_config.bck_io_num, pin_config.ws_io_num, pin_config.data_out_num, MCLK_GPIO);

    /* DEBUG:  Start test tone and block
    xTaskCreate(&audio_play_task, "audio_play_task", 4096, NULL, 8, NULL);
    while (1)
    {
        vTaskDelay(5000 / portTICK_RATE_MS);
        ESP_LOGI(TAG, "Heart beat");
    }
    */

    /* A2DP sink ESP-IDF example code */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((err = esp_bt_controller_init(&bt_cfg)) != ESP_OK)
    {
        ESP_LOGE(BT_AV_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(err));
        return;
    }

    if ((err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK)
    {
        ESP_LOGE(BT_AV_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(err));
        return;
    }

    if ((err = esp_bluedroid_init()) != ESP_OK)
    {
        ESP_LOGE(BT_AV_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(err));
        return;
    }

    if ((err = esp_bluedroid_enable()) != ESP_OK)
    {
        ESP_LOGE(BT_AV_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(err));
        return;
    }

    /* create application task */
    bt_app_task_start_up();

    /* Bluetooth device name, connection mode and profile set up */
    bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use fixed pin code
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code;
    pin_code[0] = '1';
    pin_code[1] = '2';
    pin_code[2] = '3';
    pin_code[3] = '4';
    esp_bt_gap_set_pin(pin_type, 4, pin_code);
}

void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
    {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(BT_AV_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(BT_AV_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        }
        else
        {
            ESP_LOGE(BT_AV_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    default:
    {
        ESP_LOGI(BT_AV_TAG, "event: %d", event);
        break;
    }
    }
    return;
}
static void bt_av_hdl_stack_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_AV_TAG, "%s evt %d", __func__, event);
    switch (event)
    {
    case BT_APP_EVT_STACK_UP:
    {
        /* set up device name */
        char *dev_name = "ESP_SPEAKER";
        esp_bt_dev_set_device_name(dev_name);

        esp_bt_gap_register_callback(bt_app_gap_cb);

        /* initialize AVRCP controller */
        esp_avrc_ct_init();
        esp_avrc_ct_register_callback(bt_app_rc_ct_cb);
        /* initialize AVRCP target */
        assert(esp_avrc_tg_init() == ESP_OK);
        esp_avrc_tg_register_callback(bt_app_rc_tg_cb);

        esp_avrc_rn_evt_cap_mask_t evt_set = {0};
        esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
        assert(esp_avrc_tg_set_rn_evt_cap(&evt_set) == ESP_OK);

        /* initialize A2DP sink */
        esp_a2d_register_callback(&bt_app_a2d_cb);
        esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);
        esp_a2d_sink_init();

        /* set discoverable and connectable mode, wait to be connected */
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event);
        break;
    }
}
