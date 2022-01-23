/* Multiple pipelines playback with downmix.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "i2s_stream.h"
#include "mp3_decoder.h"
#include "fatfs_stream.h"
#include "downmix.h"
#include "filter_resample.h"
#include "raw_stream.h"
#include "board.h"
#include "periph_sdcard.h"
#include "periph_button.h"

/**
esp_err_t esp_periph_send_event(esp_periph_handle_t periph, int event_id,
                                void *data, int data_len) {
    if (periph->on_evt == NULL) {
        return ESP_FAIL;
    }
    audio_event_iface_msg_t msg;
    msg.source_type = periph->periph_id;
    msg.cmd = event_id;
    msg.data = data;
    msg.data_len = data_len;
    msg.need_free_data = false;
    msg.source = periph;

    if (periph->on_evt->cb) {
        periph->on_evt->cb(&msg, periph->on_evt->user_ctx);
    }
    return audio_event_iface_sendout(periph->on_evt->iface, &msg);
} */

static const char *TAG = "MIX";

#define NUM_OF_INPUTS 2
#define SAMPLE_RATE 48000
#define BITS_PER_SAMPLE 16
#define NUM_OF_INPUT_CHANNEL 2
#define TRANSITION 1000

static char softap_ssid[32] = "JubenshaGateway";
static char softap_pass[32] = {};

static esp_periph_set_handle_t set = NULL;

static esp_event_handler_instance_t instance_any_id;
static esp_event_handler_instance_t instance_got_ip;

static audio_element_handle_t fat[NUM_OF_INPUTS] = {};
static audio_element_handle_t dec[NUM_OF_INPUTS] = {};
static audio_element_handle_t rsp[NUM_OF_INPUTS] = {};
static audio_element_handle_t raw[NUM_OF_INPUTS] = {};
static audio_pipeline_handle_t input[NUM_OF_INPUTS] = {};

static bool running[NUM_OF_INPUTS] = {};
static bool destroying = false;

static audio_element_handle_t mixer = NULL;
static audio_element_handle_t writer = NULL;
static audio_pipeline_handle_t output = NULL;

static audio_event_iface_handle_t evt = NULL;

static void setup_input(int i) {
    fatfs_stream_cfg_t fatfs_cfg = FATFS_STREAM_CFG_DEFAULT();
    fatfs_cfg.type = AUDIO_STREAM_READER;
    fat[i] = fatfs_stream_init(&fatfs_cfg);

    mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
    mp3_cfg.out_rb_size = (16 * 1024);
    mp3_cfg.task_core = 1;
    mp3_cfg.stack_in_ext = false;
    dec[i] = mp3_decoder_init(&mp3_cfg);

    rsp_filter_cfg_t rsp_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_cfg.src_rate = 48000;
    rsp_cfg.src_ch = 2;
    rsp_cfg.dest_rate = 48000;
    rsp_cfg.dest_ch = 2;
    rsp_cfg.task_core = 1;
    rsp_cfg.out_rb_size = (16 * 1024);
    rsp[i] = rsp_filter_init(&rsp_cfg);

    raw_stream_cfg_t raw_cfg = RAW_STREAM_CFG_DEFAULT();
    raw_cfg.type = AUDIO_STREAM_WRITER;
    raw_cfg.out_rb_size = (16 * 1024);
    raw[i] = raw_stream_init(&raw_cfg);

    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    input[i] = audio_pipeline_init(&pipeline_cfg);
    mem_assert(input[i]);
    audio_pipeline_register(input[i], fat[i], "fat");
    audio_pipeline_register(input[i], dec[i], "dec");
    audio_pipeline_register(input[i], rsp[i], "rsp");
    audio_pipeline_register(input[i], raw[i], "raw");

    const char *tags[4] = {"fat", "dec", "rsp", "raw"};
    audio_pipeline_link(input[i], &tags[0], 4);
}

static void setup_mixer() {
    ESP_LOGI(TAG, "[ ? ] Setup Mixer");
    downmix_cfg_t cfg = DEFAULT_DOWNMIX_CONFIG();
    cfg.downmix_info.source_num = NUM_OF_INPUTS;
    audio_element_handle_t mx = downmix_init(&cfg);

    esp_downmix_input_info_t source_info[NUM_OF_INPUTS] = {};
    for (int i = 0; i < NUM_OF_INPUTS; i++) {
        if (i == 0) {
            esp_downmix_input_info_t info = {
                .samplerate = SAMPLE_RATE,
                .channel = NUM_OF_INPUT_CHANNEL,
                .bits_num = BITS_PER_SAMPLE,
                .gain = {0, -20},
                .transit_time = TRANSITION,
            };
            source_info[i] = info;
        } else {
            esp_downmix_input_info_t info = {
                .samplerate = SAMPLE_RATE,
                .channel = NUM_OF_INPUT_CHANNEL,
                .bits_num = BITS_PER_SAMPLE,
                .gain = {-20, 0},
                .transit_time = TRANSITION,
            };
            source_info[i] = info;
        }
    }

    // TODO there are other apis to do this
    source_info_init(mx, source_info);

    for (int i = 0; i < NUM_OF_INPUTS; i++) {
        downmix_set_input_rb(mx, audio_element_get_input_ringbuf(raw[i]), i);
        downmix_set_input_rb_timeout(mx, 0, i);
    }

    downmix_set_output_type(mx, ESP_DOWNMIX_OUTPUT_TYPE_TWO_CHANNEL);
    downmix_set_out_ctx_info(mx, ESP_DOWNMIX_OUT_CTX_NORMAL);
    mixer = mx;
}

static void setup_output() {
    ESP_LOGI(TAG, "[ * ] Setup Output Pipeline");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    writer = i2s_stream_init(&i2s_cfg);
    i2s_stream_set_clk(writer, SAMPLE_RATE, BITS_PER_SAMPLE,
                       ESP_DOWNMIX_OUTPUT_TYPE_TWO_CHANNEL);

    audio_pipeline_cfg_t ppl_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    output = audio_pipeline_init(&ppl_cfg);

    audio_pipeline_register(output, mixer, "mixer");
    audio_pipeline_register(output, writer, "i2s");

    const char *tags[2] = {"mixer", "i2s"};
    audio_pipeline_link(output, &tags[0], 2);
}

static void setup_listeners() {
    ESP_LOGI(TAG, "[ * ] Setup Listeners");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    evt = audio_event_iface_init(&evt_cfg);
    for (int i = 0; i < NUM_OF_INPUTS; i++) {
        audio_pipeline_set_listener(input[i], evt);
    }
    audio_pipeline_set_listener(output, evt);
    audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);
}

static void switch_mode (bool mode) {
    if (mode) {
        downmix_set_work_mode(mixer, ESP_DOWNMIX_WORK_MODE_SWITCH_ON);
        ESP_LOGI(TAG, "foreground sound entering");
    } else {
        downmix_set_work_mode(mixer, ESP_DOWNMIX_WORK_MODE_SWITCH_OFF);
        ESP_LOGI(TAG, "foreground sound leaving");
    }
}

static void run_input(int i, const char * uri) {
    if (running[i]) {
        ESP_LOGI(TAG, "input %d running", i);
        return;
    }
    audio_element_set_uri(fat[i], uri);
    audio_pipeline_run(input[i]);
    running[i] = true;
    if (i == 1) {
        switch_mode(true);
    }
}

static void reset_input(int i) {
    audio_pipeline_reset_ringbuffer(input[i]);
    audio_pipeline_reset_elements(input[i]); 
    audio_pipeline_change_state(input[i], AEL_STATE_INIT);
}

static void handle_audio_element_finished(void *src) {
    if (src == mixer) {
        ESP_LOGI(TAG, "mixer finished");
    } else  if (src == writer) {
        ESP_LOGI(TAG, "writer finished");
    } else {
        for (int i = 0; i < NUM_OF_INPUTS; i++) {
            if (src == fat[i]) {
                ESP_LOGI(TAG, "fat %d finished", i);
                return;
            } else if (src == dec[i]) {
                ESP_LOGI(TAG, "dec %d finished", i);
                return;
            } else if (src == rsp[i]) {
                ESP_LOGI(TAG, "rsp %d finished", i);
                reset_input(i);
                running[i] = false;
                if (i == 1)
                    switch_mode(false);
                return;
            } else if (src == raw[i]) {
                ESP_LOGI(TAG, "raw %d finished", i);
                return;
            }
        }
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "connecting to ap");
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "reconnecting to ap");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) { 
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static void setup_wifi() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    assert(ap_netif);

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL,
        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL,
        &instance_got_ip));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    wifi_config_t cfg1 = {
        .sta =
            {
                .ssid = "juwanke",
                .password = "juwanke!",
                .threshold.authmode = WIFI_AUTH_WPA2_PSK,
                .pmf_cfg = {.capable = true, .required = false},
            },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg1));

    wifi_config_t cfg2 = {
        .ap =
            {
                .ssid = "",
                .ssid_len = 0,
                .max_connection = 8,
                .authmode = WIFI_AUTH_OPEN,
            },
    };

    /* sizeof ap.ssid is 32 */
    strlcpy((char *)cfg2.ap.ssid, softap_ssid, sizeof(cfg2.ap.ssid));
    cfg2.ap.ssid_len = strlen(softap_ssid);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &cfg2));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void app_main(void) {
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES
		|| ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_LOGI(TAG, "[1.0] Start audio codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE,
                         AUDIO_HAL_CTRL_START);

    ESP_LOGI(TAG, "[2.0] Start and wait for SDCARD to mount");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    set = esp_periph_set_init(&periph_cfg);
    audio_board_sdcard_init(set, SD_MODE_1_LINE);
    audio_board_key_init(set);

    setup_wifi();

    for (int i = 0; i < NUM_OF_INPUTS; i++) {
        setup_input(i);
    }
    setup_mixer();
    setup_output();
    setup_listeners();
    switch_mode(false);
    audio_pipeline_run(output);

    while (1) {
        audio_event_iface_msg_t msg;

        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
            continue;
        }

        /* set src info on-the-fly */
        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT &&
            msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
            for (int i = 0; i < NUM_OF_INPUTS; i++) {
                if (msg.source == (void *)dec[i]) {
                    audio_element_info_t fi = {}, di = {};
                    audio_element_getinfo(fat[i], &fi); 
                    audio_element_getinfo(dec[i], &di);
                    rsp_filter_set_src_info(rsp[i], di.sample_rates,
                                            di.channels);
                    ESP_LOGI(TAG,
                             "[ * ] play: %s, "
                             "sample rates: %d, bits: %d, ch: %d @ input %d",
                             fi.uri, di.sample_rates, di.bits, di.channels, i);
                }
            }
        }

        /* handle button event */
        if (msg.cmd == PERIPH_BUTTON_PRESSED) {
            int id = (int)msg.data;
            if (id == get_input_rec_id()) {
                run_input(1, "/sdcard/monster.mp3");
            } else if (id == get_input_mode_id()) {
                run_input(1, "/sdcard/nangong.mp3");
            } else if (id == get_input_play_id()) {
                run_input(0, "/sdcard/fall.mp3");
            } else if (id == get_input_set_id()) {
                run_input(0, "/sdcard/battle.mp3");
            }
            continue;
        }

        /* handle finished event */
        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT &&
            msg.cmd == AEL_MSG_CMD_REPORT_STATUS &&
            (int)msg.data == AEL_STATUS_STATE_FINISHED) {
            handle_audio_element_finished((void *)msg.source);
        }
    }

    ESP_LOGI(TAG, "[7.0] Stop all pipelines");
    /* Stop base stream pipeline, Release resources */
    audio_pipeline_stop(input[0]);
    audio_pipeline_wait_for_stop(input[0]);
    audio_pipeline_terminate(input[0]);
    audio_pipeline_unregister_more(input[0], fat[0], dec[0], rsp[0],
                                   raw[0], NULL);
    audio_pipeline_remove_listener(input[0]);
    audio_pipeline_deinit(input[0]);
    audio_element_deinit(fat[0]);
    audio_element_deinit(dec[0]);
    audio_element_deinit(rsp[0]);
    audio_element_deinit(raw[0]);

    /* Stop newcome stream pipeline, Release resources */
    audio_pipeline_stop(input[1]);
    audio_pipeline_wait_for_stop(input[1]);
    audio_pipeline_terminate(input[1]);
    audio_pipeline_unregister_more(input[1], fat[1], dec[1], rsp[1],
                                   raw[1], NULL);
    audio_pipeline_remove_listener(input[1]);
    audio_pipeline_deinit(input[1]);
    audio_element_deinit(fat[1]);
    audio_element_deinit(dec[1]);
    audio_element_deinit(rsp[1]);
    audio_element_deinit(raw[1]);

    /* Stop mixer stream pipeline, Release resources */
    audio_pipeline_stop(output);
    audio_pipeline_wait_for_stop(output);
    audio_pipeline_terminate(output);
    audio_pipeline_unregister_more(output, mixer, writer, NULL);
    audio_pipeline_remove_listener(output);

    /* Stop all peripherals before removing the listener */
    esp_periph_set_stop_all(set);
    audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);

    /* Make sure audio_pipeline_remove_listener &
     * audio_event_iface_remove_listener are called before destroying
     * event_iface */
    audio_event_iface_destroy(evt);

    /* Release resources */
    audio_pipeline_deinit(output);
    audio_element_deinit(mixer);
    audio_element_deinit(writer);
    esp_periph_set_destroy(set);
}
