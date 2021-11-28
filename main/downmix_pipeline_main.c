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

static const char *TAG = "MIX";

#define INPUT1_INDEX 0
#define INPUT2_INDEX 1
#define SAMPLE_RATE 48000
#define BITS_PER_SAMPLE 16
#define NUM_OF_INPUT_CHANNEL 2

#define TRANSITION 1000
#define MUSIC_GAIN_DB 0
#define NUM_OF_INPUTS 2

static esp_periph_set_handle_t set = NULL;

static audio_element_handle_t fat[NUM_OF_INPUTS] = {};
static audio_element_handle_t dec[NUM_OF_INPUTS] = {};
static audio_element_handle_t rsp[NUM_OF_INPUTS] = {};
static audio_element_handle_t raw[NUM_OF_INPUTS] = {};
static audio_pipeline_handle_t input[NUM_OF_INPUTS] = {};

static bool running[NUM_OF_INPUTS] = {};
static bool output_running = false;
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

static void switch_mode (bool b) {
    if (b) {
        downmix_set_work_mode(mixer, ESP_DOWNMIX_WORK_MODE_SWITCH_ON);
        ESP_LOGI(TAG, "fg switched on");
    } else {
        downmix_set_work_mode(mixer, ESP_DOWNMIX_WORK_MODE_SWITCH_OFF);
        ESP_LOGI(TAG, "fg switched off");
    }
}

static void run_output() {
    if (!output_running) {
        audio_pipeline_reset_ringbuffer(output);
        audio_pipeline_reset_elements(output);
        audio_pipeline_change_state(output, AEL_STATE_INIT);
        audio_pipeline_run(output);
        output_running = true;
    }
}

static void handle_ae_finished(void *src) {
    if (src == mixer) {
        ESP_LOGI(TAG, "mixer finished");
    } else  if (src == writer) {
        ESP_LOGI(TAG, "writer finished");
        output_running = false;
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

static void handle_ae_stopped(void *src) {
    if (src == mixer) {
        ESP_LOGI(TAG, "mixer stopped");
    } else  if (src == writer) {
        ESP_LOGI(TAG, "writer stopped");
    } else {
        for (int i = 0; i < NUM_OF_INPUTS; i++) {
            if (src == fat[i]) {
                ESP_LOGI(TAG, "fat %d stopped", i);
                return;
            } else if (src == dec[i]) {
                ESP_LOGI(TAG, "dec %d stopped", i);
                return;
            } else if (src == rsp[i]) {
                ESP_LOGI(TAG, "rsp %d stopped", i);
                return;
            } else if (src == raw[i]) {
                ESP_LOGI(TAG, "raw %d stopped", i);
                return;
            }
        }
    }
}

static void handle_rec_button() {
    const int i = 0;
    ESP_LOGI(TAG, "rec button pressed (input %d)", i);
    if (running[i]) {
        ESP_LOGI(TAG, "input %d running", i);
        return;
    }

    audio_element_set_uri(fat[i], "/sdcard/fall.mp3");
    audio_pipeline_reset_ringbuffer(input[i]);
    audio_pipeline_reset_elements(input[i]); 
    audio_pipeline_change_state(input[i], AEL_STATE_INIT);
    audio_pipeline_run(input[i]);
    running[i] = true;
    run_output();
}

static void handle_mode_button() {
    const int i = 1;
    ESP_LOGI(TAG, "mode button pressed (input %d)", i);
    if (running[i]) {
        ESP_LOGI(TAG, "input %d running", i);
        return;
    }

    audio_element_set_uri(fat[i], "/sdcard/nangong.mp3");
    audio_pipeline_reset_ringbuffer(input[i]);
    audio_pipeline_reset_elements(input[i]); 
    audio_pipeline_change_state(input[i], AEL_STATE_INIT);
    audio_pipeline_run(input[i]);
    switch_mode(true);
    running[i] = true;
    run_output();
}

static void handle_play_button() {
    ESP_LOGI(TAG, "play button pressed");
}

static void handle_set_button() {
    ESP_LOGI(TAG, "set button pressed");
}

void app_main(void) {
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "[1.0] Start audio codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE,
                         AUDIO_HAL_CTRL_START);

    ESP_LOGI(TAG, "[2.0] Start and wait for SDCARD to mount");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    set = esp_periph_set_init(&periph_cfg);
    audio_board_sdcard_init(set, SD_MODE_1_LINE);
    audio_board_key_init(set);

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
                handle_rec_button();
            } else if (id == get_input_mode_id()) {
                handle_mode_button();
            } else if (id == get_input_play_id()) {
                handle_play_button();
            } else if (id == get_input_set_id()) {
                handle_set_button();
            }
            continue;
        }

        /* handle finished event */
        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT &&
            msg.cmd == AEL_MSG_CMD_REPORT_STATUS &&
            (int)msg.data == AEL_STATUS_STATE_FINISHED) {
            handle_ae_finished((void *)msg.source);
        }

        /* handle stopped event */
        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT &&
            msg.cmd == AEL_MSG_CMD_REPORT_STATUS &&
            (int)msg.data == AEL_STATUS_STATE_STOPPED) {
            handle_ae_stopped((void *)msg.source);
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
