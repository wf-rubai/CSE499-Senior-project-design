/* Edge Impulse Arduino examples
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// These sketches are tested with 2.0.4 ESP32 Arduino Core
// https://github.com/espressif/arduino-esp32/releases/tag/2.0.4

/* Includes ---------------------------------------------------------------- */
#include <Person_Detection_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"

// Select camera model - find more camera models in camera_pins.h file here
// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h

// #define CAMERA_MODEL_ESP_EYE // Has PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#if defined(CAMERA_MODEL_ESP_EYE)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    4
#define SIOD_GPIO_NUM    18
#define SIOC_GPIO_NUM    23

#define Y9_GPIO_NUM      36
#define Y8_GPIO_NUM      37
#define Y7_GPIO_NUM      38
#define Y6_GPIO_NUM      39
#define Y5_GPIO_NUM      35
#define Y4_GPIO_NUM      14
#define Y3_GPIO_NUM      13
#define Y2_GPIO_NUM      34
#define VSYNC_GPIO_NUM   5
#define HREF_GPIO_NUM    27
#define PCLK_GPIO_NUM    25

#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define PIN_HUMAN_DETECTED   12   
#define PIN_SAMPLE_PULSE     13   
#define NUM_SAMPLES          5       
#define HUMAN_THRESHOLD      0.3f
#define SAMPLE_INTERVAL_MS   350

#else
#error "Camera model not selected"
#endif

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf; // Points to the output of the capture

// ============================================================
// SLIDING WINDOW BUFFERS
// Instead of a simple sum that resets every 5 samples,
// we store the last NUM_SAMPLES values in circular arrays.
// Each new reading overwrites the oldest one (FIFO behaviour).
// A decision is produced after EVERY new sample once the
// window is full — not just every 5th sample.
//
// Example timeline:
//   Sample 1 → buffer: [1, -, -, -, -]  window not full yet, no decision
//   Sample 2 → buffer: [1, 2, -, -, -]  window not full yet, no decision
//   Sample 3 → buffer: [1, 2, 3, -, -]  window not full yet, no decision
//   Sample 4 → buffer: [1, 2, 3, 4, -]  window not full yet, no decision
//   Sample 5 → buffer: [1, 2, 3, 4, 5]  window full → decision from avg(1,2,3,4,5)
//   Sample 6 → buffer: [6, 2, 3, 4, 5]  window full → decision from avg(2,3,4,5,6)
//   Sample 7 → buffer: [6, 7, 3, 4, 5]  window full → decision from avg(3,4,5,6,7)
// ============================================================
static float   human_window[NUM_SAMPLES]    = {0}; 
static float   nonhuman_window[NUM_SAMPLES] = {0}; 
static uint8_t window_index                 = 0;   
static uint8_t samples_collected            = 0;   

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)                  //######################################################//
    .xclk_freq_hz = 20000000,                                                   //                                                      //
    .ledc_timer = LEDC_TIMER_0,                                                 //                                                      //
    .ledc_channel = LEDC_CHANNEL_0,                                             //                                                      //
    //                                                                          //                                                      //
    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG              //  This portion is for the camera mode setup. All of   //
    .frame_size = FRAMESIZE_QVGA,   //Do not use sizes above QVGA               //  the settings are based on Video streaming. So frame //
    // Other options you can try:                                               //  size can not be more than 320x240 px. Using a lower //
    // FRAMESIZE_QQVGA2 (128x160)                                               //  JPEG quality gives a smoother video resulting a     //
    // FRAMESIZE_QCIF   (176x144)                                               //  better frame rate and less lock pixels yelling a    //
    // FRAMESIZE_QVGA   (320x240)                                               //  better result on camera move.                       //
    //                                                                          //                                                      //
    .jpeg_quality = 40, //0-63 lower number means higher quality                //                                                      //
    .fb_count = 1,      //if more than one, i2s runs in continuous mode         //                                                      //
    .fb_location = CAMERA_FB_IN_PSRAM,                                          //                                                      //
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,                                        //######################################################//
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);

void print_decision(float avg_human, float avg_nonhuman) {
    // avg_human >= HUMAN_THRESHOLD (0.3) → YES, human present
    if (avg_human >= HUMAN_THRESHOLD) {
        ei_printf("  DECISION  :  >>> YES - HUMAN DETECTED <<<\n");
        ei_printf("  CONFIDENCE:  %.1f%%\n", avg_human * 100.0f);
        digitalWrite(PIN_HUMAN_DETECTED, HIGH);
    } else {
        ei_printf("  DECISION  :  >>> NO - NO HUMAN DETECTED <<<\n");
        ei_printf("  CONFIDENCE:  %.1f%% (below %.0f%% threshold)\n",
                avg_human * 100.0f, HUMAN_THRESHOLD * 100.0f);
        digitalWrite(PIN_HUMAN_DETECTED, LOW);
    }

    ei_printf("========================================\n\n");
}

/**
* @brief      Arduino setup function
*/
void setup()
{
    Serial.begin(115200);
    // Comment out the below line to start inference immediately after upload
    while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");

    // Print configuration so user can confirm settings in Serial Monitor
    ei_printf("Config: sliding window=%d | %.0fms interval | decision every ~%.0fms | %.0f%% threshold\n",
              NUM_SAMPLES,
              (float)SAMPLE_INTERVAL_MS,
              (float)(NUM_SAMPLES * SAMPLE_INTERVAL_MS),
              HUMAN_THRESHOLD * 100.0f);

    if (ei_camera_init() == false) {
        ei_printf("Failed to initialize Camera!\r\n");
    } else {
        ei_printf("Camera initialized\r\n");
    }

    ei_printf("\nStarting continuous inference in 2 seconds...\n");
    ei_sleep(2000);
    pinMode(PIN_HUMAN_DETECTED, OUTPUT);
    pinMode(PIN_SAMPLE_PULSE,   OUTPUT);
    digitalWrite(PIN_HUMAN_DETECTED, LOW);
    digitalWrite(PIN_SAMPLE_PULSE,   LOW);
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop()
{
    if (ei_sleep(SAMPLE_INTERVAL_MS) != EI_IMPULSE_OK) {
        return;
    }

    // Allocate frame buffer for raw RGB888 image
    // Size = width * height * 3 bytes (R, G, B one byte each)
    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    // Check if allocation was successful
    if (snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    // Capture image and resize/crop to model's expected input dimensions
    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        return;
    }

    // Run the classifier on the captured frame
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        free(snapshot_buf);
        return;
    }

    // Print raw timing info for this individual inference
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
    }

#else
    ei_printf("Predictions:\r\n");                                              //#################################################//
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {                  //                                                 //
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);           //  Print the prediction results (classification)  //
        ei_printf("%.5f\r\n", result.classification[i].value);                  //                                                 //
    }                                                                           //#################################################//
#endif

    // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

#if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
    ei_printf("Visual anomalies:\r\n");
    for (uint32_t i = 0; i < result.visual_ad_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.visual_ad_grid_cells[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
    }
#endif

                                                    //#################################################################//
                                                    // SLIDING WINDOW ACCUMULATION LOGIC                               //
                                                    // Write the new sample into the circular buffer at window_index,  //
                                                    // overwriting the oldest value. Then advance the index with       //
                                                    // wrap-around so it always stays within [0, NUM_SAMPLES-1].       //
                                                    //#################################################################//


    float human_val    = result.classification[1].value;    // index 1 = "human"
    float nonhuman_val = result.classification[0].value;    // index 0 = "non human"

    if (human_val > 0.3){
        digitalWrite(PIN_SAMPLE_PULSE, HIGH);
    }else{
        digitalWrite(PIN_SAMPLE_PULSE, LOW);
    }

    human_window[window_index]    = human_val;
    nonhuman_window[window_index] = nonhuman_val;

    window_index = (window_index + 1) % NUM_SAMPLES;

    if (samples_collected < NUM_SAMPLES) {
        samples_collected++;
    }

    ei_printf("New sample | Human: %.4f | Non-Human: %.4f | Buffer filled: %d/%d\n",
              human_val, nonhuman_val, samples_collected, NUM_SAMPLES);

    if (samples_collected >= NUM_SAMPLES) {

        float human_sum    = 0.0f;
        float nonhuman_sum = 0.0f;

        for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
            human_sum    += human_window[i];
            nonhuman_sum += nonhuman_window[i];
        }

        float avg_human    = human_sum    / (float)NUM_SAMPLES;
        float avg_nonhuman = nonhuman_sum / (float)NUM_SAMPLES;

        print_decision(avg_human, avg_nonhuman);
    }

    free(snapshot_buf);
}

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {

    if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
#endif

    // Initialize the camera with the config struct defined above
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    // Initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);      // Flip it back
        s->set_brightness(s, 1); // Up the brightness just a bit
        s->set_saturation(s, 0); // Lower the saturation
    }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_awb_gain(s, 1);
#endif

    is_initialised = true;
    return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {

    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK) {
        ei_printf("Camera deinit failed\n");
        return;
    }

    is_initialised = false;
    return;
}

/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

    // Convert JPEG frame buffer to RGB888 format for the classifier
    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

    esp_camera_fb_return(fb); // Return frame buffer to driver immediately after conversion

    if (!converted) {
        ei_printf("Conversion failed\n");
        return false;
    }

    // Resize/crop only if captured frame doesn't match model input dimensions
    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
            out_buf,
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            out_buf,
            img_width,
            img_height);
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // Recalculate byte offset from pixel offset (3 bytes per pixel in RGB888)
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        // Swap BGR to RGB here
        // due to https://github.com/espressif/esp32-camera/issues/379
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];

        // Advance to the next pixel
        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }

    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif