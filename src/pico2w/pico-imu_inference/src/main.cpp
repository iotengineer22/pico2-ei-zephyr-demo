/*
 * SPDX-License-Identifier: Apache-2.0
 */

// --- Zephyr RTOS includes ---
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

// --- Edge Impulse SDK includes ---
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

// --- Constants and Macros ---
#define STANDARD_GRAVITY 9.80665f

// Sampling parameters
#define SAMPLING_RATE_HZ            104
#define SAMPLING_PERIOD_MS          (1000 / SAMPLING_RATE_HZ)

// Compile-time check for expected sensor axis count
#if EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3
#error "This implementation assumes 3-axis accelerometer data."
#endif

// Calculate the number of samples needed per inference window
#define NUM_SAMPLES_PER_INFERENCE   (EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE / EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME)

// Register the logging module
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// --- Global Variables and Device Definitions ---

// Static buffer to hold the features for one inference window
static float features_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

// Assert that the console is configured to use USB CDC ACM
BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
             "Console device is not ACM CDC UART device");

// Get the console device handle from the device tree.
// 'extern' is used to give this const global variable external linkage in C++,
// allowing it to be referenced from other files (like the Edge Impulse SDK porting layer).
extern const struct device *const uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

// Get the MPU-6050 sensor device handle from the device tree
static const struct device *const mpu6050 = DEVICE_DT_GET_ANY(invensense_mpu6050);

// --- Function Prototypes ---
static void setup_mpu6050(void);
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr);

// --- Main Application Entry Point ---
extern "C" int main(void)
{
    // 1. Initialize USB and wait for a connection
    uint32_t dtr = 0;

    if (usb_enable(NULL) != 0) {
        LOG_ERR("Failed to enable USB");
        return 1;
    }

    LOG_INF("Waiting for USB connection...");
    while (!dtr) {
        uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
        k_sleep(K_MSEC(100));
    }
    LOG_INF("USB connection established.");

    // 2. Set up the sensor
    setup_mpu6050();

    // 3. Main loop for data collection and inference
    struct sensor_value accel[3];

    while (1) {
        // --- Phase 1: Collect data for one inference window ---
        LOG_INF("Collecting %d samples for the next inference...", NUM_SAMPLES_PER_INFERENCE);

        for (uint32_t i = 0; i < NUM_SAMPLES_PER_INFERENCE; i++) {
            int64_t start_time_ms = k_uptime_get();

            if (sensor_sample_fetch(mpu6050) == 0) {
                sensor_channel_get(mpu6050, SENSOR_CHAN_ACCEL_XYZ, accel);

                size_t current_index = i * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
                features_buffer[current_index + 0] = (float)sensor_value_to_double(&accel[0]) / STANDARD_GRAVITY;
                features_buffer[current_index + 1] = (float)sensor_value_to_double(&accel[1]) / STANDARD_GRAVITY;
                features_buffer[current_index + 2] = (float)sensor_value_to_double(&accel[2]) / STANDARD_GRAVITY;
            } else {
                LOG_WRN("Failed to read sensor data, retrying sample %u...", i);
                i--; // Decrement counter to retry fetching this sample
            }

            // Sleep for the remaining time in the sampling period to maintain the sample rate
            int64_t processing_time_ms = k_uptime_get() - start_time_ms;
            int sleep_time_ms = SAMPLING_PERIOD_MS - (int)processing_time_ms;
            if (sleep_time_ms > 0) {
                k_sleep(K_MSEC(sleep_time_ms));
            }
        }
        LOG_INF("Data collection complete.");

        // --- Phase 2: Run inference on the collected data ---
        signal_t features_signal;
        features_signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
        features_signal.get_data = &raw_feature_get_data;

        ei_impulse_result_t result = {0};
        EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);

        if (res != EI_IMPULSE_OK) {
            LOG_ERR("Classifier returned error: %d", res);
        } else {
            LOG_INF("Predictions (DSP: %d ms, Classification: %d ms, Anomaly: %d ms):",
                    result.timing.dsp, result.timing.classification, result.timing.anomaly);
            for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                LOG_INF("    %s: %.5f", result.classification[ix].label,
                                        result.classification[ix].value);
            }
        #if EI_CLASSIFIER_HAS_ANOMALY == 1
            LOG_INF("    anomaly score: %.3f", result.anomaly);
        #endif
        }
    }

    return 0; // Should not be reached
}

// --- Function Definitions ---

/**
 * @brief Initializes the MPU-6050 sensor.
 */
static void setup_mpu6050(void)
{
    if (!device_is_ready(mpu6050)) {
        LOG_ERR("MPU-6050: Device is not ready.");
        return;
    }
    LOG_INF("MPU-6050: Device is ready.");
}

/**
 * @brief Callback function to provide raw sample data to the Edge Impulse SDK.
 *
 * This function is called by the classifier to get slices of the feature buffer.
 *
 * @param offset Starting index of the data to fetch.
 * @param length Number of float values to fetch.
 * @param out_ptr Pointer to the buffer where the data should be copied.
 * @return 0 on success.
 */
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr)
{
    memcpy(out_ptr, features_buffer + offset, length * sizeof(float));
    return 0;
}