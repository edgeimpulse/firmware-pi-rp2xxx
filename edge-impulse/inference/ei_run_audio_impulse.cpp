/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "model-parameters/model_metadata.h"

#if defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_MICROPHONE

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include "ei_device_raspberry_rp2xxx.h"
#include "ei_microphone.h"
#include "ei_run_impulse.h"

typedef enum
{
    INFERENCE_STOPPED,
    INFERENCE_WAITING,
    INFERENCE_SAMPLING,
    INFERENCE_DATA_READY
} inference_state_t;

static int print_results;
static uint16_t samples_per_inference;
static inference_state_t state = INFERENCE_STOPPED;
static uint64_t last_inference_ts = 0;
static bool continuous_mode = false;
static bool debug_mode = false;
static float samples_circ_buff[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
static int samples_wr_index = 0;

void ei_run_impulse(void)
{
    switch (state) {
    case INFERENCE_STOPPED:
        // nothing to do
        return;
    case INFERENCE_WAITING:
        if (ei_read_timer_ms() < (last_inference_ts + 2000)) {
            return;
        }
        state = INFERENCE_SAMPLING;
        ei_microphone_inference_reset_buffers();
        break;
    case INFERENCE_SAMPLING:
        // wait for data to be collected through callback
        if (ei_microphone_inference_is_recording()) {
            return;
        }
        state = INFERENCE_DATA_READY;
        break;
        // nothing to do, just continue to inference provcessing below
    case INFERENCE_DATA_READY:
    default:
        break;
    }

    signal_t signal;

    signal.total_length = continuous_mode ? EI_CLASSIFIER_SLICE_SIZE
                                          : EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &ei_microphone_inference_get_data;

    // run the impulse: DSP, neural network and the Anomaly algorithm
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR ei_error;
    if (continuous_mode == true) {
        ei_error = run_classifier_continuous(&signal, &result, debug_mode);
    }
    else {
        ei_error = run_classifier(&signal, &result, debug_mode);
    }
    if (ei_error != EI_IMPULSE_OK) {
        ei_printf("Failed to run impulse (%d)", ei_error);
        return;
    }

    if (continuous_mode == true) {
        if (++print_results >= (EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW >> 1)) {
            display_results(&ei_default_impulse, &result);
            print_results = 0;
        }
    }
    else {
        display_results(&ei_default_impulse, &result);
    }

    if (continuous_mode == true) {
        state = INFERENCE_SAMPLING;
    }
    else {
        ei_printf("Starting inferencing in 2 seconds...\n");
        last_inference_ts = ei_read_timer_ms();
        state = INFERENCE_WAITING;
    }
}

void ei_start_impulse(bool continuous, bool debug, bool use_max_uart_speed)
{
    const float sample_length = 1000.0f * static_cast<float>(EI_CLASSIFIER_RAW_SAMPLE_COUNT) /
        (1000.0f / static_cast<float>(EI_CLASSIFIER_INTERVAL_MS));

    continuous_mode = continuous;
    debug_mode = debug;

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: %fms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %f ms.\n", sample_length);
    ei_printf(
        "\tNo. of classes: %d\n",
        sizeof(ei_classifier_inferencing_categories) /
            sizeof(ei_classifier_inferencing_categories[0]));
    ei_printf("Starting inferencing, press 'b' to break\n");

    if (continuous == true) {
        samples_per_inference = EI_CLASSIFIER_SLICE_SIZE * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
        // In order to have meaningful classification results, continuous inference has to run over
        // the complete model window. So the first iterations will print out garbage.
        // We now use a fixed length moving average filter of half the slices per model window and
        // only print when we run the complete maf buffer to prevent printing the same classification multiple times.
        print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);
        run_classifier_init();
        state = INFERENCE_SAMPLING;
    }
    else {
        samples_per_inference = EI_CLASSIFIER_RAW_SAMPLE_COUNT *
            EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
        // it's time to prepare for sampling
        ei_printf("Starting inferencing in 2 seconds...\n");
        last_inference_ts = ei_read_timer_ms();
        state = INFERENCE_WAITING;
    }

    if (ei_microphone_inference_start(
            continuous_mode ? EI_CLASSIFIER_SLICE_SIZE : EI_CLASSIFIER_RAW_SAMPLE_COUNT,
            EI_CLASSIFIER_INTERVAL_MS) == false) {
        ei_printf(
            "ERR: Could not allocate audio buffer (size %d), this could be due to the window "
            "length of your model\r\n",
            EI_CLASSIFIER_RAW_SAMPLE_COUNT);
        return;
    }

    while (!ei_user_invoke_stop()) {
        ei_run_impulse();
        ei_sleep(1);
    }

    ei_stop_impulse();
}

void ei_stop_impulse(void)
{
    if (state != INFERENCE_STOPPED) {
        ei_printf("Inferencing stopped by user\r\n");
        // EiDevice.set_state(eiStateFinished);
        /* reset samples buffer */
        samples_wr_index = 0;
        ei_microphone_inference_end();
        run_classifier_deinit();
    }
    state = INFERENCE_STOPPED;
}

bool is_inference_running(void)
{
    return (state != INFERENCE_STOPPED);
}

#endif /* defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_MICROPHONE */