/* A. A. Petrov, 2023 */

#include "GUI.h"
#include <rtl-sdr.h>
#include <stdio.h>
#include <pthread.h>
#include <math.h>
#include <cmath>
#include <complex>
#include <fftw3.h>
#include <algorithm>

#include "util.h"

#define MHz(x) (long)((x) * 1000 * 1000)
#define kHz(x) (long)((x) * 1000)

#define PI 3.141592653589

typedef struct nbima_scan_context
{
    rtlsdr_dev_t* device;
    uint32_t lower_freq;
    uint32_t upper_freq;
    uint32_t current_freq = 0;
    uint32_t sample_rate = 1;
    uint32_t frame_size = 4096;
    uint32_t spectre_decimation = 1;
    uint32_t sleep_period; // ms
    int on = 0;
    int sample_offset = 0;

    std::complex<float>* fft_time_buffer;
    std::complex<float>* fft_freq_buffer;
    fftwf_plan fft_plan;

    float* scan_buffer;

    // dsp
    float DC_blocker_coef = 1;

    pthread_t tid_tuner;
    pthread_t tid_reception;
} nbima_scan_context;

template void Fl_force_redraw_callback<Fl_Text_Display>(void *arg);

void* thread_tuner(void* arg) {
    static nbima_scan_context* ctx = (nbima_scan_context*)arg;
    if (ctx->current_freq == 0) {
        ctx->current_freq = ctx->lower_freq;
    }

    // Debug info
    char* info = (char*)malloc(1024 * sizeof(char));
    Fl_Text_Buffer* buf = new Fl_Text_Buffer();
    debug_info_panel->buffer(buf);
    Fl::add_timeout(0.01, Fl_force_redraw_callback<Fl_Text_Display>, debug_info_panel);
    // uint32_t _buffer_size_MiB = sizeof(float) * ctx->frame_size * 
    //                       (ctx->upper_freq - ctx->lower_freq) / MHz(1) * 0.5 
    //                       / 1024 / 1024;

    while(1) {
        ctx->current_freq += MHz(1);
        ctx->sample_offset += ctx->frame_size / 2;
        if (ctx->current_freq >= ctx->upper_freq) {
            ctx->current_freq = ctx->lower_freq;
            ctx->sample_offset = 0;
        }
        rtlsdr_set_center_freq(ctx->device, ctx->current_freq);
        
        // Debug info
        uint32_t _freq = rtlsdr_get_center_freq(ctx->device);
        uint32_t _gain = rtlsdr_get_tuner_gain(ctx->device);
        printf("gain is %d\n", _gain);
        snprintf(info, 1024, "Center frequency: %f MHz\nTuner gain: %f dB", (float)_freq / MHz(1), (float)_gain / 10);
        buf->text(info);

        nbima_sleep(ctx->sleep_period);
    }
}

// len - size of the u8 buffer
void rtlsdr_cb(uint8_t* buf, uint32_t len, void* arg) {
    static nbima_scan_context* ctx = (nbima_scan_context*)arg;
    static float k = 0.8; // averaging
    float R = ctx->DC_blocker_coef;  // DC blocking

    // u8[] -> std::complex<float>[]
    for (int i = 0; i < ctx->frame_size; i++) {
        (ctx->fft_time_buffer[i]).real((float)buf[2*i]);
        (ctx->fft_time_buffer[i]).imag((float)buf[2*i + 1]);
    }

    // DC filter
    std::complex<float> dcf_buffer[2] = { 0, 0 };
    for (int i = 0; i < ctx->frame_size; i++) {
        dcf_buffer[1] = dcf_buffer[0];
        dcf_buffer[0] = ctx->fft_time_buffer[i];
        ctx->fft_time_buffer[i] = ctx->fft_time_buffer[i] - dcf_buffer[1] + R * ((i - 1 >= 0) ? ctx->fft_time_buffer[i - 1] : 0);
        // printf("%f\n", norm(ctx->fft_time_buffer[i]));
    }

    fftwf_execute(ctx->fft_plan);

    // fft amplitude -> dB PSD
    for (int i = 0; i < ctx->frame_size / 2; i++) {
        ctx->scan_buffer[i + ctx->sample_offset] *= k;

        int n = (ctx->frame_size * 3 / 4 + i) % ctx->frame_size;

        ctx->scan_buffer[i + ctx->sample_offset] += (1 - k) * 10 * log10(norm(ctx->fft_freq_buffer[n]));
    }
}

void* thread_reception(void *arg) {
    nbima_scan_context* ctx = (nbima_scan_context*)arg;

    rtlsdr_read_async(ctx->device, rtlsdr_cb, arg, 0, ctx->frame_size * 2);

    return 0;
}

float* allocate_scan_buffer(nbima_scan_context* ctx) {
    int elements_number = ctx->frame_size / 2 * ((ctx->upper_freq - ctx->lower_freq) / MHz(1));
    float *buf = new float[elements_number];
    std::fill(buf, buf + elements_number, 70);
    return buf;
}

std::complex<float>* allocate_fft_buffer(nbima_scan_context* ctx) {
    return new std::complex<float>[ctx->frame_size];
}

void nbima_scan(nbima_scan_context* ctx) {
    if (!ctx->on) {
        ctx->on = 1;
        pthread_create(&ctx->tid_reception, NULL, thread_reception, ctx);
        pthread_create(&ctx->tid_tuner, NULL, thread_tuner, ctx);
    }
}

void calibrate_btn_cb(Fl_Widget* w, void* ctx) {
    if (!((nbima_scan_context*)ctx)->on) {
        nbima_scan((nbima_scan_context*)ctx);
    }
}

int setup_sdr(rtlsdr_dev** dev, nbima_scan_context* scan_context) {
    auto r = rtlsdr_open(dev, 0);
    
    if (r) {
        printf("Could not open an SDR device. Error code: %d\n", r);
        return r;
    }

    rtlsdr_set_sample_rate(*dev, scan_context->sample_rate);
    rtlsdr_set_tuner_gain_mode(*dev, 0);
    //rtlsdr_set_tuner_gain(*dev, 115);
    rtlsdr_set_center_freq(*dev, scan_context->lower_freq);
    rtlsdr_reset_buffer(*dev);
    
    return r;
}

void dc_coef_cb(Fl_Widget* w, void* ctx) {
    ((nbima_scan_context*)ctx)->DC_blocker_coef = ((Fl_Value_Input*)w)->value();
    dc_coef_input->value(((Fl_Value_Input*)w)->value());
    dc_coef_slider->value(((Fl_Value_Input*)w)->value());
}

int main() {
    // Scan parameters
    nbima_scan_context scan_context;
    scan_context.lower_freq = MHz(30);
    scan_context.upper_freq = MHz(1030);
    scan_context.sample_rate = 2048000;
    scan_context.frame_size = 4096 / 2;

    scan_context.sleep_period = 0; // [ms]
    scan_context.spectre_decimation = 1;

    // Set up an SDR device
    rtlsdr_dev_t* device;
    setup_sdr(&device, &scan_context);

    scan_context.device = device;

    // Set up spectrum buffers for the default configuration
    scan_context.scan_buffer = allocate_scan_buffer(&scan_context);

    scan_context.fft_time_buffer = allocate_fft_buffer(&scan_context);
    scan_context.fft_freq_buffer = allocate_fft_buffer(&scan_context);
    scan_context.fft_plan = fftwf_plan_dft_1d(scan_context.frame_size,
                                              reinterpret_cast<fftwf_complex*>(scan_context.fft_time_buffer), 
                                              reinterpret_cast<fftwf_complex*>(scan_context.fft_freq_buffer), 
                                              FFTW_FORWARD, 
                                              FFTW_MEASURE);
    
    // Start GUI
#ifdef unix
    XInitThreads();
#endif
    auto window = make_window();
    window->show();
    auto mes_window = make_measurement_window();
    mes_window->show();
    
    noise_spectre_box->link_buffer(scan_context.scan_buffer, scan_context.frame_size * 500);

    btn_calibrate->callback(calibrate_btn_cb, &scan_context);
//    dc_coef_input->callback(dc_coef_cb, &scan_context);
//    dc_coef_slider->callback(dc_coef_cb, &scan_context);

    return Fl::run();
}
