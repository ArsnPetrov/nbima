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
    buf->text(info);
    // uint32_t _buffer_size_MiB = sizeof(float) * ctx->frame_size * 
    //                       (ctx->upper_freq - ctx->lower_freq) / MHz(1) * 0.5 
    //                       / 1024 / 1024;
    nbima_sleep(ctx->sleep_period);

    noise_generator_progress->set_visible();
    
    while(1) {
        ctx->current_freq += MHz(1);
        ctx->sample_offset += ctx->frame_size / 2;
        if (ctx->current_freq >= ctx->upper_freq) {
            ctx->current_freq = ctx->lower_freq;
            ctx->sample_offset = 0;
            break;
        }
        rtlsdr_set_center_freq(ctx->device, ctx->current_freq);

        noise_generator_progress->value((float)(ctx->current_freq - ctx->lower_freq) / (ctx->upper_freq - ctx->lower_freq) * 100);
        
        // Debug info
        uint32_t _freq = rtlsdr_get_center_freq(ctx->device);
        uint32_t _gain = rtlsdr_get_tuner_gain(ctx->device);
        printf("gain is %d\n", _gain);
        snprintf(info, 1024, "Center frequency: %f MHz\nTuner gain: %f dB", (float)_freq / MHz(1), (float)_gain / 10);

        nbima_sleep(ctx->sleep_period);
    }

    noise_generator_progress->hide();

    ctx->on = 0;
    start_noise_scan->value(0);
    pthread_cancel(ctx->tid_reception);

    return 0;
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

        ctx->scan_buffer[i + ctx->sample_offset] += (1 - k) * 20 * log10(norm(ctx->fft_freq_buffer[n]) / (ctx->frame_size*ctx->frame_size) / (256 * 256));
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
    std::fill(buf, buf + elements_number, -110);
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

void noise_scan_cb(Fl_Widget* w, void* ctx) {
    nbima_scan((nbima_scan_context*)ctx);
}

void calibrate_btn_cb(Fl_Widget* w, void* ctx) {
    if (!((nbima_scan_context*)ctx)->on) {
        auto mes_window = make_measurement_window();
        mes_window->show();

        noise_spectre_box->link_buffer(((nbima_scan_context*)ctx)->scan_buffer, ((nbima_scan_context*)ctx)->frame_size * 500);

        start_noise_scan->callback(noise_scan_cb, ctx);
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
    nbima_scan_context noise_generator_scan;
    noise_generator_scan.lower_freq = MHz(40);
    noise_generator_scan.upper_freq = MHz(1040);
    noise_generator_scan.sample_rate = 2048000;
    noise_generator_scan.frame_size = 4096 / 2;

    noise_generator_scan.sleep_period = 0; // [ms]
    noise_generator_scan.spectre_decimation = 1;

    // Set up an SDR device
    rtlsdr_dev_t* device;
    setup_sdr(&device, &noise_generator_scan);

    noise_generator_scan.device = device;

    // Set up spectrum buffers for the default configuration
    noise_generator_scan.scan_buffer = allocate_scan_buffer(&noise_generator_scan);

    noise_generator_scan.fft_time_buffer = allocate_fft_buffer(&noise_generator_scan);
    noise_generator_scan.fft_freq_buffer = allocate_fft_buffer(&noise_generator_scan);
    noise_generator_scan.fft_plan        = fftwf_plan_dft_1d(noise_generator_scan.frame_size,
                                              reinterpret_cast<fftwf_complex*>(noise_generator_scan.fft_time_buffer), 
                                              reinterpret_cast<fftwf_complex*>(noise_generator_scan.fft_freq_buffer), 
                                              FFTW_FORWARD, 
                                              FFTW_MEASURE);
    
    // Start GUI
#ifdef unix
    XInitThreads();
#endif
    Fl::visible_focus(0);
    auto window = make_window();
    window->show();

    btn_calibrate->callback(calibrate_btn_cb, &noise_generator_scan);
//    dc_coef_input->callback(dc_coef_cb, &scan_context);
//    dc_coef_slider->callback(dc_coef_cb, &scan_context);

    return Fl::run();
}
