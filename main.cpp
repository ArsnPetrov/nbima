/* A. A. Petrov, 2023 */

#include "GUI.h"
#include <rtl-sdr.h>
#include <stdio.h>
#include <pthread.h>
#include <math.h>
#ifdef _WIN32
#include <windows.h>
#endif
#ifdef unix
#include <unistd.h>
#include <X11/Xlib.h>
#endif
#include <cmath>
#include <complex>
#include <fftw3.h>

#define MHz(x) (long)((x) * 1000 * 1000)
#define kHz(x) (long)((x) * 1000)

#define PI 3.141592653589

typedef struct
{
    rtlsdr_dev_t* device;
    uint64_t lower_freq;
    uint64_t upper_freq;
    uint64_t current_freq = 0;
    uint64_t sample_rate = 1;
    uint32_t frame_size = 4096;
    uint32_t spectre_decimation = 1;
    uint32_t sleep_period; // ms
    int on = 0;

    std::complex<float>* fft_time_buffer;
    std::complex<float>* fft_freq_buffer;
    fftwf_plan fft_plan;

    float* scan_buffer;

    pthread_t tid_tuner;
    pthread_t tid_reception;
} nbima_scan_context;

inline void nbima_sleep(int sleepMs)
{
#ifdef _WIN32
	Sleep(sleepMs);
#else
	usleep(sleepMs * 1000);
#endif
}

void Fl_Text_Display_force_redraw_callback(void *arg)
{
	((Fl_Text_Display*)arg)->redraw();
	Fl::repeat_timeout(0.01, Fl_Text_Display_force_redraw_callback, arg);
}

void* thread_tuner(void* arg) {
    static nbima_scan_context* ctx = (nbima_scan_context*)arg;
    if (ctx->current_freq == 0) {
        ctx->current_freq = ctx->lower_freq;
    }

    // Debug info
    char* info = (char*)malloc(1024 * sizeof(char));
    Fl_Text_Buffer* buf = new Fl_Text_Buffer();
    debug_info_panel->buffer(buf);
    Fl::add_timeout(0.01, Fl_Text_Display_force_redraw_callback, debug_info_panel);
    uint32_t _buffer_size_MiB = sizeof(float) * ctx->frame_size * 
                          (ctx->upper_freq - ctx->lower_freq) / MHz(1) * 0.5 
                          / ctx->spectre_decimation / 1024 / 1024;

    while(1) {
        ctx->current_freq += MHz(1);
        if (ctx->current_freq >= ctx->upper_freq) {
            ctx->current_freq = ctx->lower_freq;
        }
        rtlsdr_set_center_freq(ctx->device, ctx->current_freq);
        
        // Debug info
        uint32_t _freq = rtlsdr_get_center_freq(ctx->device);
        snprintf(info, 1024, "Center frequency: %f MHz\nScan buffer size: %d MiB\0", (float)_freq / MHz(1), _buffer_size_MiB);
        buf->text(info);

        nbima_sleep(ctx->sleep_period);
    }
}

// len - size of the u8 buffer
void rtlsdr_cb(uint8_t* buf, uint32_t len, void* arg) {
    static nbima_scan_context* ctx = (nbima_scan_context*)arg;

    // u8[] -> std::complex<float>[]
    for (int i = 0; i < ctx->frame_size; i++) {
        (ctx->fft_time_buffer[i]).real((float)buf[2*i]);
        (ctx->fft_time_buffer[i]).imag((float)buf[2*i + 1]);
    }

    fftwf_execute(ctx->fft_plan);

    // fft amplitude -> dB PSD
    
}

void* thread_reception(void *arg) {
    nbima_scan_context* ctx = (nbima_scan_context*)arg;

    rtlsdr_read_async(ctx->device, rtlsdr_cb, arg, 0, ctx->frame_size * 2);

    return 0;
}

float* allocate_scan_buffer(nbima_scan_context* ctx) {
    return new float[ctx->frame_size * 
                    (ctx->upper_freq - ctx->lower_freq) / MHz(1) / 2 
                     / ctx->spectre_decimation];
}

std::complex<float>* allocate_fft_buffer(nbima_scan_context* ctx) {
    return new std::complex<float>[ctx->frame_size / ctx->spectre_decimation];
}

void nbima_scan(nbima_scan_context* ctx) {
    if (!ctx->on) {
        ctx->on = 1;
        pthread_create(&ctx->tid_tuner, NULL, thread_tuner, ctx);
        pthread_create(&ctx->tid_reception, NULL, thread_reception, ctx);
    }
}

void calibrate_btn_cb(Fl_Widget* w, void* ctx) {
    if (!((nbima_scan_context*)ctx)->on) {
        nbima_scan((nbima_scan_context*)ctx);
    }
}

int main() {
    // Scan parameters
    nbima_scan_context scan_context;
    scan_context.lower_freq = MHz(100);
    scan_context.upper_freq = MHz(200);
    scan_context.sample_rate = MHz(2);
    scan_context.frame_size = 2048;

    scan_context.sleep_period = 100; // [ms]
    scan_context.spectre_decimation = 1;

    // Spectrum buffers

    // Set up an SDR device
    rtlsdr_dev_t* device;
    auto r = rtlsdr_open(&device, 0);

    if (r) {
        printf("Could not open an SDR device. Error code: %d\n", r);
    }

    rtlsdr_set_sample_rate(device, scan_context.sample_rate);
    rtlsdr_set_tuner_gain_mode(device, 0);
    rtlsdr_set_center_freq(device, scan_context.lower_freq);
    rtlsdr_reset_buffer(device);

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

    btn_calibrate->callback(calibrate_btn_cb, &scan_context);

    return Fl::run();
}