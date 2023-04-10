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
#include <FL/Fl_Native_File_Chooser.H>
#include <fstream>

#include "util.h"

#define MHz(x) (long)((x) * 1000 * 1000)
#define kHz(x) (long)((x) * 1000)

#define PI 3.141592653589

pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

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
    int device_busy = 0;
    int sample_offset = 0;

    std::complex<float>* fft_time_buffer;
    std::complex<float>* fft_freq_buffer;
    fftwf_plan fft_plan;

    float* scan_buffer;

    // dsp
    float DC_blocker_coef = 1;
    
    // gui
    Fl_Button *start_btn;
    Fl_Progress *progress_bar;
    GUI_Spectre *spectre_box;

    pthread_t tid_tuner;
    pthread_t tid_reception;
} nbima_scan_context;

typedef struct nbima_calculation_context
{
    nbima_scan_context *generator_ctx;
    nbima_scan_context *measured_ctx;
    float* corrected_buffer;
    bool done = false;
};

template void Fl_force_redraw_callback<Fl_Text_Display>(void *arg);

void* thread_tuner(void* arg) {
    nbima_scan_context* ctx = (nbima_scan_context*)arg;
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

    ctx->progress_bar->set_visible();
    
    rtlsdr_set_center_freq(ctx->device, ctx->lower_freq);
    
    ctx->current_freq = ctx->lower_freq;
    ctx->start_btn->deactivate();
    
    while(1) {
        pthread_testcancel();
        
        ctx->current_freq += MHz(1);
        ctx->sample_offset = (ctx->current_freq / MHz(1) - 27) * ctx->frame_size / 2;
        if (ctx->current_freq >= ctx->upper_freq) {
            ctx->current_freq = ctx->lower_freq;
            ctx->sample_offset = 0;
            break;
        }
        rtlsdr_set_center_freq(ctx->device, ctx->current_freq);

        ctx->progress_bar->value((float)(ctx->current_freq - ctx->lower_freq) / (ctx->upper_freq - ctx->lower_freq) * 100);
        
        // Debug info
        uint32_t _freq = rtlsdr_get_center_freq(ctx->device);
        uint32_t _gain = rtlsdr_get_tuner_gain(ctx->device);
        //printf("gain is %d\n", _gain);
        snprintf(info, 1024, "Center frequency: %f MHz\nTuner gain: %f dB", (float)_freq / MHz(1), (float)_gain / 10);
        buf->text(info);

        nbima_sleep(ctx->sleep_period);
    }

    ctx->progress_bar->hide();

    ctx->device_busy = 0;
    ctx->start_btn->activate();
    rtlsdr_cancel_async(ctx->device);

    return 0;
}

// len - size of the u8 buffer
void rtlsdr_cb(uint8_t* buf, uint32_t len, void* arg) {
    nbima_scan_context* ctx = (nbima_scan_context*)arg;
    static float k = 0.8; // averaging
    float R = ctx->DC_blocker_coef;  // DC blocking
    
    //printf("reception %d\n", rand() % 14214);
    
    ctx->device_busy = 1;

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

void thread_reception_cleanup(void* arg) {
    nbima_scan_context* ctx = (nbima_scan_context*)arg;
    
    auto r = rtlsdr_cancel_async(ctx->device);
    if (!r) {
        printf("Reception cancelled\n");
    }
    else {
        printf("Reception cancel error: return code %d\n", r);
    }
}

void* thread_reception(void *arg) {
    nbima_scan_context* ctx = (nbima_scan_context*)arg;
    
    pthread_cleanup_push(thread_reception_cleanup, (void*)ctx);
    
    printf("Parameters: %d, %d, %d, %d\n", ctx->device, rtlsdr_cb, arg, ctx->frame_size * 2);
    //rtlsdr_reset_buffer(ctx->device);
    //auto r = rtlsdr_read_async(ctx->device, rtlsdr_cb, arg, 0, ctx->frame_size * 2);
//    if (r) {
//        pthread_cancel(ctx->tid_tuner);
//        printf("Error: rtlsdr_read_async returned %d\n", r);
//    }
    
    pthread_cleanup_pop(0);
    pthread_exit(0);

    return 0;
}

float get_scan_buffer_length(nbima_scan_context* ctx) {
    return ctx->frame_size / 2 * ((ctx->upper_freq - ctx->lower_freq) / MHz(1));
}

float* allocate_scan_buffer(nbima_scan_context* ctx) {
    int elements_number = get_scan_buffer_length(ctx);
    float *buf = new float[elements_number];
    std::fill(buf, buf + elements_number, -110);
    return buf;
}

std::complex<float>* allocate_fft_buffer(nbima_scan_context* ctx) {
    return new std::complex<float>[ctx->frame_size];
}

void* nbima_scan(void* arg) {
    nbima_scan_context* ctx = (nbima_scan_context*)arg;
    pthread_create(&ctx->tid_tuner, NULL, thread_tuner, ctx);
    rtlsdr_reset_buffer(ctx->device);
    auto r = rtlsdr_read_async(ctx->device, rtlsdr_cb, arg, 0, ctx->frame_size * 2);
    if (r) {
        pthread_cancel(ctx->tid_tuner);
        printf("Error: rtlsdr_read_async returned %d\n", r);
    }
    pthread_join(ctx->tid_tuner, 0);
    return 0;
}

void scan_cb(Fl_Widget* w, void* ctx) {
    pthread_t scan_session_thread;
    
    pthread_create(&scan_session_thread, NULL, nbima_scan, ctx);
}

void noise_stop_cb(Fl_Widget* w, void* arg) {
    nbima_scan_context* ctx = (nbima_scan_context*)arg;
    
    auto r = rtlsdr_cancel_async(ctx->device);
    printf("test %d\n", r);
    //pthread_cancel(ctx->tid_tuner);
}

void input_uint32_cb(Fl_Widget* w, void* arg) {
    *(uint32_t*)arg = MHz(((Fl_Valuator*)w)->value());
}

void c4t_btn_cb(Fl_Widget* w, void* arg) {
    nbima_scan_context *ctx = (nbima_scan_context*)arg;
    
    if (c4t_mes_win == NULL) {
        c4t_mes_win = make_4t_measurement_window();
    }
    c4t_mes_win->show();

    c4t_spectre_box->link_buffer(ctx->scan_buffer, get_scan_buffer_length(ctx));

    start_c4t_scan->callback(scan_cb, ctx);
    stop_c4t_scan->callback(noise_stop_cb, ctx);
    
    ctx->progress_bar = c4t_progress;
    ctx->start_btn = start_c4t_scan;
    
    c4t_lower_input->callback(input_uint32_cb, (void*)&ctx->lower_freq);
    c4t_upper_input->callback(input_uint32_cb, (void*)&ctx->upper_freq);
}

void calibrate_btn_cb(Fl_Widget* w, void* arg) {
    nbima_scan_context *ctx = (nbima_scan_context*)arg;
    
    if (noise_mes_win == NULL) {
        noise_mes_win = make_measurement_window();
    }
    noise_mes_win->show();

    noise_spectre_box->link_buffer(ctx->scan_buffer, get_scan_buffer_length(ctx));

    start_noise_scan->callback(scan_cb, ctx);
    stop_noise_scan->callback(noise_stop_cb, ctx);
    
    ctx->progress_bar = noise_generator_progress;
    ctx->start_btn = start_noise_scan;
    
    noise_lower_input->callback(input_uint32_cb, (void*)&ctx->lower_freq);
    noise_upper_input->callback(input_uint32_cb, (void*)&ctx->upper_freq);
}

int setup_sdr(rtlsdr_dev** dev, nbima_scan_context* scan_context) {
    auto r = rtlsdr_open(dev, 0);
    
    if (r) {
        printf("Could not open an SDR device. Error code: %d\n", r);
        return r;
    }

    rtlsdr_set_sample_rate(*dev, scan_context->sample_rate);
    rtlsdr_set_tuner_gain_mode(*dev, 0);
    rtlsdr_set_tuner_gain(*dev, 0);
    rtlsdr_set_center_freq(*dev, scan_context->lower_freq);
    rtlsdr_reset_buffer(*dev);
    
    return r;
}

void dc_coef_cb(Fl_Widget* w, void* ctx) {
    ((nbima_scan_context*)ctx)->DC_blocker_coef = ((Fl_Value_Input*)w)->value();
    dc_coef_input->value(((Fl_Value_Input*)w)->value());
    dc_coef_slider->value(((Fl_Value_Input*)w)->value());
}

void calculate_cb(Fl_Widget* w, void* arg) {
    nbima_calculation_context* ctx = (nbima_calculation_context*)arg;
    
    for (int i = 0; i < get_scan_buffer_length(ctx->generator_ctx); i++) {
        ctx->corrected_buffer[i] = ctx->measured_ctx->scan_buffer[i] - ctx->generator_ctx->scan_buffer[i];
    }
    
    if (corrected_window == NULL) {
        corrected_window = make_corrected_spectrum_window();
    }
    corrected_window->show();
    corrected_spectre_box->link_buffer(ctx->corrected_buffer, get_scan_buffer_length(ctx->generator_ctx));
    
    ctx->done = true;
}

template <class charT, charT sep>
class punct_facet: public std::numpunct<charT> {
protected:
    charT do_decimal_point() const { return sep; }
};

void save_csv_cb(Fl_Widget* w, void* arg) {
    nbima_calculation_context* ctx = (nbima_calculation_context*)arg;
    
    Fl_Native_File_Chooser fc;
    fc.title("Save to .csv");
    fc.type(Fl_Native_File_Chooser::BROWSE_SAVE_FILE);
    fc.preset_file("Untitled.csv");
    
    auto r = fc.show();
    
    if (r == -1) {
        printf("Error: %s\n", fc.errmsg());
    }
    else if (r == 1) {
        printf("Cancelled\n");
    }
    else {
        printf("File name: %s\n", fc.filename());
        int size = 1673;
        int decimation = get_scan_buffer_length(ctx->measured_ctx) / size;
        std::ofstream fs;
        fs.imbue(std::locale(fs.getloc(), new punct_facet<char, ','>));
        fs.open(fc.filename());
        fs << "kHz;dB\n";
        for (int i = 0; i < size; i++) {
            float freq = (ctx->measured_ctx->upper_freq - ctx->measured_ctx->lower_freq) / size * i / 1000;
            float value = 0;
            for (int j = 0; j < decimation; j++) {
                value += ctx->corrected_buffer[i * decimation + j] / decimation;
            }
            fs << freq << ";" << value << "\n";
        }
        fs.close();
    }
}

int main() {
    // Scan parameters
    nbima_scan_context noise_ctx;
    noise_ctx.lower_freq = MHz(27);
    noise_ctx.upper_freq = MHz(1700);
    noise_ctx.sample_rate = 2048000;
    noise_ctx.frame_size = 4096 / 2;

    noise_ctx.sleep_period = 0; // [ms]
    noise_ctx.spectre_decimation = 1;

    // Set up an SDR device
    rtlsdr_dev_t* device;
    setup_sdr(&device, &noise_ctx);

    noise_ctx.device = device;

    // Set up spectrum buffers for the default configuration
    noise_ctx.scan_buffer = allocate_scan_buffer(&noise_ctx);

    noise_ctx.fft_time_buffer = allocate_fft_buffer(&noise_ctx);
    noise_ctx.fft_freq_buffer = allocate_fft_buffer(&noise_ctx);
    noise_ctx.fft_plan        = fftwf_plan_dft_1d(noise_ctx.frame_size,
                                              reinterpret_cast<fftwf_complex*>(noise_ctx.fft_time_buffer), 
                                              reinterpret_cast<fftwf_complex*>(noise_ctx.fft_freq_buffer), 
                                              FFTW_FORWARD, 
                                              FFTW_MEASURE);
    
    nbima_scan_context c4t_ctx = noise_ctx;
    
    c4t_ctx.scan_buffer = allocate_scan_buffer(&c4t_ctx);
    
    // Frequency correction
    nbima_calculation_context calc_ctx;
    calc_ctx.generator_ctx = &noise_ctx;
    calc_ctx.measured_ctx = &c4t_ctx;
    calc_ctx.corrected_buffer = allocate_scan_buffer(&noise_ctx);
    
    // Start GUI
#ifdef unix
    XInitThreads();
#endif
    Fl::visible_focus(0);
    auto window = make_window();
    window->show();

    btn_calibrate->callback(calibrate_btn_cb, &noise_ctx);
    btn_4t->callback(c4t_btn_cb, &c4t_ctx);
    btn_calculate->callback(calculate_cb, &calc_ctx);
    
    csv_btn->callback(save_csv_cb, &calc_ctx);
//    dc_coef_input->callback(dc_coef_cb, &scan_context);
//    dc_coef_slider->callback(dc_coef_cb, &scan_context);

    return Fl::run();
}
