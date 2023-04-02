//
//  GUI_Spectre.cpp
//  nbima
//
//  Created by Arseny on 23.03.2023.
//

#include "GUI_Spectre.hpp"
#include <cfloat>
#include <algorithm>
#include <cmath>
#include <string>
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

GUI_Spectre::GUI_Spectre(int x, int y, int w, int h, const char* l) : Fl_Box(x, y, w, h, l) {
    buffer = NULL;
    buffer_size = 0;
};

void GUI_Spectre::link_buffer(float* buf, int s) {
    buffer = buf;
    buffer_size = s;

    Fl::add_timeout(0.01, Fl_force_redraw_callback<GUI_Spectre>, this);
};

//void GUI_Spectre::resize_buffer(int size) {};

void GUI_Spectre::draw() {
    fl_draw_box(FL_FLAT_BOX, x(), y(), w(), h(), FL_BLACK);
    
    float min = FLT_MAX;
    float max = -FLT_MAX;
    
    int decimation = buffer_size / 1000;
    
    for (int i = 0; i < buffer_size / decimation; i++) {
        float val = 0;
        for (int j = 0; j < decimation; j++) {
            val += buffer[decimation * i + j] / decimation;
        }
        
        if (std::isinf(val)) {
            continue;
        }
        
        if (val < min) {
            min = val;
        }
        
        if (val > max) {
            max = val;
        }
    }
    
    float span = max - min;
    
    float scale = 0.8 * h() / span;
    
    float bias = (0.1 * h() - min * scale) / scale;
    
    if (buffer_size == 0) {
        printf("Buffer not found\n");
        return;
    }
    
    fl_font(FL_HELVETICA, 11);
    fl_color(FL_DARK3);
    float line_value = (int)min / 2 * 2;
    float line_step = span == 0 ? 1 : std::pow(2, (int)std::log2f(span / 4));
    while(line_value < max + line_step) {
        fl_begin_line();
        fl_vertex(x(),       y() + h() - (bias + line_value) * scale);
        fl_vertex(x() + w(), y() + h() - (bias + line_value) * scale);
        fl_end_line();

        std::string freq_str = std::to_string((int)line_value);
		freq_str += " dB";
        fl_draw(freq_str.c_str(), x() + 10, y() + h() - (bias + line_value) * scale);
        line_value += line_step;
    }
    
//    // Min line
//    fl_color(FL_BLUE);
//    fl_begin_line();
//    fl_vertex(x(),       y() + h() - (bias + min) * scale);
//    fl_vertex(x() + w(), y() + h() - (bias + min) * scale);
//    fl_end_line();
//
//    // Max line
//    fl_color(FL_RED);
//    fl_begin_line();
//    fl_vertex(x(),       y() + h() - (bias + max) * scale);
//    fl_vertex(x() + w(), y() + h() - (bias + max) * scale);
//    fl_end_line();
    
    // Graph
    fl_color(FL_WHITE);
    fl_begin_line();
    for (int i = 0; i < buffer_size / decimation; i++) {
        float val = 0;
        for (int j = 0; j < decimation; j++) {
            val += buffer[decimation * i + j] / decimation;
        }

        float _x = x() + (float)w() / (buffer_size - 1) * (decimation * (i - 1));
        float _y = y() + h() - (val + bias) * scale;
        
        if (std::isnan(_y) || std::isinf(_y)) {
            continue;
        }
        
        fl_vertex(_x, _y);
    }
    fl_end_line();
};
