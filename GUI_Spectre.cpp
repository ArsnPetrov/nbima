//
//  GUI_Spectre.cpp
//  nbima
//
//  Created by Arseny on 23.03.2023.
//

#include "GUI_Spectre.hpp"
#include <cfloat>
#include <algorithm>
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
    fl_draw_box(FL_DOWN_BOX, x(), y(), w(), h(), FL_BLACK);

    fl_color(FL_WHITE);
    
    float min = FLT_MAX;
    float max = FLT_MIN;
    
    for (int i = 0; i < buffer_size; i++) {
        if (buffer[i] < min) {
            min = std::max(buffer[i], -200.f);
        }
        
        if (buffer[i] > max) {
            max = buffer[i];
        }
    }
    
    float span = max - min;
    
    float scale = 0.8 * h() / span;
    
    float bias = (0.1 * h() - min * scale) / scale;
    
    if (buffer_size == 0) {
        printf("Buffer not found\n");
        return;
    }

    int decimation = 1;
    
    for (int i = 1; i < buffer_size / decimation; i++) {
        float val0 = 0;
        float val1 = 0;

        for (int j = 0; j < decimation; j++) {
            val0 += buffer[decimation*(i - 1) + j] / decimation;
            val1 += buffer[decimation* i      + j] / decimation;
        }
        
        fl_begin_line();
        fl_vertex(x() + (float)w() / (buffer_size - 1) * (decimation*(i - 1)), y() + h() - (val0 + bias) * scale);
        fl_vertex(x() + (float)w() / (buffer_size - 1) * (decimation*i),       y() + h() - (val1 + bias) * scale);
        fl_end_line();
    }
};
