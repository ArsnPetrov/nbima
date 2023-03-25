//
//  GUI_Spectre.cpp
//  nbima
//
//  Created by Arseny on 23.03.2023.
//

#include "GUI_Spectre.hpp"
#include <cfloat>
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
};

//void GUI_Spectre::resize_buffer(int size) {};

void GUI_Spectre::draw() {
    fl_draw_box(FL_DOWN_BOX, x(), y(), w(), h(), FL_BLACK);
    
    float min = FLT_MAX;
    float max = FLT_MIN;
    
    for (int i = 0; i < buffer_size; i++) {
        if (buffer[i] < min) {
            min = buffer[i];
        }
        
        if (buffer[i] > max) {
            max = buffer[i];
        }
    }
    
    float span = max - min;
    
    float ymin = min - 0.1 * span;
    float ymax = max + 0.1 * span;
    
    float scale = h() / (ymax - ymin);
    
    float bias = span * 0.1;
    
    printf("max is %f, min is %f\n", max, min);
    
    if (buffer_size == 0) {
        printf("Buffer not found\n");
        return;
    }
    
    for (int i = 1; i < buffer_size; i++) {
        fl_color(FL_WHITE);
        
        fl_begin_line();
        fl_vertex(x() + w() / (buffer_size) * (i - 1), y() + h() - (buffer[i - 1] - bias) * scale);
        fl_vertex(x() + w() / (buffer_size) * (i), y() + h() - (buffer[i] - bias) * scale);
        fl_end_line();
    }
};
