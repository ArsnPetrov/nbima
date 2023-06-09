//
//  GUI_Spectre.hpp
//  nbima
//
//  Created by Arseny on 23.03.2023.
//

#ifndef GUI_Spectre_hpp
#define GUI_Spectre_hpp

#include <stdio.h>
#include <stdint.h>
#include <FL/Fl.H>
#include <FL/Fl_Box.H>
#include <FL/fl_draw.H>

#include "util.h"

class GUI_Spectre : public Fl_Box {
    float* buffer;
    uint32_t buffer_size;
    
public:
    GUI_Spectre(int x, int y, int w, int h, const char* l);
    
    void draw() override;
    void link_buffer(float* buffer, int size);
//    void resize_buffer(int size);
};

template void Fl_force_redraw_callback<GUI_Spectre>(void *arg);

#endif /* GUI_Spectre_hpp */
