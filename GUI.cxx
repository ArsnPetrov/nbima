// generated by Fast Light User Interface Designer (fluid) version 1.0308

#include <nl_types.h>
// Initialize I18N stuff now for menus...
#include <locale.h>
static char *_locale = setlocale(LC_MESSAGES, "");
static nl_catd _catalog = catopen("GUI", 0);
#include "GUI.h"

Fl_Button *btn_calibrate=(Fl_Button *)0;

GUI_Spectre *noise_spectre_box=(GUI_Spectre *)0;

unsigned char menu__i18n_done = 0;
Fl_Menu_Item menu_[] = {
 {"File", 0,  0, 0, 0, (uchar)FL_NORMAL_LABEL, 0, 10, 0},
 {"DSP parameters", 0,  0, 0, 0, (uchar)FL_NORMAL_LABEL, 0, 10, 0},
 {"About", 0,  0, 0, 0, (uchar)FL_NORMAL_LABEL, 0, 10, 0},
 {0,0,0,0,0,0,0,0,0}
};

Fl_Text_Display *debug_info_panel=(Fl_Text_Display *)0;

Fl_Double_Window* make_window() {
  Fl_Double_Window* w;
  { Fl_Double_Window* o = new Fl_Double_Window(1150, 540, catgets(_catalog,1,1,"nbima"));
    w = o; if (w) {/* empty */}
    o->labelsize(12);
    { Fl_Group* o = new Fl_Group(10, 50, 305, 140, catgets(_catalog,1,2,"Noise Source Calibration"));
      o->box(FL_BORDER_BOX);
      o->labelsize(11);
      o->align(Fl_Align(FL_ALIGN_TOP_LEFT));
      { Fl_Value_Input* o = new Fl_Value_Input(155, 60, 150, 20, catgets(_catalog,1,3,"Lower frequency, MHz"));
        o->labelsize(12);
        o->value(100);
        o->textsize(12);
      } // Fl_Value_Input* o
      { Fl_Value_Input* o = new Fl_Value_Input(155, 85, 150, 20, catgets(_catalog,1,4,"Upper frequency, MHz"));
        o->labelsize(12);
        o->value(1000);
        o->textsize(12);
      } // Fl_Value_Input* o
      { Fl_Value_Input* o = new Fl_Value_Input(155, 110, 150, 20, catgets(_catalog,1,5,"Frame size"));
        o->labelsize(12);
        o->value(4096);
        o->textsize(12);
      } // Fl_Value_Input* o
      { Fl_Value_Input* o = new Fl_Value_Input(155, 135, 150, 20, catgets(_catalog,1,6,"Re-tuning period, ms"));
        o->labelsize(12);
        o->value(100);
        o->textsize(12);
      } // Fl_Value_Input* o
      { btn_calibrate = new Fl_Button(230, 160, 75, 20, catgets(_catalog,1,7,"Calibrate"));
        btn_calibrate->type(1);
        btn_calibrate->labelsize(12);
      } // Fl_Button* btn_calibrate
      o->end();
    } // Fl_Group* o
    { Fl_Button* o = new Fl_Button(280, 30, 35, 20, catgets(_catalog,1,8,"help?"));
      o->box(FL_NO_BOX);
      o->labelsize(9);
      o->labelcolor(FL_DARK_BLUE);
    } // Fl_Button* o
    { noise_spectre_box = new GUI_Spectre(325, 50, 815, 145, catgets(_catalog,1,9,"Estimated noise generator frequency spectrum"));
      noise_spectre_box->box(FL_FLAT_BOX);
      noise_spectre_box->color((Fl_Color)24);
      noise_spectre_box->selection_color(FL_BACKGROUND_COLOR);
      noise_spectre_box->labeltype(FL_NORMAL_LABEL);
      noise_spectre_box->labelfont(0);
      noise_spectre_box->labelsize(11);
      noise_spectre_box->labelcolor(FL_FOREGROUND_COLOR);
      noise_spectre_box->align(Fl_Align(FL_ALIGN_TOP_LEFT));
      noise_spectre_box->when(FL_WHEN_RELEASE);
    } // GUI_Spectre* noise_spectre_box
    { Fl_Group* o = new Fl_Group(10, 215, 305, 165, catgets(_catalog,1,10,"Two-point circuit examination"));
      o->box(FL_BORDER_BOX);
      o->labelsize(11);
      o->align(Fl_Align(FL_ALIGN_TOP_LEFT));
      { Fl_Button* o = new Fl_Button(220, 350, 85, 20, catgets(_catalog,1,11,"Measure"));
        o->labelsize(12);
      } // Fl_Button* o
      { Fl_Value_Input* o = new Fl_Value_Input(155, 225, 150, 20, catgets(_catalog,1,12,"Lower frequency, MHz"));
        o->labelsize(12);
        o->value(100);
        o->textsize(12);
      } // Fl_Value_Input* o
      { Fl_Value_Input* o = new Fl_Value_Input(155, 250, 150, 20, catgets(_catalog,1,13,"Upper frequency, MHz"));
        o->labelsize(12);
        o->value(1000);
        o->textsize(12);
      } // Fl_Value_Input* o
      { Fl_Value_Input* o = new Fl_Value_Input(155, 275, 150, 20, catgets(_catalog,1,14,"Frame size"));
        o->labelsize(12);
        o->value(4096);
        o->textsize(12);
      } // Fl_Value_Input* o
      { Fl_Value_Input* o = new Fl_Value_Input(155, 325, 150, 20, catgets(_catalog,1,15,"Impedance, \316\251"));
        o->labelsize(12);
        o->textsize(12);
      } // Fl_Value_Input* o
      { Fl_Value_Input* o = new Fl_Value_Input(155, 300, 150, 20, catgets(_catalog,1,16,"Re-tuning period, ms"));
        o->labelsize(12);
        o->value(100);
        o->textsize(12);
      } // Fl_Value_Input* o
      o->end();
    } // Fl_Group* o
    { Fl_Menu_Bar* o = new Fl_Menu_Bar(0, 0, 1150, 25);
      o->box(FL_BORDER_BOX);
      if (!menu__i18n_done) {
        int i=0;
        for ( ; i<3; i++)
          if (menu_[i].label())
            menu_[i].label(catgets(_catalog,1,i+17,menu_[i].label()));
        menu__i18n_done = 1;
      }
      o->menu(menu_);
    } // Fl_Menu_Bar* o
    { Fl_Box* o = new Fl_Box(325, 215, 815, 145, catgets(_catalog,1,20,"Frequency response"));
      o->box(FL_FLAT_BOX);
      o->color((Fl_Color)24);
      o->labelsize(11);
      o->align(Fl_Align(FL_ALIGN_TOP_LEFT));
    } // Fl_Box* o
    { Fl_Box* o = new Fl_Box(325, 380, 815, 145, catgets(_catalog,1,21,"SWR"));
      o->box(FL_FLAT_BOX);
      o->color((Fl_Color)24);
      o->labelsize(11);
      o->align(Fl_Align(FL_ALIGN_TOP_LEFT));
    } // Fl_Box* o
    { debug_info_panel = new Fl_Text_Display(10, 400, 305, 125, catgets(_catalog,1,22,"Debug info"));
      debug_info_panel->labelsize(11);
      debug_info_panel->textsize(11);
      debug_info_panel->align(Fl_Align(FL_ALIGN_TOP_LEFT));
    } // Fl_Text_Display* debug_info_panel
    o->end();
  } // Fl_Double_Window* o
  return w;
}

Fl_Double_Window* make_about_window() {
  Fl_Double_Window* w;
  { Fl_Double_Window* o = new Fl_Double_Window(270, 210);
    w = o; if (w) {/* empty */}
    { new Fl_Text_Display(55, 35, 160, 110, catgets(_catalog,1,23,"nbima"));
    } // Fl_Text_Display* o
    o->end();
  } // Fl_Double_Window* o
  return w;
}

Fl_Slider *dc_coef_slider=(Fl_Slider *)0;

static void cb_dc_coef_slider(Fl_Slider* o, void*) {
  dc_coef_input->value(o->value());
}

Fl_Value_Input *dc_coef_input=(Fl_Value_Input *)0;

static void cb_dc_coef_input(Fl_Value_Input* o, void*) {
  dc_coef_slider->value(o->value());
}

Fl_Double_Window* make_dsp_window() {
  Fl_Double_Window* w;
  { Fl_Double_Window* o = new Fl_Double_Window(330, 65, catgets(_catalog,1,24,"DSP parameters"));
    w = o; if (w) {/* empty */}
    o->hotspot(o);
    { dc_coef_slider = new Fl_Slider(150, 35, 170, 20);
      dc_coef_slider->type(1);
      dc_coef_slider->labelsize(12);
      dc_coef_slider->value(1);
      dc_coef_slider->callback((Fl_Callback*)cb_dc_coef_slider);
      dc_coef_slider->align(Fl_Align(FL_ALIGN_LEFT_TOP));
    } // Fl_Slider* dc_coef_slider
    { dc_coef_input = new Fl_Value_Input(150, 10, 170, 20, catgets(_catalog,1,25,"DC blocker coefficient"));
      dc_coef_input->labelsize(12);
      dc_coef_input->value(1);
      dc_coef_input->textsize(12);
      dc_coef_input->callback((Fl_Callback*)cb_dc_coef_input);
    } // Fl_Value_Input* dc_coef_input
    o->end();
  } // Fl_Double_Window* o
  return w;
}
