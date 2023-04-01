#ifndef util_h
#define util_h

#ifdef _WIN32
#include <windows.h>
#endif
#ifdef __APPLE__
#include <unistd.h>
#endif
#ifdef unix
#include <unistd.h>
#include <X11/Xlib.h>
#endif

inline void nbima_sleep(int sleepMs)
{
#ifdef _WIN32
	Sleep(sleepMs);
#else
	usleep(sleepMs * 1000);
#endif
}

template<typename T>
void Fl_force_redraw_callback(void *arg)
{
	((T*)arg)->redraw();
	Fl::repeat_timeout(0.016, Fl_force_redraw_callback<T>, arg);
}

#endif /* GUI_Spectre_hpp */