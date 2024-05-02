#ifndef __DESTRUCTIVE_STREAM_H__
#define __DESTRUCTIVE_STREAM_H__


#include <functional>
#include <iostream>
#include <sstream>
#include <bnerf_utils/typedef.h>


namespace bnerf {
    struct dstream : public stringstream {
        dstream(function<void(const char*)> func)
            : func_(func) {}

        ~dstream() {
            const string s = str();
            func_(s.c_str());
        }

        function<void(const char*)> func_;
    };
}

//#define QLINE_SS(le) drc::dstream(drc::bind(&QLineEdit::setText, le, drc::_1)) 
#endif