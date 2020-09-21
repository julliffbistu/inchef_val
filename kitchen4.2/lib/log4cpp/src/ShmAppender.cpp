/*
 * ShmAppender.cpp
 *
 * Copyright 2000, LifeLine Networks BV (www.lifeline.nl). All rights reserved.
 * Copyright 2000, Bastiaan Bakker. All rights reserved.
 *
 * See the COPYING file for the terms of usage and distribution.
 */

#include "PortabilityImpl.hh"
#include <log4cpp/ShmAppender.hh>

namespace log4cpp {

    ShmAppender::ShmAppender(const std::string& name) : LayoutAppender(name) {
        logShmId_ = LogShmInit();
        log_msg_.mtype = 1;
    }

    ShmAppender::~ShmAppender() {
        close();
    }

    void ShmAppender::close() {
        // empty
    }

    bool ShmAppender::reopen() {
        return true;
    }

    void ShmAppender::_append(const LoggingEvent& event) {
        if(logShmId_ >= 0){
            std::string msg = _getLayout().format(event);
            uint64_t len = msg.size();
            if(len > 0){
                if(len > LogMaxLen){
                    len = LogMaxLen;
                }
                strncpy(log_msg_.mtext, msg.c_str(), len);
                msgsnd(logShmId_, &log_msg_, len, IPC_NOWAIT);
            }
        } else{
            std::cout << "log shm init failed!" << std::endl;
        }
    }
}