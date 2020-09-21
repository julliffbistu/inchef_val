/*
 * ShmAppender.hh
 *
 * Copyright 2020, Kownin. All rights reserved.
 *
 * See the COPYING file for the terms of usage and distribution.
 */

#ifndef _LOG4CPP_SHMAPPENDER_HH
#define _LOG4CPP_SHMAPPENDER_HH

#include "common_func.h"
#include <string.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <sys/msg.h>
#include <stdint.h>
#include <log4cpp/Portability.hh>
#include <string>
#include <queue>
#include <log4cpp/LayoutAppender.hh>
#include <iostream>

#define LogShmKey       "/opt/knowin/logs/"
#define LogShmProjId    0x6666
#define LogShmFlags     (IPC_CREAT | 0666)
#define LogMaxLen       (409600)

struct LogMsg
{
	long mtype;
	char mtext[LogMaxLen];
};

inline int LogShmInit()
{
    int shmid = 0;
    key_t key = 123;
    if(CreateDirectory(LogShmKey)){
        key = ftok(LogShmKey, LogShmProjId);
        if(key < 0)
        {
            std::cout << "ftok failed!" << std::endl;
            return -1;
        }

    }
    if((shmid = msgget(key, LogShmFlags)) < 0)
    {
        std::cout << "msgget failed!" << std::endl;
        return -2;
    }
    std::cout << "shmid: " << shmid << std::endl;
    return shmid;
}

inline int LogShmDestroy(int shmid)
{
    if(msgctl(shmid,IPC_RMID,NULL) < 0)
    {
        std::cout << "msgctl failed!" << std::endl;
        return -1;
    }
    return 0;
}

namespace log4cpp {

    /**
     * This class puts log messages in an shm. Its primary use
     * is in test cases, but it may be useful elsewhere as well.
     *
     * @since 0.2.4
     **/
    class LOG4CPP_EXPORT ShmAppender : public LayoutAppender {
        public:

        ShmAppender(const std::string& name);
        virtual ~ShmAppender();
        virtual bool reopen();
        virtual void close();

        protected:

        /**
         * Appends the LoggingEvent to the queue.
         * @param event the LoggingEvent to layout and append to the queue.
         **/
        virtual void _append(const LoggingEvent& event);

        private:
        LogMsg log_msg_;
        int logShmId_;
    };
}

#endif // _LOG4CPP_SHMAPPENDER_HH
