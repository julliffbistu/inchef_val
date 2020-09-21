#ifndef _COMMON_FUNC_H
#define _COMMON_FUNC_H

#include "rlog.h"
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <execinfo.h>
#include <iomanip>

static bool CreateDirectory(const string path)
{
    bool ret = true;
    if(access(path.c_str(), 0) != 0)
    {
        if (mkdir(path.c_str(), 0777) !=0){
            ret = false;
            std::cout << "mkdir failed!" << std::endl;
        }
    }
    return ret;
}

static uint64_t GetTickCount()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static void dump(void)
{
#define BACKTRACE_SIZE   16
    int j, nptrs;
    void *buffer[BACKTRACE_SIZE];
    char **strings;

    nptrs = backtrace(buffer, BACKTRACE_SIZE);
    strings = backtrace_symbols(buffer, nptrs);
    if (strings == NULL) {
        rlog.e("backtrace_symbols error!");
        exit(EXIT_FAILURE);
    }

    for (j = 0; j < nptrs; j++)
        rlog.w("  [%02d] %s", j, strings[j]);

    free(strings);
}

static void signal_handler(int signo)
{
    rlog.w("=========>>>catch signal %d <<<=========", signo);
    rlog.w("backtrace start...");
    dump();
    rlog.w("backtrace end...\n");

    signal(signo, SIG_DFL);
    raise(signo);
}
#endif
