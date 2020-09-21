#include "rlog.h"
#include <stdlib.h>
#include <log4cpp/ShmAppender.hh>
#include <math.h>
#include <thread>

#define MODULE_NAME "LogService"

int fd_= -1;
uint8_t maxBackupIndex_ = 10;
uint64_t maxFileSize_ = (20*1024*1024);
const string fileName_ = "/opt/knowin/logs/algorithm.log";
int flags_ = (O_CREAT | O_APPEND | O_WRONLY);
mode_t mode_ = 00666;
