#include "log_service.h"

void RollOver() {
    close(fd_);
    if (maxBackupIndex_ > 0) {
        std::ostringstream filename_stream;
        int maxBackupIndexWidth = log10((float)maxBackupIndex_)+1;
        filename_stream << fileName_ << "." << std::setw( maxBackupIndexWidth ) << std::setfill( '0' ) << maxBackupIndex_ << std::ends;
        // remove the very last (oldest) file
        std::string last_log_filename = filename_stream.str();
        // std::cout << last_log_filename << std::endl; // removed by request on sf.net #140
        remove(last_log_filename.c_str());

        // rename each existing file to the consequent one
        for(unsigned int i = maxBackupIndex_; i > 1; i--) {
            filename_stream.str(std::string());
            filename_stream << fileName_ << '.' << std::setw( maxBackupIndexWidth ) << std::setfill( '0' ) << i - 1 << std::ends;  // set padding so the files are listed in order
            rename(filename_stream.str().c_str(), last_log_filename.c_str());
            last_log_filename = filename_stream.str();
        }
        // new file will be numbered 1
        rename(fileName_.c_str(), last_log_filename.c_str());
    }
    fd_ = open(fileName_.c_str(), flags_, mode_);
}

void LogThread()
{
    LogMsg log_msg_;
    int logShmId_ = LogShmInit();
    if(logShmId_ < 0){
        std::cout << "log shm init failed!" << std::endl;
        return;
    }
    log_msg_.mtype = 1;
    uint64_t len;
    fd_ = open(fileName_.c_str(), flags_, mode_);
    if(fd_ < 0){
        std::cout << "open log file failed!" << std::endl;
        return;
    }
    while(1){
        len = msgrcv(logShmId_, &log_msg_, LogMaxLen, 1, 0);
        if(len > 0){
            string msg = string(log_msg_.mtext, len);
            if (!write(fd_, msg.data(), len)) {
                std::cout << "write log to file failed!" << std::endl;
            }
            off_t offset = ::lseek(fd_, 0, SEEK_END);
            if (offset >= 0) {
                if(static_cast<size_t>(offset) >= maxFileSize_) {
                    RollOver();
                }
            }
        }
    }
    if (fd_!=-1) {
        ::close(fd_);
        fd_=-1;
    }
}

void SetDefaultLogLevel()
{
    string level = "notice";
    rlog.n("set logService loglevel to [%s]", level.c_str());
    rlog.setPriority(level);
}

int main(int argc, char** argv)
{
    rlog.SetModuleName(MODULE_NAME);
    SetDefaultLogLevel();
    signal(SIGSEGV, signal_handler);
    signal(SIGABRT, signal_handler);
    std::thread logT(LogThread);
    logT.join();
    return 0;
}

