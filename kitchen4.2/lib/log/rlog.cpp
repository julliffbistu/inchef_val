/**
 * @file Log.cpp
 * @author sxw@knowin.com
 * @brief
 * @version 0.1
 * @date 2019-12-04
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "rlog.h"
using namespace std;
#define MAXSIZE (1024)

string strFormat(const char* fmt, va_list arglist) {
  size_t size = MAXSIZE;
  char* strResult = new char[size];
  va_list arg;
  while (true) {
    va_copy(arg, arglist);
    int n = vsnprintf(strResult, size, fmt, arg);
    va_end(arg);
    if (n > -1) {
      if (static_cast<size_t>(n) < size){
        string s(strResult);
        delete[] strResult;
        return s;
      } else{
        delete[] strResult;
        size *= 2;
        strResult = new char[size];
        continue;
      }
    } else{
      delete[] strResult;
      return NULL;
    }
  }
}

Log* Log::logPtr = NULL;
Log& logKnowin = Log::getInstance();

//================================
Log& Log::getInstance() {
  if (logPtr == NULL) {
    logPtr = new Log;
  }
  return *logPtr;
}

Log::Log() : categoryRef(log4cpp::Category::getRoot()) {
  log4cpp::PropertyConfigurator::configure(CONFPATH);
  module_name_ = " (null) ";
}

void Log::redirectConfPath(const char* path) { log4cpp::PropertyConfigurator::configure(path); }

bool Log::setPriority(log4cpp::Priority::Value p) {
  switch (p) {
    case log4cpp::Priority::FATAL:
      break;
    case log4cpp::Priority::ERROR:
      break;
    case log4cpp::Priority::WARN:
      break;
    case log4cpp::Priority::NOTICE:
      break;
    case log4cpp::Priority::INFO:
      break;
    case log4cpp::Priority::DEBUG:
      break;
    default:
      return false;
      break;
  }
  categoryRef.setRootPriority(p);
  return true;
}

bool Log::setPriority(const string &level) {
    log4cpp::Priority::Value p;

    if (level == "fatal")
        p = log4cpp::Priority::FATAL;
    else if (level == "error")
        p = log4cpp::Priority::ERROR;
    else if (level == "warn")
        p = log4cpp::Priority::WARN;
    else if (level == "notice")
        p = log4cpp::Priority::NOTICE;
    else if (level == "info")
        p = log4cpp::Priority::INFO;
    else if (level == "debug")
        p = log4cpp::Priority::DEBUG;
    else
        p = log4cpp::Priority::INFO;

    categoryRef.setRootPriority(p);
    return true;
}

bool Log::getPriority(string &level) {
    log4cpp::Priority::Value p;
    p = categoryRef.getRootPriority();
    switch(p){
        case log4cpp::Priority::FATAL:
            level = "fatal";
            break;
        case log4cpp::Priority::WARN:
            level = "warn";
            break;
        case log4cpp::Priority::NOTICE:
            level = "notice";
            break;
        case log4cpp::Priority::INFO:
            level = "info";
            break;
        case log4cpp::Priority::DEBUG:
            level = "debug";
            break;
        default:
            level = "info";
            break;
    }
    return true;
}

void Log::f(const char* fmt, ...) {
  va_list arglist;
  va_start(arglist, fmt);
  std::string strLog = strFormat(fmt, arglist);
  va_end(arglist);
  categoryRef.fatal(module_name_ + strLog);
}

void Log::e(const char* fmt, ...) {
  va_list arglist;
  va_start(arglist, fmt);
  std::string strLog = strFormat(fmt, arglist);
  va_end(arglist);
  categoryRef.error(module_name_ + strLog);
}

void Log::w(const char* fmt, ...) {
  va_list arglist;
  va_start(arglist, fmt);
  std::string strLog = strFormat(fmt, arglist);
  va_end(arglist);
  categoryRef.warn(module_name_ + strLog);
}

void Log::n(const char* fmt, ...) {
  va_list arglist;
  va_start(arglist, fmt);
  std::string strLog = strFormat(fmt, arglist);
  va_end(arglist);
  categoryRef.notice(module_name_ + strLog);
}

void Log::i(const char* fmt, ...) {
  va_list arglist;
  va_start(arglist, fmt);
  std::string strLog = strFormat(fmt, arglist);
  va_end(arglist);
  categoryRef.info(module_name_ + strLog);
}

void Log::d(const char* fmt, ...) {
  va_list arglist;
  va_start(arglist, fmt);
  std::string strLog = strFormat(fmt, arglist);
  va_end(arglist);
  categoryRef.debug(module_name_ + strLog);
}

/*  add for python call log api  */
extern "C"{
void fatal(char* str){
  rlog.f(str);
}

void error(char* str){
  rlog.e(str);
}

void warn(char* str){
  rlog.w(str);
}

void notice(char* str){
  rlog.n(str);
}

void info(char* str){
  rlog.i(str);
}

void debug(char* str){
  rlog.d(str);
}
void set_modulename(const char* module_name)
{
  rlog.SetModuleName(module_name);
}
void set_priority(const char* level)
{
  rlog.setPriority(level);
}
}
