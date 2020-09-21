/*
  使用说明:
    1.编译需包含本头文件,使用时直接调用 API1 或者 API2 函数
*/
#pragma once
#ifndef __Log_H__
#define __Log_H__
#include "log4cpp/Category.hh"
#include "log4cpp/PropertyConfigurator.hh"
#include <string>
#define CONFPATH "/opt/knowin/configs/log4cpp.conf"
#define p_FATAL log4cpp ::Priority::FATAL
#define p_ERROR log4cpp ::Priority::ERROR
#define p_WARN log4cpp ::Priority::WARN
#define p_NOTICE log4cpp ::Priority::NOTICE
#define p_INFO log4cpp ::Priority::INFO
#define p_DEBUG log4cpp ::Priority::DEBUG

using namespace std;
#define LOGAPI void
#define PriorityAPI bool

class Log {
 public:
  static Log &getInstance();
  log4cpp::Category &categoryRef;
  //------------------API1------------------API1------------------API1------------------API1------------------API1
  LOGAPI f(const char *fmt, ...);
  LOGAPI e(const char *fmt, ...);
  LOGAPI w(const char *fmt, ...);
  LOGAPI n(const char *fmt, ...);
  LOGAPI i(const char *fmt, ...);
  LOGAPI d(const char *fmt, ...);
  LOGAPI redirectConfPath(const char* path) __nonnull((2));
  PriorityAPI setPriority(log4cpp::Priority::Value);
  PriorityAPI setPriority(const string &level);
  PriorityAPI getPriority(string &level);

  void SetModuleName(const string module_name)
  {
    module_name_ = " (" + module_name + ") ";
  }

 private:
  Log();
  static Log *logPtr;
  string module_name_;
};
extern Log &logKnowin;
//------------------API2------------------API2------------------API2------------------API2------------------API2

#define rlog logKnowin
#define logFat(moduleName, fmt, ...) \
  rlog.categoryRef.fatal(moduleName #fmt, ##__VA_ARGS__)
#define logErr(moduleName, fmt, ...) \
  rlog.categoryRef.error(moduleName #fmt, ##__VA_ARGS__)
#define logWar(moduleName, fmt, ...) \
  rlog.categoryRef.warn(moduleName #fmt, ##__VA_ARGS__)
#define logNot(moduleName, fmt, ...) \
  rlog.categoryRef.notice(moduleName #fmt, ##__VA_ARGS__)
#define logInf(moduleName, fmt, ...) \
  rlog.categoryRef.info(moduleName #fmt, ##__VA_ARGS__)
#define logDeb(moduleName, fmt, ...) \
  rlog.categoryRef.debug(moduleName #fmt, ##__VA_ARGS__)

#endif
