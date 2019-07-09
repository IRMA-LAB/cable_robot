/**
 * @file main.cpp
 * @author Simone Comari
 * @date 22 Mar 2019
 * @brief This file includes the main function where the CableRobot app is started.
 */

#include <QApplication>
#include <QtDebug>

#include "gui/login_window.h"
#include "libs/easyloggingpp/src/easylogging++.h"
#include "utils/easylog_wrapper.h"

INITIALIZE_EASYLOGGINGPP

int main(int argc, char* argv[])
{
  START_EASYLOGGINGPP(argc, argv);
  // Configure all loggers
  el::Loggers::configureFromGlobal(SRCDIR "/config/logs.conf");

  QApplication a(argc, argv);
  qRegisterMetaType<grabec::GSWDriveInPdos>("grabec::GSWDriveInPdos");
  qRegisterMetaType<id_t>("id_t");
  qRegisterMetaType<cv::Mat>("cv::Mat");
  qRegisterMetaType<CameraParams>("CameraParams");
  qRegisterMetaType<std::bitset<3>>("std::bitset<3>");
  CLOG(INFO, "event") << "App START";

  LoginWindow w;
  w.show();
  CLOG(INFO, "event") << "Prompt login window";

  return a.exec();
}
