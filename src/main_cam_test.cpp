#include <QApplication>

#include "libs/easyloggingpp/src/easylogging++.h"
#include "utils/easylog_wrapper.h"

#include "gui/camera/camera_widget.h"

INITIALIZE_EASYLOGGINGPP

int main(int argc, char* argv[])
{
  START_EASYLOGGINGPP(argc, argv);
  // Configure all loggers
  el::Loggers::configureFromGlobal(SRCDIR "/config/logs.conf");

  QApplication a(argc, argv);
  qRegisterMetaType<cv::Mat>("cv::Mat");
  qRegisterMetaType<CameraParams>("CameraParams");

  CameraWidget w;
  w.show();

  return a.exec();
}
