#include <QApplication>

#include "libs/easyloggingpp/src/easylogging++.h"
#include "libcdpr/tools/robotconfigjsonparser.h"

#include "utils/easylog_wrapper.h"
#include "gui/homing/homing_interface_vision.h"

INITIALIZE_EASYLOGGINGPP

int main(int argc, char* argv[])
{
  START_EASYLOGGINGPP(argc, argv);
  // Configure all loggers
  el::Loggers::configureFromGlobal(SRCDIR "/config/logs.conf");

  QApplication a(argc, argv);
  qRegisterMetaType<cv::Mat>("cv::Mat");
  qRegisterMetaType<CameraParams>("CameraParams");

  RobotConfigJsonParser parser;
  QString default_filename(SRCDIR);
  default_filename.append("config/default.json");
  CLOG(INFO, "event") << "Loaded default configuration file '" << default_filename << "'";
  CLOG(INFO, "event") << "Parsing configuration file '" << default_filename << "'...";
  grabcdpr::Params params;
  parser.ParseFile(default_filename, &params);
  CableRobot robot(nullptr, params);
  HomingInterfaceVision w(nullptr, &robot);
  w.show();

  return a.exec();
}
