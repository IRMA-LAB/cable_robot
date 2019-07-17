#include <QApplication>

#include "libcdpr/tools/robotconfigjsonparser.h"
#include "libs/easyloggingpp/src/easylogging++.h"

#include "gui/homing/homing_interface_vision.h"
#include "utils/easylog_wrapper.h"

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
  grabcdpr::RobotParams params;
  parser.ParseFile(default_filename, &params);
  CableRobot robot(nullptr, params);
  VisionParams vision_params;
  vision_params.camera.fill(
    {704.794078001274, 0.0, 320.0, 0.0, 718.766056466343, 240.0, 0.0, 0.0, 1.0},
    {-0.527425626579164, 1.30450069781873, 0.0116411242466877, 0.0270049043250451,
     5.0712967746883});
  HomingInterfaceVision w(nullptr, &robot, vision_params);
  w.show();

  return a.exec();
}
