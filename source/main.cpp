#include <QApplication> // Common Qt inslusion

#include "cablerobotmaster.h"    //User includes, the master we programmed,
#include "cablerobotinterface.h" // And its inteface

// Declare master as global variable (usefull for qt signal and slot system)
static CableRobotMaster cable_robot_master;

int main(int argc, char* argv[])
{
  QApplication a(argc, argv);                                          // Qt default
  CableRobotInterface cable_robot_interface(nullptr, &cable_robot_master); // the GUI

  // Master properties are left to defaults
  cable_robot_master.Start();      // Start the master
  cable_robot_interface.show();  // Show Gui
  qRegisterMetaType<QVector<double>>("QVector<double>");

  return a.exec();
}
