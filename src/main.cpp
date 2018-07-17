#include <QApplication> // Common Qt inslusion

#include "cablerobotmaster.h"    //User includes, the master we programmed,
#include "cablerobotinterface.h" // And its inteface

static CableRobotMaster cableRobotMaster; // Declare master as global variable (usefull
                                   // for qt signal and slot system)

int main(int argc, char* argv[])
{
  QApplication a(argc, argv);                                          // Qt default
  CableRobotInterface cableRobotInterface(nullptr, &cableRobotMaster); // the
                                                                       // GUI

  // Master properties are left to defaults
  cableRobotMaster.Start();   // Start the master
  cableRobotInterface.show(); // Show Gui
  qRegisterMetaType<QVector<double>>("QVector<double>");
  return a.exec();
}
