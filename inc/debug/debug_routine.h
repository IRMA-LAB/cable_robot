/**
 * @file debug_routine.h
 * @author Simone Comari
 * @date 03 Jul 2019
 * @brief This file include the skeleton of a simple debug application to be launched from
 * the main GUI when DEBUG_GUI define is set to 1 (in .pro file).
 */

#ifndef CABLE_ROBOT_DEBUG_SINGLE_DRIVE_SYSID_H
#define CABLE_ROBOT_DEBUG_SINGLE_DRIVE_SYSID_H

#include "robot/cablerobot.h"

/**
 * @brief The DebugClass class
 */
class DebugClass: public QObject
{
  Q_OBJECT
 public:
  /**
   * @brief Constructor
   * @param parent The parent object.
   * @param robot A pointer to the robot object.
   */
  DebugClass(QObject* parent, CableRobot* robot);
  ~DebugClass();

  /**
   * @brief Start the debug app (whatever it is).
   * @note Final user should implement debug procedure here..
   */
  void start();

 signals:
  /**
   * @brief Signal notifying debug routine is completed.
   */
  void debugCompleted() const;

 private slots:
  void stop() { emit debugCompleted(); }

 private:
  CableRobot* robot_;
};

#endif // CABLE_ROBOT_DEBUG_SINGLE_DRIVE_SYSID_H
