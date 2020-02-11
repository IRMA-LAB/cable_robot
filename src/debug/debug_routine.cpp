/**
 * @file debug_routine.cpp
 * @author Simone Comari
 * @date 03 Jul 2019
 * @brief This file includes definitions of classes present in debug_routine.h.
 */

#include "debug/debug_routine.h"

DebugClass::DebugClass(QObject* parent, CableRobot* robot)
  : QObject(parent), robot_(robot)
{
  // Init here..
}

DebugClass::~DebugClass()
{
  // Deinit here..
}

//--------- Public functions ---------------------------------------------------------//

void DebugClass::start()
{
  // Debug routine here..
  QTimer::singleShot(500, this, SLOT(stop()));
}
