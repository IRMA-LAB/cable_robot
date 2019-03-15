/**
 * @file matlab_thread.cpp
 * @author Simone Comari
 * @date 07 Mar 2019
 * @brief This file includes definitions of class present in matlab_thread.h.
 */

#include "homing/matlab_thread.h"

void MatlabThread::run()
{
  // Check if script location was provided
  if (script_loc_.isEmpty())
  {
    emit printToQConsole("WARNING: Script location not given");
    return;
  }
  QFileInfo check_file(script_loc_);
  // Check if file exists
  if (!check_file.exists())
  {
    emit printToQConsole(QString("WARNING: Script '%1' does not exist").arg(script_loc_));
    return;
  }
  // Check if it is really a matlab file and no directory
  if (!(check_file.isFile() && check_file.suffix() == "m"))
  {
    emit printToQConsole(
      QString("WARNING: Script '%1' is not a Matlab file").arg(script_loc_));
    return;
  }
  // Run matlab optimization script from shell command
  emit printToQConsole(QString("Running matlab script '%1'").arg(script_loc_));
  RunMatlabScript(script_loc_.toStdString());
  emit printToQConsole("Matlab script execution completed");
  emit resultsReady();
}
