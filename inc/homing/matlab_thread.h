#ifndef MATLABENGINETHREAD_H
#define MATLABENGINETHREAD_H

#include <QThread>
#include <QFileInfo>

#include "grabcommon.h"

class MatlabThread: public QThread
{
  Q_OBJECT
 public:
  MatlabThread(QObject* parent) : QThread(parent) {}
  MatlabThread(QObject* parent, const QString& script_loc)
    : QThread(parent), script_loc_(script_loc)
  {}

  void SetMatlabScriptLoc(const QString& script_loc) { script_loc_ = script_loc; }

 private:
  QString script_loc_;

  void run() override;

 signals:
  void resultsReady() const;
  void printToQConsole(const QString&) const;
};

#endif // MATLABENGINETHREAD_H
