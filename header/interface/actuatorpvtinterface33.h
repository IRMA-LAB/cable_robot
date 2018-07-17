#ifndef ACTUATORPVTINTERFACE33_H
#define ACTUATORPVTINTERFACE33_H

#include <QWidget>
#include <QCloseEvent>
#include <QTableWidgetItem>
#include <QFile>
#include <QDir>
#include <QVector>
#include <QTextStream>

#include "cablerobotmaster.h" //User declaration
#include "common.h"

namespace Ui
{
class ActuatorPvtInterface33;
}

class ActuatorPvtInterface33 : public QWidget
{
  Q_OBJECT
private:
  QFile data_file_;

public:
  explicit ActuatorPvtInterface33(QWidget* parent = 0, CableRobotMaster* master = 0);
  ~ActuatorPvtInterface33();

signals:
  void GoBackIdle(int);
  void SendEnableRequest(int);
  void SendClearFaultRequest();
  void SendStartRequest();
  void SendExportDataRequest();
  void SendSimulationData(int, double*, double*, double*);

private slots:
  void on_ImportDataButton_clicked();
  void on_StartButton_toggled(bool checked);
  void on_ExportDataButton_clicked();
  void on_EnableButton_toggled(bool checked);
  void on_ClearFaultsButton_clicked();

  void on_StartButton_clicked();

public slots:
  void CollectFaultPresentAdvice(int theMotor);
  void CollectEnableCommandProcessed(int status, int motor);
  void CollectClearFaultRequestProcessed(int motor);
  void CollectActuatorPvt33Control(int state);
  void CollectData(double s0, double s1, double s2);

private:
  CableRobotMaster* cable_robot_master_;
  QVector<double> pulley_angles_out_[3];
  QVector<double> cable_len_in_[3];
  CableRobot* cable_robot_;
  Ui::ActuatorPvtInterface33* ui;

  virtual void closeEvent(QCloseEvent* event);
};

#endif // ACTUATORPVTINTERFACE33_H
